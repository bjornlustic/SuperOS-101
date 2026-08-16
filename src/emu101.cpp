// SuperOS-101 — 80C49 emulator host layer
//
// Runs the original SH-101 mask ROM on the MCS-48 core (mcs48.h), passing the
// emulated ports straight through to the socket: our AVR ports map 1:1 onto
// the 80C49's BUS/P1/P2/T0/T1 (pins.h), so no translation is needed — the
// ROM's own active-low values, settle delays, and scan patterns hit the
// analog board exactly as on the original CPU.
//
// Pacing: 6 MHz resonator -> 400k machine cycles/s (15 clocks/cycle). The
// interpreter is throttled to that rate; if the AVR can't keep up it runs
// flat out and the emulated firmware simply runs proportionally slower (the
// DAC values are unaffected; scan latency and software delay loops stretch).
// The one analog consequence, the ROM's per-pass sample/hold refresh
// stretching with it, is compensated by the host (ShRefresh below).
//
// MIDI: notes 53..84 (the SH-101's own keys F3..C6, same mapping as SuperOS)
// are injected into the emulated key matrix, so the ROM sees them exactly as
// key presses (they arpeggiate, record, transpose like keys). Notes outside
// that range have no key and are ignored. Whatever the ROM plays (CV-OUT
// S/H value + GATE) is echoed on MIDI OUT as note = CV + 36, like SuperOS.
//
// "Battery RAM": the original kept its 100-step sequence alive in the
// 80C49's RAM on a battery. Here RAM 0x1C..0x7F (the ROM's START clears
// 0x02..0x1B itself, so that is the whole persistent set) is mirrored to
// internal EEPROM: restored before the ROM runs, and after any change has
// been quiet for 1 s the changed bytes are written back, one per loop pass
// (a byte write blocks ~3.4 ms; the pacer resyncs). Only recording changes
// this region, so wear is a non-issue.
#include <Arduino.h>
#include "emu101.h"
#include "pins.h"
#include "hw.h"
#include "menu.h"
#include "midiclock.h"
#include "midi.h"
#include "sysex.h"

#include <string.h>
#include "combined.h"
#include <avr/eeprom.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#define PROGMEM
#define pgm_read_byte(p) (*(const uint8_t*)(p))
#define cli() ((void)0)
#define sei() ((void)0)
#endif
#include "rom101_data.h"
#define MCS48_ROM_BASE ROM101      // 2 KB-aligned (tools/mkrom.py), as mcs48.h requires

#include "emu101_hle.h"     // native fast path for the ROM's key-scan loop (defines MCS48_HLE_ENTRY)
#include "mcs48.h"

static Mcs48 g_cpu;

// --- speed telemetry -----------------------------------------------------------
static uint32_t g_cycAcc = 0, g_passAcc = 0, g_rateT0 = 0;
static uint16_t g_cyc100 = 0, g_pps = 0, g_eps = 0, g_runPm = 0, g_shPm = 0;
static uint32_t g_runUsAcc = 0, g_shUsAcc = 0;
static uint16_t g_benchNoIntUs = 0, g_benchIntUs = 0;   // boot bench (BenchRun)
static uint16_t g_benchDacUs = 0, g_benchPollUs = 0;      // 10x hw::DacWrite / 100x midi::Poll
static uint16_t g_benchB[6];                               // 100x: Poll, T1Watch, RateTick, RamPersistTick, micros, button scan
static volatile uint16_t g_edgeAcc = 0;   // T1 falling edges seen (INT0 / poll)
static void RateTick(uint32_t ms, uint16_t ran) {
  g_cycAcc += ran;
  if ((uint32_t)(ms - g_rateT0) >= 1000) {
    g_cyc100 = (uint16_t)(g_cycAcc / 100); g_pps = (uint16_t)g_passAcc;
    cli(); g_eps = g_edgeAcc; g_edgeAcc = 0; sei();
    g_runPm = (uint16_t)(g_runUsAcc / 1000); g_shPm = (uint16_t)(g_shUsAcc / 1000);
    g_runUsAcc = 0; g_shUsAcc = 0;
    g_cycAcc = 0; g_passAcc = 0; g_rateT0 = ms;
  }
}

// --- MIDI keys into the matrix, ROM output onto MIDI OUT ---------------------
static uint32_t g_midiKeys = 0;        // bit k = MIDI holds key k (0 = F3)
static uint8_t  g_cvOut    = 0;        // last value the ROM wrote to the CV-OUT S/H
static bool     g_gate     = false;    // last GATE the ROM drove
static int16_t  g_echoNote = -1;       // note currently sounding on MIDI OUT

static const uint8_t MIDI_KEY0 = 53;   // MIDI note of key 0 (F3), as SuperOS

static void MidiNoteOn(uint8_t note, uint8_t) {
  int16_t k = (int16_t)note - MIDI_KEY0;
  if (k >= 0 && k < 32) g_midiKeys |= (1UL << k);
}
static void MidiNoteOff(uint8_t note) {
  int16_t k = (int16_t)note - MIDI_KEY0;
  if (k >= 0 && k < 32) g_midiKeys &= ~(1UL << k);
}
static void EchoOut() {
  int16_t note = g_gate ? (int16_t)g_cvOut + 36 : -1;
  if (note == g_echoNote) return;
  if (note >= 0) midi::NoteOn((uint8_t)note);          // new note first: legato-friendly
  if (g_echoNote >= 0) midi::NoteOff((uint8_t)g_echoNote);
  g_echoNote = note;
}

// --- sample/hold refresh -----------------------------------------------------
// The three S/H caps (CV-OUT, VCO range, RANDOM) hold their DAC voltage only
// as long as the ROM keeps re-writing them, once per pass. On the real chip
// a pass is 1.5-3.5 ms; the interpreter runs ~7x slower (4 MHz core, rail
// budget), so a pass takes 10-25 ms and the caps droop audibly between
// refreshes (a pitch warble that SuperOS, at a 0.9 ms pass, does not have).
// The host therefore re-presents the last value the ROM connected to each S/H
// at a SuperOS-like cadence, between emulated instructions, and then restores
// the ROM's BUS latch (data first, disconnected; then its route). The ROM
// cannot tell: INS A,BUS reads its own latch, and a scan column it left low
// is back on the pins before its next instruction. Only S/H the ROM has
// actually written are refreshed, never the one it currently holds connected.
// Costs about 12% of the interpreter's time at the 2 ms cadence.
static uint8_t  g_shVal[3];              // last value connected to route 0..2
static uint8_t  g_shKnown = 0;           // bit r = g_shVal[r] is valid
static const uint16_t SH_REFRESH_US = 2000;

static void ShRefresh() {
  uint8_t saved = BUS_PORT;              // == the ROM's bus latch (bus_write mirrors it)
  uint8_t held  = (uint8_t)(saved >> 6); // route the ROM has connected now (3 = none)
  // The two pitch S/H (CV-OUT, VCO range) only: the RANDOM S/H is the LFO's
  // S&H waveform, refreshed by the ROM's own pass often enough, and each
  // route costs ~130 us of interpreter time per refresh.
  for (uint8_t r = ROUTE_CV_OUT; r <= ROUTE_VCO; ++r)
    if ((g_shKnown & (1u << r)) && r != held) hw::DacWrite(g_shVal[r], (Route)r);
  BUS_PORT = (uint8_t)(saved | 0xC0);    // ROM's data back on the ladder, no S/H
  delayMicroseconds(hw::SETTLE_LADDER_US);
  BUS_PORT = saved;                      // and its route (or scan column)
  delayMicroseconds(hw::SETTLE_SCAN_US);
}

// --- core hooks: 1:1 port pass-through ---------------------------------------
uint8_t mcs48_rom_read(uint16_t addr) { return pgm_read_byte(&ROM101[addr & 0x7FF]); }
void    mcs48_bus_write(uint8_t v)    {
  BUS_PORT = v;
  uint8_t route = (uint8_t)(v >> 6);
  if (route != ROUTE_NONE) {
    g_shVal[route] = (uint8_t)(~v & 0x3F);
    g_shKnown |= (uint8_t)(1u << route);
    if (route == ROUTE_CV_OUT) { ++g_passAcc; g_cvOut = g_shVal[route]; EchoOut(); }
  }
}
// P1 rows are active-low; the BUS latch says which keyboard bank (DB0..3
// low) the ROM is scanning, so pull the MIDI-held keys of that bank low too.
uint8_t mcs48_p1_read() {
  uint8_t rows = P1_PIN;
  uint8_t bus = BUS_PORT;
  for (uint8_t bank = 0; bank < 4; ++bank)
    if (!(bus & (1u << bank))) rows &= (uint8_t)~(uint8_t)(g_midiKeys >> (8 * bank));
  return rows;
}
void    mcs48_p2_write(uint8_t v)     {
  P2_PORT = v;
  bool gate = !(v & (1u << P2_GATE));
  if (gate != g_gate) { g_gate = gate; EchoOut(); }
}
bool    mcs48_t0() { return (PIND & (1u << T0_HOLD_BIT))  != 0; }

// --- T1 (LFO / EXT CLK) edge capture ------------------------------------------
// The ROM samples T1 once per pass (one JNT1, SH101.asm:768) and calls it a
// clock when the sample went H->L since the previous pass. On the real chip a
// pass is 1.5-3.5 ms; here it is 10-30 ms, so a clock above roughly half the
// pass rate aliases: edges are missed, then bunch up, and the sequencer/arp
// lurches. The host therefore watches the real T1 (every loop pass and at
// every ROM read), counts falling edges, and shows the ROM a virtual T1 that
// hands them out one at a time: a pending edge is delivered as H on one read
// and L on the next, so every real edge becomes exactly one ROM edge (at
// most one per two passes; beyond that they queue, capped so a runaway
// clock cannot leave the sequencer stepping long after it slowed down).
// On the AVR the pin edges are caught by INT0 (PD0 = T1), so even an EXT CLK
// pulse shorter than a loop pass counts; the host sim polls the pin instead.
// With MIDI sync ON the divided MIDI clock (midiclock.h) is the source
// instead of the pin (polled after midi::Poll, which is what advances it).
static bool             g_t1Level   = true;   // last source level seen
static volatile uint8_t g_t1Pending = 0;      // falling edges captured, not yet shown to the ROM
static bool             g_t1Shown   = true;   // last level returned to the ROM
static const uint8_t T1_PENDING_MAX = 2;   // small: a clock faster than the ROM can follow drops edges instead of lagging

#ifdef __AVR__
ISR(INT0_vect) {
  ++g_edgeAcc;
  if (!midiclk::SyncOn() && g_t1Pending < T1_PENDING_MAX) ++g_t1Pending;
}
static inline void T1PinEdgeInit() {
  EICRA = (uint8_t)((EICRA & ~((1u << ISC01) | (1u << ISC00))) | (1u << ISC01)); // falling edge
  EIFR  = (1u << INTF0);
  EIMSK |= (1u << INT0);
}
#else
static inline void T1PinEdgeInit() {}
#endif

static inline bool T1Source() {
  if (midiclk::SyncOn()) return midiclk::T1Level();
  return (PIND & (1u << T1_CLOCK_BIT)) != 0;
}
static inline void T1Watch() {
  bool lvl = T1Source();
#ifdef __AVR__
  bool countHere = midiclk::SyncOn();     // pin edges arrive via INT0
#else
  bool countHere = true;
#endif
  if (countHere && g_t1Level && !lvl) {
    cli();
    ++g_edgeAcc;
    if (g_t1Pending < T1_PENDING_MAX) ++g_t1Pending;
    sei();
  }
  g_t1Level = lvl;
}
bool    mcs48_t1() {
  T1Watch();
  if (g_t1Pending) {
    if (g_t1Shown) {                                       // deliver the edge
      g_t1Shown = false;
      cli(); --g_t1Pending; sei();
    } else g_t1Shown = true;                               // set up the next one
    return g_t1Shown;
  }
  g_t1Shown = g_t1Level;
  return g_t1Shown;
}

namespace emu101 {

bool available() { return ROM101_PRESENT != 0; }
const uint8_t* ram() { return g_cpu.ram; }   // sim / bench inspection
uint16_t cyclesPerSec100() { return g_cyc100; }
uint16_t passesPerSec()    { return g_pps; }
uint16_t t1EdgesPerSec()   { return g_eps; }
uint16_t runPerMille()     { return g_runPm; }
uint16_t shPerMille()      { return g_shPm; }
uint16_t benchUs(bool ints) { return ints ? g_benchIntUs : g_benchNoIntUs; }
uint16_t benchDacUs()  { return g_benchDacUs; }
uint16_t benchPollUs() { return g_benchPollUs; }
uint16_t benchB(uint8_t i) { return i < 6 ? g_benchB[i] : 0; }

static void OnSysex(const uint8_t* body, uint8_t len) { sysex::HandleCommon(body, len); }
static void OnRealtime(uint8_t b) { midiclk::OnRealtime(b); }

// --- battery-RAM persistence -------------------------------------------------
static uint8_t  g_ramShadow[EMU_RAM_COUNT];   // what EEPROM holds (or held at load)
static uint8_t  g_ramSeen[EMU_RAM_COUNT];     // what the change scan last saw
static uint32_t g_ramQuietMs = 0;              // last time the region changed
static bool     g_ramDirty   = false;

static void RamLoad() {
  // The shadow is what EEPROM really holds (0xFF where never written), so
  // the first save after a blank EEPROM writes every byte, zeros included:
  // the ROM's LOAD clears the region to 0 and expects 0, not 0xFF, after
  // the last step.
  for (uint8_t i = 0; i < EMU_RAM_COUNT; ++i) g_ramShadow[i] = eeprom_read_byte(EE_EMU_RAM + i);
  if (eeprom_read_byte(EE_EMU_MAGIC) == EE_EMU_MAGIC_VAL)
    memcpy(g_cpu.ram + EMU_RAM_FIRST, g_ramShadow, EMU_RAM_COUNT);
  memcpy(g_ramSeen, g_cpu.ram + EMU_RAM_FIRST, EMU_RAM_COUNT);
}
// Called every pass: note changes; once quiet for 1 s, write one changed
// byte per pass (each write costs ~3.4 ms of blocking).
static void RamPersistTick(uint32_t ms) {
  static uint8_t scan = 0;
  // change detection, a few bytes per pass so a pass stays cheap
  for (uint8_t n = 0; n < 8; ++n) {
    uint8_t v = g_cpu.ram[EMU_RAM_FIRST + scan];
    if (v != g_ramSeen[scan]) { g_ramSeen[scan] = v; g_ramDirty = true; g_ramQuietMs = ms; }
    if (++scan >= EMU_RAM_COUNT) scan = 0;   // no modulo: -Os made that a division call per byte
  }
  if (!g_ramDirty || (uint32_t)(ms - g_ramQuietMs) < 1000) return;
  for (uint8_t i = 0; i < EMU_RAM_COUNT; ++i) {
    uint8_t v = g_cpu.ram[EMU_RAM_FIRST + i];
    if (v != g_ramShadow[i]) {
      eeprom_update_byte(EE_EMU_RAM + i, v);
      g_ramShadow[i] = v;
      return;                                  // one byte per pass
    }
  }
  eeprom_update_byte(EE_EMU_MAGIC, EE_EMU_MAGIC_VAL);   // image complete
  g_ramDirty = false;
}

// Boot bench: interpreter speed alone, interrupts off / on (probe raw[27..30]).
static uint16_t BenchRun(bool ints) {
#ifdef __AVR__
  Mcs48 c; mcs48_reset(c);
  uint8_t sreg = SREG;
  if (ints) sei(); else cli();
  TCNT1 = 0;
  uint16_t ran = 0;
  while (ran < 4000) ran = (uint16_t)(ran + mcs48_run(c, 64));
  uint16_t t = TCNT1;
  SREG = sreg;
  return (uint16_t)(t * 2UL);   // F_CPU/8 ticks -> us at 4 MHz
#else
  (void)ints; return 0;
#endif
}

void setup() {
  hw::Init();
  T1PinEdgeInit();
  g_hleOk = HleCheck();
  g_benchNoIntUs = BenchRun(false);
  g_benchIntUs   = BenchRun(true);
  BUS_PORT = 0xFF; P2_PORT = 0xFF;   // the bench ran ROM code against the ports; re-idle
#ifdef __AVR__
  { uint32_t t = micros(); for (uint8_t i = 0; i < 10; ++i) hw::DacWrite(0, ROUTE_NONE); g_benchDacUs = (uint16_t)(micros() - t); BUS_PORT = 0xFF; }
  { uint32_t t = micros(); for (uint8_t i = 0; i < 100; ++i) midi::Poll();        g_benchB[0] = (uint16_t)(micros() - t); }
  { uint32_t t = micros(); for (uint8_t i = 0; i < 100; ++i) T1Watch();           g_benchB[1] = (uint16_t)(micros() - t); }
  { uint32_t t = micros(); for (uint8_t i = 0; i < 100; ++i) RateTick(millis(), 0); g_benchB[2] = (uint16_t)(micros() - t); }
  { uint32_t t = micros(); for (uint8_t i = 0; i < 100; ++i) RamPersistTick(millis()); g_benchB[3] = (uint16_t)(micros() - t); }
  { uint32_t t = micros(); for (uint8_t i = 0; i < 100; ++i) { (void)micros(); }  g_benchB[4] = (uint16_t)(micros() - t); }
  { uint32_t t = micros(); for (uint8_t i = 0; i < 100; ++i) { uint8_t sb = BUS_PORT; (void)hw::ScanRead(COL_BUTTONS); BUS_PORT = sb; } g_benchB[5] = (uint16_t)(micros() - t); }
  g_benchPollUs = g_benchB[0];
#endif
  midi::Handlers h;
  h.noteOn = MidiNoteOn; h.noteOff = MidiNoteOff;
  h.sysex = OnSysex; h.realtime = OnRealtime;
  midi::SetHandlers(h);
  // 80C49 reset state: all port latches high (gate off, LEDs per reset).
  BUS_PORT = 0xFF;
  P2_PORT  = 0xFF;
  mcs48_reset(g_cpu);
  RamLoad();          // the "battery" kept the sequence: restore it before the ROM runs
}

// Slices: 30 instructions (~48 cycles = 120 us of emulated time, ~0.4 ms real
// at the 4 MHz core clock), the granularity of the S/H refresh check;
// SLICES_PER_PASS of them per host loop() pass, between which the
// MIDI/menu/persistence polls and the host loop overhead run once (they cost
// ~0.3 ms, several emulated instructions' worth, per pass).
// The host sim keeps ~200 cycles = 500 us of virtual time per loop(): its
// self-test counts loops at that rate, and virtual time is not real time.
#ifdef SUPEROS_SIM
static const uint8_t SLICE_INSNS = 64;
static const uint8_t SLICES_PER_PASS = 2;
#else
static const uint8_t SLICE_INSNS = 30;
static const uint8_t SLICES_PER_PASS = 4;
#endif

void loop() {
  // Several slices per host pass: the S/H refresh check and the sync-clock
  // watch sit between slices (cheap), the polls below run once per pass.
  // Fewer host passes per ROM pass = less overhead per emulated instruction.
  uint16_t ran = 0;
  static uint32_t s_shLastUs = 0;
  for (uint8_t i = 0; i < SLICES_PER_PASS; ++i) {
    uint32_t tRun = micros();
    ran = (uint16_t)(ran + mcs48_run(g_cpu, SLICE_INSNS));
    uint32_t nowUs = micros();
    g_runUsAcc += nowUs - tRun;
    // S/H refresh at a SuperOS-like cadence (see ShRefresh); time-based, since
    // the droop is per real millisecond however slow the interpreter runs.
    if ((uint32_t)(nowUs - s_shLastUs) >= SH_REFRESH_US) { s_shLastUs = nowUs; ShRefresh(); g_shUsAcc += micros() - nowUs; }
    if (midiclk::SyncOn()) T1Watch();   // divided-clock edges (pin edges arrive via INT0)
  }

#ifdef SUPEROS_SIM
  // Host sim: advance virtual time instead of throttling (2.5 us/cycle).
  delayMicroseconds((uint32_t)ran * 5 / 2);
#else
  // Throttle to real time; if we're behind by more than ~10 ms, resync
  // instead of sprinting to catch up.
  static uint32_t s_dueUs = 0;
  if (s_dueUs == 0) s_dueUs = micros();
  s_dueUs += (uint32_t)ran * 5 / 2;
  uint32_t now = micros();
  if ((int32_t)(s_dueUs - now) > 0) {
    // ahead of real time: sleep the difference (hw::IdleWaitUs) instead of
    // spinning, then trim any remainder
    int32_t ahead = (int32_t)(s_dueUs - now);
    if (ahead > 8) hw::IdleWaitUs((uint16_t)(ahead > 60000 ? 60000 : ahead));
    while ((int32_t)(s_dueUs - micros()) > 0) {}
  }
  else if ((int32_t)(now - s_dueUs) > 10000) s_dueUs = now;
#endif

  midi::Poll();   // notes into the matrix, divided clock (emulated T1), SysEx
  T1Watch();      // sync-clock edge right after Poll advanced it
  uint32_t ms = millis();
  RateTick(ms, ran);
  RamPersistTick(ms);

  // Config-menu combo poll every ~10 ms: scan the buttons ourselves, without
  // disturbing the BUS latch the emulated ROM left behind (route 11 scans
  // can't corrupt a S/H; restoring the latch keeps the ROM's own scan state).
  static uint32_t s_lastPollMs = 0;
  if (ms - s_lastPollMs >= 10) {
    s_lastPollMs = ms;
    uint8_t savedBus = BUS_PORT;
    uint8_t buttons = hw::ScanRead(COL_BUTTONS);
    BUS_PORT = savedBus;
    menu::NotifyButtons(buttons, ms);
  }
}

} // namespace emu101
