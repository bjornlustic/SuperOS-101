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
// flat out and the emulated firmware simply runs proportionally slower (pitch
// CVs are unaffected — only scan latency and software delay loops stretch).
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
#include "mcs48.h"
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
#endif
#include "rom101_data.h"

static Mcs48 g_cpu;

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

// --- core hooks: 1:1 port pass-through ---------------------------------------
uint8_t mcs48_rom_read(uint16_t addr) { return pgm_read_byte(&ROM101[addr & 0x7FF]); }
void    mcs48_bus_write(uint8_t v)    {
  BUS_PORT = v;
  if ((v & 0xC0) == (ROUTE_CV_OUT << 6)) { g_cvOut = (uint8_t)(~v & 0x3F); EchoOut(); }
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
// With MIDI sync ON the divided MIDI clock replaces the LFO/EXT-CLK on the
// emulated T1 pin, so the stock ROM's arp/sequencer runs DAW-locked too.
bool    mcs48_t1() {
  if (midiclk::SyncOn()) return midiclk::T1Level();
  return (PIND & (1u << T1_CLOCK_BIT)) != 0;
}

namespace emu101 {

bool available() { return ROM101_PRESENT != 0; }
const uint8_t* ram() { return g_cpu.ram; }   // sim / bench inspection

static void OnSysex(const uint8_t* body, uint8_t len) { sysex::HandleCommon(body, len); }
static void OnRealtime(uint8_t b) { midiclk::OnRealtime(b); }

// --- battery-RAM persistence -------------------------------------------------
static uint8_t  g_ramShadow[EMU_RAM_COUNT];   // what EEPROM holds (or held at load)
static uint8_t  g_ramSeen[EMU_RAM_COUNT];     // what the change scan last saw
static uint32_t g_ramQuietMs = 0;              // last time the region changed
static bool     g_ramDirty   = false;

static void RamLoad() {
  if (eeprom_read_byte(EE_EMU_MAGIC) == EE_EMU_MAGIC_VAL)
    for (uint8_t i = 0; i < EMU_RAM_COUNT; ++i)
      g_cpu.ram[EMU_RAM_FIRST + i] = eeprom_read_byte(EE_EMU_RAM + i);
  memcpy(g_ramShadow, g_cpu.ram + EMU_RAM_FIRST, EMU_RAM_COUNT);
  memcpy(g_ramSeen,   g_cpu.ram + EMU_RAM_FIRST, EMU_RAM_COUNT);
}
// Called every pass: note changes; once quiet for 1 s, write one changed
// byte per pass (each write costs ~3.4 ms of blocking).
static void RamPersistTick(uint32_t ms) {
  static uint8_t scan = 0;
  // change detection, a few bytes per pass so a pass stays cheap
  for (uint8_t n = 0; n < 8; ++n) {
    uint8_t v = g_cpu.ram[EMU_RAM_FIRST + scan];
    if (v != g_ramSeen[scan]) { g_ramSeen[scan] = v; g_ramDirty = true; g_ramQuietMs = ms; }
    scan = (uint8_t)((scan + 1) % EMU_RAM_COUNT);
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

void setup() {
  hw::Init();
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

// One slice per loop() pass: 200 cycles = 500 us of emulated time.
static const uint16_t SLICE_CYCLES = 200;

void loop() {
  uint16_t ran = 0;
  while (ran < SLICE_CYCLES) ran = (uint16_t)(ran + mcs48_step(g_cpu));

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
  if ((int32_t)(s_dueUs - now) > 0)       while ((int32_t)(s_dueUs - micros()) > 0) {}
  else if ((int32_t)(now - s_dueUs) > 10000) s_dueUs = now;
#endif

  midi::Poll();   // notes into the matrix, divided clock (emulated T1), SysEx
  RamPersistTick(millis());

  // Config-menu combo poll every ~10 ms: scan the buttons ourselves, without
  // disturbing the BUS latch the emulated ROM left behind (route 11 scans
  // can't corrupt a S/H; restoring the latch keeps the ROM's own scan state).
  static uint32_t s_lastPollMs = 0;
  uint32_t ms = millis();
  if (ms - s_lastPollMs >= 10) {
    s_lastPollMs = ms;
    uint8_t savedBus = BUS_PORT;
    uint8_t buttons = hw::ScanRead(COL_BUTTONS);
    BUS_PORT = savedBus;
    menu::NotifyButtons(buttons, ms);
  }
}

} // namespace emu101
