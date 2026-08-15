// SuperOS-101 emulator — host-side hardware model + CLI
//
// Compiles the REAL firmware (src/main.cpp and its headers) against sim/shim/
// and models the analog-board side of the 80C49 socket at pin level:
//   - BUS writes: route bits DB7/DB6 latch the active-low 6-bit value into the
//     matching 4052 sample/hold (CV-out / VCO-range / random), exactly as the
//     hardware would. Route 11 never touches a S/H (the firmware invariant).
//   - Scan: when the firmware pulls one BUS column low, P1 reads return the
//     simulated keyboard / panel-switch / button rows in the active-low domain.
//   - P2: GATE (bit 7, inverted) and CLOCK-RESET (bit 4) are decoded for display.
//   - T1 = clock input, T0 = HOLD pedal (active-low).
//
// Run `./superos101-sim` for the interactive CLI (type `help`), or
// `./superos101-sim --test` for the scripted self-test (exit 0 = pass).
#include <Arduino.h>  // sim/shim — registers, BusPort, setup()/loop()
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csetjmp>
#include <string>
#include <sstream>
#include <iostream>

#include <deque>

// Combined-build hooks (src/combined.h / boot.cpp / emu101.h / midiclock.h)
uint8_t combined_current_fw();
void RebootTo(uint8_t fw);
namespace emu101 { bool available(); }
#include "midiclock.h"   // inline; resolved via -I ../src
#include "midi.h"
#include "flash_persist.h"
static const uint8_t SIM_FW_EMU = 1;

// MIDI-in byte queue, drained by midi::Poll() via sim_midi_pop().
static std::deque<uint8_t> g_midiQ;
extern "C" int16_t sim_midi_pop() {
  if (g_midiQ.empty()) return -1;
  int16_t b = g_midiQ.front(); g_midiQ.pop_front(); return b;
}
// MIDI-out capture (device -> host), inspected by the self-test / `midi out`.
static std::deque<uint8_t> g_midiOut;
extern "C" void sim_midi_out(uint8_t b) { g_midiOut.push_back(b); if (g_midiOut.size() > 4096) g_midiOut.pop_front(); }

// RebootTo() in the sim lands here: back to boot, firmware re-dispatches.
static jmp_buf g_rebootJmp;
static bool    g_rebootArmed = false;
extern "C" void sim_reboot() {
  if (g_rebootArmed) longjmp(g_rebootJmp, 1);
  fprintf(stderr, "sim_reboot before boot?\n");
  exit(1);
}

// ---------------------------------------------------------------------------
// AVR register storage
// ---------------------------------------------------------------------------
uint8_t sim_ddrc, sim_ddrb, sim_portb, sim_ddrf, sim_portf = 0x80, sim_ddrd = 0xFF, sim_portd;

// ---------------------------------------------------------------------------
// Simulated synth state (what the analog board / panel present to the CPU)
// ---------------------------------------------------------------------------
namespace sim {

static uint32_t g_keys      = 0;      // held keys, bit 0 = lowest (F3), 1 = down
static uint8_t  g_buttons   = 0;      // function buttons, 1 = pressed (Button enum bits)
static bool     g_pedalHold = false;  // T0 pedal, closed = HOLD
static bool     g_clock     = false;  // T1 level

// Panel switches (friendly form; raw byte computed at scan time)
static uint8_t  g_rangeFeet = 8;      // 16 / 8 / 4 / 2
static int      g_octave    = 0;      // -1 down / 0 mid / +1 up
static int      g_adsr      = 0;      // 0 = GATE, 1 = LFO, 2 = GATE+TRIG

// 4052 sample/holds, indexed by Route (0 = CV-out, 1 = VCO range, 2 = random)
static uint8_t  g_sh[3]     = {0, 0, 0};
static uint8_t  g_busLast   = 0xFF;
static uint64_t g_virtualUs = 0;

// RANGE/OCT/ADSR raw row (active-low, see controls::ReadPanel):
//   d0 = RANGE 16'/4'   d1 = RANGE 16'/8'   d3 = ADSR LFO   d4 = ADSR GATE+TRIG
//   d5 = OCT (decoded DOWN when set alone)  d6 = OCT (decoded UP when set)
static uint8_t SwitchRow() {
  uint8_t raw = 0xFF;
  if (g_rangeFeet == 16 || g_rangeFeet == 4) raw &= ~(1u << 0);
  if (g_rangeFeet == 16 || g_rangeFeet == 8) raw &= ~(1u << 1);
  if (g_adsr == 1) raw &= ~(1u << 3);
  if (g_adsr == 2) raw &= ~(1u << 4);
  // Octave decode in controls.h: d6 set -> UP, else d5 set -> DOWN, else MIDDLE.
  raw &= ~((1u << 5) | (1u << 6));
  if (g_octave > 0) raw |= (1u << 6);
  if (g_octave < 0) raw |= (1u << 5);
  return raw;
}

// P1 read: which BUS column (DB0–5) is the firmware holding low?
uint8_t ReadP1() {
  uint8_t lowMask = (uint8_t)~g_busLast;
  for (uint8_t col = 0; col < 6; ++col) {
    if (!(lowMask & (1u << col))) continue;
    if (col < 4)  return (uint8_t)~((g_keys >> (col * 8)) & 0xFF); // kbd bank, active-low
    if (col == 4) return SwitchRow();
    return (uint8_t)~g_buttons;                                    // buttons, active-low
  }
  return 0xFF; // no column driven -> all rows pulled up
}

uint8_t ReadPinD() {
  uint8_t v = 0;
  if (g_clock)      v |= (1u << 0); // T1
  if (!g_pedalHold) v |= (1u << 1); // T0 active-low: open = 1
  return v;
}

// BUS write hook: model the 4052. Route != 11 connects that S/H to the ladder.
BusPort& BusPort::operator=(uint8_t v) {
  g_busLast = v;
  uint8_t route = (uint8_t)(v >> 6);
  if (route != 3) g_sh[route] = (uint8_t)(~v) & 0x3F; // active-low data
  return *this;
}
BusPort::operator uint8_t() const { return g_busLast; }
BusPort busPort;

} // namespace sim

void delayMicroseconds(uint32_t us) { sim::g_virtualUs += us; }
uint32_t micros() { return (uint32_t)sim::g_virtualUs; }
uint32_t millis() { return (uint32_t)(sim::g_virtualUs / 1000); }

// ---------------------------------------------------------------------------
// EEPROM shim (file-backed)
// ---------------------------------------------------------------------------
static const size_t EE_SIZE = 4096;
static uint8_t g_eeprom[EE_SIZE];
static bool    g_eeLoaded = false;

static std::string EepromPath() {
  const char* p = getenv("SIM_EEPROM");
  return p ? p : "sim-eeprom.bin";
}
static void EepromLoad() {
  if (g_eeLoaded) return;
  g_eeLoaded = true;
  memset(g_eeprom, 0xFF, EE_SIZE); // erased state, like a fresh chip
  if (FILE* f = fopen(EepromPath().c_str(), "rb")) {
    size_t n = fread(g_eeprom, 1, EE_SIZE, f);
    (void)n;
    fclose(f);
  }
}
static void EepromFlush() {
  if (FILE* f = fopen(EepromPath().c_str(), "wb")) {
    fwrite(g_eeprom, 1, EE_SIZE, f);
    fclose(f);
  }
}
uint8_t eeprom_read_byte(const uint8_t* addr) {
  EepromLoad();
  size_t a = (size_t)(uintptr_t)addr;
  return a < EE_SIZE ? g_eeprom[a] : 0xFF;
}
void eeprom_update_byte(uint8_t* addr, uint8_t value) {
  EepromLoad();
  size_t a = (size_t)(uintptr_t)addr;
  if (a < EE_SIZE && g_eeprom[a] != value) { g_eeprom[a] = value; EepromFlush(); }
}

// ---------------------------------------------------------------------------
// Flash arena shim (file-backed, 256-byte pages; see src/flash_eeprom.h)
// ---------------------------------------------------------------------------
static const size_t FL_PAGES = 0x200, FL_PAGE = 256;
static uint8_t g_flash[FL_PAGES * FL_PAGE];
static bool    g_flLoaded = false;
static std::string FlashPath() {
  const char* p = getenv("SIM_FLASH");
  return p ? p : "sim-flash.bin";
}
static void FlashLoad() {
  if (g_flLoaded) return;
  g_flLoaded = true;
  memset(g_flash, 0xFF, sizeof(g_flash));
  if (FILE* f = fopen(FlashPath().c_str(), "rb")) { size_t n = fread(g_flash, 1, sizeof(g_flash), f); (void)n; fclose(f); }
}
static void FlashFlush() {
  if (FILE* f = fopen(FlashPath().c_str(), "wb")) { fwrite(g_flash, 1, sizeof(g_flash), f); fclose(f); }
}
extern "C" uint8_t sim_flash_program(uint16_t page, const uint8_t* buf) {
  FlashLoad();
  if (page < 0x100 || page > 0x1DF) return 1;   // same guard as flash_service.c
  memcpy(g_flash + (size_t)page * FL_PAGE, buf, FL_PAGE);
  FlashFlush();
  return 0;
}
extern "C" void sim_flash_read(uint16_t page, uint8_t* buf) {
  FlashLoad();
  if (page >= FL_PAGES) { memset(buf, 0xFF, FL_PAGE); return; }
  memcpy(buf, g_flash + (size_t)page * FL_PAGE, FL_PAGE);
}

// ---------------------------------------------------------------------------
// Driving the firmware
// ---------------------------------------------------------------------------
static void RunLoops(int n) { while (n-- > 0) loop(); }

// One clock pulse: T1 high for a loop pass, then low (the firmware fires on H->L).
static void ClockTick() {
  sim::g_clock = true;  RunLoops(1);
  sim::g_clock = false; RunLoops(1);
}

static bool GateOn()     { return !(sim_portf & 0x80); }
static bool ClockReset() { return  (sim_portf & 0x10) == 0; }   // active-LOW pulse

static void PrintState() {
  printf("[%s] cv=%2u  gate=%s  vco-range=%2u  random=%2u  clkrst=%d  p2=%02X  keys=%08X  virt=%lluus\n",
         combined_current_fw() == SIM_FW_EMU ? "emu" : "sos",
         sim::g_sh[0], GateOn() ? "ON " : "off", sim::g_sh[1], sim::g_sh[2],
         ClockReset() ? 1 : 0, sim_portf, sim::g_keys, (unsigned long long)sim::g_virtualUs);
}

// ---------------------------------------------------------------------------
// Interactive CLI
// ---------------------------------------------------------------------------
static int ButtonBit(const std::string& name) {
  if (name == "load")   return 0;
  if (name == "play")   return 1;
  if (name == "down")   return 2;
  if (name == "ud")     return 3;
  if (name == "up")     return 4;
  if (name == "hold")   return 5;
  if (name == "ktrans") return 7;
  return -1;
}

static void Help() {
  puts("commands:");
  puts("  key <0-31> down|up          press/release a keyboard key (0 = F3)");
  puts("  btn <name> down|up|tap      load play down ud up hold ktrans");
  puts("  sw range 16|8|4|2           RANGE switch");
  puts("  sw oct up|mid|down          OCTAVE transpose");
  puts("  sw adsr gate|lfo|trig       ADSR mode (trig = GATE+TRIG, last-note prio)");
  puts("  pedal on|off                HOLD pedal (T0)");
  puts("  tick [n]                    n clock pulses (default 1), arp/seq advance");
  puts("  run [n]                     run n main-loop passes (default 1)");
  puts("  mode emu|superos            write the firmware-select byte and reboot");
  puts("  midi start|stop|continue    MIDI realtime transport");
  puts("  midi clock [n]              n MIDI clock bytes (24 ppqn), one per loop pass");
  puts("  midi sync on|off            toggle MIDI clock sync (as the config menu does)");
  puts("  midi div <0-9>              division: 0=1/4 1=1/4d 2=1/4t 3=1/8 4=1/8d");
  puts("                              5=1/8t 6=1/16 7=1/16d 8=1/16t 9=1/32");
  puts("  state                       show firmware, S/H CVs, gate, held keys");
  puts("  quit");
}

static void Repl() {
  if (setjmp(g_rebootJmp)) puts("[reboot]");
  g_rebootArmed = true;
  setup();
  RunLoops(2); // settle: panel decode + initial S/H writes
  puts("SuperOS-101 sim. `help` for commands.");
  PrintState();
  static std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream ss(line);
    std::string cmd; ss >> cmd;
    if (cmd.empty()) continue;
    if (cmd == "quit" || cmd == "exit") break;
    if (cmd == "help") { Help(); continue; }
    if (cmd == "state") { PrintState(); continue; }
    if (cmd == "key") {
      int k = -1; std::string act; ss >> k >> act;
      if (k < 0 || k > 31 || (act != "down" && act != "up")) { puts("usage: key <0-31> down|up"); continue; }
      if (act == "down") sim::g_keys |= (1UL << k); else sim::g_keys &= ~(1UL << k);
      RunLoops(1);
    } else if (cmd == "btn") {
      std::string name, act; ss >> name >> act;
      int b = ButtonBit(name);
      if (b < 0) { puts("unknown button"); continue; }
      if (act == "down")      { sim::g_buttons |= (1u << b);  RunLoops(1); }
      else if (act == "up")   { sim::g_buttons &= ~(1u << b); RunLoops(1); }
      else if (act == "tap")  { sim::g_buttons |= (1u << b);  RunLoops(1);
                                sim::g_buttons &= ~(1u << b); RunLoops(1); }
      else { puts("usage: btn <name> down|up|tap"); continue; }
    } else if (cmd == "sw") {
      std::string which, val; ss >> which >> val;
      if (which == "range")      sim::g_rangeFeet = atoi(val.c_str());
      else if (which == "oct")   sim::g_octave = (val == "up") ? 1 : (val == "down") ? -1 : 0;
      else if (which == "adsr")  sim::g_adsr = (val == "lfo") ? 1 : (val == "trig") ? 2 : 0;
      else { puts("usage: sw range|oct|adsr <val>"); continue; }
      RunLoops(1);
    } else if (cmd == "pedal") {
      std::string val; ss >> val;
      sim::g_pedalHold = (val == "on");
      RunLoops(1);
    } else if (cmd == "tick") {
      int n = 1; ss >> n;
      while (n-- > 0) { ClockTick(); PrintState(); }
      continue;
    } else if (cmd == "run") {
      int n = 1; ss >> n;
      RunLoops(n);
    } else if (cmd == "midi") {
      std::string sub, val; ss >> sub >> val;
      if (sub == "start")         { g_midiQ.push_back(0xFA); RunLoops(1); }
      else if (sub == "continue") { g_midiQ.push_back(0xFB); RunLoops(1); }
      else if (sub == "stop")     { g_midiQ.push_back(0xFC); RunLoops(1); }
      else if (sub == "clock") {
        int n = val.empty() ? 1 : atoi(val.c_str());
        while (n-- > 0) { g_midiQ.push_back(0xF8); RunLoops(1); }
      }
      else if (sub == "sync")     { midiclk::SetSync(val == "on"); RunLoops(1); }
      else if (sub == "div")      { midiclk::SetDiv((uint8_t)atoi(val.c_str())); RunLoops(1); }
      else { puts("usage: midi start|stop|continue|clock [n]|sync on/off|div <0-9>"); continue; }
    } else if (cmd == "mode") {
      std::string val; ss >> val;
      if (val == "emu" && !emu101::available()) { puts("no ROM embedded"); continue; }
      if (val == "emu" || val == "superos") RebootTo(val == "emu" ? 1 : 0); // longjmps
      puts("usage: mode emu|superos");
      continue;
    } else {
      puts("unknown command; `help`");
      continue;
    }
    PrintState();
  }
}

// ---------------------------------------------------------------------------
// Self-test: keyboard, priority, arp, sequencer, EEPROM persistence
// ---------------------------------------------------------------------------
static void MidiByte(uint8_t b) { g_midiQ.push_back(b); RunLoops(1); }
static void MidiClocks(int n)   { while (n-- > 0) MidiByte(0xF8); }

static int g_fails = 0;
#define CHECK(cond, msg) do { \
    if (cond) printf("  PASS  %s\n", msg); \
    else      { printf("  FAIL  %s\n", msg); ++g_fails; } \
  } while (0)

static int SelfTest() {
  setenv("SIM_EEPROM", "sim-selftest-eeprom.bin", 1);
  remove("sim-selftest-eeprom.bin");
  setenv("SIM_FLASH", "sim-selftest-flash.bin", 1);
  remove("sim-selftest-flash.bin");
  g_rebootArmed = true;
  setup();
  RunLoops(2);

  // Defaults: RANGE 8' -> VCO range CV 0x18, oct MIDDLE, gate off.
  puts("panel / DAC protocol:");
  CHECK(sim::g_sh[1] == 0x18, "RANGE 8' -> VCO range CV 0x18");
  CHECK(!GateOn(), "gate off with no keys");
  sim::g_rangeFeet = 16; RunLoops(1);
  CHECK(sim::g_sh[1] == 0x0C, "RANGE 16' -> VCO range CV 0x0C");
  sim::g_rangeFeet = 8; RunLoops(1);

  // Keyboard: key 0 -> CV = 0 + 0x0C (oct mid) + 5 (CV_BASE) = 17.
  puts("mono keyboard:");
  sim::g_keys = 1UL << 0; RunLoops(1);
  CHECK(sim::g_sh[0] == 17 && GateOn(), "key 0 -> CV 17, gate on");
  sim::g_keys |= 1UL << 10; RunLoops(1);
  CHECK(sim::g_sh[0] == 17, "lowest-note priority holds key 0 over key 10");
  sim::g_keys = 0; RunLoops(1);
  CHECK(!GateOn() && sim::g_sh[0] == 17, "release -> gate off, CV held");
  sim::g_octave = 1; sim::g_keys = 1UL << 0; RunLoops(1);
  CHECK(sim::g_sh[0] == 29, "octave UP -> CV 17+12");
  sim::g_octave = 0; sim::g_keys = 0; RunLoops(1);

  // Last-note priority (ADSR = GATE+TRIG).
  sim::g_adsr = 2; RunLoops(1);
  sim::g_keys = 1UL << 10; RunLoops(1);
  sim::g_keys |= 1UL << 2; RunLoops(1);
  CHECK(sim::g_sh[0] == 2 + 17, "last-note priority follows the fresh key");
  sim::g_adsr = 0; sim::g_keys = 0; RunLoops(1);

  // Arpeggiator UP over {0, 4, 7}, clock-advanced.
  puts("arpeggiator:");
  sim::g_buttons = 1u << 4; RunLoops(1); sim::g_buttons = 0; RunLoops(1); // tap UP
  sim::g_keys = (1UL << 0) | (1UL << 4) | (1UL << 7);
  ClockTick(); CHECK(sim::g_sh[0] == 17 && GateOn(), "arp step 1 = key 0");
  ClockTick(); CHECK(sim::g_sh[0] == 21, "arp step 2 = key 4");
  ClockTick(); CHECK(sim::g_sh[0] == 24, "arp step 3 = key 7");
  ClockTick(); CHECK(sim::g_sh[0] == 17, "arp wraps to key 0");
  sim::g_keys = 0; RunLoops(1);
  CHECK(!GateOn(), "arp gate off when keys released");
  sim::g_buttons = 1u << 4; RunLoops(1); sim::g_buttons = 0; RunLoops(1); // arp off

  // Sequencer: record 0, 2, rest; save; play back.
  puts("sequencer:");
  sim::g_buttons = 1u << 0; RunLoops(1); sim::g_buttons = 0; RunLoops(1); // LOAD (record)
  sim::g_keys = 1UL << 0; RunLoops(1); sim::g_keys = 0; RunLoops(1);
  sim::g_keys = 1UL << 2; RunLoops(1); sim::g_keys = 0; RunLoops(1);
  sim::g_buttons = 1u << 7; RunLoops(1); sim::g_buttons = 0; RunLoops(1); // KEY-TRANSP = rest
  sim::g_buttons = 1u << 0; RunLoops(1); sim::g_buttons = 0; RunLoops(1); // LOAD (save)
  {
    uint8_t buf[101]; uint8_t n = persist::Store().read(persist::FB_PATTERN_BASE + 0, buf, sizeof(buf));
    CHECK(n == 4 && buf[0] == 3, "pattern 0 saved to the flash arena (len 3)");
  }
  sim::g_buttons = 1u << 1; RunLoops(1); sim::g_buttons = 0; RunLoops(1); // PLAY
  ClockTick(); CHECK(sim::g_sh[0] == 17 && GateOn(), "seq starts at step 1 -> note 0 (CV 17)");
  ClockTick(); CHECK(sim::g_sh[0] == 19 && GateOn(), "seq step 2 -> note 2 (CV 19)");
  ClockTick(); CHECK(!GateOn(), "seq rest -> gate off");
  ClockTick(); CHECK(sim::g_sh[0] == 17 && GateOn(), "seq wraps -> note 0 (CV 17)");

  // MIDI in/out: note 53 = key 0 (CV 17); the engine echoes notes on the OUT.
  puts("MIDI:");
  sim::g_buttons = 1u << 1; RunLoops(1); sim::g_buttons = 0; RunLoops(1); // PLAY off
  g_midiOut.clear();
  for (uint8_t b : {0x90, 53, 100}) g_midiQ.push_back(b); RunLoops(1);
  CHECK(sim::g_sh[0] == 17 && GateOn(), "MIDI note 53 -> key 0, CV 17, gate on");
  CHECK(g_midiOut.size() >= 3 && g_midiOut[0] == 0x90 && g_midiOut[1] == 53, "note echoed on MIDI OUT (note on 53)");
  for (uint8_t b : {0x80, 53, 0}) g_midiQ.push_back(b); RunLoops(1);
  CHECK(!GateOn(), "MIDI note off -> gate off");
  CHECK(g_midiOut.size() >= 6 && g_midiOut[3] == 0x80 && g_midiOut[4] == 53, "note off echoed on MIDI OUT");
  for (uint8_t b : {0x90, 40, 100}) g_midiQ.push_back(b); RunLoops(1);   // below the keyboard
  CHECK(sim::g_sh[0] == 4 && GateOn(), "MIDI note 40 (out of key range) -> CV 4 mono override");
  for (uint8_t b : {0x80, 40, 0}) g_midiQ.push_back(b); RunLoops(1);
  // SysEx: config request, pattern round trip via 0x12/0x10.
  g_midiOut.clear();
  for (uint8_t b : {0xF0, 0x7D, 0x20, 0xF7}) g_midiQ.push_back(b); RunLoops(1);
  CHECK(g_midiOut.size() > 8 && g_midiOut[0] == 0xF0 && g_midiOut[1] == 0x7D && g_midiOut[2] == 0x21 && g_midiOut[3] == 0,
        "SysEx 0x20 -> 0x21 config reply (fw 0)");
  {
    uint8_t raw[4] = {3, 20, 0x40 | 0, 0x80 | 22};   // len 3: note 20, rest, tie 22
    uint8_t x = 0; for (uint8_t i = 0; i < 4; ++i) x ^= raw[i];
    uint8_t packed[8]; uint8_t n = midi::Pack7(raw, 4, packed);
    g_midiOut.clear();
    for (uint8_t b : {(uint8_t)0xF0, (uint8_t)0x7D, (uint8_t)0x12, (uint8_t)5, (uint8_t)(x & 0x7F), (uint8_t)((x >> 7) & 1)}) g_midiQ.push_back(b);
    for (uint8_t i = 0; i < n; ++i) g_midiQ.push_back(packed[i]);
    g_midiQ.push_back(0xF7); RunLoops(1);
    CHECK(g_midiOut.size() == 5 && g_midiOut[2] == 0x14 && g_midiOut[3] == 0, "SysEx 0x12 set pattern 5 -> ack 0");
    g_midiOut.clear();
    for (uint8_t b : {0xF0, 0x7D, 0x10, 5, 0xF7}) g_midiQ.push_back(b); RunLoops(1);
    uint8_t body[8] = {0}; uint8_t m = midi::Unpack7(&g_midiOut[6], (uint8_t)(g_midiOut.size() - 7), body, sizeof(body));
    CHECK(g_midiOut[2] == 0x11 && g_midiOut[3] == 5 && m == 4 && memcmp(body, raw, 4) == 0, "SysEx 0x10 -> 0x11 returns the same pattern");
    for (int i = 0; i < 2000; ++i) { delayMicroseconds(1000); RunLoops(1); }   // > 500 ms quiet: deferred flush
    uint8_t buf[101]; uint8_t k = persist::Store().read(persist::FB_PATTERN_BASE + 5, buf, sizeof(buf));
    CHECK(k == 4 && buf[0] == 3 && buf[3] == (0x80 | 22), "pattern 5 flushed to the flash arena");
  }
  sim::g_buttons = 1u << 1; RunLoops(1); sim::g_buttons = 0; RunLoops(1); // PLAY back on (pattern 0)

  // Firmware invariant: a scan must never corrupt a held S/H (route 11 only).
  puts("invariants:");
  uint8_t cvBefore = sim::g_sh[0], rngBefore = sim::g_sh[1];
  sim::g_keys = 1UL << 5; RunLoops(3); sim::g_keys = 0;
  CHECK(sim::g_sh[1] == rngBefore, "scans left the VCO-range S/H untouched");
  (void)cvBefore;

  // MIDI clock sync. The engine is still in sequencer-play mode with the
  // 3-step sequence (note 0, note 2, rest) from the tests above.
  puts("MIDI clock sync:");
  midiclk::SetSync(true); midiclk::SetDiv(6);   // 1/16 = 6 ticks at 24 ppqn
  uint8_t cvBeforeSync = sim::g_sh[0];
  ClockTick();
  CHECK(sim::g_sh[0] == cvBeforeSync, "sync on: T1 (LFO) clock is ignored");
  MidiByte(0xFA);                                // Start: rewind to the downbeat
  MidiClocks(1);                                 // first clock = downbeat edge
  CHECK(sim::g_sh[0] == 17 && GateOn(), "Start + 1st clock -> seq step 0 (CV 17)");
  MidiClocks(6);                                 // next division boundary
  CHECK(sim::g_sh[0] == 19, "6 clocks at 1/16 -> next step (CV 19)");
  CHECK(ClockReset(), "division edge pulses CLOCK RESET (LFO hard sync)");
  uint8_t rndHeld = sim::g_sh[2];
  MidiClocks(5);                                 // inside the division: no edge
  CHECK(sim::g_sh[2] == rndHeld, "random S/H holds between division edges");
  MidiByte(0xFC);                                // Stop
  MidiClocks(12);
  CHECK(sim::g_sh[0] == 19, "after Stop, clocks are ignored");
  midiclk::SetSync(false);
  ClockTick();
  CHECK(!GateOn(), "sync off: T1 clock works again (rest step -> gate off)");

  // Config menu: HOLD+KEY-TRANSPOSE 2 s -> menu; LOAD -> reboot into the
  // 80C49 emulator (the original mask ROM, if embedded).
  puts("config menu / combined boot:");
  if (emu101::available()) {
    if (!setjmp(g_rebootJmp)) {
      sim::g_buttons = (1u << 5) | (1u << 7);   // HOLD + KEY-TRANSPOSE
      RunLoops(40000);                          // > 2 s virtual, enters menu
      sim::g_buttons = 0; RunLoops(2);
      sim::g_buttons = 1u << 3; RunLoops(1);    // UP&DOWN: toggle MIDI sync on
      sim::g_buttons = 0; RunLoops(1);
      CHECK(midiclk::SyncOn(), "menu UP&DOWN toggles MIDI sync on");
      sim::g_buttons = 1u << 3; RunLoops(1);    // and back off
      sim::g_buttons = 0; RunLoops(1);
      CHECK(!midiclk::SyncOn(), "menu UP&DOWN toggles MIDI sync off");
      sim::g_buttons = 1u << 0; RunLoops(1);    // LOAD -> RebootTo(FW_EMU)
      CHECK(false, "menu LOAD should have rebooted");
    } else {
      setup();                                  // boot lands in emulator mode
    }
    CHECK(combined_current_fw() == SIM_FW_EMU, "menu LOAD -> rebooted into emulator");

    puts("80C49 emulator (original mask ROM):");
    sim::g_buttons = 0;
    RunLoops(4000);                             // ~2 s virtual of ROM execution
    CHECK(sim::g_sh[1] == 0x18, "ROM writes RANGE 8' CV 0x18 to the VCO S/H");
    CHECK(!GateOn(), "ROM: gate off with no keys");
    sim::g_keys = 1UL << 0; RunLoops(400);
    CHECK(GateOn(), "ROM: key 0 -> gate on");
    CHECK(sim::g_sh[0] == 17, "ROM: key 0 -> CV 17 (matches SuperOS model)");
    sim::g_keys = 0; RunLoops(400);
    CHECK(!GateOn(), "ROM: release -> gate off");

    // MIDI sync in 80C49 mode: the divided clock rides the emulated T1 pin,
    // so the stock ROM's arpeggiator advances on division boundaries. Slices
    // are stretched (RunLoops 10 = 5 ms per tick) so the ~3.5 ms ROM loop
    // samples both halves of the T1 square.
    midiclk::SetSync(true); midiclk::SetDiv(6);           // 1/16
    sim::g_buttons = 1u << 2; RunLoops(30);               // tap DOWN: arp down
    sim::g_buttons = 0; RunLoops(30);
    sim::g_keys = (1UL << 0) | (1UL << 4);
    RunLoops(30);
    g_midiQ.push_back(0xFA); RunLoops(10);
    for (int i = 0; i < 24; ++i) { g_midiQ.push_back(0xF8); RunLoops(10); }
    CHECK(GateOn(), "ROM + MIDI sync: arp gate on");
    uint8_t cvA = sim::g_sh[0];
    for (int i = 0; i < 6; ++i) { g_midiQ.push_back(0xF8); RunLoops(10); }
    uint8_t cvB = sim::g_sh[0];
    CHECK(cvA != cvB && (cvA == 17 || cvA == 21) && (cvB == 17 || cvB == 21),
          "ROM + MIDI sync: arp alternates keys 0/4 on divisions");
    sim::g_keys = 0; midiclk::SetSync(false); RunLoops(30);

    // Menu works from the emulator side too: PLAY -> back to SuperOS.
    if (!setjmp(g_rebootJmp)) {
      sim::g_buttons = (1u << 5) | (1u << 7);
      RunLoops(4800);                           // 2 s at 500 us/slice
      sim::g_buttons = 0; RunLoops(4);
      sim::g_buttons = 1u << 1; RunLoops(1);    // PLAY -> RebootTo(FW_SUPEROS)
      CHECK(false, "menu PLAY should have rebooted");
    } else {
      setup();
    }
    CHECK(combined_current_fw() != SIM_FW_EMU, "menu PLAY -> rebooted into SuperOS");
  } else {
    puts("  SKIP  no mask ROM embedded (SH101-reference dump absent)");
  }

  remove("sim-selftest-eeprom.bin");
  printf("\n%s (%d failure%s)\n", g_fails ? "SELF-TEST FAILED" : "self-test passed",
         g_fails, g_fails == 1 ? "" : "s");
  return g_fails ? 1 : 0;
}

int main(int argc, char** argv) {
  if (argc > 1 && !strcmp(argv[1], "--test")) return SelfTest();
  Repl();
  return 0;
}
