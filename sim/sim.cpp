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
#include <set>
#include <cstdlib>
#include <cstring>
#include <csetjmp>
#include <string>
#include <sstream>
#include <iostream>

#include <deque>
#include <vector>

// Combined-build hooks (src/combined.h / boot.cpp / emu101.h / midiclock.h)
uint8_t combined_current_fw();
void SwitchTo(uint8_t fw);
namespace emu101 { bool available(); const uint8_t* ram(); uint8_t romP2(); }
#include "midiclock.h"   // inline; resolved via -I ../src
#include "midi.h"
#include "usbmidi_codec.h"
#include "flash_persist.h"
#include "cvdac.h"
#include "engine.h"      // performance settings (DirSetting) read back directly
#include "pins.h"        // P2_GATE et al: the probe's P2 image is decoded by bit name
#include "sysex.h"       // the 0x3E bench-window clock helpers, pinned below
static const uint8_t SIM_FW_EMU = 1;
extern uint16_t g_simCvCode[2];   // last MCP4822 code per channel (sim_cvdac_write, below)

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

// Advance virtual time. Panel tests need this after tapping a function
// button: fresh keys are ignored for 30 ms after button activity (the
// engine's scan-bleed guard), exactly as a human's timing naturally allows.
static void WaitVirtMs(uint16_t ms) {
  uint32_t until = millis() + ms;
  while ((int32_t)(until - millis()) > 0) RunLoops(1);
}

// One clock pulse: T1 high for a loop pass, then low (the firmware fires on H->L).
static void ClockTick() {
  sim::g_clock = true;  RunLoops(1);
  sim::g_clock = false; RunLoops(1);
}

static bool GateOn()     { return !(sim_portf & 0x80); }
static bool ClockReset() { return  (sim_portf & 0x10) == 0; }   // active-LOW pulse

// Arp/seq play start fires a CLOCK-RESET pulse, and the firmware swallows
// T1 edges while the pulse is active plus one re-priming pass after it (the
// LFO restart's own transitions must not read as clock edges). Let that
// window pass before driving test clock edges.
static void SettleClockReset() {
  int guard = 0;
  while (ClockReset() && ++guard < 10000) RunLoops(1);
  uint32_t until = millis() + 5;               // CLK_SETTLE_MS + margin
  while ((int32_t)(until - millis()) > 0 && ++guard < 20000) RunLoops(1);
  RunLoops(1);                                 // clears the resync latch
}

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

// Print every complete SysEx message sitting in the TX capture as one
// `<- xx xx ...` line of hex, F0/F7 stripped, then CLEAR the capture. Anything
// that is not SysEx (note echoes, clock) is skipped: the callers here are
// asking about the protocol.
//
// Draining here rather than clearing before each command is deliberate. The
// 101 emits UNSOLICITED 0x15 step-position messages while the sequencer runs,
// so a reader that cleared first would throw away exactly the playhead reports
// the time channel is built on.
static void PrintSysexOut() {
  std::string line;
  bool in = false;
  for (uint8_t b : g_midiOut) {
    if (b == 0xF0) { in = true; line = "<-"; continue; }
    if (b == 0xF7) {
      if (in && !line.empty()) puts(line.c_str());
      in = false; continue;
    }
    if (in) { char buf[8]; snprintf(buf, sizeof(buf), " %02X", b); line += buf; }
  }
  g_midiOut.clear();
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
  puts("  syx <hex...>                send a SysEx body (after F0 7D), print replies");
  puts("  runq [n]                    run n passes, print any SysEx emitted");
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
    } else if (cmd == "syx") {
      // Inject a SysEx body (hex, the bytes BETWEEN F0 7D and F7) and print
      // every SysEx the firmware sends back, one `<- ` line per message.
      //
      // This is the machine-readable channel the G12 suite adapter drives when
      // there is no hardware lease to be had (tools/diag/suite_adapter.py,
      // SimLink). It matters because the sim links the REAL firmware, so a
      // sweep run through here exercises src/main.cpp's actual OnSysex and
      // src/engine.h's actual setStep/setLength/randomize, not a stand-in.
      // What it cannot show is anything electrical or timing-related on the
      // device, so a green sim sweep is a necessary result and not a
      // sufficient one; the adapter says which transport produced a run.
      std::vector<uint8_t> body;
      std::string tok;
      bool bad = false;
      while (ss >> tok) {
        char* end = nullptr;
        long v = strtol(tok.c_str(), &end, 16);
        if (*end || v < 0 || v > 0xFF) { bad = true; break; }
        body.push_back((uint8_t)v);
      }
      if (bad || body.empty()) { puts("usage: syx <hex bytes after F0 7D>"); continue; }
      g_midiOut.clear();
      g_midiQ.push_back(0xF0); g_midiQ.push_back(0x7D);
      for (uint8_t b : body) g_midiQ.push_back(b);
      g_midiQ.push_back(0xF7);
      RunLoops(2);
      PrintSysexOut();
      puts("[ok]");
      continue;
    } else if (cmd == "runq") {
      // Run loop passes and report any SysEx emitted, without sending
      // anything. This is how the adapter collects the 101's UNSOLICITED
      // outgoing 0x15 step-position reports, which are the playhead and so
      // the time channel's anchors.
      int n = 1; ss >> n;
      RunLoops(n < 1 ? 1 : n);   // no clear: see PrintSysexOut
      PrintSysexOut();
      puts("[ok]");
      continue;
    } else if (cmd == "mode") {
      std::string val; ss >> val;
      if (val == "emu" && !emu101::available()) { puts("no ROM embedded"); continue; }
      if (val == "emu" || val == "superos") { SwitchTo(val == "emu" ? 1 : 0); RunLoops(2); } // in-process
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
// Host clock at a real tempo (120 BPM, 20.8 ms per 0xF8): the uClock smoother
// fires a tick on its PREDICTED time, so back-to-back bytes are a burst to it,
// not a tempo. Each tick is followed by one period of loop passes, so a check
// after MidiClocks(n) sees the n-th tick fired.
static const uint32_t SIM_TICK_US = 20833;
static void MidiClocks(int n) {
  while (n-- > 0) {
    uint32_t until = micros() + SIM_TICK_US;
    MidiByte(0xF8);
    while ((int32_t)(until - micros()) > 0) RunLoops(1);
  }
}

static int g_fails = 0;
#define CHECK(cond, msg) do { \
    if (cond) printf("  PASS  %s\n", msg); \
    else      { printf("  FAIL  %s\n", msg); ++g_fails; } \
  } while (0)

// 0x34 -> 0x35 build identity. Run against BOTH firmwares, because the reason
// the handler sits in sysex::HandleCommon rather than in main.cpp's
// SuperOS-only switch is precisely that the emulator build is the one where
// nobody can tell from outside what is resident. Asserting it only on the
// SuperOS side would leave the interesting half untested.
//
// What would have to break for this to go red, since a test whose mechanism
// cannot fail still prints a pass: the third check is the load-bearing one. If
// tools/buildrev.py throws, or the define is dropped from the sim Makefile (or,
// on the device, from either extra_scripts list in platformio.ini, which does
// NOT merge), the firmware falls back to "unknown" and this fails. The sha's
// value is compared against the macro rather than against a literal, so the
// test does not go red every time HEAD moves; sim.cpp and main.cpp are built by
// the same rule with the same flags, so a stale binary is stale in both.
static void CheckVersionReply(const char* fw) {
  char msg[192];
  g_midiOut.clear();
  for (uint8_t b : {(uint8_t)0xF0, (uint8_t)0x7D, (uint8_t)0x34, (uint8_t)0xF7}) g_midiQ.push_back(b);
  RunLoops(1);
  // Scan for the reply rather than assume it sits at offset 0: this wire also
  // carries note echoes and the outbound 0x15 step notifications, so the reply
  // is not always alone in the buffer.
  std::string s;
  bool found = false;
  for (size_t i = 0; i + 3 < g_midiOut.size() && !found; ++i) {
    if (g_midiOut[i] != 0xF0 || g_midiOut[i + 1] != 0x7D || g_midiOut[i + 2] != 0x35) continue;
    size_t e = i + 3;
    while (e < g_midiOut.size() && g_midiOut[e] != 0xF7) ++e;
    if (e >= g_midiOut.size()) break;              // unterminated: not a reply
    s.assign(g_midiOut.begin() + i + 3, g_midiOut.begin() + e);
    found = true;
  }
  snprintf(msg, sizeof(msg), "SysEx 0x34 -> 0x35 answered in %s", fw);
  CHECK(found, msg);
  if (!found) return;

  bool ascii = !s.empty();
  for (char c : s) if (c < 0x20 || c > 0x7E) ascii = false;
  snprintf(msg, sizeof(msg), "0x35 payload is 7-bit printable ASCII in %s (\"%s\")", fw, s.c_str());
  CHECK(ascii, msg);

  const std::string want = SUPEROS_VERSION " " SUPEROS_BUILD_REV;
  snprintf(msg, sizeof(msg), "0x35 is \"<version> <rev>\" and carries the build's real revision in %s "
           "(not the \"unknown\" fallback)", fw);
  CHECK(s == want && std::string(SUPEROS_BUILD_REV) != "unknown", msg);
}

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
  // The check above passes under BOTH lowest-note and first-note priority,
  // because the key pressed first is also the lower one. claim-096 D2 is the
  // gesture that separates them: take the HIGH key first, then add a lower.
  // Lowest-note priority must move to the lower key and give it back on
  // release; first-note priority never moves. This was red before the
  // engine.h:295 fix and is the only case in the suite that could catch it.
  sim::g_keys = 1UL << 10; RunLoops(1);
  CHECK(sim::g_sh[0] == 27, "key 10 alone -> CV 27");
  sim::g_keys |= 1UL << 0; RunLoops(1);
  CHECK(sim::g_sh[0] == 17, "lower key joining takes priority (D2)");
  sim::g_keys = 1UL << 10; RunLoops(1);
  CHECK(sim::g_sh[0] == 27, "releasing the lower key gives it back (D2)");
  sim::g_keys = 0; RunLoops(1);
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
  WaitVirtMs(35);                                     // scan-bleed guard after the tap
  sim::g_keys = (1UL << 0) | (1UL << 4) | (1UL << 7);
  RunLoops(1);          // arp start pass: fires the CLOCK-RESET pulse
  SettleClockReset();
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
  WaitVirtMs(35);                                     // scan-bleed guard after the tap
  sim::g_keys = 1UL << 0; RunLoops(1); sim::g_keys = 0; RunLoops(1);
  sim::g_keys = 1UL << 2; RunLoops(1); sim::g_keys = 0; RunLoops(1);
  sim::g_buttons = 1u << 7; RunLoops(1); sim::g_buttons = 0; RunLoops(1); // KEY-TRANSP = rest
  sim::g_buttons = 1u << 0; RunLoops(1); sim::g_buttons = 0; RunLoops(1); // LOAD (save)
  {
    uint8_t buf[101]; uint8_t n = persist::Store().read(persist::FB_PATTERN_BASE + 0, buf, sizeof(buf));
    CHECK(n == 5 && buf[0] == 3, "pattern 0 saved to the flash arena (len 3 + ratchet byte)");
  }
  sim::g_buttons = 1u << 1; RunLoops(1); sim::g_buttons = 0; RunLoops(1); // PLAY
  CHECK(sim::g_sh[0] == 17 && GateOn(), "PLAY -> step 1 sounds immediately (CV 17)");
  // The LFO-restart glitch right after the reset pulse must NOT advance the
  // sequencer off step 1 (it read as an edge once, and step 1 sounded
  // skipped on hardware): blip the clock inside the settle window.
  sim::g_clock = true; RunLoops(1); sim::g_clock = false; RunLoops(1);
  CHECK(sim::g_sh[0] == 17, "reset-release glitch does not skip step 1");
  SettleClockReset();
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

  // CV DAC (cvdac.h): cutoff CC + accent models, observed on the MCP4822
  // codes. Each sim pass counts as 1 ms in cvdac::Tick (pass << 1 ms).
  puts("CV DAC:");
  {
    // Unipolar variant (first fabbed boards before rework: R8 100k, no bias): center 0, additive only.
    cvdac::SetHw(cvdac::HW_V11_UNIPOLAR); RunLoops(2500);
    CHECK(g_simCvCode[0] == 0 && g_simCvCode[1] == cvdac::VCA_PARK, "unipolar board: VOUTA rests at 0 V, VOUTB parked at 650");
    for (uint8_t b : {(uint8_t)0xB0, (uint8_t)74, (uint8_t)127}) g_midiQ.push_back(b); RunLoops(200);
    CHECK(g_simCvCode[0] == cvdac::DAC_MAX, "unipolar board: CC74=127 -> DAC full scale (0.55 throw, clamped)");
    for (uint8_t b : {(uint8_t)0xB0, (uint8_t)74, (uint8_t)0}) g_midiQ.push_back(b); RunLoops(200);
    CHECK(g_simCvCode[0] == 0, "unipolar board: CC74=0 -> 0 V");
    // The rest of the block exercises the bipolar variant (R8 27k + R10 68k), the default.
    cvdac::SetHw(cvdac::HW_V11_BIPOLAR); RunLoops(15);
    const uint16_t C = cvdac::CENTER_DEFAULT_BY_HW[1], F = cvdac::FADER_LSB_BY_HW[1], P = cvdac::VCA_PARK;
    { cvdac::Settings s3 = cvdac::Get(); s3.vcaAmount = 127; cvdac::Set(s3);
      CHECK(cvdac::Get().vcaAmount == cvdac::VCA_AMOUNT_MAX, "VCA amount is capped at 100 (BA662 control-current limit)");
      s3.vcaAmount = 64; cvdac::Set(s3); }
    auto cc = [](uint8_t v) { for (uint8_t b : {(uint8_t)0xB0, (uint8_t)74, v}) g_midiQ.push_back(b); RunLoops(1); };
    auto noteOn  = [](uint8_t vel) { for (uint8_t b : {(uint8_t)0x90, (uint8_t)53, vel}) g_midiQ.push_back(b); RunLoops(1); };
    auto noteOff = []() { for (uint8_t b : {(uint8_t)0x80, (uint8_t)53, (uint8_t)0}) g_midiQ.push_back(b); RunLoops(1); };
    // Track both channels' maxima over N passes (the 7 ms one-pole attack
    // needs ~4 tau = 28 passes to reach its plateau).
    auto peaks = [](int passes, uint16_t& ma, uint16_t& mb) {
      ma = mb = 0;
      while (passes-- > 0) { RunLoops(1); if (g_simCvCode[0] > ma) ma = g_simCvCode[0]; if (g_simCvCode[1] > mb) mb = g_simCvCode[1]; }
    };
    // The MIDI block above played notes at velocity 100 = the accent threshold,
    // so let that accent bleed off fully first (env tail ~8 tau).
    RunLoops(2500);
    CHECK(g_simCvCode[0] == C && g_simCvCode[1] == P, "rest: VOUTA at center, VOUTB parked at 650");
    cc(127); RunLoops(150);
    CHECK(g_simCvCode[0] == cvdac::DAC_MAX, "CC74=127 -> center + one fader throw (2105+2025 clamps at 4095 = +98 %)");
    cc(64);  RunLoops(150);
    CHECK(g_simCvCode[0] == C, "CC74=64 -> back to center");
    cc(0);   RunLoops(150);
    CHECK(g_simCvCode[0] == C - F, "CC74=0 -> center - one throw (80 LSB above 0 = -100 %)");
    // smoothness: one 7-bit CC step (32 LSB) must not jump in a single pass
    cc(64);  RunLoops(150); uint16_t before = g_simCvCode[0];
    cc(65);  uint16_t after1 = g_simCvCode[0]; RunLoops(150); uint16_t settled = g_simCvCode[0];
    CHECK(after1 - before < 8 && settled - before > 28 && settled - before < 36, "one CC step glides (no 32-LSB jump) and settles at +1/64 throw");
    // 14-bit: CC 106 refines CC 74
    for (uint8_t b : {(uint8_t)0xB0, (uint8_t)106, (uint8_t)64}) g_midiQ.push_back(b); RunLoops(150);
    CHECK(g_simCvCode[0] - settled > 12 && g_simCvCode[0] - settled < 20, "CC106 (LSB) adds half a CC step (14-bit cutoff)");
    cc(64);  RunLoops(150);
    // MC-202 accent: velocity 127 clears the threshold; both channels bump, then decay.
    noteOn(127);
    uint16_t pa, pb; peaks(40, pa, pb);
    CHECK(pb > P + 1650 && pb <= P + 1740, "MC-202 accent: VCA +1 envelope at amount 64");
    CHECK(pa > C + 550 && pa < C + 700,    "MC-202 accent: VCF peak-only bump (~0.32 throw at amount 100)");
    RunLoops(100);
    uint16_t bMid = g_simCvCode[1];
    CHECK(bMid < pb && bMid > P, "MC-202 accent decays (300 ms)");
    noteOff(); RunLoops(2500);
    CHECK(g_simCvCode[0] == C && g_simCvCode[1] == P, "after the accent both channels return to rest");
    noteOn(50); RunLoops(10);
    CHECK(g_simCvCode[1] == P && g_simCvCode[0] == C, "velocity 50 < threshold 100: no accent");
    noteOff(); RunLoops(5);
    // TB-303 model: fixed 200 ms decay, build-up across consecutive accents.
    cvdac::SetMode(cvdac::MODE_TB303);
    uint16_t p1, p2, q;
    noteOn(127); peaks(40, p1, q); noteOff(); RunLoops(10);
    noteOn(127); peaks(40, p2, q); noteOff();
    CHECK(p1 > C + 300, "TB-303 accent: VCF sweep present");
    // default buildup 64 = +0.125 of the full sweep per hit ~= 70 LSB at these amounts
    CHECK(p2 > p1 + 50, "TB-303 accent: second accent 50 ms later sweeps further (C13 build-up)");
    RunLoops(2500);
    CHECK(g_simCvCode[0] == C && g_simCvCode[1] == P, "TB-303 accent fully bled off");
    cvdac::SetMode(cvdac::MODE_MC202);
  }
  // SysEx: config request, pattern round trip via 0x12/0x10.
  g_midiOut.clear();
  for (uint8_t b : {0xF0, 0x7D, 0x20, 0xF7}) g_midiQ.push_back(b); RunLoops(1);
  CHECK(g_midiOut.size() > 8 && g_midiOut[0] == 0xF0 && g_midiOut[1] == 0x7D && g_midiOut[2] == 0x21 && g_midiOut[3] == 0,
        "SysEx 0x20 -> 0x21 config reply (fw 0)");
  CheckVersionReply("SuperOS");
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
    // The reply appends the packed ratchet tail (one byte for len 3, all off);
    // the ratchet-less 0x12 above is the pre-ratchet-host compatibility path.
    CHECK(g_midiOut[2] == 0x11 && g_midiOut[3] == 5 && m == 5 && memcmp(body, raw, 4) == 0 && body[4] == 0,
          "SysEx 0x10 -> 0x11 returns the same pattern (+ ratchet tail)");
    for (int i = 0; i < 2000; ++i) { delayMicroseconds(1000); RunLoops(1); }   // > 500 ms quiet: deferred flush
    uint8_t buf[101]; uint8_t k = persist::Store().read(persist::FB_PATTERN_BASE + 5, buf, sizeof(buf));
    CHECK(k == 5 && buf[0] == 3 && buf[3] == (0x80 | 22) && buf[4] == 0, "pattern 5 flushed to the flash arena");
  }

  // 0x3B -> 0x3C bench probe, SuperOS LED/gate words (gate G12 observability,
  // and the harness G10's tempo blink will be measured with). The probe had no
  // test at all before this; these fail against a probe without raw[97..113].
  {
    puts("probe (0x3B -> 0x3C):");
    auto probe = [&](uint8_t* raw, uint8_t want) -> int {   // returns raw byte count
      g_midiOut.clear();
      for (uint8_t b : {(uint8_t)0xF0, (uint8_t)0x7D, (uint8_t)0x3B, (uint8_t)0xF7}) g_midiQ.push_back(b);
      RunLoops(1);
      if (g_midiOut.size() < 4 || g_midiOut[2] != 0x3C) return -1;
      size_t n = (g_midiOut.size() - 4) / 2;                // F0 7D 3C .. F7, two nibbles per byte
      for (size_t i = 0; i < n && i < want; ++i) raw[i] = (uint8_t)((g_midiOut[3 + 2 * i] << 4) | g_midiOut[4 + 2 * i]);
      return (int)n;
    };
    uint8_t raw[128] = {0};
    int n = probe(raw, sizeof(raw));
    CHECK(n >= 125, "0x3C carries the SuperOS LED/gate words, the rail set and store health");
    // Store health (A606's claim-036, fleet warning): on the 606 every write
    // acked out of a RAM cache while nothing reached flash, and only a reboot
    // exposed it. By this point the self-test has already flushed patterns to
    // the arena and read them back, so a store reporting anything less than
    // "mounted and writable" here would mean those passes were cache echoes.
    // Honest limit: on the host ServicePresent() is a constant true, so bit0 is
    // only a wiring check here and the real one is the hardware probe. bit2 is
    // load-bearing in both: it comes from an actual mount of the arena.
    CHECK((raw[124] & 0x01) != 0, "probe store health: SPM service answered");
    CHECK((raw[124] & 0x04) != 0, "probe store health: arena mounted and writable");
    // The rail set (claim-038). Nothing has run a bench window, so the set is
    // empty and raw[121] must say so: a host that reads the sum without
    // checking the count would read 0 as "0 mV" instead of "never measured".
    CHECK(raw[121] == 0, "probe rail set: sample count 0 before any 0x3E window");
    // raw[97] is the live P2 image. The gate bit is INVERTED (0 = gate on), so
    // a resting pattern-0 machine with no note must read it high.
    CHECK((raw[97] & (1u << P2_GATE)) != 0, "probe P2 image: gate bit high = gate off at rest");
    // Drive a note and confirm the image follows the gate through the probe,
    // i.e. the probe reads the live port image and not a frozen copy.
    for (uint8_t b : {(uint8_t)0x90, (uint8_t)53, (uint8_t)100}) g_midiQ.push_back(b); RunLoops(2);
    probe(raw, sizeof(raw));
    CHECK((raw[97] & (1u << P2_GATE)) == 0, "probe P2 image: gate bit low = gate on while a note sounds");
    for (uint8_t b : {(uint8_t)0x80, (uint8_t)53, (uint8_t)0}) g_midiQ.push_back(b); RunLoops(2);
    probe(raw, sizeof(raw));
    CHECK((raw[97] & (1u << P2_GATE)) != 0, "probe P2 image: gate released after note off");
    // Toggle counters: the gate just changed twice, so bit 7 must have counted
    // transitions once the one-second window rolls. Advance past the window.
    for (int i = 0; i < 1100; ++i) { delayMicroseconds(1000); RunLoops(1); }
    probe(raw, sizeof(raw));
    uint16_t gateTog = (uint16_t)((raw[98 + 2 * P2_GATE] << 8) | raw[99 + 2 * P2_GATE]);
    CHECK(gateTog == 0, "probe toggles/s: a quiet second reports zero gate transitions");
    for (uint8_t b : {(uint8_t)0x90, (uint8_t)53, (uint8_t)100}) g_midiQ.push_back(b); RunLoops(2);
    for (uint8_t b : {(uint8_t)0x80, (uint8_t)53, (uint8_t)0}) g_midiQ.push_back(b); RunLoops(2);
    for (int i = 0; i < 1100; ++i) { delayMicroseconds(1000); RunLoops(1); }
    probe(raw, sizeof(raw));
    gateTog = (uint16_t)((raw[98 + 2 * P2_GATE] << 8) | raw[99 + 2 * P2_GATE]);
    CHECK(gateTog >= 2, "probe toggles/s: a note on+off inside the window counts 2 gate transitions");

    // The function-LED select (P20-22, one-hot via IC8 4556) reported through
    // the same image. Both docs/FIRMWARE.md and the manual listed this as "still
    // stubbed / not yet implemented" long after engine.h ledSelect() was wired to
    // main.cpp's P2 assembly; this is the check that settles it rather than a
    // second reading of the source. 1 LOAD, 2 PLAY, 3 DOWN, 4 U&D, 5 UP, 0 none.
    auto ledSel = [&]() -> uint8_t {
      probe(raw, sizeof(raw));
      return (uint8_t)(raw[97] & 0x07);
    };
    // Button bits are controls.h BTN_*: 0 LOAD, 1 PLAY, 2 DOWN, 3 U&D, 4 UP.
    auto tap = [&](uint8_t btn) { sim::g_buttons = (uint32_t)(1u << btn); RunLoops(2); sim::g_buttons = 0; RunLoops(2); };
    tap(4);
    CHECK(ledSel() == 5, "probe P2 image: arp UP lights function-LED select 5");
    tap(2);
    CHECK(ledSel() == 3, "probe P2 image: arp DOWN lights select 3");
    tap(3);
    CHECK(ledSel() == 4, "probe P2 image: arp UP&DOWN lights select 4");
    tap(3);   // second press leaves the arp -> keyboard, all off
    CHECK(ledSel() == 0, "probe P2 image: back to keyboard lights no function LED");
  }

  // 0x3E bench window clock select (gate G13, claim-038). This is the rail
  // instrument, so the thing it must never get wrong is which clock it ran at.
  // It got it wrong twice: the window "restored" CLKPR to /1 rather than to the
  // app's own /4, so a /2 window left the machine at 16 MHz with the UART
  // divisor still computed for 4 MHz, and since nothing could REQUEST 16 MHz,
  // the 16 MHz rows of the 2026-08-25 sweep were that leak instead of the mode
  // asked for. Two of those six rows were labelled with a clock the machine was
  // not running. Every check below fails against the previous logic.
  {
    puts("bench window clock (0x3E):");
    const uint8_t app = 2;                        // the app's divide, /4
    CHECK(sysex::BenchClkps(0, app) == app, "0x3E mode 0 leaves the app's own divide (the 4 MHz control)");
    CHECK(sysex::BenchClkps(1, app) == 1, "0x3E bit0 selects /2 = 8 MHz");
    CHECK(sysex::BenchClkps(8, app) == 0, "0x3E bit3 selects /1 = 16 MHz");
    CHECK(sysex::BenchClkps(9, app) == 0, "0x3E bit3 wins over bit0");
    CHECK(sysex::BenchClkps(6, app) == app, "0x3E USB-off + sleep alone do not move the clock");
    CHECK(sysex::BenchCoreHz(4000000UL, app, app) == 4000000UL, "0x3E core Hz at the app divide = 4 MHz");
    CHECK(sysex::BenchCoreHz(4000000UL, app, 1) == 8000000UL, "0x3E core Hz at /2 = 8 MHz");
    CHECK(sysex::BenchCoreHz(4000000UL, app, 0) == 16000000UL, "0x3E core Hz at /1 = 16 MHz");
    // The UART divisor has to follow the real core clock or DIN MIDI comes out
    // at the wrong baud for the rest of the session, silently.
    CHECK(sysex::BenchUbrr(4000000UL) == 7, "0x3E UBRR1L 4 MHz -> 7");
    CHECK(sysex::BenchUbrr(8000000UL) == 15, "0x3E UBRR1L 8 MHz -> 15");
    CHECK(sysex::BenchUbrr(16000000UL) == 31, "0x3E UBRR1L 16 MHz -> 31");
    // The ADC needs 50-200 kHz for ten honest bits. Choosing the divider from
    // the compile-time F_CPU ran it at 250 kHz during a 16 MHz window, i.e. out
    // of spec in the one condition claim-038 cares about.
    for (uint32_t hz : {4000000UL, 8000000UL, 16000000UL}) {
      uint32_t adcHz = hz >> sysex::AdcPrescaleBits(hz);
      CHECK(adcHz >= 50000UL && adcHz <= 200000UL, "ADC clock stays in 50-200 kHz at every bench clock");
    }
  }

  // Live-edit opcodes (gate G4): 0x16 set step, 0x18 set length, 0x15 randomize.
  {
    auto edit = [&](std::vector<uint8_t> body) -> int {   // F0 7D <body> F7 -> ack status or -1
      g_midiOut.clear();
      g_midiQ.push_back(0xF0); g_midiQ.push_back(0x7D);
      for (uint8_t b : body) g_midiQ.push_back(b);
      g_midiQ.push_back(0xF7); RunLoops(1);
      return (g_midiOut.size() >= 5 && g_midiOut[2] == 0x14) ? (int)g_midiOut[3] : -1;
    };
    auto readpat = [&](uint8_t p, uint8_t *out) -> int {  // returns len, fills out[] with step bytes
      g_midiOut.clear();
      for (uint8_t b : {(uint8_t)0xF0,(uint8_t)0x7D,(uint8_t)0x10,p,(uint8_t)0xF7}) g_midiQ.push_back(b); RunLoops(1);
      uint8_t raw[130] = {0}; uint8_t m = midi::Unpack7(&g_midiOut[6], (uint8_t)(g_midiOut.size()-7), raw, sizeof(raw));
      if (m < 1) return -1; for (uint8_t i = 0; i < raw[0]; i++) out[i] = raw[i+1]; return raw[0];
    };
    uint8_t st[130];
    CHECK(edit({0x18, 5, 4}) == 0 && readpat(5, st) == 4, "G4 0x18 set length 5->4");
    CHECK(edit({0x16, 5, 0, 0, 56}) == 0, "G4 0x16 set step note ack 0");
    readpat(5, st);
    CHECK(st[0] == (56 - 36) && (st[0] >> 6) == 0, "G4 0x16 step0 = note cv20");
    CHECK(edit({0x16, 5, 3, 0, 1}) == 0, "G4 0x16 lane3 gate ack 0");
    readpat(5, st);
    CHECK((st[0] >> 6) == 2, "G4 0x16 lane3 -> legato, cv kept");
    CHECK(edit({0x16, 9, 0, 0, 56}) == 2, "G4 0x16 bad pat -> ack 2");
    CHECK(edit({0x16, 5, 1, 0, 56}) == 2, "G4 0x16 lane1 (accent absent) -> ack 2");
    CHECK(edit({0x18, 5, 0}) == 2, "G4 0x18 length 0 -> ack 2");
    uint8_t a[130], b[130];
    edit({0x2E, 5, 0x7F, 64, 57, 96}); int la = readpat(5, a);
    edit({0x2E, 5, 0x7F, 64, 57, 96}); int lb = readpat(5, b);
    CHECK(la == lb && la > 0 && memcmp(a, b, la) == 0, "G4 0x2E randomize deterministic (same seed)");
    // 0x38 was this opcode for exactly one commit and is CMD_FRAM_PROBE on
    // the 606. It must be as dead here as inbound 0x15 is.
    CHECK(edit({0x38, 5, 0x7F, 64, 57, 96}) == -1, "withdrawn 0x38 is not acked");
    // Lanes: 0x7F all, 0 pitches only, 3 legato only, 1/2 absent (mono).
    CHECK(edit({0x2E, 5, 1, 64, 57, 96}) == 2, "0x2E lane1 (accent absent) -> ack 2");
    CHECK(edit({0x2E, 5, 2, 64, 57, 96}) == 2, "0x2E lane2 (slide absent) -> ack 2");
    CHECK(edit({0x2E, 5, 9, 64, 57, 96}) == 2, "0x2E unknown lane -> ack 2");
    // LANE_NOTE keeps the note/rest layout; LANE_GATE keeps pitches too.
    edit({0x18, 5, 8}); for (uint8_t i = 0; i < 8; i++) edit({0x16, 5, 0, i, (uint8_t)(48 + i)});
    uint8_t g0[130]; int lg = readpat(5, g0);
    edit({0x2E, 5, 0, 100, 11, 0});
    uint8_t g1[130]; readpat(5, g1);
    bool layout_kept = true, some_pitch_moved = false;
    for (int i = 0; i < lg; i++) {
      if ((g0[i] >> 6) != (g1[i] >> 6)) layout_kept = false;
      if ((g0[i] & 0x3F) != (g1[i] & 0x3F)) some_pitch_moved = true;
    }
    CHECK(layout_kept && some_pitch_moved, "0x2E lane0 moves pitches, keeps the layout");
    uint8_t h0[130]; int lh = readpat(5, h0);
    edit({0x2E, 5, 3, 127, 11, 0});
    uint8_t h1[130]; readpat(5, h1);
    bool pitch_kept = true, all_legato = true;
    for (int i = 0; i < lh; i++) {
      if ((h0[i] & 0x3F) != (h1[i] & 0x3F)) pitch_kept = false;
      if ((h1[i] >> 6) != 2) all_legato = false;
    }
    CHECK(pitch_kept && all_legato, "0x2E lane3 density 127 -> all legato, pitches kept");

    // THE ECHO HAZARD, asserted rather than assumed (claim-049). RANDOMIZE
    // moved off 0x15 because this machine EMITS 0x15 every step as its
    // playhead, so a thru/echo path feeds the device its own notification. The
    // test is not "0x38 works" but "the exact bytes we emit are INERT coming
    // back": a real 3-byte SendStep frame, and a 6-byte one long enough to
    // have been accepted by the old handler. Neither may touch the pattern.
    uint8_t before[130]; int lbefore = readpat(5, before);
    CHECK(edit({0x15, 5, 0}) == -1, "0x15 echo (playhead frame) is not acked");
    uint8_t after[130]; int lafter = readpat(5, after);
    CHECK(lbefore == lafter && memcmp(before, after, lbefore) == 0,
          "0x15 echo leaves the pattern untouched");
    CHECK(edit({0x15, 5, 0, 64, 57, 96}) == -1,
          "0x15 with a full randomize payload is ignored, not acked");
    int lafter2 = readpat(5, after);
    CHECK(lbefore == lafter2 && memcmp(before, after, lbefore) == 0,
          "0x15 randomize-shaped payload does not randomize");
  }

  // G12 combinatorial matrix (claim-031's lesson: single features pass, their
  // COMBINATIONS corrupt). The 101 is mono with no chain/accent, so the axes that
  // interact are: length boundary x legato/tie present x edit type, exercised in
  // combination. Invariant after every edit: the pattern reads back clean --
  // length in 1..128, every step byte a valid cv(<=63)+type(<=2). State channel.
  {
    auto sx = [&](std::vector<uint8_t> body) {
      g_midiOut.clear(); g_midiQ.push_back(0xF0); g_midiQ.push_back(0x7D);
      for (uint8_t b : body) g_midiQ.push_back(b);
      g_midiQ.push_back(0xF7); RunLoops(1);
      return (g_midiOut.size() >= 5 && g_midiOut[2] == 0x14) ? (int)g_midiOut[3] : -1;
    };
    auto valid = [&](uint8_t p) -> bool {                  // read back and check no corruption
      g_midiOut.clear();
      for (uint8_t b : {(uint8_t)0xF0,(uint8_t)0x7D,(uint8_t)0x10,p,(uint8_t)0xF7}) g_midiQ.push_back(b); RunLoops(1);
      if (g_midiOut.size() < 8 || g_midiOut[2] != 0x11) return false;
      uint8_t raw[130] = {0}; uint8_t m = midi::Unpack7(&g_midiOut[6], (uint8_t)(g_midiOut.size()-7), raw, sizeof(raw));
      if (m < 1) return false;
      uint8_t len = raw[0];
      if (len < 1 || len > 128 || m < (uint8_t)(len + 1)) return false;
      for (uint8_t i = 0; i < len; i++) { if ((raw[1+i] & 0x3F) > 63) return false; if ((raw[1+i] >> 6) > 2) return false; }
      return true;
    };
    const uint8_t P = 6;                                   // scratch slot
    const uint8_t lens[] = {1, 2, 16, 128};
    int combos = 0, bad = 0;
    for (uint8_t li = 0; li < 4; li++) {
      for (uint8_t legato = 0; legato < 2; legato++) {
        // build: length L, alternate note/rest, optionally set some legato
        sx({0x18, P, lens[li]});
        uint8_t L = lens[li];
        for (uint8_t i = 0; i < L; i++) sx({0x16, P, 0, i, (uint8_t)((i & 1) ? 0 : (uint8_t)(40 + (i & 7)))});
        if (legato) for (uint8_t i = 0; i < L; i += 4) if ((i & 1) == 0) sx({0x16, P, 3, i, 1});
        if (!valid(P)) bad++; combos++;
        // now COMBINE edits at length/wrap boundaries while "playing" (advance the head)
        sx({0x1D, P}); MidiByte(0xFA);                     // select + start: head at wrap
        MidiClocks(30);                                    // advance across the pattern (wrap)
        sx({0x16, P, 0, (uint8_t)(L - 1), 55});            // edit the last step (wrap boundary)
        sx({0x18, P, (uint8_t)(li ? lens[li-1] : 1)});     // shrink length under the head
        sx({0x2E, P, 0, 100, 21, 33});                     // randomize lane 0 (pitches), deterministic
        sx({0x16, P, 3, 0, (uint8_t)(legato ? 0 : 1)});    // toggle a tie
        MidiClocks(12); MidiByte(0xFC);                    // more clock, then stop
        if (!valid(P)) bad++; combos++;
      }
    }
    CHECK(bad == 0, "G12 matrix: length x legato x combined edits never corrupt the pattern");
    if (bad) printf("   %d/%d combos corrupted\n", bad, combos);
    sx({0x1D, 0}); MidiByte(0xFC); RunLoops(2);   // restore: pattern 0, transport stopped
  }

  sim::g_buttons = 1u << 1; RunLoops(1); sim::g_buttons = 0; RunLoops(1); // PLAY back on (pattern 0)

  // Firmware invariant: a scan must never corrupt a held S/H (route 11 only).
  puts("invariants:");
  uint8_t cvBefore = sim::g_sh[0], rngBefore = sim::g_sh[1];
  sim::g_keys = 1UL << 5; RunLoops(3); sim::g_keys = 0;
  CHECK(sim::g_sh[1] == rngBefore, "scans left the VCO-range S/H untouched");
  (void)cvBefore;

  // USB-MIDI event codec (src/usbmidi_codec.h): every byte stream the
  // firmware emits must survive encode -> CIN-length decode unchanged, and
  // realtime bytes must pass straight through, even inside a SysEx.
  puts("USB-MIDI codec:");
  {
    std::vector<uint8_t> in, out;
    auto sysex = [&](uint8_t bodyLen) {           // F0 7D <bodyLen bytes> F7
      in.push_back(0xF0); in.push_back(0x7D);
      for (uint8_t i = 0; i < bodyLen; ++i) in.push_back((uint8_t)(0x10 + i));
      in.push_back(0xF7);
    };
    in.insert(in.end(), {0x90, 60, 100, 0x80, 60, 0, 0xC0, 5, 0xB0, 7, 127});
    for (uint8_t n = 0; n <= 9; ++n) sysex(n);    // 0..9 body bytes: every CIN 5/6/7 tail
    in.insert(in.end(), {0x90, 61, 90});
    usbmidi::Encoder enc; uint8_t ev[4];
    size_t nEv = 0;
    for (uint8_t b : in) if (usbmidi::Encode(enc, b, ev)) {
      ++nEv;
      for (uint8_t i = 0; i < usbmidi::CinLen(ev[0]); ++i) out.push_back(ev[1 + i]);
    }
    CHECK(out == in, "encode -> decode reproduces the byte stream (notes, PC, CC, SysEx 0..9)");
    CHECK(enc.n == 0 && !enc.sx, "encoder idle after a complete stream");
    // realtime inside a SysEx: emitted at once as its own event
    usbmidi::Encoder e2; std::vector<uint8_t> o2; size_t rtAt = 99;
    const uint8_t mixed[] = {0xF0, 0x7D, 0x11, 0xF8, 0x12, 0x13, 0x14, 0xF7};
    for (uint8_t b : mixed) if (usbmidi::Encode(e2, b, ev)) {
      if (ev[0] == 0x0F && ev[1] == 0xF8) rtAt = o2.size();
      else for (uint8_t i = 0; i < usbmidi::CinLen(ev[0]); ++i) o2.push_back(ev[1 + i]);
    }
    const std::vector<uint8_t> want = {0xF0, 0x7D, 0x11, 0x12, 0x13, 0x14, 0xF7};
    CHECK(o2 == want && rtAt == 3, "realtime byte inside a SysEx passes through at once, SysEx intact");
    (void)nEv;
  }

  // MIDI clock sync. The engine is still in sequencer-play mode with the
  // 3-step sequence (note 0, note 2, rest) from the tests above.
  puts("MIDI clock sync:");
  midiclk::SetSync(true); midiclk::SetDiv(6);   // 1/16 = 6 ticks at 24 ppqn
  midiclk::SetLfoSync(true); midiclk::SetLfoDiv(6);   // LFO hard-sync on the same boundary
  // Sync ON is STRICT: with no host clock on the wire the machine freezes;
  // the LFO/EXT-CLK never stands in.
  {
    uint32_t until = millis() + 320;               // ticks from earlier tests age out
    while ((int32_t)(until - millis()) > 0) RunLoops(1);
  }
  uint8_t cvBeforeSync = sim::g_sh[0];
  ClockTick();
  CHECK(sim::g_sh[0] == cvBeforeSync, "sync on, no host clock: T1 (LFO) steps NOTHING (strict slave)");
  MidiClocks(2);                                 // ticks arrive: the host's clock drives
  uint8_t cvUnderClock = sim::g_sh[0];
  ClockTick();
  CHECK(sim::g_sh[0] == cvUnderClock, "sync on, host clock alive: T1 (LFO) clock is ignored");
  MidiByte(0xFA);                                // Start: rewind to the downbeat
  MidiClocks(1);                                 // first clock = downbeat edge
  CHECK(sim::g_sh[0] == 17 && GateOn(), "Start + 1st clock -> seq step 0 (CV 17)");
  MidiClocks(5);
  {   // 6th clock: the division boundary. The 2 ms CLOCK-RESET pulse is
      // over long before the tick period is, so watch for it as it fires.
    bool sawRst = false;
    uint32_t until = micros() + SIM_TICK_US;
    MidiByte(0xF8);
    while ((int32_t)(until - micros()) > 0) { RunLoops(1); sawRst |= ClockReset(); }
    CHECK(sim::g_sh[0] == 19, "6 clocks at 1/16 -> next step (CV 19)");
    CHECK(sawRst, "division edge pulses CLOCK RESET (LFO hard sync)");
  }
  uint8_t rndHeld = sim::g_sh[2];
  MidiClocks(5);                                 // inside the division: no edge
  CHECK(sim::g_sh[2] == rndHeld, "random S/H holds between division edges");
  MidiByte(0xFC);                                // Stop
  MidiClocks(12);
  CHECK(sim::g_sh[0] == 19, "after Stop, clocks are ignored");
  midiclk::SetSync(false);
  ClockTick();
  CHECK(!GateOn(), "sync off: T1 clock works again (rest step -> gate off)");

  // Panel scan bleed: a function-button press can read as a phantom key on
  // the button's own P1 row (hardware finding: pressing LOAD recorded key 8
  // as step 0). The engine must neither sound nor record such a key.
  puts("panel scan bleed guard:");
  {
    midiclk::SetSync(false); RunLoops(2);
    for (uint8_t v : {0xF0, 0x7D, 0x1D, 5, 0xF7}) g_midiQ.push_back(v); RunLoops(2);   // work on pattern 5
    sim::g_buttons = 1u << 0; sim::g_keys = 1UL << 8; RunLoops(3);   // LOAD + bleed key 8
    sim::g_buttons = 0; sim::g_keys = 0; RunLoops(3);
    CHECK(!GateOn(), "LOAD press with scan-bleed key: nothing sounds");
    { uint32_t until = millis() + 140; while ((int32_t)(until - millis()) > 0) RunLoops(1); }
    sim::g_keys = 1UL << 2; RunLoops(20); sim::g_keys = 0; RunLoops(5);   // record two real notes
    sim::g_keys = 1UL << 5; RunLoops(20); sim::g_keys = 0; RunLoops(5);
    sim::g_buttons = 1u << 0; RunLoops(2); sim::g_buttons = 0; RunLoops(3);  // LOAD: exit
    sim::g_buttons = 1u << 1; RunLoops(2); sim::g_buttons = 0; RunLoops(3);  // PLAY (sync off: step 1 now)
    CHECK(GateOn() && sim::g_sh[0] == 19, "pattern starts at the FIRST REAL note (no phantom step 0)");
    SettleClockReset();
    ClockTick();
    CHECK(sim::g_sh[0] == 22, "second recorded note follows");
    ClockTick();
    CHECK(sim::g_sh[0] == 19, "and the pattern is exactly two steps (wraps)");
    sim::g_buttons = 1u << 1; RunLoops(2); sim::g_buttons = 0; RunLoops(3);  // stop
    for (uint8_t v : {0xF0, 0x7D, 0x1D, 0, 0xF7}) g_midiQ.push_back(v); RunLoops(2);   // pattern 0 back
  }

  // PLAY vs the host clock's actual state (midiclock.h Driving / UserPlay):
  // the four ways a player meets "sync on" and none of them is silent.
  puts("play vs clock state:");
  {
    auto sxPlay = [&](uint8_t on) {
      for (uint8_t v : {(uint8_t)0xF0, (uint8_t)0x7D, (uint8_t)0x1E, on, (uint8_t)0xF7}) g_midiQ.push_back(v);
      RunLoops(2);
    };
    auto waitMs = [&](uint16_t ms) {
      uint32_t until = millis() + ms;
      while ((int32_t)(until - millis()) > 0) RunLoops(1);
    };
    midiclk::SetSync(true); midiclk::SetDiv(6);
    sxPlay(0); RunLoops(2);
    waitMs(320);                            // ticks from the tests above age out
    // (1) sync on, host silent: PLAY only ARMS (strict slave, no LFO
    // fallback); the machine starts the moment the host's clock speaks.
    sxPlay(1);
    CHECK(!GateOn(), "sync on, silent host: PLAY arms silently (strict slave)");
    waitMs(8); ClockTick();
    CHECK(!GateOn(), "sync on, silent host: the LFO steps nothing");
    MidiClocks(1);                         // the host's clock speaks: step 0
    CHECK(GateOn() && sim::g_sh[0] == 17, "armed PLAY starts on the first host tick");
    sxPlay(0);
    // (2) host ticks without ever sending Start (phase unknown): PLAY
    // re-anchors the divider, so step 0 lands on the very next tick.
    MidiClocks(4);                          // stream alive, mid-division, no 0xFA
    sxPlay(1);
    CHECK(!GateOn(), "unanchored clock: PLAY arms silently");
    MidiClocks(1);
    CHECK(GateOn() && sim::g_sh[0] == 17, "unanchored clock: step 0 lands on the next tick");
    sxPlay(0);
    // (3) an 0xFA-anchored stream keeps its phase: a LOCAL play press while
    // the host runs waits for the host's own next division boundary instead
    // of re-anchoring (quantized launch). 0xFA itself now starts play (sync
    // on = transport slave), so stop locally first to test the re-arm.
    MidiByte(0xFA); MidiClocks(1);          // downbeat: playing, step 0
    CHECK(GateOn() && sim::g_sh[0] == 17, "0xFA started play at the downbeat");
    sxPlay(0);                              // local stop; the host keeps ticking
    MidiClocks(2);                          // mid-division
    sxPlay(1);
    MidiClocks(1);                         // tick 4 of 6: still inside the division
    CHECK(!GateOn(), "anchored clock: PLAY holds for the host's boundary");
    MidiClocks(3);                          // ticks 5,6,7: the edge fires on tick 7 (tick counter back at 0)
    CHECK(GateOn() && sim::g_sh[0] == 17, "anchored clock: step 0 lands on the boundary");
    sxPlay(0);
    // (4) host sent Stop, then the player presses PLAY: still strict, the
    // machine arms and waits for the host's clock.
    MidiByte(0xFC);
    waitMs(320);                            // the dead stream ages out
    sxPlay(1);
    CHECK(!GateOn(), "after host Stop: PLAY arms, nothing free-runs");
    waitMs(8); ClockTick();
    CHECK(!GateOn(), "after host Stop: the LFO still steps nothing");
    MidiClocks(1);
    CHECK(GateOn() && sim::g_sh[0] == 17, "after host Stop: the armed PLAY starts when ticks resume");
    sxPlay(0);
    // (5b) sync ON is full transport slaving: the host's 0xFA starts the
    // sequencer from silence, and 0xFC stops it (back to keyboard mode).
    waitMs(320);
    MidiByte(0xFA); MidiClocks(1);
    CHECK(GateOn() && sim::g_sh[0] == 17, "sync on: 0xFA starts the sequencer itself (step 0)");
    MidiClocks(6);
    CHECK(sim::g_sh[0] == 19, "sync on: it keeps stepping on the host's divisions");
    MidiByte(0xFC); RunLoops(2);
    waitMs(20); ClockTick();
    CHECK(sim::g_sh[0] == 19 && !GateOn(), "sync on: 0xFC stops it (keyboard mode, T1 steps nothing)");
    // sync OFF keeps the old semantics: 0xFA never force-starts
    midiclk::SetSync(false); RunLoops(2);
    MidiByte(0xFA); MidiClocks(1); RunLoops(2);
    CHECK(!GateOn(), "sync off: 0xFA does not start the sequencer");
    midiclk::SetSync(true); RunLoops(2);
    // (5) after a Stop, a fresh Start-less stream is an implicit continue:
    // the machine clocks from it again (0xFC must not permanently silence a
    // host that never sends Start), and PLAY locks to it.
    MidiClocks(2);
    sxPlay(1);
    MidiClocks(1);
    CHECK(GateOn() && sim::g_sh[0] == 17, "resumed Start-less stream: PLAY locks, step 0 on the next tick");
    MidiClocks(6);
    CHECK(sim::g_sh[0] == 19, "resumed Start-less stream: steps follow the division, not the LFO");
    sxPlay(0);
    midiclk::SetSync(false); RunLoops(2);
  }

  // Jitter smoothing (uclock.h): ticks arrive with pseudo-random
  // jitter; the step boundaries must land on the smoothed timebase, tighter
  // than the arrival jitter they were carried by.
  puts("clock jitter smoothing:");
  {
    auto sxP = [&](uint8_t on) {
      for (uint8_t v : {(uint8_t)0xF0, (uint8_t)0x7D, (uint8_t)0x1E, on, (uint8_t)0xF7}) g_midiQ.push_back(v);
      RunLoops(2);
    };
    midiclk::SetSync(true); midiclk::SetDiv(6); midiclk::SetLfoSync(false);
    {   // pattern 6: four NOTE steps, so every division retriggers the gate
      std::vector<uint8_t> raw = {4, 17, 21, 24, 29, 0};   // len, steps, ratchet byte
      uint16_t x = 0; for (uint8_t v : raw) x ^= v;
      std::vector<uint8_t> b2 = {0xF0, 0x7D, 0x12, 6, (uint8_t)(x & 0x7F), (uint8_t)((x >> 7) & 1)};
      for (size_t i = 0; i < raw.size(); i += 7) {
        uint8_t msb = 0; size_t nn = std::min<size_t>(7, raw.size() - i);
        for (size_t k = 0; k < nn; ++k) if (raw[i + k] & 0x80) msb |= 1 << k;
        b2.push_back(msb); for (size_t k = 0; k < nn; ++k) b2.push_back(raw[i + k] & 0x7F);
      }
      b2.push_back(0xF7);
      for (uint8_t v : b2) g_midiQ.push_back(v); RunLoops(2);
      for (uint8_t v : {0xF0, 0x7D, 0x1D, 6, 0xF7}) g_midiQ.push_back(v); RunLoops(2);
    }
    sxP(1);
    MidiByte(0xFA);
    // 20 ms ticks (25 BPM-ish virtual, irrelevant) with +/-2.5 ms jitter:
    // the ratio matches a real USB host (a few ms on a 20.8 ms tick), and a
    // raw-arrival stepper would show interval deviations up to twice the
    // jitter (~5 ms).
    const uint32_t TICK_US = 20000;              // -> 120 ms divisions
    uint32_t base = micros();
    uint32_t lcg = 12345;
    std::vector<uint32_t> rises;
    bool prevGate = GateOn();
    int32_t maxInJit = 0;
    for (int t = 1; t <= 6 * 30; ++t) {
      lcg = lcg * 1103515245u + 12345u;
      int32_t jit = (int32_t)((lcg >> 16) % 5001) - 2500;   // -2.5..+2.5 ms
      if (jit > maxInJit) maxInJit = jit; if (-jit > maxInJit) maxInJit = -jit;
      uint32_t target = base + (uint32_t)t * TICK_US + (uint32_t)jit;
      while ((int32_t)(target - micros()) > 0) {
        RunLoops(1);
        bool g = GateOn();
        if (g && !prevGate) rises.push_back(micros());
        prevGate = g;
      }
      MidiByte(0xF8);
      bool g = GateOn(); if (g && !prevGate) rises.push_back(micros()); prevGate = g;
    }
    sxP(0); MidiByte(0xFC);
    for (uint8_t v : {0xF0, 0x7D, 0x1D, 0, 0xF7}) g_midiQ.push_back(v); RunLoops(2);   // pattern 0 back
    midiclk::SetSync(false); RunLoops(2);
    // Skip the first few intervals: the period EMA is still converging from
    // its 120 BPM seed to the driven tempo there, which is a tempo change,
    // not jitter.
    int32_t maxDev = 0;
    for (size_t i = 5; i < rises.size(); ++i) {
      int32_t dev = (int32_t)(rises[i] - rises[i - 1]) - 120000;
      if (dev > maxDev) maxDev = dev; if (-dev > maxDev) maxDev = -dev;
    }
    printf("  (%d step onsets; worst interval dev %+.1f ms against +/-%.1f ms tick jitter)\n",
           (int)rises.size(), maxDev / 1000.0, maxInJit / 1000.0);

    CHECK(rises.size() >= 25, "jittered clock: one step per division, none dropped");
    CHECK(maxDev < 2500, "jittered clock: step intervals smoother than the wire (raw would be ~5 ms)");
  }

  // uClock robustness against real masters (uclock.h): every 0xF8 fires
  // exactly one divider tick, so step count == ticks / division under any
  // delivery pattern, and the first clock after Start is the downbeat.
  puts("uClock: MPC Live starts, USB bursts, tempo changes:");
  {
    auto sxP = [&](uint8_t on) {
      for (uint8_t v : {(uint8_t)0xF0, (uint8_t)0x7D, (uint8_t)0x1E, on, (uint8_t)0xF7}) g_midiQ.push_back(v);
      RunLoops(2);
    };
    auto waitUs = [&](uint32_t us) { uint32_t u = micros() + us; while ((int32_t)(u - micros()) > 0) RunLoops(1); };
    midiclk::SetSync(true); midiclk::SetDiv(6); midiclk::SetLfoSync(false);
    for (uint8_t v : {0xF0, 0x7D, 0x1D, 6, 0xF7}) g_midiQ.push_back(v); RunLoops(2);   // 4 NOTE steps
    // Deliver ticks at the given arrival offsets (us from t0); record every
    // gate rise. 'group' ticks arriving at one offset model a USB burst.
    auto drive = [&](const std::vector<uint32_t>& at, std::vector<uint32_t>& rises) {
      uint32_t t0 = micros(); bool prev = GateOn();
      for (uint32_t a : at) {
        while ((int32_t)(t0 + a - micros()) > 0) {
          RunLoops(1); bool g = GateOn(); if (g && !prev) rises.push_back(micros() - t0); prev = g;
        }
        g_midiQ.push_back(0xF8);
      }
      uint32_t end = micros() + 60000;
      while ((int32_t)(end - micros()) > 0) {
        RunLoops(1); bool g = GateOn(); if (g && !prev) rises.push_back(micros() - t0); prev = g;
      }
    };
    // (1) MPC Live: silent while stopped, Start then the first clock 1 ms
    // later, 129 BPM (19.38 ms ticks). Twelve starts: step 0 must sound on
    // the FIRST clock every time (the 808 played one clock late on 8 of 12
    // before its fix), and one step per 6 ticks after it.
    int onTime = 0, countOk = 0;
    for (int run = 0; run < 12; ++run) {
      sxP(0); waitUs(450000);                    // stopped, dead wire
      MidiByte(0xFA); waitUs(1000);
      std::vector<uint32_t> at, rises;
      for (int t = 0; t < 48; ++t) at.push_back((uint32_t)t * 19380);
      drive(at, rises);
      MidiByte(0xFC); RunLoops(2);
      if (!rises.empty() && rises[0] < 1500) ++onTime;
      if (rises.size() == 8) ++countOk;
    }
    printf("  (MPC Live starts: %d/12 on the first clock, %d/12 with exactly 8 steps in 48 ticks)\n", onTime, countOk);
    CHECK(onTime == 12, "MPC Live start: step 0 on the first clock after Start, 12 of 12");
    CHECK(countOk == 12, "MPC Live start: one step per division, none dropped or doubled");
    // (2) USB burst delivery: ticks arrive in pairs and triples (host frames
    // batching bytes). Step count must still be exact, intervals near 6 ticks.
    {
      MidiByte(0xFA);
      std::vector<uint32_t> at, rises;
      for (int t = 0; t < 120; ++t) {
        uint32_t ideal = (uint32_t)t * 20833;
        uint32_t a = (t % 3 == 1) ? ideal + 20833 : (t % 3 == 2 ? ideal : ideal);   // t%3==1 slips into t+1's frame
        at.push_back(a);
      }
      std::sort(at.begin(), at.end());
      drive(at, rises);
      MidiByte(0xFC); RunLoops(2);
      int32_t worst = 0;
      for (size_t i = 3; i < rises.size(); ++i) {
        int32_t d = (int32_t)(rises[i] - rises[i - 1]) - 6 * 20833;
        if (d < 0) d = -d; if (d > worst) worst = d;
      }
      printf("  (USB bursts: %zu steps in 120 ticks, worst interval dev %.1f ms)\n", rises.size(), worst / 1000.0);
      CHECK(rises.size() == 20, "burst-delivered clock: exactly one step per 6 ticks");
      CHECK(worst < 3000, "burst-delivered clock: step intervals stay on the grid");
    }
    // (3) tempo changes mid-run: 120 -> 90 -> 180 BPM, and an exact 2x jump.
    {
      MidiByte(0xFA);
      std::vector<uint32_t> at, rises; uint32_t t = 0;
      for (int i = 0; i < 48; ++i) { at.push_back(t); t += 20833; }
      for (int i = 0; i < 48; ++i) { at.push_back(t); t += 27778; }
      for (int i = 0; i < 48; ++i) { at.push_back(t); t += 13889; }
      for (int i = 0; i < 48; ++i) { at.push_back(t); t += 27778; }
      drive(at, rises);
      MidiByte(0xFC); RunLoops(2);
      printf("  (tempo changes: %zu steps in 192 ticks)\n", rises.size());
      CHECK(rises.size() == 32, "tempo changes: step count exact across 120/90/180/90 BPM");
    }
    sxP(0);
    for (uint8_t v : {0xF0, 0x7D, 0x1D, 0, 0xF7}) g_midiQ.push_back(v); RunLoops(2);
    midiclk::SetSync(false); RunLoops(2);
  }

  // Performance features, end to end through the panel / SysEx and the real
  // engine: direction modes, loop length, queued pattern switch, transpose
  // hold + latch, pedal route, random arp, tune mode, real-time record.
  puts("performance features:");
  {
    auto sx = [&](std::initializer_list<uint8_t> b) {
      g_midiQ.push_back(0xF0); g_midiQ.push_back(0x7D);
      for (uint8_t v : b) g_midiQ.push_back(v);
      g_midiQ.push_back(0xF7); RunLoops(3);
    };
    auto push = [&](uint8_t pat, std::vector<uint8_t> steps) {   // 0x12, no ratchet tail
      std::vector<uint8_t> raw = {(uint8_t)steps.size()};
      raw.insert(raw.end(), steps.begin(), steps.end());
      uint16_t x = 0; for (uint8_t v : raw) x ^= v;
      std::vector<uint8_t> b2 = {0x12, pat, (uint8_t)(x & 0x7F), (uint8_t)((x >> 7) & 1)};
      for (size_t i = 0; i < raw.size(); i += 7) {
        uint8_t msb = 0; size_t nn = std::min<size_t>(7, raw.size() - i);
        for (size_t k = 0; k < nn; ++k) if (raw[i + k] & 0x80) msb |= 1 << k;
        b2.push_back(msb); for (size_t k = 0; k < nn; ++k) b2.push_back(raw[i + k] & 0x7F);
      }
      g_midiQ.push_back(0xF0); g_midiQ.push_back(0x7D);
      for (uint8_t v : b2) g_midiQ.push_back(v);
      g_midiQ.push_back(0xF7); RunLoops(3);
    };
    // Human-plausible 35 ms settle after every button edge: the scan-bleed
    // guard ignores new keys for 30 ms around function-button edges.
    auto btn = [&](int bit, bool down) { if (down) sim::g_buttons |= (uint8_t)(1u << bit); else sim::g_buttons &= (uint8_t)~(1u << bit); RunLoops(3); WaitVirtMs(35); };
    auto tap = [&](int bit) { btn(bit, true); btn(bit, false); };
    auto key = [&](int k, bool down) { if (down) sim::g_keys |= 1UL << k; else sim::g_keys &= ~(1UL << k); RunLoops(3); };
    auto play = [&]() { tap(1); SettleClockReset(); };
    auto run = [&](int n) {            // n clock steps; CV per step, "-" for a gate-off step
      std::string out;
      for (int i = 0; i < n; ++i) {
        ClockTick();
        out += (GateOn() ? std::to_string(sim::g_sh[0]) : std::string("-")) + " ";
      }
      return out;
    };
    auto first = [&]() { return (GateOn() ? std::to_string(sim::g_sh[0]) : std::string("-")) + " "; };
    const int B_LOAD = 0, B_PLAY = 1, B_DOWN = 2, B_UD = 3, B_UP = 4, B_HOLD = 5, B_KT = 7;
    midiclk::SetSync(false); RunLoops(2);
    WaitVirtMs(40);
    push(7, {10, 11, 12, 13});
    push(4, {30, 31});
    sx({0x1D, 7});

    // Directions (SysEx 0x65 id 0), panel PLAY start = step 1 at the press.
    const char* want[4] = {"10 11 12 13 10 11 12 13 10 ", "13 12 11 10 13 12 11 10 13 ",
                           "10 11 12 13 12 11 10 11 12 ", ""};
    const char* nm[4] = {"FWD", "BWD", "PING-PONG", "RANDOM"};
    for (uint8_t d = 0; d < 3; ++d) {
      sx({0x65, 0, d});
      play(); std::string got = first() + run(8); tap(B_PLAY); WaitVirtMs(40);
      printf("  (%s: %s)\n", nm[d], got.c_str());
      CHECK(got == want[d], (std::string("direction ") + nm[d] + " plays the pattern in that order").c_str());
    }
    {
      sx({0x65, 0, 3});
      play(); std::string got = first() + run(31); tap(B_PLAY); WaitVirtMs(40);
      int seen[4] = {0, 0, 0, 0}; bool inSet = true; int n = 0;
      for (size_t i = 0; i < got.size();) {
        size_t j = got.find(' ', i); int v = atoi(got.substr(i, j - i).c_str()); i = j + 1; ++n;
        if (v >= 10 && v <= 13) seen[v - 10]++; else inSet = false;
      }
      printf("  (RANDOM 32 steps: %d/%d/%d/%d)\n", seen[0], seen[1], seen[2], seen[3]);
      CHECK(inSet && n == 32 && seen[0] && seen[1] && seen[2] && seen[3],
            "direction RANDOM plays only the pattern's steps, and all of them");
    }
    // Direction on the panel while playing (SH-1oh1): UP / DOWN / U&D,
    // DOWN + U&D together = random. Play continues throughout.
    play();
    tap(B_DOWN);
    CHECK(engine::DirSetting() == engine::DIR_BWD && (sim_portf & 7) == 2, "playing: DOWN = backward, still playing");
    tap(B_UD);
    CHECK(engine::DirSetting() == engine::DIR_PINGPONG, "playing: U&D = ping-pong");
    btn(B_DOWN, true); tap(B_UD); btn(B_DOWN, false);
    CHECK(engine::DirSetting() == engine::DIR_RANDOM, "playing: DOWN + U&D together = random");
    tap(B_UP);
    CHECK(engine::DirSetting() == engine::DIR_FWD && (sim_portf & 7) == 2, "playing: UP = forward, still playing");
    tap(B_PLAY); WaitVirtMs(40);

    // Real-time loop length: HOLD + key 1 while playing = first 2 steps.
    play();
    btn(B_HOLD, true); key(1, true); key(1, false); btn(B_HOLD, false);
    std::string ll = run(5);
    CHECK(ll == "11 10 11 10 11 ", "HOLD + key 2 while playing loops the first 2 steps");
    btn(B_HOLD, true); key(1, true); key(1, false); btn(B_HOLD, false);
    ll = run(4);
    CHECK(ll == "12 13 10 11 ", "the same key again restores the full pattern");

    // Queued pattern switch: PLAY + key 5 mid-pattern waits for the loop end.
    run(1);                                                  // at step 12 (3rd)
    btn(B_PLAY, true); key(4, true); key(4, false); btn(B_PLAY, false);
    std::string qs = run(5);
    printf("  (queued switch: %s)\n", qs.c_str());
    CHECK(qs == "13 30 31 30 31 ", "pattern select while playing lands at the loop end");
    CHECK(GateOn() && (sim_portf & 7) == 2, "and play continues (PLAY LED)");
    // Song: PLAY held + keys 8 then 5 = chain 7 -> 4, looping.
    btn(B_PLAY, true); key(7, true); key(7, false); key(4, true); key(4, false); btn(B_PLAY, false);
    std::string song = run(12);
    printf("  (song 8,5: %s)\n", song.c_str());
    CHECK(song == "10 11 12 13 30 31 10 11 12 13 30 31 ", "PLAY + keys 8, 5 = song: each pattern in turn, looping");
    // Instant switch: key 5 alone while playing takes over at the same position.
    btn(B_PLAY, true); key(7, true); key(7, false); btn(B_PLAY, false);   // back to plain pattern 8 at the loop end
    run(3);                                                  // 10 11 12: position 2
    key(4, true); key(4, false);                             // pattern 5 takes position 2 % 2 = 0 -> next is its step 1
    std::string sw = run(3);
    printf("  (instant switch: %s)\n", sw.c_str());
    CHECK(sw == "31 30 31 ", "key 5 while playing switches NOW, position kept (mod length)");
    tap(B_PLAY); WaitVirtMs(40);
    // From stop, PLAY + key starts that pattern on release.
    btn(B_PLAY, true); key(7, true); key(7, false); btn(B_PLAY, false); SettleClockReset();
    CHECK(GateOn() && sim::g_sh[0] == 10 && (sim_portf & 7) == 2, "stopped: PLAY + key 8 starts pattern 8 on release");

    // KEY TRANSPOSE, stock: hold + key transposes from the lowest C (key 7),
    // and it STAYS after release (LED lit) until KEY TRANSPOSE + lowest C.
    btn(B_KT, true); key(9, true);
    CHECK(sim::g_sh[0] == 12, "KEY TRANSPOSE + D (C + 2) transposes the sounding step by +2");
    key(9, false); btn(B_KT, false);
    CHECK(run(1) == "13 " && (sim_portf & 0x40), "stock: the transpose stays after release, LED lit");
    btn(B_KT, true); key(7, true); key(7, false); btn(B_KT, false);
    CHECK(run(1) == "12 " && !(sim_portf & 0x40), "KEY TRANSPOSE + lowest C returns to normal, LED off");
    // Transpose mode (double tap): plain keys transpose while playing.
    tap(B_KT); tap(B_KT);
    key(12, true); key(12, false);
    CHECK(run(1) == "18 ", "transpose mode: a plain key (F, C + 5) transposes, no button held");
    bool blinkA = false, blinkB = false;
    for (int i = 0; i < 20; ++i) { WaitVirtMs(30); if (sim_portf & 0x40) blinkA = true; else blinkB = true; }
    CHECK(blinkA && blinkB, "transpose mode: KEY-TRANSPOSE LED blinks");
    key(7, true); key(7, false);
    CHECK(run(1) == "10 ", "transpose mode: the lowest C = normal");
    WaitVirtMs(500);
    tap(B_KT); tap(B_KT);
    WaitVirtMs(300);
    CHECK(!(sim_portf & 0x40), "double tap again leaves transpose mode (LED off)");
    // Shift (SH-1oh1): KEY TRANSPOSE + UP = next step now, + DOWN = previous.
    run(1);                                                  // 11 sounding
    btn(B_KT, true); tap(B_UP);
    CHECK(sim::g_sh[0] == 12, "shift: KEY TRANSPOSE + UP plays the next step now");
    tap(B_DOWN); btn(B_KT, false);
    CHECK(sim::g_sh[0] == 11, "shift: KEY TRANSPOSE + DOWN plays the previous step");
    CHECK(run(1) == "12 ", "and the pattern carries on from there");
    WaitVirtMs(500);

    // Pedal route: pedal = KEY TRANSPOSE; HOLD no longer latches from it.
    sx({0x65, 1, 1});
    sim::g_pedalHold = true; RunLoops(3); key(4, true);
    CHECK(sim::g_sh[0] == 12 - 3 && (sim_portf & 0x20), "pedal routed to KEY TRANSPOSE: pedal + A transposes -3, HOLD LED off");
    key(4, false); key(7, true); key(7, false); sim::g_pedalHold = false; RunLoops(3);
    CHECK(run(1) == "13 ", "pedal + lowest C: back to normal");
    sx({0x65, 1, 0});
    tap(B_PLAY); WaitVirtMs(40);
    // Stock: the transpose applies to keyboard playing too.
    btn(B_KT, true); key(9, true); key(9, false); btn(B_KT, false);
    key(0, true);
    CHECK(GateOn() && sim::g_sh[0] == 19, "keyboard mode: F3 plays 2 semitones up under the transpose");
    key(0, false);
    btn(B_KT, true); key(7, true); key(7, false); btn(B_KT, false);
    WaitVirtMs(500);

    // Random arp (SH-1oh1): DOWN + U&D together; the same again = off.
    btn(B_DOWN, true); tap(B_UD); btn(B_DOWN, false);
    sim::g_keys = (1UL << 0) | (1UL << 4) | (1UL << 7); RunLoops(3);
    SettleClockReset();                                      // arp start pulses CLOCK RESET
    std::string ar; bool blinkOn = false, blinkOff = false;
    for (int i = 0; i < 24; ++i) {
      ClockTick(); ar += std::to_string(sim::g_sh[0]) + " ";
      WaitVirtMs(70); uint8_t l = sim_portf & 7; blinkOn |= (l == 4); blinkOff |= (l == 0);
    }
    bool okSet = true, noRep = true; std::vector<int> vs;
    for (size_t i = 0; i < ar.size();) { size_t j = ar.find(' ', i); vs.push_back(atoi(ar.substr(i, j - i).c_str())); i = j + 1; }
    int dist[64] = {0};
    for (size_t i = 0; i < vs.size(); ++i) {
      if (vs[i] != 17 && vs[i] != 21 && vs[i] != 24) okSet = false;
      if (i && vs[i] == vs[i - 1]) noRep = false; dist[vs[i]]++;
    }
    printf("  (random arp: %s)\n", ar.c_str());
    CHECK(okSet && noRep && dist[17] && dist[21] && dist[24], "random arp: held keys only, all of them, no immediate repeats");
    CHECK(blinkOn && blinkOff, "random arp: U&D LED blinks");
    sim::g_keys = 0; RunLoops(3);
    btn(B_DOWN, true); tap(B_UD); btn(B_DOWN, false);
    CHECK((sim_portf & 7) == 0, "DOWN + U&D again: arp off");

    // Tune mode: KEY TRANSPOSE + LOAD from keyboard mode holds A (440 Hz at 8').
    g_midiOut.clear();
    btn(B_KT, true); tap(B_LOAD); btn(B_KT, false); WaitVirtMs(20);
    bool note69 = false;
    for (size_t i = 0; i + 1 < g_midiOut.size(); ++i) if ((g_midiOut[i] & 0xF0) == 0x90 && g_midiOut[i + 1] == 69) note69 = true;
    CHECK(GateOn() && sim::g_sh[0] == 33, "tune: gate held on A (CV 33 = MIDI 69 at 8')");
    CHECK(note69, "tune: MIDI OUT plays A 440 (note 69) too");
    tap(B_UP);
    CHECK(!GateOn() && (sim_portf & 7) == 0, "tune: any function button leaves it (and does nothing else)");
    WaitVirtMs(40);

    // Stock "Correcting and Adding": LOAD while playing keeps the pattern
    // and re-loads from the step after the one sounding.
    sx({0x1D, 7});
    play(); run(1);                                          // 11 sounding
    tap(B_LOAD);
    CHECK((sim_portf & 7) == 1, "LOAD while playing enters LOAD (LOAD LED)");
    key(0, true); key(0, false);                             // overwrites step 3 (12) with F3 = 17
    tap(B_LOAD);
    play();
    std::string ca = first() + run(4);
    printf("  (corrected: %s)\n", ca.c_str());
    CHECK(ca == "10 11 17 13 10 ", "LOAD mid-play: only the steps entered are replaced, the rest still plays");
    tap(B_PLAY); WaitVirtMs(40);
    // LOAD from stop clears (stock); DOWN deletes the last entered step (SH-1oh1).
    sx({0x1D, 5});
    tap(B_LOAD);
    key(0, true); key(0, false); key(2, true); key(2, false); key(4, true); key(4, false);
    tap(B_DOWN);
    key(5, true); key(5, false);
    tap(B_LOAD);
    play();
    std::string dl = first() + run(3);
    CHECK(dl == "17 19 22 17 ", "LOAD: DOWN takes back the last step before it is replaced");
    tap(B_PLAY); WaitVirtMs(40);
    push(7, {10, 11, 12, 13});

    // Real-time record into an empty pattern (becomes 16 rest steps).
    push(3, {});
    sx({0x1D, 3});
    play();
    btn(B_PLAY, true); tap(B_LOAD); btn(B_PLAY, false);      // RT record on
    // The empty pattern became 16 rests and rewound: the next clock plays
    // step 0, so after two clocks step 1 is sounding, first half.
    run(2);
    key(3, true);
    CHECK(GateOn() && sim::g_sh[0] == 20, "RT record: a key in the first half sounds and lands on this step");
    key(3, false); run(3);                                   // step 4
    sim::g_clock = true; RunLoops(2);                        // step 5's half-step passed
    key(7, true);                                            // -> step 5
    sim::g_clock = false; RunLoops(2);                       // step 5 plays it
    CHECK(GateOn() && sim::g_sh[0] == 24, "RT record: a late key lands on (and plays at) the next step");
    sim::g_clock = true; RunLoops(2);                        // still held at 5's half: tie into 6
    key(7, false);
    sim::g_clock = false; RunLoops(2);
    btn(B_PLAY, true); tap(B_LOAD); btn(B_PLAY, false);      // RT record off
    tap(B_PLAY); WaitVirtMs(40);
    play();
    std::string rt = first() + run(9);
    // legato 5 -> 6: the gate must not drop between them
    printf("  (RT recorded pattern: %s)\n", rt.c_str());
    CHECK(rt == "- 20 - - - 24 24 - - - ", "RT record: playback has the notes where they landed, the held note tied");
    tap(B_PLAY); WaitVirtMs(40);
    sx({0x1D, 0});
    sx({0x65, 0, 0});
  }

  // AUTO LFO division (lfoDiv 127): the division follows the measured
  // free-running LFO period against the incoming tick spacing.
  puts("LFO auto division:");
  {
    for (uint8_t v : {0xF0, 0x7D, 0x22, 0, 1, 6, 1, 127, 0xF7}) g_midiQ.push_back(v);
    RunLoops(2);
    CHECK(midiclk::LfoDivIdx() == midiclk::LFO_DIV_AUTO, "0x22 accepts lfoDiv 127 (AUTO)");
    // ticks every ~10 ms, LFO square at ~120 ms period = 12 ticks = 1/8 (idx 3)
    MidiByte(0xFA);
    uint32_t phase = millis();
    for (int t = 0; t < 240; ++t) {
      MidiByte(0xF8);
      uint32_t until = millis() + 10;
      while ((int32_t)(until - millis()) > 0) {
        sim::g_clock = ((millis() - phase) / 60) & 1;   // 60 ms half-period
        RunLoops(1);
      }
    }
    // Expected division: the nearest LfoTicks(i)*tickMs to the measured LFO
    // period, computed from what the firmware actually measured (sim loop
    // granularity stretches the driven 10 ms ticks to ~12 ms).
    {
      uint8_t want = 0; uint32_t bestE = 0xFFFFFFFF;
      for (uint8_t i = 0; i < midiclk::LFO_DIV_COUNT; ++i) {
        uint32_t p = (uint32_t)midiclk::LfoTicks(i) * midiclk::S().tickMs;
        uint32_t e = p > midiclk::LfoPeriodMs() ? p - midiclk::LfoPeriodMs() : midiclk::LfoPeriodMs() - p;
        if (e < bestE) { bestE = e; want = i; }
      }
      printf("  (AUTO: measured LFO %u ms, tick %u ms -> chose %u, nearest is %u)\n",
             midiclk::LfoPeriodMs(), midiclk::S().tickMs, midiclk::LfoDivEff(), want);
      CHECK(midiclk::LfoDivEff() == want, "AUTO picks the division nearest the measured LFO period");
    }
    CHECK(midiclk::LfoPeriodMs() > 90 && midiclk::LfoPeriodMs() < 150,
          "AUTO's measured LFO period is in range");
    // extended LFO divisions: 10 = 1/2, 11 = 1/1 accepted and reported
    for (uint8_t v : {0xF0, 0x7D, 0x22, 0, 1, 6, 1, 11, 0xF7}) g_midiQ.push_back(v);
    RunLoops(2);
    CHECK(midiclk::LfoDivIdx() == 11 && midiclk::LfoTicks(11) == 96, "lfoDiv 11 = 1/1 (96 ticks)");
    MidiByte(0xFC);
    midiclk::SetSync(false); midiclk::SetLfoDiv(0); sim::g_clock = false; RunLoops(2);
  }

  // Ratchets: a 2x step retriggers once, evenly inside its clock period
  // (SysEx 0x1B; needs one measured period first, so play two full steps
  // before counting). Each MidiByte pass is ~1 ms of virtual time; 6 ticks
  // spaced 10 passes apart makes a ~60 ms division.
  puts("ratchets:");
  {
    midiclk::SetSync(true); midiclk::SetDiv(6);
    auto sx14ok = [&]() {                       // send 0x12 for pattern 7: two notes
      std::vector<uint8_t> raw = {2, (uint8_t)17, (uint8_t)19};
      raw.push_back(0);                          // packed ratchet byte (none yet)
      uint16_t x = 0; for (uint8_t v : raw) x ^= v;
      std::vector<uint8_t> b = {0xF0, 0x7D, 0x12, 7, (uint8_t)(x & 0x7F), (uint8_t)((x >> 7) & 1)};
      for (size_t i = 0; i < raw.size(); i += 7) {
        uint8_t msb = 0; size_t nn = std::min<size_t>(7, raw.size() - i);
        for (size_t k = 0; k < nn; ++k) if (raw[i + k] & 0x80) msb |= 1 << k;
        b.push_back(msb); for (size_t k = 0; k < nn; ++k) b.push_back(raw[i + k] & 0x7F);
      }
      b.push_back(0xF7);
      for (uint8_t v : b) g_midiQ.push_back(v); RunLoops(2);
    };
    sx14ok();
    for (uint8_t v : {0xF0, 0x7D, 0x1B, 7, 0, 1, 0xF7}) g_midiQ.push_back(v); RunLoops(2);  // step 0 -> 2x
    for (uint8_t v : {0xF0, 0x7D, 0x1D, 7, 0xF7}) g_midiQ.push_back(v); RunLoops(2);        // select 7
    for (uint8_t v : {0xF0, 0x7D, 0x1E, 1, 0xF7}) g_midiQ.push_back(v); RunLoops(2);        // play
    MidiByte(0xFA);
    auto division = [&]() { for (int t = 0; t < 6; ++t) { MidiByte(0xF8); RunLoops(10); } };
    division(); division();                     // two steps: the period is now measured
    // Next division starts step 0 (2x): sample the gate through the period.
    int rises = 0; bool prev = GateOn();
    for (int t = 0; t < 6; ++t) {
      MidiByte(0xF8);
      for (int i = 0; i < 10; ++i) { RunLoops(1); bool g = GateOn(); if (g && !prev) ++rises; prev = g; }
    }
    // rise 1 = the step's own note start, rise 2 = the ratchet retrigger
    CHECK(rises >= 2, "2x ratchet: the gate retriggers inside the step");
    for (uint8_t v : {0xF0, 0x7D, 0x1E, 0, 0xF7}) g_midiQ.push_back(v); RunLoops(2);        // stop
    midiclk::SetSync(false);
  }

  // Config menu: HOLD+KEY-TRANSPOSE 2 s -> menu; LOAD -> switch into the
  // 80C49 emulator (the original mask ROM, if embedded). The switch runs IN
  // PROCESS (combined.h SwitchTo): no reboot, no longjmp, no USB drop.
  puts("config menu / combined boot:");
  if (emu101::available()) {
    sim::g_buttons = (1u << 5) | (1u << 7);   // HOLD + KEY-TRANSPOSE
    RunLoops(40000);                          // > 2 s virtual, enters menu
    sim::g_buttons = 0; RunLoops(2);
    sim::g_buttons = 1u << 3; RunLoops(1);    // UP&DOWN: toggle MIDI sync on
    sim::g_buttons = 0; RunLoops(1);
    CHECK(midiclk::SyncOn(), "menu UP&DOWN toggles MIDI sync on");
    sim::g_buttons = 1u << 3; RunLoops(1);    // and back off
    sim::g_buttons = 0; RunLoops(1);
    CHECK(!midiclk::SyncOn(), "menu UP&DOWN toggles MIDI sync off");
    sim::g_buttons = 1u << 0; RunLoops(1);    // LOAD -> SwitchTo(FW_EMU)
    sim::g_buttons = 0; RunLoops(2);          // the pending switch runs next pass
    CHECK(combined_current_fw() == SIM_FW_EMU, "menu LOAD -> switched into the emulator (no reboot)");

    puts("80C49 emulator (original mask ROM):");
    sim::g_buttons = 0;
    RunLoops(4000);                             // ~2 s virtual of ROM execution
    CHECK(sim::g_sh[1] == 0x18, "ROM writes RANGE 8' CV 0x18 to the VCO S/H");
    CHECK(!GateOn(), "ROM: gate off with no keys");
    CheckVersionReply("the 80C49 emulator");
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
    // arp off again (tap DOWN), keyboard mode for the MIDI test
    sim::g_buttons = 1u << 2; RunLoops(30); sim::g_buttons = 0; RunLoops(30);

    // MIDI notes into the emulated key matrix: note 53 = key 0 -> the ROM
    // plays CV 17 with the gate on, and echoes it on MIDI OUT.
    g_midiOut.clear();
    g_midiQ.push_back(0x90); g_midiQ.push_back(53); g_midiQ.push_back(100);
    RunLoops(40);
    CHECK(GateOn() && sim::g_sh[0] == 17, "ROM + MIDI note 53 -> key 0, gate on, CV 17");
    CHECK(g_midiOut.size() >= 3 && g_midiOut[0] == 0x90 && g_midiOut[1] == 53,
          "ROM output echoed on MIDI OUT (note on 53)");
    g_midiQ.push_back(0x80); g_midiQ.push_back(53); g_midiQ.push_back(0);
    RunLoops(40);
    CHECK(!GateOn(), "ROM + MIDI note off -> gate off");

    // Battery RAM: record a 2-step sequence with the ROM (LOAD, keys, LOAD),
    // then the changed RAM 0x1C..0x7F lands in EEPROM after 1 s of quiet.
    {
      auto tap = [&](uint8_t bit) { sim::g_buttons = 1u << bit; RunLoops(40); sim::g_buttons = 0; RunLoops(40); };
      tap(0);                                                   // LOAD
      sim::g_keys = 1UL << 0; RunLoops(40); sim::g_keys = 0; RunLoops(40);
      sim::g_keys = 1UL << 4; RunLoops(40); sim::g_keys = 0; RunLoops(40);
      tap(0);                                                   // LOAD: done
      RunLoops(3000);                                           // 1.5 s quiet + writes
      bool magic = eeprom_read_byte(EE_EMU_MAGIC) == EE_EMU_MAGIC_VAL;
      int nz = 0; for (uint8_t i = 0; i < EMU_RAM_COUNT; ++i) if (eeprom_read_byte(EE_EMU_RAM + i)) ++nz;
      CHECK(magic && nz > 0, "ROM sequence persisted to EEPROM (magic + data)");
      CHECK(eeprom_read_byte(EE_EMU_RAM + 10) == 0, "EEPROM image carries the ROM's zeros after the last step (not 0xFF)");
    }

    // Sync ON slaves the ROM's transport: Start taps PLAY from idle, Stop
    // taps it back off (the LED select is the mode readback).
    {
      midiclk::SetSync(true); RunLoops(30);
      g_midiQ.push_back(0xFA); RunLoops(400);
      CHECK((emu101::romP2() & 0x07) == 2, "ROM + sync on: 0xFA starts the ROM's sequencer (PLAY LED)");
      g_midiQ.push_back(0xFC); RunLoops(400);
      CHECK((emu101::romP2() & 0x07) != 2, "ROM + sync on: 0xFC stops it");
      midiclk::SetSync(false); RunLoops(30);
    }

    // Menu works from the emulator side too: PLAY -> back to SuperOS.
    sim::g_buttons = (1u << 5) | (1u << 7);
    RunLoops(4800);                           // 2 s at 500 us/slice
    sim::g_buttons = 0; RunLoops(4);
    sim::g_buttons = 1u << 1; RunLoops(1);    // PLAY -> SwitchTo(FW_SUPEROS)
    sim::g_buttons = 0; RunLoops(2);          // the pending switch runs next pass
    CHECK(combined_current_fw() != SIM_FW_EMU, "menu PLAY -> switched into SuperOS (no reboot)");
  } else {
    puts("  SKIP  no mask ROM embedded (SH101-reference dump absent)");
  }

  remove("sim-selftest-eeprom.bin");
  printf("\n%s (%d failure%s)\n", g_fails ? "SELF-TEST FAILED" : "self-test passed",
         g_fails, g_fails == 1 ? "" : "s");
  return g_fails ? 1 : 0;
}


// ===========================================================================
// G14 Phase 0 (claim-096): the divergence list, engine.h vs the mask ROM
// ===========================================================================
// The subject is src/engine.h running natively. The ORACLE is the genuine
// 80C49 mask ROM running under the emulator. Both are in this one binary and
// both write the same observable surface, which is the whole reason this is
// affordable: sim::g_sh[0] is the CV-OUT sample/hold the pitch DAC follows,
// sim::g_sh[1] the VCO range S/H, sim_portf bit7 the GATE (LOW = on) and bit4
// CLOCK RESET (active-LOW pulse). Nothing here reads either implementation's
// internals; if a behaviour can only be compared by looking inside, it is
// reported UNTESTED rather than matched.
//
// THREE THINGS THIS HARNESS DOES NOT DO, stated so they are not read as
// coverage:
//
//  1. It drives by VIRTUAL TIME, not by loop passes. A SuperOS pass and a ROM
//     pass differ by more than an order of magnitude, so `RunLoops(40)` means
//     two different amounts of machine time to the two subjects and any
//     comparison built on it compares the harness with itself. RunUs() is the
//     only stepping primitive here.
//  2. It compares an ORDERED SEQUENCE of observable changes, not per-pass
//     samples. The two subjects reach the same state at different instants by
//     construction; requiring them to reach it in the same order is the
//     strongest thing that is true.
//  3. The SOUND channel has no pass condition (CLEANROOM.md), so nothing here
//     claims one. Edge placement in microseconds is recorded and printed, and
//     is NOT scored.
//
// Everything SuperOS has that the ROM never had (SysEx, the 8-pattern flash
// store, host edits, MIDI clock division) is an EXTENSION, not a divergence.
// Extensions are listed separately and are not counted either way.
namespace g14 {

struct Obs {
  uint8_t cv, vco;
  bool gate, clkrst;
  bool rawGate;   // emulator only: the gate bit the ROM itself wrote (emu101.h
                  // romP2()), before this firmware's gate shaping. In SuperOS
                  // it mirrors `gate`, so the stream is only meaningful on the
                  // oracle side and is used to ATTRIBUTE a gate divergence to
                  // the ROM or to our shaping, never to score the subject.
  bool operator==(const Obs& o) const {
    return cv == o.cv && vco == o.vco && gate == o.gate && clkrst == o.clkrst
        && rawGate == o.rawGate;
  }
};
struct Ev { uint64_t us; Obs o; };

static std::vector<Ev> g_trace;
static bool     g_recording = false;
static Obs      g_last{0, 0, false, false, false};
static bool     g_haveLast = false;
static uint64_t g_t0 = 0;
static int      g_break = 0;     // deliberate subject damage; 0 = none

static Obs Sample() {
  Obs o;
  o.cv     = sim::g_sh[0];
  o.vco    = sim::g_sh[1];
  o.gate   = !(sim_portf & 0x80);
  o.clkrst = (sim_portf & 0x10) == 0;
  o.rawGate = combined_current_fw() == SIM_FW_EMU
              ? !(emu101::romP2() & (1u << P2_GATE)) : o.gate;
  // --- deliberate damage, applied at the OBSERVATION point on the SUBJECT
  // only. The comparator must be seen to go red before an empty divergence
  // list means anything (CLEANROOM Phase 0 red condition, claim-081). Doing
  // it here rather than in engine.h keeps the shipped engine untouched while
  // still proving the comparator can fail.
  if (g_break && combined_current_fw() != SIM_FW_EMU) {
    if (g_break == 1) o.cv = (uint8_t)((o.cv + 1) & 0x3F);   // pitch off by one
    if (g_break == 2) o.gate = !o.gate;                       // gate inverted
    if (g_break == 3) o.vco = (uint8_t)((o.vco + 8) & 0x3F);  // wrong VCO range
  }
  return o;
}

static void Poll() {
  if (!g_recording) return;
  Obs o = Sample();
  if (!g_haveLast || !(o == g_last)) {
    g_trace.push_back(Ev{sim::g_virtualUs - g_t0, o});
    g_last = o; g_haveLast = true;
  }
}

// Step the firmware for `us` of VIRTUAL time, sampling after every pass.
static void RunUs(uint64_t us) {
  const uint64_t end = sim::g_virtualUs + us;
  int guard = 0;
  while (sim::g_virtualUs < end) {
    const uint64_t before = sim::g_virtualUs;
    loop();
    Poll();
    if (sim::g_virtualUs == before && ++guard > 2000000) break;   // no-time pass
  }
}

// One clock pulse on T1, in real time rather than in passes: the ROM's loop is
// ~3.5 ms, so a pulse shorter than that is invisible to it and a comparison
// driven in passes would report the ROM as deaf to a clock SuperOS heard.
static void ClockPulseUs(uint64_t highUs, uint64_t lowUs) {
  sim::g_clock = true;  RunUs(highUs);
  sim::g_clock = false; RunUs(lowUs);
}

static void Quiet() {
  sim::g_keys = 0; sim::g_buttons = 0; sim::g_pedalHold = false;
  sim::g_clock = false;
  RunUs(300000);
}
static void Tap(uint8_t bit) {
  sim::g_buttons = (uint8_t)(1u << bit); RunUs(120000);
  sim::g_buttons = 0;                    RunUs(120000);
}

// Record a BASELINE event before the body runs. Without it the first event of
// each trace is whatever each subject happened to reach inside its own first
// pass, which differ by an order of magnitude, and that alone made 9 of the
// first 18 behaviours read as divergences on the harness's own start-up. Both
// subjects genuinely carry the previous CV across the gap (the ROM in the S/H,
// engine.h in lastCv_); that is only comparable if the starting point is in
// the trace.
static void Begin() { g_trace.clear(); g_haveLast = false; g_recording = true;
                      g_t0 = sim::g_virtualUs; Poll(); }
static std::vector<Ev> End() { g_recording = false; return g_trace; }

// --- the behaviours -------------------------------------------------------
// Each is a plain function run identically against both subjects. Adding one
// here is the whole cost of extending coverage.
typedef void (*Body)();

static void B_key_single()  { sim::g_keys = 1UL << 0; RunUs(400000);
                              sim::g_keys = 0;        RunUs(400000); }
static void B_key_high()    { sim::g_keys = 1UL << 20; RunUs(400000);
                              sim::g_keys = 0;         RunUs(400000); }
static void B_key_priority(){ sim::g_keys = 1UL << 10; RunUs(300000);
                              sim::g_keys |= 1UL << 0; RunUs(300000);   // lower joins
                              sim::g_keys = 1UL << 10; RunUs(300000);   // lower leaves
                              sim::g_keys = 0;         RunUs(300000); }
static void B_key_legato()  { sim::g_keys = 1UL << 4;  RunUs(300000);
                              sim::g_keys |= 1UL << 8; RunUs(300000);
                              sim::g_keys = 1UL << 8;  RunUs(300000);
                              sim::g_keys = 0;         RunUs(300000); }
// A panel switch and a key release in the same instant are two stimuli the two
// subjects sample in a different order. They are separated in time here. That
// is the harness being fair, not the test being loosened: the switch is still
// exercised, just not simultaneously with a second event.
static void B_octave_up()   { sim::g_octave = 1;  RunUs(200000);
                              sim::g_keys = 1UL << 0; RunUs(400000);
                              sim::g_keys = 0;        RunUs(300000);
                              sim::g_octave = 0;      RunUs(200000); }
static void B_octave_down() { sim::g_octave = -1; RunUs(200000);
                              sim::g_keys = 1UL << 12; RunUs(400000);
                              sim::g_keys = 0;         RunUs(300000);
                              sim::g_octave = 0;       RunUs(200000); }
static void B_range_16()    { sim::g_rangeFeet = 16; RunUs(500000);
                              sim::g_rangeFeet = 8;  RunUs(500000); }
static void B_range_4()     { sim::g_rangeFeet = 4;  RunUs(500000);
                              sim::g_rangeFeet = 8;  RunUs(500000); }
static void B_range_2()     { sim::g_rangeFeet = 2;  RunUs(500000);
                              sim::g_rangeFeet = 8;  RunUs(500000); }
static void B_arp_up()      { Tap(4); sim::g_keys = (1UL<<0)|(1UL<<4)|(1UL<<7);
                              RunUs(200000);
                              for (int i = 0; i < 8; ++i) ClockPulseUs(6000, 6000);
                              sim::g_keys = 0; RunUs(200000); Tap(4); }
static void B_arp_down()    { Tap(2); sim::g_keys = (1UL<<0)|(1UL<<4)|(1UL<<7);
                              RunUs(200000);
                              for (int i = 0; i < 8; ++i) ClockPulseUs(6000, 6000);
                              sim::g_keys = 0; RunUs(200000); Tap(2); }
static void B_arp_updown()  { Tap(3); sim::g_keys = (1UL<<0)|(1UL<<4)|(1UL<<7);
                              RunUs(200000);
                              for (int i = 0; i < 10; ++i) ClockPulseUs(6000, 6000);
                              sim::g_keys = 0; RunUs(200000); Tap(3); }
static void B_arp_keychange(){Tap(4); sim::g_keys = (1UL<<0)|(1UL<<4);
                              RunUs(200000);
                              for (int i = 0; i < 4; ++i) ClockPulseUs(6000, 6000);
                              sim::g_keys = (1UL<<0)|(1UL<<4)|(1UL<<9);
                              for (int i = 0; i < 6; ++i) ClockPulseUs(6000, 6000);
                              sim::g_keys = 0; RunUs(200000); Tap(4); }
// The arp gate result below turns on the ROM holding the gate high across
// steps. That could be a property of the T1 duty cycle the harness happens to
// use rather than of the ROM, so it is driven at three duties. If the ROM's
// gate behaviour is the same at 8%, 50% and 92%, duty is not the explanation.
static void B_arp_narrow()  { Tap(4); sim::g_keys = (1UL<<0)|(1UL<<4)|(1UL<<7);
                              RunUs(200000);
                              for (int i = 0; i < 8; ++i) ClockPulseUs(1000, 11000);
                              sim::g_keys = 0; RunUs(200000); Tap(4); }
static void B_arp_wide()    { Tap(4); sim::g_keys = (1UL<<0)|(1UL<<4)|(1UL<<7);
                              RunUs(200000);
                              for (int i = 0; i < 8; ++i) ClockPulseUs(11000, 1000);
                              sim::g_keys = 0; RunUs(200000); Tap(4); }
static void B_hold_pedal()  { sim::g_pedalHold = true;  sim::g_keys = 1UL << 2;
                              RunUs(300000);
                              sim::g_keys = 0;          RunUs(400000);
                              sim::g_pedalHold = false; RunUs(300000); }
static void B_hold_button() { Tap(5); sim::g_keys = 1UL << 2; RunUs(300000);
                              sim::g_keys = 0;               RunUs(400000);
                              Tap(5);                        RunUs(200000); }
static void B_seq_record()  { Tap(0);                                    // LOAD
                              sim::g_keys = 1UL<<0; RunUs(200000); sim::g_keys = 0; RunUs(200000);
                              sim::g_keys = 1UL<<4; RunUs(200000); sim::g_keys = 0; RunUs(200000);
                              sim::g_keys = 1UL<<7; RunUs(200000); sim::g_keys = 0; RunUs(200000);
                              Tap(0);                                    // LOAD: done
                              Tap(1);                                    // PLAY
                              for (int i = 0; i < 9; ++i) ClockPulseUs(6000, 6000);
                              Tap(1); RunUs(200000); }
static void B_seq_rest()    { Tap(0);
                              sim::g_keys = 1UL<<0; RunUs(200000); sim::g_keys = 0; RunUs(200000);
                              Tap(7);                                    // KEY-TRANS = rest
                              sim::g_keys = 1UL<<7; RunUs(200000); sim::g_keys = 0; RunUs(200000);
                              Tap(0); Tap(1);
                              for (int i = 0; i < 9; ++i) ClockPulseUs(6000, 6000);
                              Tap(1); RunUs(200000); }
static void B_clock_reset() { sim::g_adsr = 1;                            // ADSR = LFO
                              sim::g_keys = 1UL << 0; RunUs(400000);
                              sim::g_keys = 0;        RunUs(300000);
                              sim::g_adsr = 0;        RunUs(200000); }

struct Behaviour { const char* name; const char* channel; Body fn; };
static const Behaviour BEHAVIOURS[] = {
  {"keyboard: single key on/off",            "state", B_key_single},
  {"keyboard: key near the top of the range","state", B_key_high},
  {"keyboard: note priority, lower key joins and leaves", "state", B_key_priority},
  {"keyboard: second key while one is held", "state", B_key_legato},
  {"panel: OCTAVE up",                       "state", B_octave_up},
  {"panel: OCTAVE down",                     "state", B_octave_down},
  {"panel: RANGE 16'",                       "state", B_range_16},
  {"panel: RANGE 4'",                        "state", B_range_4},
  {"panel: RANGE 2'",                        "state", B_range_2},
  {"arp UP over three keys, 8 clocks",       "time",  B_arp_up},
  {"arp DOWN over three keys, 8 clocks",     "time",  B_arp_down},
  {"arp UP&DOWN over three keys, 10 clocks", "time",  B_arp_updown},
  {"arp: key added mid-run",                 "time",  B_arp_keychange},
  {"arp UP, narrow clock (8% duty)",         "time",  B_arp_narrow},
  {"arp UP, wide clock (92% duty)",          "time",  B_arp_wide},
  {"HOLD pedal latches a key",               "state", B_hold_pedal},
  {"HOLD button latches a key",              "state", B_hold_button},
  {"sequencer: record 3 notes, play 9 clocks","time", B_seq_record},
  {"sequencer: a rest recorded with KEY-TRANSPOSE", "time", B_seq_rest},
  {"CLOCK RESET pulse on a new key (ADSR=LFO)", "state", B_clock_reset},
};
static const int N_BEHAVIOURS = (int)(sizeof(BEHAVIOURS) / sizeof(BEHAVIOURS[0]));

// Extensions: SuperOS has these and the original never did, so they are
// outside 1:1 by construction. Listed so the reader can see they were
// considered and excluded on purpose rather than forgotten.
static const char* EXTENSIONS[] = {
  "SysEx (0x10..0x4D): the ROM has no host protocol at all",
  "8 flash-backed patterns of up to 128 steps: the ROM has one battery-RAM pattern",
  "MIDI clock division table (midiclock.h): the ROM has only the T1 pin",
  "live host edits under clock (0x16/0x18/0x2E, gate G4)",
  "USB-MIDI, the bench probe (0x3B) and the 0x3E bench window",
};
static const int N_EXTENSIONS = (int)(sizeof(EXTENSIONS) / sizeof(EXTENSIONS[0]));

// --- the comparison -------------------------------------------------------
// Three DERIVED streams, not one, because a single root cause otherwise gets
// counted once per behaviour and the list stops being a list of causes:
//
//   cv     the ordered distinct CV-OUT values (what pitch, in what order)
//   gate   the CV in force at each gate RISE (what you would hear retriggered)
//   order  the raw interleave of the two (whether CV leads the gate or not)
//
// A behaviour whose cv and gate streams match but whose order stream does not
// has ONE divergence, in edge ordering, and saying that is worth more than
// eight rows saying "diverges".
//
// The BASELINE event (index 0) is excluded from all three. It is the CV each
// subject was holding when the behaviour started, which is inherited from
// whatever ran before, so once any behaviour diverges every later baseline
// differs too and the list fills with cascade. It is printed, never scored,
// and "does CV persist after the gate closes" is its own row instead.
static std::vector<std::string> CvSeq(const std::vector<Ev>& t) {
  std::vector<std::string> v; std::string prev;
  for (size_t i = 1; i < t.size(); ++i) {
    char b[16]; snprintf(b, sizeof(b), "%u", t[i].o.cv);
    if (b != prev) { v.push_back(b); prev = b; }
  }
  return v;
}
static std::vector<std::string> GateSeq(const std::vector<Ev>& t) {
  std::vector<std::string> v; bool prev = t.empty() ? false : t[0].o.gate;
  for (size_t i = 1; i < t.size(); ++i) {
    if (t[i].o.gate && !prev) { char b[16];
      snprintf(b, sizeof(b), "%u", t[i].o.cv); v.push_back(b); }
    prev = t[i].o.gate;
  }
  return v;
}
static std::vector<std::string> OrderSeq(const std::vector<Ev>& t) {
  std::vector<std::string> v; std::string prev;
  for (size_t i = 1; i < t.size(); ++i) {
    char b[32]; snprintf(b, sizeof(b), "%u/%c", t[i].o.cv, t[i].o.gate ? 'G' : '.');
    if (b != prev) { v.push_back(b); prev = b; }
  }
  return v;
}
// The gate stream as the ROM wrote it, before shaping. Same shape as GateSeq.
static std::vector<std::string> RawGateSeq(const std::vector<Ev>& t) {
  std::vector<std::string> v; bool prev = t.empty() ? false : t[0].o.rawGate;
  for (size_t i = 1; i < t.size(); ++i) {
    if (t[i].o.rawGate && !prev) { char b[16];
      snprintf(b, sizeof(b), "%u", t[i].o.cv); v.push_back(b); }
    prev = t[i].o.rawGate;
  }
  return v;
}
static std::vector<std::string> VcoSeq(const std::vector<Ev>& t) {
  std::vector<std::string> v; std::string prev;
  for (size_t i = 1; i < t.size(); ++i) {
    char b[16]; snprintf(b, sizeof(b), "%u", t[i].o.vco);
    if (b != prev) { v.push_back(b); prev = b; }
  }
  return v;
}
static std::string Join(const std::vector<std::string>& v, size_t cap = 22) {
  std::string s;
  for (size_t i = 0; i < v.size() && i < cap; ++i) { if (i) s += " "; s += v[i]; }
  if (v.size() > cap) s += " ...";
  return s;
}

static std::vector<std::vector<Ev> > g_sos, g_emu;

static void RunAll(std::vector<std::vector<Ev> >& into) {
  into.clear();
  for (int i = 0; i < N_BEHAVIOURS; ++i) {
    Quiet();
    Begin();
    BEHAVIOURS[i].fn();
    into.push_back(End());
    Quiet();
  }
}

int Main() {
  const char* bs = getenv("G14_BREAK");
  g_break = bs ? atoi(bs) : 0;
  setenv("SIM_EEPROM", "sim-g14-eeprom.bin", 1);
  setenv("SIM_FLASH",  "sim-g14-flash.bin",  1);
  remove("sim-g14-eeprom.bin");
  remove("sim-g14-flash.bin");

  if (!emu101::available()) {
    puts("G14 Phase 0: NO MASK ROM EMBEDDED. There is no oracle, so there is");
    puts("no divergence list. This is not a green result.");
    return 2;
  }
  printf("G14 Phase 0 -- claim-096, the 101's divergence list\n");
  printf("subject: src/engine.h native   oracle: 80C49 mask ROM under emu101\n");
  if (g_break)
    printf("\n*** RED-CONDITION RUN: subject deliberately damaged (break %d).\n"
           "    Divergences here are the harness working, not the engine failing.\n"
           "    A CLEAN result under a break is a defect in the comparator. ***\n",
           g_break);

  g_rebootArmed = true;
  if (setjmp(g_rebootJmp) == 0) setup(); else setup();
  RunAll(g_sos);

  SwitchTo(1); RunLoops(1);   // in-process switch, performed at the next pass
  if (combined_current_fw() != SIM_FW_EMU) {
    puts("failed to reach emulator mode; no comparison possible");
    return 2;
  }
  RunAll(g_emu);

  int matched = 0, untested = 0;
  int dCv = 0, dGate = 0, dOrder = 0, dVco = 0, dAny = 0;
  printf("\n%-3s %-6s %-46s %s\n", "#", "chan", "behaviour", "cv gate order vco");
  for (int i = 0; i < N_BEHAVIOURS; ++i) {
    std::vector<std::string> ac = CvSeq(g_sos[i]),  bc = CvSeq(g_emu[i]);
    std::vector<std::string> ag = GateSeq(g_sos[i]), bg = GateSeq(g_emu[i]);
    std::vector<std::string> ao = OrderSeq(g_sos[i]), bo = OrderSeq(g_emu[i]);
    std::vector<std::string> av = VcoSeq(g_sos[i]),  bv = VcoSeq(g_emu[i]);
    bool empty = ao.empty() && bo.empty();
    bool okc = ac == bc, okg = ag == bg, oko = ao == bo, okv = av == bv;
    if (empty) { ++untested;
      printf("%-3d %-6s %-46s %s\n", i + 1, BEHAVIOURS[i].channel,
             BEHAVIOURS[i].name, "UNTESTED: no observable change in EITHER");
      continue; }
    if (!okc) ++dCv;
    if (!okg) ++dGate;
    if (!oko) ++dOrder;
    if (!okv) ++dVco;
    bool all = okc && okg && oko && okv;
    if (all) ++matched; else ++dAny;
    printf("%-3d %-6s %-46s %s %s %s %s\n", i + 1, BEHAVIOURS[i].channel,
           BEHAVIOURS[i].name,
           okc ? "ok " : "CV ", okg ? "ok  " : "GATE", oko ? "ok   " : "ORDER",
           okv ? "ok " : "VCO");
    if (!all) {
      if (!okc) { printf("        cv    engine %s\n", Join(ac).c_str());
                  printf("        cv    ROM    %s\n", Join(bc).c_str()); }
      if (!okg) { printf("        gate  engine %s\n", Join(ag).c_str());
                  printf("        gate  ROM    %s   (physical pin)\n", Join(bg).c_str());
                  std::vector<std::string> br = RawGateSeq(g_emu[i]);
                  printf("        gate  ROM    %s   (RAW, pre-shaping)\n", Join(br).c_str());
                  printf("        ATTRIBUTION: %s\n",
                         br == ag ? "the RAW ROM gate MATCHES engine.h. The divergence is "
                                    "in this firmware's own gate shaping (emu101.cpp), NOT "
                                    "in the original."
                                  : "the raw ROM gate differs from engine.h too, so the "
                                    "divergence is not shaping alone."); }
      if (okc && okg && !oko) {
                  printf("        order engine %s\n", Join(ao).c_str());
                  printf("        order ROM    %s\n", Join(bo).c_str()); }
      if (!okv) { printf("        vco   engine %s\n", Join(av).c_str());
                  printf("        vco   ROM    %s\n", Join(bv).c_str()); }
      printf("        baseline cv engine %u, ROM %u (inherited, not scored)\n",
             g_sos[i].empty() ? 0 : g_sos[i][0].o.cv,
             g_emu[i].empty() ? 0 : g_emu[i][0].o.cv);
    }
  }

  printf("\nmatched %d   diverged %d   UNTESTED %d   of %d behaviours\n",
         matched, dAny, untested, N_BEHAVIOURS);
  // A303's check, and it caught a real hole here. Their accent channel was
  // constant 0 on both sides across 400 cases, so its "0 divergences" carried
  // no information at all. Count the DISTINCT values each stream actually took
  // across every behaviour and BOTH subjects: a stream that never varies
  // cannot report a divergence and must not be quoted as scope.
  {
    std::set<int> vc, vg, vv;
    for (int i = 0; i < N_BEHAVIOURS; ++i)
      for (int k = 0; k < 2; ++k) {
        const std::vector<Ev>& t = k ? g_emu[i] : g_sos[i];
        for (size_t j = 0; j < t.size(); ++j) {
          vc.insert(t[j].o.cv);
          vg.insert(t[j].o.gate ? 1 : 0);
          vv.insert(t[j].o.vco);
        }
      }
    printf("stream information (distinct values over all behaviours, both subjects):\n"
           "  cv %zu   gate %zu   vco %zu\n", vc.size(), vg.size(), vv.size());
    if (vv.size() < 2)
      printf("  WARNING: vco is CONSTANT across every case. Its 0 divergences\n"
             "  carry NO information and it must not be counted as covered.\n");
    if (vc.size() < 2) printf("  WARNING: cv is CONSTANT, same caveat.\n");
    if (vg.size() < 2) printf("  WARNING: gate is CONSTANT, same caveat.\n");
  }
  printf("by stream:  cv %d   gate %d   order %d   vco %d\n",
         dCv, dGate, dOrder, dVco);
  printf("\nNOT COVERED HERE, and not counted either way:\n");
  printf("  sound channel  -- no pass condition exists (CLEANROOM.md); edge\n");
  printf("                    placement is recorded but NOT scored\n");
  printf("  %d SuperOS extensions the original never had:\n", N_EXTENSIONS);
  for (int i = 0; i < N_EXTENSIONS; ++i) printf("    - %s\n", EXTENSIONS[i]);
  printf("\nRED CONDITION: re-run with G14_BREAK=1 (pitch off by one), 2 (gate\n");
  printf("inverted) or 3 (wrong VCO range). If those do not turn this list red,\n");
  printf("a clean list means nothing (claim-081).\n");
  return 0;
}

} // namespace g14

int main(int argc, char** argv) {
  // Line-buffer stdout. A terminal gives us this for free, a PIPE does not: C
  // stdio switches to 4 kB full buffering the moment stdout is not a tty, so
  // every `[ok]` a driving script waits for sits in this process's buffer while
  // that script blocks on read() and this one blocks on getline(). Both sides
  // then look alive and idle, which is what it cost to find. The G12 suite
  // adapter drives this REPL over a pipe (tools/diag/suite_adapter.py SimLink),
  // so this line is load-bearing for the sim transport, not a tidy-up.
  setvbuf(stdout, nullptr, _IOLBF, 0);
  if (argc > 1 && !strcmp(argv[1], "--test")) return SelfTest();
  if (argc > 1 && !strcmp(argv[1], "--g14"))  return g14::Main();
  Repl();
  return 0;
}

// --- CV DAC (cvdac.h) host hook: record the last code per channel -----------
uint16_t g_simCvCode[2] = {0, 0};
extern "C" void sim_cvdac_write(uint8_t ch, uint16_t code) { g_simCvCode[ch & 1] = code; }
