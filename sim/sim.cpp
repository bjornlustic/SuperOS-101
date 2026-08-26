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
#include <vector>

// Combined-build hooks (src/combined.h / boot.cpp / emu101.h / midiclock.h)
uint8_t combined_current_fw();
void RebootTo(uint8_t fw);
namespace emu101 { bool available(); const uint8_t* ram(); }
#include "midiclock.h"   // inline; resolved via -I ../src
#include "midi.h"
#include "usbmidi_codec.h"
#include "flash_persist.h"
#include "cvdac.h"
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
    edit({0x38, 5, 0, 64, 57, 96}); int la = readpat(5, a);
    edit({0x38, 5, 0, 64, 57, 96}); int lb = readpat(5, b);
    CHECK(la == lb && la > 0 && memcmp(a, b, la) == 0, "G4 0x38 randomize deterministic (same seed)");

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
        sx({0x38, P, 1, 100, 21, 33});                     // randomize notes-only, deterministic
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
  // Line-buffer stdout. A terminal gives us this for free, a PIPE does not: C
  // stdio switches to 4 kB full buffering the moment stdout is not a tty, so
  // every `[ok]` a driving script waits for sits in this process's buffer while
  // that script blocks on read() and this one blocks on getline(). Both sides
  // then look alive and idle, which is what it cost to find. The G12 suite
  // adapter drives this REPL over a pipe (tools/diag/suite_adapter.py SimLink),
  // so this line is load-bearing for the sim transport, not a tidy-up.
  setvbuf(stdout, nullptr, _IOLBF, 0);
  if (argc > 1 && !strcmp(argv[1], "--test")) return SelfTest();
  Repl();
  return 0;
}

// --- CV DAC (cvdac.h) host hook: record the last code per channel -----------
uint16_t g_simCvCode[2] = {0, 0};
extern "C" void sim_cvdac_write(uint8_t ch, uint16_t code) { g_simCvCode[ch & 1] = code; }
