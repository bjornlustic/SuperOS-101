// SuperOS-101 — main loop
//
// Mirrors the original SH-101 program loop (steps 3–13, see docs/FIRMWARE.md).
// The DAC/mux protocol and scan invariants are faithful (src/hw.h). Panel decode
// (src/controls.h) and the keyboard/arp/sequencer engine (src/engine.h) are wired
// in. Remaining stubs: KEY-TRANSPOSE shift value, the function-LED 4552 multiplex,
// and exact clock-reset/portamento timing — all marked at their sites.
#include <Arduino.h>
#include "pins.h"
#include "hw.h"
#include "controls.h"
#include "engine.h"
#include "menu.h"
#include "midiclock.h"

static engine::NoteEngine    g_engine;
static controls::Buttons     g_buttons;
static bool                  g_lastClock = false;
static uint8_t               g_rnd = 0xA5;
static uint8_t               g_rndHeld = 0;       // synced S&H latch (MIDI sync on)
static uint32_t              g_rstUntilMs = 0;    // CLOCK-RESET pulse stretcher

// Scan the 4x8 keyboard into a 32-bit held-key mask (bit 0 = lowest key).
static uint32_t ReadKeyboard() {
  uint32_t mask = 0;
  for (uint8_t bank = 0; bank < 4; ++bank) {
    uint8_t keys = hw::ScanRead((ScanCol)(COL_KBD_BANK0 + bank)); // 1 = down
    mask |= (uint32_t)keys << (bank * 8);
  }
  return mask;
}

// LFO/EXT-CLK H->L edge (SH101.asm:768-789) + a placeholder random byte.
static uint8_t ClockAndRandom(bool &edge) {
  bool now = hw::ClockLevel();
  edge = (g_lastClock && !now);
  g_lastClock = now;
  uint8_t x = g_rnd; x ^= x << 3; x ^= x >> 5; x ^= x << 1;
  g_rnd = x ? x : 0xA5;
  return g_rnd & 0x3F;
}

// Entry points renamed for the combined build; src/boot.cpp owns the real
// setup()/loop() and dispatches here or to the 80C49 emulator (emu101.cpp).
void superos_setup() {
  hw::Init();
  g_engine.begin();
#ifdef USB_SERIAL
  Serial.begin(115200);
#endif
}

void superos_loop() {
  // MIDI realtime: divided clock, transport (midiclock.h)
  midiclk::Poll();
  bool divEdge = midiclk::ConsumeEdge();
  if (midiclk::ConsumeStart()) g_engine.restart();

  // 3. RANGE / OCTAVE / ADSR
  controls::Panel panel = controls::ReadPanel();

  // 4. RANGE DATA OUTPUT -> VCO sample/hold (octave-select CV)
  hw::DacWrite(panel.rangeOffset, ROUTE_VCO);

  // 5. KEYBOARD READ + 8. FUNCTION SWITCH READ
  uint32_t keys = ReadKeyboard();
  g_buttons.Scan();
  menu::NotifyButtons(g_buttons.now, millis());  // HOLD+KEY-TRANSP 2 s -> config menu

  // 6/7. CLOCK CHECK + RANDOM DATA OUTPUT -> RANDOM sample/hold
  // Sync ON: the divided MIDI clock replaces the T1 (LFO/EXT-CLK) edge, and
  // the random S&H steps once per division instead of free-running.
  bool clockEdge;
  uint8_t random = ClockAndRandom(clockEdge);
  if (midiclk::SyncOn()) {
    clockEdge = divEdge;
    if (divEdge) g_rndHeld = random;
    random = g_rndHeld;
  }
  hw::DacWrite(random, ROUTE_RANDOM);

  // 9-12. LOAD/PLAY/ARP/keyboard -> note CV, then CV OUTPUT -> CV-OUT sample/hold
  engine::Output out = g_engine.update(keys, panel, g_buttons, clockEdge, random);
  hw::DacWrite(out.cv, ROUTE_CV_OUT);

  // 13. GATE / CLOCK-RESET / (LED + PORTAMENTO: TODO)
  // CLOCK RESET fires on engine events and, with sync ON, on every division
  // boundary (hard-syncs the analog LFO). Stretched to >=2 ms: our loop pass
  // is far shorter than the original's 3.5 ms, so a 1-pass pulse would be
  // too narrow for the LFO reset circuit.
  if (out.clockReset || (midiclk::SyncOn() && divEdge))
    g_rstUntilMs = millis() + 2;
#ifdef DEBUG
  // Bench diagnostics on USB serial (app-debug build only): raw pin states
  // every 500 ms, so wiring faults are visible without a scope.
  // Only when a terminal is open (DTR): an unopened host connection makes
  // Serial writes stall the loop, and a stalled loop lets the S/H caps
  // droop — audible as rhythmic pitch/gate artifacts at the print rate.
  static uint32_t s_dbgNext = 0;
  if (Serial && millis() >= s_dbgNext) {
    s_dbgNext = millis() + 500;
    Serial.print(F("PIND=0x"));  Serial.print(PIND, HEX);
    Serial.print(F(" T0="));     Serial.print((PIND >> T0_HOLD_BIT) & 1);
    Serial.print(F(" btnraw=0x")); Serial.print(g_buttons.now, HEX);
    Serial.print(F(" hold="));   Serial.print(g_buttons.hold);
    Serial.print(F(" keys=0x")); Serial.println(keys, HEX);
  }
#endif
  // LED drive (SH101.asm:90-128): the two direct LEDs have OPPOSITE
  // polarities — HOLD is lit by P25=0, KEY-TRANSPOSE by P26=1 (extra
  // inverter TR10). Function-LED select 0 = all off, 1-5 = exactly one of
  // LOAD/PLAY/DOWN/U&D/UP. Idle = everything off: select 0, P25=1, P26=0.
  uint8_t p2 = (uint8_t)(1u << P2_LED_HOLD);
  if (g_buttons.hold)  p2 &= (uint8_t)~(1u << P2_LED_HOLD);  // 0 -> HOLD lit
  if (!out.gate)       p2 |= (uint8_t)(1u << P2_GATE);      // bit=1 -> gate OFF
  if ((int32_t)(g_rstUntilMs - millis()) > 0)
    p2 |= (uint8_t)(1u << P2_CLOCK_RST);
  // TODO: arp/seq mode on the function-LED select (1-5), KT LED (P26=1) when
  // KEY-TRANSPOSE lands.
  hw::P2Write(p2);
}
