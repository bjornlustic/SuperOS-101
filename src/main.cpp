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
  uint8_t p2 = 0x00;
  if (!out.gate)       p2 |= (uint8_t)(1u << P2_GATE);      // bit=1 -> gate OFF
  if ((int32_t)(g_rstUntilMs - millis()) > 0)
    p2 |= (uint8_t)(1u << P2_CLOCK_RST);
  // TODO: function-LED one-hot via IC8 4552 (P20–P22) + HOLD/KEY-TRANSP LEDs.
  hw::P2Write(p2);
}
