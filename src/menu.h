// SuperOS-101 — config menu (firmware select), modeled on the 303's FUNCTION+
// CLEAR menu but driven by the SH-101's own panel.
//
// Entry: hold HOLD + KEY-TRANSPOSE for 2 s while running (either firmware), or
// at power-on. (The original ROM's Test Mode combo is LOAD + KEY-TRANSPOSE at
// power-on; ours is chosen not to collide with it, and both buttons are inert
// in normal SuperOS play.)
//
// In the menu (gate forced off, HOLD LED lit as the menu marker; the function
// LED shows the running firmware — LOAD lit = emulator, PLAY lit = SuperOS;
// KEY-TRANSP LED lit = MIDI clock sync on):
//   LOAD          -> reboot into the 80C49 emulator (original mask ROM)
//   PLAY          -> reboot into SuperOS
//   UP&DOWN       -> toggle MIDI clock sync
//   UP / DOWN     -> next / previous MIDI clock division
//   keys 1-10     -> MIDI clock division direct: 1/4 1/4d 1/4t 1/8 1/8d 1/8t
//                    1/16 1/16d 1/16t 1/32 (lowest key = 1/4)
//   key 11 / 12   -> accent model MC-202 / TB-303 (cvdac.h)
//   key 13        -> toggle MIDI filter cutoff (CC) on/off
//   key 14        -> toggle LFO hard-sync
//   keys 15-18    -> sequencer direction FWD / BWD / PING-PONG / RANDOM
//   key 19        -> pedal jack route: HOLD <-> KEY TRANSPOSE
//   keys 20-31    -> sync delay 0 2 4 6 8 10 12 15 18 21 25 30 ms
//   KEY-TRANSPOSE -> exit, resume the running firmware
#pragma once
#include <Arduino.h>
#include "pins.h"
#include "hw.h"
#include "controls.h"
#include "combined.h"
#include "emu101.h"
#include "midiclock.h"
#include "cvdac.h"
#include "engine.h"

namespace menu {

static const uint32_t COMBO_MS = 2000;

struct State {
  bool active = false;
  uint8_t prev = 0xFF;
  uint32_t prevKeys = 0xFFFFFFFF;   // all-set so entry-held keys don't edge
  uint32_t comboStart = 0;
};
inline State& S() { static State s; return s; }

inline bool Active() { return S().active; }
inline void Exit()   { S().active = false; S().comboStart = 0; }
inline void Enter()  {
  S().active = true; S().prev = 0xFF; S().prevKeys = 0xFFFFFFFF;
  S().comboStart = 0;
}

// Called every pass by whichever firmware runs, with buttons in 1=pressed form.
inline void NotifyButtons(uint8_t buttons, uint32_t nowMs) {
  const uint8_t combo = (uint8_t)((1u << controls::BTN_HOLD) | (1u << controls::BTN_KEYTRANS));
  if (buttons == combo) {   // exact: all-pressed (floating P1) must not enter
    if (!S().comboStart) S().comboStart = nowMs ? nowMs : 1;
    else if (!S().active && nowMs - S().comboStart >= COMBO_MS) Enter();
  } else {
    S().comboStart = 0;
  }
}

inline bool PowerOnComboHeld() {
  uint8_t b = hw::ScanRead(COL_BUTTONS);
  // Exact match, not a mask: a floating/unwired P1 reads all-pressed and
  // must not false-enter the menu (same guard as the bootloader's combo).
  return b == ((1u << controls::BTN_HOLD) | (1u << controls::BTN_KEYTRANS));
}

// One menu pass; call instead of the firmware loop while Active().
inline void Poll() {
  // Scan without disturbing whatever the paused firmware left on the BUS.
  uint8_t savedBus = BUS_PORT;
  uint8_t now = hw::ScanRead(COL_BUTTONS);
  uint32_t keysNow = 0;
  for (uint8_t bank = 0; bank < 4; ++bank)
    keysNow |= (uint32_t)hw::ScanRead((ScanCol)(COL_KBD_BANK0 + bank)) << (bank * 8);
  BUS_PORT = savedBus;
  uint8_t  rise    = (uint8_t)(now & ~S().prev);
  uint32_t keyRise = keysNow & ~S().prevKeys;
  S().prev = now;
  S().prevKeys = keysNow;

  // Gate off, clock reset released; HOLD LED on (P25=0 -> TR16 off -> D41 lit);
  // function LED one-hot: 1 = LOAD (emulator running), 2 = PLAY (SuperOS);
  // KEY-TRANSP LED (P26=1 -> lit, via inverter TR10; HOLD is the opposite
  // polarity, P25=1 -> off) = MIDI clock sync on. Function LED 1 (LOAD) =
  // emulator selected, 2 (PLAY) = SuperOS.
  uint8_t p2 = (uint8_t)(P2_IDLE & ~(1u << P2_LED_HOLD));   // HOLD LED lit
  p2 |= (combined_current_fw() == FW_EMU) ? 1 : 2;
  if (midiclk::SyncOn()) p2 |= (uint8_t)(1u << P2_LED_KTRANS);
  hw::P2Write(p2);

  // MIDI clock sync: UP&DOWN toggles; UP/DOWN cycle the division; the 10
  // lowest keys select it directly (table in midiclock.h).
  if (rise & (1u << controls::BTN_UD)) midiclk::SetSync(!midiclk::SyncOn());
  if (rise & (1u << controls::BTN_UP))
    midiclk::SetDiv((uint8_t)((midiclk::DivIdx() + 1) % midiclk::DIV_COUNT));
  if (rise & (1u << controls::BTN_DOWN))
    midiclk::SetDiv((uint8_t)((midiclk::DivIdx() + midiclk::DIV_COUNT - 1) % midiclk::DIV_COUNT));
  for (uint8_t k = 0; k < midiclk::DIV_COUNT; ++k)
    if (keyRise & (1UL << k)) midiclk::SetDiv(k);
  // CV DAC: keys 11/12 pick the accent model, key 13 toggles MIDI cutoff.
  if (keyRise & (1UL << 10)) cvdac::SetMode(cvdac::MODE_MC202);
  if (keyRise & (1UL << 11)) cvdac::SetMode(cvdac::MODE_TB303);
  if (keyRise & (1UL << 12)) cvdac::SetCutoffEnabled(!(cvdac::Get().flags & cvdac::F_MIDI_CUTOFF));
  // Key 14: toggle LFO hard-sync (with MIDI sync on: OFF = the analog LFO
  // free-runs for modulation while the sequencer still steps from the clock;
  // ON = CLOCK RESET locks it to the clock at its own division, editor 0x22).
  if (keyRise & (1UL << 13)) midiclk::SetLfoSync(!midiclk::LfoSyncOn());
  // Keys 15-18: sequencer direction FWD / BWD / PING-PONG / RANDOM; key 19
  // toggles the pedal route HOLD <-> KEY TRANSPOSE (SuperOS; engine.h).
  for (uint8_t k = 0; k < engine::DIR_COUNT; ++k)
    if (keyRise & (1UL << (14 + k))) { engine::DirSetting() = k; engine::SaveFeatures(); }
  // Keys 20-31: sync delay (the slaved machine plays this many ms later).
  static const uint8_t DLY[12] = {0, 2, 4, 6, 8, 10, 12, 15, 18, 21, 25, 30};
  for (uint8_t k = 0; k < 12; ++k)
    if (keyRise & (1UL << (19 + k))) midiclk::SetSyncDelay(DLY[k]);
  if (keyRise & (1UL << 18)) {
    controls::PedalRoute() = controls::PedalRoute() == controls::PEDAL_HOLD ? controls::PEDAL_TRANSPOSE : controls::PEDAL_HOLD;
    engine::SaveFeatures();
  }

  if (rise & (1u << controls::BTN_LOAD)) {
    if (emu101::available()) SwitchTo(FW_EMU);   // no ROM embedded -> ignore
  }
  if (rise & (1u << controls::BTN_PLAY))     SwitchTo(FW_SUPEROS);
  if (rise & (1u << controls::BTN_KEYTRANS)) Exit();
}

} // namespace menu
