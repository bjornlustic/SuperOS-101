// SuperOS-101 — front-panel decode layer
//
// Panel switches (RANGE / OCTAVE-TRANSPOSE / ADSR) and the function buttons.
// Ported from SH101.asm: step 3 (range/oct/adsr, asm:352-449) and the function-
// switch read (asm:833-931). The switch reads use the RAW active-low domain
// (a closed switch = 0) to match the original logic 1:1.
#pragma once
#include <Arduino.h>
#include "pins.h"
#include "hw.h"

namespace controls {

// --- RANGE / OCTAVE / ADSR (drive COL_SWITCHES, read P1 raw) ----------------
// Raw P1 bit meaning at COL_SWITCHES (SH101.asm:357-365, 0 = switch active):
//   d0 S3B RANGE16'/4'  d1 S3A RANGE16'/8'  d3 ADSR LFO  d4 ADSR GATE+TRIG
//   d5 OCT UP           d6 OCT DOWN
struct Panel {
  uint8_t rangeOffset;      // octave base CV: 16'=0x0C 8'=0x18 4'=0x24 2'=0x30
  uint8_t octaveOffset;     // transpose: DOWN=0x00 MIDDLE=0x0C UP=0x18
  bool    adsrLfo;          // S7 = LFO  (env retriggered by LFO)
  bool    lastNotePriority; // S7 = GATE+TRIG -> last-note priority (else lowest)
};

inline Panel ReadPanel() {
  uint8_t raw = hw::ScanReadRaw(COL_SWITCHES);
  Panel p;

  // ADSR mode / key priority (asm:367-373)
  p.lastNotePriority = !(raw & (1u << 4)); // d4 active (GATE+TRIG) -> last-note
  p.adsrLfo          = !(raw & (1u << 3)); // d3 active -> LFO trigger

  // RANGE (asm:391-413): start 0x30; d0 active -12, d1 active -24
  uint8_t r5 = 0x30;
  if (!(raw & (1u << 0))) r5 -= 12;
  if (!(raw & (1u << 1))) r5 -= 24;
  p.rangeOffset = r5;

  // OCTAVE TRANSPOSE (asm:420-435)
  if      (raw & (1u << 6)) p.octaveOffset = 0x18; // UP
  else if (raw & (1u << 5)) p.octaveOffset = 0x00; // DOWN
  else                      p.octaveOffset = 0x0C; // MIDDLE
  return p;
}

// --- Function buttons (drive COL_BUTTONS, read P1) --------------------------
// P1 bit -> button at COL_BUTTONS (SH101.asm:69-76, here in 1=pressed domain):
//   P10 LOAD  P11 PLAY  P12 DOWN  P13 U&D  P14 UP  P15 HOLD  P17 KEY-TRANSPOSE
enum Button : uint8_t {
  BTN_LOAD = 0, BTN_PLAY = 1, BTN_DOWN = 2, BTN_UD = 3,
  BTN_UP = 4, BTN_HOLD = 5, BTN_KEYTRANS = 7,
};

struct Buttons {
  uint8_t now  = 0;   // 1 = currently pressed
  uint8_t prev = 0;
  bool    hold = false; // panel HOLD OR external pedal (T0)

  void Scan() {
    prev = now;
    now  = hw::ScanRead(COL_BUTTONS);                 // 1 = pressed
    hold = (now & (1u << BTN_HOLD)) || hw::HoldRequested();
  }
  bool down(Button b)    const { return (now  >> b) & 1; }              // held
  bool pressed(Button b) const { return ((now & ~prev) >> b) & 1; }     // rising edge
  bool released(Button b) const { return ((prev & ~now) >> b) & 1; }    // falling edge
};

} // namespace controls
