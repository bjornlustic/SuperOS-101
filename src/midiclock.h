// SuperOS-101 - MIDI clock sync: realtime parsing, clock divider
//
// Fed by the MIDI driver (midi.h) with the realtime bytes of the DIN stream and
// derives a musical clock from the 24-ppqn realtime stream. When sync is ON
// (config menu / SysEx 0x22), the divided clock replaces the LFO/EXT-CLK as
// the arp/sequencer clock, pulses CLOCK RESET on every division boundary
// (hard-syncing the analog LFO's triangle/square), and gates the random S/H
// so it steps on the beat. In 80C49 mode the divided clock is injected on the
// emulated T1 pin instead.
//
// All divisions are integer tick counts at 24 ppqn, so there is no drift.
#pragma once
#include <Arduino.h>
#include <avr/eeprom.h>
#include "combined.h"

namespace midiclk {

// Division table, indexed by the 10 lowest keyboard keys in the config menu:
//   0 1/4   1 1/4d  2 1/4t  3 1/8   4 1/8d  5 1/8t  6 1/16  7 1/16d 8 1/16t 9 1/32
static const uint8_t DIV_COUNT = 10;
static const uint8_t DIV_TICKS[DIV_COUNT] = {24, 36, 16, 12, 18, 8, 6, 9, 4, 3};
static const uint8_t DIV_DEFAULT = 6;   // 1/16

struct State {
  bool    sync    = false;
  uint8_t divIdx  = DIV_DEFAULT;
  bool    running = false;
  uint8_t tick    = 0;      // 0..ticks-1 within the current division
  bool    edge    = false;  // division boundary since last Consume
  bool    half    = false;  // half-division point since last Consume (gate off)
  bool    started = false;  // 0xFA received since last Consume
  bool    stopped = false;  // 0xFC received since last Consume
};
inline State& S() { static State s; return s; }

// --- settings (EEPROM, see combined.h map) ---------------------------------
inline void Save() {
  eeprom_update_byte(EE_CFG_MAGIC, EE_CFG_MAGIC_VAL);
  eeprom_update_byte(EE_CFG_FLAGS, S().sync ? 1 : 0);
  eeprom_update_byte(EE_CFG_DIV, S().divIdx);
}

inline void Begin() {
  if (eeprom_read_byte(EE_CFG_MAGIC) == EE_CFG_MAGIC_VAL) {
    S().sync = (eeprom_read_byte(EE_CFG_FLAGS) & 1) != 0;
    uint8_t d = eeprom_read_byte(EE_CFG_DIV);
    S().divIdx = (d < DIV_COUNT) ? d : DIV_DEFAULT;
  }
}

inline bool    SyncOn() { return S().sync; }
inline uint8_t DivIdx() { return S().divIdx; }
inline void SetSync(bool on) { S().sync = on; Save(); }
inline void SetDiv(uint8_t i) {
  if (i < DIV_COUNT) { S().divIdx = i; S().tick = 0; Save(); }
}

// --- realtime bytes (from midi::Handlers.realtime) --------------------------
// The first 0xF8 after Start is the downbeat (MIDI spec), so the edge fires
// on tick 0 of each division.
inline void OnRealtime(uint8_t b) {
  switch (b) {
    case 0xF8:
      if (S().running) {
        if (S().tick == 0) S().edge = true;
        if (S().tick == DIV_TICKS[S().divIdx] / 2) S().half = true;
        if (++S().tick >= DIV_TICKS[S().divIdx]) S().tick = 0;
      }
      break;
    case 0xFA: S().running = true; S().tick = 0; S().started = true; break;
    case 0xFB: S().running = true; break;
    case 0xFC: S().running = false; S().stopped = true; break;
    default: break;
  }
}

inline bool ConsumeEdge()    { bool e = S().edge;    S().edge = false;    return e; }
inline bool ConsumeHalfEdge(){ bool h = S().half;    S().half = false;    return h; }
inline bool ConsumeStart()   { bool s = S().started; S().started = false; return s; }
inline bool ConsumeStop()    { bool s = S().stopped; S().stopped = false; return s; }
inline bool Running()        { return S().running; }

// Divided clock as a T1 level for the 80C49 emulator: low in the first half
// of the division, high in the second, so the ROM's H->L trigger lands
// exactly on the division boundary.
inline bool T1Level() {
  return (uint16_t)S().tick * 2 >= DIV_TICKS[S().divIdx];
}

} // namespace midiclk
