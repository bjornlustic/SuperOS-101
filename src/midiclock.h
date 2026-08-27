// SuperOS-101 - MIDI clock sync: realtime parsing, clock divider
//
// Fed by the MIDI driver (midi.h) with the realtime bytes of the DIN stream and
// derives a musical clock from the 24-ppqn realtime stream. When sync is ON
// (config menu / SysEx 0x22) AND clock bytes are actually arriving, the divided
// clock replaces the LFO/EXT-CLK as the arp/sequencer clock, pulses CLOCK RESET
// on every division boundary (hard-syncing the analog LFO's triangle/square),
// and gates the random S/H so it steps on the beat. In 80C49 mode the divided
// clock is injected on the emulated T1 pin instead.
//
// Sync ON with NO clock on the wire falls back to the LFO/EXT-CLK, so a PLAY
// press always plays: "sync on" means "the host's clock wins when it is
// speaking", not "the machine is mute until a host shows up". The one
// exception is an explicit 0xFC: the host said stop, so the fallback stays
// silent until the next 0xFA/0xFB or until the player presses PLAY here
// (UserPlay), which is fresh local intent and overrides the host's stop.
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
// The LFO's own division reaches below the sequencer's table: 10 = 1/2 (48
// ticks), 11 = 1/1 (96). And 127 = AUTO: the division follows the LFO RATE
// slider (the measured free-running LFO period picks the nearest division).
static const uint8_t LFO_DIV_COUNT = 12;
static const uint8_t LFO_DIV_AUTO  = 127;
inline uint8_t LfoTicks(uint8_t i) {
  if (i < DIV_COUNT) return DIV_TICKS[i];
  return i == 10 ? 48 : i == 11 ? 96 : 24;
}

// A 0xF8 gap longer than this means the host's clock is gone (one tick at
// 10 BPM is 250 ms; any real tempo ticks far faster).
static const uint16_t TICK_TIMEOUT_MS = 300;

struct State {
  bool    sync    = false;
  uint8_t divIdx  = DIV_DEFAULT;
  // LFO hard-sync: with sync ON, pulse CLOCK RESET on lfoDiv boundaries so
  // the analog LFO locks to the clock at a musical rate (its own division,
  // independent of the sequencer's). Toggle OFF to leave the LFO free-running
  // for modulation while the sequencer still steps from the divided clock.
  bool    lfoSync = true;
  uint8_t lfoDiv  = 0;      // 1/4 by default; LFO_DIV_AUTO = follow the slider
  uint8_t lfoTick = 0;
  bool    lfoEdge = false;
  // running defaults TRUE: a host that streams 0xF8 without ever sending
  // Start (common over USB, where a Start can be lost to re-enumeration)
  // still clocks the machine. An explicit 0xFC is honoured until the next
  // 0xFA/0xFB; toggling sync re-arms it.
  bool    running = true;
  uint8_t tick    = 0;      // 0..ticks-1 within the current division
  bool    edge    = false;  // division boundary since last Consume
  bool    half    = false;  // half-division point since last Consume (gate off)
  bool    started = false;  // 0xFA received since last Consume
  bool    stopped = false;  // 0xFC received since last Consume
  // Liveness + phase anchoring
  uint32_t lastTickMs = 0;  // millis of the last 0xF8 (0 = never seen)
  uint8_t  tickMs     = 21; // EMA of 0xF8 spacing (21 ms ~ 120 BPM)
  bool     hostStopped = false; // 0xFC seen; cleared by 0xFA/0xFB/UserPlay
  bool     anchored    = false; // 0xFA seen since this clock stream came alive
  // AUTO LFO division: measured free-running LFO period and the chosen index
  uint16_t lfoPerMs = 0;    // EMA of clean T1 fall-to-fall intervals (0 = none)
  uint32_t lfoMeasMs = 0;   // millis of the last clean sample
  uint8_t  autoIdx  = 0;    // division AUTO currently drives
  uint8_t  autoVote = 0;    // consecutive samples agreeing on a different index
  uint8_t  autoWant = 0;    // the index those samples voted for
};
inline State& S() { static State s; return s; }

// --- settings (EEPROM, see combined.h map) ---------------------------------
inline void Save() {
  eeprom_update_byte(EE_CFG_MAGIC, EE_CFG_MAGIC_VAL);
  // flags bit0 = sync, bit1 = LFO sync OFF (inverted so pre-existing EEPROM
  // images, which have bit1 clear, read as the old always-synced behaviour)
  eeprom_update_byte(EE_CFG_FLAGS, (uint8_t)((S().sync ? 1 : 0) | (S().lfoSync ? 0 : 2)));
  eeprom_update_byte(EE_CFG_DIV, S().divIdx);
  eeprom_update_byte(EE_CFG_LFODIV, S().lfoDiv);
}

inline void Begin() {
  if (eeprom_read_byte(EE_CFG_MAGIC) == EE_CFG_MAGIC_VAL) {
    uint8_t f = eeprom_read_byte(EE_CFG_FLAGS);
    S().sync = (f & 1) != 0;
    S().lfoSync = (f & 2) == 0;
    uint8_t d = eeprom_read_byte(EE_CFG_DIV);
    S().divIdx = (d < DIV_COUNT) ? d : DIV_DEFAULT;
    uint8_t l = eeprom_read_byte(EE_CFG_LFODIV);
    S().lfoDiv = (l < LFO_DIV_COUNT || l == LFO_DIV_AUTO) ? l : 0;
  }
}

inline bool    SyncOn() { return S().sync; }
inline uint8_t DivIdx() { return S().divIdx; }
inline bool    LfoSyncOn() { return S().lfoSync; }
inline uint8_t LfoDivIdx() { return S().lfoDiv; }
inline uint8_t LfoDivEff() { return S().lfoDiv == LFO_DIV_AUTO ? S().autoIdx : S().lfoDiv; }
inline void SetSync(bool on) {
  S().sync = on; S().running = true; S().hostStopped = false; S().anchored = false;
  S().tick = 0; S().lfoTick = 0; Save();
}
inline void SetDiv(uint8_t i) {
  if (i < DIV_COUNT) { S().divIdx = i; S().tick = 0; Save(); }
}
inline void SetLfoSync(bool on) { S().lfoSync = on; S().lfoTick = 0; Save(); }
inline void SetLfoDiv(uint8_t i) {
  if (i < LFO_DIV_COUNT || i == LFO_DIV_AUTO) {
    S().lfoDiv = i; S().lfoTick = 0;
    if (i == LFO_DIV_AUTO) { S().autoVote = 0; S().lfoPerMs = 0; }
    Save();
  }
}

// --- clock liveness ---------------------------------------------------------
inline bool TickAlive(uint32_t nowMs) {
  return S().lastTickMs != 0 && (uint32_t)(nowMs - S().lastTickMs) <= TICK_TIMEOUT_MS;
}
// The divided clock steps the machine only while the host is actually
// speaking (sync on, ticks arriving, not stopped). Everything else in the
// firmware keys stepping decisions on this, not on SyncOn().
inline bool Driving(uint32_t nowMs) {
  return S().sync && S().running && TickAlive(nowMs);
}
// Sync is configured on but the host is silent or stopped: the LFO/EXT-CLK
// steps the machine instead, EXCEPT after an explicit 0xFC (host said stop),
// which stays silent until 0xFA/0xFB or a local PLAY press.
inline bool FallbackToLfo(uint32_t nowMs) {
  return S().sync && !Driving(nowMs) && !S().hostStopped;
}
// A local PLAY press (panel or SysEx 0x1E): fresh intent, overrides a host
// stop. If a clock is alive but the host never sent 0xFA (free-streaming
// clock, phase meaningless), re-anchor the divider so step 1 lands on the
// very next tick and the groove starts at the button. An 0xFA-anchored
// stream keeps its phase: the press arms and step 1 lands on the host's own
// next division boundary.
inline void UserPlay(uint32_t nowMs) {
  S().hostStopped = false;
  if (!TickAlive(nowMs)) S().anchored = false;
  else S().running = true;   // ticks are live: lock to them, even after an 0xFC
  if (Driving(nowMs) && !S().anchored) { S().tick = 0; S().lfoTick = 0; }
}

// --- realtime bytes (from midi::Handlers.realtime) --------------------------
// The first 0xF8 after Start is the downbeat (MIDI spec), so the edge fires
// on tick 0 of each division. running defaults true (see State): a host that
// streams clock without ever sending Start still steps the machine; only an
// explicit Stop silences the divider, until the next Start/Continue or a
// sync toggle. (Gating ticks on a Start-only running flag left the machine
// silent with sync ON in both firmwares whenever the Start was never seen;
// the 80C49 path pins T1Level() to a constant when tick never moves.)
inline void OnRealtime(uint8_t b) {
  switch (b) {
    case 0xF8: {
      uint32_t now = millis();
      if (S().lastTickMs != 0) {
        uint32_t d = now - S().lastTickMs;
        if (d > TICK_TIMEOUT_MS) {
          // A fresh stream after a dead gap: phase unknown, and it counts as
          // an implicit continue. A host that streams clock without ever
          // sending Start must still clock the machine, including after an
          // earlier 0xFC ended its previous stream.
          S().anchored = false;
          S().running = true;
          S().hostStopped = false;
        } else if (d <= 60) S().tickMs = (uint8_t)(((uint16_t)S().tickMs * 3 + d + 2) / 4);
      }
      S().lastTickMs = now;
      if (S().running) {
        if (S().tick == 0) S().edge = true;
        if (S().tick == DIV_TICKS[S().divIdx] / 2) S().half = true;
        if (++S().tick >= DIV_TICKS[S().divIdx]) S().tick = 0;
        if (S().lfoTick == 0) S().lfoEdge = true;
        if (++S().lfoTick >= LfoTicks(LfoDivEff())) S().lfoTick = 0;
      }
      break;
    }
    case 0xFA:
      S().running = true; S().hostStopped = false; S().anchored = true;
      S().tick = 0; S().lfoTick = 0; S().started = true;
      break;
    case 0xFB: S().running = true; S().hostStopped = false; break;
    case 0xFC: S().running = false; S().hostStopped = true; S().stopped = true; break;
    default: break;
  }
}

inline bool ConsumeEdge()    { bool e = S().edge;    S().edge = false;    return e; }
inline bool ConsumeLfoEdge() { bool e = S().lfoEdge; S().lfoEdge = false; return e; }
inline bool ConsumeHalfEdge(){ bool h = S().half;    S().half = false;    return h; }
inline bool ConsumeStart()   { bool s = S().started; S().started = false; return s; }
inline bool ConsumeStop()    { bool s = S().stopped; S().stopped = false; return s; }
inline bool Running()        { return S().running; }

// --- AUTO LFO division (lfoDiv == LFO_DIV_AUTO) -----------------------------
// The main loop feeds clean free-running T1 fall-to-fall intervals here (an
// interval is clean when no CLOCK-RESET pulse fired inside it: a hard-synced
// LFO's forced transitions measure the reset rate, not the slider). The
// nearest division of the incoming clock wins, with a 2-sample vote so a
// single noisy interval cannot flap the division.
inline void LfoPeriodSample(uint16_t ms, uint32_t nowMs) {
  State& s = S();
  if (ms < 10 || ms > 30000) return;
  s.lfoPerMs = s.lfoPerMs ? (uint16_t)(((uint32_t)s.lfoPerMs * 3 + ms) / 4) : ms;
  s.lfoMeasMs = nowMs;
  if (s.lfoDiv != LFO_DIV_AUTO) return;
  // nearest division by period against the measured tick spacing
  uint8_t best = 0; uint32_t bestErr = 0xFFFFFFFF;
  for (uint8_t i = 0; i < LFO_DIV_COUNT; ++i) {
    uint32_t p = (uint32_t)LfoTicks(i) * s.tickMs;
    uint32_t e = p > s.lfoPerMs ? p - s.lfoPerMs : s.lfoPerMs - p;
    if (e < bestErr) { bestErr = e; best = i; }
  }
  if (best == s.autoIdx) { s.autoVote = 0; return; }
  if (s.autoWant == best && ++s.autoVote >= 2) {
    s.autoIdx = best; s.autoVote = 0; s.lfoTick = 0;
  } else if (s.autoWant != best) { s.autoWant = best; s.autoVote = 1; }
}
// AUTO needs to hear the slider: when hard-sync has starved the measurement
// (no clean interval for two of the chosen division's periods), skip one
// reset pulse so the LFO free-runs for a division and a clean interval lands.
inline bool AutoListenSkip(uint32_t nowMs) {
  State& s = S();
  if (s.lfoDiv != LFO_DIV_AUTO) return false;
  uint32_t divMs = (uint32_t)LfoTicks(s.autoIdx) * s.tickMs;
  return s.lfoMeasMs == 0 || (uint32_t)(nowMs - s.lfoMeasMs) > divMs * 2;
}
inline uint16_t LfoPeriodMs() { return S().lfoPerMs; }

// Divided clock as a T1 level for the 80C49 emulator: low in the first half
// of the division, high in the second, so the ROM's H->L trigger lands
// exactly on the division boundary.
inline bool T1Level() {
  return (uint16_t)S().tick * 2 >= DIV_TICKS[S().divIdx];
}

} // namespace midiclk
