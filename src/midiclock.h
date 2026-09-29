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
// Sync ON is STRICT transport slaving: the host's Start starts the sequencer,
// Stop stops it, and only the divided clock ever steps it. A synced machine
// with no clock on the wire stays silent (a local PLAY press arms it, and it
// starts the moment the host's clock speaks). The LFO/EXT-CLK steps the
// machine only with sync OFF.
//
// All divisions are integer tick counts at 24 ppqn, so there is no drift.
#pragma once
#include <Arduino.h>
#include <avr/eeprom.h>
#include "combined.h"
#include "uclock.h"

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

static const uint8_t SYNC_DELAY_MAX = 60;   // ms; 16 queued ticks cover it past 300 BPM

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
  // running defaults TRUE: a host that streams 0xF8 without ever sending
  // Start (common over USB, where a Start can be lost to re-enumeration)
  // still clocks the machine. An explicit 0xFC is honoured until the next
  // 0xFA/0xFB; toggling sync re-arms it.
  bool    running = true;
  uint8_t tick    = 0;      // 0..ticks-1 within the current division
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
  // Clock edges pending for the engine, set when a SMOOTHED tick crosses a
  // boundary (Service), consumed once each by the main loop.
  bool edge = false, half = false, lfoEdge = false;
  bool tickEv = false;      // a tick fired this pass (ratchets); lastTick = its index in the division
  uint8_t lastTick = 0;
  uint8_t pending = 0;      // ticks uclk released, not yet into the divider
  // Ticks fired since Start (or since the stream / a local PLAY anchored it).
  // Every division phase is DERIVED from it (tick = abs % division), so
  // changing the clock division or the LFO division mid-play never moves the
  // grid: the next boundary is still on the host's beat.
  uint32_t abs = 0;
  // Sync delay: every released tick waits delayMs before it reaches the
  // divider, so the whole slaved machine (steps, the downbeat after Start,
  // LFO resets, the 80C49's T1) plays later by a fixed amount. Trims the 101
  // against gear whose sound path is slower (a mixer, an interface, a DAW).
  // A clock cannot be played EARLIER than it arrives, so only positive.
  uint8_t  delayMs = 0;     // 0..SYNC_DELAY_MAX
  uint32_t dq[16];          // release times + delay, in us (ring)
  uint8_t  dqHead = 0, dqN = 0;
};

inline State& S() { static State s; return s; }

// Forget owed ticks and pending edges: after a transport event or a divider
// re-anchor, nothing counted against the old phase may fire later, and the
// next 0xF8 fires on arrival (uclk seeds from the wire).
inline void DropTicks() { uclk::Reset(); S().pending = 0; S().dqN = 0; S().edge = S().half = S().lfoEdge = false; }

// --- settings (EEPROM, see combined.h map) ---------------------------------
inline void Save() {
  eeprom_update_byte(EE_CFG_MAGIC, EE_CFG_MAGIC_VAL);
  // flags bit0 = sync, bit1 = LFO sync OFF (inverted so pre-existing EEPROM
  // images, which have bit1 clear, read as the old always-synced behaviour)
  eeprom_update_byte(EE_CFG_FLAGS, (uint8_t)((S().sync ? 1 : 0) | (S().lfoSync ? 0 : 2)));
  eeprom_update_byte(EE_CFG_DIV, S().divIdx);
  eeprom_update_byte(EE_CFG_LFODIV, S().lfoDiv);
  eeprom_update_byte(EE_CFG_SYNCDLY, S().delayMs);
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
    uint8_t dl = eeprom_read_byte(EE_CFG_SYNCDLY);   // 0xFF on images from before it existed
    S().delayMs = dl <= SYNC_DELAY_MAX ? dl : 0;
  }
}

inline bool    SyncOn() { return S().sync; }
inline uint8_t DivIdx() { return S().divIdx; }
inline bool    LfoSyncOn() { return S().lfoSync; }
inline uint8_t LfoDivIdx() { return S().lfoDiv; }
inline uint8_t LfoDivEff() { return S().lfoDiv == LFO_DIV_AUTO ? S().autoIdx : S().lfoDiv; }
inline void SetSync(bool on) {
  S().sync = on; S().running = true; S().hostStopped = false; S().anchored = false;
  S().tick = 0; S().lfoTick = 0; S().abs = 0; DropTicks(); Save();
}
inline void SetDiv(uint8_t i) {
  // The grid stays put: the division's phase comes from the absolute tick
  // count, so the next boundary lands on the host's beat (no reset, no drop).
  if (i < DIV_COUNT && i != S().divIdx) { S().divIdx = i; S().tick = (uint8_t)(S().abs % DIV_TICKS[i]); Save(); }
}
inline uint8_t SyncDelayMs() { return S().delayMs; }
inline void SetSyncDelay(uint8_t ms) { if (ms <= SYNC_DELAY_MAX && ms != S().delayMs) { S().delayMs = ms; S().dqN = 0; Save(); } }
inline void SetLfoSync(bool on) { S().lfoSync = on; Save(); }
inline void SetLfoDiv(uint8_t i) {
  if (i < LFO_DIV_COUNT || i == LFO_DIV_AUTO) {
    S().lfoDiv = i; S().lfoTick = (uint8_t)(S().abs % LfoTicks(LfoDivEff()));
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
  if (Driving(nowMs) && !S().anchored) { S().tick = 0; S().lfoTick = 0; S().abs = 0; DropTicks(); }
}

// --- realtime bytes (from midi::Handlers.realtime) --------------------------
// 0xF8 only timestamps and counts the byte (uclk owes the engine one tick);
// the divider advances in Service(), when the smoothed timebase says the
// tick is due. The first 0xF8 after Start is the downbeat (MIDI spec): Start
// drops the smoother, so that tick fires on arrival as tick 0 of a division.
// running defaults true (see State): a host that streams clock without ever
// sending Start still steps the machine; only an explicit Stop silences the
// divider, until the next Start/Continue or a sync toggle.
inline void OnRealtime(uint8_t b) {
  State& s = S();
  switch (b) {
    case 0xF8: {
      uint32_t now = millis();
      if (s.lastTickMs != 0) {
        uint32_t d = now - s.lastTickMs;
        if (d > TICK_TIMEOUT_MS) {
          // A fresh stream after a dead gap: phase unknown, and it counts as
          // an implicit continue (a host that streams clock without ever
          // sending Start must still clock the machine, including after an
          // earlier 0xFC). The new stream's first tick IS a boundary: an
          // armed PLAY starts right here, on arrival.
          s.anchored = false; s.running = true; s.hostStopped = false;
          s.tick = 0; s.lfoTick = 0; s.abs = 0;
          DropTicks();
        } else if (d <= 60) s.tickMs = (uint8_t)(((uint16_t)s.tickMs * 3 + d + 2) / 4);
      }
      s.lastTickMs = now;
      if (s.running) uclk::OnClockByte(micros());
      break;
    }
    case 0xFA:
      s.running = true; s.hostStopped = false; s.anchored = true;
      s.tick = 0; s.lfoTick = 0; s.abs = 0; s.started = true;
      DropTicks();
      break;
    case 0xFB: s.running = true; s.hostStopped = false; DropTicks(); break;
    case 0xFC: s.running = false; s.hostStopped = true; s.stopped = true; DropTicks(); break;
    default: break;
  }
}

// Fire due ticks into the divider. Call once per main-loop pass (and from
// the emulator's T1 watch). A USB burst drains through uclk::Poll at once
// but reaches the divider ONE tick per call, so no boundary is ever merged
// with the next and the emulated T1 level never skips a transition.
inline void Service() {
  State& s = S();
  uint8_t& pending = s.pending;
  if (!s.running) { pending = 0; return; }
  const uint32_t nowUs = micros();
  uint8_t n = uclk::Poll(nowUs);
  if (s.delayMs) {
    // Delay line: each released tick is due delayMs later. At most one tick
    // leaves per call, like the undelayed path.
    const uint32_t d = (uint32_t)s.delayMs * 1000;
    while (n--) {
      if (s.dqN < 16) { s.dq[(uint8_t)(s.dqHead + s.dqN) & 15] = nowUs + d; ++s.dqN; }
    }
    if (!s.dqN || (int32_t)(nowUs - s.dq[s.dqHead]) < 0) return;
    s.dqHead = (uint8_t)((s.dqHead + 1) & 15); --s.dqN;
  } else {
    pending = (uint8_t)(pending + n > 250 ? 250 : pending + n);
    if (!pending) return;
    --pending;
  }
  const uint8_t ticksN = DIV_TICKS[s.divIdx];
  const uint8_t lfoN   = LfoTicks(LfoDivEff());
  s.tick    = (uint8_t)(s.abs % ticksN);
  s.lfoTick = (uint8_t)(s.abs % lfoN);
  if (s.tick == 0) s.edge = true;
  if (s.tick == ticksN / 2) s.half = true;
  if (s.lfoTick == 0) s.lfoEdge = true;
  s.lastTick = s.tick; s.tickEv = true;
  ++s.abs;
  s.tick    = (uint8_t)(s.abs % ticksN);
  s.lfoTick = (uint8_t)(s.abs % lfoN);
}

inline bool Take(bool& f)    { bool v = f; f = false; return v; }
inline bool ConsumeEdge()    { return Take(S().edge); }
inline bool ConsumeLfoEdge() { return Take(S().lfoEdge); }
inline bool ConsumeHalfEdge(){ return Take(S().half); }
inline bool ConsumeStart()   { return Take(S().started); }
inline bool ConsumeStop()    { return Take(S().stopped); }
inline bool Running()        { return S().running; }
// Divisions whose boundary has fired since Start: the sequencer's step count
// on the host's grid (Resync, division changes).
inline uint16_t DivisionsSinceStart() {
  const uint32_t a = S().abs; const uint8_t n = DIV_TICKS[S().divIdx];
  return a ? (uint16_t)((a - 1) / n + 1) : 0;
}
inline uint8_t DivTicks() { return DIV_TICKS[S().divIdx]; }
// One division in microseconds at the smoothed tempo (ratchet spacing).
inline uint32_t DivPeriodUs() { return (uint32_t)DIV_TICKS[S().divIdx] * uclk::S().perUs; }
inline bool ConsumeTick(uint8_t& idx) { if (!S().tickEv) return false; S().tickEv = false; idx = S().lastTick; return true; }

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
    s.autoIdx = best; s.autoVote = 0; s.lfoTick = (uint8_t)(s.abs % LfoTicks(best));
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
