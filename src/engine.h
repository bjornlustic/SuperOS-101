// SuperOS-101 - note engine: keyboard / arpeggiator / sequencer
//
// Clean-room behavioural implementation matching the original's OBSERVABLE
// behaviour (this is our own firmware, see docs/FIRMWARE.md). It is NOT a
// transliteration of the 8049 code; where exact original semantics are
// uncertain (mode latching, tie/rest edge cases) it's marked TODO/verify.
//
//   - portamento AUTO: PORTAMENTO OFF (P23) is pulsed on every non-legato note
//                (fresh key from silence, arp step, sequencer note step) and
//                left alone on legato notes (new key while one is held, legato
//                steps), so with the switch on AUTO only legato notes glide
//   - keyboard : mono, lowest- or last-note priority (panel-selected); MIDI
//                notes in the keyboard's range act as keys, notes outside it
//                play as a mono override in keyboard mode
//   - hold     : HOLD button / pedal latches the held keys (arp keeps running)
//   - arp      : UP / DOWN / UP&DOWN over held keys, advanced by the clock
//   - sequencer: 8 patterns x up to 128 steps (note / rest / legato), kept in RAM
//                and persisted in the flash arena (flash_persist.h); pattern
//                select = PLAY held + one of the 8 lowest keys, or SysEx 0x1D
#pragma once
#include <Arduino.h>
#include "controls.h"
#include "combined.h"
#include "flash_persist.h"
#include "midiclock.h"
#include <avr/eeprom.h>

namespace engine {

static const uint8_t MAX_STEPS    = 128;
static const uint8_t NUM_PATTERNS = persist::NUM_PATTERNS;   // 8
static const uint8_t CV_BASE      = 5;   // cv_dac_val returns value+5 (SH101.asm:992)
static const uint8_t CV_MAX       = 63;  // 6-bit DAC
// MIDI note <-> CV: key 0 (F3) at the middle octave (0x0C) is MIDI 53, so
// cv = note - 36 (MIDI 36..99 covers the whole 6-bit DAC range).
static const uint8_t MIDI_CV_OFFSET = 36;

enum Mode : uint8_t {
  MODE_KEYBOARD, MODE_ARP_UP, MODE_ARP_DOWN, MODE_ARP_UPDOWN, MODE_SEQ_PLAY, MODE_SEQ_LOAD,
  MODE_ARP_RANDOM,   // U&D pressed again while in U&D (U&D LED blinks)
};
inline bool IsArp(Mode m) {
  return m == MODE_ARP_UP || m == MODE_ARP_DOWN || m == MODE_ARP_UPDOWN || m == MODE_ARP_RANDOM;
}
// Sequencer play direction (global setting, EEPROM, config menu keys 15-18,
// PLAY held + UP / DOWN / U&D, SysEx 0x65 id 0).
enum SeqDir : uint8_t { DIR_FWD = 0, DIR_BWD = 1, DIR_PINGPONG = 2, DIR_RANDOM = 3, DIR_COUNT = 4 };
// Tiny shared PRNG for the random arp / random direction (xorshift16).
inline uint8_t Rnd8() { static uint16_t st = 0xACE1; st ^= st << 7; st ^= st >> 9; st ^= st << 8; return (uint8_t)st; }
// Persistent performance settings (EEPROM 0x020.., combined.h map): the
// sequencer direction and the pedal route. Written only when not clocked
// (an EEPROM byte write stalls ~3.4 ms; main.cpp defers it).
inline uint8_t& DirSetting() { static uint8_t d = DIR_FWD; return d; }
inline bool& FeatDirty() { static bool d = false; return d; }
inline void LoadFeatures() {
  if (eeprom_read_byte((uint8_t*)EE_A_FEAT_MAGIC) != EE_FEAT_MAGIC_VAL) return;
  uint8_t d = eeprom_read_byte((uint8_t*)EE_A_FEAT_DIR);
  DirSetting() = d < DIR_COUNT ? d : DIR_FWD;
  uint8_t pm = eeprom_read_byte((uint8_t*)EE_A_FEAT_PEDAL);
  controls::PedalRoute() = pm <= controls::PEDAL_TRANSPOSE ? pm : controls::PEDAL_HOLD;
}
inline void SaveFeatures() {
  eeprom_update_byte((uint8_t*)EE_A_FEAT_MAGIC, EE_FEAT_MAGIC_VAL);
  eeprom_update_byte((uint8_t*)EE_A_FEAT_DIR, DirSetting());
  eeprom_update_byte((uint8_t*)EE_A_FEAT_PEDAL, controls::PedalRoute());
  FeatDirty() = false;
}
// KEY TRANSPOSE reference: key 7 (C4, MIDI 60) plays the sequence as written.
static const uint8_t TRANSPOSE_REF_KEY = 7;
// Legato (2): this step's note starts WITHOUT the gate dropping after the
// previous step: no envelope retrigger, and (portamento AUTO) a glide into it.
enum StepType : uint8_t { STEP_NOTE = 0, STEP_REST = 1, STEP_LEGATO = 2 };

// Edit lanes, shared with the fleet's edit opcodes (0x16 SET STEP, 0x2E
// RANDOMIZE). Lanes 1 and 2 are accent and slide and DO NOT EXIST on the 101:
// it is mono. They are named here so a refusal is a deliberate decision at a
// named lane rather than a default case swallowing an unknown byte.
static const uint8_t LANE_NOTE   = 0;
static const uint8_t LANE_ACCENT = 1;   // absent on the 101 -> ack 2
static const uint8_t LANE_SLIDE  = 2;   // absent on the 101 -> ack 2
static const uint8_t LANE_GATE   = 3;
static const uint8_t LANE_ALL    = 0x7F;

// One step byte, the SH-101's own model: 6-bit CV + 2 type bits.
inline uint8_t  StepByte(uint8_t cv, StepType t) { return (uint8_t)((cv & 0x3F) | ((uint8_t)t << 6)); }
inline uint8_t  StepCv  (uint8_t s) { return s & 0x3F; }
inline StepType StepKind(uint8_t s) { uint8_t t = s >> 6; return t > 2 ? STEP_NOTE : (StepType)t; }

struct Output { uint8_t cv; bool gate; bool clockReset; bool portaOff; };

// --- held-key (32-bit) mask helpers -----------------------------------------
inline int8_t lowestKey (uint32_t m) { for (uint8_t k = 0; k < 32; ++k) if (m & (1UL << k)) return (int8_t)k; return -1; }
inline int8_t highestKey(uint32_t m) { for (int8_t k = 31; k >= 0; --k) if (m & (1UL << k)) return k; return -1; }
inline int8_t keyAbove  (uint32_t m, int8_t k) { for (int8_t i = k + 1; i < 32; ++i) if (m & (1UL << i)) return i; return -1; }
inline int8_t keyBelow  (uint32_t m, int8_t k) { for (int8_t i = k - 1; i >= 0; --i) if (m & (1UL << i)) return i; return -1; }

// --- arpeggiator ------------------------------------------------------------
class Arp {
  int8_t cur = -1;
  int8_t dir = 1;
 public:
  bool gate = false;   // on at each step, off at the half-step
  int8_t current() const { return cur; }
  void reset() { cur = -1; dir = 1; gate = false; }
  int8_t step(uint32_t mask, Mode mode) {
    if (!mask) { cur = -1; return -1; }
    if (cur < 0 || !(mask & (1UL << cur))) {        // (re)start
      dir = (mode == MODE_ARP_DOWN) ? -1 : 1;
      cur = (mode == MODE_ARP_DOWN) ? highestKey(mask) : lowestKey(mask);
      return cur;
    }
    int8_t nxt;
    if (mode == MODE_ARP_RANDOM) {
      // A random held key, never the same one twice in a row when there is
      // a choice (a repeat reads as a dropped step).
      uint8_t n = 0; for (uint8_t k = 0; k < 32; ++k) if (mask & (1UL << k)) ++n;
      uint8_t pick = (uint8_t)(Rnd8() % n);
      nxt = -1;
      for (int8_t k = 0; k < 32; ++k) if (mask & (1UL << k)) { if (pick-- == 0) { nxt = k; break; } }
      if (nxt == cur && n > 1) { nxt = keyAbove(mask, cur); if (nxt < 0) nxt = lowestKey(mask); }
    } else if (mode == MODE_ARP_UP) {
      nxt = keyAbove(mask, cur); if (nxt < 0) nxt = lowestKey(mask);
    } else if (mode == MODE_ARP_DOWN) {
      nxt = keyBelow(mask, cur); if (nxt < 0) nxt = highestKey(mask);
    } else {                                         // UP&DOWN
      if (dir > 0) { nxt = keyAbove(mask, cur); if (nxt < 0) { dir = -1; nxt = keyBelow(mask, cur); } }
      else         { nxt = keyBelow(mask, cur); if (nxt < 0) { dir =  1; nxt = keyAbove(mask, cur); } }
      if (nxt < 0) nxt = cur;
    }
    cur = nxt;
    return cur;
  }
};

// --- sequencer: 8 patterns in RAM, flash-arena backed --------------------------
class Sequencer {
  uint8_t step_[NUM_PATTERNS][MAX_STEPS];
  // Per-step ratchet, 2 bits each (0 = single hit, 1 = 2x, 2 = 3x): the step's
  // note retriggers evenly inside its clock period. Packed 4 steps/byte so the
  // whole lane costs 256 bytes, not another KB of a 2.5 KB SRAM part.
  uint8_t rat_[NUM_PATTERNS][MAX_STEPS / 4];
  static const uint8_t RAT_MAX = 2;
  static uint8_t RatBytes(uint8_t len) { return (uint8_t)((len + 3) >> 2); }
  uint8_t len_[NUM_PATTERNS];
  uint8_t dirty_ = 0;                 // bit p = pattern p differs from flash
  uint8_t cur_ = 0;                   // selected pattern
  uint8_t pos_ = 0;
  uint8_t curCv_ = CV_BASE; bool curGate_ = false;
  bool    stepChanged_ = false;
  bool    selChanged_ = false;
  uint8_t edited_ = 0;                // bit p = edited on the panel since last consume
  // Playback shape (runtime; DirSetting() persists via the engine's settings):
  int8_t  ppStep_ = 1;                // ping-pong travel direction
  bool    started_ = false;           // false: the next advance plays the FIRST step
  bool    halfSeen_ = false;          // the current step's half-step has passed
  uint8_t loopLen_ = 0;               // real-time loop length override, 0 = full pattern
  uint8_t queued_ = 0xFF;             // pattern switch waiting for the loop end
  uint8_t cycle_ = 0;                 // steps played in the current loop pass
  uint8_t rndNext_ = 0;               // RANDOM: the step the next advance plays (for look-ahead)
  uint8_t effLen() const {
    uint8_t len = len_[cur_];
    return (loopLen_ && loopLen_ < len) ? loopLen_ : len;
  }
  uint8_t cycleLen() const {
    uint8_t l = effLen();
    return (DirSetting() == DIR_PINGPONG && l > 1) ? (uint8_t)(2 * l - 2) : l;
  }
  uint8_t firstStep() const { uint8_t l = effLen(); return (DirSetting() == DIR_BWD && l) ? (uint8_t)(l - 1) : 0; }
  // The step the next advance() will play, without moving.
  uint8_t peekNext() const {
    uint8_t l = effLen();
    if (!l) return 0;
    if (!started_) return firstStep();
    if (queued_ != 0xFF && cycle_ >= cycleLen()) return 0;   // switching: unknown pattern, treat as a fresh attack
    switch (DirSetting()) {
      case DIR_BWD:      return pos_ == 0 || pos_ >= l ? (uint8_t)(l - 1) : (uint8_t)(pos_ - 1);
      case DIR_PINGPONG: {
        if (l == 1) return 0;
        int16_t n = (int16_t)pos_ + ppStep_;
        if (n >= l) n = l - 2; else if (n < 0) n = 1;
        return (uint8_t)n;
      }
      case DIR_RANDOM:   return rndNext_ < l ? rndNext_ : 0;
      default:           return (uint8_t)(pos_ + 1 >= l ? 0 : pos_ + 1);
    }
  }
 public:
  uint8_t direction() const { return DirSetting(); }
  void setDirection(uint8_t d) { if (d < DIR_COUNT) { DirSetting() = d; FeatDirty() = true; ppStep_ = 1; rndNext_ = (uint8_t)(Rnd8() % (effLen() ? effLen() : 1)); } }
  uint8_t loopLength() const { return loopLen_; }
  // Real-time loop length (panel HOLD + key while playing, SysEx 0x65 id 4):
  // plays only the first n steps without touching the pattern; 0 = full.
  void setLoopLength(uint8_t n) {
    loopLen_ = n;
    if (effLen() && pos_ >= effLen()) pos_ = (uint8_t)(effLen() - 1);
    if (cycle_ >= cycleLen()) cycle_ = 0;
  }
  uint8_t queued() const { return queued_; }
  // Pattern change while playing: waits for the end of the current loop pass
  // so the switch lands on the bar, like a drum machine's pattern select.
  void queue(uint8_t p) { if (p < NUM_PATTERNS) queued_ = (p == cur_) ? 0xFF : p; }
  bool halfSeen() const { return halfSeen_; }
  uint8_t nextIndex() const { return peekNext(); }
  // Real-time record: overwrite one step in place (pattern data, persisted).
  void writeStep(uint8_t idx, uint8_t stepByte) {
    if (idx >= len_[cur_]) return;
    step_[cur_][idx] = stepByte; ratchetRaw(cur_, idx, 0);
    dirty_ |= (uint8_t)(1u << cur_); stepChanged_ = true;
  }
  // The step sounding now: its gate/CV change immediately (real-time record
  // on the current step).
  void soundNow(uint8_t cv, bool gate) { curCv_ = cv; curGate_ = gate; }
  uint8_t consumeEdited() { uint8_t e = edited_; edited_ = 0; return e; }
  // Runtime state back to boot values (in-process firmware switch); pattern
  // CONTENT is not touched here, loadAll() reloads it from flash right after.
  void resetRuntime() {
    dirty_ = 0; cur_ = 0; pos_ = 0; curCv_ = CV_BASE; curGate_ = false;
    stepChanged_ = false; selChanged_ = false; edited_ = 0;
    ppStep_ = 1; started_ = false; halfSeen_ = false; loopLen_ = 0; queued_ = 0xFF; cycle_ = 0;
  }
  Sequencer() { memset(len_, 0, sizeof(len_)); }
  uint8_t current() const { return cur_; }
  uint8_t length(uint8_t p) const { return p < NUM_PATTERNS ? len_[p] : 0; }
  uint8_t length() const { return len_[cur_]; }
  const uint8_t* steps(uint8_t p) const { return step_[p < NUM_PATTERNS ? p : 0]; }
  uint8_t position() const { return pos_; }
  bool consumeStepChanged() { bool c = stepChanged_; stepChanged_ = false; return c; }

  void select(uint8_t p) {
    if (p < NUM_PATTERNS && p != cur_) { cur_ = p; loopLen_ = 0; rewind(); selChanged_ = true; }
    queued_ = 0xFF;
  }
  bool consumeSelChanged() { bool c = selChanged_; selChanged_ = false; return c; }
  void markEdited() { edited_ |= (uint8_t)(1u << cur_); }
  void clear() { len_[cur_] = 0; pos_ = 0; memset(rat_[cur_], 0, sizeof(rat_[cur_])); dirty_ |= (uint8_t)(1u << cur_); }
  uint8_t ratchet(uint8_t p, uint8_t i) const {
    return (uint8_t)((rat_[p < NUM_PATTERNS ? p : 0][i >> 2] >> ((i & 3) * 2)) & 3);
  }
  uint8_t ratchetCur() const { return ratchet(cur_, pos_); }
  void ratchetRaw(uint8_t p, uint8_t i, uint8_t v) {
    uint8_t sh = (uint8_t)((i & 3) * 2);
    rat_[p][i >> 2] = (uint8_t)((rat_[p][i >> 2] & ~(3u << sh)) | ((v & 3u) << sh));
  }
  // SysEx 0x1B SET RATCHET [pat, step, 0..2] (0 = off/1x). ack 2 when out of range.
  bool setRatchet(uint8_t p, uint8_t idx, uint8_t v) {
    if (p >= NUM_PATTERNS || idx >= len_[p] || v > RAT_MAX) return false;
    ratchetRaw(p, idx, v);
    dirty_ |= (uint8_t)(1u << p);
    if (p == cur_) stepChanged_ = true;
    return true;
  }
  // rewind so the NEXT advance() plays step 0 (advance pre-increments). A
  // pattern that STARTS with a legato step glides from the last pitch of the
  // pattern (across the loop point, 303-style), so seed the held CV from the
  // last step and start with the gate closed.
  void rewind() {
    uint8_t len = effLen();
    pos_ = len ? (uint8_t)(len - 1) : 0;
    if (len) curCv_ = StepCv(step_[cur_][len - 1]);
    curGate_ = false;
    started_ = false; halfSeen_ = false; cycle_ = 0; ppStep_ = 1;
  }
  void record(uint8_t cv, StepType t) {
    if (len_[cur_] < MAX_STEPS) {
      ratchetRaw(cur_, len_[cur_], 0);
      step_[cur_][len_[cur_]++] = StepByte(cv, t);
      dirty_ |= (uint8_t)(1u << cur_);
    }
  }
  // Panel LOAD mode: cycle the last recorded step's ratchet (UP button).
  void cycleLastRatchet(int8_t dir) {
    if (!len_[cur_]) return;
    uint8_t i = (uint8_t)(len_[cur_] - 1);
    uint8_t r = ratchet(cur_, i);
    r = (uint8_t)((r + (dir > 0 ? 1 : RAT_MAX)) % (RAT_MAX + 1));
    ratchetRaw(cur_, i, r);
    dirty_ |= (uint8_t)(1u << cur_);
  }
  // Replace pattern p wholesale (SysEx 0x12). Marks dirty; flush() persists.
  // rat: optional packed ratchet bytes after the steps (old hosts send none).
  bool set(uint8_t p, const uint8_t* steps, uint8_t len, const uint8_t* rat = nullptr, uint8_t ratBytes = 0) {
    if (p >= NUM_PATTERNS || len > MAX_STEPS) return false;
    memcpy(step_[p], steps, len);
    memset(rat_[p], 0, sizeof(rat_[p]));
    if (rat && ratBytes) memcpy(rat_[p], rat, ratBytes > RatBytes(len) ? RatBytes(len) : ratBytes);
    len_[p] = len;
    dirty_ |= (uint8_t)(1u << p);
    if (p == cur_ && pos_ >= len) rewind();
    return true;
  }
  const uint8_t* ratchets(uint8_t p) const { return rat_[p < NUM_PATTERNS ? p : 0]; }

  // Live host edits (SysEx 0x16/0x18/0x15, docs/EDIT_OPCODES.md), atomic in place
  // so an edit under clock does not rewrite the whole pattern (gate G4). The 101
  // is mono: lane 0 = note (value = MIDI note, 0 = rest), lane 3 = gate (0 normal
  // retrigger, 1 legato/tie); lanes 1/2 (accent/slide) do not exist here. Any
  // out-of-range pat/lane/step/value returns false -> SysEx ack status 2.
  bool setStep(uint8_t p, uint8_t lane, uint8_t idx, uint8_t value) {
    if (p >= NUM_PATTERNS || idx >= len_[p]) return false;   // grow with 0x18 first
    uint8_t s = step_[p][idx];
    if (lane == 0) {                                         // note field
      if (value == 0) s = StepByte(0, STEP_REST);
      else { int16_t cv = (int16_t)value - MIDI_CV_OFFSET;
             if (cv < 0) cv = 0; else if (cv > CV_MAX) cv = CV_MAX;
             s = StepByte((uint8_t)cv, STEP_NOTE); }         // a note edit clears legato
    } else if (lane == 3) {                                  // gate/legato field
      if (StepKind(s) == STEP_REST) return false;            // no gate on a rest
      s = StepByte(StepCv(s), value ? STEP_LEGATO : STEP_NOTE);
    } else return false;                                     // lanes 1/2 absent
    step_[p][idx] = s; dirty_ |= (uint8_t)(1u << p);
    if (p == cur_) stepChanged_ = true;
    return true;
  }
  bool setLength(uint8_t p, uint8_t len) {
    if (p >= NUM_PATTERNS || len < 1 || len > MAX_STEPS) return false;
    for (uint8_t i = len_[p]; i < len; ++i) step_[p][i] = StepByte(0, STEP_REST);
    len_[p] = len; dirty_ |= (uint8_t)(1u << p);
    if (p == cur_ && pos_ >= len) rewind();
    return true;
  }
  // RANDOMIZE, addressed by LANE rather than the retired `scope` byte
  // (claim-057). `scope` was unfixable because two machines read it
  // defensibly differently: 1 meant "note values only" here and "the selected
  // instrument" on the 606. A lane is the same unit the edit opcodes already
  // address, so it cannot mean two things.
  //
  //   LANE_ALL  0x7F  time + pitch (what scope 0 did)
  //   LANE_NOTE 0     pitches only, keeping the note/rest layout (scope 1)
  //   LANE_GATE 3     legato flags only, keeping pitches and the layout
  //   lanes 1, 2      accent and slide: this machine is MONO and has neither,
  //                   so they are REFUSED, not quietly reinterpreted.
  //
  // density 0 clears, 127 fills, else P(note) = density/127. seed 14-bit:
  // 0 = entropy, nonzero = deterministic (seeded xorshift) so a G4 capture
  // repeats.
  //
  // DRAW ORDER IS PART OF THE CONTRACT, because the host models this to check
  // it: exactly one draw per step for the density decision, plus a SECOND draw
  // only where a pitch is written. So a rest costs one draw and a note costs
  // two, and a model that draws once per step desyncs at the first rest and
  // reads as a device bug. LANE_GATE writes no pitch, so it is one draw per
  // step throughout.
  bool randomize(uint8_t p, uint8_t lane, uint8_t density, uint16_t seed, uint8_t entropy) {
    if (p >= NUM_PATTERNS || len_[p] == 0) return false;
    if (lane != LANE_ALL && lane != LANE_NOTE && lane != LANE_GATE) return false;
    uint16_t st = seed ? seed : (uint16_t)(entropy | 0x100);           // never 0
    auto rnd = [&st]() -> uint8_t { st ^= st << 7; st ^= st >> 9; st ^= st << 8; return (uint8_t)(st & 0x7F); };
    for (uint8_t i = 0; i < len_[p]; ++i) {
      uint8_t r = rnd();
      bool on = density >= 127 ? true : (density == 0 ? false : (r < density));
      if (lane == LANE_NOTE) {                                         // pitches only
        if (StepKind(step_[p][i]) == STEP_REST) continue;
        step_[p][i] = StepByte(rnd() & CV_MAX, StepKind(step_[p][i]));
      } else if (lane == LANE_GATE) {                                  // legato only
        if (StepKind(step_[p][i]) == STEP_REST) continue;              // no gate on a rest
        step_[p][i] = StepByte(StepCv(step_[p][i]), on ? STEP_LEGATO : STEP_NOTE);
      } else {
        step_[p][i] = on ? StepByte(rnd() & CV_MAX, STEP_NOTE) : StepByte(0, STEP_REST);
      }
    }
    dirty_ |= (uint8_t)(1u << p);
    if (p == cur_) stepChanged_ = true;
    return true;
  }

  // advance one step on a clock edge; returns true if a NON-legato note
  // started (caller pulses PORTAMENTO OFF).
  bool advance() {
    // Loop pass complete with a pattern switch waiting: switch now, and the
    // new pattern starts at its first step.
    if (started_ && queued_ != 0xFF && cycle_ >= cycleLen()) {
      cur_ = queued_; queued_ = 0xFF; loopLen_ = 0; selChanged_ = true;
      started_ = false; cycle_ = 0; ppStep_ = 1;
    }
    uint8_t len = effLen();
    if (len == 0) { curGate_ = false; return false; }
    uint8_t nx = peekNext();
    if (!started_ || cycle_ >= cycleLen()) cycle_ = 0;
    if (DirSetting() == DIR_PINGPONG && started_ && len > 1) {
      if ((int16_t)pos_ + ppStep_ >= len || (int16_t)pos_ + ppStep_ < 0) ppStep_ = (int8_t)-ppStep_;
    }
    pos_ = nx; started_ = true; halfSeen_ = false; ++cycle_;
    if (DirSetting() == DIR_RANDOM) rndNext_ = (uint8_t)(Rnd8() % len);
    stepChanged_ = true;
    uint8_t s = step_[cur_][pos_];
    switch (StepKind(s)) {
      case STEP_REST:   curGate_ = false; return false;
      case STEP_LEGATO: curCv_ = StepCv(s); curGate_ = true; return false;   // gate already on: no retrigger, glide
      default:          curCv_ = StepCv(s); curGate_ = true; return true;
    }
  }
  // Half-step (clock's other edge): close the gate so the next note
  // retriggers, unless the NEXT step is legato (then the gate carries over,
  // 303-style look-ahead).
  void halfStep() {
    halfSeen_ = true;
    uint8_t len = effLen();
    if (!len || !curGate_) return;
    uint8_t next = step_[cur_][peekNext()];
    if (StepKind(next) != STEP_LEGATO) curGate_ = false;
  }
  uint8_t cv()   const { return curCv_; }
  bool    gate() const { return curGate_; }

  // --- persistence (flash arena; payload [0]=len [1..len]=steps, then
  // RatBytes(len) packed ratchet bytes; records written before ratchets
  // existed are exactly 1+len and load with every ratchet off) ---
  void loadAll() {
    uint8_t buf[MAX_STEPS + 1 + MAX_STEPS / 4];
    for (uint8_t p = 0; p < NUM_PATTERNS; ++p) {
      len_[p] = 0;
      memset(rat_[p], 0, sizeof(rat_[p]));
      if (!persist::Available()) continue;
      uint8_t n = persist::Store().read((uint16_t)(persist::FB_PATTERN_BASE + p), buf, sizeof(buf));
      if (n >= 1 && buf[0] <= MAX_STEPS && n >= (uint8_t)(1 + buf[0])) {
        len_[p] = buf[0];
        memcpy(step_[p], buf + 1, len_[p]);
        if (n >= (uint8_t)(1 + len_[p] + RatBytes(len_[p])))
          memcpy(rat_[p], buf + 1 + len_[p], RatBytes(len_[p]));
      }
    }
    dirty_ = 0;
  }
  bool dirty() const { return dirty_ != 0; }
  // Persist ONE dirty pattern (a page write halts the CPU for a few ms).
  // Returns true if something was written.
  bool flushOne() {
    if (!dirty_) return false;
    for (uint8_t p = 0; p < NUM_PATTERNS; ++p) {
      if (!(dirty_ & (1u << p))) continue;
      dirty_ &= (uint8_t)~(1u << p);
      if (!persist::Available()) return false;
      uint8_t buf[MAX_STEPS + 1 + MAX_STEPS / 4];
      buf[0] = len_[p];
      memcpy(buf + 1, step_[p], len_[p]);
      memcpy(buf + 1 + len_[p], rat_[p], RatBytes(len_[p]));
      persist::Store().write((uint16_t)(persist::FB_PATTERN_BASE + p), buf, (uint8_t)(len_[p] + 1 + RatBytes(len_[p])));
      return true;
    }
    return false;
  }
  void flushAll() { while (flushOne()) {} }
};

// --- top-level engine -------------------------------------------------------
class NoteEngine {
  Mode mode_ = MODE_KEYBOARD;
  Arp  arp_;
  Sequencer seq_;
  uint32_t prevMask_ = 0;
  uint32_t holdMask_ = 0;      // keys latched by HOLD
  int8_t   heldKey_  = -1;
  uint8_t  lastCv_   = CV_BASE;
  // KEY TRANSPOSE (PLAY and arp modes): hold KEY TRANSPOSE and press a key;
  // the offset from key 7 (C4) transposes the sequence / arp, and returns to
  // zero when the button is released (stock). A DOUBLE TAP latches it: the
  // offset stays after release and the KEY-TRANSPOSE LED stays lit; double
  // tap again to unlatch (offset back to 0).
  int8_t   transpose_ = 0;
  bool     ktLatched_ = false;
  uint32_t ktLastPressMs_ = 0;
  // Tune mode: KEY TRANSPOSE held + LOAD. Holds the gate open on A (key 16:
  // A4 = 440 Hz at 8', TRANSPOSE middle) so the TUNE knob can be set against
  // a tuner or another instrument. Any function button leaves it.
  bool     tune_ = false;
  // Real-time record: PLAY held + LOAD while playing. Keys overwrite the
  // step they land on (the next step once the half-step has passed), a key
  // still held at the next step ties into it (legato), KEY TRANSPOSE writes
  // a rest. PLAY held + LOAD again (or stop) leaves it; edits persist at stop.
  bool     rtRec_ = false;
  int8_t   rtKey_ = -1;            // key recorded and still held (tie source)
  uint32_t ktKeys_ = 0;            // keys consumed as transpose targets, until released
  uint32_t lenKeys_ = 0;           // keys consumed as loop-length targets, until released
  bool     prevTune_ = false;
  bool     playHeldUsed_ = false;  // PLAY+key selected a pattern (suppress toggle)
  bool     playStart_ = false;     // pulse CLOCK RESET once when play begins
  bool     playNow_   = false;     // panel/editor play: step 1 sounds at the press (0xFA leaves this clear)
  int16_t  prevExtCv_ = -1;
  // Ratchet playback: the step's extra hits are scheduled inside the measured
  // clock period (works for the LFO and the divided MIDI clock alike; the
  // first step after play start has no period yet and plays single).
  uint32_t lastEdgeMs_ = 0;
  uint16_t periodMs_   = 0;
  uint32_t stepStartMs_ = 0;
  uint8_t  subTotal_ = 1, subLeft_ = 0;

  uint8_t noteCv(int8_t key, const controls::Panel& p) const {
    int v = key + (int)p.octaveOffset + CV_BASE;
    return (uint8_t)(v < 0 ? 0 : (v > CV_MAX ? CV_MAX : v));
  }
  // The 101 is LOWEST-note priority unless the panel selects last-note.
  // claim-096 D2: the old middle arm kept heldKey_ whenever it was still in
  // the mask, which is FIRST-note priority, not lowest: hold key 10 and add
  // key 0 and the engine never moved, where the ROM takes the lower key and
  // gives it back on release. Only the last-note path may keep a held key.
  int8_t pickKey(uint32_t mask, const controls::Panel& p) {
    if (!mask) return -1;
    if (!p.lastNotePriority) { heldKey_ = lowestKey(mask); return heldKey_; }
    uint32_t fresh = mask & ~prevMask_;
    if (fresh)                                            heldKey_ = lowestKey(fresh);
    else if (heldKey_ >= 0 && (mask & (1UL << heldKey_))) { /* keep the last key */ }
    else                                                  heldKey_ = lowestKey(mask);
    return heldKey_;
  }
  void startPlay() {
    mode_ = MODE_SEQ_PLAY; seq_.rewind(); playStart_ = true; playNow_ = true;
    midiclk::UserPlay(millis());   // fresh intent: override a host stop, anchor phase
  }
  void stopPlay() { if (rtRec_) { rtRec_ = false; seq_.markEdited(); } rtKey_ = -1; mode_ = MODE_KEYBOARD; }
  void selectMode(controls::Buttons& b, uint32_t rawKeys) {
    using namespace controls;
    const uint32_t nowMs = millis();
    // Tune mode: KEY TRANSPOSE + LOAD toggles; any other function press leaves.
    if (tune_) {
      if (b.now & ~b.prev & ~(1u << BTN_KEYTRANS)) tune_ = false;
      return;
    }
    if (b.down(BTN_KEYTRANS) && b.pressed(BTN_LOAD) && mode_ == MODE_KEYBOARD) { tune_ = true; return; }
    // KEY TRANSPOSE double tap = latch toggle (not in LOAD, where it is the
    // rest button, nor while real-time recording).
    if (b.pressed(BTN_KEYTRANS) && mode_ != MODE_SEQ_LOAD && !rtRec_) {
      if (ktLastPressMs_ && nowMs - ktLastPressMs_ < 400) {
        ktLatched_ = !ktLatched_;
        if (!ktLatched_) transpose_ = 0;
        ktLastPressMs_ = 0;
      } else ktLastPressMs_ = nowMs;
    }
    if (b.released(BTN_KEYTRANS) && !ktLatched_) transpose_ = 0;
    const bool playHeld = b.down(BTN_PLAY) && mode_ != MODE_SEQ_LOAD;
    if (mode_ == MODE_SEQ_LOAD) {
      // While recording, UP/DOWN cycle the last recorded step's ratchet
      // (1x -> 2x -> 3x) instead of switching into an arp mid-recording.
      if (b.pressed(BTN_UP))   seq_.cycleLastRatchet(+1);
      if (b.pressed(BTN_DOWN)) seq_.cycleLastRatchet(-1);
    } else if (playHeld) {
      // PLAY held + UP / DOWN / U&D: sequencer direction FWD / BWD /
      // PING-PONG, U&D again = RANDOM.
      if (b.pressed(BTN_UP))   { seq_.setDirection(DIR_FWD); playHeldUsed_ = true; }
      if (b.pressed(BTN_DOWN)) { seq_.setDirection(DIR_BWD); playHeldUsed_ = true; }
      if (b.pressed(BTN_UD))   { seq_.setDirection(seq_.direction() == DIR_PINGPONG ? DIR_RANDOM : DIR_PINGPONG);
                                 playHeldUsed_ = true; }
    } else {
    if (b.pressed(BTN_UP))   mode_ = (mode_ == MODE_ARP_UP)     ? MODE_KEYBOARD : MODE_ARP_UP;
    if (b.pressed(BTN_DOWN)) mode_ = (mode_ == MODE_ARP_DOWN)   ? MODE_KEYBOARD : MODE_ARP_DOWN;
    if (b.pressed(BTN_UD))   mode_ = (mode_ == MODE_ARP_UPDOWN) ? MODE_ARP_RANDOM
                                   : (mode_ == MODE_ARP_RANDOM) ? MODE_KEYBOARD : MODE_ARP_UPDOWN;
    }
    // PLAY: held + one of the 8 lowest keys selects the pattern (queued to
    // the loop end while playing); a plain press/release toggles sequencer
    // play (toggle happens on release so the select gesture does not also
    // start/stop the sequencer).
    if (b.pressed(BTN_PLAY)) playHeldUsed_ = false;
    if (playHeld) {
      uint32_t fresh = rawKeys & ~prevMask_ & 0xFF;
      if (fresh) {
        uint8_t p = (uint8_t)lowestKey(fresh);
        if (mode_ == MODE_SEQ_PLAY) seq_.queue(p); else seq_.select(p);
        playHeldUsed_ = true;
      }
      // PLAY held + LOAD while playing: real-time record on/off.
      if (b.pressed(BTN_LOAD) && mode_ == MODE_SEQ_PLAY) {
        rtRec_ = !rtRec_; rtKey_ = -1; playHeldUsed_ = true;
        if (rtRec_ && seq_.length() == 0) { seq_.setLength(seq_.current(), 16); seq_.rewind(); }
        if (!rtRec_) seq_.markEdited();
        return;
      }
    }
    if (b.released(BTN_PLAY) && !playHeldUsed_) {
      if (mode_ == MODE_SEQ_PLAY) stopPlay();
      else startPlay();
    }
    if (b.pressed(BTN_LOAD) && !b.down(BTN_PLAY)) {
      if (mode_ == MODE_SEQ_LOAD) { seq_.flushAll(); seq_.markEdited(); mode_ = MODE_KEYBOARD; }
      else { rtRec_ = false; seq_.clear(); mode_ = MODE_SEQ_LOAD; }
    }
  }
 public:
  Mode mode() const { return mode_; }
  Sequencer& seq() { return seq_; }
  void begin() { seq_.loadAll(); }
  // Every member back to its boot value (in-process firmware switch): a
  // stale mode or held-key mask from an earlier stint must not leak in.
  // Keep in step with the member list above.
  void reset() {
    mode_ = MODE_KEYBOARD; arp_.reset(); seq_.resetRuntime();
    prevMask_ = 0; holdMask_ = 0; heldKey_ = -1; lastCv_ = CV_BASE;
    transpose_ = 0; ktLatched_ = false; ktLastPressMs_ = 0; ktKeys_ = 0;
    tune_ = false; rtRec_ = false; rtKey_ = -1;
    playHeldUsed_ = false; playStart_ = false; playNow_ = false; prevExtCv_ = -1;
    lastEdgeMs_ = 0; periodMs_ = 0; stepStartMs_ = 0; subTotal_ = 1; subLeft_ = 0;
  }
  // MIDI Start: align the sequence and arpeggiator to the incoming downbeat.
  void restart() { seq_.rewind(); arp_.reset(); playStart_ = true; }
  // Host transport start (0xFA with sync on): enter play mode WITHOUT the
  // local-press playNow_. Timing begins at the first 0xF8 after Start (MIDI
  // spec), and that tick's boundary edge sounds step 0; an immediate
  // fallback start here would play step 0 twice (once at the Start, again
  // one loop pass later on the downbeat tick). Only from idle keyboard
  // mode: an active arp is already a clock-driven performance and the
  // host's Start must not stomp it into sequencer play (recording is
  // likewise left alone).
  void playFromHost() { if (mode_ == MODE_KEYBOARD) mode_ = MODE_SEQ_PLAY; }
  // SysEx 0x1D / editor: select a pattern (ignored while recording).
  // While playing, the switch waits for the end of the loop pass.
  void selectPattern(uint8_t p) {
    if (mode_ == MODE_SEQ_PLAY) seq_.queue(p);
    else if (mode_ != MODE_SEQ_LOAD) seq_.select(p);
  }
  void setPlaying(bool on) {
    if (mode_ == MODE_SEQ_LOAD) return;
    if (on && mode_ != MODE_SEQ_PLAY) startPlay();
    else if (!on && mode_ == MODE_SEQ_PLAY) stopPlay();
  }
  // SysEx 0x65 feature knobs and their readback (main.cpp).
  int8_t transpose() const { return transpose_; }
  bool   transposeLatched() const { return ktLatched_; }
  void   setTranspose(int8_t t, bool latch) { transpose_ = t; ktLatched_ = latch; }
  bool   tuning() const { return tune_; }
  void   setTune(bool on) { tune_ = on && mode_ == MODE_KEYBOARD; }
  bool   rtRecording() const { return rtRec_; }
  void   setRtRec(bool on) {
    if (mode_ != MODE_SEQ_PLAY) return;
    if (on && !rtRec_ && seq_.length() == 0) { seq_.setLength(seq_.current(), 16); seq_.rewind(); }
    if (!on && rtRec_) seq_.markEdited();
    rtRec_ = on; rtKey_ = -1;
  }
  // KEY-TRANSPOSE LED: lit while a transpose is active or latched.
  bool   ktLed() const { return ktLatched_ || transpose_ != 0; }
  // Function-LED select for the current mode (4556 one-hot, SH101.asm:90-128):
  // 0 none, 1 LOAD, 2 PLAY, 3 DOWN, 4 UP&DOWN, 5 UP.
  uint8_t ledSelect() const {
    const bool blink = (millis() >> 7) & 1;          // ~4 Hz
    if (tune_) return blink ? 1 : 0;                 // tune: LOAD blinks
    if (rtRec_) return blink ? 1 : 2;                // real-time record: PLAY/LOAD alternate
    switch (mode_) {
      case MODE_ARP_RANDOM: return blink ? 4 : 0;    // random arp: U&D blinks
      case MODE_ARP_UP:     return 5;
      case MODE_ARP_DOWN:   return 3;
      case MODE_ARP_UPDOWN: return 4;
      case MODE_SEQ_PLAY:   return 2;
      case MODE_SEQ_LOAD:   return 1;
      default:              return 0;
    }
  }

  // mask: panel keys | in-range MIDI notes (bit 0 = F3). extCv: mono CV from
  // a MIDI note outside the keyboard range (-1 = none). hold: HOLD button or
  // pedal. clockEdge: arp/seq clock.
  Output update(uint32_t mask, int16_t extCv, bool hold, const controls::Panel& p,
                controls::Buttons& b, bool clockEdge, bool halfEdge) {
    selectMode(b, mask);
    if (tune_) {
      // A at the panel's octave: key 16 = A4 = 440 Hz at 8', TRANSPOSE middle.
      Output t{noteCv(16, p), true, false, !prevTune_};
      prevTune_ = true; prevMask_ = mask; prevExtCv_ = extCv; lastCv_ = t.cv;
      return t;
    }
    if (prevTune_) { prevTune_ = false; prevMask_ = mask; }   // no key edge out of tune mode
    if (b.down(controls::BTN_PLAY) && mode_ != MODE_SEQ_LOAD) mask &= ~0xFFUL;  // select keys are not notes
    // KEY TRANSPOSE held in PLAY / arp modes: a fresh key sets the transpose
    // (offset from key 7, C4) instead of playing. Keys pressed during the
    // hold stay out of the arp until released.
    const bool transposing = (mode_ == MODE_SEQ_PLAY && !rtRec_) || IsArp(mode_);
    if (transposing && b.down(controls::BTN_KEYTRANS)) {
      uint32_t fresh = mask & ~prevMask_ & ~ktKeys_;
      if (fresh) { transpose_ = (int8_t)(lowestKey(fresh) - TRANSPOSE_REF_KEY); ktLastPressMs_ = 0; }   // a transpose press is not half a double tap
      ktKeys_ |= fresh;
    }
    ktKeys_ &= mask;
    mask &= ~ktKeys_;
    // HOLD button + key while playing: real-time loop length (key n = the
    // first n+1 steps; the same key again = full pattern). Not the pedal:
    // the pedal keeps its HOLD / transpose role.
    if (mode_ == MODE_SEQ_PLAY && !rtRec_ && b.down(controls::BTN_HOLD)) {
      uint32_t fresh = mask & ~prevMask_ & ~lenKeys_;
      if (fresh) {
        uint8_t n = (uint8_t)(lowestKey(fresh) + 1);
        seq_.setLoopLength(seq_.loopLength() == n ? 0 : n);
      }
      lenKeys_ |= fresh;
    }
    lenKeys_ &= mask;
    mask &= ~lenKeys_;

    // HOLD latch: keys pressed while HOLD is active stay held; a fresh chord
    // replaces the latched set (SH-101 hold behaviour, verify edge cases).
    if (hold) {
      if (mask & ~prevMask_) holdMask_ = (mask & ~prevMask_) | (holdMask_ & mask);
      if (!mask && !holdMask_) {}
      mask |= holdMask_;
    } else {
      holdMask_ = 0;
    }

    Output o{lastCv_, false, false, false};
    switch (mode_) {
      case MODE_ARP_UP: case MODE_ARP_DOWN: case MODE_ARP_UPDOWN: case MODE_ARP_RANDOM:
        // CLOCK RESET only when the arp starts (first key after silence), so
        // the LFO clock is not restarted on every step: a per-step reset made
        // the step spacing depend on the LFO phase and sounded uneven.
        if (mask && !prevMask_) o.clockReset = true;
        if (clockEdge) { int8_t k = arp_.step(mask, mode_); if (k >= 0) { lastCv_ = noteCv(k, p); arp_.gate = true; o.portaOff = true; } }
        if (halfEdge) arp_.gate = false;
        if (!mask) arp_.reset();
        o.cv = lastCv_;
        o.gate = (mask != 0) && arp_.gate;
        break;

      case MODE_SEQ_PLAY:
        if (playStart_) {
          o.clockReset = true; playStart_ = false;   // LFO in phase with step 1
          subTotal_ = 1; subLeft_ = 0; lastEdgeMs_ = 0;   // no stale ratchet state into a new run
          // Panel/editor PLAY (playNow_), internal clock only: step 1 sounds
          // NOW, the LFO hard-reset gives step 2 one period later. With sync
          // ON the press only ARMS: step 1 lands on the next division edge
          // of an 0xFA-anchored stream, on the very next tick of a
          // never-started stream (UserPlay re-anchored the divider at the
          // press), or, with no clock on the wire at all, whenever the
          // host's clock shows up. A synced machine NEVER free-runs: the
          // host's transport and clock are the only things that make it
          // step. 0xFA never sets playNow_.
          if (playNow_) {
            playNow_ = false;
            if (!midiclk::SyncOn()) {
              if (seq_.advance()) o.portaOff = true;
              lastCv_ = seq_.cv();
              clockEdge = false; halfEdge = false;
            }
          }
        }
        if (clockEdge) {
          uint32_t now = millis();
          if (lastEdgeMs_) { uint32_t d = now - lastEdgeMs_; if (d > 0 && d < 4000) periodMs_ = (uint16_t)d; }
          lastEdgeMs_ = now;
          bool fresh = seq_.advance();
          if (fresh) o.portaOff = true;
          lastCv_ = seq_.cv();
          // Ratchet: a NOTE step (fresh) with ratchet r plays r extra evenly
          // spaced hits inside this period; needs a measured period first.
          uint8_t r = (fresh && periodMs_) ? seq_.ratchetCur() : 0;
          subTotal_ = (uint8_t)(r + 1);
          subLeft_  = r;
          stepStartMs_ = now;
        }
        if (halfEdge && rtRec_ && rtKey_ >= 0) {
          // The recorded key is still held at the half-step: tie it into the
          // next step (legato, same pitch), written BEFORE the half-step so
          // the gate carries over, 303-style.
          if (mask & (1UL << rtKey_)) seq_.writeStep(seq_.nextIndex(), StepByte(noteCv(rtKey_, p), STEP_LEGATO));
          else rtKey_ = -1;
        }
        if (halfEdge && subTotal_ <= 1) seq_.halfStep();   // ratcheted steps gate by sub-hit instead
        if (rtRec_) {
          // Real-time record: a key lands on the step it was pressed in, or on
          // the next one once this step's half-step has passed (nearest step).
          uint32_t fresh = mask & ~prevMask_;
          bool rest = b.pressed(controls::BTN_KEYTRANS);
          if (fresh || rest) {
            const bool now = !seq_.halfSeen();
            const uint8_t idx = now ? seq_.position() : seq_.nextIndex();
            if (rest) { seq_.writeStep(idx, StepByte(0, STEP_REST)); if (now) seq_.soundNow(seq_.cv(), false); rtKey_ = -1; }
            else {
              int8_t k = lowestKey(fresh);
              uint8_t cv = noteCv(k, p);
              seq_.writeStep(idx, StepByte(cv, STEP_NOTE));
              rtKey_ = k;
              if (now) { seq_.soundNow(cv, true); lastCv_ = cv; o.portaOff = true; }
            }
          }
        }
        o.cv = seq_.gate() ? seq_.cv() : lastCv_;
        o.gate = seq_.gate();
        if (subLeft_ && periodMs_ && seq_.gate()) {
          uint32_t now = millis();
          uint8_t  done = (uint8_t)(subTotal_ - subLeft_);
          uint32_t at = stepStartMs_ + (uint32_t)periodMs_ * done / subTotal_;
          uint16_t gap = (uint16_t)(periodMs_ / (subTotal_ * 4));
          if (gap < 2) gap = 2; else if (gap > 12) gap = 12;
          if ((int32_t)(now - at) >= 0) { --subLeft_; o.portaOff = true; }  // sub-hit: fresh attack
          else if ((int32_t)(now - (at - gap)) >= 0) o.gate = false;       // gate gap before it
        }
        break;

      case MODE_SEQ_LOAD: {
        // record: KEY-TRANSPOSE = rest, HOLD-held = legato, else the note.
        // (The fn-button mask guard above keeps the LOAD press's own scan
        // bleed from ever appearing as a fresh key here.)
        if (b.pressed(controls::BTN_KEYTRANS)) seq_.record(lastCv_, STEP_REST);
        uint32_t fresh = mask & ~prevMask_;
        if (fresh) { int8_t k = lowestKey(fresh); if (k >= 0) seq_.record(noteCv(k, p), b.down(controls::BTN_HOLD) ? STEP_LEGATO : STEP_NOTE); }
        if (extCv >= 0 && extCv != prevExtCv_) seq_.record((uint8_t)extCv, b.down(controls::BTN_HOLD) ? STEP_LEGATO : STEP_NOTE);
        // live monitor through the keyboard while recording
        int8_t lk = lowestKey(mask);
        if (lk >= 0) { lastCv_ = noteCv(lk, p); o.cv = lastCv_; o.gate = true; }
        else if (extCv >= 0) { lastCv_ = (uint8_t)extCv; o.cv = lastCv_; o.gate = true; }
        break;
      }

      default: { // MODE_KEYBOARD
        int8_t k = pickKey(mask, p);
        if (k >= 0) {
          lastCv_ = noteCv(k, p); o.cv = lastCv_; o.gate = true;
          if (p.adsrLfo && (mask & ~prevMask_)) o.clockReset = true; // reset LFO on new key
          if (!prevMask_ && prevExtCv_ < 0) o.portaOff = true;       // fresh from silence: no glide (AUTO)
        } else if (extCv >= 0) {                     // MIDI note outside the keyboard
          lastCv_ = (uint8_t)extCv; o.cv = lastCv_; o.gate = true;
          if (p.adsrLfo && extCv != prevExtCv_) o.clockReset = true;
          if (!prevMask_ && prevExtCv_ < 0) o.portaOff = true;
        }
        break;
      }
    }

    if (transpose_ && (mode_ == MODE_SEQ_PLAY || IsArp(mode_))) {
      int16_t v = (int16_t)o.cv + transpose_;
      o.cv = (uint8_t)(v < 0 ? 0 : (v > CV_MAX ? CV_MAX : v));
    }
    prevMask_ = mask;
    prevExtCv_ = extCv;
    return o;
  }
};

} // namespace engine
