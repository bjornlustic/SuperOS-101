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
//   - sequencer: 16 patterns x up to 128 steps (note / rest / legato), kept in RAM
//                and persisted in the flash arena (flash_persist.h); pattern
//                select = PLAY held + one of the 16 lowest keys, or SysEx 0x1D
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
  MODE_ARP_RANDOM,   // DOWN + U&D together (SH-1oh1); U&D LED blinks
};
inline bool IsArp(Mode m) {
  return m == MODE_ARP_UP || m == MODE_ARP_DOWN || m == MODE_ARP_UPDOWN || m == MODE_ARP_RANDOM;
}
// Sequencer play direction (global setting, EEPROM, config menu keys 15-18,
// UP / DOWN / U&D / DOWN+U&D while playing, SysEx 0x65 id 0).
enum SeqDir : uint8_t { DIR_FWD = 0, DIR_BWD = 1, DIR_PINGPONG = 2, DIR_RANDOM = 3, DIR_COUNT = 4 };
// Tiny shared PRNG for the random arp / random direction (xorshift16).
inline uint8_t Rnd8() { static uint16_t st = 0xACE1; st ^= st << 7; st ^= st >> 9; st ^= st << 8; return (uint8_t)st; }
// Persistent performance settings (EEPROM 0x020.., combined.h map): the
// sequencer direction and the pedal route. Written only when not clocked
// (an EEPROM byte write stalls ~3.4 ms; main.cpp defers it).
inline uint8_t& DirSetting() { static uint8_t d = DIR_FWD; return d; }
inline uint8_t& ArpRange() { static uint8_t r = 1; return r; }   // arp octave range 1..3
// Performance modes (NoteEngine::perfKeys): while the sequencer plays, the
// keys are the overlay strip and the TRANSPOSE switch picks its row. Saved;
// on unless turned off (0x65 id 16), erased EEPROM reads as on.
inline bool& PerfOn() { static bool on = true; return on; }
inline bool& FeatDirty() { static bool d = false; return d; }
inline void LoadFeatures() {
  if (eeprom_read_byte((uint8_t*)EE_A_FEAT_MAGIC) != EE_FEAT_MAGIC_VAL) return;
  uint8_t d = eeprom_read_byte((uint8_t*)EE_A_FEAT_DIR);
  DirSetting() = d < DIR_COUNT ? d : DIR_FWD;
  uint8_t pm = eeprom_read_byte((uint8_t*)EE_A_FEAT_PEDAL);
  controls::PedalRoute() = pm <= controls::PEDAL_TRANSPOSE ? pm : controls::PEDAL_HOLD;
  uint8_t ar = eeprom_read_byte((uint8_t*)EE_A_FEAT_ARPRANGE);   // 0xFF before it existed
  ArpRange() = (ar >= 1 && ar <= 3) ? ar : 1;
  PerfOn() = eeprom_read_byte((uint8_t*)EE_A_FEAT_PERF) != 0;   // 0xFF before it existed = on
}
inline void SaveFeatures() {
  eeprom_update_byte((uint8_t*)EE_A_FEAT_MAGIC, EE_FEAT_MAGIC_VAL);
  eeprom_update_byte((uint8_t*)EE_A_FEAT_DIR, DirSetting());
  eeprom_update_byte((uint8_t*)EE_A_FEAT_PEDAL, controls::PedalRoute());
  eeprom_update_byte((uint8_t*)EE_A_FEAT_ARPRANGE, ArpRange());
  eeprom_update_byte((uint8_t*)EE_A_FEAT_PERF, PerfOn() ? 1 : 0);
  FeatDirty() = false;
}
// Scales, the SuperOS-303's 35 presets in the same order (bit 0 = C). A
// pattern's scale is the preset mask rotated to a root, stored per pattern.
static const uint8_t NUM_SCALES = 35;
static const uint16_t SCALE_MASKS[NUM_SCALES] = {   // 70 B of RAM; plenty free
  0xAB5, 0x5AD, 0x6AD, 0x6B5, 0xAD5, 0x5AB, 0x56B, 0x555, 0x6DB, 0xB6D,
  0x4E9, 0x4A9, 0x295, 0x9AD, 0x9B5, 0x6CD, 0x5B3, 0xAAD, 0xB55, 0x6D5,
  0x55B, 0x57B, 0x9B3, 0x9CD, 0x18D, 0x4A3, 0x463, 0x28D, 0x18B, 0x1A3,
  0xDDD, 0x9E7, 0x8E3, 0xD75, 0xBEF };
inline uint16_t ScalePreset(uint8_t i, uint8_t root) {
  uint16_t m = SCALE_MASKS[i < NUM_SCALES ? i : 0];
  root %= 12;
  return (uint16_t)(((m << root) | (m >> (12 - root))) & 0xFFF);
}
// KEY TRANSPOSE reference: key 7 (C4, MIDI 60) plays the sequence as written.
static const uint8_t TRANSPOSE_REF_KEY = 7;
// Performance octave variations 1..6 (7 = a random octave per step): octaves
// added to a note by its step index mod 8.
static const int8_t OCT_VAR[6][8] = {
  {0, 1, 0, 1, 0, 1, 0, 1}, {0, 0, 0, 1, 0, 0, 0, 1}, {0, -1, 0, -1, 0, -1, 0, -1},
  {0, 1, 0, -1, 0, 1, 0, -1}, {0, 0, 1, 1, 0, 0, 1, 1}, {0, 0, 0, 0, 1, 1, 1, 1} };
// Legato (2): this step's note starts WITHOUT the gate dropping after the
// previous step: no envelope retrigger, and (portamento AUTO) a glide into it.
enum StepType : uint8_t { STEP_NOTE = 0, STEP_REST = 1, STEP_LEGATO = 2 };

// Edit lanes, shared with the fleet's edit opcodes (0x16 SET STEP, 0x2E
// RANDOMIZE). Lane 2 (slide) DOES NOT EXIST on the 101; lane 1 (accent) does
// since the CV DAC mod (a per-step bit that fires the accent envelope). They
// are named here so a refusal is a deliberate decision at a named lane rather
// than a default case swallowing an unknown byte.
static const uint8_t LANE_NOTE   = 0;
static const uint8_t LANE_ACCENT = 1;   // per-step accent (CV DAC mod), 0x16 only; RANDOMIZE refuses it
static const uint8_t LANE_SLIDE  = 2;   // absent on the 101 -> ack 2
static const uint8_t LANE_GATE   = 3;
static const uint8_t LANE_FILTER = 4;   // per-step filter CV, 0 none / 1..127 (0x16 only)
static const uint8_t LANE_ALL    = 0x7F;

// One step byte, the SH-101's own model: 6-bit CV + 2 type bits.
inline uint8_t  StepByte(uint8_t cv, StepType t) { return (uint8_t)((cv & 0x3F) | ((uint8_t)t << 6)); }
inline uint8_t  StepCv  (uint8_t s) { return s & 0x3F; }
inline StepType StepKind(uint8_t s) { uint8_t t = s >> 6; return t > 2 ? STEP_NOTE : (StepType)t; }

// accent: an accented sequencer step starts this pass. stepStart: a sequencer
// step (any kind) starts this pass, with filter = its per-step filter value
// (0 = none, else 1..127 on the cutoff CC's scale).
struct Output { uint8_t cv; bool gate; bool clockReset; bool portaOff; bool accent; bool stepStart; uint8_t filter; };

// --- held-key (32-bit) mask helpers -----------------------------------------
inline int8_t lowestKey (uint32_t m) { for (uint8_t k = 0; k < 32; ++k) if (m & (1UL << k)) return (int8_t)k; return -1; }
inline int8_t highestKey(uint32_t m) { for (int8_t k = 31; k >= 0; --k) if (m & (1UL << k)) return k; return -1; }
inline int8_t keyAbove  (uint32_t m, int8_t k) { for (int8_t i = k + 1; i < 32; ++i) if (m & (1UL << i)) return i; return -1; }
inline int8_t keyBelow  (uint32_t m, int8_t k) { for (int8_t i = k - 1; i >= 0; --i) if (m & (1UL << i)) return i; return -1; }

// --- arpeggiator ------------------------------------------------------------
class Arp {
  int8_t cur = -1;
  int8_t dir = 1;
  uint8_t oct = 0;     // octave offset of the current note, 0..range-1
 public:
  bool gate = false;   // on at each step, off at the half-step
  int8_t current() const { return cur; }
  uint8_t octave() const { return oct; }
  void reset() { cur = -1; dir = 1; oct = 0; gate = false; }
  // range: 1..3 octaves. The held keys play in order through each octave
  // (UP: low -> high, octave by octave; DOWN from the top octave; U&D
  // bounces across the whole span; RANDOM picks a key and an octave).
  int8_t step(uint32_t mask, Mode mode, uint8_t range = 1) {
    if (!mask) { cur = -1; return -1; }
    if (range < 1) range = 1;
    if (oct >= range) oct = 0;
    if (cur < 0 || !(mask & (1UL << cur))) {        // (re)start
      dir = (mode == MODE_ARP_DOWN) ? -1 : 1;
      cur = (mode == MODE_ARP_DOWN) ? highestKey(mask) : lowestKey(mask);
      oct = (mode == MODE_ARP_DOWN) ? (uint8_t)(range - 1) : 0;
      return cur;
    }
    int8_t nxt;
    if (mode == MODE_ARP_RANDOM) {
      // A random held key in a random octave, never the same note twice in
      // a row when there is a choice (a repeat reads as a dropped step).
      uint8_t n = 0; for (uint8_t k = 0; k < 32; ++k) if (mask & (1UL << k)) ++n;
      uint8_t pick = (uint8_t)(Rnd8() % n);
      uint8_t o = (uint8_t)(Rnd8() % range);
      nxt = -1;
      for (int8_t k = 0; k < 32; ++k) if (mask & (1UL << k)) { if (pick-- == 0) { nxt = k; break; } }
      if (nxt == cur && o == oct && n * range > 1) {
        if (n > 1) { nxt = keyAbove(mask, cur); if (nxt < 0) nxt = lowestKey(mask); }
        else o = (uint8_t)((o + 1) % range);
      }
      oct = o;
    } else if (mode == MODE_ARP_UP) {
      nxt = keyAbove(mask, cur);
      if (nxt < 0) { nxt = lowestKey(mask); oct = (uint8_t)((oct + 1) % range); }
    } else if (mode == MODE_ARP_DOWN) {
      nxt = keyBelow(mask, cur);
      if (nxt < 0) { nxt = highestKey(mask); oct = (uint8_t)((oct + range - 1) % range); }
    } else {                                         // UP&DOWN across the span
      if (dir > 0) {
        nxt = keyAbove(mask, cur);
        if (nxt < 0) {
          if (oct + 1 < range) { ++oct; nxt = lowestKey(mask); }
          else { dir = -1; nxt = keyBelow(mask, cur); if (nxt < 0 && oct > 0) { --oct; nxt = cur; } }
        }
      } else {
        nxt = keyBelow(mask, cur);
        if (nxt < 0) {
          if (oct > 0) { --oct; nxt = highestKey(mask); }
          else { dir = 1; nxt = keyAbove(mask, cur); if (nxt < 0 && range > 1) { ++oct; nxt = cur; } }
        }
      }
      if (nxt < 0) nxt = cur;
    }
    cur = nxt;
    return cur;
  }
};

// --- sequencer: 16 patterns in RAM, flash-arena backed --------------------------
class Sequencer {
 public:
  static const uint8_t CHAIN_MAX = 16;
 private:
  uint8_t step_[NUM_PATTERNS][MAX_STEPS];
  // Per-step ratchet, 2 bits each (0 = single hit, 1 = 2x, 2 = 3x): the step's
  // note retriggers evenly inside its clock period. Packed 4 steps/byte so the
  // whole lane costs 256 bytes, not another KB of a 2.5 KB SRAM part.
  uint8_t rat_[NUM_PATTERNS][MAX_STEPS / 4];
  // Per-step accent, 1 bit each (bit i&7 of byte i>>3): the step fires the CV
  // DAC's accent envelope (cvdac.h) as a velocity-127 note would.
  uint8_t acc_[NUM_PATTERNS][MAX_STEPS / 8];
  // Per-step filter CV (0 = none: the cutoff CC / FREQ slider; 1..127 on the
  // CC's scale, 64 = the slider's own position). All 16 lanes would cost 2 KB of
  // an 8 KB part whose stack needs the room, so the lanes live in the flash
  // arena (FB_FILTER_BASE + p) and two are cached: slot 0 is always the
  // current pattern's (playback reads it), slot 1 holds the pattern queued
  // next (preloaded between notes, so a switch on the bar is a swap, not a
  // 5 ms flash read) or the last other pattern an editor changed. Dirty slots
  // are written by flushOne() with the patterns.
  struct Lane { uint8_t pat; bool dirty; uint8_t v[MAX_STEPS]; };
  Lane lane_[2] = {{0xFF, false, {}}, {0xFF, false, {}}};
  uint16_t laneChanged_ = 0;          // bit p = lane p changed (mirror to the editor, 0x2C)
  // Scale per pattern: bits 0-11 = allowed pitch classes (bit 0 = C), bit 15
  // = on. Non-destructive: steps keep their pitch, the output snaps to the
  // nearest allowed note (lower on a tie), before any transpose (as the 303).
  uint16_t scale_[NUM_PATTERNS];
  static const uint8_t RAT_MAX = 2;
  static uint8_t RatBytes(uint8_t len) { return (uint8_t)((len + 3) >> 2); }
 public:
  static uint8_t AccBytes(uint8_t len) { return (uint8_t)((len + 7) >> 3); }
 private:
  uint8_t len_[NUM_PATTERNS];
  uint16_t dirty_ = 0;                // bit p = pattern p differs from flash
  uint8_t cur_ = 0;                   // selected pattern
  uint8_t pos_ = 0;
  uint8_t curCv_ = CV_BASE; bool curGate_ = false;
  bool    stepChanged_ = false;
  bool    selChanged_ = false;
  uint16_t edited_ = 0;               // bit p = edited on the panel since last consume
  int16_t rotPending_ = 0;            // nudge waiting for the next half-step (playing pattern)
  uint16_t scaleChanged_ = 0;         // bit p = scale changed (mirror to the editor, 0x2A)
  // Playback shape (runtime; DirSetting() persists via the engine's settings):
  bool    started_ = false;           // false: the next advance plays the FIRST step
  bool    halfSeen_ = false;          // the current step's half-step has passed
  uint8_t loopLen_ = 0;               // real-time loop length override, 0 = full pattern
  uint8_t queued_ = 0xFF;             // pattern switch waiting for the loop end
  // Song: PLAY held + several keys chains patterns (SH-1oh1 3.6); each loop
  // end moves to the next entry, wrapping. chainN_ <= 1 = no song.
  uint8_t chain_[16];
  uint8_t chainN_ = 0, chainPos_ = 0;
  uint8_t wcur_ = 0;                  // LOAD write cursor (stock: re-load from the playing position)
  // Position is DERIVED, never walked: cycle_ is the index of the next step
  // within the current loop pass, and steps_ counts every step played since
  // play started. Direction, loop length, instant switches and division
  // changes re-derive cycle_ from steps_ (realign), so the pattern stays on
  // the one whatever is changed while it plays.
  uint8_t cycle_ = 0;
  uint16_t steps_ = 0;
  uint8_t rndNext_ = 0;               // RANDOM: the step the next advance plays (for look-ahead)
  uint8_t effLen() const {
    uint8_t len = len_[cur_];
    return (loopLen_ && loopLen_ < len) ? loopLen_ : len;
  }
  // Ping-pong repeats its end steps (0..n-1, n-1..0): 2n steps, so a pattern
  // that fits the bar still fits it after a full round trip.
  uint8_t cycleLen() const {
    uint8_t l = effLen();
    return (DirSetting() == DIR_PINGPONG && l > 1) ? (uint8_t)(2 * l) : l;
  }
  // The step played at index i of the loop pass.
  uint8_t stepAt(uint8_t i) const {
    uint8_t l = effLen();
    if (!l) return 0;
    uint8_t s;
    switch (DirSetting()) {
      case DIR_BWD:      s = (uint8_t)(l - 1 - i % l); break;
      case DIR_PINGPONG: s = i < l ? i : (uint8_t)(2 * l - 1 - i); break;
      case DIR_RANDOM:   s = rndNext_ < l ? rndNext_ : 0; break;
      default:           s = (uint8_t)(i % l); break;
    }
    if (!perf.shift) return s;
    int16_t j = (int16_t)(((int16_t)s - perf.shift) % l);   // shift time: later (+) plays an earlier step
    return (uint8_t)(j < 0 ? j + l : j);
  }
  // The step the next advance() will play, without moving.
  uint8_t peekNext() const {
    uint8_t cl = cycleLen();
    if (!cl) return 0;
    if (started_ && queued_ != 0xFF && cycle_ == 0) return 0;   // switching: unknown pattern, treat as a fresh attack
    return stepAt((uint8_t)(cycle_ % cl));
  }
  // --- performance transforms, applied at playback (the pattern is never written) ---
  // The other sequence for the Combine functions: the next pattern number.
  uint8_t other() const { return (uint8_t)((cur_ + 1) % NUM_PATTERNS); }
  uint8_t firstNoteCv(uint8_t p) const {
    for (uint8_t k = 0; k < len_[p]; ++k) if (StepKind(step_[p][k]) != STEP_REST) return StepCv(step_[p][k]);
    return 0;
  }
  // Reorder notes: the pitch of note number r (rests keep their places) comes
  // from note number t.
  uint8_t reorderSrc(uint8_t i) const {
    const uint8_t l = effLen(); const uint8_t* st = step_[cur_];
    uint8_t n = 0, r = 0;
    for (uint8_t k = 0; k < l; ++k) if (StepKind(st[k]) != STEP_REST) { if (k < i) ++r; ++n; }
    if (n < 2) return i;
    uint8_t t;
    switch (perf.reorder) {
      case 1:  t = (uint8_t)(n - 1 - r); break;                                         // reverse
      case 2:  { uint8_t h = (uint8_t)((n + 1) / 2); t = r < h ? 2 * r : 2 * (r - h) + 1; break; }   // cycle even: 0 2 4 .. 1 3 5
      case 3:  { uint8_t h = (uint8_t)(n / 2); t = r < h ? 2 * r + 1 : 2 * (r - h); break; }         // cycle odd: 1 3 5 .. 0 2 4
      case 4:  {                                                                        // prime order: a prime stride
        static const uint8_t PR[] = {3, 5, 7, 11, 13};
        uint8_t p = 1;
        for (uint8_t q : PR) if (n % q) { p = q; break; }
        t = (uint8_t)((uint16_t)r * p % n); break;
      }
      case 5:  t = (uint8_t)((r + 1) % n); break;                                       // start one forward
      default: t = (uint8_t)((r + n - 1) % n); break;                                   // start one back
    }
    for (uint8_t k = 0; k < l; ++k) if (StepKind(st[k]) != STEP_REST && t-- == 0) return k;
    return i;
  }
  // Combine pitch with the other sequence: intervals above each pattern's
  // first note, combined, folded by octaves into the DAC range.
  uint8_t combinePitch(uint8_t cv, uint8_t i) const {
    const uint8_t o = other(), ol = len_[o];
    if (!ol) return cv;
    const uint8_t os = step_[o][i % ol];
    if (StepKind(os) == STEP_REST) return cv;
    const int16_t base = firstNoteCv(cur_), a = (int16_t)cv - base, b = (int16_t)StepCv(os) - firstNoteCv(o);
    int16_t v;
    switch (perf.pcomb) {
      case 1:  v = a + b; break;
      case 2:  v = a - b; break;
      case 3:  v = a * b; break;
      case 4:  v = b ? a / b : a; break;
      default: v = a > b ? a - b : b - a; break;
    }
    v += base;
    if (v < 0) v = (int16_t)((v % 12 + 12) % 12);
    else if (v > CV_MAX) v = (int16_t)(CV_MAX - 11 + (v - (CV_MAX - 11)) % 12);
    return (uint8_t)v;
  }
  // Step i as it plays: rhythm combined with the other sequence, then the
  // pitch reordered, combined and octave-varied.
  uint8_t playByte(uint8_t i) const {
    const uint8_t s = step_[cur_][i];
    const bool note = StepKind(s) != STEP_REST;
    if (perf.tcomb && len_[other()]) {
      const uint8_t os = step_[other()][i % len_[other()]];
      const bool on = StepKind(os) != STEP_REST;
      bool r;
      switch (perf.tcomb) {
        case 1:  r = note && on; break;
        case 2:  r = note || on; break;
        case 3:  r = !(note || on); break;
        case 4:  r = note != on; break;
        default: r = !(note && on); break;
      }
      if (!r) return StepByte(StepCv(s), STEP_REST);
      if (!note) return StepByte(on ? StepCv(os) : curCv_, STEP_NOTE);   // a step the logic filled
    }
    if (!note) return s;
    uint8_t cv = StepCv(s);
    if (perf.reorder) cv = StepCv(step_[cur_][reorderSrc(i)]);
    if (perf.pcomb) cv = combinePitch(cv, i);
    if (perf.octVar) {
      int8_t o = perf.octVar <= 6 ? OCT_VAR[perf.octVar - 1][i & 7] : (int8_t)(Rnd8() % 3) - 1;
      int16_t v = (int16_t)cv + 12 * o;
      if (v >= 0 && v <= CV_MAX) cv = (uint8_t)v;
    }
    return StepByte(cv, StepKind(s));
  }
 public:
  // Performance state (NoteEngine::perfKeys sets it; 0 / -1 = off).
  struct Perf {
    uint8_t reorder = 0;  // 1 reverse, 2 cycle even, 3 cycle odd, 4 prime, 5 one forward, 6 one back
    int8_t  root = -1;    // quantizer root 0..11 (C = 0)
    int8_t  type = -1;    // quantizer scale: SCALE_MASKS 0..6 (major minor dorian mixolydian lydian phrygian locrian)
    uint8_t pcomb = 0;    // pitch with the other sequence: 1 add 2 subtract 3 multiply 4 divide 5 difference
    uint8_t octVar = 0;   // octave variation 1..7
    int8_t  shift = 0;    // shift time, steps later (+) / earlier (-)
    uint8_t tcomb = 0;    // rhythm with the other sequence: 1 AND 2 OR 3 NOR 4 XOR 5 NAND
  } perf;
  void mute() { curGate_ = false; }       // a skipped clock slot sounds nothing
  uint8_t direction() const { return DirSetting(); }
  void setDirection(uint8_t d) {
    if (d >= DIR_COUNT) return;
    DirSetting() = d; FeatDirty() = true;
    rndNext_ = (uint8_t)(Rnd8() % (effLen() ? effLen() : 1));
    realign(steps_);
  }
  uint16_t stepsPlayed() const { return steps_; }
  // Put the playhead where it would be had the current shape been playing
  // since the start: the next step is index (steps % loop-pass length). Used
  // for every live change and for Resync (steps from the clock since Start).
  void realign(uint16_t steps) {
    steps_ = steps;
    uint8_t cl = cycleLen();
    cycle_ = cl ? (uint8_t)(steps % cl) : 0;
    if (steps) started_ = true;
  }
  uint8_t loopLength() const { return loopLen_; }
  // Real-time loop length (panel HOLD + key while playing, SysEx 0x65 id 4):
  // plays only the first n steps without touching the pattern; 0 = full.
  void setLoopLength(uint8_t n) {
    loopLen_ = n;
    if (effLen() && pos_ >= effLen()) pos_ = (uint8_t)(effLen() - 1);
    realign(steps_);
  }
  uint8_t queued() const { return queued_; }
  // Pattern change while playing: waits for the end of the current loop pass
  // so the switch lands on the bar, like a drum machine's pattern select.
  void queue(uint8_t p) { if (p < NUM_PATTERNS) { queued_ = (p == cur_) ? 0xFF : p; chainN_ = 0; } }
  // Song: play list[0..n) in order, looping, starting at the next loop end
  // (playing) or right away (stopped: the caller selects list[0] first).
  void setChain(const uint8_t* list, uint8_t n, bool fromNow) {
    if (!n) return;
    if (n > CHAIN_MAX) n = CHAIN_MAX;
    memcpy(chain_, list, n);
    // queued_ is always chain_[chainPos_]: the entry the next loop end plays.
    if (fromNow) select(chain_[0]);                     // (select clears the song)
    chainN_ = n > 1 ? n : 0;
    chainPos_ = fromNow ? (uint8_t)(1 % n) : 0;
    queued_ = (fromNow && n == 1) ? 0xFF : chain_[chainPos_];
  }
  uint8_t chainLength() const { return chainN_; }
  // Instant switch while playing (SH-1oh1 3.5.3): the new pattern takes over
  // at the SAME position (modulo its length), so patterns whose lengths are
  // multiples of each other stay in sync. Cancels a song / queued switch.
  void switchNow(uint8_t p) {
    if (p >= NUM_PATTERNS) return;
    queued_ = 0xFF; chainN_ = 0;
    if (p == cur_) return;
    cur_ = p; loopLen_ = 0; selChanged_ = true; laneFollow();
    uint8_t l = effLen();
    if (l) pos_ = (uint8_t)(pos_ % l);
    realign(steps_);
  }
  // Shift (SH-1oh1 3.3.2, KEY TRANSPOSE + UP / DOWN while playing): play the
  // next step now, or the previous one, moving the pattern against the clock.
  bool nudge(int8_t d) {
    uint8_t l = effLen();
    if (!l || !started_) return false;
    if (d < 0) {                       // back two in the pass, then advance one
      uint8_t cl = cycleLen();
      cycle_ = (uint8_t)((cycle_ + 2 * cl - 2) % cl);
      steps_ = steps_ >= 2 ? (uint16_t)(steps_ - 2) : 0;
    }
    return advance();                  // a shift moves the phase on purpose
  }
  bool halfSeen() const { return halfSeen_; }
  uint8_t nextIndex() const { return peekNext(); }
  // Real-time record: overwrite one step in place (pattern data, persisted).
  void writeStep(uint8_t idx, uint8_t stepByte) {
    if (idx >= len_[cur_]) return;
    step_[cur_][idx] = stepByte; ratchetRaw(cur_, idx, 0); accentRaw(cur_, idx, false);
    dirty_ |= (uint16_t)(1u << cur_); stepChanged_ = true;
  }
  // The step sounding now: its gate/CV change immediately (real-time record
  // on the current step).
  void soundNow(uint8_t cv, bool gate) { curCv_ = cv; curGate_ = gate; }
  uint16_t consumeEdited() { uint16_t e = edited_; edited_ = 0; return e; }
  // Runtime state back to boot values (in-process firmware switch); pattern
  // CONTENT is not touched here, loadAll() reloads it from flash right after.
  void resetRuntime() {
    dirty_ = 0; cur_ = 0; pos_ = 0; curCv_ = CV_BASE; curGate_ = false;
    lane_[0].pat = lane_[1].pat = 0xFF; lane_[0].dirty = lane_[1].dirty = false; laneChanged_ = 0;
    stepChanged_ = false; selChanged_ = false; edited_ = 0;
    started_ = false; halfSeen_ = false; loopLen_ = 0; queued_ = 0xFF; cycle_ = 0; steps_ = 0; rotPending_ = 0;
    chainN_ = 0; chainPos_ = 0; wcur_ = 0; perf = Perf();
  }
  Sequencer() { memset(len_, 0, sizeof(len_)); for (uint8_t i = 0; i < NUM_PATTERNS; ++i) scale_[i] = 0x0FFF; }
  uint8_t current() const { return cur_; }
  uint8_t length(uint8_t p) const { return p < NUM_PATTERNS ? len_[p] : 0; }
  uint8_t length() const { return len_[cur_]; }
  const uint8_t* steps(uint8_t p) const { return step_[p < NUM_PATTERNS ? p : 0]; }
  uint8_t position() const { return pos_; }
  bool consumeStepChanged() { bool c = stepChanged_; stepChanged_ = false; return c; }

  void select(uint8_t p) {
    if (p < NUM_PATTERNS && p != cur_) { cur_ = p; loopLen_ = 0; laneFollow(); rewind(); selChanged_ = true; }
    queued_ = 0xFF; chainN_ = 0;
  }
  bool consumeSelChanged() { bool c = selChanged_; selChanged_ = false; return c; }
  void markEdited() { edited_ |= (uint16_t)(1u << cur_); }
  void clear() {
    len_[cur_] = 0; pos_ = 0; wcur_ = 0; memset(rat_[cur_], 0, sizeof(rat_[cur_])); memset(acc_[cur_], 0, sizeof(acc_[cur_]));
    dirty_ |= (uint16_t)(1u << cur_);
    laneFollow();
    bool any = false; for (uint8_t i = 0; i < MAX_STEPS; ++i) any |= lane_[0].v[i] != 0;
    if (any) { memset(lane_[0].v, 0, MAX_STEPS); lane_[0].dirty = true; laneChanged_ |= (uint16_t)(1u << cur_); }
  }
  // Stock "Correcting and Adding" (OM p.35): LOAD while playing keeps the
  // pattern and re-loads from the step after the one sounding; steps past
  // the corrected ones stay and play.
  void beginOverwrite() { uint8_t l = len_[cur_]; wcur_ = (l && started_) ? (uint8_t)((pos_ + 1) % l) : 0; if (wcur_ == 0 && l) wcur_ = l; }
  uint8_t writeCursor() const { return wcur_; }
  // LOAD mode DOWN (SH-1oh1 3.2): take back the last entered step.
  void deleteLast() {
    if (!wcur_) return;
    if (wcur_ == len_[cur_]) --len_[cur_];
    --wcur_;
    dirty_ |= (uint16_t)(1u << cur_);
  }
  uint8_t ratchet(uint8_t p, uint8_t i) const {
    return (uint8_t)((rat_[p < NUM_PATTERNS ? p : 0][i >> 2] >> ((i & 3) * 2)) & 3);
  }
  uint8_t ratchetCur() const { return ratchet(cur_, pos_); }
  void ratchetRaw(uint8_t p, uint8_t i, uint8_t v) {
    uint8_t sh = (uint8_t)((i & 3) * 2);
    rat_[p][i >> 2] = (uint8_t)((rat_[p][i >> 2] & ~(3u << sh)) | ((v & 3u) << sh));
  }
  // --- filter lane cache ---
  void laneWrite(Lane& l) {
    if (l.pat < NUM_PATTERNS && l.dirty && persist::Available())
      persist::Store().write((uint16_t)(persist::FB_FILTER_BASE + l.pat), l.v, MAX_STEPS);
    l.dirty = false;
  }
  void laneRead(uint8_t p, uint8_t* dst) const {
    uint8_t n = persist::Available() ? persist::Store().read((uint16_t)(persist::FB_FILTER_BASE + p), dst, MAX_STEPS) : 0;
    for (uint8_t i = n; i < MAX_STEPS; ++i) dst[i] = 0;
    for (uint8_t i = 0; i < n; ++i) dst[i] &= 0x7F;
  }
  void laneLoad(Lane& l, uint8_t p) { if (l.pat == p) return; laneWrite(l); laneRead(p, l.v); l.pat = p; l.dirty = false; }
  // Slot 0 = cur_ (call whenever cur_ changes).
  void laneFollow() {
    if (lane_[0].pat == cur_) return;
    Lane t = lane_[0]; lane_[0] = lane_[1]; lane_[1] = t;   // the old current waits in slot 1
    if (lane_[0].pat != cur_) laneLoad(lane_[0], cur_);
  }
  // The lane of p for an edit (loads into slot 1 when it is not cached).
  Lane& laneFor(uint8_t p) {
    if (lane_[0].pat == p) return lane_[0];
    laneLoad(lane_[1], p);
    return lane_[1];
  }
 public:
  // Between notes (or stopped): preload the queued pattern's lane so the switch
  // on the bar is a swap. Called every main-loop pass.
  void laneService(bool quietNow) {
    if (queued_ < NUM_PATTERNS && queued_ != cur_ && lane_[1].pat != queued_ && quietNow) laneLoad(lane_[1], queued_);
  }
  uint8_t filter(uint8_t p, uint8_t i) {
    if (p >= NUM_PATTERNS || i >= MAX_STEPS) return 0;
    if (lane_[0].pat == p) return lane_[0].v[i];
    if (lane_[1].pat == p) return lane_[1].v[i];
    uint8_t tmp[MAX_STEPS]; laneRead(p, tmp); return tmp[i];
  }
  // The whole lane of p into dst (MAX_STEPS bytes), cached or from flash.
  void filterLane(uint8_t p, uint8_t* dst) {
    if (p >= NUM_PATTERNS) { memset(dst, 0, MAX_STEPS); return; }
    if (lane_[0].pat == p) memcpy(dst, lane_[0].v, MAX_STEPS);
    else if (lane_[1].pat == p) memcpy(dst, lane_[1].v, MAX_STEPS);
    else laneRead(p, dst);
  }
  // SysEx 0x16 lane 4: one step (setStep refuses a value on a rest, as the accent).
  bool setFilter(uint8_t p, uint8_t idx, uint8_t v) {
    if (p >= NUM_PATTERNS || idx >= len_[p] || v > 127) return false;
    Lane& l = laneFor(p);
    if (l.v[idx] != v) { l.v[idx] = v; l.dirty = true; laneChanged_ |= (uint16_t)(1u << p); }
    return true;
  }
  // SysEx 0x2D: the lane whole (n values from step 0; the rest 0).
  bool setFilterLane(uint8_t p, const uint8_t* v, uint8_t n) {
    if (p >= NUM_PATTERNS || n > MAX_STEPS) return false;
    Lane& l = laneFor(p);
    bool changed = false;
    for (uint8_t i = 0; i < MAX_STEPS; ++i) {
      uint8_t nv = i < n ? (uint8_t)(v[i] & 0x7F) : 0;
      if (l.v[i] != nv) { l.v[i] = nv; changed = true; }
    }
    if (changed) { l.dirty = true; laneChanged_ |= (uint16_t)(1u << p); }
    return true;
  }
  uint16_t consumeLaneChanged() { uint16_t c = laneChanged_; laneChanged_ = 0; return c; }
  // The playing step's filter value. A TIE (legato step) with none of its own
  // holds the value of the note it continues (walking back through the chain),
  // so a held note keeps one cutoff; a note with 0, and every rest (a value a
  // whole-lane write or a nudge left on one is ignored), releases to the CC.
  uint8_t filterNow() const {
    const Lane& l = lane_[0];
    if (l.pat != cur_) return 0;
    uint8_t len = effLen(); if (!len) return 0;
    uint8_t i = (uint8_t)(pos_ % len);
    for (uint8_t k = 0; k < len; ++k) {
      if (StepKind(step_[cur_][i]) == STEP_REST) return 0;
      if (l.v[i]) return l.v[i];
      if (StepKind(step_[cur_][i]) != STEP_LEGATO) return 0;
      i = (uint8_t)((i + len - 1) % len);
    }
    return 0;
  }
  bool accent(uint8_t p, uint8_t i) const { return (acc_[p < NUM_PATTERNS ? p : 0][i >> 3] >> (i & 7)) & 1; }
  bool accentCur() const { return accent(cur_, pos_); }
  void accentRaw(uint8_t p, uint8_t i, bool on) {
    if (on) acc_[p][i >> 3] |= (uint8_t)(1u << (i & 7)); else acc_[p][i >> 3] &= (uint8_t)~(1u << (i & 7));
  }
  // SysEx 0x1B SET RATCHET [pat, step, 0..2] (0 = off/1x). ack 2 when out of range.
  bool setRatchet(uint8_t p, uint8_t idx, uint8_t v) {
    if (p >= NUM_PATTERNS || idx >= len_[p] || v > RAT_MAX) return false;
    ratchetRaw(p, idx, v);
    dirty_ |= (uint16_t)(1u << p);
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
    started_ = false; halfSeen_ = false; cycle_ = 0; steps_ = 0;
  }
  void record(uint8_t cv, StepType t, bool acc = false) {
    if (wcur_ < MAX_STEPS) {
      ratchetRaw(cur_, wcur_, 0); accentRaw(cur_, wcur_, acc && t != STEP_REST);
      step_[cur_][wcur_++] = StepByte(cv, t);
      if (wcur_ > len_[cur_]) len_[cur_] = wcur_;
      dirty_ |= (uint16_t)(1u << cur_);
    }
  }
  // Panel LOAD mode: cycle the last recorded step's ratchet (UP button).
  void cycleLastRatchet(int8_t dir) {
    if (!wcur_) return;
    uint8_t i = (uint8_t)(wcur_ - 1);
    uint8_t r = ratchet(cur_, i);
    r = (uint8_t)((r + (dir > 0 ? 1 : RAT_MAX)) % (RAT_MAX + 1));
    ratchetRaw(cur_, i, r);
    dirty_ |= (uint16_t)(1u << cur_);
  }
  // Replace pattern p wholesale (SysEx 0x12). Marks dirty; flush() persists.
  // tail: optional packed ratchet bytes after the steps, then packed accent
  // bytes (old hosts send none, or ratchets only).
  bool set(uint8_t p, const uint8_t* steps, uint8_t len, const uint8_t* tail = nullptr, uint8_t tailBytes = 0) {
    if (p >= NUM_PATTERNS || len > MAX_STEPS) return false;
    memcpy(step_[p], steps, len);
    memset(rat_[p], 0, sizeof(rat_[p]));
    memset(acc_[p], 0, sizeof(acc_[p]));
    uint8_t rb = tailBytes > RatBytes(len) ? RatBytes(len) : tailBytes;
    if (tail && rb) memcpy(rat_[p], tail, rb);
    uint8_t ab = (uint8_t)(tailBytes - rb); if (ab > AccBytes(len)) ab = AccBytes(len);
    if (tail && ab) memcpy(acc_[p], tail + rb, ab);
    len_[p] = len;
    dirty_ |= (uint16_t)(1u << p);
    if (p == cur_ && pos_ >= len) rewind();
    return true;
  }
  const uint8_t* ratchets(uint8_t p) const { return rat_[p < NUM_PATTERNS ? p : 0]; }
  const uint8_t* accents(uint8_t p) const { return acc_[p < NUM_PATTERNS ? p : 0]; }

  // Live host edits (SysEx 0x16/0x18/0x15, docs/EDIT_OPCODES.md), atomic in place
  // so an edit under clock does not rewrite the whole pattern (gate G4). The 101
  // is mono: lane 0 = note (value = MIDI note, 0 = rest), lane 1 = accent (0/1,
  // not on a rest), lane 3 = gate (0 normal retrigger, 1 legato/tie); lane 2
  // (slide) does not exist here. Any
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
    } else if (lane == LANE_FILTER) {
      if (StepKind(s) == STEP_REST && value) return false;   // no filter CV on a rest (clearing it is fine)
      return setFilter(p, idx, value);
    } else if (lane == LANE_ACCENT) {
      if (StepKind(s) == STEP_REST || value > 1) return false;   // no accent on a rest
      accentRaw(p, idx, value);
    } else return false;                                     // lane 2 absent
    step_[p][idx] = s; dirty_ |= (uint16_t)(1u << p);
    if (p == cur_) stepChanged_ = true;
    return true;
  }
  bool setLength(uint8_t p, uint8_t len) {
    if (p >= NUM_PATTERNS || len < 1 || len > MAX_STEPS) return false;
    for (uint8_t i = len_[p]; i < len; ++i) { step_[p][i] = StepByte(0, STEP_REST); accentRaw(p, i, false); }
    len_[p] = len; dirty_ |= (uint16_t)(1u << p);
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
    dirty_ |= (uint16_t)(1u << p);
    if (p == cur_) stepChanged_ = true;
    return true;
  }

  // advance one step on a clock edge; returns true if a NON-legato note
  // started (caller pulses PORTAMENTO OFF).
  bool advance() {
    // Loop pass complete with a pattern switch waiting: switch now, and the
    // new pattern starts at its first step.
    if (started_ && queued_ != 0xFF && cycle_ == 0) {
      if (queued_ != cur_) { cur_ = queued_; loopLen_ = 0; selChanged_ = true; laneFollow(); }
      if (chainN_ > 1) { chainPos_ = (uint8_t)((chainPos_ + 1) % chainN_); queued_ = chain_[chainPos_]; }
      else queued_ = 0xFF;
    }
    applyRotate();                          // a ratcheted step has no half-step: land it here
    uint8_t len = effLen();
    if (len == 0) { curGate_ = false; return false; }
    const uint8_t cl = cycleLen();
    if (cycle_ >= cl) cycle_ = (uint8_t)(cycle_ % cl);
    pos_ = stepAt(cycle_); started_ = true; halfSeen_ = false;
    cycle_ = (uint8_t)((cycle_ + 1) % cl); ++steps_;
    if (DirSetting() == DIR_RANDOM) rndNext_ = (uint8_t)(Rnd8() % len);
    stepChanged_ = true;
    uint8_t s = playByte(pos_);
    switch (StepKind(s)) {
      case STEP_REST:   curGate_ = false; return false;
      case STEP_LEGATO: curCv_ = StepCv(s); curGate_ = true; return false;   // gate already on: no retrigger, glide
      default:          curCv_ = StepCv(s); curGate_ = true; return true;
    }
  }
  // Half-step (clock's other edge): close the gate so the next note
  // retriggers, unless the NEXT step is legato (then the gate carries over,
  // 303-style look-ahead).
  // --- nudge: rotate the pattern's notes one or more steps left / right ---
  // The playhead and the clock are untouched (timing stays locked); only the
  // notes move. On the playing pattern the rotation waits for the next
  // half-step: the sounding note has already started and the next step is
  // not chosen yet, so nothing is cut, doubled or re-triggered. Stopped (or
  // another pattern), it applies at once. n > 0 = right (later), n < 0 = left.
  void rotateNow(uint8_t p, int16_t n) {
    uint8_t len = len_[p];
    if (!len) return;
    int16_t k = (int16_t)(n % len); if (k < 0) k += len;
    if (!k) return;
    uint8_t tmp[MAX_STEPS], rt[MAX_STEPS];
    for (uint8_t i = 0; i < len; ++i) { tmp[i] = step_[p][i]; rt[i] = (uint8_t)(ratchet(p, i) | (accent(p, i) << 2)); }
    for (uint8_t i = 0; i < len; ++i) {
      uint8_t src = (uint8_t)((i + len - k) % len);
      step_[p][i] = tmp[src]; ratchetRaw(p, i, rt[src] & 3); accentRaw(p, i, rt[src] >> 2);
    }
    {   // the filter lane moves with its steps
      Lane& l = laneFor(p);
      bool any = false;
      for (uint8_t i = 0; i < len; ++i) { tmp[i] = l.v[i]; any |= tmp[i] != 0; }
      if (any) {
        for (uint8_t i = 0; i < len; ++i) l.v[i] = tmp[(uint8_t)((i + len - k) % len)];
        l.dirty = true; laneChanged_ |= (uint16_t)(1u << p);
      }
    }
    dirty_ |= (uint16_t)(1u << p); edited_ |= (uint16_t)(1u << p);
    if (p == cur_) stepChanged_ = true;
  }
  void rotate(uint8_t p, int16_t n, bool playing) {
    if (p >= NUM_PATTERNS || !n) return;
    if (playing && p == cur_) { rotPending_ = (int16_t)(rotPending_ + n); return; }
    rotateNow(p, n);
  }
  bool rotatePending() const { return rotPending_ != 0; }
  void applyRotate() { if (rotPending_) { int16_t n = rotPending_; rotPending_ = 0; rotateNow(cur_, n); } }
  void halfStep() {
    applyRotate();                          // nudge lands here: between notes, never mid-note
    halfSeen_ = true;
    uint8_t len = effLen();
    if (!len || !curGate_) return;
    uint8_t next = playByte(peekNext());
    if (StepKind(next) != STEP_LEGATO) curGate_ = false;
  }
  // A performance root or scale type overrides the pattern's own scale.
  uint8_t cv()   const {
    if (perf.root < 0 && perf.type < 0) return quantize(curCv_);
    return Snap(curCv_, ScalePreset(perf.type < 0 ? 0 : (uint8_t)perf.type, perf.root < 0 ? 0 : (uint8_t)perf.root));
  }
  // --- scale ---
  uint16_t scale(uint8_t p) const { return scale_[p < NUM_PATTERNS ? p : 0]; }
  bool scaleOn() const { return (scale_[cur_] & 0x8000) && (scale_[cur_] & 0xFFF); }
  void setScale(uint8_t p, uint16_t mask, bool on) {
    if (p >= NUM_PATTERNS) return;
    mask &= 0xFFF; if (!mask) mask = 0xFFF;
    scale_[p] = (uint16_t)(mask | (on ? 0x8000 : 0));
    dirty_ |= (uint16_t)(1u << p); scaleChanged_ |= (uint16_t)(1u << p);
  }
  uint16_t consumeScaleChanged() { uint16_t c = scaleChanged_; scaleChanged_ = 0; return c; }
  // cv -> nearest allowed note (MIDI = cv + 36, so C is cv 0, 12, 24 ...).
  uint8_t quantize(uint8_t cv, uint8_t p = 0xFF) const {
    uint16_t sc = scale_[p < NUM_PATTERNS ? p : cur_];
    if (!(sc & 0x8000)) return cv;
    return Snap(cv, sc & 0xFFF);
  }
  static uint8_t Snap(uint8_t cv, uint16_t m) {
    if (!m || ((m >> (cv % 12)) & 1)) return cv;
    for (int8_t d = 1; d <= 12; ++d) {
      int16_t lo = (int16_t)cv - d, hi = (int16_t)cv + d;
      if (lo >= 0 && ((m >> (lo % 12)) & 1)) return (uint8_t)lo;
      if (hi <= CV_MAX && ((m >> (hi % 12)) & 1)) return (uint8_t)hi;
    }
    return cv;
  }
  // --- musical randomizers (SuperOS-303 style; SysEx 0x68, panel) ---
  // ops: 0 full (rhythm + pitch), 1 pitches, 2 rhythm (note / rest / legato,
  // pitches kept), 3 mutate (2-4 small edits), 4 octaves, 5 legato flags.
  // Pitches: a semitone of the C4 octave and an octave weighted 20% down /
  // 60% centre / 20% up, snapped to the pattern's scale when it is on (so
  // written notes are in key). density 0..127 = share of note steps; 0xFF =
  // the 303's panel weights (60% note, 25% rest, 15% legato). seed 0 =
  // entropy; nonzero repeats exactly. Ratchets are kept.
  static const uint8_t RND_WEIGHTED = 0xFF;
  bool randomizeMusical(uint8_t p, uint8_t op, uint8_t density, uint16_t seed, uint16_t entropy) {
    if (p >= NUM_PATTERNS || op > 5) return false;
    uint16_t st = seed ? seed : (uint16_t)(entropy | 1);
    auto rnd = [&st](uint8_t n) -> uint8_t { st ^= st << 7; st ^= st >> 9; st ^= st << 8; return (uint8_t)(st % n); };
    auto pitch = [&]() -> uint8_t {
      uint8_t semi = rnd(13);                       // one draw per value, fixed order
      uint8_t o = rnd(10);
      int16_t cv = 24 + semi + (o < 2 ? -12 : (o >= 8 ? 12 : 0));
      return quantize((uint8_t)cv, p);
    };
    uint8_t len = len_[p];
    if (!len) { if (op != 0) return false; len = 16; setLength(p, 16); }
    if (op == 3) {                                   // mutate
      uint8_t passes = (uint8_t)(2 + rnd(3));
      for (uint8_t i = 0; i < passes; ++i) {
        uint8_t k = rnd(len), a = rnd(4), sb = step_[p][k];
        StepType t = StepKind(sb);
        if (a == 0 && t != STEP_REST) sb = StepByte(pitch(), t);
        else if (a == 1) sb = (t == STEP_REST) ? StepByte(pitch(), STEP_NOTE) : StepByte(StepCv(sb), STEP_REST);
        else if (a == 2 && t != STEP_REST) sb = StepByte(StepCv(sb), t == STEP_LEGATO ? STEP_NOTE : STEP_LEGATO);
        else if (a == 3 && t != STEP_REST) {
          int16_t c = (int16_t)StepCv(sb) + (rnd(2) ? 1 : -1);
          if (c < 0) c = 0; if (c > CV_MAX) c = CV_MAX;
          uint8_t q = quantize((uint8_t)c, p);
          if (q == StepCv(sb)) q = quantize((uint8_t)(c + (c > StepCv(sb) ? 1 : -1)), p);
          sb = StepByte(q, t);
        }
        step_[p][k] = sb;
      }
    } else {
      StepType prev = STEP_REST;
      for (uint8_t i = 0; i < len; ++i) {
        uint8_t sb = step_[p][i];
        StepType t = StepKind(sb);
        if (op == 0 || op == 2) {                    // rhythm
          if (density == RND_WEIGHTED) {
            uint8_t r = rnd(20);
            t = (i == 0 || prev == STEP_REST) ? (r < 12 ? STEP_NOTE : STEP_REST)
                                              : (r < 12 ? STEP_NOTE : (r < 17 ? STEP_REST : STEP_LEGATO));
          } else {
            uint8_t r = rnd(127);
            t = (r >= density) ? STEP_REST
                               : ((i && prev != STEP_REST && rnd(4) == 0) ? STEP_LEGATO : STEP_NOTE);
          }
          uint8_t cv = StepCv(sb);
          if (op == 0 && t != STEP_REST) cv = pitch();
          if (op == 2 && t != STEP_REST && StepKind(sb) == STEP_REST) cv = pitch();   // a new note needs a pitch
          sb = StepByte(cv, t);
        } else if (t != STEP_REST) {
          if (op == 1) sb = StepByte(pitch(), t);
          else if (op == 4) {                        // octave: keep the pitch class
            uint8_t o = rnd(10);
            int16_t c = (StepCv(sb) % 12) + 24 + (o < 2 ? -12 : (o >= 8 ? 12 : 0));
            sb = StepByte((uint8_t)c, t);
          } else if (op == 5 && i) sb = StepByte(StepCv(sb), rnd(4) == 0 ? STEP_LEGATO : STEP_NOTE);
        }
        step_[p][i] = sb;
        prev = StepKind(sb);
      }
    }
    dirty_ |= (uint16_t)(1u << p); edited_ |= (uint16_t)(1u << p);
    if (p == cur_) stepChanged_ = true;
    return true;
  }
  bool    gate() const { return curGate_; }

  // --- persistence (flash arena; payload [0]=len [1..len]=steps, then
  // RatBytes(len) packed ratchet bytes, 2 scale bytes, AccBytes(len) packed
  // accent bytes; records written before each tail existed end early and
  // load with it off) ---
  void loadAll() {
    uint8_t buf[MAX_STEPS + 1 + MAX_STEPS / 4 + 2 + MAX_STEPS / 8];
    for (uint8_t p = 0; p < NUM_PATTERNS; ++p) {
      len_[p] = 0;
      memset(rat_[p], 0, sizeof(rat_[p]));
      memset(acc_[p], 0, sizeof(acc_[p]));
      scale_[p] = 0x0FFF;
      if (!persist::Available()) continue;
      uint8_t n = persist::Store().read((uint16_t)(persist::FB_PATTERN_BASE + p), buf, sizeof(buf));
      if (n >= 1 && buf[0] <= MAX_STEPS && n >= (uint8_t)(1 + buf[0])) {
        len_[p] = buf[0];
        memcpy(step_[p], buf + 1, len_[p]);
        if (n >= (uint8_t)(1 + len_[p] + RatBytes(len_[p])))
          memcpy(rat_[p], buf + 1 + len_[p], RatBytes(len_[p]));
        // scale: 2 bytes after the ratchet tail (records from before scales
        // end at the tail and load as scale off, all notes)
        uint8_t so = (uint8_t)(1 + len_[p] + RatBytes(len_[p]));
        if (n >= (uint8_t)(so + 2)) {
          uint16_t sc = (uint16_t)(buf[so] | ((uint16_t)buf[so + 1] << 8));
          scale_[p] = (sc & 0xFFF) ? (uint16_t)(sc & 0x8FFF) : 0x0FFF;
        }
        if (n >= (uint8_t)(so + 2 + AccBytes(len_[p]))) memcpy(acc_[p], buf + so + 2, AccBytes(len_[p]));
      }
    }
    dirty_ = 0;
    lane_[0].pat = lane_[1].pat = 0xFF; lane_[0].dirty = lane_[1].dirty = false;
    laneFollow();
  }
  bool dirty() const { return dirty_ != 0 || lane_[0].dirty || lane_[1].dirty; }
  // Persist ONE dirty pattern (a page write halts the CPU for a few ms).
  // Returns true if something was written.
  bool flushOne() {
    if (!dirty_) {   // patterns first, then the filter lanes
      for (uint8_t k = 0; k < 2; ++k) if (lane_[k].dirty) { laneWrite(lane_[k]); return true; }
      return false;
    }
    for (uint8_t p = 0; p < NUM_PATTERNS; ++p) {
      if (!(dirty_ & (1u << p))) continue;
      dirty_ &= (uint16_t)~(1u << p);
      if (!persist::Available()) return false;
      uint8_t buf[MAX_STEPS + 1 + MAX_STEPS / 4 + 2 + MAX_STEPS / 8];
      buf[0] = len_[p];
      memcpy(buf + 1, step_[p], len_[p]);
      memcpy(buf + 1 + len_[p], rat_[p], RatBytes(len_[p]));
      uint8_t so = (uint8_t)(1 + len_[p] + RatBytes(len_[p]));
      buf[so] = (uint8_t)(scale_[p] & 0xFF); buf[so + 1] = (uint8_t)(scale_[p] >> 8);
      memcpy(buf + so + 2, acc_[p], AccBytes(len_[p]));
      persist::Store().write((uint16_t)(persist::FB_PATTERN_BASE + p), buf, (uint8_t)(so + 2 + AccBytes(len_[p])));
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
  // KEY TRANSPOSE, stock (OM p.36): hold KEY TRANSPOSE and press a key; the
  // offset from the lowest C (key 7) transposes everything that plays and
  // STAYS (LED lit); KEY TRANSPOSE + the lowest C returns to normal.
  // Added: a DOUBLE TAP enters transpose mode (LED blinks): in sequencer play
  // and in a HOLD-latched arp, plain key presses transpose without holding
  // the button. Double tap again to leave it (the transpose itself stays).
  int8_t   transpose_ = 0;
  bool     ktLatched_ = false;     // transpose mode (double tap)
  uint32_t ktLastPressMs_ = 0;
  uint8_t  songBuf_[Sequencer::CHAIN_MAX];
  uint8_t  songN_ = 0;             // keys collected while PLAY is held
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
  // Ratchet playback: the step's extra hits split its clock period evenly.
  // Under MIDI clock the period is the smoothed tempo (one division of the
  // uClock timebase) and, when the division divides evenly (1/16 = 6 ticks:
  // 2x on tick 3, 3x on ticks 2 and 4), each hit fires ON its clock tick, so
  // ratchets sit on the host's grid at any BPM. On the LFO / EXT CLK the
  // period is the measured edge spacing (the first step after play start has
  // no period yet and plays single).
  uint32_t lastEdgeUs_ = 0;
  uint32_t periodUs_   = 0;
  uint32_t stepStartUs_ = 0;
  uint8_t  subTotal_ = 1, subLeft_ = 0;
  bool     subOnTicks_ = false;

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
  // Transpose target: a fresh key sets the offset from the lowest C.
  // A key used for transposing also closes the double-tap window: press KT,
  // transpose, press KT again at once is two transposes, not transpose mode.
  void setTransposeKey(int8_t k) { transpose_ = (int8_t)(k - TRANSPOSE_REF_KEY); ktLastPressMs_ = 0; }
  void selectMode(controls::Buttons& b, uint32_t rawKeys) {
    using namespace controls;
    const uint32_t nowMs = millis();
    // Tune mode: KEY TRANSPOSE + LOAD toggles; any other function press leaves.
    if (tune_) {
      if (b.now & ~b.prev & ~(1u << BTN_KEYTRANS)) tune_ = false;
      return;
    }
    const bool kt = b.down(BTN_KEYTRANS);
    if (kt && b.pressed(BTN_LOAD) && mode_ == MODE_KEYBOARD) { tune_ = true; return; }
    // KEY TRANSPOSE double tap = transpose mode on/off (not in LOAD, where it
    // is the rest button, nor while real-time recording).
    if (b.pressed(BTN_KEYTRANS) && mode_ != MODE_SEQ_LOAD && !rtRec_ && !b.down(BTN_PLAY)) {
      if (ktLastPressMs_ && nowMs - ktLastPressMs_ < 400) { ktLatched_ = !ktLatched_; ktLastPressMs_ = 0; }
      else ktLastPressMs_ = nowMs;
    }
    const bool playHeld = b.down(BTN_PLAY) && mode_ != MODE_SEQ_LOAD;
    const bool downUd = (b.pressed(BTN_DOWN) && b.down(BTN_UD)) || (b.pressed(BTN_UD) && b.down(BTN_DOWN));
    if (mode_ == MODE_SEQ_LOAD) {
      // While recording: UP cycles the last step's ratchet (1x -> 2x -> 3x),
      // DOWN takes the last step back (SH-1oh1).
      if (b.pressed(BTN_UP))   seq_.cycleLastRatchet(+1);
      if (b.pressed(BTN_DOWN)) seq_.deleteLast();
    } else if (playHeld) {
      // PLAY held: UP / DOWN / U&D belong to the randomizers (below).
    } else if (mode_ == MODE_SEQ_PLAY && b.down(BTN_HOLD)) {
      // Nudge: HOLD + UP / DOWN while playing moves the notes one step right
      // / left (lands at the next half-step, never mid-note).
      if (b.pressed(BTN_UP))   seq_.rotate(seq_.current(), +1, true);
      if (b.pressed(BTN_DOWN)) seq_.rotate(seq_.current(), -1, true);
    } else if (mode_ == MODE_SEQ_PLAY && kt) {
      // Shift (SH-1oh1): KEY TRANSPOSE + UP / DOWN plays the next / previous
      // step now, moving the pattern against the clock.
      if (b.pressed(BTN_UP))   nudge_ = +1;
      if (b.pressed(BTN_DOWN)) nudge_ = -1;
    } else if (IsArp(mode_) && kt) {
      // Arp octave range: KEY TRANSPOSE + UP / DOWN = one octave more / less
      // (1..3). Persists.
      if (b.pressed(BTN_UP)   && ArpRange() < 3) { ++ArpRange(); FeatDirty() = true; }
      if (b.pressed(BTN_DOWN) && ArpRange() > 1) { --ArpRange(); FeatDirty() = true; }
    } else if (mode_ == MODE_SEQ_PLAY) {
      // Direction while playing (SH-1oh1): UP forward, DOWN backward, U&D
      // ping-pong, DOWN + U&D together random.
      if (downUd)                   seq_.setDirection(DIR_RANDOM);
      else if (b.pressed(BTN_UP))   seq_.setDirection(DIR_FWD);
      else if (b.pressed(BTN_DOWN)) seq_.setDirection(DIR_BWD);
      else if (b.pressed(BTN_UD))   seq_.setDirection(DIR_PINGPONG);
    } else {
      // Arp (stock: the same button again = off); DOWN + U&D together =
      // random (SH-1oh1), again = off.
      // The pair's first press already switched the mode, so the toggle
      // decides from the mode BEFORE that press.
      if (downUd) mode_ = (preArp_ == MODE_ARP_RANDOM) ? MODE_KEYBOARD : MODE_ARP_RANDOM;
      else {
        if (b.now & ~b.prev & ((1u << BTN_UP) | (1u << BTN_DOWN) | (1u << BTN_UD))) preArp_ = mode_;
        if (b.pressed(BTN_UP))   mode_ = (mode_ == MODE_ARP_UP)     ? MODE_KEYBOARD : MODE_ARP_UP;
        if (b.pressed(BTN_DOWN)) mode_ = (mode_ == MODE_ARP_DOWN)   ? MODE_KEYBOARD : MODE_ARP_DOWN;
        if (b.pressed(BTN_UD))   mode_ = (mode_ == MODE_ARP_UPDOWN) ? MODE_KEYBOARD : MODE_ARP_UPDOWN;
      }
    }
    // PLAY held + keys 1-16 (SH-1oh1): one key selects a pattern, several in
    // order make a song. On release: stopped -> play it from the start;
    // playing -> it follows at the end of the current loop pass. A plain
    // PLAY press/release toggles play.
    if (b.pressed(BTN_PLAY)) { playHeldUsed_ = false; songN_ = 0; }
    if (playHeld) {
      uint32_t fresh = rawKeys & ~prevRaw_ & 0xFFFF;   // keys 1-16; own edge memory: the held-PLAY keys never reach prevMask_
      if (!kt)
        for (uint8_t k = 0; k < NUM_PATTERNS; ++k)
          if ((fresh & (1UL << k)) && songN_ < Sequencer::CHAIN_MAX) { songBuf_[songN_++] = k; playHeldUsed_ = true; }
      // PLAY held + U&D / UP / DOWN: randomize the pattern (rhythm + pitch) /
      // its pitches / mutate it (SuperOS-303 style, in the pattern's scale).
      const uint16_t ent = (uint16_t)micros();
      if (b.pressed(BTN_UD))   { seq_.randomizeMusical(seq_.current(), 0, Sequencer::RND_WEIGHTED, 0, ent); playHeldUsed_ = true; }
      if (b.pressed(BTN_UP))   { seq_.randomizeMusical(seq_.current(), 1, Sequencer::RND_WEIGHTED, 0, ent); playHeldUsed_ = true; }
      if (b.pressed(BTN_DOWN)) { seq_.randomizeMusical(seq_.current(), 3, Sequencer::RND_WEIGHTED, 0, ent); playHeldUsed_ = true; }
      // Scale: PLAY held + KEY TRANSPOSE + a key = that note as the root
      // (Major first; the same key again steps through the 35 scales); PLAY
      // held + a KEY TRANSPOSE tap alone = scale on / off.
      if (kt) {
        uint32_t f = rawKeys & ~prevRaw_;
        if (f) {
          uint8_t root = (uint8_t)((lowestKey(f) + 5) % 12);       // key 1 = F
          scalePreset_ = (root == scaleRoot_ && scaleKeyUsed_) ? (uint8_t)((scalePreset_ + 1) % NUM_SCALES) : 0;
          scaleRoot_ = root; scaleKeyUsed_ = true;
          seq_.setScale(seq_.current(), ScalePreset(scalePreset_, root), true);
          playHeldUsed_ = true;
        }
      }
      if (b.pressed(BTN_KEYTRANS)) scaleKeyUsed_ = false;
      if (b.released(BTN_KEYTRANS) && !scaleKeyUsed_) {
        uint16_t sc = seq_.scale(seq_.current());
        seq_.setScale(seq_.current(), sc & 0xFFF, !(sc & 0x8000));
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
    if (b.released(BTN_PLAY)) {
      if (songN_) {
        if (mode_ == MODE_SEQ_PLAY) seq_.setChain(songBuf_, songN_, false);
        else { seq_.setChain(songBuf_, songN_, true); startPlay(); }
        songN_ = 0;
      } else if (!playHeldUsed_) {
        if (mode_ == MODE_SEQ_PLAY) stopPlay();
        else startPlay();
      }
    }
    // LOAD: from stop, record a new pattern (stock: clears). While playing,
    // stock "Correcting and Adding": keep the pattern, re-load from the step
    // after the one sounding. LOAD again stores.
    if (b.pressed(BTN_LOAD) && !b.down(BTN_PLAY) && !kt) {
      if (mode_ == MODE_SEQ_LOAD) { seq_.flushAll(); seq_.markEdited(); mode_ = MODE_KEYBOARD; }
      else if (mode_ == MODE_SEQ_PLAY) { rtRec_ = false; seq_.beginOverwrite(); mode_ = MODE_SEQ_LOAD; }
      else { rtRec_ = false; seq_.clear(); mode_ = MODE_SEQ_LOAD; }
    }
  }
  // --- performance modes -------------------------------------------------
  // While the sequencer plays (not recording, no function button held, not in
  // transpose mode) the keys are the overlay strip above them and the
  // TRANSPOSE switch picks the row: L = 1 Global, M = 2 Pitch, H = 3 Time.
  //   1  keys 1-12 transpose (C = none) | 13-19 octave variation 1-7 |
  //      20-24 LFO/S&H sync 1/16 1/8 1/4 1/2 1/1 | 25-32 patterns 1-8 (now)
  //   2  1-7 reorder notes (as written, reverse, cycle even, cycle odd,
  //      prime, one forward, one back) | 8-19 quantizer root C..B |
  //      20-26 scale major minor dorian mixolydian lydian phrygian locrian |
  //      27-31 combine pitch with the next pattern: add sub mul div diff | 32 reset
  //   3  1-7 shift time 1/4 R, 1/4 L, 1/8 R, 1/8 L, 1/16 R, 1/16 L, 3/16 R |
  //      8-12 skip every 4th, every 8th, even, odd, every 5th clock |
  //      13-17 add/delete grid 1/4 1/8 1/16 1/4T 1/16T, 18-21 its logic NOR OR
  //      AND XOR | 22-26 ratchet while held: 1/32 1/16 1/16T 3x 7x |
  //      27-31 combine rhythm with the next pattern: AND OR NOR XOR NAND | 32 reset
  // A latched choice pressed again turns off; reset + a key turns that
  // function off; reset alone clears the row. Time values count one step as
  // a 16th; the add/delete grid is the MIDI clock when it drives the 101,
  // else 24 ppqn derived from the measured step period.
  uint32_t perfKeys_ = 0;          // keys consumed by the overlay, until released
  bool     pResetUsed_ = false;    // reset key combined with another key
  uint8_t  pSkip_ = 0, pGrid_ = 0, pLogic_ = 0, pRoll_ = 0, pLfo_ = 0;
  uint8_t  pLfoPrevDiv_ = 0; bool pLfoPrevSync_ = true;
  uint16_t pClk_ = 0;              // step clock edges since play start (skip)
  bool     pA_ = false, pC_ = false;   // add/delete: step clock level, combined level
  uint32_t pAnchorUs_ = 0;         // derived grid origin (play start)
  bool     rollOn_ = false; uint8_t rollCv_ = 0; uint32_t rollNextUs_ = 0, rollHitUs_ = 0;

  static uint8_t perfRow(const controls::Panel& p) { return p.octaveOffset == 0x00 ? 1 : (p.octaveOffset == 0x0C ? 2 : 3); }
  void setPerfLfo(uint8_t v) {
    static const uint8_t DIV[5] = {6, 3, 0, 10, 11};   // midiclock LFO divisions 1/16 1/8 1/4 1/2 1/1
    if (v && !pLfo_) { pLfoPrevDiv_ = midiclk::LfoDivIdx(); pLfoPrevSync_ = midiclk::LfoSyncOn(); }
    if (v) { midiclk::SetLfoDiv(DIV[v - 1], false); midiclk::SetLfoSync(true, false); }
    else if (pLfo_) { midiclk::SetLfoDiv(pLfoPrevDiv_, false); midiclk::SetLfoSync(pLfoPrevSync_, false); }
    pLfo_ = v;
  }
  void perfResetRow(uint8_t row) {
    Sequencer::Perf& f = seq_.perf;
    if (row == 2) { f.reorder = 0; f.root = -1; f.type = -1; f.pcomb = 0; }
    if (row == 3) { f.shift = 0; pSkip_ = 0; pGrid_ = 0; pLogic_ = 0; f.tcomb = 0; }
  }
  void perfKeys(uint32_t mask, const controls::Panel& p) {
    static const int8_t SHIFT[7] = {4, -4, 2, -2, 1, -1, 3};
    const uint8_t row = perfRow(p);
    Sequencer::Perf& f = seq_.perf;
    const uint32_t fresh = mask & ~prevMask_ & ~perfKeys_;
    perfKeys_ |= fresh;
    const bool rst = row > 1 && (mask & (1UL << 31));
    if (fresh & (1UL << 31)) pResetUsed_ = false;
    // A latched option: the same again (or reset + it) = off.
    auto pick = [rst](uint8_t cur, uint8_t v) -> uint8_t { return (rst || cur == v) ? 0 : v; };
    auto pickS = [rst](int8_t cur, int8_t v) -> int8_t { return (rst || cur == v) ? -1 : v; };
    for (uint8_t k = 0; k < 32; ++k) {
      if (!(fresh & (1UL << k)) || (row > 1 && k == 31)) continue;
      if (rst) pResetUsed_ = true;
      if (row == 1) {
        if (k < 12)       transpose_ = (int8_t)(k - TRANSPOSE_REF_KEY);
        else if (k < 19)  f.octVar = pick(f.octVar, (uint8_t)(k - 11));
        else if (k < 24)  setPerfLfo(pick(pLfo_, (uint8_t)(k - 18)));
        else              seq_.switchNow((uint8_t)(k - 24));
      } else if (row == 2) {
        if (k < 7)        f.reorder = pick(f.reorder, k);
        else if (k < 19)  f.root = pickS(f.root, (int8_t)((k + 5) % 12));    // key 1 = F
        else if (k < 26)  f.type = pickS(f.type, (int8_t)(k - 19));
        else              f.pcomb = pick(f.pcomb, (uint8_t)(k - 25));
      } else {
        if (k < 7)        f.shift = (rst || f.shift == SHIFT[k]) ? 0 : SHIFT[k];
        else if (k < 12)  pSkip_ = pick(pSkip_, (uint8_t)(k - 6));
        else if (k < 17)  pGrid_ = pick(pGrid_, (uint8_t)(k - 11));
        else if (k < 21)  pLogic_ = pick(pLogic_, (uint8_t)(k - 16));
        else if (k < 26)  {}                                                 // ratchet: held, below
        else              f.tcomb = pick(f.tcomb, (uint8_t)(k - 25));
      }
    }
    const uint32_t rk = row == 3 ? (mask & (0x1FUL << 21)) : 0;
    pRoll_ = rk ? (uint8_t)(lowestKey(rk) - 20) : 0;
  }
  // Skip and add/delete rewrite the step clock before the sequencer sees it.
  void perfClock(bool& edge, bool& half, bool clocked) {
    if (pGrid_ || pLogic_) {
      // The step clock as a square (high from its edge to its half-step)
      // through a logic gate with a fixed grid clock; the result's rising
      // edge steps, its falling edge is the half-step.
      static const uint8_t GRID[5] = {24, 12, 6, 16, 4};   // 1/4 1/8 1/16 1/4T 1/16T at 24 ppqn
      if (edge) pA_ = true;
      if (half) pA_ = false;
      const uint8_t g = GRID[(pGrid_ ? pGrid_ : 1) - 1];
      bool b = false;
      if (clocked) { uint32_t t = midiclk::AbsTicks(); b = t && ((t - 1) % g) < g / 2; }
      else if (periodUs_ >= 6) b = ((micros() - pAnchorUs_) / (periodUs_ / 6)) % g < g / 2u;
      bool c;
      switch (pLogic_ ? pLogic_ : 2) {
        case 1:  c = !(pA_ || b); break;
        case 2:  c = pA_ || b; break;
        case 3:  c = pA_ && b; break;
        default: c = pA_ != b; break;
      }
      edge = c && !pC_; half = !c && pC_; pC_ = c;
    }
    if (edge && pSkip_) {
      const uint16_t n = pClk_++;
      bool skip;
      switch (pSkip_) {
        case 1:  skip = n % 4 == 3; break;
        case 2:  skip = n % 8 == 7; break;
        case 3:  skip = n % 2 == 1; break;
        case 4:  skip = n % 2 == 0; break;
        default: skip = n % 5 == 4; break;
      }
      if (skip) { edge = false; seq_.mute(); subLeft_ = 0; }
    }
  }
  int8_t nudge_ = 0;
  uint8_t scaleRoot_ = 0, scalePreset_ = 0;
  bool scaleKeyUsed_ = false;
  Mode preArp_ = MODE_KEYBOARD;
  uint32_t prevRaw_ = 0;
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
    tune_ = false; rtRec_ = false; rtKey_ = -1; nudge_ = 0; songN_ = 0;
    playHeldUsed_ = false; playStart_ = false; playNow_ = false; prevExtCv_ = -1;
    lastEdgeUs_ = 0; periodUs_ = 0; stepStartUs_ = 0; subTotal_ = 1; subLeft_ = 0; subOnTicks_ = false;
    setPerfLfo(0); perfKeys_ = 0; pResetUsed_ = false; pSkip_ = pGrid_ = pLogic_ = pRoll_ = 0;
    pClk_ = 0; pA_ = pC_ = false; rollOn_ = false;
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
  // SysEx 0x69 / editor nudge: rotate pattern p (queued to the next
  // half-step when it is the one playing).
  void   nudgePattern(uint8_t p, int16_t n) {
    if (mode_ == MODE_SEQ_LOAD) return;
    seq_.rotate(p, n, mode_ == MODE_SEQ_PLAY);
    if (mode_ != MODE_SEQ_PLAY && p == seq_.current()) seq_.applyRotate();
  }
  // Resync (editor button, SysEx 0x65 id 9): put the sequence back on the
  // one. Under MIDI clock that is the host's grid since its Start; otherwise
  // the steps counted since PLAY. Also undoes a shift.
  void   resync() {
    if (mode_ != MODE_SEQ_PLAY) return;
    seq_.realign(midiclk::Driving(millis()) ? midiclk::DivisionsSinceStart() : seq_.stepsPlayed());
  }
  // The clock division changed mid-play: the grid did not move (midiclock.h),
  // so re-derive the pattern position from the divisions since Start.
  void   divisionChanged() { if (mode_ == MODE_SEQ_PLAY && midiclk::Driving(millis())) seq_.realign(midiclk::DivisionsSinceStart()); }
  void   setRtRec(bool on) {
    if (mode_ != MODE_SEQ_PLAY) return;
    if (on && !rtRec_ && seq_.length() == 0) { seq_.setLength(seq_.current(), 16); seq_.rewind(); }
    if (!on && rtRec_) seq_.markEdited();
    rtRec_ = on; rtKey_ = -1;
  }
  // KEY-TRANSPOSE LED: lit while a transpose is active (stock); blinks in
  // transpose mode.
  bool   ktLed() const { return ktLatched_ ? ((millis() >> 7) & 1) : transpose_ != 0; }
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
    prevRaw_ = mask;
    if (tune_) {
      // A at the panel's octave: key 16 = A4 = 440 Hz at 8', TRANSPOSE middle.
      Output t{noteCv(16, p), true, false, !prevTune_, false, false, 0};
      prevTune_ = true; prevMask_ = mask; prevExtCv_ = extCv; lastCv_ = t.cv;
      return t;
    }
    if (prevTune_) { prevTune_ = false; prevMask_ = mask; }   // no key edge out of tune mode
    if (b.down(controls::BTN_PLAY) && mode_ != MODE_SEQ_LOAD) {
      mask &= ~0xFFFFUL;                                // pattern keys 1-16 are not notes
      if (b.down(controls::BTN_KEYTRANS)) mask = 0;     // scale-root keys are not notes either
    }
    // Performance modes: the keys are the overlay while the sequencer plays.
    {
      const bool on = PerfOn() && mode_ == MODE_SEQ_PLAY && !rtRec_ && !b.now && !ktLatched_;
      if (on && perfRow(p) > 1 && (perfKeys_ & (1UL << 31)) && !(mask & (1UL << 31)) && !pResetUsed_)
        perfResetRow(perfRow(p));                       // reset tapped alone
      if (on) perfKeys(mask, p); else pRoll_ = 0;
      perfKeys_ &= mask;
      mask &= ~perfKeys_;
    }
    // KEY TRANSPOSE held (any mode but LOAD / real-time record): a fresh key
    // sets the transpose instead of playing. In transpose mode (double tap),
    // plain keys do the same in sequencer play and in a HOLD-latched arp.
    // Keys used this way stay out of the voice until released.
    {
      const bool ktDown = b.down(controls::BTN_KEYTRANS) && mode_ != MODE_SEQ_LOAD && !rtRec_ && !b.down(controls::BTN_PLAY);
      const bool modeKeys = ktLatched_ && ((mode_ == MODE_SEQ_PLAY && !rtRec_) || (IsArp(mode_) && hold && holdMask_));
      if (ktDown || modeKeys) {
        uint32_t fresh = mask & ~prevMask_ & ~ktKeys_ & ~holdMask_;
        if (fresh) setTransposeKey(lowestKey(fresh));
        ktKeys_ |= fresh;
      }
      ktKeys_ &= mask;
      mask &= ~ktKeys_;
    }
    // Instant pattern switch (SH-1oh1 3.5.3): keys 1-16 while playing, with
    // nothing held, switch now and keep the position.
    if (mode_ == MODE_SEQ_PLAY && !rtRec_ && !b.down(controls::BTN_HOLD) && !b.down(controls::BTN_PLAY)) {
      uint32_t fresh = mask & ~prevMask_ & 0xFFFF;
      if (fresh) seq_.switchNow((uint8_t)lowestKey(fresh));
    }
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

    Output o{lastCv_, false, false, false, false, false, 0};
    switch (mode_) {
      case MODE_ARP_UP: case MODE_ARP_DOWN: case MODE_ARP_UPDOWN: case MODE_ARP_RANDOM:
        // CLOCK RESET only when the arp starts (first key after silence), so
        // the LFO clock is not restarted on every step: a per-step reset made
        // the step spacing depend on the LFO phase and sounded uneven.
        if (mask && !prevMask_) o.clockReset = true;
        if (clockEdge) {
          int8_t k = arp_.step(mask, mode_, ArpRange());
          if (k >= 0) {
            int16_t v = (int16_t)noteCv(k, p) + 12 * arp_.octave();
            lastCv_ = (uint8_t)(v > CV_MAX ? CV_MAX : v);
            arp_.gate = true; o.portaOff = true;
          }
        }
        if (halfEdge) arp_.gate = false;
        if (!mask) arp_.reset();
        o.cv = lastCv_;
        o.gate = (mask != 0) && arp_.gate;
        break;

      case MODE_SEQ_PLAY: {
        bool began = false;                             // a step started this pass
        if (playStart_) {
          o.clockReset = true; playStart_ = false;   // LFO in phase with step 1
          subTotal_ = 1; subLeft_ = 0; lastEdgeUs_ = 0;   // no stale ratchet state into a new run
          pClk_ = 0; pA_ = pC_ = false; pAnchorUs_ = micros();
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
              began = true;
              lastCv_ = seq_.cv();
              clockEdge = false; halfEdge = false;
            }
          }
        }
        if (nudge_) {                                   // shift: a step now, off the clock
          if (seq_.nudge(nudge_)) o.portaOff = true;
          began = true;
          lastCv_ = seq_.cv(); subTotal_ = 1; subLeft_ = 0;
          nudge_ = 0;
        }
        uint8_t tickIdx = 0;
        const bool tickNow = midiclk::ConsumeTick(tickIdx);    // consumed every pass: never stale
        const bool clocked = midiclk::Driving(millis());
        if (clockEdge) {   // the step period, from the step clock itself
          const uint32_t now = micros();
          if (clocked) periodUs_ = midiclk::DivPeriodUs();
          else if (lastEdgeUs_) { uint32_t d = now - lastEdgeUs_; if (d > 0 && d < 4000000UL) periodUs_ = d; }
          lastEdgeUs_ = now;
        }
        perfClock(clockEdge, halfEdge, clocked);
        if (clockEdge) {
          const uint32_t now = micros();
          bool fresh = seq_.advance();
          if (fresh) o.portaOff = true;
          began = true;
          lastCv_ = seq_.cv();
          // Ratchet: a NOTE step (fresh) with ratchet r plays r extra evenly
          // spaced hits inside this period; needs a period first.
          uint8_t r = (fresh && periodUs_) ? seq_.ratchetCur() : 0;
          subTotal_ = (uint8_t)(r + 1);
          subLeft_  = r;
          subOnTicks_ = clocked && (midiclk::DivTicks() % subTotal_) == 0;
          stepStartUs_ = now;
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
        if (subLeft_ && periodUs_ && seq_.gate()) {
          const uint32_t now = micros();
          const uint8_t  done = (uint8_t)(subTotal_ - subLeft_);
          const uint32_t at = stepStartUs_ + periodUs_ * done / subTotal_;
          uint32_t gap = periodUs_ / (subTotal_ * 4u);
          if (gap < 2000) gap = 2000; else if (gap > 12000) gap = 12000;
          // On the clock: the hit IS the tick at done/subTotal of the division
          // (with a late-tick backstop); otherwise the time.
          const bool hit = subOnTicks_
              ? ((tickNow && tickIdx == (uint8_t)(midiclk::DivTicks() * done / subTotal_)) ||
                 (int32_t)(now - (at + gap)) >= 0)
              : (int32_t)(now - at) >= 0;
          if (hit) { --subLeft_; o.portaOff = true; }                        // sub-hit: fresh attack
          else if ((int32_t)(now - (at - gap)) >= 0) o.gate = false;          // gate gap before it
        }
        // Performance ratchet (held): the note sounding at the press repeats
        // at the chosen rate while the pattern runs on underneath.
        if (pRoll_ && periodUs_) {
          static const uint8_t RN[5] = {1, 1, 2, 1, 1}, RD[5] = {2, 1, 3, 3, 7};   // of a step: 1/32 1/16 1/16T 3x 7x
          const uint32_t iv = periodUs_ * RN[pRoll_ - 1] / RD[pRoll_ - 1];
          const uint32_t now = micros();
          if (!rollOn_) { rollOn_ = true; rollCv_ = o.cv; rollNextUs_ = now; }
          if ((int32_t)(now - rollNextUs_) >= 0) {
            rollHitUs_ = now; rollNextUs_ += iv;
            if ((int32_t)(now - rollNextUs_) >= 0) rollNextUs_ = now + iv;   // fell behind: no burst
            o.portaOff = true;
          }
          o.cv = rollCv_;
          o.gate = (uint32_t)(now - rollHitUs_) < iv / 2;
        } else rollOn_ = false;
        // LFO/S&H sync without the MIDI clock: CLOCK RESET every n steps
        // (with it, midiclock's own LFO division does this on the ticks).
        if (began && pLfo_ && !clocked) {
          static const uint8_t N[5] = {1, 2, 4, 8, 16};
          if ((uint16_t)(seq_.stepsPlayed() - 1) % N[pLfo_ - 1] == 0) o.clockReset = true;
        }
        // Accented step: fires at the step's start (legato ones too, 303
        // style) and on each ratchet sub-hit.
        o.accent = o.gate && (began || o.portaOff) && seq_.accentCur();
        if (began) { o.stepStart = true; o.filter = seq_.filterNow(); }
        break;
      }

      case MODE_SEQ_LOAD: {
        // record: KEY-TRANSPOSE = rest, HOLD-held = legato, U&D-held =
        // accent (both combine), else the note. (The fn-button mask guard
        // above keeps the LOAD press's own scan bleed from ever appearing as
        // a fresh key here.)
        if (b.pressed(controls::BTN_KEYTRANS)) seq_.record(lastCv_, STEP_REST);
        const StepType t = b.down(controls::BTN_HOLD) ? STEP_LEGATO : STEP_NOTE;
        const bool acc = b.down(controls::BTN_UD);
        uint32_t fresh = mask & ~prevMask_;
        if (fresh) { int8_t k = lowestKey(fresh); if (k >= 0) seq_.record(noteCv(k, p), t, acc); }
        if (extCv >= 0 && extCv != prevExtCv_) seq_.record((uint8_t)extCv, t, acc);
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

    if (transpose_ && mode_ != MODE_SEQ_LOAD) {   // stock: everything that plays
      int16_t v = (int16_t)o.cv + transpose_;
      o.cv = (uint8_t)(v < 0 ? 0 : (v > CV_MAX ? CV_MAX : v));
    }
    prevMask_ = mask;
    prevExtCv_ = extCv;
    return o;
  }
};

} // namespace engine
