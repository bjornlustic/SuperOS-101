// SuperOS-101 - note engine: keyboard / arpeggiator / sequencer
//
// Clean-room behavioural implementation matching the original's OBSERVABLE
// behaviour (this is our own firmware, see docs/FIRMWARE.md). It is NOT a
// transliteration of the 8049 code; where exact original semantics are
// uncertain (mode latching, tie/rest edge cases) it's marked TODO/verify.
//
//   - keyboard : mono, lowest- or last-note priority (panel-selected); MIDI
//                notes in the keyboard's range act as keys, notes outside it
//                play as a mono override in keyboard mode
//   - hold     : HOLD button / pedal latches the held keys (arp keeps running)
//   - arp      : UP / DOWN / UP&DOWN over held keys, advanced by the clock
//   - sequencer: 8 patterns x up to 100 steps (note / rest / tie), kept in RAM
//                and persisted in the flash arena (flash_persist.h); pattern
//                select = PLAY held + one of the 8 lowest keys, or SysEx 0x1D
#pragma once
#include <Arduino.h>
#include "controls.h"
#include "combined.h"
#include "flash_persist.h"

namespace engine {

static const uint8_t MAX_STEPS    = 100;
static const uint8_t NUM_PATTERNS = persist::NUM_PATTERNS;   // 8
static const uint8_t CV_BASE      = 5;   // cv_dac_val returns value+5 (SH101.asm:992)
static const uint8_t CV_MAX       = 63;  // 6-bit DAC
// MIDI note <-> CV: key 0 (F3) at the middle octave (0x0C) is MIDI 53, so
// cv = note - 36 (MIDI 36..99 covers the whole 6-bit DAC range).
static const uint8_t MIDI_CV_OFFSET = 36;

enum Mode : uint8_t {
  MODE_KEYBOARD, MODE_ARP_UP, MODE_ARP_DOWN, MODE_ARP_UPDOWN, MODE_SEQ_PLAY, MODE_SEQ_LOAD,
};
enum StepType : uint8_t { STEP_NOTE = 0, STEP_REST = 1, STEP_TIE = 2 };

// One step byte, the SH-101's own model: 6-bit CV + 2 type bits.
inline uint8_t  StepByte(uint8_t cv, StepType t) { return (uint8_t)((cv & 0x3F) | ((uint8_t)t << 6)); }
inline uint8_t  StepCv  (uint8_t s) { return s & 0x3F; }
inline StepType StepKind(uint8_t s) { uint8_t t = s >> 6; return t > 2 ? STEP_NOTE : (StepType)t; }

struct Output { uint8_t cv; bool gate; bool clockReset; };

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
  int8_t current() const { return cur; }
  void reset() { cur = -1; dir = 1; }
  int8_t step(uint32_t mask, Mode mode) {
    if (!mask) { cur = -1; return -1; }
    if (cur < 0 || !(mask & (1UL << cur))) {        // (re)start
      dir = (mode == MODE_ARP_DOWN) ? -1 : 1;
      cur = (mode == MODE_ARP_DOWN) ? highestKey(mask) : lowestKey(mask);
      return cur;
    }
    int8_t nxt;
    if (mode == MODE_ARP_UP) {
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
  uint8_t len_[NUM_PATTERNS];
  uint8_t dirty_ = 0;                 // bit p = pattern p differs from flash
  uint8_t cur_ = 0;                   // selected pattern
  uint8_t pos_ = 0;
  uint8_t curCv_ = CV_BASE; bool curGate_ = false;
  bool    stepChanged_ = false;
 public:
  Sequencer() { memset(len_, 0, sizeof(len_)); }
  uint8_t current() const { return cur_; }
  uint8_t length(uint8_t p) const { return p < NUM_PATTERNS ? len_[p] : 0; }
  uint8_t length() const { return len_[cur_]; }
  const uint8_t* steps(uint8_t p) const { return step_[p < NUM_PATTERNS ? p : 0]; }
  uint8_t position() const { return pos_; }
  bool consumeStepChanged() { bool c = stepChanged_; stepChanged_ = false; return c; }

  void select(uint8_t p) { if (p < NUM_PATTERNS && p != cur_) { cur_ = p; rewind(); } }
  void clear() { len_[cur_] = 0; pos_ = 0; dirty_ |= (uint8_t)(1u << cur_); }
  // rewind so the NEXT advance() plays step 0 (advance pre-increments)
  void rewind() { pos_ = len_[cur_] ? (uint8_t)(len_[cur_] - 1) : 0; }
  void record(uint8_t cv, StepType t) {
    if (len_[cur_] < MAX_STEPS) { step_[cur_][len_[cur_]++] = StepByte(cv, t); dirty_ |= (uint8_t)(1u << cur_); }
  }
  // Replace pattern p wholesale (SysEx 0x12). Marks dirty; flush() persists.
  bool set(uint8_t p, const uint8_t* steps, uint8_t len) {
    if (p >= NUM_PATTERNS || len > MAX_STEPS) return false;
    memcpy(step_[p], steps, len);
    len_[p] = len;
    dirty_ |= (uint8_t)(1u << p);
    if (p == cur_ && pos_ >= len) rewind();
    return true;
  }

  // advance one step on a clock edge; returns true if a fresh note triggered.
  bool advance() {
    uint8_t len = len_[cur_];
    if (len == 0) { curGate_ = false; return false; }
    if (++pos_ >= len) pos_ = 0;
    stepChanged_ = true;
    uint8_t s = step_[cur_][pos_];
    switch (StepKind(s)) {
      case STEP_REST: curGate_ = false; return false;
      case STEP_TIE:  curGate_ = true;  return false;       // hold prev cv, no retrigger
      default:        curCv_ = StepCv(s); curGate_ = true; return true;
    }
  }
  uint8_t cv()   const { return curCv_; }
  bool    gate() const { return curGate_; }

  // --- persistence (flash arena; payload [0]=len [1..]=steps) ---
  void loadAll() {
    uint8_t buf[MAX_STEPS + 1];
    for (uint8_t p = 0; p < NUM_PATTERNS; ++p) {
      len_[p] = 0;
      if (!persist::Available()) continue;
      uint8_t n = persist::Store().read((uint16_t)(persist::FB_PATTERN_BASE + p), buf, sizeof(buf));
      if (n >= 1 && buf[0] <= MAX_STEPS && n >= (uint8_t)(1 + buf[0])) {
        len_[p] = buf[0];
        memcpy(step_[p], buf + 1, len_[p]);
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
      uint8_t buf[MAX_STEPS + 1];
      buf[0] = len_[p];
      memcpy(buf + 1, step_[p], len_[p]);
      persist::Store().write((uint16_t)(persist::FB_PATTERN_BASE + p), buf, (uint8_t)(len_[p] + 1));
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
  uint8_t  keyShift_ = 0;      // KEY TRANSPOSE (TODO: SH101.asm:844-866)
  bool     playHeldUsed_ = false;  // PLAY+key selected a pattern (suppress toggle)
  int16_t  prevExtCv_ = -1;

  uint8_t noteCv(int8_t key, const controls::Panel& p) const {
    int v = key + (int)p.octaveOffset + keyShift_ + CV_BASE;
    return (uint8_t)(v < 0 ? 0 : (v > CV_MAX ? CV_MAX : v));
  }
  int8_t pickKey(uint32_t mask, const controls::Panel& p) {
    if (!mask) return -1;
    uint32_t fresh = mask & ~prevMask_;
    if (p.lastNotePriority && fresh)            heldKey_ = lowestKey(fresh);
    else if (heldKey_ >= 0 && (mask & (1UL << heldKey_))) { /* keep */ }
    else                                         heldKey_ = lowestKey(mask);
    return heldKey_;
  }
  void selectMode(controls::Buttons& b, uint32_t rawKeys) {
    using namespace controls;
    if (b.pressed(BTN_UP))   mode_ = (mode_ == MODE_ARP_UP)     ? MODE_KEYBOARD : MODE_ARP_UP;
    if (b.pressed(BTN_DOWN)) mode_ = (mode_ == MODE_ARP_DOWN)   ? MODE_KEYBOARD : MODE_ARP_DOWN;
    if (b.pressed(BTN_UD))   mode_ = (mode_ == MODE_ARP_UPDOWN) ? MODE_KEYBOARD : MODE_ARP_UPDOWN;
    // PLAY: held + one of the 8 lowest keys selects the pattern; a plain
    // press/release toggles sequencer play (toggle happens on release so the
    // select gesture does not also start/stop the sequencer).
    if (b.pressed(BTN_PLAY)) playHeldUsed_ = false;
    if (b.down(BTN_PLAY) && mode_ != MODE_SEQ_LOAD) {
      uint32_t fresh = rawKeys & ~prevMask_ & 0xFF;
      if (fresh) { seq_.select((uint8_t)lowestKey(fresh)); playHeldUsed_ = true; }
    }
    if (b.released(BTN_PLAY) && !playHeldUsed_)
      mode_ = (mode_ == MODE_SEQ_PLAY) ? MODE_KEYBOARD : MODE_SEQ_PLAY;
    if (b.pressed(BTN_LOAD)) {
      if (mode_ == MODE_SEQ_LOAD) { seq_.flushAll(); mode_ = MODE_KEYBOARD; }
      else { seq_.clear(); mode_ = MODE_SEQ_LOAD; }
    }
  }
 public:
  Mode mode() const { return mode_; }
  Sequencer& seq() { return seq_; }
  void begin() { seq_.loadAll(); }
  // MIDI Start: align the sequence and arpeggiator to the incoming downbeat.
  void restart() { seq_.rewind(); arp_.reset(); }
  // SysEx 0x1D / editor: select a pattern (ignored while recording).
  void selectPattern(uint8_t p) { if (mode_ != MODE_SEQ_LOAD) seq_.select(p); }
  void setPlaying(bool on) {
    if (mode_ == MODE_SEQ_LOAD) return;
    if (on && mode_ != MODE_SEQ_PLAY) { mode_ = MODE_SEQ_PLAY; seq_.rewind(); }
    else if (!on && mode_ == MODE_SEQ_PLAY) mode_ = MODE_KEYBOARD;
  }
  // Function-LED select for the current mode (4556 one-hot, SH101.asm:90-128):
  // 0 none, 1 LOAD, 2 PLAY, 3 DOWN, 4 UP&DOWN, 5 UP.
  uint8_t ledSelect() const {
    switch (mode_) {
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
                controls::Buttons& b, bool clockEdge) {
    selectMode(b, mask);
    if (b.down(controls::BTN_PLAY) && mode_ != MODE_SEQ_LOAD) mask &= ~0xFFUL;  // select keys are not notes

    // HOLD latch: keys pressed while HOLD is active stay held; a fresh chord
    // replaces the latched set (SH-101 hold behaviour, verify edge cases).
    if (hold) {
      if (mask & ~prevMask_) holdMask_ = (mask & ~prevMask_) | (holdMask_ & mask);
      if (!mask && !holdMask_) {}
      mask |= holdMask_;
    } else {
      holdMask_ = 0;
    }

    Output o{lastCv_, false, false};
    switch (mode_) {
      case MODE_ARP_UP: case MODE_ARP_DOWN: case MODE_ARP_UPDOWN:
        if (clockEdge) { int8_t k = arp_.step(mask, mode_); if (k >= 0) { lastCv_ = noteCv(k, p); o.clockReset = true; } }
        if (!mask) arp_.reset();
        o.cv = lastCv_;
        o.gate = (mask != 0) && (arp_.current() >= 0);
        break;

      case MODE_SEQ_PLAY:
        if (clockEdge && seq_.advance()) { lastCv_ = seq_.cv(); o.clockReset = true; }
        o.cv = seq_.gate() ? seq_.cv() : lastCv_;
        o.gate = seq_.gate();
        break;

      case MODE_SEQ_LOAD: {
        // record: KEY-TRANSPOSE = rest, HOLD-held = tie, else the note.
        if (b.pressed(controls::BTN_KEYTRANS)) seq_.record(lastCv_, STEP_REST);
        uint32_t fresh = mask & ~prevMask_;
        if (fresh) { int8_t k = lowestKey(fresh); if (k >= 0) seq_.record(noteCv(k, p), b.down(controls::BTN_HOLD) ? STEP_TIE : STEP_NOTE); }
        if (extCv >= 0 && extCv != prevExtCv_) seq_.record((uint8_t)extCv, b.down(controls::BTN_HOLD) ? STEP_TIE : STEP_NOTE);
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
        } else if (extCv >= 0) {                     // MIDI note outside the keyboard
          lastCv_ = (uint8_t)extCv; o.cv = lastCv_; o.gate = true;
          if (p.adsrLfo && extCv != prevExtCv_) o.clockReset = true;
        }
        break;
      }
    }

    prevMask_ = mask;
    prevExtCv_ = extCv;
    return o;
  }
};

} // namespace engine
