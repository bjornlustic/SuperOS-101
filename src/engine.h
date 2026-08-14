// SuperOS-101 — note engine: keyboard / arpeggiator / sequencer
//
// Clean-room behavioural implementation matching the original's OBSERVABLE
// behaviour (this is our own firmware — see docs/FIRMWARE.md). It is NOT a
// transliteration of the 8049 code; where exact original semantics are
// uncertain (mode latching, tie/rest edge cases) it's marked TODO/verify.
//
//   - keyboard : mono, lowest- or last-note priority (panel-selected)
//   - arp      : UP / DOWN / UP&DOWN over held keys, advanced by the clock
//   - sequencer: up to 100 steps (note / rest / tie), stored in AVR EEPROM
//                (survives battery removal — an upgrade over the original's
//                 battery-backed internal RAM)
#pragma once
#include <Arduino.h>
#include <avr/eeprom.h>
#include "controls.h"
#include "combined.h"

namespace engine {

static const uint8_t MAX_STEPS = 100;
static const uint8_t CV_BASE   = 5;   // cv_dac_val returns value+5 (SH101.asm:992)
static const uint8_t CV_MAX    = 63;  // 6-bit DAC

enum Mode : uint8_t {
  MODE_KEYBOARD, MODE_ARP_UP, MODE_ARP_DOWN, MODE_ARP_UPDOWN, MODE_SEQ_PLAY, MODE_SEQ_LOAD,
};
enum StepType : uint8_t { STEP_NOTE = 0, STEP_REST = 1, STEP_TIE = 2 };

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
  // advance one step over `mask` in the given mode; returns the new key (or -1).
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

// --- sequencer (EEPROM-backed) ----------------------------------------------
class Sequencer {
  uint8_t note_[MAX_STEPS];
  uint8_t type_[MAX_STEPS];
  uint8_t len_ = 0, pos_ = 0;
  uint8_t curCv_ = CV_BASE; bool curGate_ = false;

  // EEPROM layout (at EE_SEQ_BASE, see combined.h map):
  // [0]=magic [1]=len [2..101]=note [102..201]=type
  static const uint16_t EE_BASE = EE_SEQ_BASE;
  static const uint8_t  EE_MAGIC = 0x51; // 'Q'
 public:
  uint8_t length() const { return len_; }
  void clear() { len_ = 0; pos_ = 0; }
  // rewind so the NEXT advance() plays step 0 (advance pre-increments)
  void rewind() { pos_ = len_ ? (uint8_t)(len_ - 1) : 0; }
  void record(uint8_t cv, StepType t) { if (len_ < MAX_STEPS) { note_[len_] = cv; type_[len_] = t; ++len_; } }

  // advance one step on a clock edge; returns true if a fresh note triggered.
  bool advance() {
    if (len_ == 0) { curGate_ = false; return false; }
    if (++pos_ >= len_) pos_ = 0;
    switch (type_[pos_]) {
      case STEP_REST: curGate_ = false; return false;
      case STEP_TIE:  curGate_ = true;  return false;       // hold prev cv, no retrigger
      default:        curCv_ = note_[pos_]; curGate_ = true; return true;
    }
  }
  uint8_t cv()   const { return curCv_; }
  bool    gate() const { return curGate_; }

  void load() {
    if (eeprom_read_byte((uint8_t*)EE_BASE) != EE_MAGIC) { clear(); return; }
    len_ = eeprom_read_byte((uint8_t*)(EE_BASE + 1));
    if (len_ > MAX_STEPS) { clear(); return; }
    for (uint8_t i = 0; i < len_; ++i) {
      note_[i] = eeprom_read_byte((uint8_t*)(uintptr_t)(EE_BASE + 2 + i));
      type_[i] = eeprom_read_byte((uint8_t*)(uintptr_t)(EE_BASE + 2 + MAX_STEPS + i));
    }
  }
  void save() {
    eeprom_update_byte((uint8_t*)EE_BASE, EE_MAGIC);
    eeprom_update_byte((uint8_t*)(EE_BASE + 1), len_);
    for (uint8_t i = 0; i < len_; ++i) {
      eeprom_update_byte((uint8_t*)(uintptr_t)(EE_BASE + 2 + i), note_[i]);
      eeprom_update_byte((uint8_t*)(uintptr_t)(EE_BASE + 2 + MAX_STEPS + i), type_[i]);
    }
  }
};

// --- top-level engine -------------------------------------------------------
class NoteEngine {
  Mode mode_ = MODE_KEYBOARD;
  Arp  arp_;
  Sequencer seq_;
  uint32_t prevMask_ = 0;
  int8_t   heldKey_  = -1;
  uint8_t  lastCv_   = CV_BASE;
  uint8_t  keyShift_ = 0; // KEY TRANSPOSE (TODO: SH101.asm:844-866)

  uint8_t noteCv(int8_t key, const controls::Panel& p) const {
    int v = key + (int)p.octaveOffset + keyShift_ + CV_BASE;
    return (uint8_t)(v < 0 ? 0 : (v > CV_MAX ? CV_MAX : v));
  }
  // mono keyboard key choice, honouring last-note vs lowest-note priority.
  int8_t pickKey(uint32_t mask, const controls::Panel& p) {
    if (!mask) return -1;
    uint32_t fresh = mask & ~prevMask_;
    if (p.lastNotePriority && fresh)            heldKey_ = lowestKey(fresh);
    else if (heldKey_ >= 0 && (mask & (1UL << heldKey_))) { /* keep */ }
    else                                         heldKey_ = lowestKey(mask);
    return heldKey_;
  }
  void selectMode(controls::Buttons& b) {
    using namespace controls;
    if (b.pressed(BTN_UP))   mode_ = (mode_ == MODE_ARP_UP)     ? MODE_KEYBOARD : MODE_ARP_UP;
    if (b.pressed(BTN_DOWN)) mode_ = (mode_ == MODE_ARP_DOWN)   ? MODE_KEYBOARD : MODE_ARP_DOWN;
    if (b.pressed(BTN_UD))   mode_ = (mode_ == MODE_ARP_UPDOWN) ? MODE_KEYBOARD : MODE_ARP_UPDOWN;
    if (b.pressed(BTN_PLAY)) mode_ = (mode_ == MODE_SEQ_PLAY)   ? MODE_KEYBOARD : MODE_SEQ_PLAY;
    if (b.pressed(BTN_LOAD)) {
      if (mode_ == MODE_SEQ_LOAD) { seq_.save(); mode_ = MODE_KEYBOARD; }
      else { seq_.clear(); mode_ = MODE_SEQ_LOAD; }
    }
  }
 public:
  Mode mode() const { return mode_; }
  void begin() { seq_.load(); }
  // MIDI Start: align the sequence and arpeggiator to the incoming downbeat.
  void restart() { seq_.rewind(); arp_.reset(); }

  Output update(uint32_t mask, const controls::Panel& p, controls::Buttons& b,
                bool clockEdge, uint8_t /*random*/) {
    selectMode(b);
    Output o{lastCv_, false, false};

    switch (mode_) {
      case MODE_ARP_UP: case MODE_ARP_DOWN: case MODE_ARP_UPDOWN:
        if (clockEdge) { int8_t k = arp_.step(mask, mode_); if (k >= 0) { lastCv_ = noteCv(k, p); o.clockReset = true; } }
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
        // live monitor through the keyboard while recording
        int8_t lk = lowestKey(mask);
        if (lk >= 0) { lastCv_ = noteCv(lk, p); o.cv = lastCv_; o.gate = true; }
        break;
      }

      default: { // MODE_KEYBOARD
        int8_t k = pickKey(mask, p);
        if (k >= 0) {
          lastCv_ = noteCv(k, p); o.cv = lastCv_; o.gate = true;
          if (p.adsrLfo && (mask & ~prevMask_)) o.clockReset = true; // reset LFO on new key
        }
        break;
      }
    }

    prevMask_ = mask;
    return o;
  }
};

} // namespace engine
