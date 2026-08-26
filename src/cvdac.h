// SuperOS-101 - MCP4822 CV DAC: MIDI filter cutoff + accent (VCF and VCA)
//
// Hardware (SuperOS-101CPU v1.1, design.py Block 6): U5 MCP4822 dual 12-bit
// DAC, bit-banged SPI mode 0 on PORTA: PA0 = ~CS, PA1 = SCK, PA2 = SDI, ~LDAC
// tied low (both outputs update when ~CS rises). Gain x2 -> 1 LSB = 1 mV,
// 0..4095 mV. Neither output is filtered: the DAC is a DC level, so "zipper"
// on CC moves is handled here by slewing, not by an RC.
//
// Two board variants (Settings.hw):
//   HW_V11_UNIPOLAR (0) = the first boards FABBED 2026-08-20 before rework: R8 100k, R9 4.7k, no
//     -5 V bias. VOUTA -> R8 100k -> "VCF" pad -> IC18b cutoff summing node
//     (virtual ground): 0..41 uA = 0..0.55 of a FREQ-fader throw (a throw is
//     15 V / 200k = 75 uA), ADDITIVE only; "center" (no contribution) = 0 V.
//     VOUTB -> R9 4.7k -> "VCA" pad: up to (4.096-0.65)/4.7k = 0.73 mA =
//     0.94x the synth's envelope current (~0.78 mA).
//   HW_V11_BIPOLAR (1, default) = the current design (R8 27k, R9 2.2k, R10 68k from
//     the synth's -5 V via the "NEG5" pad), or a fabbed board after the rework
//     (swap R8 -> 27k, R9 -> 2.2k, add 68k from -5 V to the VCF pad):
//     one throw = 2025 LSB, center = 2105 LSB (the rail is -5.3 V: 78 uA *
//     27k; calibrate per unit), range -104..+98 % of a throw from any slider
//     position; VCA up to 1.23 mA = 1.6x envelope at the capped amount 100.
//   VCA node: TR28 collector = BA662 pin 1, a current input parked at ~0.65 V.
//   VOUTB idles at 650 LSB so R9 neither sources nor sinks current.
//
// Accent models (selectable, see docs/FIRMWARE.md "CV DAC"):
//   MC-202  the synth's own envelope, gated by the CPU, scaled by the ACCENT
//           pot, ~7 ms RC on the attack, summed 1:1 into the VCA control and,
//           through two diodes (~1.2 V threshold) and 150k, into the cutoff
//           summer as a modest peak-only bump (~0.4 fader throw at max).
//   TB-303  dedicated accent pulse: the filter envelope is forced to a fixed
//           short decay, the sweep goes through the RESONANCE network and a
//           capacitor (C13) that keeps charge between notes, so consecutive
//           accents build up; depth scales with resonance. The SH-101's RES
//           slider is a passive attenuator in the audio feedback loop (not a
//           CV), so it cannot be read: depth is the resDepth parameter, set to
//           taste; the build-up is a leaky integrator.
// Both: the accent envelope is generated here (we cannot see the analog ADSR),
// triggered by a fresh note whose MIDI velocity >= velThreshold (or every
// note with the ACCENT_ALL flag), ticked once per main-loop pass (~1 kHz).
// Accent depths are defined in LSB on VOUTA, so on the unipolar board they
// are worth 27/100 of the bipolar board's in fader terms (0.55 throw max).
//
// Settings live in internal EEPROM 0x010..0x01F (combined.h map). SysEx:
//   0x23 -> 0x24 <12 bytes> <codeA lo7 hi5> <codeB lo7 hi5>  request / reply
//   0x25 <12 bytes> -> 0x24  set config        (see Pack()/Unpack() for the body)
#pragma once
#include <Arduino.h>
#include <avr/eeprom.h>   // settings at 0x010.. (map documented in combined.h)

namespace cvdac {

// --- electrical constants (LSB = mV) -----------------------------------------
static const uint16_t DAC_MAX        = 4095;
static const uint16_t VCA_PARK       = 650;    // BA662 pin-1 node voltage: no current either way
static const uint16_t VCA_SPAN       = DAC_MAX - VCA_PARK;   // 3445 LSB: 2x envelope (2.2k) / 0.94x (4.7k)
// BA662 pin 1 is a CA3080-class control-current input rated around 2 mA. The
// synth's own envelope already puts ~0.78 mA in at its peak, so the accent is
// capped so that envelope + accent stays near that rating:
//   amount 100 -> 0.65 + 3445 * 100/127 mV = 3.36 V -> (3.36 - 0.65)/2.2k = 1.23 mA
//   0.78 + 1.23 = 2.0 mA. Amounts above VCA_AMOUNT_MAX are clamped everywhere
// (Unpack/Set), so neither SysEx nor EEPROM can exceed it.
static const uint8_t  VCA_AMOUNT_MAX = 100;
// Accent depth at amount 127, in LSB on VOUTA.
static const uint16_t VCF_ACC_MC202  = 810;    // bipolar board: ~0.4 fader throw (MC-202 150k path after the diodes)
static const uint16_t VCF_ACC_TB303  = 1620;   // bipolar board: ~0.8 fader throw at full resDepth (303 sweep is big)
// Timing (ms).
static const uint8_t  ATTACK_TAU_MS  = 7;      // MC-202 R184 330k * C74 22n
static const uint16_t TB303_DECAY_MS = 200;    // the 303's accent-forced short decay
static const uint16_t TB303_BUILD_TAU_MS = 400; // C13 bleed between accented notes
static const uint16_t TB303_BUILD_CAP = 16384;  // 0.5 of full sweep (Q15); env alone gives 0.5
// Cutoff de-zipper: a 7-bit CC step is 32 LSB (~2 semitones of cutoff), so the
// target is approached with a one-pole glide (1/16 of the remaining distance
// per ms, tau ~16 ms: a full-range jump lands in <100 ms, a single CC step
// becomes a smooth 50 ms curve) with a floor so it always converges exactly.
static const uint8_t  SLEW_SHIFT      = 4;     // remaining/16 per ms
static const uint8_t  SLEW_MIN_LSB    = 2;     // per ms, convergence floor
static const uint16_t MC202_THR_Q15  = 4915;   // 0.15 of peak ~= the two-diode drop vs a ~8 V envelope

enum Hw : uint8_t { HW_V11_UNIPOLAR = 0, HW_V11_BIPOLAR = 1 };
// Per-variant: LSB per full FREQ-fader throw and the no-contribution center.
static const uint16_t FADER_LSB_BY_HW[2]  = { 7500, 2025 };   // 75 uA * 100k / 27k
static const uint16_t CENTER_DEFAULT_BY_HW[2] = { 0, 2105 };  // unipolar: 0 V; bipolar: 5.3 V/68k = 78 uA * 27k (the rail is -5.3 V)

enum Mode : uint8_t { MODE_MC202 = 0, MODE_TB303 = 1 };
enum Flag : uint8_t {
  F_MIDI_CUTOFF = 1 << 0,   // CC moves the cutoff; off = VOUTA sits at center (+ accent)
  F_VCF_ACCENT  = 1 << 1,   // accent bumps the filter
  F_VCA_ACCENT  = 1 << 2,   // accent boosts the VCA
  F_ACCENT_ALL  = 1 << 3,   // every fresh note accents (panel/sequencer notes have no velocity)
  F_CUTOFF_UNIPOLAR = 1 << 4, // CC 0..127 = 0..+1 throw (forced on the unipolar board); default bipolar
};

struct Settings {
  uint16_t center    = 2105;  // LSB; ~2105 on the bipolar board (-5.3 V rail; calibrate), 0 on the unipolar one
  uint8_t  cutoffCc  = 74;
  uint8_t  flags     = F_MIDI_CUTOFF | F_VCF_ACCENT | F_VCA_ACCENT;
  uint8_t  mode      = MODE_MC202;
  uint8_t  vcfAmount = 100;   // 0..127
  uint8_t  vcaAmount = 64;    // 0..VCA_AMOUNT_MAX(100); bipolar board: 64 = +1 envelope, 100 = +1.6; unipolar: 100 = +0.74
  uint8_t  decay10ms = 30;    // MC-202 accent decay, units of 10 ms (1..127); 303 uses TB303_DECAY_MS
  uint8_t  velThr    = 100;   // MIDI velocity at/above which a note is accented
  uint8_t  resDepth  = 64;    // 303: stands in for the RESONANCE slider (0..127)
  uint8_t  buildup   = 64;    // 303: per-accent C13 charge (0..127)
  uint8_t  hw        = HW_V11_BIPOLAR;    // board variant (see header); full-range MIDI cutoff needs the bipolar board
};

// EEPROM map (combined.h: 0x010.. free).
#define EE_CV_MAGIC ((uint8_t*)0x010)
#define EE_CV_BASE  ((uint8_t*)0x011)        // CFG_LEN bytes, Pack() order
static const uint8_t EE_CV_MAGIC_VAL = 0x5E;   // 0x5C/0x5D = earlier layouts (reset to defaults)
static const uint8_t CFG_LEN = 12;

struct State {
  Settings s;
  // cutoff
  int16_t  cutoffTarget = 0;   // LSB offset from center
  int16_t  cutoffNow    = 0;
  uint8_t  ccLsb        = 0;   // 14-bit CC: LSB (CC cutoffCc+32) last received
  // accent envelope, Q15 (0..32767)
  uint16_t env = 0;
  bool     attacking = false;
  uint16_t build = 0;          // 303 C13 charge, Q15
  // output
  uint16_t codeA = 2105, codeB = VCA_PARK;
  uint16_t lastA = 0xFFFF, lastB = 0xFFFF;
  uint32_t lastTickUs = 0;
};
inline State& S() { static State s; return s; }
inline uint16_t FaderLsb() { return FADER_LSB_BY_HW[S().s.hw & 1]; }

// --- settings (EEPROM + SysEx body) -----------------------------------------
// Body (12 bytes, all 7-bit): center lo7, center hi5, cc, flags, mode,
// vcfAmount, vcaAmount, decay10ms, velThr, resDepth, buildup, hw.
inline void Pack(const Settings& s, uint8_t* b) {
  b[0] = s.center & 0x7F; b[1] = (s.center >> 7) & 0x1F;
  b[2] = s.cutoffCc & 0x7F; b[3] = s.flags & 0x7F; b[4] = s.mode & 0x7F;
  b[5] = s.vcfAmount & 0x7F; b[6] = s.vcaAmount & 0x7F; b[7] = s.decay10ms & 0x7F;
  b[8] = s.velThr & 0x7F; b[9] = s.resDepth & 0x7F; b[10] = s.buildup & 0x7F;
  b[11] = s.hw & 0x7F;
}
inline bool Unpack(const uint8_t* b, Settings& s) {
  uint16_t c = (uint16_t)((b[0] & 0x7F) | ((uint16_t)(b[1] & 0x1F) << 7));
  if (c > DAC_MAX) return false;
  s.center = c; s.cutoffCc = b[2] & 0x7F; s.flags = b[3] & 0x7F;
  s.mode = (b[4] & 0x7F) ? MODE_TB303 : MODE_MC202;
  s.vcfAmount = b[5] & 0x7F; s.vcaAmount = b[6] & 0x7F;
  if (s.vcaAmount > VCA_AMOUNT_MAX) s.vcaAmount = VCA_AMOUNT_MAX;   // BA662 control-current cap
  s.decay10ms = (b[7] & 0x7F) ? (b[7] & 0x7F) : 1;
  s.velThr = (b[8] & 0x7F) ? (b[8] & 0x7F) : 1;
  s.resDepth = b[9] & 0x7F; s.buildup = b[10] & 0x7F;
  s.hw = (b[11] & 0x7F) ? HW_V11_BIPOLAR : HW_V11_UNIPOLAR;
  return true;
}
inline void Save() {
  uint8_t b[CFG_LEN]; Pack(S().s, b);
  for (uint8_t i = 0; i < CFG_LEN; ++i) eeprom_update_byte(EE_CV_BASE + i, b[i]);
  eeprom_update_byte(EE_CV_MAGIC, EE_CV_MAGIC_VAL);
}
inline void Load() {
  if (eeprom_read_byte(EE_CV_MAGIC) != EE_CV_MAGIC_VAL) return;   // defaults
  uint8_t b[CFG_LEN];
  for (uint8_t i = 0; i < CFG_LEN; ++i) b[i] = eeprom_read_byte(EE_CV_BASE + i);
  Settings s; if (Unpack(b, s)) S().s = s;
}
inline const Settings& Get() { return S().s; }
inline void Set(const Settings& s) { S().s = s; if (S().s.vcaAmount > VCA_AMOUNT_MAX) S().s.vcaAmount = VCA_AMOUNT_MAX; Save(); }
inline void SetMode(uint8_t m) { S().s.mode = m ? MODE_TB303 : MODE_MC202; Save(); }
inline void SetCenter(uint16_t c) { if (c <= DAC_MAX) { S().s.center = c; Save(); } }
// Switch board variant: also resets the center to that variant's default.
inline void SetHw(uint8_t hw) {
  S().s.hw = hw ? HW_V11_BIPOLAR : HW_V11_UNIPOLAR;
  S().s.center = CENTER_DEFAULT_BY_HW[S().s.hw];
  Save();
}

// --- MCP4822 driver ------------------------------------------------------------
// Command word: bit15 channel (0 = A), bit13 ~GA (0 = x2 gain), bit12 ~SHDN
// (1 = active), bits 11..0 data. SPI mode 0, MSB first, ~CS framing per word.
#ifdef __AVR__
inline void BusInit() {
  PORTA |= _BV(0);                    // ~CS high before it becomes an output
  PORTA &= (uint8_t)~(_BV(1) | _BV(2));
  DDRA  |= _BV(0) | _BV(1) | _BV(2);
}
// noinline: one shared copy of the bit loop (see the 303's fram.h note).
__attribute__((noinline)) inline void Word(uint16_t w) {
  PORTA &= (uint8_t)~_BV(0);          // ~CS low
  for (uint16_t bit = 0x8000; bit; bit >>= 1) {
    if (w & bit) PORTA |= _BV(2); else PORTA &= (uint8_t)~_BV(2);
    PORTA |= _BV(1);                  // rising edge: DAC samples SDI
    PORTA &= (uint8_t)~_BV(1);
  }
  PORTA |= _BV(0);                    // ~CS high: latch (LDAC is tied low)
}
#else
extern "C" void sim_cvdac_write(uint8_t ch, uint16_t code);   // host sim hook
inline void BusInit() {}
inline void Word(uint16_t w) { sim_cvdac_write((uint8_t)(w >> 15), (uint16_t)(w & 0x0FFF)); }
#endif
inline void WriteA(uint16_t code) { Word((uint16_t)(0x1000 | (code & 0x0FFF))); }
inline void WriteB(uint16_t code) { Word((uint16_t)(0x9000 | (code & 0x0FFF))); }

inline void Flush() {
  State& st = S();
  if (st.codeA != st.lastA) { WriteA(st.codeA); st.lastA = st.codeA; }
  if (st.codeB != st.lastB) { WriteB(st.codeB); st.lastB = st.codeB; }
}

inline void Begin() {
  Load();
  BusInit();
  State& st = S();
  st.codeA = st.s.center; st.codeB = VCA_PARK;
  st.lastA = st.lastB = 0xFFFF;
  Flush();                            // outputs known before the first note
  st.lastTickUs = micros();
}

// --- inputs --------------------------------------------------------------------
// Cutoff CC, 7- or 14-bit (MSB on cutoffCc, optional LSB on cutoffCc+32 per the
// MIDI convention: an MSB alone means LSB 0; an LSB refines the last MSB).
// Bipolar board: 0..16383 = -1..+1 fader throw around the slider's own position
// (8192 = no change), or 0..+1 with F_CUTOFF_UNIPOLAR. Unipolar board: always
// 0..+1 throw worth of LSB (the DAC clamps at its 0.55-throw ceiling).
inline void ApplyCutoff(uint16_t v14) {
  State& st = S();
  const int32_t F = FaderLsb();
  int32_t t;
  if (st.s.hw == HW_V11_UNIPOLAR || (st.s.flags & F_CUTOFF_UNIPOLAR)) t = (int32_t)v14 * F / 16383;
  else if (v14 >= 8192)                t = ((int32_t)v14 - 8192) * F / 8191;   // 16383 -> +1 throw
  else                                 t = ((int32_t)v14 - 8192) * F / 8192;   //     0 -> -1 throw
  if (t > 8000) t = 8000; if (t < -8000) t = -8000;
  st.cutoffTarget = (int16_t)t;
}
inline void OnCc(uint8_t num, uint8_t val) {
  State& st = S();
  if (!(st.s.flags & F_MIDI_CUTOFF)) return;
  static uint8_t msb = 64;
  if (num == st.s.cutoffCc) { msb = val & 0x7F; st.ccLsb = 0; ApplyCutoff((uint16_t)(msb << 7)); }
  else if (st.s.cutoffCc < 96 && num == (uint8_t)(st.s.cutoffCc + 32)) { st.ccLsb = val & 0x7F; ApplyCutoff((uint16_t)((msb << 7) | st.ccLsb)); }
}
inline void SetCutoffEnabled(bool on) {
  State& st = S();
  if (on) st.s.flags |= F_MIDI_CUTOFF; else { st.s.flags &= (uint8_t)~F_MIDI_CUTOFF; st.cutoffTarget = 0; }
  Save();
}

// A fresh (non-legato) note started. vel = MIDI velocity (0 for panel /
// sequencer notes, which carry none). Returns true if it was accented.
inline bool NoteOn(uint8_t vel) {
  State& st = S();
  bool acc = (st.s.flags & F_ACCENT_ALL) || (vel && vel >= st.s.velThr);
  if (!acc) return false;
  st.attacking = true;                // MC-202 attack RC; the 303 pulse is just as fast
  if (st.s.mode == MODE_TB303) {
    // C13 charge: +0.25 of full sweep per hit at buildup 127, capped at 0.5,
    // on top of the 0.5 a single accent's envelope contributes (see Tick):
    // one accent sweeps ~0.73, a run of them climbs to the full sweep.
    uint32_t b = (uint32_t)st.build + (uint32_t)st.s.buildup * 64;
    st.build = b > TB303_BUILD_CAP ? TB303_BUILD_CAP : (uint16_t)b;
  }
  return true;
}

// --- per-pass update ---------------------------------------------------------
// Runs the accent envelope and the cutoff slew with the pass's real elapsed
// time, then writes whichever DAC channel changed. ~1 kHz from the main loop;
// a stalled pass (flash page write) just makes one longer step.
inline void Tick() {
  State& st = S();
  uint32_t now = micros();
  uint32_t dtUs = now - st.lastTickUs; st.lastTickUs = now;
  uint16_t dt = (uint16_t)(dtUs > 20000 ? 20 : (dtUs + 500) / 1000);   // ms, capped
  if (!dt && dtUs) dt = 1;

  // cutoff glide toward the CC target (one-pole per ms with a convergence floor)
  for (uint8_t k = 0; k < dt; ++k) {
    int16_t d = (int16_t)(st.cutoffTarget - st.cutoffNow);
    if (!d) break;
    int16_t step = (int16_t)(d >> SLEW_SHIFT);
    if (d > 0 && step < SLEW_MIN_LSB) step = (d < SLEW_MIN_LSB) ? d : SLEW_MIN_LSB;
    if (d < 0 && step > -SLEW_MIN_LSB) step = (d > -SLEW_MIN_LSB) ? d : -SLEW_MIN_LSB;
    st.cutoffNow = (int16_t)(st.cutoffNow + step);
  }

  // accent envelope (Q15)
  if (st.attacking) {
    // one-pole toward full scale, tau = ATTACK_TAU_MS
    if (dt >= ATTACK_TAU_MS) st.env = 32767;
    else st.env = (uint16_t)(st.env + (uint32_t)(32767 - st.env) * dt / ATTACK_TAU_MS);
    if (st.env >= 32100) { st.env = 32767; st.attacking = false; }
  } else if (st.env) {
    // exponential decay in integer math: never let the step round to 0, and
    // snap to 0 below 0.5 % of full scale so the outputs return exactly to rest
    uint16_t tau = (st.s.mode == MODE_TB303) ? TB303_DECAY_MS : (uint16_t)st.s.decay10ms * 10;
    uint32_t dec = (uint32_t)st.env * dt / tau; if (!dec) dec = dt;
    st.env = (dec >= st.env) ? 0 : (uint16_t)(st.env - dec);
    if (st.env < 164) st.env = 0;
  }
  if (st.build) {
    uint32_t dec = (uint32_t)st.build * dt / TB303_BUILD_TAU_MS; if (!dec) dec = dt;
    st.build = (dec >= st.build) ? 0 : (uint16_t)(st.build - dec);
    if (st.build < 164) st.build = 0;
  }

  // VCF contribution (LSB; on the unipolar board the same LSB are 27/100 of
  // the fader-relative depth, and the DAC tops out at 0.55 throw)
  int32_t vcf = 0;
  if (st.s.flags & F_VCF_ACCENT) {
    if (st.s.mode == MODE_TB303) {
      // a lone accent sweeps 0.5 of full; the C13 charge (build) adds up to 0.5,
      // so runs of accents climb toward the full sweep like the 303's squelch
      uint32_t sweep = (uint32_t)st.env / 2 + st.build; if (sweep > 32767) sweep = 32767;
      vcf = (int32_t)(sweep * VCF_ACC_TB303 / 32767) * st.s.resDepth / 127 * st.s.vcfAmount / 127;
    } else {
      int32_t above = (int32_t)st.env - MC202_THR_Q15;     // only the peak passes the diodes
      if (above > 0) vcf = above * VCF_ACC_MC202 / (32767 - MC202_THR_Q15) * st.s.vcfAmount / 127;
    }
    if (st.s.hw == HW_V11_UNIPOLAR) vcf = vcf * 100 / 27;   // same fader-relative depth through 100k
  }
  int32_t a = (int32_t)st.s.center + st.cutoffNow + vcf;
  st.codeA = a < 0 ? 0 : (a > DAC_MAX ? DAC_MAX : (uint16_t)a);

  // VCA contribution: bipolar board amount 127 = +2 envelopes, 64 = +1 (MC-202);
  // unipolar board (4.7k) amount 127 = +0.94 envelope
  int32_t b = VCA_PARK;
  if (st.s.flags & F_VCA_ACCENT) b += (int32_t)st.env * VCA_SPAN / 32767 * st.s.vcaAmount / 127;
  st.codeB = b > DAC_MAX ? DAC_MAX : (uint16_t)b;

  Flush();
}

// Telemetry for the editor / bench.
inline uint16_t CodeA() { return S().codeA; }
inline uint16_t CodeB() { return S().codeB; }
inline bool     AccentActive() { return S().env != 0; }

} // namespace cvdac
