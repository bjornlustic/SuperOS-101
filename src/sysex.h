// SuperOS-101 - SysEx commands common to both firmwares (see midi.h header)
//   0x2F switch to the 80C49 emulator, 0x4D switch to SuperOS, 0x4A enter the
//   SysEx bootloader, 0x40 emulator status (answered only by the emulator).
// Plus the MIDI channel setting shared by both sides (internal EEPROM).
#pragma once
#include <Arduino.h>
#include <avr/eeprom.h>
#include "combined.h"
#include "midi.h"
#include "emu101.h"
#include "usb_gate.h"
#include "hw.h"      // P2Observer: the SuperOS LED/gate probe words
#include "flash_persist.h"   // persist::Health: the probe's store-health byte

#ifndef SUPEROS_VERSION
#define SUPEROS_VERSION "dev"
#endif
// The git sha this image was built from, defined by tools/buildrev.py. A build
// that reaches this fallback genuinely does not know what it is, and says so:
// the word carries no hex so it can never be misread as a revision.
#ifndef SUPEROS_BUILD_REV
#define SUPEROS_BUILD_REV "unknown"
#endif

namespace sysex {

inline uint8_t LoadChannel() {
  if (eeprom_read_byte(EE_CFG_MAGIC) != EE_CFG_MAGIC_VAL) return 0;
  uint8_t c = eeprom_read_byte(EE_CFG_CHAN);
  return c < 16 ? c : 0;
}
inline void SaveChannel(uint8_t c) {
  eeprom_update_byte(EE_CFG_MAGIC, EE_CFG_MAGIC_VAL);
  eeprom_update_byte(EE_CFG_CHAN, (uint8_t)(c & 0x0F));
}

// Soft thru. Not gated on EE_CFG_MAGIC: an image from before this byte existed
// reads 0xFF here, which is off, and the byte is only ever written as 0 or 1.
inline bool LoadThru() { return eeprom_read_byte(EE_CFG_THRU) == 1; }
inline void SaveThru(bool on) { eeprom_update_byte(EE_CFG_THRU, on ? 1 : 0); }
// Clock source (midi.h CLK_*). Same rule: an older image's 0xFF reads as both.
inline uint8_t LoadClockFrom() { uint8_t v = eeprom_read_byte(EE_CFG_CLKSRC); return v <= 2 ? v : 0; }
inline void SaveClockFrom(uint8_t v) { eeprom_update_byte(EE_CFG_CLKSRC, v); }
inline uint8_t LoadNoteOut() { uint8_t v = eeprom_read_byte(EE_CFG_NOTEOUT); return v <= 3 ? v : 0; }
inline void SaveNoteOut(uint8_t v) { eeprom_update_byte(EE_CFG_NOTEOUT, v); }

// 0x34 -> 0x35 build identity: "<version> <sha>[*]" in 7-bit ASCII, e.g.
// "0.2 0642b7f" clean or "0.2 0642b7f*" dirty. Same opcode pair and same
// string shape as the 606 (its src/midi.cpp:40-41, CMD_REQ_VERSION /
// CMD_VERSION), which is the G9 answer: one reply a host can parse for all
// four machines rather than a per-machine dialect.
//
// It lives in HandleCommon, NOT in main.cpp's SuperOS-only switch, so the
// machine answers in EITHER firmware. The emulator build is exactly the case
// where nobody can tell from outside what is resident, so it is the one that
// most needs to be able to say.
inline void SendVersion() {
  static const char kVer[] = SUPEROS_VERSION;
  static const char kRev[] = SUPEROS_BUILD_REV;
  // Same hazard as the 0x3C probe below: midi::SendSysex takes a uint8_t
  // length, so an over-long body would wrap and send a truncated reply that
  // still looks well-formed. Fail the build instead.
  static_assert(1 + sizeof(kVer) + sizeof(kRev) <= 255,
                "0x35 version body exceeds the uint8_t SysEx length");
  uint8_t body[1 + sizeof(kVer) + sizeof(kRev)];   // opcode + ver + ' ' + rev
  uint8_t n = 0;
  body[n++] = 0x35;
  for (uint8_t i = 0; kVer[i]; ++i) body[n++] = (uint8_t)(kVer[i] & 0x7F);
  body[n++] = ' ';
  for (uint8_t i = 0; kRev[i]; ++i) body[n++] = (uint8_t)(kRev[i] & 0x7F);
  midi::SendSysex(body, n);
}

inline void SendEmuStatus() {
  uint8_t body[2 + 24];
  body[0] = 0x41; body[1] = 1;
  uint8_t n = 2;
  for (const char* v = SUPEROS_VERSION; *v && n < sizeof(body); ++v) body[n++] = (uint8_t)(*v & 0x7F);
  midi::SendSysex(body, n);
}

// 0x3B -> 0x3C bench probe: raw registers as hi/lo nibble pairs.
//   [0] GPIOR1 = MCUSR the bootloader saw at its last entry (reset cause:
//       bit0 PORF, bit1 EXTRF, bit2 BORF, bit3 WDRF, bit4 JTRF)
//   [1] USBCON [2] UDCON [3] UDIEN [4] USBSTA [5] UDFNUML
//   [6] usb_configuration (SUPEROS_USB_MIDI builds, else 0)
//   [7] boot counter since power-on (boot.cpp, survives resets)
//   [8] stage the previous boot last reached (boot.cpp)
//   [9..10] last loop pass us (hi, lo)  [11..12] max pass us since last probe
//   [13..14] VCC in mV (hi, lo): internal 1.1 V bandgap read against AVCC.
//            The pitch DAC's reference is this rail, so it is the pitch scale.
//            A MEAN of 8 samples, and deliberately coarse: read the resolution
//            note above [114..123] before treating it as a rail measurement.

// The ADC wants 50-200 kHz to deliver all ten bits (AT90USB1286 datasheet
// 24.4), and on this machine the core clock is NOT a compile-time constant:
// the 0x3E bench window changes it at runtime. Choosing the divider from
// F_CPU therefore under-clocks the ADC at 4 MHz and runs it at 250 kHz, out
// of spec, during a 16 MHz window -- which is the one condition the rail
// question is actually asked in. Pick the smallest divider that stays at or
// under 200 kHz for the clock we are really running at.
inline uint8_t AdcPrescaleBits(uint32_t coreHz) {
  uint8_t bits = 1;                       // ADPS 0 and 1 both mean /2
  while (bits < 7 && (coreHz >> bits) > 200000UL) ++bits;
  return bits;
}

// One set of raw bandgap samples. Raw ADC codes, not millivolts: the
// conversion to mV is lossy and the host can do it better than we can.
struct VccSamples {
  uint32_t sum;      // sum of `n` 10-bit codes
  uint16_t min, max; // the extremes across the set: this IS the ripple
  uint16_t n;
};

inline uint16_t VccCodeToMv(uint16_t adc) {
  return adc ? (uint16_t)((1100UL * 1024UL) / adc) : 0;
}

#ifdef __AVR__
// Open the bandgap channel and let it settle. Leaves the ADC powered and
// selected so a set can be collected a few samples at a time.
inline void VccSampleBegin(uint32_t coreHz) {
  PRR0 &= (uint8_t)~(1 << PRADC);
  ADMUX  = (1 << REFS0) | 0x1E;           // AVCC ref, 1.1 V bandgap in
  ADCSRA = (uint8_t)((1 << ADEN) | AdcPrescaleBits(coreHz));
  delayMicroseconds(400);                 // bandgap settle (~70 us typ)
  ADCSRA |= (1 << ADSC); while (ADCSRA & (1 << ADSC)) {}   // discard the first
}
inline void VccSampleEnd() { ADCSRA = 0; PRR0 |= (1 << PRADC); }

// Take up to `count` conversions into `s`. A full set of 256 costs ~27 ms of
// ADC time, which is far longer than a loop pass, and stalling the pass would
// starve the S/H refresh and move the very rail under test. So the set is
// gathered across passes instead of in one blocking burst.
inline void VccSampleStep(VccSamples& s, uint8_t count) {
  while (count--) {
    ADCSRA |= (1 << ADSC); while (ADCSRA & (1 << ADSC)) {}
    uint16_t v = ADC;
    if (!s.n || v < s.min) s.min = v;
    if (!s.n || v > s.max) s.max = v;
    s.sum += v; ++s.n;
  }
}
#endif

inline uint16_t MeasureVccMv() {
#ifdef __AVR__
  uint8_t prr = PRR0;
  VccSampleBegin(F_CPU);
  VccSamples s = {0, 0, 0, 0};
  VccSampleStep(s, 8);
  VccSampleEnd(); PRR0 = prr;
  return VccCodeToMv((uint16_t)(s.sum / 8));
#else
  return 0;
#endif
}
//   [15..16] VCC in mV sampled while the USB engine was parked (0x3D hold-off)
//   [17..18] emulator: emulated 80C49 machine cycles per second / 100
//            (the real 6 MHz chip = 4000); 0 in SuperOS
//   [19..20] emulator: ROM program passes per second (S/H refresh rate,
//            T1 clock sample rate); 0 in SuperOS
//   [21..22] emulator: T1 (LFO / EXT CLK) falling edges per second
//   [23..24] emulator: share of time inside the interpreter (per mille)
//   [25..26] emulator: share of time inside the S/H refresh (per mille)
//   [27..28] emulator: boot bench, us per 4000 emulated cycles, interrupts off
//   [29..30] emulator: same with interrupts on
//   [31..32] emulator: boot bench, us for 10 hw::DacWrite
//   [33..34] emulator: boot bench, us for 100 midi::Poll
//   [35..46] emulator: boot bench, us for 100x T1Watch, RateTick, RamPersistTick,
//            micros(), button scan (5 words; word 0 repeats [33..34])
//   [47..48] emulator: T1 falling edges per second as seen by the host-pass poll
//   [49..50] emulator: INT0 edges per second rejected as bounces (< 0.5 ms low)
//   [51..52] emulator: last gate off->on gap (us)  [53..54] max gap since last probe
//   [55..56] emulator: gate-on events per second
//   [57..58] emulator: last off->on gap on the physical GATE pin (us, after shaping)
//   [59..60] emulator: PORTAMENTO OFF pulses per second  [61..62] last pulse length (us)
//   [63..64] emulator: last PORTAMENTO OFF rise minus last CV-OUT change (us)
//   [65..80] emulator: ROM P2 bit 0..7 transitions per second (8 words)
//   [81..82] emulator: last ROM gate on-length (us)  [83..84] last gate drop minus CV change (us)
//   [85..86] emulator: incoming MIDI note-ons per second
//   [87..88] emulator: ROM key-column reads per second with a key down  [89..90] same, buttons
//   [91..96] emulator: S/H value changes per second: CV-OUT, VCO, RANDOM
//   [97]     SuperOS: last P2 image written (hw.h P2Observer). Decode with the
//            pins.h bit names, and mind the two inversions: GATE (P27) is 0
//            when the gate is ON, HOLD (P25) is 0 when the HOLD LED is LIT,
//            KEY-TRANSPOSE (P26) is 1 when lit, CLOCK RESET (P24) is 0 while
//            pulsed. P20-22 are the function-LED select: 0 none, 1-5 exactly
//            one of LOAD / PLAY / DOWN / U&D / UP.
//   [98..113] SuperOS: P2 bit 0..7 transitions per second (8 words), the same
//            shape as the emulator's [65..80] so one host reader does both.
//            This is where a blink RATE is read: G10 asserts the tempo
//            indicator tracks incoming MIDI clock, and a rate is the thing
//            that claim actually makes. The 101 has no dedicated tempo LED
//            (claim-022), so its indicator is KEY-TRANSPOSE, bit 6.
//
// The rail set, and why it is raw codes rather than millivolts (claim-038, G13):
//   [114..116] sum of the parked set's 10-bit ADC codes (hi..lo, 24-bit)
//   [117..118] the set's MINIMUM code   [119..120] its MAXIMUM code
//   [121]      samples in the set / 16 (256 -> 16). 0 = the set never completed
//              and every word above it is meaningless.
//   [122]      the CLKPS the window actually applied: 0 = 16 MHz, 1 = 8 MHz,
//              2 = 4 MHz. Reported because it was assumed once and wrong.
//   [123]      the bench mode byte as applied
//   [124]      store health (flash_persist.h Health): bit0 SPM service answered,
//              bit1 mount attempted, bit2 arena mounted and writable. A606's
//              claim-036: the 606 acked every write from a RAM cache while
//              nothing reached flash, and only a reboot exposed it.
// [13..16] answer in millivolts and cannot answer claim-038. The bandgap is
// read against AVCC, so one ADC code near 5 V is 22 mV (226 -> 4984 mV,
// 227 -> 4962 mV), and on this machine 57 mV is a semitone: one code is 39
// cents. The claim is about ~2.85 mV of ripple, so the published mean is 8x
// too coarse, and a mean of ANY precision cannot show ripple at all.
// So publish the set instead. MIN and MAX bound the ripple directly. The SUM
// recovers detail below one code only if the rail carries at least a code of
// noise to dither with, so if min == max the honest reading is "ripple is
// under one code", a BOUND and not a measurement. Publishing min and max
// beside the sum is what lets the host tell those two apart, which the mean
// could not, and it is why a single 4962 in the last sweep was one LSB of
// dither rather than a signal.
}  // namespace sysex
extern uint16_t g_vccParkedMv;   // boot.cpp
extern sysex::VccSamples g_vccParkedSet;   // boot.cpp: the 0x3E parked set
extern uint8_t  g_benchMode;     // boot.cpp (SysEx 0x3E)
extern uint8_t  g_benchApplied;  // boot.cpp: mode actually in force
extern uint8_t  g_benchClkps;    // boot.cpp: prescale actually in force
extern uint32_t g_benchUntil;
namespace sysex {
inline void SendProbe() {
  // 127 is the CEILING, not a size: the static_assert below is the real limit
  // (1 + 2*127 = 255, the largest uint8_t SysEx length). raw[125..126] is the
  // last free word and G5's key-scan bound has it. THE NEXT ADDITION CANNOT
  // APPEND: it has to change the body encoding from two nibbles per byte to
  // 7-bit packing (8 bytes carry 7), which buys room but moves every offset and
  // so breaks every reader that indexes rather than scans. Do that deliberately
  // and in one commit, not as a side effect of wanting one more word.
  uint8_t raw[127] = {0};
  uint16_t vcc = MeasureVccMv(); raw[13] = vcc >> 8; raw[14] = vcc & 0xFF;
  raw[15] = g_vccParkedMv >> 8; raw[16] = g_vccParkedMv & 0xFF;
  raw[7] = combined_boot_count();
  raw[8] = combined_prev_stage();
  if (combined_current_fw() == FW_EMU) {
    uint16_t c = emu101::cyclesPerSec100(), p = emu101::passesPerSec();
    raw[17] = c >> 8; raw[18] = c & 0xFF; raw[19] = p >> 8; raw[20] = p & 0xFF;
    uint16_t e = emu101::t1EdgesPerSec(); raw[21] = e >> 8; raw[22] = e & 0xFF;
    uint16_t r = emu101::runPerMille();   raw[23] = r >> 8; raw[24] = r & 0xFF;
    uint16_t h = emu101::shPerMille();    raw[25] = h >> 8; raw[26] = h & 0xFF;
    uint16_t b0 = emu101::benchUs(false), b1 = emu101::benchUs(true);
    raw[27] = b0 >> 8; raw[28] = b0 & 0xFF; raw[29] = b1 >> 8; raw[30] = b1 & 0xFF;
    uint16_t bd = emu101::benchDacUs(), bp = emu101::benchPollUs();
    raw[31] = bd >> 8; raw[32] = bd & 0xFF; raw[33] = bp >> 8; raw[34] = bp & 0xFF;
    for (uint8_t i = 0; i < 6; ++i) { uint16_t v = emu101::benchB(i); raw[35 + 2 * i] = v >> 8; raw[36 + 2 * i] = v & 0xFF; }
    uint16_t pe = emu101::t1PollEdgesPerSec(); raw[47] = pe >> 8; raw[48] = pe & 0xFF;
    uint16_t rj = emu101::t1RejectedPerSec();  raw[49] = rj >> 8; raw[50] = rj & 0xFF;
    uint16_t gl = emu101::gateGapUs(false), gm = emu101::gateGapUs(true), go = emu101::gateOnsPerSec();
    raw[51] = gl >> 8; raw[52] = gl & 0xFF; raw[53] = gm >> 8; raw[54] = gm & 0xFF; raw[55] = go >> 8; raw[56] = go & 0xFF;
    uint16_t gp = emu101::gatePhysGapUs(); raw[57] = gp >> 8; raw[58] = gp & 0xFF;
    uint16_t pp_ = emu101::portaPulsesPerSec(), pl = emu101::portaPulseLenUs(), pd = emu101::portaAfterCvUs();
    raw[59] = pp_ >> 8; raw[60] = pp_ & 0xFF; raw[61] = pl >> 8; raw[62] = pl & 0xFF; raw[63] = pd >> 8; raw[64] = pd & 0xFF;
    for (uint8_t b = 0; b < 8; ++b) { uint16_t t = emu101::p2TogglesPerSec(b); raw[65 + 2 * b] = t >> 8; raw[66 + 2 * b] = t & 0xFF; }
    uint16_t ol = emu101::gateOnLenUs(), cg = emu101::cvToGateOffUs();
    raw[81] = ol >> 8; raw[82] = ol & 0xFF; raw[83] = cg >> 8; raw[84] = cg & 0xFF;
    uint16_t mi = emu101::midiNoteOnsPerSec(); raw[85] = mi >> 8; raw[86] = mi & 0xFF;
    uint16_t ks = emu101::keysSeenPerSec(), bs = emu101::btnsSeenPerSec();
    raw[87] = ks >> 8; raw[88] = ks & 0xFF; raw[89] = bs >> 8; raw[90] = bs & 0xFF;
    for (uint8_t r = 0; r < 3; ++r) { uint16_t c2 = emu101::shChangesPerSec(r); raw[91 + 2 * r] = c2 >> 8; raw[92 + 2 * r] = c2 & 0xFF; }
    // G5's key-scan bound. APPENDED at the end rather than placed with the other
    // emulator words, so no existing offset moves and probe101.py-style readers
    // that index by offset keep working (that exact class of break is why
    // DEVICES.md's 0x21 field count mattered). Max over the last second: the
    // gate wants a bound, not a sample.
    uint16_t kg = emu101::keyToGateUs(true); raw[125] = kg >> 8; raw[126] = kg & 0xFF;
  }
  {   // SuperOS LED / gate observability (hw.h P2Observer)
    hw::P2Observer& o = hw::P2Obs();
    raw[97] = o.last;
    for (uint8_t b = 0; b < 8; ++b) { raw[98 + 2 * b] = o.togPerSec[b] >> 8; raw[99 + 2 * b] = o.togPerSec[b] & 0xFF; }
  }
  {   // the 0x3E parked rail set (claim-038)
    const VccSamples& s = g_vccParkedSet;
    raw[114] = (uint8_t)(s.sum >> 16); raw[115] = (uint8_t)(s.sum >> 8); raw[116] = (uint8_t)s.sum;
    raw[117] = s.min >> 8; raw[118] = s.min & 0xFF;
    raw[119] = s.max >> 8; raw[120] = s.max & 0xFF;
    raw[121] = (uint8_t)(s.n >> 4);
    raw[122] = g_benchClkps;
    raw[123] = g_benchApplied;
  }
  raw[124] = persist::Health();   // store health (flash_persist.h), never mounts
  // THE PROBE USED TO REPORT ITSELF. combined_pass_us(true) clears the max on
  // read, and this reply is 257 bytes through the BLOCKING SendSysex (HalWrite
  // spins when the TX ring fills, midi.h:187), so the pass that answers a probe
  // is by far the longest pass the firmware runs. Reading the max here and
  // sending afterwards meant every probe reported the cost of the PREVIOUS
  // probe's own reply, and it read as a recurring ~14.8 ms hitch in the machine:
  // 14768 +/- 16 us, far too reproducible for anything but one code path running
  // once per probe. I reported that to CORE as a G7 defect on a machine that was
  // acting as a capture control before catching it.
  //
  // So: sample here, and have the next recorded pass excluded after the send
  // (combined_pass_skip_next, below). That does not hide a real hitch, it stops
  // manufacturing a fake one, and it makes the question decidable: after this, a
  // 14.8 ms max IS the machine.
  uint16_t pl = combined_pass_us(false), pm = combined_pass_us(true);
  raw[9] = pl >> 8; raw[10] = pl & 0xFF; raw[11] = pm >> 8; raw[12] = pm & 0xFF;
#ifdef __AVR__
  raw[0] = GPIOR1; raw[1] = USBCON; raw[2] = UDCON; raw[3] = UDIEN;
  raw[4] = USBSTA; raw[5] = UDFNUML;
#ifdef SUPEROS_USB_MIDI
  raw[6] = usb_configuration;
#endif
#endif
  // midi::SendSysex takes a uint8_t length, and this body is two nibbles per
  // raw byte plus the opcode. Past 127 raw bytes it would wrap and send a
  // truncated reply that still looks well-formed, which is the shape of bug
  // this fleet has already paid for once (claim-002's ring arithmetic). Fail
  // the build instead.
  static_assert(1 + 2 * sizeof(raw) <= 255, "0x3C probe body exceeds the uint8_t SysEx length");
  uint8_t body[1 + 2 * sizeof(raw)];
  body[0] = 0x3C;
  for (uint8_t i = 0; i < sizeof(raw); ++i) { body[1 + 2 * i] = raw[i] >> 4; body[2 + 2 * i] = raw[i] & 0x0F; }
  midi::SendSysex(body, sizeof(body));
  // Exclude the pass we just spent answering, so the next probe does not report
  // this one's 257-byte reply as a hitch in the machine. Clearing the max here
  // would NOT do it: the duration is recorded at the top of the next loop().
  // See combined_pass_skip_next() in boot.cpp.
  combined_pass_skip_next();
  // The pass counter was not the only counter the reply could reach. If this
  // 0x3B arrived between the ROM's keyboard read and its next P2 write, the
  // key-scan measurement is still armed and has just had the whole blocking
  // reply charged to it: measured bimodal at ~4.2 ms and ~19.9 ms, and an A/B
  // that injects one probe into the measured window reproduces the high mode on
  // demand (emu101.cpp, keyScanAbort). Drop it for the same reason and by the
  // same rule: telemetry may cost the machine time, but it may not then report
  // that time as the machine's.
  if (combined_current_fw() == FW_EMU) emu101::keyScanAbort();
}

// --- 0x3E bench window: which clock it runs at -------------------------------
// The window's job is to change the machine's load and clock for a few seconds
// so the rail can be read under each, so the ONE thing it must never get wrong
// is what clock it actually selected. The first version got it wrong twice:
// it assumed the app runs undivided, so it "restored" CLKPR to /1 and left the
// machine at 16 MHz after any /2 window, with the UART divisor still computed
// for 4 MHz; and with no way to ask for 16 MHz at all, the 16 MHz rows in the
// 2026-08-25 claim-038 sweep were that leak rather than the mode requested.
// These are pure so the sim can pin them (sim.cpp, G12).
//
//   bit0 0x01  core /2  -> 8 MHz
//   bit1 0x02  USB engine fully off (PLL stopped, clock frozen)
//   bit2 0x04  idle-sleep at the end of every pass
//   bit3 0x08  core /1  -> 16 MHz (wins over bit0)
//
// mode 0 leaves the app's own divide, which makes it the 4 MHz control rather
// than a no-op. The probe reports the clock that was APPLIED ([122]), so a
// row can no longer be labelled with a clock the machine was not running.
inline uint8_t BenchClkps(uint8_t mode, uint8_t baseClkps) {
  if (mode & 0x08) return 0;              // /1
  if (mode & 0x01) return 1;              // /2
  return baseClkps;                       // the app's own divide
}
// The app's F_CPU is the core clock at baseClkps, and each step of CLKPS below
// that doubles the real one.
inline uint32_t BenchCoreHz(uint32_t baseHz, uint8_t baseClkps, uint8_t clkps) {
  return clkps <= baseClkps ? (baseHz << (baseClkps - clkps))
                            : (baseHz >> (clkps - baseClkps));
}
inline uint8_t BenchUbrr(uint32_t coreHz) {
  return (uint8_t)(coreHz / 16 / 31250 - 1);   // 31250 baud, U2X off
}

// Returns true if the command was handled here.
inline bool HandleCommon(const uint8_t* body, uint8_t len) {
  if (len < 1) return true;
  switch (body[0]) {
    case 0x34: SendVersion(); return true;   // build identity, both firmwares
    case 0x3B: SendProbe(); return true;
    case 0x3D:   // bench: park the USB engine for body[1] seconds (usb_gate.h)
#if defined(__AVR__) && defined(SUPEROS_USB_MIDI)
      if (len >= 2) usb_gate_holdoff(body[1]);
#endif
      return true;
    case 0x3E:   // bench load experiment: <mode> <seconds> (boot.cpp g_benchMode)
#ifdef __AVR__
      if (len >= 3) { g_benchMode = body[1]; g_benchUntil = millis() + (uint32_t)body[2] * 1000; }
#endif
      return true;
    // Firmware switches run IN PROCESS at the top of the next loop() pass
    // (combined.h SwitchTo): the USB link never drops, so the host keeps its
    // port bindings and the editor just sees the 0x21 fw byte change.
    case 0x2F: if (emu101::available()) SwitchTo(FW_EMU); return true;
    case 0x4D: SwitchTo(FW_SUPEROS); return true;
    case 0x4A: EnterBootloader(); return true;
    case 0x40: if (combined_current_fw() == FW_EMU) SendEmuStatus(); return true;
    default: return false;
  }
}

} // namespace sysex
