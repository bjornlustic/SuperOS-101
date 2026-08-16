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

#ifndef SUPEROS_VERSION
#define SUPEROS_VERSION "dev"
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
//            The R-2R DAC's reference is this rail, so it is the pitch scale.
inline uint16_t MeasureVccMv() {
#ifdef __AVR__
  uint8_t prr = PRR0; PRR0 &= (uint8_t)~(1 << PRADC);
  ADMUX  = (1 << REFS0) | 0x1E;                       // AVCC ref, 1.1 V bandgap in
  // ADC clock 50-200 kHz: /128 at 8-16 MHz, /64 below
  ADCSRA = (uint8_t)((1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (F_CPU >= 8000000UL ? (1 << ADPS0) : 0));
  delayMicroseconds(400);                             // bandgap settle
  uint32_t acc = 0;
  for (uint8_t i = 0; i < 9; ++i) {                   // discard first, average 8
    ADCSRA |= (1 << ADSC); while (ADCSRA & (1 << ADSC)) {}
    if (i) acc += ADC;
  }
  ADCSRA = 0; PRR0 = prr;
  uint16_t adc = (uint16_t)(acc / 8);
  return adc ? (uint16_t)((1100UL * 1024UL) / adc) : 0;
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
}  // namespace sysex
extern uint16_t g_vccParkedMv;   // boot.cpp
extern uint8_t  g_benchMode;     // boot.cpp (SysEx 0x3E)
extern uint32_t g_benchUntil;
namespace sysex {
inline void SendProbe() {
  uint8_t raw[97] = {0};
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
  }
  uint16_t pl = combined_pass_us(false), pm = combined_pass_us(true);
  raw[9] = pl >> 8; raw[10] = pl & 0xFF; raw[11] = pm >> 8; raw[12] = pm & 0xFF;
#ifdef __AVR__
  raw[0] = GPIOR1; raw[1] = USBCON; raw[2] = UDCON; raw[3] = UDIEN;
  raw[4] = USBSTA; raw[5] = UDFNUML;
#ifdef SUPEROS_USB_MIDI
  raw[6] = usb_configuration;
#endif
#endif
  uint8_t body[1 + 2 * sizeof(raw)];
  body[0] = 0x3C;
  for (uint8_t i = 0; i < sizeof(raw); ++i) { body[1 + 2 * i] = raw[i] >> 4; body[2 + 2 * i] = raw[i] & 0x0F; }
  midi::SendSysex(body, sizeof(body));
}

// Returns true if the command was handled here.
inline bool HandleCommon(const uint8_t* body, uint8_t len) {
  if (len < 1) return true;
  switch (body[0]) {
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
    case 0x2F: if (emu101::available()) RebootTo(FW_EMU); return true;   // does not return
    case 0x4D: RebootTo(FW_SUPEROS); return true;
    case 0x4A: EnterBootloader(); return true;
    case 0x40: if (combined_current_fw() == FW_EMU) SendEmuStatus(); return true;
    default: return false;
  }
}

} // namespace sysex
