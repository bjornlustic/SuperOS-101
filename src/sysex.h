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

// Returns true if the command was handled here.
inline bool HandleCommon(const uint8_t* body, uint8_t len) {
  if (len < 1) return true;
  switch (body[0]) {
    case 0x2F: if (emu101::available()) RebootTo(FW_EMU); return true;   // does not return
    case 0x4D: RebootTo(FW_SUPEROS); return true;
    case 0x4A: EnterBootloader(); return true;
    case 0x40: if (combined_current_fw() == FW_EMU) SendEmuStatus(); return true;
    default: return false;
  }
}

} // namespace sysex
