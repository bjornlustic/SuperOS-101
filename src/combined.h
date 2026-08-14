// SuperOS-101 — combined SuperOS + 80C49-emulator build
//
// One image, one firmware active per boot, selected by a byte in internal
// EEPROM and switchable from the config menu (src/menu.h) — the same scheme
// as the SuperOS-303 combined build.
//
// Internal EEPROM map (AT90USB1286, 4096 bytes):
//   0x000        firmware select: 0/0xFF = SuperOS, 1 = 80C49 emulator
//   0x001        settings magic (EE_CFG_MAGIC_VAL when initialized)
//   0x002        settings flags: bit0 = MIDI clock sync on
//   0x003        MIDI clock division index (midiclock.h table)
//   0x010..0x0DA SuperOS sequencer store (magic + len + 100 notes + 100 types)
//   0x0DB..      free (future: emulator battery-RAM persistence, user ROM store)
#pragma once
#include <stdint.h>

#define EE_FW_SELECT ((uint8_t*)0x000)
#define EE_CFG_MAGIC ((uint8_t*)0x001)
#define EE_CFG_FLAGS ((uint8_t*)0x002)
#define EE_CFG_DIV   ((uint8_t*)0x003)
static const uint8_t  EE_CFG_MAGIC_VAL = 0x4D;   // 'M'
static const uint16_t EE_SEQ_BASE = 0x010;

static const uint8_t FW_SUPEROS = 0;   // also 0xFF (virgin EEPROM)
static const uint8_t FW_EMU     = 1;

uint8_t combined_current_fw();
void RebootTo(uint8_t fw);   // persist the select byte, then reset the MCU
void EnterBootloader();      // park BOOT_MAGIC in GPIOR0, jump to 0x1F000
