// flash_store.h -- app-side access to the boot-section SPM flash service.
//
// The routine that actually writes flash lives in the bootloader image
// (src/bootloader/flash_service.c, byte 0x1FE00); this is just the caller
// plus a service-present probe. Same entry/ABI as SuperOS-303.
#pragma once
#include <Arduino.h>
#include <avr/pgmspace.h>

static constexpr uint32_t FLASH_SERVICE_ENTRY = 0x1FE00UL;
// avr:51 function pointers are WORD addresses: byte 0x1FE00 -> word 0xFF00.
typedef uint8_t (*FlashServiceFn)(uint16_t page, const uint8_t *buf);
static const FlashServiceFn kFlashService = (FlashServiceFn)(FLASH_SERVICE_ENTRY >> 1);

static constexpr uint32_t FLASH_PAGE_SIZE = 256;
static constexpr uint8_t  FLASH_OK             = 0;
static constexpr uint8_t  FLASH_ERR_NO_SERVICE = 0xFE; // SPM service not installed

// True if the SPM service trampoline is installed at the entry address. The
// entry is a JMP whose first opcode word is 0x940C.
inline bool flash_service_present() {
  return pgm_read_word_far(FLASH_SERVICE_ENTRY) == 0x940C;
}

// Erase + write one 256-byte page from a RAM buffer. Returns FLASH_OK on
// success. Halts the CPU for a few ms during the write (interrupts off:
// the MIDI UART rings hold ~20 ms of traffic, so a write between loop passes
// is safe). Never calls into the boot section unless the service is present.
inline uint8_t flash_write_page(uint16_t page, const uint8_t *buf) {
  if (!flash_service_present()) return FLASH_ERR_NO_SERVICE;
  return kFlashService(page, buf);
}

inline void flash_read_page(uint16_t page, uint8_t *buf) {
  uint32_t a = (uint32_t)page * FLASH_PAGE_SIZE;
  for (uint16_t i = 0; i < FLASH_PAGE_SIZE; ++i) buf[i] = pgm_read_byte_far(a + i);
}
