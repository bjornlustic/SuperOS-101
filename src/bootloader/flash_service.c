/*
 * flash_service.c -- SPM flash-write service for SuperOS-101.
 *
 * SPM (flash self-write) can ONLY execute from the boot section, so the app
 * cannot write flash directly. This tiny service is linked into the bootloader
 * image at the top of the boot section (entry trampoline at byte 0x1FE00,
 * body from 0x1FE40) and the running app calls into it to erase+write one
 * 256-byte flash page. Same ABI and address as the SuperOS-303 service, so
 * the app-side caller (flash_store.h) is identical.
 *
 * ABI (avr-gcc default): page in r24:r25, buf pointer in r22:r23, returns
 * status in r24. avr:51 function pointers are WORD addresses, so the app calls
 * word 0xFF00 (= byte 0x1FE00).
 */
#include <avr/io.h>
#include <avr/boot.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <stdint.h>

#define PAGE_SIZE 256

/* Emulated-EEPROM arena window = 0x10000..0x1DFFF (RWW, below the 0x1E000
 * NRWW edge). The service refuses anything outside it, so a buggy app can
 * never erase the application code (pages 0x000..0x0FF), the boot section, or
 * the arena's neighbours. flash_eeprom.h currently uses 0x160..0x1DF. */
#define ARENA_FIRST_PAGE 0x100u
#define ARENA_LAST_PAGE  0x1DFu

uint8_t flash_service_impl(uint16_t page, const uint8_t *buf);

/* Trampoline pinned at byte 0x1FE00 by the linker (--section-start). The app
 * ICALLs here; this JMPs to the real implementation, leaving the argument
 * registers untouched so they pass straight through. */
__attribute__((section(".service_entry"), naked, used))
void flash_service_entry(void) {
  __asm__ __volatile__("jmp flash_service_impl");
}

__attribute__((section(".service_text"), used))
uint8_t flash_service_impl(uint16_t page, const uint8_t *buf) {
  if (page < ARENA_FIRST_PAGE || page > ARENA_LAST_PAGE)
    return 1; /* out of arena -- reject */

  /* page * 256 via shift so no __mul libgcc call is needed under -nostdlib. */
  uint32_t addr = (uint32_t)page << 8;

  uint8_t sreg = SREG;
  cli(); /* interrupt vectors live in RWW and are unreadable during the write */

  eeprom_busy_wait();

  /* The arena lives above the 64 KB boundary. SPM page erase/write address
   * the flash via RAMPZ:Z; set RAMPZ ourselves so the boot_page_* macros hit
   * the real page (page fill writes the temp buffer by word offset). */
  RAMPZ = (uint8_t)(addr >> 16);

  boot_page_erase(addr);
  boot_spm_busy_wait();

  for (uint16_t i = 0; i < PAGE_SIZE; i += 2) {
    uint16_t w = (uint16_t)buf[i] | ((uint16_t)buf[i + 1] << 8);
    boot_page_fill(addr + i, w);
  }

  boot_page_write(addr);
  boot_spm_busy_wait();
  boot_rww_enable();

  RAMPZ = 0; /* restore for plain LPM */
  SREG = sreg;
  return 0;
}
