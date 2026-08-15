// flash_persist.h -- device glue for the flash-arena store + block-id map.
//
// One FlashEeprom instance (g_flash) over the SPM service; on the host sim the
// arena is a RAM image persisted to sim/sim-flash.bin. Block ids:
//   FB_PATTERN_BASE + p   (p = 0..NUM_PATTERNS-1)  pattern p
//   FB_SETTINGS                                     reserved (settings live
//                                                   in internal EEPROM today)
// Pattern payload: [0] len (0..128), [1..len] step bytes = cv(6 bit) | type<<6
// (type: 0 note, 1 rest, 2 tie), i.e. the SH-101's own one-byte-per-step
// model. Max 129 bytes per record.
#pragma once
#include <stdint.h>
#include "flash_eeprom.h"

namespace persist {

static const uint8_t  NUM_PATTERNS    = 8;
static const uint16_t FB_PATTERN_BASE = 0;
static const uint16_t FB_SETTINGS     = 8;
static_assert(FB_SETTINGS + 1 <= FE_MAX_BLOCKS, "block map exceeds FE_MAX_BLOCKS");

#ifdef __AVR__
} // namespace persist
#include "flash_store.h"
namespace persist {
inline uint8_t ProgramHook(uint16_t page, const uint8_t* buf) { return flash_write_page(page, buf); }
inline void    ReadHook(uint16_t page, uint8_t* buf)          { flash_read_page(page, buf); }
inline bool    ServicePresent() { return flash_service_present(); }
#else
} // namespace persist
extern "C" uint8_t sim_flash_program(uint16_t page, const uint8_t* buf);
extern "C" void    sim_flash_read(uint16_t page, uint8_t* buf);
namespace persist {
inline uint8_t ProgramHook(uint16_t page, const uint8_t* buf) { return sim_flash_program(page, buf); }
inline void    ReadHook(uint16_t page, uint8_t* buf)          { sim_flash_read(page, buf); }
inline bool    ServicePresent() { return true; }
#endif

inline FlashEeprom& Store() { static FlashEeprom fe; return fe; }

// Mount the arena. Returns false when the SPM service is missing (bootloader
// without flash_service.c): the store then reads empty and writes fail, and
// the firmware keeps running with RAM-only patterns.
inline bool Begin() {
  static bool ok = false, done = false;
  if (!done) { done = true; ok = ServicePresent() && Store().begin(ProgramHook, ReadHook); }
  return ok;
}
inline bool Available() { return Begin(); }

} // namespace persist
