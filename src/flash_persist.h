// flash_persist.h -- device glue for the flash-arena store + block-id map.
//
// One FlashEeprom instance (g_flash) over the SPM service; on the host sim the
// arena is a RAM image persisted to sim/sim-flash.bin. Block ids:
//   FB_PATTERN_BASE + p   (p = 0..NUM_PATTERNS-1)  pattern p
//   FB_SETTINGS                                     reserved (settings live
//                                                   in internal EEPROM today)
// Pattern payload: [0] len (0..128), [1..len] step bytes = cv(6 bit) | type<<6
// (type: 0 note, 1 rest, 2 tie), i.e. the SH-101's own one-byte-per-step
// model, then ceil(len/4) packed ratchet bytes (2 bits/step, 0 = single hit,
// 1 = 2x, 2 = 3x). Records from before ratchets are 1+len bytes and load with
// ratchets off. Max 161 bytes per record (FE_MAX_PAYLOAD is 242).
#pragma once
#include <stdint.h>
#include "flash_eeprom.h"

namespace persist {

static const uint8_t  NUM_PATTERNS    = 16;   // keys 1-16; each up to 128 steps
static const uint16_t FB_PATTERN_BASE = 0;
static const uint16_t FB_SETTINGS     = 16;   // reserved, unused (was 8 with 8 patterns; never written)
// Per-step filter CV lane of pattern p (engine.h): MAX_STEPS bytes, 0..127, in
// its own block like the SuperOS-303's (fcv_codec.h) so the pattern record and
// its format are untouched; an older store reads these ids as absent (all 0).
static const uint16_t FB_FILTER_BASE  = 17;
static_assert(FB_FILTER_BASE + NUM_PATTERNS <= FE_MAX_BLOCKS, "block map exceeds FE_MAX_BLOCKS");

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
//
// The mount result is kept at namespace scope rather than inside Begin() so a
// diagnostic can ASK whether the store is live without mounting it as a side
// effect. That matters because of A606's claim-036: on the 606 nothing was
// reaching flash at all while the RAM cache answered every read-back, so the
// writes acked and looked correct and only a reboot exposed it. The real fix
// there was observability, and a health bit nobody can read is not one.
inline bool& MountAttempted() { static bool v = false; return v; }
inline bool& MountOk()        { static bool v = false; return v; }
inline bool Begin() {
  if (!MountAttempted()) {
    MountAttempted() = true;
    MountOk() = ServicePresent() && Store().begin(ProgramHook, ReadHook);
  }
  return MountOk();
}
inline bool Available() { return Begin(); }

// Store health for the 0x3B probe. Read-only: never mounts.
//   bit0 the bootloader's SPM service answered
//   bit1 a mount has been attempted
//   bit2 the arena is mounted and writable
// The 101 cannot lose its service the way the 606 did: flash_service.c is
// inside this machine's OWN bootloader (it keeps its own, claim-005), and an
// app .syx cannot reach the boot section. Reported anyway, because "cannot
// happen by construction" is what the 606 believed too.
inline uint8_t Health() {
  return (uint8_t)((ServicePresent() ? 1 : 0) |
                   (MountAttempted() ? 2 : 0) |
                   (MountAttempted() && MountOk() ? 4 : 0));
}

} // namespace persist
