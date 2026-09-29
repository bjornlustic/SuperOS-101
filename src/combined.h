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
//   0x004        MIDI channel 0..15 (sysex.h)
//   0x010        CV DAC settings magic (cvdac.h, EE_CV_MAGIC_VAL)
//   0x011..0x01C CV DAC settings: center(2), cutoff CC, flags, accent mode,
//                VCF/VCA amounts, decay, velocity threshold, res depth, build-up,
//                board variant (1 = bipolar, default; 0 = unreworked first-batch v1.1)
//   0x020        feature magic (EE_FEAT_MAGIC_VAL), 0x021 sequencer direction,
//   0x022        pedal route (main.cpp LoadFeatures)
//   0x030..      free (SuperOS patterns live in the flash arena, flash_persist.h)
//   0x100        emulator RAM-image magic (EE_EMU_MAGIC_VAL when valid)
//   0x101..0x164 emulator "battery RAM": the 80C49's RAM 0x1C..0x7F (the 100
//                sequencer steps the original kept alive on its battery; the
//                ROM's START clears 0x02..0x1B itself). emu101.cpp.
#pragma once
#include <stdint.h>

// The EEPROM map. Addresses are INTEGER constants and the pointers are derived
// from them, because a cast from a pointer to an integer is not a constant
// expression and every static_assert below silently failed to compile as one.
//
// Why the asserts are here at all: A808 measured two record overlaps in one
// instrument on 2026-08-26, one of them rewriting the first five bytes of the
// emulator pattern store every four seconds. Silent by construction, because
// nothing reads back what it did not write, so an overlap looks exactly like
// data that was never saved.
//
// The 101 is CLEAN today and has no EEPROM-published telemetry at all: its
// probe is live SysEx 0x3B and src/diag/ touches no EEPROM. These exist so the
// NEXT record cannot be added without the map being checked, which is the part
// that was missing rather than the part that was wrong.
static const uint16_t EE_A_FW_SELECT = 0x000;
static const uint16_t EE_A_CFG_MAGIC = 0x001;
static const uint16_t EE_A_CFG_FLAGS = 0x002;
static const uint16_t EE_A_CFG_DIV   = 0x003;
static const uint16_t EE_A_CFG_CHAN  = 0x004;
static const uint16_t EE_A_CFG_LFODIV = 0x005;  // LFO hard-sync division (midiclock.h)
static const uint16_t EE_CFG_END     = 0x006;   // one past EE_A_CFG_LFODIV
static const uint16_t EE_CV_FIRST    = 0x010;   // cvdac.h's EE_CV_MAGIC
static const uint16_t EE_A_FEAT_MAGIC = 0x020;  // SuperOS performance settings (main.cpp)
static const uint16_t EE_A_FEAT_DIR   = 0x021;  // sequencer direction (engine.h SeqDir)
static const uint16_t EE_A_FEAT_PEDAL = 0x022;  // pedal route (controls.h PedalMode)
static const uint16_t EE_FEAT_END     = 0x023;
static const uint16_t EE_FREE_FIRST  = 0x030;
static const uint16_t EE_A_EMU_MAGIC = 0x100;
static const uint16_t EE_A_EMU_RAM   = 0x101;
// Named EE_TOTAL_BYTES, not EE_SIZE: sim/sim.cpp already has an EE_SIZE of
// 4096 for the same quantity and the two collided on type. Same value, and the
// collision is the sim confirming the number rather than contradicting it.
static const uint16_t EE_TOTAL_BYTES = 0x1000;  // E2END + 1, AT90USB1286

#define EE_EMU_MAGIC ((uint8_t*)EE_A_EMU_MAGIC)
#define EE_EMU_RAM   ((uint8_t*)EE_A_EMU_RAM)
#define EE_FW_SELECT ((uint8_t*)EE_A_FW_SELECT)
#define EE_CFG_MAGIC ((uint8_t*)EE_A_CFG_MAGIC)
#define EE_CFG_FLAGS ((uint8_t*)EE_A_CFG_FLAGS)
#define EE_CFG_DIV   ((uint8_t*)EE_A_CFG_DIV)
#define EE_CFG_CHAN  ((uint8_t*)EE_A_CFG_CHAN)
#define EE_CFG_LFODIV ((uint8_t*)EE_A_CFG_LFODIV)

static const uint8_t EE_EMU_MAGIC_VAL = 0x49;   // '8049'
static const uint8_t EMU_RAM_FIRST    = 0x1C;   // first persisted 80C49 RAM byte
static const uint8_t EMU_RAM_COUNT    = 0x80 - 0x1C;   // 100 bytes
static const uint8_t EE_CFG_MAGIC_VAL = 0x4D;   // 'M'
static const uint8_t EE_FEAT_MAGIC_VAL = 0x46;  // 'F'
static_assert(EE_FEAT_END <= EE_FREE_FIRST, "the feature block runs into the free arena");

static_assert(EE_A_CFG_LFODIV + 1 == EE_CFG_END, "the CFG block moved");
static_assert(EE_CFG_END <= EE_CV_FIRST, "the CFG block runs into the CV block");
static_assert(EE_FREE_FIRST <= EE_A_EMU_MAGIC,
              "the free arena runs into the emulator RAM image");
static_assert(EE_A_EMU_RAM == EE_A_EMU_MAGIC + 1,
              "the emulator RAM image must follow its magic byte");
// The emulator image is the LAST record, so this is the one bound with nothing
// after it to catch an overrun.
static_assert(EE_A_EMU_RAM + EMU_RAM_COUNT <= EE_TOTAL_BYTES,
              "the emulator RAM image runs past the end of EEPROM");

static const uint8_t FW_SUPEROS = 0;   // also 0xFF (virgin EEPROM)
static const uint8_t FW_EMU     = 1;

uint8_t combined_current_fw();
uint8_t combined_boot_count();  // boots since power-on (survives resets; SysEx 0x3B)
uint8_t combined_prev_stage();  // where the previous boot ended (SysEx 0x3B)
uint16_t combined_pass_us(bool clearMax); // loop pass time: last (false) / max since last clear (true)
void combined_pass_skip_next();  // drop the next recorded pass from the max (the probe's own reply pass)
// Switch firmware IN PROCESS: persist the select byte, tear one side down,
// bring the other up. No MCU reset and no USB detach: both firmwares are one
// image with one USB identity, so the host keeps its bindings (a reboot here
// used to re-enumerate and strand every host that had the port open:
// finding 8, and stale Chrome/Ableton clients on this bench). The request is
// deferred to the top of the next loop() pass, since it arrives from inside
// the running firmware's own dispatch.
void SwitchTo(uint8_t fw);
void EnterBootloader();      // park BOOT_MAGIC in GPIOR0, jump to 0x1F000
