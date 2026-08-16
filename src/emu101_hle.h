// SuperOS-101 — 80C49 emulator: native fast path for the SH-101 ROM's key scan
//
// Included by emu101.cpp (and the lockstep test harness) after rom101_data.h
// and before mcs48.h. Requires: ROM101, pgm_read_byte, PROGMEM.
#pragma once
#include <stdint.h>

// --- native fast path for the ROM's per-key scan loop --------------------------
// keyboard_read (SH101.asm:626-668) checks the 8 keys of a bank one bit at a
// time: rrc / jc / jf1 / (last-note-priority bookkeeping) / inc r3 / djnz r6.
// At 8 cycles per key and 32 keys that loop is ~40% of every ROM pass, and
// the pass rate is what the emulator is short of (S/H refresh, clock
// sampling). The loop is entered only by falling through `mov r6,#8` at
// 0x0C4, so mcs48.h hands control here when a MOV Rr,# lands on 0x0C6; this
// executes the loop natively, instruction for instruction (same registers,
// same carry, same F1, same exit PC, same cycle count), and returns to the
// interpreter at whichever exit the ROM would have taken. Enabled only if
// the ROM bytes match the expected code (HleCheck, at setup).
//   0C6 rrc a            0C7 jc 0D7          0C9 jf1 0DD
//   0CB xch a,r4  0CC dec a  0CD xch a,r2  0CE rrc a  0CF xch a,r2
//   0D0 jnz 0DC   0D2 jc 0DC   0D4 cpl f1   0D5 jmp 0AC
//   0D7 (key_is_down)    0DC xch a,r4   0DD inc r3   0DE djnz r6,0C6   0E0 (rd_next_bank)
static const uint8_t HLE_SIG[] PROGMEM = {
  0xBE,0x08, 0x67, 0xF6,0xD7, 0x76,0xDD, 0x2C, 0x07, 0x2A, 0x67, 0x2A, 0x96,0xDC,
  0xF6,0xDC, 0xB5, 0x04,0xAC, 0x95, 0xFB, 0xA1, 0x04,0xE0, 0x2C, 0x1B, 0xEE,0xC6 };
static const uint16_t HLE_SIG_AT = 0x0C4;
static bool g_hleOk = false;
static bool HleCheck() {
  for (uint8_t i = 0; i < sizeof(HLE_SIG); ++i)
    if (pgm_read_byte(&ROM101[HLE_SIG_AT + i]) != pgm_read_byte(&HLE_SIG[i])) return false;
  return true;
}
#define MCS48_HLE_ENTRY 0x0C6
static inline uint8_t mcs48_hle(const uint8_t*& pp, uint8_t& a, uint8_t& psw, uint8_t& f1,
                                uint8_t* /*ram*/, uint8_t* regs) {
  if (!g_hleOk) return 0;
  const uint8_t* const rom = ROM101;
  uint8_t r2 = regs[2], r3 = regs[3], r4 = regs[4], r6 = regs[6];
  uint8_t C = (uint8_t)(psw >> 7);
  uint8_t cyc = 0, t;
  for (;;) {
    t = (uint8_t)(a & 1); a = (uint8_t)((a >> 1) | (C << 7)); C = t;   // 0C6 rrc a
    cyc = (uint8_t)(cyc + 3);                                             // + 0C7 jc
    if (C) { pp = rom + 0x0D7; break; }                                   // key_is_down
    cyc = (uint8_t)(cyc + 2);                                             // 0C9 jf1
    if (!f1) {                                                            // last-note priority
      t = a; a = r4; r4 = t;                                              // 0CB xch a,r4
      a--;                                                                // 0CC dec a
      t = a; a = r2; r2 = t;                                              // 0CD xch a,r2
      t = (uint8_t)(a & 1); a = (uint8_t)((a >> 1) | (C << 7)); C = t;   // 0CE rrc a
      t = a; a = r2; r2 = t;                                              // 0CF xch a,r2
      cyc = (uint8_t)(cyc + 7);                                           // 5 x 1 + 0D0 jnz
      if (a == 0) {
        cyc = (uint8_t)(cyc + 2);                                         // 0D2 jc
        if (!C) {                                                         // 0D4 cpl f1, 0D5 jmp 0AC
          f1 ^= 1; cyc = (uint8_t)(cyc + 3);
          pp = rom + 0x0AC; break;
        }
      }
      t = a; a = r4; r4 = t; cyc = (uint8_t)(cyc + 1);                    // 0DC xch a,r4
    }
    r3++; cyc = (uint8_t)(cyc + 3);                                       // 0DD inc r3, 0DE djnz r6
    if (--r6) continue;
    pp = rom + 0x0E0; break;                                              // rd_next_bank
  }
  regs[2] = r2; regs[3] = r3; regs[4] = r4; regs[6] = r6;
  psw = (uint8_t)((psw & 0x7F) | (C << 7));
  return cyc;
}
