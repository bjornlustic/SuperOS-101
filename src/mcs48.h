// SuperOS-101 — MCS-48 (Intel 8049 / Toshiba TMP80C49) CPU core
//
// Portable interpreter for running the original SH-101 mask ROM on the
// AT90USB1286 (and in the host sim). Follows the SuperOS-303 layout, where a
// ucom4 core runs the TB-303's uPD650 ROM.
//
// The SH-101 ROM (per the Joe Britt disassembly header) uses: no interrupts,
// no timer/counter start (T is just a register), register bank 0 only, one
// stack level, no MOVX, no 8243 expander. The full base ISA is implemented
// anyway; unused corners (counter mode, expander ports, external RAM) are
// stubbed and marked.
//
// Speed matters here: the emulated ROM runs only as fast as the AVR (4 MHz
// core clock, see platformio.ini) can dispatch it, and the ROM's pass rate is
// its S/H refresh and clock-sample rate. So the run loop keeps its hot state
// in registers and avoids per-instruction work:
//   - the PC is a pointer into the 2 KB ROM image, which the includer places
//     2 KB-aligned (MCS48_ROM_BASE): fetch is one post-incremented flash read,
//     the 11-bit address is the pointer's low bits, and the page for JCOND /
//     MOVP is a mask on it. Linear execution never wraps 0x7FF -> 0 (the ROM
//     never runs off its end); JMP/CALL/RET/JMPP/JCOND all recompute from
//     the base.
//   - the register bank base is a cached pointer, re-derived only by the
//     three instructions that can change PSW.BS.
//   - the timer (unused by this ROM) is only serviced while running.
//
// Host hooks the includer must provide (see emu101.cpp / the sim):
//   MCS48_ROM_BASE        — macro (or mcs48_rom_base() hook): the 2 KB-aligned
//                           ROM image (const uint8_t* in PROGMEM on the AVR)
//   mcs48_bus_write(v)    — BUS/DB0-7 latch write (DAC + route + scan commons)
//   mcs48_p1_read()       — P1 pins (scan rows, external pull-ups)
//   mcs48_p2_write(v)     — P2 latch write (LEDs, GATE, CLOCK RESET, PORTA OFF)
//   mcs48_t0() / mcs48_t1() — T0 (HOLD, active-low) / T1 (clock) pin levels
// Optional native fast path (ROM-specific, see emu101.cpp): if the includer
// defines MCS48_HLE_ENTRY (an 11-bit ROM address) and
//   uint8_t mcs48_hle(const uint8_t*& pp, uint8_t& a, uint8_t& psw, uint8_t& f1,
//                     uint8_t* ram, uint8_t* regs)
// then whenever a MOV Rr,# lands the PC exactly on MCS48_HLE_ENTRY the hook
// runs instead of the interpreter, leaving the state as the ROM code would
// and returning the machine cycles it stood in for. (MOV Rr,# is the only
// hook point because the SH-101's hot loop is entered by falling through
// its `mov r6,#8`; a generic core would check every instruction.)
#pragma once
#include <stdint.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#define M48_LPM(p)      pgm_read_byte(p)
#define M48_TBL_ATTR    PROGMEM
// avr-gcc emits `goto *p` as a 2-clock ijmp only when p already sits in Z,
// otherwise as push/push/ret (8 clocks); the empty asm pins it into Z.
#define M48_DISPATCH(op) do { void* t_; \
    asm("" : "=z"(t_) : "0"((void*)pgm_read_word(&m48_tbl[(op)]))); goto *t_; } while (0)
#else
#define M48_LPM(p)      (*(const uint8_t*)(p))
#define M48_TBL_ATTR
#define M48_DISPATCH(op) goto *m48_tbl[(op)]
#endif
// The includer normally #defines MCS48_ROM_BASE to its ROM array before
// including this header (a link-time constant, best code); otherwise the
// hook function below is used (evaluated once per mcs48_run()).
#ifndef MCS48_ROM_BASE
const uint8_t* mcs48_rom_base();
#define MCS48_ROM_BASE mcs48_rom_base()
#endif

struct Mcs48 {
  uint16_t pc;        // 11-bit
  uint8_t  a, t, psw; // PSW: CY.80 AC.40 F0.20 BS.10 1.08 SP.07
  uint8_t  f1, dbf;
  uint8_t  p1, p2, bus;          // port latches
  uint8_t  timerOn, cntOn, tf;   // timer/counter (unused by this ROM)
  uint8_t  pre;                  // /32 machine-cycle prescaler
  uint8_t  ram[128];
};

uint8_t mcs48_rom_read(uint16_t addr);   // 2 KB mask ROM (inspection; the run loop reads MCS48_ROM_BASE)
void    mcs48_bus_write(uint8_t v);
uint8_t mcs48_p1_read();
void    mcs48_p2_write(uint8_t v);
bool    mcs48_t0();
bool    mcs48_t1();

inline void mcs48_reset(Mcs48& c) {
  c.pc = 0; c.a = 0; c.t = 0; c.psw = 0x08; c.f1 = 0; c.dbf = 0;
  c.p1 = 0xFF; c.p2 = 0xFF; c.bus = 0xFF;   // ports reset high (gate off)
  c.timerOn = c.cntOn = c.tf = 0; c.pre = 0;
  for (uint8_t i = 0; i < 128; ++i) c.ram[i] = 0;
}

// Run `insns` instructions (1..255; a fast path counts as one); returns the
// machine cycles they took. Instruction-counted rather than cycle-counted
// because that is the cheapest loop test on the AVR; the callers pace and
// account by the returned cycles.
#ifdef __GNUC__
__attribute__((optimize("O2")))
#endif
inline uint16_t mcs48_run(Mcs48& c, uint8_t insns) {
  const uint8_t* const rom = (const uint8_t*)MCS48_ROM_BASE;
  const uint8_t* pp = rom + (c.pc & 0x7FF);   // PC as a pointer into the ROM
  uint8_t  a   = c.a;
  uint8_t  psw = c.psw;
  uint8_t* ram = c.ram;
  uint8_t* regs = ram + ((psw & 0x10) ? 24 : 0);
  uint8_t  timerOn = c.timerOn;
  uint16_t ran = 0;

  // pointer <-> 11-bit address, page-relative targets (rom is 2 KB aligned)
  #define M48_PC()        ((uint16_t)((uintptr_t)pp & 0x7FF))
  #define M48_INPAGE(lo)  ((const uint8_t*)(((uintptr_t)pp & ~(uintptr_t)0xFF) | (lo)))
  #define M48_FETCH()     (cy = 2, M48_LPM(pp++))
  #define M48_REG(r)      regs[(r)]
  #define M48_MEM(r)      ram[M48_REG(r) & 0x7F]
  #define M48_SYNC_RB()   (regs = ram + ((psw & 0x10) ? 24 : 0))
  #define M48_JCOND(cond) do { uint8_t lo_ = M48_FETCH(); \
      if (cond) pp = M48_INPAGE(lo_); } while (0)
  #define M48_ADD(v, cin) do { uint8_t v_ = (v), ci_ = (cin); \
      uint16_t r_ = (uint16_t)a + v_ + ci_; \
      uint8_t ac_ = (uint8_t)(((a & 0x0F) + (v_ & 0x0F) + ci_) > 0x0F); \
      psw = (uint8_t)((psw & 0x3F) | ((r_ > 0xFF) ? 0x80 : 0) | (ac_ ? 0x40 : 0)); \
      a = (uint8_t)r_; } while (0)
  #define M48_PUSH() do { uint8_t sp_ = psw & 7; uint16_t pc_ = M48_PC(); \
      ram[8 + 2 * sp_]     = (uint8_t)(pc_ & 0xFF); \
      ram[8 + 2 * sp_ + 1] = (uint8_t)(((pc_ >> 8) & 0x0F) | (psw & 0xF0)); \
      psw = (uint8_t)((psw & 0xF8) | ((sp_ + 1) & 7)); } while (0)
  #define M48_POP_PC() do { uint8_t sp_ = (uint8_t)(((psw & 7) + 7) & 7); \
      pp = rom + (ram[8 + 2 * sp_] | ((ram[8 + 2 * sp_ + 1] & 0x0F) << 8)); \
      psw = (uint8_t)((psw & 0xF8) | sp_); } while (0)

  // Dispatch table (GCC computed goto), one entry per opcode; in flash on
  // the AVR. Cheaper than the switch's __tablejump2__ helper by ~8 clocks.
  static const void* const m48_tbl[256] M48_TBL_ATTR = {
    &&M48_L_00, &&M48_L_default, &&M48_L_02, &&M48_L_03, &&M48_L_default, &&M48_L_05, &&M48_L_default, &&M48_L_07,
    &&M48_L_08, &&M48_L_09, &&M48_L_0A, &&M48_L_default, &&M48_L_0C, &&M48_L_0D, &&M48_L_0E, &&M48_L_0F,
    &&M48_L_10, &&M48_L_11, &&M48_L_default, &&M48_L_13, &&M48_L_default, &&M48_L_15, &&M48_L_16, &&M48_L_17,
    &&M48_L_18, &&M48_L_19, &&M48_L_1A, &&M48_L_1B, &&M48_L_1C, &&M48_L_1D, &&M48_L_1E, &&M48_L_1F,
    &&M48_L_20, &&M48_L_21, &&M48_L_default, &&M48_L_23, &&M48_L_default, &&M48_L_25, &&M48_L_26, &&M48_L_27,
    &&M48_L_28, &&M48_L_29, &&M48_L_2A, &&M48_L_2B, &&M48_L_2C, &&M48_L_2D, &&M48_L_2E, &&M48_L_2F,
    &&M48_L_30, &&M48_L_31, &&M48_L_default, &&M48_L_default, &&M48_L_default, &&M48_L_35, &&M48_L_36, &&M48_L_37,
    &&M48_L_default, &&M48_L_39, &&M48_L_3A, &&M48_L_default, &&M48_L_3C, &&M48_L_3D, &&M48_L_3E, &&M48_L_3F,
    &&M48_L_40, &&M48_L_41, &&M48_L_42, &&M48_L_43, &&M48_L_default, &&M48_L_45, &&M48_L_46, &&M48_L_47,
    &&M48_L_48, &&M48_L_49, &&M48_L_4A, &&M48_L_4B, &&M48_L_4C, &&M48_L_4D, &&M48_L_4E, &&M48_L_4F,
    &&M48_L_50, &&M48_L_51, &&M48_L_default, &&M48_L_53, &&M48_L_default, &&M48_L_55, &&M48_L_56, &&M48_L_57,
    &&M48_L_58, &&M48_L_59, &&M48_L_5A, &&M48_L_5B, &&M48_L_5C, &&M48_L_5D, &&M48_L_5E, &&M48_L_5F,
    &&M48_L_60, &&M48_L_61, &&M48_L_62, &&M48_L_default, &&M48_L_default, &&M48_L_65, &&M48_L_default, &&M48_L_67,
    &&M48_L_68, &&M48_L_69, &&M48_L_6A, &&M48_L_6B, &&M48_L_6C, &&M48_L_6D, &&M48_L_6E, &&M48_L_6F,
    &&M48_L_70, &&M48_L_71, &&M48_L_default, &&M48_L_default, &&M48_L_default, &&M48_L_75, &&M48_L_76, &&M48_L_77,
    &&M48_L_78, &&M48_L_79, &&M48_L_7A, &&M48_L_7B, &&M48_L_7C, &&M48_L_7D, &&M48_L_7E, &&M48_L_7F,
    &&M48_L_80, &&M48_L_81, &&M48_L_default, &&M48_L_83, &&M48_L_default, &&M48_L_85, &&M48_L_86, &&M48_L_default,
    &&M48_L_88, &&M48_L_89, &&M48_L_8A, &&M48_L_default, &&M48_L_8C, &&M48_L_8D, &&M48_L_8E, &&M48_L_8F,
    &&M48_L_90, &&M48_L_91, &&M48_L_default, &&M48_L_93, &&M48_L_default, &&M48_L_95, &&M48_L_96, &&M48_L_97,
    &&M48_L_98, &&M48_L_99, &&M48_L_9A, &&M48_L_default, &&M48_L_9C, &&M48_L_9D, &&M48_L_9E, &&M48_L_9F,
    &&M48_L_A0, &&M48_L_A1, &&M48_L_default, &&M48_L_A3, &&M48_L_default, &&M48_L_A5, &&M48_L_default, &&M48_L_A7,
    &&M48_L_A8, &&M48_L_A9, &&M48_L_AA, &&M48_L_AB, &&M48_L_AC, &&M48_L_AD, &&M48_L_AE, &&M48_L_AF,
    &&M48_L_B0, &&M48_L_B1, &&M48_L_default, &&M48_L_B3, &&M48_L_default, &&M48_L_B5, &&M48_L_B6, &&M48_L_default,
    &&M48_L_B8, &&M48_L_B9, &&M48_L_BA, &&M48_L_BB, &&M48_L_BC, &&M48_L_BD, &&M48_L_BE, &&M48_L_BF,
    &&M48_L_default, &&M48_L_default, &&M48_L_default, &&M48_L_default, &&M48_L_default, &&M48_L_C5, &&M48_L_C6, &&M48_L_C7,
    &&M48_L_C8, &&M48_L_C9, &&M48_L_CA, &&M48_L_CB, &&M48_L_CC, &&M48_L_CD, &&M48_L_CE, &&M48_L_CF,
    &&M48_L_D0, &&M48_L_D1, &&M48_L_default, &&M48_L_D3, &&M48_L_default, &&M48_L_D5, &&M48_L_default, &&M48_L_D7,
    &&M48_L_D8, &&M48_L_D9, &&M48_L_DA, &&M48_L_DB, &&M48_L_DC, &&M48_L_DD, &&M48_L_DE, &&M48_L_DF,
    &&M48_L_default, &&M48_L_default, &&M48_L_default, &&M48_L_E3, &&M48_L_default, &&M48_L_E5, &&M48_L_E6, &&M48_L_E7,
    &&M48_L_E8, &&M48_L_E9, &&M48_L_EA, &&M48_L_EB, &&M48_L_EC, &&M48_L_ED, &&M48_L_EE, &&M48_L_EF,
    &&M48_L_F0, &&M48_L_F1, &&M48_L_default, &&M48_L_default, &&M48_L_default, &&M48_L_F5, &&M48_L_F6, &&M48_L_F7,
    &&M48_L_F8, &&M48_L_F9, &&M48_L_FA, &&M48_L_FB, &&M48_L_FC, &&M48_L_FD, &&M48_L_FE, &&M48_L_FF,
  };

  do {
    uint8_t op = M48_LPM(pp++);
    uint8_t cy = 1;
    M48_DISPATCH(op);
      M48_L_00: goto m48_done;                                              // NOP
      M48_L_02: c.bus = a; mcs48_bus_write(c.bus); cy = 2; goto m48_done;   // OUTL BUS,A
      M48_L_03: M48_ADD(M48_FETCH(), 0); goto m48_done;                     // ADD A,#
      M48_L_05: goto m48_done;                                              // EN I (no ext INT wired)
      M48_L_07: a--; goto m48_done;                                         // DEC A
      M48_L_08: a = c.bus; cy = 2; goto m48_done;                           // INS A,BUS (output-only bus)
      M48_L_09: a = mcs48_p1_read(); cy = 2; goto m48_done;                 // IN A,P1
      M48_L_0A: a = c.p2; cy = 2; goto m48_done;                            // IN A,P2 (latch)
      M48_L_0C: M48_L_0D: M48_L_0E: M48_L_0F: a = 0; cy = 2; goto m48_done; // MOVD A,Pp (no 8243)
      M48_L_10: M48_L_11: M48_MEM(op & 1)++; goto m48_done;                // INC @Rr
      M48_L_13: M48_ADD(M48_FETCH(), (uint8_t)(psw >> 7)); goto m48_done;   // ADDC A,#
      M48_L_15: goto m48_done;                                              // DIS I
      M48_L_16: { uint8_t f = c.tf; c.tf = 0; M48_JCOND(f); } goto m48_done;// JTF
      M48_L_17: a++; goto m48_done;                                         // INC A
      M48_L_18: M48_L_19: M48_L_1A: M48_L_1B:
      M48_L_1C: M48_L_1D: M48_L_1E: M48_L_1F: M48_REG(op & 7)++; goto m48_done; // INC Rr
      M48_L_20: M48_L_21: { uint8_t& m = M48_MEM(op & 1);          // XCH A,@Rr
          uint8_t t = a; a = m; m = t; } goto m48_done;
      M48_L_23: a = M48_FETCH(); goto m48_done;                             // MOV A,#
      M48_L_25: goto m48_done;                                              // EN TCNTI (ROM never enables)
      M48_L_26: M48_JCOND(!mcs48_t0()); goto m48_done;                      // JNT0
      M48_L_27: a = 0; goto m48_done;                                       // CLR A
      M48_L_28: M48_L_29: M48_L_2A: M48_L_2B:
      M48_L_2C: M48_L_2D: M48_L_2E: M48_L_2F: {                  // XCH A,Rr
          uint8_t& r = M48_REG(op & 7); uint8_t t = a; a = r; r = t; } goto m48_done;
      M48_L_30: M48_L_31: { uint8_t& m = M48_MEM(op & 1);          // XCHD A,@Rr
          uint8_t t = (uint8_t)(a & 0x0F);
          a = (uint8_t)((a & 0xF0) | (m & 0x0F));
          m = (uint8_t)((m & 0xF0) | t); } goto m48_done;
      M48_L_35: goto m48_done;                                              // DIS TCNTI
      M48_L_36: M48_JCOND(mcs48_t0()); goto m48_done;                       // JT0
      M48_L_37: a = (uint8_t)~a; goto m48_done;                             // CPL A
      M48_L_39: c.p1 = a; cy = 2; goto m48_done;                            // OUTL P1,A (latch only; P1 is input here)
      M48_L_3A: c.p2 = a; mcs48_p2_write(c.p2); cy = 2; goto m48_done;      // OUTL P2,A
      M48_L_3C: M48_L_3D: M48_L_3E: M48_L_3F: cy = 2; goto m48_done;     // MOVD Pp,A (no 8243)
      M48_L_40: M48_L_41: a |= M48_MEM(op & 1); goto m48_done;             // ORL A,@Rr
      M48_L_42: a = c.t; goto m48_done;                                     // MOV A,T
      M48_L_43: a |= M48_FETCH(); goto m48_done;                            // ORL A,#
      M48_L_45: c.cntOn = 1; timerOn = 0; goto m48_done;                    // STRT CNT (unused)
      M48_L_46: M48_JCOND(!mcs48_t1()); goto m48_done;                      // JNT1
      M48_L_47: a = (uint8_t)((a << 4) | (a >> 4)); goto m48_done;          // SWAP A
      M48_L_48: M48_L_49: M48_L_4A: M48_L_4B:
      M48_L_4C: M48_L_4D: M48_L_4E: M48_L_4F: a |= M48_REG(op & 7); goto m48_done; // ORL A,Rr
      M48_L_50: M48_L_51: a &= M48_MEM(op & 1); goto m48_done;             // ANL A,@Rr
      M48_L_53: a &= M48_FETCH(); goto m48_done;                            // ANL A,#
      M48_L_55: timerOn = 1; c.cntOn = 0; c.pre = 0; goto m48_done;         // STRT T (unused)
      M48_L_56: M48_JCOND(mcs48_t1()); goto m48_done;                       // JT1
      M48_L_57: {                                                   // DA A
          if ((a & 0x0F) > 9 || (psw & 0x40)) a = (uint8_t)(a + 6);
          if ((a >> 4) > 9 || (psw & 0x80)) { a = (uint8_t)(a + 0x60); psw |= 0x80; }
        } goto m48_done;
      M48_L_58: M48_L_59: M48_L_5A: M48_L_5B:
      M48_L_5C: M48_L_5D: M48_L_5E: M48_L_5F: a &= M48_REG(op & 7); goto m48_done; // ANL A,Rr
      M48_L_60: M48_L_61: M48_ADD(M48_MEM(op & 1), 0); goto m48_done;      // ADD A,@Rr
      M48_L_62: c.t = a; goto m48_done;                                     // MOV T,A
      M48_L_65: timerOn = 0; c.cntOn = 0; goto m48_done;                    // STOP TCNT
      M48_L_67: { uint8_t cy0 = (uint8_t)(psw >> 7);                // RRC A
          psw = (uint8_t)((psw & 0x7F) | ((a & 1) << 7));
          a = (uint8_t)((a >> 1) | (cy0 << 7)); } goto m48_done;
      M48_L_68: M48_L_69: M48_L_6A: M48_L_6B:
      M48_L_6C: M48_L_6D: M48_L_6E: M48_L_6F: M48_ADD(M48_REG(op & 7), 0); goto m48_done; // ADD A,Rr
      M48_L_70: M48_L_71: M48_ADD(M48_MEM(op & 1), (uint8_t)(psw >> 7)); goto m48_done;   // ADDC A,@Rr
      M48_L_75: goto m48_done;                                              // ENT0 CLK (unused)
      M48_L_76: M48_JCOND(c.f1); goto m48_done;                             // JF1
      M48_L_77: a = (uint8_t)((a >> 1) | (a << 7)); goto m48_done;          // RR A
      M48_L_78: M48_L_79: M48_L_7A: M48_L_7B:
      M48_L_7C: M48_L_7D: M48_L_7E: M48_L_7F:
          M48_ADD(M48_REG(op & 7), (uint8_t)(psw >> 7)); goto m48_done;      // ADDC A,Rr
      M48_L_80: M48_L_81: a = 0xFF; cy = 2; goto m48_done;                 // MOVX A,@Rr (no ext RAM)
      M48_L_83: M48_POP_PC(); cy = 2; goto m48_done;                        // RET
      M48_L_85: psw &= (uint8_t)~0x20; goto m48_done;                       // CLR F0
      M48_L_86: M48_JCOND(false); goto m48_done;                            // JNI (INT never asserted)
      M48_L_88: c.bus |= M48_FETCH(); mcs48_bus_write(c.bus); goto m48_done;// ORL BUS,#
      M48_L_89: c.p1 |= M48_FETCH(); goto m48_done;                         // ORL P1,# (latch only)
      M48_L_8A: c.p2 |= M48_FETCH(); mcs48_p2_write(c.p2); goto m48_done;   // ORL P2,#
      M48_L_8C: M48_L_8D: M48_L_8E: M48_L_8F: cy = 2; goto m48_done;     // ORLD Pp,A (no 8243)
      M48_L_90: M48_L_91: cy = 2; goto m48_done;                           // MOVX @Rr,A (no ext RAM)
      M48_L_93: { uint8_t sp = (uint8_t)(((psw & 7) + 7) & 7);      // RETR
          pp = rom + (ram[8 + 2 * sp] | ((ram[8 + 2 * sp + 1] & 0x0F) << 8));
          psw = (uint8_t)((ram[8 + 2 * sp + 1] & 0xF0) | 0x08 | sp);
          M48_SYNC_RB(); cy = 2; } goto m48_done;
      M48_L_95: psw ^= 0x20; goto m48_done;                                 // CPL F0
      M48_L_96: M48_JCOND(a != 0); goto m48_done;                           // JNZ
      M48_L_97: psw &= 0x7F; goto m48_done;                                 // CLR C
      M48_L_98: c.bus &= M48_FETCH(); mcs48_bus_write(c.bus); goto m48_done;// ANL BUS,#
      M48_L_99: c.p1 &= M48_FETCH(); goto m48_done;                         // ANL P1,# (latch only)
      M48_L_9A: c.p2 &= M48_FETCH(); mcs48_p2_write(c.p2); goto m48_done;   // ANL P2,#
      M48_L_9C: M48_L_9D: M48_L_9E: M48_L_9F: cy = 2; goto m48_done;     // ANLD Pp,A (no 8243)
      M48_L_A0: M48_L_A1: M48_MEM(op & 1) = a; goto m48_done;              // MOV @Rr,A
      M48_L_A3: a = M48_LPM(M48_INPAGE(a)); cy = 2; goto m48_done;          // MOVP A,@A
      M48_L_A5: c.f1 = 0; goto m48_done;                                    // CLR F1
      M48_L_A7: psw ^= 0x80; goto m48_done;                                 // CPL C
      M48_L_A8: M48_L_A9: M48_L_AA: M48_L_AB:
      M48_L_AC: M48_L_AD: M48_L_AE: M48_L_AF: M48_REG(op & 7) = a; goto m48_done; // MOV Rr,A
      M48_L_B0: M48_L_B1: M48_MEM(op & 1) = M48_FETCH(); goto m48_done;    // MOV @Rr,#
      M48_L_B3: pp = M48_INPAGE(M48_LPM(M48_INPAGE(a))); cy = 2; goto m48_done; // JMPP @A
      M48_L_B5: c.f1 ^= 1; goto m48_done;                                   // CPL F1
      M48_L_B6: M48_JCOND((psw & 0x20) != 0); goto m48_done;                // JF0
      M48_L_B8: M48_L_B9: M48_L_BA: M48_L_BB:
      M48_L_BC: M48_L_BD: M48_L_BE: M48_L_BF: M48_REG(op & 7) = M48_FETCH(); // MOV Rr,#
#ifdef MCS48_HLE_ENTRY
          if (pp == rom + MCS48_HLE_ENTRY) cy = (uint8_t)(cy + mcs48_hle(pp, a, psw, c.f1, ram, regs));
#endif
          goto m48_done;
      M48_L_C5: psw &= (uint8_t)~0x10; M48_SYNC_RB(); goto m48_done;        // SEL RB0
      M48_L_C6: M48_JCOND(a == 0); goto m48_done;                           // JZ
      M48_L_C7: a = psw; goto m48_done;                                     // MOV A,PSW
      M48_L_C8: M48_L_C9: M48_L_CA: M48_L_CB:
      M48_L_CC: M48_L_CD: M48_L_CE: M48_L_CF: M48_REG(op & 7)--; goto m48_done; // DEC Rr
      M48_L_D0: M48_L_D1: a ^= M48_MEM(op & 1); goto m48_done;             // XRL A,@Rr
      M48_L_D3: a ^= M48_FETCH(); goto m48_done;                            // XRL A,#
      M48_L_D5: psw |= 0x10; M48_SYNC_RB(); goto m48_done;                  // SEL RB1
      M48_L_D7: psw = (uint8_t)(a | 0x08); M48_SYNC_RB(); goto m48_done;    // MOV PSW,A
      M48_L_D8: M48_L_D9: M48_L_DA: M48_L_DB:
      M48_L_DC: M48_L_DD: M48_L_DE: M48_L_DF: a ^= M48_REG(op & 7); goto m48_done; // XRL A,Rr
      M48_L_E3: a = M48_LPM(rom + (0x300 | a)); cy = 2; goto m48_done;      // MOVP3 A,@A
      M48_L_E5: c.dbf = 0; goto m48_done;                                   // SEL MB0
      M48_L_E6: M48_JCOND(!(psw & 0x80)); goto m48_done;                    // JNC
      M48_L_E7: a = (uint8_t)((a << 1) | (a >> 7)); goto m48_done;          // RL A
      M48_L_E8: M48_L_E9: M48_L_EA: M48_L_EB:
      M48_L_EC: M48_L_ED: M48_L_EE: M48_L_EF: {                  // DJNZ Rr
          uint8_t lo = M48_FETCH();
          if (--M48_REG(op & 7)) pp = M48_INPAGE(lo);
        } goto m48_done;
      M48_L_F0: M48_L_F1: a = M48_MEM(op & 1); goto m48_done;              // MOV A,@Rr
      M48_L_F5: c.dbf = 1; goto m48_done;                                   // SEL MB1 (2 KB ROM: ignored)
      M48_L_F6: M48_JCOND((psw & 0x80) != 0); goto m48_done;                // JC
      M48_L_F7: { uint8_t cy0 = (uint8_t)(psw >> 7);                // RLC A
          psw = (uint8_t)((psw & 0x7F) | (a & 0x80));
          a = (uint8_t)((a << 1) | cy0); } goto m48_done;
      M48_L_F8: M48_L_F9: M48_L_FA: M48_L_FB:
      M48_L_FC: M48_L_FD: M48_L_FE: M48_L_FF: a = M48_REG(op & 7); goto m48_done;  // MOV A,Rr
      M48_L_default: {
        // JMP / CALL / JBb spread across the map; everything else is a NOP.
        uint8_t low5 = (uint8_t)(op & 0x1F);
        if (low5 == 0x04) {                                          // JMP addr
          uint8_t lo = M48_FETCH();
          pp = rom + ((((uint16_t)op & 0xE0) << 3) | lo);           // DBF ignored (2 KB ROM)
        } else if (low5 == 0x14) {                                   // CALL addr
          uint8_t lo = M48_FETCH();
          M48_PUSH();
          pp = rom + ((((uint16_t)op & 0xE0) << 3) | lo);
        } else if (low5 == 0x12) {                                   // JBb
          uint8_t bit = (uint8_t)((op >> 5) & 7);
          M48_JCOND((a >> bit) & 1);
        }
      } goto m48_done;
  m48_done:

    // Timer: xtal/480 = one tick per 32 machine cycles (unused by this ROM).
    if (__builtin_expect(timerOn, 0)) {
      c.pre = (uint8_t)(c.pre + cy);
      while (c.pre >= 32) { c.pre = (uint8_t)(c.pre - 32); if (++c.t == 0) c.tf = 1; }
    }
    ran = (uint16_t)(ran + cy);
  } while (--insns);

  c.pc = M48_PC(); c.a = a; c.psw = psw; c.timerOn = timerOn;
  return ran;

  #undef M48_PC
  #undef M48_INPAGE
  #undef M48_FETCH
  #undef M48_REG
  #undef M48_MEM
  #undef M48_SYNC_RB
  #undef M48_JCOND
  #undef M48_ADD
  #undef M48_PUSH
  #undef M48_POP_PC
}

// Execute one instruction; returns machine cycles taken (1 or 2).
inline uint8_t mcs48_step(Mcs48& c) { return (uint8_t)mcs48_run(c, 1); }
