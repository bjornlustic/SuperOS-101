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
// Host hooks the includer must provide (see emu101.cpp / the sim):
//   mcs48_rom_read(addr)  — 2 KB mask ROM
//   mcs48_bus_write(v)    — BUS/DB0-7 latch write (DAC + route + scan commons)
//   mcs48_p1_read()       — P1 pins (scan rows, external pull-ups)
//   mcs48_p2_write(v)     — P2 latch write (LEDs, GATE, CLOCK RESET, PORTA OFF)
//   mcs48_t0() / mcs48_t1() — T0 (HOLD, active-low) / T1 (clock) pin levels
#pragma once
#include <stdint.h>

struct Mcs48 {
  uint16_t pc;        // 11-bit
  uint8_t  a, t, psw; // PSW: CY.80 AC.40 F0.20 BS.10 1.08 SP.07
  uint8_t  f1, dbf;
  uint8_t  p1, p2, bus;          // port latches
  uint8_t  timerOn, cntOn, tf;   // timer/counter (unused by this ROM)
  uint8_t  pre;                  // /32 machine-cycle prescaler
  uint8_t  ram[128];
};

uint8_t mcs48_rom_read(uint16_t addr);
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

// Execute one instruction; returns machine cycles taken (1 or 2).
inline uint8_t mcs48_step(Mcs48& c) {
  uint8_t op = mcs48_rom_read(c.pc); c.pc = (c.pc + 1) & 0x7FF;
  uint8_t cy = 1;

  #define M48_FETCH()   (cy = 2, m48_tmp = mcs48_rom_read(c.pc), c.pc = (c.pc + 1) & 0x7FF, m48_tmp)
  uint8_t m48_tmp;
  #define M48_RB        ((c.psw & 0x10) ? 24 : 0)
  #define M48_REG(r)    c.ram[M48_RB + (r)]
  #define M48_MEM(r)    c.ram[M48_REG(r) & 0x7F]
  #define M48_JCOND(cond) do { uint8_t lo_ = M48_FETCH(); \
      if (cond) c.pc = (uint16_t)((c.pc & 0x700) | lo_); } while (0)
  #define M48_ADD(v, cin) do { uint8_t v_ = (v), ci_ = (cin); \
      uint16_t r_ = (uint16_t)c.a + v_ + ci_; \
      uint8_t ac_ = (uint8_t)(((c.a & 0x0F) + (v_ & 0x0F) + ci_) > 0x0F); \
      c.psw = (uint8_t)((c.psw & 0x3F) | ((r_ > 0xFF) ? 0x80 : 0) | (ac_ ? 0x40 : 0)); \
      c.a = (uint8_t)r_; } while (0)
  #define M48_PUSH() do { uint8_t sp_ = c.psw & 7; \
      c.ram[8 + 2 * sp_]     = (uint8_t)(c.pc & 0xFF); \
      c.ram[8 + 2 * sp_ + 1] = (uint8_t)(((c.pc >> 8) & 0x0F) | (c.psw & 0xF0)); \
      c.psw = (uint8_t)((c.psw & 0xF8) | ((sp_ + 1) & 7)); } while (0)

  switch (op) {
    case 0x00: break;                                              // NOP
    case 0x02: c.bus = c.a; mcs48_bus_write(c.bus); cy = 2; break; // OUTL BUS,A
    case 0x03: M48_ADD(M48_FETCH(), 0); break;                     // ADD A,#
    case 0x05: break;                                              // EN I (no ext INT wired)
    case 0x07: c.a--; break;                                       // DEC A
    case 0x08: c.a = c.bus; cy = 2; break;                         // INS A,BUS (output-only bus)
    case 0x09: c.a = mcs48_p1_read(); cy = 2; break;               // IN A,P1
    case 0x0A: c.a = c.p2; cy = 2; break;                          // IN A,P2 (latch)
    case 0x0C: case 0x0D: case 0x0E: case 0x0F: c.a = 0; cy = 2; break; // MOVD A,Pp (no 8243)
    case 0x10: case 0x11: M48_MEM(op & 1)++; break;                // INC @Rr
    case 0x13: M48_ADD(M48_FETCH(), (uint8_t)(c.psw >> 7)); break; // ADDC A,#
    case 0x15: break;                                              // DIS I
    case 0x16: { uint8_t f = c.tf; c.tf = 0; M48_JCOND(f); } break;// JTF
    case 0x17: c.a++; break;                                       // INC A
    case 0x18: case 0x19: case 0x1A: case 0x1B:
    case 0x1C: case 0x1D: case 0x1E: case 0x1F: M48_REG(op & 7)++; break; // INC Rr
    case 0x20: case 0x21: { uint8_t& m = M48_MEM(op & 1);          // XCH A,@Rr
        uint8_t t = c.a; c.a = m; m = t; } break;
    case 0x23: c.a = M48_FETCH(); break;                           // MOV A,#
    case 0x25: break;                                              // EN TCNTI (ROM never enables)
    case 0x26: M48_JCOND(!mcs48_t0()); break;                      // JNT0
    case 0x27: c.a = 0; break;                                     // CLR A
    case 0x28: case 0x29: case 0x2A: case 0x2B:
    case 0x2C: case 0x2D: case 0x2E: case 0x2F: {                  // XCH A,Rr
        uint8_t& r = M48_REG(op & 7); uint8_t t = c.a; c.a = r; r = t; } break;
    case 0x30: case 0x31: { uint8_t& m = M48_MEM(op & 1);          // XCHD A,@Rr
        uint8_t t = (uint8_t)(c.a & 0x0F);
        c.a = (uint8_t)((c.a & 0xF0) | (m & 0x0F));
        m   = (uint8_t)((m & 0xF0) | t); } break;
    case 0x35: break;                                              // DIS TCNTI
    case 0x36: M48_JCOND(mcs48_t0()); break;                       // JT0
    case 0x37: c.a = (uint8_t)~c.a; break;                         // CPL A
    case 0x39: c.p1 = c.a; cy = 2; break;                          // OUTL P1,A (latch only; P1 is input here)
    case 0x3A: c.p2 = c.a; mcs48_p2_write(c.p2); cy = 2; break;    // OUTL P2,A
    case 0x3C: case 0x3D: case 0x3E: case 0x3F: cy = 2; break;     // MOVD Pp,A (no 8243)
    case 0x40: case 0x41: c.a |= M48_MEM(op & 1); break;           // ORL A,@Rr
    case 0x42: c.a = c.t; break;                                   // MOV A,T
    case 0x43: c.a |= M48_FETCH(); break;                          // ORL A,#
    case 0x45: c.cntOn = 1; c.timerOn = 0; break;                  // STRT CNT (unused)
    case 0x46: M48_JCOND(!mcs48_t1()); break;                      // JNT1
    case 0x47: c.a = (uint8_t)((c.a << 4) | (c.a >> 4)); break;    // SWAP A
    case 0x48: case 0x49: case 0x4A: case 0x4B:
    case 0x4C: case 0x4D: case 0x4E: case 0x4F: c.a |= M48_REG(op & 7); break; // ORL A,Rr
    case 0x50: case 0x51: c.a &= M48_MEM(op & 1); break;           // ANL A,@Rr
    case 0x53: c.a &= M48_FETCH(); break;                          // ANL A,#
    case 0x55: c.timerOn = 1; c.cntOn = 0; c.pre = 0; break;       // STRT T (unused)
    case 0x56: M48_JCOND(mcs48_t1()); break;                       // JT1
    case 0x57: {                                                   // DA A
        if ((c.a & 0x0F) > 9 || (c.psw & 0x40)) c.a = (uint8_t)(c.a + 6);
        if ((c.a >> 4) > 9 || (c.psw & 0x80)) { c.a = (uint8_t)(c.a + 0x60); c.psw |= 0x80; }
      } break;
    case 0x58: case 0x59: case 0x5A: case 0x5B:
    case 0x5C: case 0x5D: case 0x5E: case 0x5F: c.a &= M48_REG(op & 7); break; // ANL A,Rr
    case 0x60: case 0x61: M48_ADD(M48_MEM(op & 1), 0); cy = 1; break; // ADD A,@Rr
    case 0x62: c.t = c.a; break;                                   // MOV T,A
    case 0x65: c.timerOn = 0; c.cntOn = 0; break;                  // STOP TCNT
    case 0x67: { uint8_t cy0 = (uint8_t)(c.psw >> 7);              // RRC A
        c.psw = (uint8_t)((c.psw & 0x7F) | ((c.a & 1) << 7));
        c.a = (uint8_t)((c.a >> 1) | (cy0 << 7)); } break;
    case 0x68: case 0x69: case 0x6A: case 0x6B:
    case 0x6C: case 0x6D: case 0x6E: case 0x6F: M48_ADD(M48_REG(op & 7), 0); break; // ADD A,Rr
    case 0x70: case 0x71: M48_ADD(M48_MEM(op & 1), (uint8_t)(c.psw >> 7)); break;   // ADDC A,@Rr
    case 0x75: break;                                              // ENT0 CLK (unused)
    case 0x76: M48_JCOND(c.f1); break;                             // JF1
    case 0x77: c.a = (uint8_t)((c.a >> 1) | (c.a << 7)); break;    // RR A
    case 0x78: case 0x79: case 0x7A: case 0x7B:
    case 0x7C: case 0x7D: case 0x7E: case 0x7F:
        M48_ADD(M48_REG(op & 7), (uint8_t)(c.psw >> 7)); break;    // ADDC A,Rr
    case 0x80: case 0x81: c.a = 0xFF; cy = 2; break;               // MOVX A,@Rr (no ext RAM)
    case 0x83: { uint8_t sp = (uint8_t)(((c.psw & 7) + 7) & 7);    // RET
        c.pc = (uint16_t)(c.ram[8 + 2 * sp] | ((c.ram[8 + 2 * sp + 1] & 0x0F) << 8));
        c.psw = (uint8_t)((c.psw & 0xF8) | sp); cy = 2; } break;
    case 0x85: c.psw &= (uint8_t)~0x20; break;                     // CLR F0
    case 0x86: M48_JCOND(false); break;                            // JNI (INT never asserted)
    case 0x88: c.bus |= M48_FETCH(); mcs48_bus_write(c.bus); break;// ORL BUS,#
    case 0x89: c.p1 |= M48_FETCH(); break;                         // ORL P1,# (latch only)
    case 0x8A: c.p2 |= M48_FETCH(); mcs48_p2_write(c.p2); break;   // ORL P2,#
    case 0x8C: case 0x8D: case 0x8E: case 0x8F: cy = 2; break;     // ORLD Pp,A (no 8243)
    case 0x90: case 0x91: cy = 2; break;                           // MOVX @Rr,A (no ext RAM)
    case 0x93: { uint8_t sp = (uint8_t)(((c.psw & 7) + 7) & 7);    // RETR
        c.pc = (uint16_t)(c.ram[8 + 2 * sp] | ((c.ram[8 + 2 * sp + 1] & 0x0F) << 8));
        c.psw = (uint8_t)((c.ram[8 + 2 * sp + 1] & 0xF0) | 0x08 | sp); cy = 2; } break;
    case 0x95: c.psw ^= 0x20; break;                               // CPL F0
    case 0x96: M48_JCOND(c.a != 0); break;                         // JNZ
    case 0x97: c.psw &= 0x7F; break;                               // CLR C
    case 0x98: c.bus &= M48_FETCH(); mcs48_bus_write(c.bus); break;// ANL BUS,#
    case 0x99: c.p1 &= M48_FETCH(); break;                         // ANL P1,# (latch only)
    case 0x9A: c.p2 &= M48_FETCH(); mcs48_p2_write(c.p2); break;   // ANL P2,#
    case 0x9C: case 0x9D: case 0x9E: case 0x9F: cy = 2; break;     // ANLD Pp,A (no 8243)
    case 0xA0: case 0xA1: M48_MEM(op & 1) = c.a; break;            // MOV @Rr,A
    case 0xA3: c.a = mcs48_rom_read((uint16_t)((c.pc & 0x700) | c.a)); cy = 2; break; // MOVP A,@A
    case 0xA5: c.f1 = 0; break;                                    // CLR F1
    case 0xA7: c.psw ^= 0x80; break;                               // CPL C
    case 0xA8: case 0xA9: case 0xAA: case 0xAB:
    case 0xAC: case 0xAD: case 0xAE: case 0xAF: M48_REG(op & 7) = c.a; break; // MOV Rr,A
    case 0xB0: case 0xB1: M48_MEM(op & 1) = M48_FETCH(); break;    // MOV @Rr,#
    case 0xB3: c.pc = (uint16_t)((c.pc & 0x700) |                  // JMPP @A
                  mcs48_rom_read((uint16_t)((c.pc & 0x700) | c.a))); cy = 2; break;
    case 0xB5: c.f1 ^= 1; break;                                   // CPL F1
    case 0xB6: M48_JCOND((c.psw & 0x20) != 0); break;              // JF0
    case 0xB8: case 0xB9: case 0xBA: case 0xBB:
    case 0xBC: case 0xBD: case 0xBE: case 0xBF: M48_REG(op & 7) = M48_FETCH(); break; // MOV Rr,#
    case 0xC5: c.psw &= (uint8_t)~0x10; break;                     // SEL RB0
    case 0xC6: M48_JCOND(c.a == 0); break;                         // JZ
    case 0xC7: c.a = c.psw; break;                                 // MOV A,PSW
    case 0xC8: case 0xC9: case 0xCA: case 0xCB:
    case 0xCC: case 0xCD: case 0xCE: case 0xCF: M48_REG(op & 7)--; break; // DEC Rr
    case 0xD0: case 0xD1: c.a ^= M48_MEM(op & 1); break;           // XRL A,@Rr
    case 0xD3: c.a ^= M48_FETCH(); break;                          // XRL A,#
    case 0xD5: c.psw |= 0x10; break;                               // SEL RB1
    case 0xD7: c.psw = (uint8_t)(c.a | 0x08); break;               // MOV PSW,A
    case 0xD8: case 0xD9: case 0xDA: case 0xDB:
    case 0xDC: case 0xDD: case 0xDE: case 0xDF: c.a ^= M48_REG(op & 7); break; // XRL A,Rr
    case 0xE3: c.a = mcs48_rom_read((uint16_t)(0x300 | c.a)); cy = 2; break;   // MOVP3 A,@A
    case 0xE5: c.dbf = 0; break;                                   // SEL MB0
    case 0xE6: M48_JCOND(!(c.psw & 0x80)); break;                  // JNC
    case 0xE7: c.a = (uint8_t)((c.a << 1) | (c.a >> 7)); break;    // RL A
    case 0xE8: case 0xE9: case 0xEA: case 0xEB:
    case 0xEC: case 0xED: case 0xEE: case 0xEF: {                  // DJNZ Rr
        uint8_t lo = M48_FETCH();
        if (--M48_REG(op & 7)) c.pc = (uint16_t)((c.pc & 0x700) | lo);
      } break;
    case 0xF0: case 0xF1: c.a = M48_MEM(op & 1); break;            // MOV A,@Rr
    case 0xF5: c.dbf = 1; break;                                   // SEL MB1 (2 KB ROM: ignored)
    case 0xF6: M48_JCOND((c.psw & 0x80) != 0); break;              // JC
    case 0xF7: { uint8_t cy0 = (uint8_t)(c.psw >> 7);              // RLC A
        c.psw = (uint8_t)((c.psw & 0x7F) | (c.a & 0x80));
        c.a = (uint8_t)((c.a << 1) | cy0); } break;
    case 0xF8: case 0xF9: case 0xFA: case 0xFB:
    case 0xFC: case 0xFD: case 0xFE: case 0xFF: c.a = M48_REG(op & 7); break;  // MOV A,Rr
    default: {
      // JMP / CALL / JBb spread across the map; everything else is a NOP.
      uint8_t low5 = (uint8_t)(op & 0x1F);
      if (low5 == 0x04) {                                          // JMP addr
        uint8_t lo = M48_FETCH();
        c.pc = (uint16_t)((((uint16_t)op & 0xE0) << 3) | lo);      // DBF ignored (2 KB ROM)
      } else if (low5 == 0x14) {                                   // CALL addr
        uint8_t lo = M48_FETCH();
        M48_PUSH();
        c.pc = (uint16_t)((((uint16_t)op & 0xE0) << 3) | lo);
      } else if (low5 == 0x12) {                                   // JBb
        uint8_t bit = (uint8_t)((op >> 5) & 7);
        M48_JCOND((c.a >> bit) & 1);
      }
    } break;
  }

  // Timer: xtal/480 = one tick per 32 machine cycles (unused by this ROM).
  if (c.timerOn) {
    c.pre = (uint8_t)(c.pre + cy);
    while (c.pre >= 32) { c.pre = (uint8_t)(c.pre - 32); if (++c.t == 0) c.tf = 1; }
  }

  #undef M48_FETCH
  #undef M48_RB
  #undef M48_REG
  #undef M48_MEM
  #undef M48_JCOND
  #undef M48_ADD
  #undef M48_PUSH
  return cy;
}
