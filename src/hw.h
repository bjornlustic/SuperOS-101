// SuperOS-101 — low-level SH-101 (80C49 socket) driver
//
// Faithfully reproduces the original CPU's pin-level protocol so the analog board
// behaves exactly as designed. The two invariants that MUST hold (see FIRMWARE.md):
//   1. DB0–5 are ACTIVE-LOW — we always output the one's-complement of the value.
//   2. The 4052 route is 11 (disconnected) during EVERY scan, so a held S/H CV is
//      never corrupted. ScanRead()/DacWrite() are built so this can't be violated.
//   3. A S/H is only ever (re)connected with its data already settled on the
//      ladder: present with route 11 first, then switch the route. This holds for
//      DacWrite() by construction; any host code that scans while an emulated
//      ROM holds a S/H connected must restore the BUS in those two steps too
//      (emu101.cpp menu poll / ShRefresh). A single restoring write reconnects
//      the S/H while the ladder is still slewing from the scan value: a blip on
//      the pitch S/H that the analog board's trigger detector turned into
//      envelope retriggers (heard as a per-step stutter, 2026-08-15).
#pragma once
#include <Arduino.h>
#include "pins.h"
#ifdef __AVR__
#include <avr/interrupt.h>
#include <avr/sleep.h>
#endif

namespace hw {

// --- Idle-sleep microsecond wait ---------------------------------------------
// The S/H settle dwells are the bulk of a loop pass, and the SH-101's +5 V
// regulator (the DAC reference) has almost no current headroom with this
// board (platformio.ini [env:app]). Instead of spinning, park the CPU core in
// idle sleep with Timer1 (F_CPU/8 ticks) as the wake alarm; the USB, UART and
// Timer0 interrupts still run and merely wake us early, after which we go
// back to sleep until the alarm. Wakes ~2 us late at most, never early. Falls
// back to a spin when interrupts are globally off (no wake source then).
// ISR(TIMER1_COMPA_vect) is an empty handler in boot.cpp.
#ifdef __AVR__
inline void IdleWaitUs(uint16_t us) {
  if (!(SREG & (1 << SREG_I))) { delayMicroseconds(us); return; }
  // Timer1 ticks at F_CPU/8: 0.5 tick/us at 4 MHz, 1 at 8, 2 at 16. Shifts,
  // not a 32-bit divide (that cost ~150 us per call at 4 MHz).
#if F_CPU == 4000000L
  const uint16_t ticks = (uint16_t)(us >> 1);
#elif F_CPU == 8000000L
  const uint16_t ticks = us;
#elif F_CPU == 16000000L
  const uint16_t ticks = (uint16_t)(us > 32767 ? 65535 : (us << 1));
#else
#error "IdleWaitUs: add the tick scale for this F_CPU"
#endif
  TCNT1  = 0;
  OCR1A  = ticks;
  TIFR1  = (1 << OCF1A);
  TIMSK1 = (1 << OCIE1A);
  for (;;) {
    cli();
    if (TCNT1 >= ticks) { sei(); break; }
    sleep_enable();
    sei();            // the instruction after sei() always runs: no lost wake
    sleep_cpu();
    sleep_disable();
  }
  TIMSK1 = 0;
}
#else
inline void IdleWaitUs(uint16_t us) { delayMicroseconds(us); }
#endif

// Settle times. The original's CPU-execution settle is only a few µs; on the analog
// side the R-2R ladder + 4052 sample/hold RC dominate. MEASURE on the real board and
// raise these to whatever the S/H needs to acquire/settle (TODO in docs/HARDWARE.md).
// Bench finding (first hardware, 2026-08-14): 10 µs acquire left the S/H
// caps near 0 V — gate fired but the VCO sat sub-audio ("gate but no note").
// 100 µs ≈ 5-10 RC of the 4052 Ron into the S/H cap; still MEASURE and trim.
static const uint8_t SETTLE_LADDER_US  = 10;  // after presenting data, before connecting S/H
static const uint8_t SETTLE_ACQUIRE_US = 100; // S/H acquisition while connected
static const uint8_t SETTLE_SCAN_US    = 3;   // column-low to P1-read settle

inline void Init() {
#ifdef __AVR__
  // Timer1 = the IdleWaitUs alarm: normal mode, F_CPU/8, compare interrupt
  // armed only inside a wait. (boot.cpp powers the unused timers down via
  // PRR; this one is needed.) Idle is the only sleep mode used.
  PRR0  &= (uint8_t)~(1 << PRTIM1);
  TCCR1A = 0;
  TCCR1B = (1 << CS11);
  TIMSK1 = 0;
  set_sleep_mode(SLEEP_MODE_IDLE);
#endif
  // BUS (DB0–7) output; idle = 0xFF = value 0 (active-low all-high) with route 11.
  BUS_DDR  = 0xFF;
  BUS_PORT = 0xFF;

  // P1 (P10–7) input WITH internal pull-ups. The original's "50K pullups"
  // (SH101.asm:63) are the 80C49's own quasi-bidirectional port pull-ups,
  // NOT board resistors — proven on hardware: without these, every row
  // floats low and reads as pressed. The AVR must supply its own.
  P1_DDR  = 0x00;
  P1_PORT = 0xFF;

  // P2 (P20–7) output; idle image (pins.h P2_IDLE): GATE off, LEDs off,
  // CLOCK RESET released (idle HIGH; holding it low freezes the LFO).
  P2_DDR  = 0xFF;
  P2_PORT = P2_IDLE;

  // T1 (clock) / T0 (HOLD) inputs. T0 gets the internal pull-up: an unwired
  // pedal jack must read inactive, not float low (a floating T0 latched HOLD
  // permanently during bring-up). T1 is left high-Z like the 80C49's T1: it
  // is driven by TR11 on the synth board (LFO / EXT CLK inverter), and a
  // pull-up there would load that node for nothing.
  DDRD  &= (uint8_t)~((1u << T1_CLOCK_BIT) | (1u << T0_HOLD_BIT));
  PORTD |= (uint8_t)(1u << T0_HOLD_BIT);
  PORTD &= (uint8_t)~(1u << T1_CLOCK_BIT);
}

// --- DAC + 4052 mux ---------------------------------------------------------
// Write a 6-bit value (0..63) to one sample/hold, reproducing the original order:
// present the (active-low) data with NO S/H connected -> let the ladder settle ->
// connect the target S/H so its cap acquires the settled voltage.
inline void DacWrite(uint8_t value, Route route) {
  uint8_t data = (uint8_t)~value;                 // active-low data on DB0–5
  BUS_PORT = data | (uint8_t)(ROUTE_NONE << 6);   // route 11: present, disconnected
  delayMicroseconds(SETTLE_LADDER_US);
  BUS_PORT = (uint8_t)((data & 0x3F) | (route << 6)); // connect S/H, data unchanged
  IdleWaitUs(SETTLE_ACQUIRE_US);                  // core asleep while the cap acquires
}

// --- Scan -------------------------------------------------------------------
// Pull one BUS column LOW (route stays 11 by construction) and read P1 RAW:
// a closed switch reads 0 (P1 is active-low, ~50 kΩ external pull-ups). This is
// the domain the original switch-decode logic uses (SH101.asm step 3).
inline uint8_t ScanReadRaw(ScanCol col) {
  BUS_PORT = (uint8_t)~(1u << col);   // bits 6,7 stay high -> route 11 (disconnected)
  delayMicroseconds(SETTLE_SCAN_US);
  return P1_PIN;                       // raw: 0 = closed
}

// Same scan, but inverted to the friendly domain: 1 = pressed/closed.
inline uint8_t ScanRead(ScanCol col) { return (uint8_t)~ScanReadRaw(col); }

// --- P2 outputs -------------------------------------------------------------
// Every P2 write goes through here, and here is where SuperOS is observed.
// The emulator already publishes what the ROM does with P2 (per-bit toggle
// counts, probe words [65..80]); the SuperOS side published nothing, so its
// LEDs, gate and CLOCK-RESET could only be checked in the sim. That gap is
// what stops G10's tempo blink and G11's progress bar being measurable on
// hardware: an indicator nobody can read is one nobody can test (G12).
//
// Two things are recorded, deliberately the same shape as the emulator's so
// one host-side reader covers both firmwares: the last image written (which
// LED is lit, HOLD, KEY-TRANSPOSE, GATE, CLOCK-RESET, PORTAMENTO OFF, all in
// one byte) and per-bit transitions per second (a blink RATE, which is what
// G10's "blinks on the incoming tempo" actually asserts).
//
// Cost is 33 bytes of SRAM and it is compiled into every build rather than
// hidden behind a probe flag, because a bench control that is absent from the
// shipping image measures a different firmware than the one that ships.
struct P2Observer {
  uint8_t  last = P2_IDLE;
  uint16_t togAcc[8] = {0}, togPerSec[8] = {0};
  uint32_t t0 = 0;

  inline void note(uint8_t value) {
    uint8_t d = (uint8_t)(value ^ last);
    last = value;
    for (uint8_t b = 0; b < 8; ++b) if (d & (1u << b)) ++togAcc[b];
  }
  // Roll the one-second window. Called from the loop, not from note(), so a
  // burst of writes cannot make the window shorter than a second.
  inline void tick(uint32_t ms) {
    if ((uint32_t)(ms - t0) < 1000) return;
    t0 = ms;
    for (uint8_t b = 0; b < 8; ++b) { togPerSec[b] = togAcc[b]; togAcc[b] = 0; }
  }
};
inline P2Observer& P2Obs() { static P2Observer o; return o; }

inline void P2Write(uint8_t value) { P2Obs().note(value); P2_PORT = value; }

// --- T0 / T1 ----------------------------------------------------------------
inline bool ClockLevel()     { return (PIND & (1u << T1_CLOCK_BIT)) != 0; }
inline bool HoldRequested()  { return (PIND & (1u << T0_HOLD_BIT)) == 0; } // active-low

} // namespace hw
