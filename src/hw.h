// SuperOS-101 — low-level SH-101 (80C49 socket) driver
//
// Faithfully reproduces the original CPU's pin-level protocol so the analog board
// behaves exactly as designed. The two invariants that MUST hold (see FIRMWARE.md):
//   1. DB0–5 are ACTIVE-LOW — we always output the one's-complement of the value.
//   2. The 4052 route is 11 (disconnected) during EVERY scan, so a held S/H CV is
//      never corrupted. ScanRead()/DacWrite() are built so this can't be violated.
#pragma once
#include <Arduino.h>
#include "pins.h"

namespace hw {

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

  // T1 (clock) / T0 (HOLD) inputs, WITH internal pull-ups: both are
  // active-low/driven lines on a fully wired board, but an unwired pedal
  // jack or clock line must read inactive, not float low (a floating T0
  // latched HOLD permanently during bring-up).
  DDRD  &= (uint8_t)~((1u << T1_CLOCK_BIT) | (1u << T0_HOLD_BIT));
  PORTD |= (uint8_t)((1u << T1_CLOCK_BIT) | (1u << T0_HOLD_BIT));
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
  delayMicroseconds(SETTLE_ACQUIRE_US);
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
inline void P2Write(uint8_t value) { P2_PORT = value; }

// --- T0 / T1 ----------------------------------------------------------------
inline bool ClockLevel()     { return (PIND & (1u << T1_CLOCK_BIT)) != 0; }
inline bool HoldRequested()  { return (PIND & (1u << T0_HOLD_BIT)) == 0; } // active-low

} // namespace hw
