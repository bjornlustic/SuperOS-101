// SuperOS-101 — AT90USB1286 ↔ SH-101 (80C49 socket) pin map
//
// The 80C49 has three 8-bit groups; we map each onto a whole AVR port so the
// firmware can do single-instruction port reads/writes (matching SuperOS-303/606).
// Only the *meaning* of each port differs from the 303. Full rationale + the
// behavioural spec are in docs/HARDWARE.md and docs/FIRMWARE.md.
//
//   AVR PORTC  ->  80C49 BUS  (DB0–DB7)   OUTPUT   DAC value + 4052 route + scan commons
//   AVR PORTB  ->  80C49 P1   (P10–P17)   INPUT    keyboard / switch / button scan rows
//   AVR PORTF  ->  80C49 P2   (P20–P27)   OUTPUT   LED selects, GATE, CLOCK-RESET, PORTA-OFF
//   AVR PD0    ->  80C49 T1                INPUT    clock in (LFO / EXT-CLK), INT0-capable
//   AVR PD1    ->  80C49 T0                INPUT    HOLD in (pedal/panel), active-low
//   AVR PD2/3  ->  (new) MIDI UART1 RX/TX           added via the panel breakout
//
// NOTE: these AVR↔socket assignments are the *proposed* board wiring. They are
// freely chooseable when we lay out the new DIP-40 board; the firmware only assumes
// the three whole-port groupings above. Adjust here if the layout changes.
#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// PORTC = BUS (DB0–DB7), output
// ---------------------------------------------------------------------------
// DB0–DB5 = 6-bit DAC value, ACTIVE-LOW (we output the one's-complement).
// DB6–DB7 = 4052 route select.  DB0–DB3 double as active-low keyboard-bank
// commons; DB4 = switch common; DB5 = button common (see ScanCol below).
#define BUS_PORT   PORTC
#define BUS_DDR    DDRC

// 4052 route (DB7,DB6). Reproduces the original encoding (FIRMWARE.md):
enum Route : uint8_t {
  ROUTE_CV_OUT = 0,  // 00 -> note pitch / KCV (CV-out jack + VCO pitch)
  ROUTE_VCO    = 1,  // 01 -> range + key-shift (octave offset) to VCO
  ROUTE_RANDOM = 2,  // 10 -> random waveform (random-LFO / S&H)
  ROUTE_NONE   = 3,  // 11 -> disconnected (settle / scan-safe)
};

// Scan columns: which BUS line is pulled LOW. DB6/DB7 stay HIGH (route 11) for
// every scan, so a held S/H CV can never be corrupted (invariant in FIRMWARE.md).
enum ScanCol : uint8_t {
  COL_KBD_BANK0 = 0, // DB0 low: keys F3–C4
  COL_KBD_BANK1 = 1, // DB1 low: C#4–G#4
  COL_KBD_BANK2 = 2, // DB2 low: A4–E5
  COL_KBD_BANK3 = 3, // DB3 low: F5–C6
  COL_SWITCHES  = 4, // DB4 low: RANGE (S3) / ADSR (S7) / OCT up-down
  COL_BUTTONS   = 5, // DB5 low: function buttons (LOAD/PLAY/DOWN/U&D/UP/HOLD/KEY-TRANSP)
};

// ---------------------------------------------------------------------------
// PORTB = P1 (P10–P17), input — scan rows
// ---------------------------------------------------------------------------
// The original's "50K pullups" on P1 (SH101.asm:63) are the 80C49's internal
// quasi-bidirectional port pull-ups — there are none on the SH-101 board, so
// we enable the AVR's internal pull-ups to match. A closed switch reads 0.
// ISP (SCK/MOSI/MISO = PB1/2/3) shares this port but is only active during
// programming, exactly as on the 303.
#define P1_PIN     PINB
#define P1_DDR     DDRB
#define P1_PORT    PORTB

// ---------------------------------------------------------------------------
// PORTF = P2 (P20–P27), output
// ---------------------------------------------------------------------------
// Bit positions within the P2 byte (from SH101.asm / FIRMWARE.md).
#define P2_PORT    PORTF
#define P2_DDR     DDRF

// P20–P22 together select one function LED via IC8 (4556); see SH101.asm:90-128.
enum P2Bit : uint8_t {
  P2_LED_SEL0 = 0,   // P20  function-LED select bit 0
  P2_LED_SEL1 = 1,   // P21  function-LED select bit 1
  P2_LED_SEL2 = 2,   // P22  function-LED select bit 2
  P2_PORTA_OFF = 3,  // P23  PORTAMENTO OFF
  P2_CLOCK_RST = 4,  // P24  CLOCK RESET
  P2_LED_HOLD = 5,   // P25  HOLD LED select
  P2_LED_KTRANS = 6, // P26  KEY-TRANSPOSE LED select
  P2_GATE = 7,       // P27  GATE  (INVERTED: bit=0 -> gate ON, bit=1 -> gate OFF)
};

// Idle P2 image, matching the original ROM's resting value (0xB7 with LED
// select 7; select 0 is the same "all off" per the 4556 truth table):
//   GATE off (P27=1), KT LED off (P26=0), HOLD LED off (P25=1),
//   CLOCK RESET idle HIGH (P24=1, the reset pulse is active-LOW),
//   PORTAMENTO OFF idle low (P23=0), function-LED select 0 = none lit.
#define P2_IDLE ((uint8_t)((1u << P2_GATE) | (1u << P2_LED_HOLD) | (1u << P2_CLOCK_RST)))

// ---------------------------------------------------------------------------
// PORTD = T1 (clock), T0 (HOLD), MIDI
// ---------------------------------------------------------------------------
#define T1_CLOCK_BIT  0  // PD0 / INT0  — clock input (active level per board)
#define T0_HOLD_BIT   1  // PD1 / INT1  — HOLD input, active-low (closed = HOLD)
// PD2 = MIDI RX (Serial1), PD3 = MIDI TX (Serial1) — wired later via breakout.
