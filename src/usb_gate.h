// SuperOS-101 - USB-C engine gating for the SUPEROS_USB_MIDI build.
// Ported from SuperOS-303 (src/usb_sof.h), which inherited it from the
// SuperOS-606 bring-up. Same AT90USB1286, same Teensy usb_midi core.
//
// usb_sof_alive(): gate for every USB-MIDI write. The core's TX wait
// (send_raw in cores/usb_midi/usb_api.cpp) uses the USB frame counter
// UDFNUML as its timeout clock. SOFs stop when the host parks the port
// (failed enumeration, reset storm, cable pulled at the wrong moment) while
// usb_configuration can still read as set, and then every send spins forever
// waiting for bank room. SOF = 1 ms cadence when the bus is alive; 5 ms of
// stillness = USB is down, drop the byte instead of risking the spin.
//
// usb_vbus_gate(): attach exactly when a cable is present. VBUS high ->
// attach (PLL/engine clock brought up if needed); VBUS low -> detach with the
// USB device interrupts masked. The unplugged branch is ENFORCED every pass,
// never edge-triggered: with the cable out, the floating D+/D- lines fake
// wake-up events and the core's WAKEUPI handler would otherwise re-attach
// behind the gate's back (606 field bug, 2026-08-01). The plugged branch only
// intervenes when WE parked the port (DETACH set); a legitimate host suspend
// stays the core's business. Called once per loop() pass.
//
// SH-101 specific: the PLL and the engine clock are NOT stopped when
// unplugged. The synth's +5 V regulator (VR-1, "D/A WIDTH") is the R-2R DAC's
// reference and is loaded to its limit by this board: the USB engine's
// current alone moved VCC 4.98 -> 4.58 V and the pitch by ~7 semitones
// (measured 2026-08-15). Keeping the engine clocked whether or not a cable
// is in keeps the load, and therefore the tuning, constant; VR-1 is trimmed
// once with this board installed (README, Tuning).
#pragma once
#if defined(__AVR__) && defined(SUPEROS_USB_MIDI)
#include <Arduino.h>
#include <util/delay.h>

static inline bool usb_sof_alive() {
  static uint8_t  last_fn     = 0;
  static uint32_t last_adv_ms = 0;
  const uint8_t  fn  = UDFNUML;
  const uint32_t now = millis();
  if (fn != last_fn) { last_fn = fn; last_adv_ms = now; return true; }
  return (uint32_t)(now - last_adv_ms) < 5;
}

// Bench hold-off (SysEx 0x3D <s>): treat the port as unplugged until this
// time, so the effect of the running USB engine on the analog side can be
// heard without touching the cable. 0 = no hold-off.
extern uint32_t g_usbHoldUntil;   // defined in boot.cpp
static inline void usb_gate_holdoff(uint16_t seconds) { g_usbHoldUntil = millis() + (uint32_t)seconds * 1000; }

static inline void usb_vbus_gate() {
  // Boot settle: stay parked for the first 400 ms even with a cable in, so
  // setup()-time scans and init run with the engine off, and, after a
  // firmware switch or bootloader round trip, the host has finished tearing
  // down the previous device before this one re-attaches (re-attaching into
  // that teardown left the host resetting the bus mid-enumeration).
  const bool held = g_usbHoldUntil && (int32_t)(millis() - g_usbHoldUntil) < 0;
  if (!held) g_usbHoldUntil = 0;
  if ((USBSTA & _BV(VBUS)) && millis() >= 400 && !held) {
    if (UDCON & _BV(DETACH)) {                // parked by us: bring USB up
      if (!(PLLCSR & _BV(PLOCK))) {
        PLLCSR = 0x16;                        // core's PLL_CONFIG() for the
        // AT90USB1286 @ 16 MHz; lock takes ~100 us. Bounded wait (the core's
        // own usb_init waits forever, see the boot guard in boot.cpp): no
        // lock within ~2 ms -> stay parked this pass, retry next pass.
        uint16_t spins = 2000;
        while (!(PLLCSR & _BV(PLOCK)) && --spins) _delay_us(1);
        if (!(PLLCSR & _BV(PLOCK))) { PLLCSR = 0; return; }
      }
      USBCON &= (uint8_t)~_BV(FRZCLK);        // engine clock on
      UDIEN   = _BV(EORSTE) | _BV(SOFE);      // the core's normal ISR set
      UDCON  &= (uint8_t)~_BV(DETACH);        // attach: host enumerates
    }
  } else {
    UDIEN   = 0;                              // no wake/suspend ISR ping-pong
    UDCON  |= _BV(DETACH);
    // engine clock and PLL stay running: constant rail load (see header)
    if (!(PLLCSR & _BV(PLOCK))) { PLLCSR = 0x16; }
    else USBCON &= (uint8_t)~_BV(FRZCLK);
  }
}

// Park the engine (detached, clock frozen, device ISRs off) right after the
// core's pre-main usb_init() brought it up live; the per-pass gate re-attaches
// it (after the 150 ms boot settle) if a cable is in. Call first in setup().
static inline void usb_gate_begin() {
  UDIEN   = 0;
  UDCON  |= _BV(DETACH);       // engine clock keeps running (constant load)
}

// Drop off the bus before a reset / bootloader jump so the host registers
// the detach and re-enumerates the next identity cleanly.
static inline void usb_gate_detach() {
  UDIEN = 0;
  UDCON |= _BV(DETACH);
  _delay_ms(30);
  USBCON = 0;
  // The PLL is left running: the AVR's PLL can miss lock when it is stopped
  // and restarted within milliseconds (bench 2026-08-15: bootloader entry
  // from the emulator came up without USB), and the next stage (bootloader,
  // or the core's usb_init after a watchdog reset) rewrites PLLCSR anyway.
}
#endif
