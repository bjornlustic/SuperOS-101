// SuperOS-101 - USB-MIDI 1.0 event-packet codec (pure, no hardware).
//
// USB-MIDI moves MIDI as 4-byte events: [cable<<4 | CIN, b1, b2, b3]. The
// Code Index Number says how many of the three bytes are MIDI (table 4-1).
// Encode() turns a byte stream (as midi.h emits it: explicit status, SysEx
// F0 7D .. F7, realtime anywhere) into events; CinLen() is the decode side.
// Used by midi.h on the AVR and by the sim self-test on the host.
#pragma once
#include <stdint.h>

namespace usbmidi {

// MIDI bytes carried by each CIN.
inline uint8_t CinLen(uint8_t cin) {
  static const uint8_t k[16] = { 0, 0, 2, 3, 3, 1, 2, 3, 3, 3, 3, 3, 2, 2, 3, 1 };
  return k[cin & 0x0F];
}

struct Encoder {
  uint8_t b[3];
  uint8_t n = 0, need = 0;
  bool    sx = false;
  void reset() { n = 0; sx = false; }
};

// Feed one byte. Returns true when ev[0..3] holds a complete event to send.
// Realtime bytes (F8..FF) become their own event immediately, even mid-message,
// without disturbing the message in flight. System common F1..F6 is not
// produced by this firmware and is dropped.
inline bool Encode(Encoder& e, uint8_t byte, uint8_t ev[4]) {
  if (byte >= 0xF8) { ev[0] = 0x0F; ev[1] = byte; ev[2] = 0; ev[3] = 0; return true; }
  if (byte == 0xF0) { e.sx = true; e.b[0] = byte; e.n = 1; return false; }
  if (e.sx) {
    e.b[e.n++] = byte;
    if (byte == 0xF7) {                       // CIN 5/6/7: SysEx ends with 1/2/3 bytes
      ev[0] = (uint8_t)(0x04 + e.n);
      ev[1] = e.b[0]; ev[2] = e.n > 1 ? e.b[1] : 0; ev[3] = e.n > 2 ? e.b[2] : 0;
      e.sx = false; e.n = 0;
      return true;
    }
    if (e.n == 3) {                           // CIN 4: SysEx starts or continues
      ev[0] = 0x04; ev[1] = e.b[0]; ev[2] = e.b[1]; ev[3] = e.b[2];
      e.n = 0;
      return true;
    }
    return false;
  }
  if (byte & 0x80) {                          // status byte
    if (byte >= 0xF0) { e.n = 0; return false; }
    e.b[0] = byte; e.n = 1;
    uint8_t hi = byte & 0xF0;
    e.need = (hi == 0xC0 || hi == 0xD0) ? 2 : 3;
    return false;
  }
  if (!e.n) return false;                     // data with no status: drop
  e.b[e.n++] = byte;
  if (e.n < e.need) return false;
  ev[0] = (uint8_t)(e.b[0] >> 4);             // CIN = status high nibble
  ev[1] = e.b[0]; ev[2] = e.b[1]; ev[3] = e.need == 3 ? e.b[2] : 0;
  e.n = 0;
  return true;
}

} // namespace usbmidi
