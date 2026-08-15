// SuperOS-101 - DIN MIDI driver + parser (UART1, PD2 RX / PD3 TX)
//
// One driver for both firmwares. Interrupt-driven RX and TX rings so the loop
// pass (which spends ~350 us in DAC settle waits) can never overrun the UART's
// 2-byte hardware buffer at 320 us/byte. Poll() drains the RX ring through a
// small parser and hands out:
//   - realtime bytes            -> midiclk::OnRealtime() (clock/start/stop)
//   - channel messages          -> midi::Handlers (note on/off, CC, PC)
//   - SysEx F0 7D <cmd> .. F7   -> midi::Handlers.sysex(body, len), body[0]=cmd
// Everything else is dropped. Running status is honoured on input; output is
// always explicit-status.
//
// SysEx protocol (manufacturer 0x7D, same family as the SuperOS-303):
//   0x10 <p>              request pattern p          -> 0x11
//   0x11 <p> <xlo> <xhi> <7-bit packed pattern>      pattern data (both ways)
//   0x12 <p> <xlo> <xhi> <packed>                     set pattern p  -> 0x14
//   0x13                  request all patterns        -> 0x11 x NUM_PATTERNS
//   0x14 <code>           ack: 0 ok, 1 bad checksum, 2 bad pattern/length
//   0x15 <p> <step>       (device) sequencer step position, sent on change
//   0x1D <p>              select pattern p (as PLAY + key on the panel)
//   0x20                  request config              -> 0x21
//   0x21 <fw> <chan> <sync> <div> <pat> <version ascii...>   fw: 0 SuperOS
//   0x22 <chan> <sync> <div>  set config (chan 0..15, sync 0/1, div index)
//   0x2F                  switch to the 80C49 emulator (reboots)
//   0x40                  emulator status request     -> 0x41 (emulator only)
//   0x41 <1> <version ascii...>
//   0x4D                  switch to SuperOS (reboots)
//   0x4A                  enter the SysEx bootloader
// A SuperOS unit ignores 0x40 and an emulator unit ignores 0x20, so an editor
// sends both and whichever firmware runs answers.
#pragma once
#include <Arduino.h>

namespace midi {

static const uint8_t RX_CAP  = 64;    // power of two
static const uint8_t TX_CAP  = 128;   // power of two
static const uint8_t SX_MAX  = 160;   // largest inbound SysEx body (0x12 = 152)

struct Handlers {
  void (*noteOn )(uint8_t note, uint8_t vel) = nullptr;   // vel > 0
  void (*noteOff)(uint8_t note)              = nullptr;
  void (*cc     )(uint8_t num, uint8_t val)  = nullptr;
  void (*program)(uint8_t num)               = nullptr;
  void (*sysex  )(const uint8_t* body, uint8_t len) = nullptr; // body[0] = cmd
  void (*realtime)(uint8_t status)           = nullptr;
};

struct State {
  Handlers h;
  uint8_t  channel = 0;             // 0..15 (channel 1..16), receive + send
  // RX ring (ISR producer)
  volatile uint8_t rxBuf[RX_CAP];
  volatile uint8_t rxW = 0, rxR = 0;
  // TX ring (ISR consumer)
  volatile uint8_t txBuf[TX_CAP];
  volatile uint8_t txW = 0, txR = 0;
  // parser
  uint8_t  status = 0;              // running status (channel messages)
  uint8_t  data[2]; uint8_t nData = 0, need = 0;
  bool     inSx = false; bool sxDrop = false;
  uint8_t  sx[SX_MAX]; uint8_t sxLen = 0;
  bool     hostSeen = false;        // an editor spoke SysEx to us
};
inline State& S() { static State s; return s; }

// --- HAL -------------------------------------------------------------------
#ifdef __AVR__
inline void HalBegin() {
  UBRR1H = 0;
  UBRR1L = (uint8_t)(F_CPU / 16 / 31250 - 1);       // 31250 baud
  UCSR1A = 0;
  UCSR1C = (uint8_t)((1 << UCSZ11) | (1 << UCSZ10)); // 8N1
  UCSR1B = (uint8_t)((1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1));
  DDRD  |= (1 << 3);                                  // TX
  DDRD  &= (uint8_t)~(1 << 2); PORTD |= (1 << 2);     // RX with pull-up (idle high)
}
inline void HalKickTx() { UCSR1B |= (1 << UDRIE1); }
inline void HalIsrRx() {
  uint8_t b = UDR1;
  State& s = S();
  uint8_t w = (uint8_t)((s.rxW + 1) & (RX_CAP - 1));
  if (w != s.rxR) { s.rxBuf[s.rxW] = b; s.rxW = w; }   // else overrun: drop
}
inline void HalIsrTx() {
  State& s = S();
  if (s.txR == s.txW) { UCSR1B &= (uint8_t)~(1 << UDRIE1); return; }
  UDR1 = s.txBuf[s.txR];
  s.txR = (uint8_t)((s.txR + 1) & (TX_CAP - 1));
}
inline int16_t HalRead() {
  State& s = S();
  if (s.rxR == s.rxW) return -1;
  uint8_t b = s.rxBuf[s.rxR];
  s.rxR = (uint8_t)((s.rxR + 1) & (RX_CAP - 1));
  return b;
}
inline uint8_t TxFree() {
  State& s = S();
  return (uint8_t)((s.txR - s.txW - 1) & (TX_CAP - 1));
}
inline void HalWrite(uint8_t b) {
  State& s = S();
  uint8_t w = (uint8_t)((s.txW + 1) & (TX_CAP - 1));
  while (w == s.txR) { /* ring full: wait for the ISR to drain */ }
  s.txBuf[s.txW] = b; s.txW = w;
  HalKickTx();
}
#else
extern "C" int16_t sim_midi_pop();          // sim.cpp: -1 = queue empty
extern "C" void    sim_midi_out(uint8_t b); // sim.cpp: capture device output
inline void HalBegin() {}
inline int16_t HalRead() { return sim_midi_pop(); }
inline uint8_t TxFree() { return TX_CAP - 1; }
inline void HalWrite(uint8_t b) { sim_midi_out(b); }
#endif

// --- output ----------------------------------------------------------------
inline void Send(uint8_t b) { HalWrite(b); }
inline void NoteOn (uint8_t note, uint8_t vel = 100) {
  Send((uint8_t)(0x90 | S().channel)); Send(note & 0x7F); Send(vel & 0x7F);
}
inline void NoteOff(uint8_t note) {
  Send((uint8_t)(0x80 | S().channel)); Send(note & 0x7F); Send(0);
}
// SysEx: F0 7D body... F7 (body bytes must be < 0x80).
inline void SendSysex(const uint8_t* body, uint8_t len) {
  Send(0xF0); Send(0x7D);
  for (uint8_t i = 0; i < len; ++i) Send(body[i] & 0x7F);
  Send(0xF7);
}
// Non-blocking variant for telemetry: dropped if the ring cannot take it whole.
inline bool TrySendSysex(const uint8_t* body, uint8_t len) {
  if (TxFree() < (uint8_t)(len + 3)) return false;
  SendSysex(body, len);
  return true;
}

// 7-bit packing (same scheme as the 303: one MSB-bitmap byte per 7 data bytes).
inline uint8_t PackedLen(uint8_t n) { return (uint8_t)(n + (n + 6) / 7); }
inline uint8_t Pack7(const uint8_t* src, uint8_t n, uint8_t* dst) {
  uint8_t o = 0;
  for (uint8_t i = 0; i < n; i += 7) {
    uint8_t cnt = (uint8_t)((n - i < 7) ? (n - i) : 7);
    uint8_t msb = 0;
    for (uint8_t k = 0; k < cnt; ++k) if (src[i + k] & 0x80) msb |= (uint8_t)(1u << k);
    dst[o++] = msb;
    for (uint8_t k = 0; k < cnt; ++k) dst[o++] = src[i + k] & 0x7F;
  }
  return o;
}
// Returns unpacked byte count, or 0 on a malformed stream.
inline uint8_t Unpack7(const uint8_t* src, uint8_t n, uint8_t* dst, uint8_t cap) {
  uint8_t o = 0, i = 0;
  while (i < n) {
    uint8_t msb = src[i++];
    for (uint8_t k = 0; k < 7 && i < n; ++k) {
      if (o >= cap) return 0;
      dst[o++] = (uint8_t)(src[i++] | ((msb >> k) & 1 ? 0x80 : 0));
    }
  }
  return o;
}

// --- input -----------------------------------------------------------------
inline void Begin(uint8_t channel) { S().channel = channel & 0x0F; HalBegin(); }
inline void SetChannel(uint8_t c) { S().channel = c & 0x0F; }
inline uint8_t Channel() { return S().channel; }
inline bool HostSeen() { return S().hostSeen; }
inline void SetHandlers(const Handlers& h) { S().h = h; }

inline void Dispatch(uint8_t status, uint8_t d1, uint8_t d2) {
  State& s = S();
  if ((status & 0x0F) != s.channel) return;
  switch (status & 0xF0) {
    case 0x90: if (d2) { if (s.h.noteOn) s.h.noteOn(d1, d2); break; } /* fallthrough */
    case 0x80: if (s.h.noteOff) s.h.noteOff(d1); break;
    case 0xB0: if (s.h.cc) s.h.cc(d1, d2); break;
    case 0xC0: if (s.h.program) s.h.program(d1); break;
    default: break;
  }
}

inline void Feed(uint8_t b) {
  State& s = S();
  if (b >= 0xF8) { if (s.h.realtime) s.h.realtime(b); return; }   // realtime: transparent
  if (b == 0xF0) { s.inSx = true; s.sxDrop = false; s.sxLen = 0; s.status = 0; return; }
  if (b == 0xF7) {
    if (s.inSx) {
      s.inSx = false;
      if (!s.sxDrop && s.sxLen >= 2 && s.sx[0] == 0x7D) {
        s.hostSeen = true;
        if (s.h.sysex) s.h.sysex(s.sx + 1, (uint8_t)(s.sxLen - 1));
      }
    }
    return;
  }
  if (b & 0x80) {                       // status byte
    s.inSx = false;
    if (b >= 0xF0) { s.status = 0; s.need = 0; return; }   // other system common: ignore
    s.status = b; s.nData = 0;
    uint8_t hi = b & 0xF0;
    s.need = (hi == 0xC0 || hi == 0xD0) ? 1 : 2;
    return;
  }
  // data byte
  if (s.inSx) {
    if (s.sxLen < SX_MAX) s.sx[s.sxLen++] = b; else s.sxDrop = true;
    return;
  }
  if (!s.status) return;
  s.data[s.nData++] = b;
  if (s.nData >= s.need) {
    Dispatch(s.status, s.data[0], s.need == 2 ? s.data[1] : 0);
    s.nData = 0;                        // running status: keep s.status
  }
}

inline void Poll() {
  int16_t b;
  uint8_t budget = RX_CAP;               // bound the work per pass
  while (budget-- && (b = HalRead()) >= 0) Feed((uint8_t)b);
}

} // namespace midi
