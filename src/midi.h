// SuperOS-101 - MIDI driver + parser: DIN (UART1, PD2 RX / PD3 TX) and, in the
// SUPEROS_USB_MIDI build, the USB-C port (Teensy usb_midi core, one cable).
//
// One driver for both firmwares. Interrupt-driven RX and TX rings so the loop
// pass (which spends ~350 us in DAC settle waits) can never overrun the UART's
// 2-byte hardware buffer at 320 us/byte. Poll() drains the DIN RX ring and the
// USB OUT endpoint, each through its own parser instance (so a SysEx arriving
// on USB cannot be corrupted by notes on DIN and vice versa), and hands out:
//   - realtime bytes            -> midiclk::OnRealtime() (clock/start/stop)
//   - channel messages          -> midi::Handlers (note on/off, CC, PC)
//   - SysEx F0 7D <cmd> .. F7   -> midi::Handlers.sysex(body, len), body[0]=cmd
// Everything else is dropped. Running status is honoured on input; output is
// always explicit-status.
//
// Output routing: notes / realtime go to every port (Send). SysEx (editor
// replies, step position, mirrored edits) goes to the ports on which an editor
// has spoken SysEx to us (sticky per boot), or to every port before any has.
// So a USB-only editor session never blocks the loop on the 31250-baud DIN
// ring, and a DIN editor never sees its replies vanish into an unread USB pipe.
//
// USB path: raw 4-byte USB-MIDI event packets are read straight from the OUT
// endpoint and unpacked by Code Index Number into the byte parser, bypassing
// the core's 60-byte SysEx buffer (our 0x12 pattern write is ~155 bytes).
// Output is packetised on the fly (usbmidi_codec.h) and every write is gated
// on usb_gate.h (configured + SOF alive) with a bounded wait for endpoint
// room, so an unplugged or parked port costs nothing and can never spin the
// loop.
//
// SysEx protocol (manufacturer 0x7D, same family as the SuperOS-303):
//   0x10 <p>              request pattern p          -> 0x11
//   0x11 <p> <xlo> <xhi> <7-bit packed pattern>      pattern data (both ways)
//   0x12 <p> <xlo> <xhi> <packed>                     set pattern p  -> 0x14
//   0x13                  request all patterns        -> 0x11 x NUM_PATTERNS
//   0x14 <code>           ack: 0 ok, 1 bad length/checksum, 2 out of range
//   0x15 <p> <step>       OUTBOUND ONLY: sequencer step position, on change.
//                         There is deliberately NO inbound 0x15. RANDOMIZE used
//                         to live here and moved to 0x2E (claim-049/claim-057):
//                         this machine emits 0x15 every step, so an echoed
//                         playhead report arrives at its own input. On the 101
//                         that was an ACK STORM (one 0x14 per step), not data
//                         loss: the old handler needed len >= 6 and SendStep
//                         emits 3. Moved anyway -- differing payload lengths are
//                         a coincidence, not a protocol guarantee.
//   0x2E <p><lane><density><slo><shi>  RANDOMIZE (gate G4, docs/EDIT_OPCODES.md)
//                           lane 0x7F=all (time+pitch), 0=pitches only, 3=legato
//                           only; 1/2 (accent/slide) absent here -> ack 2.
//                           density 0 clears/127 fills; seed=slo|(shi<<7),
//                           0=entropy else deterministic                   -> 0x14
//                           NOT 0x38: that is CMD_FRAM_PROBE on the 606.
//   0x16 <p><lane><step><value>  SET STEP (gate G4): lane 0 note (value=MIDI note,
//                           0=rest), lane 1 accent (0/1, not on a rest), lane 3
//                           gate (0 normal/1 legato); lane 2 absent       -> 0x14
//                           Accent also travels as a packed tail on 0x11/0x12
//                           after the ratchet tail (1 bit per step, bit i&7 of
//                           byte i>>3; absent = all off) and in the flash record.
//   0x18 <p><length>      SET LENGTH (gate G4), 1..127 over SysEx            -> 0x14
//                           NOT 1..MAX_STEPS: MAX_STEPS is 128 and setLength
//                           accepts it, but this field is ONE 7-bit data byte,
//                           so 128 sets the status bit and truncates the
//                           message (claim-061). The top step of the longest
//                           pattern is panel-only. Same limit on 0x16's step
//                           index. A test that writes 128 is sending a
//                           malformed message, not exercising a boundary.
//   0x1B <p><step><val>   SET RATCHET (0 single, 1 = 2x, 2 = 3x): the step's
//                           note retriggers evenly inside its clock period.
//                           Same opcode number as the 303's ratchet. The lane
//                           travels as a packed tail on 0x11/0x12 (2 bits per
//                           step after the step bytes; absent = all off) and
//                           in the flash record the same way.            -> 0x14
//   0x1D <p>              select pattern p (as PLAY + key on the panel)
//   0x1E <x>              transport: SuperOS x=1 play / 0 stop; the emulator
//                         taps its PLAY button (toggle) for any x
//   0x20                  request config              -> 0x21
//                           0x21: <fw><chan><sync><div><pat><storeOk>
//                           <lfoSync><lfoDiv><version ascii...>. 0x22 takes
//                           <chan><sync><div>[<lfoSync><lfoDiv>]: with sync
//                           on, lfoSync ON pulses CLOCK RESET on lfoDiv
//                           boundaries (LFO locked to the clock at its own
//                           musical rate); OFF leaves the analog LFO
//                           free-running for modulation while the sequencer
//                           still steps from the divided clock. lfoDiv takes
//                           0..9 (the shared division table), 10 = 1/2,
//                           11 = 1/1, or 127 = AUTO: the division follows the
//                           LFO RATE slider (measured free-run period vs the
//                           incoming tick spacing, midiclock.h). SuperOS
//                           only; the 80C49 mode never hard-synced its LFO.
//                           Sync ON is STRICT transport slaving: 0xFA starts
//                           the sequencer (from idle; an active arp is left
//                           alone), 0xFC stops it, and only the divided
//                           clock ever steps it. A synced machine with no
//                           clock on the wire stays silent; a local PLAY
//                           press arms it for the moment the clock speaks.
//                           In 80C49 mode Start/Stop tap the ROM's PLAY
//                           button, keyed on the function-LED readback.
//   0x21 <fw> <chan> <sync> <div> <pat> <version ascii...>   fw: 0 SuperOS
//   0x22 <chan> <sync> <div>  set config (chan 0..15, sync 0/1, div index)
//   (CC cutoffCc, default 74, moves the filter; CC cutoffCc+32 = 106 is its
//    14-bit LSB; see cvdac.h)
//   0x23                  request CV DAC config        -> 0x24
//   0x24 <12 cfg bytes> <codeA lo7 hi5> <codeB lo7 hi5>   (cvdac.h Pack order:
//                         center lo7/hi5, cutoff CC, flags, accent mode,
//                         VCF amount, VCA amount, decay/10ms, vel threshold,
//                         res depth, build-up, board variant; then the live
//                         DAC codes)
//   0x25 <12 cfg bytes>   set CV DAC config            -> 0x24
//   0x2F                  switch to the 80C49 emulator (reboots)
//   0x3B                  bench probe                 -> 0x3C (sysex.h: reset
//                         cause, USB regs, boot count, previous boot stage)
//   0x3D <s>              bench: park the USB engine for s seconds
//   0x3E <mode> <s>       bench: rail-load experiment for s seconds (boot.cpp):
//                         bit0 core /2 (8 MHz), bit1 USB engine + PLL off,
//                         bit2 idle-sleep every pass, bit3 core /1 (16 MHz,
//                         wins over bit0). mode 0 keeps the app's own 4 MHz,
//                         so it is the control rather than a no-op. The last
//                         second's rail samples land in the 0x3C parked slot
//                         as a raw ADC set (sum/min/max), and 0x3C reports the
//                         prescale that was APPLIED, not the one requested.
//   0x40                  emulator status request     -> 0x41 (emulator only)
//   0x41 <1> <version ascii...>
//   0x4D                  switch to SuperOS (reboots)
//   0x4A                  enter the SysEx bootloader
// A SuperOS unit ignores 0x40 and an emulator unit ignores 0x20, so an editor
// sends both and whichever firmware runs answers.
#pragma once
#include <Arduino.h>

#if defined(__AVR__) && defined(SUPEROS_USB_MIDI)
#define MIDI_HAS_USB 1
#include "usb_midi/usb_private.h"   // core endpoint numbers + usb_configuration
#include "usb_gate.h"
#include "usbmidi_codec.h"
#else
#define MIDI_HAS_USB 0
#endif

namespace midi {

static const uint8_t RX_CAP  = 64;    // power of two
static const uint8_t TX_CAP  = 128;   // power of two
static const uint8_t SX_MAX  = 208;   // largest inbound SysEx body (0x12 with the ratchet and accent tails = 207)

// Port bitmask (Send / reply routing).
enum : uint8_t { PORT_DIN = 0x01, PORT_USB = 0x02, PORT_ALL = 0x03 };

struct Handlers {
  void (*noteOn )(uint8_t note, uint8_t vel) = nullptr;   // vel > 0
  void (*noteOff)(uint8_t note)              = nullptr;
  void (*cc     )(uint8_t num, uint8_t val)  = nullptr;
  void (*program)(uint8_t num)               = nullptr;
  void (*sysex  )(const uint8_t* body, uint8_t len) = nullptr; // body[0] = cmd
  void (*realtime)(uint8_t status)           = nullptr;
};

// One byte-stream parser per input port.
struct Parser {
  uint8_t  status = 0;              // running status (channel messages)
  uint8_t  data[2]; uint8_t nData = 0, need = 0;
  bool     inSx = false; bool sxDrop = false;
  uint8_t  sx[SX_MAX]; uint8_t sxLen = 0;
};

struct State {
  Handlers h;
  uint8_t  channel = 0;             // 0..15 (channel 1..16), receive + send
  // DIN RX ring (ISR producer)
  volatile uint8_t rxBuf[RX_CAP];
  volatile uint8_t rxW = 0, rxR = 0;
  // DIN TX ring (ISR consumer)
  volatile uint8_t txBuf[TX_CAP];
  volatile uint8_t txW = 0, txR = 0;
  Parser   din;
#if MIDI_HAS_USB
  Parser   usb;
  usbmidi::Encoder utx;             // USB-MIDI TX packetiser
#endif
  uint8_t  hostPorts = 0;           // ports an editor has spoken SysEx on (sticky)
  bool     thru = false;            // soft thru: mirror channel messages + realtime
  uint8_t  clkFrom = PORT_ALL;      // ports whose realtime (clock, start, stop) drives the machine
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

// --- USB HAL (Teensy usb_midi core, raw event packets) ---------------------
// The core (cores/usb_midi/usb.c) does enumeration, SOF flushing of the IN
// endpoint and the endpoint config; we talk to the two bulk endpoints
// directly (the core's own accessors are private and its read() path caps
// SysEx at 60 bytes). Register sequences mirror usb_api.cpp.
#if MIDI_HAS_USB
inline bool UsbUp() { return usb_configuration && usb_sof_alive(); }

// Queue one 4-byte event on the IN endpoint. The two 64-byte banks take 32
// events; a pattern reply is ~52, so wait (interrupts on) for the host to
// drain a bank, but never longer than ~3 ms and never once the bus has died
// (SOF stopped) or the configuration dropped: then the event is dropped and
// the caller moves on. Bounded by construction; the 303's boot-loop came from
// a wait keyed only on the frame counter. The core's SOF ISR sends whatever
// sits in the bank each 1 ms frame; a bank we fill is handed over at once.
inline bool UsbSendEvent(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
  const uint32_t t0 = millis();
  for (;;) {
    uint8_t sreg = SREG;
    cli();
    if (!usb_configuration) { SREG = sreg; return false; }
    UENUM = MIDI_TX_ENDPOINT;
    if (UEINTX & (1 << RWAL)) {
      UEDATX = b0; UEDATX = b1; UEDATX = b2; UEDATX = b3;
      if (!(UEINTX & (1 << RWAL))) UEINTX = 0x3A;   // bank full: hand it to the host
      SREG = sreg;
      return true;
    }
    SREG = sreg;
    if ((uint32_t)(millis() - t0) > 3 || !usb_sof_alive()) return false;
  }
}

// Fetch one 4-byte event from the OUT endpoint; false when none is pending.
inline bool UsbReadEvent(uint8_t* pkt) {
  bool got = false;
  uint8_t sreg = SREG;
  cli();
  if (usb_configuration) {
    UENUM = MIDI_RX_ENDPOINT;
    for (;;) {
      uint8_t c = UEINTX;
      if (c & (1 << RWAL)) {
        pkt[0] = UEDATX; pkt[1] = UEDATX; pkt[2] = UEDATX; pkt[3] = UEDATX;
        if (!(UEINTX & (1 << RWAL))) UEINTX = 0x6B;   // bank drained: release it
        got = true;
        break;
      }
      if (c & (1 << RXOUTI)) { UEINTX = 0x6B; continue; }   // empty bank: ack, look again
      break;
    }
  }
  SREG = sreg;
  return got;
}

// Feed one output byte to the packetiser; sends a 4-byte event when complete.
inline void UsbWrite(uint8_t b) {
  State& s = S();
  if (!UsbUp()) { s.utx.reset(); return; }             // port down: drop, resync
  uint8_t ev[4];
  if (usbmidi::Encode(s.utx, b, ev)) UsbSendEvent(ev[0], ev[1], ev[2], ev[3]);
}
#endif

// --- output ----------------------------------------------------------------
inline void SendTo(uint8_t ports, uint8_t b) {
#if MIDI_HAS_USB
  if (ports & PORT_USB) UsbWrite(b);
#endif
  if (ports & PORT_DIN) HalWrite(b);
}
// Ports SysEx goes to: wherever an editor has spoken to us, else everywhere.
inline uint8_t HostPorts() { uint8_t p = S().hostPorts; return p ? p : PORT_ALL; }
inline void Send(uint8_t b) { SendTo(PORT_ALL, b); }
inline void NoteOn (uint8_t note, uint8_t vel = 100) {
  Send((uint8_t)(0x90 | S().channel)); Send(note & 0x7F); Send(vel & 0x7F);
}
inline void NoteOff(uint8_t note) {
  Send((uint8_t)(0x80 | S().channel)); Send(note & 0x7F); Send(0);
}
// SysEx: F0 7D body... F7 (body bytes must be < 0x80), to the editor port(s).
inline void SendSysex(const uint8_t* body, uint8_t len) {
  const uint8_t p = HostPorts();
  SendTo(p, 0xF0); SendTo(p, 0x7D);
  for (uint8_t i = 0; i < len; ++i) SendTo(p, body[i] & 0x7F);
  SendTo(p, 0xF7);
}
// Non-blocking variant for telemetry: dropped if the DIN ring (when DIN is a
// destination) cannot take it whole. USB writes never block.
inline bool TrySendSysex(const uint8_t* body, uint8_t len) {
  if ((HostPorts() & PORT_DIN) && TxFree() < (uint8_t)(len + 3)) return false;
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
inline bool HostSeen() { return S().hostPorts != 0; }
// Soft thru: every channel message and realtime byte received is re-sent
// explicit-status to the other side (DIN in -> DIN + USB out, USB in -> DIN
// out, never back into the port it came from). SysEx is not forwarded: F0 7D
// is this machine's editor protocol. The caller mutes the machine's own note
// echo while this is on (main.cpp MidiOut, emu101.cpp), or OUT carries both.
inline bool Thru() { return S().thru; }
inline void SetThru(bool on) { S().thru = on; }
// Clock source: 0 = DIN and USB (as before), 1 = DIN only, 2 = USB only.
// Realtime from the other port is not acted on (still forwarded by thru), so
// a host that clocks both jacks does not step the machine twice per tick.
enum : uint8_t { CLK_ANY = 0, CLK_DIN = 1, CLK_USB = 2 };
inline uint8_t ClockFrom() { return S().clkFrom == PORT_DIN ? CLK_DIN : S().clkFrom == PORT_USB ? CLK_USB : CLK_ANY; }
inline void SetClockFrom(uint8_t v) {
  if (v <= CLK_USB) S().clkFrom = v == CLK_DIN ? PORT_DIN : v == CLK_USB ? PORT_USB : PORT_ALL;
}
// A forwarded message is dropped whole, never blocked on, when the DIN ring
// cannot take it: HalWrite spins on a full ring and this runs mid-parse.
inline void ThruSend(uint8_t port, const uint8_t* msg, uint8_t n) {
  uint8_t out = (port == PORT_USB) ? PORT_DIN : PORT_ALL;
  if ((out & PORT_DIN) && TxFree() < n) out &= (uint8_t)~PORT_DIN;
  for (uint8_t i = 0; i < n; ++i) SendTo(out, msg[i]);
}
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

inline void Feed(Parser& p, uint8_t port, uint8_t b) {
  State& s = S();
  if (b >= 0xF8) {                                                // realtime: transparent
    if (s.thru) ThruSend(port, &b, 1);
    if (s.h.realtime && (port & s.clkFrom)) s.h.realtime(b);
    return;
  }
  if (b == 0xF0) { p.inSx = true; p.sxDrop = false; p.sxLen = 0; p.status = 0; return; }
  if (b == 0xF7) {
    if (p.inSx) {
      p.inSx = false;
      if (!p.sxDrop && p.sxLen >= 2 && p.sx[0] == 0x7D) {
        s.hostPorts |= port;
        if (s.h.sysex) s.h.sysex(p.sx + 1, (uint8_t)(p.sxLen - 1));
      }
    }
    return;
  }
  if (b & 0x80) {                       // status byte
    p.inSx = false;
    if (b >= 0xF0) { p.status = 0; p.need = 0; return; }   // other system common: ignore
    p.status = b; p.nData = 0;
    uint8_t hi = b & 0xF0;
    p.need = (hi == 0xC0 || hi == 0xD0) ? 1 : 2;
    return;
  }
  // data byte
  if (p.inSx) {
    if (p.sxLen < SX_MAX) p.sx[p.sxLen++] = b; else p.sxDrop = true;
    return;
  }
  if (!p.status) return;
  p.data[p.nData++] = b;
  if (p.nData >= p.need) {
    if (s.thru) {
      uint8_t m[3] = {p.status, p.data[0], p.data[1]};
      ThruSend(port, m, p.need == 2 ? 3 : 2);
    }
    Dispatch(p.status, p.data[0], p.need == 2 ? p.data[1] : 0);
    p.nData = 0;                        // running status: keep p.status
  }
}
inline void Poll() {
  State& s = S();
  int16_t b;
  uint8_t budget = RX_CAP;               // bound the work per pass
  while (budget-- && (b = HalRead()) >= 0) Feed(s.din, PORT_DIN, (uint8_t)b);
#if MIDI_HAS_USB
  // Drain up to a full endpoint bank (2 x 64 B = 32 packets) plus slack per
  // pass; a 0x12 pattern write is ~52 packets and finishes on the next pass.
  uint8_t pkt[4];
  for (uint8_t n = 0; n < 48 && UsbReadEvent(pkt); ++n) {
    uint8_t len = usbmidi::CinLen(pkt[0]);
    for (uint8_t i = 0; i < len; ++i) Feed(s.usb, PORT_USB, pkt[1 + i]);
  }
#endif
}

} // namespace midi
