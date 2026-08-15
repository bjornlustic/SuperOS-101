// SuperOS-101: DAC path diagnostic firmware (env:diag)
//
// Isolates the DAC -> 4052 -> sample/hold path from every other layer:
// no engine, no emulator, no menu, no MIDI input. Every scan works, so
// DB0-5 are proven good at the socket; the prime suspects for the
// constant-pitch fault are DB6/DB7 (4052 route select, socket pins 18/19,
// AVR PC6/PC7, TQFP pins 41/42). This build tests exactly those two pins
// without a multimeter.
//
// Run mode (default at power-on):
//   * No keys pressed:  CV OUT staircase, 0..63 repeating, 4 steps/sec.
//                       Working path = endless rising pitch sweep.
//                       Broken path  = one constant note.
//   * Any key pressed:  CV = 0x0C + key index (lowest key = lowest pitch).
//   * RANGE switch:     read every pass, written to the VCO S/H (route 01).
//   * Gate is held ON so the current CV is always audible.
//   * KT LED blinking fast (4 Hz): the boot self-test read back a stuck
//     DB6 or DB7 pad (short to a rail or a neighbour on the CPU board).
//
// Boot self-test (runs once in setup, streamed over DIN MIDI OUT every 2 s):
//   For each BUS pin: drive low / drive high and read the pad back
//   (a stuck readback = short at the pad; an open joint canNOT be seen
//   this way), then release the pin and measure pull-up rise time and
//   no-pull decay as tight-loop counts. A pad with nothing attached
//   rises near-instantly; a pin wired through to the mainboard carries
//   visible capacitance and load. Compare DB6/DB7 against DB0-5.
//   Full port readbacks are reported so a bridge between two bus pins can
//   be told apart from a short to a rail (a bridged pair still goes low
//   when the WHOLE port is driven low; a rail short does not).
//   SysEx, one data byte per nibble pair (hi nibble first):
//     F0 7D 44  bootPinc bootMcusr allLow allHigh rdLow[0..7] rdHigh[0..7]
//               rise[0..7] fall[0..7] floatRise[0..7]  F7
//   44 payload bytes; count 255 = timed out. Decode with tools/recvsyx.
//   bootPinc is PINC sampled before anything is driven: after a LONG (60 s)
//   power-off, a floating net reads LOW there, an externally-held net HIGH.
//
// Probe mode (press HOLD to toggle in and out; HOLD LED blinks at 1 Hz):
//   DB6 and DB7 output a square wave: LOW 500 ms / HIGH 500 ms, DB0-5
//   high. The HOLD LED mirrors it (lit on the LOW half).
//   a) With a DMM or scope: probe socket pins 18 (DB6) and 19 (DB7)
//      against pin 20 (GND): each must show a 0V/5V square in step with
//      the HOLD LED. Stuck = that line never reaches the socket.
//   b) With NO instrument, just a jumper wire: hold a wire from socket
//      pin 18 (or 19) to any P1 row, socket pins 27-34. The AVR watches
//      the rows (proven good: the keyboard scan uses them) for its own
//      square; when a row follows it for two consecutive half-cycles
//      (hold the wire steady for ~2 s), the GATE opens (the synth
//      drones), the KT LED lights, and SysEx F0 7D 45 <mask> F7 is sent.
//      Square seen  -> the route line reaches the socket: the CPU board
//                      is fine, the fault is on the mainboard side
//                      (4052 / IC10 area or its S/H switches).
//      Nothing seen -> the line dies between the AVR pad and the socket
//                      pin: reflow TQFP pin 41/42 and header pin 18/19.
//
// Deliberately self-contained: own port init and own DAC write with a very
// long (1 ms) acquire, so a too-short settle can be ruled out too.
#include <Arduino.h>
#include <string.h>

#ifdef USB_DISABLED
// The Teensy core calls usb_init() unconditionally; with USB_DISABLED it
// compiles no USB code, so provide the empty stub (same as boot.cpp).
extern "C" void usb_init(void) {}
#endif

// Port map (pins.h values, copied so this file has no other dependencies)
#define BUS_PORT PORTC
#define BUS_DDR  DDRC
#define BUS_PIN  PINC
#define P1_PIN   PINB
#define P1_DDR   DDRB
#define P1_PORT  PORTB
#define P2_PORT  PORTF
#define P2_DDR   DDRF

static const uint8_t P2_PORTA_OFF  = 3;
static const uint8_t P2_LED_HOLD   = 5;   // lit on 0
static const uint8_t P2_LED_KTRANS = 6;   // lit on 1 (opposite polarity, hw.h)
static const uint8_t P2_GATE       = 7;   // 0 = gate ON
static const uint8_t BTN_PLAY_BIT  = 1;   // P1 bits at the button column
static const uint8_t BTN_HOLD_BIT  = 5;

// Very generous settle/acquire so timing cannot be the reason this fails.
static const uint16_t DIAG_LADDER_US  = 20;
static const uint16_t DIAG_ACQUIRE_US = 1000;

// --- boot self-test results (latched, streamed every 2 s) -------------------
static uint8_t bootMcusr;   // reset cause (bootloader parks MCUSR in GPIOR1):
                            // bit0 PORF power-on, bit1 EXTRF external/ISP,
                            // bit2 BORF brown-out, bit3 WDRF watchdog.
                            // BORF after touching the bus = the contention
                            // current dipped the 5V rail: hard external source.
static uint8_t bootPinc;    // raw PINC sampled FIRST thing in setup, pins still
                            // tri-state from reset. After a long power-off, a
                            // floating (undriven) net has bled its charge and
                            // reads LOW here; a net held high externally reads
                            // HIGH no matter how long the power was off.
static uint8_t stAllLow;    // PINC with the whole port driven LOW  (high bit = tied to +5V/ext net)
static uint8_t stAllHigh;   // PINC with the whole port driven HIGH (low bit  = tied to GND/ext net)
static uint8_t stRdLow[8];  // full PINC readback with ONLY pin p driven low
static uint8_t stRdHigh[8]; // full PINC readback with ONLY pin p driven high
static uint8_t stFailLow;   // derived: bit set = pin driven LOW read back HIGH
static uint8_t stFailHigh;  // derived: bit set = pin driven HIGH read back LOW
static uint8_t stRise[8];   // pull-up rise time, loop counts (255 = timeout)
static uint8_t stFall[8];   // released decay,   loop counts (255 = no DC load)
static uint8_t stFloatRise[8]; // drive low, release with NO pull-up: only an
                               // EXTERNAL source can lift it (255 = none does)

// --- mode / detector state --------------------------------------------------
static bool     probe;           // false = run mode, true = probe mode
static bool     holdPrev, playPrev;
static uint32_t holdT, lastBtn, lastReport, lastSample;
static bool     sqPrev;
static uint8_t  mismatch[8];     // per-row samples that missed the driven level
static uint16_t sampleTotal;
static uint8_t  lastHalfMatched; // rows that tracked the whole previous half
static uint8_t  detected;        // rows that tracked two consecutive halves
static uint8_t  detSent;

// --- DIN MIDI OUT (UART1 TX, PD3 via the panel breakout) --------------------
static void midiInit() {
  UBRR1H = 0;
  UBRR1L = (uint8_t)(F_CPU / 16 / 31250 - 1);        // 31250 baud
  UCSR1A = 0;
  UCSR1B = (uint8_t)(1 << TXEN1);
  UCSR1C = (uint8_t)((1 << UCSZ11) | (1 << UCSZ10)); // 8N1
}

static void midiTx(uint8_t b) {
  while (!(UCSR1A & (1u << UDRE1))) {}
  UDR1 = b;
}

static void midiNibbles(uint8_t b) { midiTx(b >> 4); midiTx(b & 0x0F); }

static void midiReport() {
  midiTx(0xF0); midiTx(0x7D); midiTx(0x44);
  midiNibbles(bootPinc);
  midiNibbles(bootMcusr);
  midiNibbles(stAllLow);
  midiNibbles(stAllHigh);
  for (uint8_t i = 0; i < 8; ++i) midiNibbles(stRdLow[i]);
  for (uint8_t i = 0; i < 8; ++i) midiNibbles(stRdHigh[i]);
  for (uint8_t i = 0; i < 8; ++i) midiNibbles(stRise[i]);
  for (uint8_t i = 0; i < 8; ++i) midiNibbles(stFall[i]);
  for (uint8_t i = 0; i < 8; ++i) midiNibbles(stFloatRise[i]);
  midiTx(0xF7);
}

static void midiEvent(uint8_t mask) {
  midiTx(0xF0); midiTx(0x7D); midiTx(0x45);
  midiNibbles(mask);
  midiTx(0xF7);
}

// --- DAC / scan (unchanged from the original diag) --------------------------
static void dac(uint8_t value, uint8_t route) {
  uint8_t data = (uint8_t)~value;                    // DB0-5 active-low
  BUS_PORT = (uint8_t)(data | 0xC0);                 // route 11: present only
  delayMicroseconds(DIAG_LADDER_US);
  BUS_PORT = (uint8_t)((data & 0x3F) | (route << 6)); // connect the S/H
  delayMicroseconds(DIAG_ACQUIRE_US);
  BUS_PORT = 0xFF;                                   // back to disconnected
}

static uint8_t scan(uint8_t col) {                   // returns 1 = closed
  BUS_PORT = (uint8_t)~(1u << col);                  // DB6/7 stay high
  delayMicroseconds(3);
  uint8_t v = (uint8_t)~P1_PIN;
  BUS_PORT = 0xFF;
  return v;
}

// --- boot electrical self-test ----------------------------------------------
// Transient route connects during the patterns are harmless: run mode
// rewrites both S/H CVs continuously afterwards.
static void selfTest() {
  cli();
  // Whole-port patterns: a pin that stays high with the entire port driven
  // low is tied to +5V or an external net; a pin that fails only the
  // single-pin pattern below was fighting another bus pin's driver (bridge).
  BUS_PORT = 0x00; delayMicroseconds(5); stAllLow  = BUS_PIN;
  BUS_PORT = 0xFF; delayMicroseconds(5); stAllHigh = BUS_PIN;
  for (uint8_t p = 0; p < 8; ++p) {
    uint8_t m = (uint8_t)(1u << p);
    BUS_PORT = (uint8_t)~m;                          // this pin low, rest high
    delayMicroseconds(5);
    stRdLow[p] = BUS_PIN;
    if (stRdLow[p] & m) stFailLow |= m;
    BUS_PORT = m;                                    // this pin high, rest low
    delayMicroseconds(5);
    stRdHigh[p] = BUS_PIN;
    if (!(stRdHigh[p] & m)) stFailHigh |= m;
    BUS_PORT = 0xFF;
  }
  for (uint8_t p = 0; p < 8; ++p) {
    uint8_t m = (uint8_t)(1u << p);
    // pull-up rise: discharge the net, release to input with pull-up, count.
    BUS_PORT = (uint8_t)~m;
    delayMicroseconds(10);
    BUS_DDR  = (uint8_t)~m;
    BUS_PORT = 0xFF;                                 // pull-up on
    uint8_t c = 0;
    while (c < 255 && !(BUS_PIN & m)) ++c;
    stRise[p] = c;
    BUS_DDR = 0xFF; BUS_PORT = 0xFF;
    delayMicroseconds(10);
    // released decay: charge the net, release with the pull-up OFF, count
    // until the external DC load (if any) drags it below Vil.
    BUS_DDR  = (uint8_t)~m;
    BUS_PORT = (uint8_t)(0xFF ^ m);                  // pull-up off on this pin
    c = 0;
    while (c < 255 && (BUS_PIN & m)) ++c;
    stFall[p] = c;
    BUS_DDR = 0xFF; BUS_PORT = 0xFF;
    delayMicroseconds(10);
    // float rise: drive low, release with NO pull-up. Only an external
    // source (mainboard pull-up / bias) can lift it back; measures how
    // strong the external pull on the net is (255 = no external pull).
    BUS_PORT = (uint8_t)~m;
    delayMicroseconds(10);
    BUS_DDR  = (uint8_t)~m;                          // input, PORT bit 0 = no pull
    c = 0;
    while (c < 255 && !(BUS_PIN & m)) ++c;
    stFloatRise[p] = c;
    BUS_DDR = 0xFF; BUS_PORT = 0xFF;
  }
  sei();
}

// --- driver duel (PLAY button; needs a jumper socket pin 18 <-> pin 19) ------
// DB7 is stuck high even with the whole port low. Two remaining explanations:
// the net is held at a rail by an external source, or PC7's own low-side
// driver is dead and the readback sees the net's held charge. With DB6's
// proven-good driver jumpered onto the DB7 net, one of them must lose:
//   A: float PC7 (input, no pull), drive PC6 LOW 3 us, read both pads.
//   B: the mirror image (float PC6, drive PC7 LOW).
// Verdict from A:  PC7 reads LOW  -> the net CAN be pulled down: PC7's output
//                                    driver is dead (fault is the AVR pin).
//                                    KT LED solid for 3 s.
//                  PC7 HIGH and PC6 HIGH -> even the good driver loses: the
//                                    net is held hard at 5V externally.
//                                    KT LED fast-blinks for 3 s.
//                  PC7 HIGH, PC6 LOW -> the jumper is not making contact
//                                    (or a pad-to-socket open): no LED,
//                                    press the wire harder and retry.
// Full readbacks go out as SysEx F0 7D 46 duelA duelB F7 (nibble pairs).
static uint8_t duelA, duelB;

static void duelTest(uint32_t now) {
  cli();
  BUS_DDR = 0xFF; BUS_PORT = 0xFF;
  delayMicroseconds(5);
  BUS_DDR  = 0x7F;                                   // A: PC7 floats,
  BUS_PORT = 0x3F;                                   //    PC6 drives low
  delayMicroseconds(3);
  duelA = BUS_PIN;
  BUS_DDR = 0xFF; BUS_PORT = 0xFF;
  delayMicroseconds(5);
  BUS_DDR  = 0xBF;                                   // B: PC6 floats,
  BUS_PORT = 0x3F;                                   //    PC7 drives low
  delayMicroseconds(3);
  duelB = BUS_PIN;
  BUS_DDR = 0xFF; BUS_PORT = 0xFF;
  sei();

  midiTx(0xF0); midiTx(0x7D); midiTx(0x46);
  midiNibbles(duelA);
  midiNibbles(duelB);
  midiTx(0xF7);

  // blocking 3 s LED verdict, then back to the previous mode
  bool pc7Low  = !(duelA & 0x80);
  bool pc6High =  (duelA & 0x40);
  uint32_t until = now + 3000;
  while ((uint32_t)(millis() - until) & 0x80000000UL) {   // millis < until
    uint8_t p2 = (uint8_t)((1u << P2_PORTA_OFF) | (1u << P2_GATE) | (1u << P2_LED_HOLD));
    if (pc7Low)                                     // driver dead: solid
      p2 |= (uint8_t)(1u << P2_LED_KTRANS);
    else if (pc6High && ((millis() / 125) & 1))     // held at rail: fast blink
      p2 |= (uint8_t)(1u << P2_LED_KTRANS);
    P2_PORT = p2;
  }
}

// --- probe mode --------------------------------------------------------------
static void resetDetector() {
  memset(mismatch, 0, sizeof mismatch);
  sampleTotal = 0;
  lastHalfMatched = 0;
  detected = 0;
  if (detSent) { midiEvent(0); detSent = 0; }
}

static void probeLoop(uint32_t now) {
  bool hi = (now / 500) & 1;

  if (hi != sqPrev) {                                // half-cycle boundary
    sqPrev = hi;
    uint8_t matched = 0;
    if (sampleTotal >= 100) {                        // ignore partial halves
      for (uint8_t r = 0; r < 8; ++r)
        if (mismatch[r] <= (uint8_t)(sampleTotal >> 3))  // <= 12.5% bad samples
          matched |= (uint8_t)(1u << r);
    }
    // consecutive halves have opposite levels, so a stuck row can't pass both
    detected = (uint8_t)(matched & lastHalfMatched);
    lastHalfMatched = matched;
    memset(mismatch, 0, sizeof mismatch);
    sampleTotal = 0;
    if (detected != detSent) { midiEvent(detected); detSent = detected; }
  }

  BUS_PORT = hi ? 0xFF : 0x3F;                       // square on DB6+DB7 only

  if ((uint32_t)(now - lastSample) >= 2) {           // sample rows every 2 ms
    lastSample = now;
    delayMicroseconds(5);
    uint8_t rows = P1_PIN;
    uint8_t bad  = hi ? (uint8_t)~rows : rows;       // bits not at the driven level
    for (uint8_t r = 0; r < 8; ++r)
      if (((bad >> r) & 1) && mismatch[r] < 255) ++mismatch[r];
    if (sampleTotal < 60000) ++sampleTotal;
  }

  uint8_t p2 = (uint8_t)(1u << P2_PORTA_OFF);
  if (detected) p2 |= (uint8_t)(1u << P2_LED_KTRANS); // loopback seen: KT LED on
  else          p2 |= (uint8_t)(1u << P2_GATE);       // and GATE opens (drone)
  if (hi)       p2 |= (uint8_t)(1u << P2_LED_HOLD);   // HOLD LED lit on LOW half
  P2_PORT = p2;
}

// --- run mode -----------------------------------------------------------------
static void runLoop(uint32_t now) {
  // RANGE -> VCO S/H (route 01), same decode as controls.h
  BUS_PORT = (uint8_t)~(1u << 4);                    // DB4 low = switches
  delayMicroseconds(3);
  uint8_t rawSw = P1_PIN;                            // raw: 0 = active
  BUS_PORT = 0xFF;
  uint8_t range = 0x30;
  if (!(rawSw & (1u << 0))) range -= 12;
  if (!(rawSw & (1u << 1))) range -= 24;
  dac(range, 1);                                     // route 01 -> VCO

  // CV: pressed key, else free-running staircase
  uint32_t keys = 0;
  for (uint8_t b = 0; b < 4; ++b)
    keys |= (uint32_t)scan(b) << (b * 8);

  uint8_t cv;
  if (keys) {
    uint8_t i = 0;
    while (!(keys & 1)) { keys >>= 1; ++i; }
    cv = (uint8_t)(0x0C + i);
  } else {
    cv = (uint8_t)((now / 250) & 0x3F);              // 4 steps/sec, 0..63
  }
  dac(cv, 0);                                        // route 00 -> CV out

  // gate permanently ON so the current CV is always audible; KT LED blinks
  // 4 Hz if the boot self-test saw DB6/DB7 stuck at the pad
  uint8_t p2 = (uint8_t)((1u << P2_PORTA_OFF) | (1u << P2_LED_HOLD));
  if (((stFailLow | stFailHigh) & 0xC0) && ((now / 125) & 1))
    p2 |= (uint8_t)(1u << P2_LED_KTRANS);
  P2_PORT = p2;

  if ((uint32_t)(now - lastReport) >= 2000) {
    lastReport = now;
    midiReport();
  }
}

void setup() {
  bootPinc  = PINC;  // BEFORE any port config: pins are tri-state from reset
  bootMcusr = GPIOR1;
  GPIOR1 = 0;
  BUS_DDR = 0xFF; BUS_PORT = 0xFF;
  P1_DDR  = 0x00; P1_PORT  = 0xFF;                   // internal pull-ups
  P2_DDR  = 0xFF;
  // gate ON (bit7=0), portamento off, HOLD LED off (bit5=1), KT LED off (bit6=0)
  P2_PORT = (uint8_t)((1u << P2_PORTA_OFF) | (1u << P2_LED_HOLD));
  DDRD  &= (uint8_t)~0x03;                           // T0/T1 inputs
  PORTD |= 0x03;                                     // with pull-ups

  midiInit();
  selfTest();
  midiReport();
}

void loop() {
  uint32_t now = millis();

  // Button scan only every 50 ms: in probe mode a per-pass scan would chop
  // the DB6/DB7 square into glitches (scan drives them high for ~3 us).
  if ((uint32_t)(now - lastBtn) >= 50) {
    lastBtn = now;
    uint8_t buttons = scan(5);                       // DB5 low = buttons
    bool hb = (buttons >> BTN_HOLD_BIT) & 1;
    if (hb && !holdPrev && (uint32_t)(now - holdT) > 250) {
      probe = !probe;                                // HOLD toggles probe mode
      holdT = now;
      sqPrev = (now / 500) & 1;
      resetDetector();
    }
    holdPrev = hb;
    bool pb = (buttons >> BTN_PLAY_BIT) & 1;
    if (pb && !playPrev) duelTest(now);              // PLAY runs the driver duel
    playPrev = pb;
  }

  if (probe) probeLoop(now);
  else       runLoop(now);
}
