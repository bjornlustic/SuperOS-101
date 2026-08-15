// SuperOS-101 — 80C49 emulator host layer
//
// Runs the original SH-101 mask ROM on the MCS-48 core (mcs48.h), passing the
// emulated ports straight through to the socket: our AVR ports map 1:1 onto
// the 80C49's BUS/P1/P2/T0/T1 (pins.h), so no translation is needed — the
// ROM's own active-low values, settle delays, and scan patterns hit the
// analog board exactly as on the original CPU.
//
// Pacing: 6 MHz resonator -> 400k machine cycles/s (15 clocks/cycle). The
// interpreter is throttled to that rate; if the AVR can't keep up it runs
// flat out and the emulated firmware simply runs proportionally slower (pitch
// CVs are unaffected — only scan latency and software delay loops stretch).
#include <Arduino.h>
#include "emu101.h"
#include "mcs48.h"
#include "pins.h"
#include "hw.h"
#include "menu.h"
#include "midiclock.h"
#include "midi.h"
#include "sysex.h"

#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#define PROGMEM
#define pgm_read_byte(p) (*(const uint8_t*)(p))
#endif
#include "rom101_data.h"

static Mcs48 g_cpu;

// --- core hooks: 1:1 port pass-through ---------------------------------------
uint8_t mcs48_rom_read(uint16_t addr) { return pgm_read_byte(&ROM101[addr & 0x7FF]); }
void    mcs48_bus_write(uint8_t v)    { BUS_PORT = v; }
uint8_t mcs48_p1_read()               { return P1_PIN; }
void    mcs48_p2_write(uint8_t v)     { P2_PORT = v; }
bool    mcs48_t0() { return (PIND & (1u << T0_HOLD_BIT))  != 0; }
// With MIDI sync ON the divided MIDI clock replaces the LFO/EXT-CLK on the
// emulated T1 pin, so the stock ROM's arp/sequencer runs DAW-locked too.
bool    mcs48_t1() {
  if (midiclk::SyncOn()) return midiclk::T1Level();
  return (PIND & (1u << T1_CLOCK_BIT)) != 0;
}

namespace emu101 {

bool available() { return ROM101_PRESENT != 0; }

static void OnSysex(const uint8_t* body, uint8_t len) { sysex::HandleCommon(body, len); }
static void OnRealtime(uint8_t b) { midiclk::OnRealtime(b); }

void setup() {
  hw::Init();
  midi::Handlers h;
  h.sysex = OnSysex; h.realtime = OnRealtime;
  midi::SetHandlers(h);
  // 80C49 reset state: all port latches high (gate off, LEDs per reset).
  BUS_PORT = 0xFF;
  P2_PORT  = 0xFF;
  mcs48_reset(g_cpu);
}

// One slice per loop() pass: 200 cycles = 500 us of emulated time.
static const uint16_t SLICE_CYCLES = 200;

void loop() {
  uint16_t ran = 0;
  while (ran < SLICE_CYCLES) ran = (uint16_t)(ran + mcs48_step(g_cpu));

#ifdef SUPEROS_SIM
  // Host sim: advance virtual time instead of throttling (2.5 us/cycle).
  delayMicroseconds((uint32_t)ran * 5 / 2);
#else
  // Throttle to real time; if we're behind by more than ~10 ms, resync
  // instead of sprinting to catch up.
  static uint32_t s_dueUs = 0;
  if (s_dueUs == 0) s_dueUs = micros();
  s_dueUs += (uint32_t)ran * 5 / 2;
  uint32_t now = micros();
  if ((int32_t)(s_dueUs - now) > 0)       while ((int32_t)(s_dueUs - micros()) > 0) {}
  else if ((int32_t)(now - s_dueUs) > 10000) s_dueUs = now;
#endif

  midi::Poll();   // keep the divided clock (emulated T1, see mcs48_t1) live + SysEx

  // Config-menu combo poll every ~10 ms: scan the buttons ourselves, without
  // disturbing the BUS latch the emulated ROM left behind (route 11 scans
  // can't corrupt a S/H; restoring the latch keeps the ROM's own scan state).
  static uint32_t s_lastPollMs = 0;
  uint32_t ms = millis();
  if (ms - s_lastPollMs >= 10) {
    s_lastPollMs = ms;
    uint8_t savedBus = BUS_PORT;
    uint8_t buttons = hw::ScanRead(COL_BUTTONS);
    BUS_PORT = savedBus;
    menu::NotifyButtons(buttons, ms);
  }
}

} // namespace emu101
