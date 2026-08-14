// SuperOS-101 emulator — Arduino/AVR register shim (host build only)
//
// Stands in for <Arduino.h> + <avr/io.h> when the firmware is compiled natively.
// The AVR port registers become sim-side hooks: writes to PORTC (the 80C49 BUS)
// go through a proxy object so the emulator can model the 4052 route latching
// and the scan matrix; PINB/PIND reads are computed from the simulated panel.
#pragma once
#include <stdint.h>

namespace sim {

// Proxy for PORTC (BUS). Every write is observed by the hardware model.
struct BusPort {
  BusPort& operator=(uint8_t v);
  operator uint8_t() const;
};
extern BusPort busPort;

uint8_t ReadP1();    // PINB — scan rows, computed from BUS column + panel state
uint8_t ReadPinD();  // PIND — T1 clock (bit 0), T0 HOLD (bit 1, active-low)

} // namespace sim

extern uint8_t sim_ddrc, sim_ddrb, sim_portb, sim_ddrf, sim_portf, sim_ddrd;

#define PORTC (sim::busPort)
#define DDRC  sim_ddrc
#define PORTB sim_portb
#define DDRB  sim_ddrb
#define PINB  (sim::ReadP1())
#define PORTF sim_portf
#define DDRF  sim_ddrf
#define DDRD  sim_ddrd
#define PIND  (sim::ReadPinD())

void delayMicroseconds(uint32_t us);
uint32_t micros();   // virtual time (advanced by delayMicroseconds)
uint32_t millis();

void setup();
void loop();
