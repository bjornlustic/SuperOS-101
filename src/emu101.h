// SuperOS-101 — 80C49 emulator firmware side (original mask ROM on the AVR)
#pragma once

namespace emu101 {
bool available();   // true when the mask ROM is embedded (rom101_data.h)
void setup();
void loop();
const uint8_t* ram();   // the emulated 80C49's 128 B RAM (inspection only)
// Speed telemetry (SysEx 0x3B probe), refreshed once a second:
uint16_t cyclesPerSec100(); // emulated machine cycles per second / 100 (real chip = 4000)
uint16_t passesPerSec();    // ROM program passes per second (CV-OUT S/H writes)
uint16_t t1EdgesPerSec();   // T1 (LFO / EXT CLK) falling edges per second (INT0 / sync poll)
uint16_t t1PollEdgesPerSec(); // same, as seen by the once-per-host-pass poll (diagnostic)
uint16_t t1RejectedPerSec();  // INT0 falling edges rejected as bounces (pin back high within 0.5 ms)
uint16_t runPerMille();     // share of wall time inside the interpreter, per mille
uint16_t shPerMille();      // share of wall time inside the S/H refresh, per mille
uint16_t benchUs(bool ints); // boot bench: us for 4000 emulated cycles (10 ms real chip), interrupts off/on
uint16_t benchDacUs();       // boot bench: us for 10 hw::DacWrite
uint16_t benchPollUs();      // boot bench: us for 100 midi::Poll
uint16_t benchB(uint8_t i);  // boot bench: us for 100x Poll/T1Watch/RateTick/RamPersistTick/micros/button-scan
}
