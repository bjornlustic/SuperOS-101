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
uint16_t gateGapUs(bool max); // ROM gate off->on gap: last / max since last read (us)
uint16_t gateOnsPerSec();     // ROM gate-on events per second
uint16_t gatePhysGapUs();     // last off->on gap on the physical GATE pin (after shaping)
uint16_t portaPulsesPerSec(); // ROM PORTAMENTO OFF (P23) rising edges per second
uint16_t portaPulseLenUs();   // last PORTAMENTO OFF pulse length (us)
uint16_t portaAfterCvUs();    // last PORTAMENTO OFF rise minus last CV-OUT value change (us)
uint16_t p2TogglesPerSec(uint8_t bit); // ROM P2 bit transitions per second
uint16_t gateOnLenUs();   // last ROM gate on-length (us)
uint16_t cvToGateOffUs(); // last ROM gate drop minus last CV-OUT change (us)
uint16_t midiNoteOnsPerSec(); // incoming MIDI note-ons per second (DIN + USB)
uint16_t keyToGateUs(bool max); // G5: us from the ROM's keyboard read (step 5) to its
                              // next P2/GATE write (step 13). This is f * T_pass, the
                              // second term of the key-to-gate bound (1 + f) * T_pass.
void keyScanAbort();          // drop any in-flight key-to-gate measurement, so the
                              // pass that answers a 0x3B does not report the cost of
                              // the reply as the machine's key-scan latency. Same
                              // defect and same fix as combined_pass_skip_next().
uint16_t keysSeenPerSec();    // ROM keyboard-column P1 reads per second with a row low
uint16_t shChangesPerSec(uint8_t route); // S/H value changes per second: 0 CV-OUT, 1 VCO, 2 RANDOM
uint16_t btnsSeenPerSec();    // ROM button-column P1 reads per second with a row low
uint16_t runPerMille();     // share of wall time inside the interpreter, per mille
uint16_t shPerMille();      // share of wall time inside the S/H refresh, per mille
uint16_t benchUs(bool ints); // boot bench: us for 4000 emulated cycles (10 ms real chip), interrupts off/on
uint16_t benchDacUs();       // boot bench: us for 10 hw::DacWrite
uint16_t benchPollUs();      // boot bench: us for 100 midi::Poll
uint16_t benchB(uint8_t i);  // boot bench: us for 100x Poll/T1Watch/RateTick/RamPersistTick/micros/button-scan
}
