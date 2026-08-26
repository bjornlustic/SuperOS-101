# G10 tempo-LED phase-lock: 101 implementation draft (NOT YET APPLIED)

Shared design: SuperOS-USBC/docs/G9_INVENTORY.md 6a (claim-022). One phase
accumulator mod 48 (48 half-ticks/beat), advanced +2 per MIDI 0xF8 when slaved,
aligned to 0 on Start; tempo_blink() = phase < 24 (LED on the first half of each
beat). Per-machine quirk = the LED output only.

Gated behind the 101's G3 baseline (finding 23, pending CORE's confirmation) and
lands with the shared MIDI-clock subsystem (subsystem 2), since the 101's
midiclock.h is that subsystem's base.

## The 101's LED-map quirk (measured)

The 101 has NO dedicated tempo LED. Its function LEDs are ONE-HOT (exactly one of
LOAD/PLAY/DOWN/U&D/UP via IC8 4556, pins.h P20-22), so there is no spare LED to
blink without stealing the mode indication. The two directly-driven LEDs are HOLD
(P25) and KEY-TRANSPOSE (P26). P26 is ALREADY the sync indicator (menu.h lights it
when SyncOn), and it is free during normal play, so it is the natural tempo LED:

  led_bit = P2_LED_KTRANS (P26), lit when tempo_blink() && SyncOn.

Limitation to state: when NOT slaved, the 101's tempo is the analog LFO / EXT-CLK
(no digital phase), so there is nothing to blink phase-locked; G10 only requires
it while slaved, which is exactly when the MIDI-clock phase exists. So free-running
tempo-blink is out of scope for the 101 (analog), which is a per-machine reality.

## midiclock.h -- add the shared beat-phase accumulator

```cpp
// In struct State:
uint8_t beatPhase = 0;     // 0..47, mod 48 = 48 half-ticks/beat (G10 shared phase)

// In OnRealtime(), case 0xF8 (inside the existing block, alongside the divider):
S().beatPhase = (uint8_t)((S().beatPhase + 2) % 48);   // 2 half-ticks per 24-ppqn tick
// case 0xFA (Start): S().beatPhase = 0;   // align to the downbeat
// case 0xFB (Continue): leave beatPhase as-is (resume in phase)
// (Stop 0xFC holds the phase; running=false already gates blinking off if wanted)

inline bool TempoBlink() { return S().beatPhase < 24; }   // shared verbatim
```

Note: beatPhase is the BEAT phase (mod 48 = per quarter note), distinct from the
existing `tick`/DIV_TICKS divider (which is the arp/seq division). G10 blinks on
the TEMPO (the beat), not the division. Both count the same 0xF8 stream; keep them
separate.

## main.cpp -- drive P26 from the phase while slaved

In superos_loop, where p2 is assembled (main.cpp ~284):
```cpp
// G10: KEY-TRANSPOSE LED (P26, the sync indicator) blinks on the beat while
// slaved to MIDI clock. One-hot function LEDs have no spare, so P26 is the map.
if (midiclk::SyncOn() && midiclk::Running() && midiclk::TempoBlink())
    p2 |= (uint8_t)(1u << P2_LED_KTRANS);      // P26=1 -> KTRANS LED lit
```
(P26 is lit by bit=1, per main.cpp's existing "KEY-TRANSPOSE by P26=1" comment.)

## Verification (G3-measurable, headless)

* Sim: slave the 101 (sync on), send 0xFA then 0xF8 stream, read P2 across ticks;
  assert P26 toggles with a 50% duty cycle at the beat rate, phase-aligned to 0xFA.
  Add to sim/g1_test.py as a G10 check.
* Hardware: the LED is not on MIDI/telemetry today. Either (a) add the beatPhase to
  the 0x3B probe so the host can read the phase and confirm phase-lock over the
  wire (rule 7: measurable via telemetry), or (b) patch the LED to an SSL channel
  for a G3-style capture. (a) is the headless route; recommend adding beatPhase to
  the probe in the same commit.

## Also in the same commit (G8: docs match code)
* Manual: note the KEY-TRANSPOSE LED blinks on the beat while MIDI-clock slaved.
* This lands WITH the shared MIDI-clock subsystem, so beatPhase belongs in the
  shared clock core CORE extracts from the 101's midiclock.h.
```
