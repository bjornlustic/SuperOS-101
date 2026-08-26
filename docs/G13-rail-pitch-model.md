# G13 target one for the 101: the pitch rail

Status: **scoped, computed, not simulated and not measured.** Everything below
that is a number is arithmetic from the design and the firmware constants, not a
SPICE result and not a bench result. It exists so the model has a target to hit
and a way to be wrong, per G13's rule that a model is validated against something
already measured before it is trusted on something new.

Owner: A101 (this machine's circuits). Method and shared model library: CORE
(claim-034).

## Why this circuit and not another

G13 says start where a simulation changes a live decision. On the 101 that is the
clock. The 101 runs at 4 MHz **solely** as a supply-rail fix for a ~5-cent pitch
shift seen at 8 MHz, and it costs the emulator 4x: G5 records 25.3 % of the 80C49's
real rate, accepted as correct precisely because the divide is bought on purpose.
The same interpreter clears real time at 16 MHz.

So if the rail can be fixed, the 101's emulator throughput goes up fourfold and its
G5 stops being an exception. That is the same shape as the 606's claim-028/030
pulldown question, on a different machine, and it is worth a model for the same
reason: the alternative is soldering and hoping.

## The mechanism, stated precisely enough to be wrong

The usual shorthand in this project is "the +5 V rail is the DAC reference, so it
is the pitch scale". True, but it hides the part that matters:

**The pitch DAC is not on the CPU board.** `design.py` is explicit that the stock
SH-101 CPU "only drives the 6-bit pitch ladder, GATE and LEDs" and that the board
takes `+5V` straight from socket pin 40 with **no regulator of its own** (only
local decoupling: C13 10 uF bulk plus 100 nF parts). The R-2R ladder lives on the
synth board and is driven by the AVR's `PORTC` (BUS DB0-7, socket pins 12-19).

So the chain is:

    AVR core current (rises with clock)
      -> droop across the supply impedance from the synth's regulator,
         through the harness and the DIP-40 socket, to the board's VCC pin
      -> AVR VCC falls
      -> PORTC V_OH falls with it (V_OH tracks VCC; the ladder is driven by
         logic levels, not by a reference pin)
      -> the ladder's full-scale, and therefore every pitch CV, scales down
      -> pitch goes flat, proportionally

The reference is the AVR's **output-high voltage**, not a reference pin anywhere.
That is why a firmware-only change (the clock divide) moves it at all, and it is
the detail a model must get right: modelling this as an ideal voltage reference
would miss it entirely, the way modelling the 606's germanium diodes as silicon
would have hidden that fault.

## The budget the model has to resolve

From the firmware constants: `MIDI_CV_OFFSET` 36 and `CV_MAX` 63, so code 0 is
note 36 and code 63 is note 99. That is 63 semitones, 5.25 octaves, at the
SH-101's 1 V/octave.

Pitch error is proportional to the *fractional* change in V_OH, so for a
fractional droop d at a CV of V volts:

    error (cents) = 1200 * d * V

At middle C (note 60, code 24, V = 2.00 V), 5 cents needs

    d = 5 / (1200 * 2.00) = 0.208 %,  i.e. about 10.4 mV on a 5 V rail

**So the model must resolve single-digit millivolts on a 5 V rail.** That is the
number to design the simulation around, and it is uncomfortably small: it means
probe placement, connector resistance and the regulator's output impedance all
matter, and a model that is only good to tens of millivolts cannot answer this
question at all.

Sanity check on plausibility, not a result: the AT90USB1286 draws roughly 25 mA
more at 16 MHz than at 4 MHz. Moving the rail 10 mV on 25 mA needs only **0.4 ohm**
of series impedance between the synth's regulator and the socket pin. Forty-year-old
harness plus a DIP socket plus a connector reaches that easily. The mechanism is not
exotic; it is ordinary, which is why it is worth simulating rather than arguing.

## The prediction that can falsify it

This is the part worth having before any model is built, because it is cheap and it
decides whether the whole story is right:

**The error must be proportional to pitch, and zero at the bottom of the range.**
A rail-scale error is multiplicative. Same 10 mV droop:

| note | code | CV | predicted error |
|---|---|---|---|
| 36 (bottom) | 0 | 0.00 V | **0 cents** |
| 60 | 24 | 2.00 V | 5 cents |
| 99 (top) | 63 | 5.25 V | 13 cents |

If a bench measurement shows the 8 MHz shift is roughly **constant** across the
keyboard, it is an offset, not a scale error, the reference story is wrong, and no
amount of decoupling will fix it. If it grows with pitch, the model is on the right
track before a single SPICE netlist exists.

This is measurable headlessly and needs no simulation and no hands: drive notes at
the bottom, middle and top over MIDI at 4 MHz and at 8 MHz and compare recorded
pitch. It needs the audio lease, so it is CORE's capture to schedule, not mine to
take.

## What the model needs, and what would be a substitute

Per G13, cite the part or say it is a stand-in:

* **AT90USB1286 supply current vs frequency**: datasheet active-mode figures.
  A stand-in until measured: the real modulation is not DC, it is switching
  current at the scan and DAC-write cadence, so the DC figure sets the offset and
  says nothing about the dynamic part.
* **Supply impedance from the synth regulator to socket pin 40**: **not known.**
  This is the dominant unknown and the model is worthless without a real value.
  It has to be measured or taken from the service manual's supply section.
* **Board decoupling as built**: C13 10 uF bulk plus 100 nF per supply pin, from
  `design.py` (`build_mcu`). This part is exact, because the board is a skidl
  design and can be read rather than guessed.
* **The R-2R ladder and any post-ladder gain on the synth board**: from the SH-101
  service manual, not yet extracted. Full-scale works out near 5.25 V against a
  ~4.9 V ladder output, so there is gain of about 1.07 after it; that should be
  read off the schematic rather than inferred from my arithmetic above.
* **V_OH vs load and vs VCC** for the AVR's port pins: datasheet curves. Modelling
  V_OH as exactly VCC would understate the effect.

## What a pass would buy

If the model reproduces a ~5-cent shift at 8 MHz from a plausible supply impedance,
then it can be asked the question that matters: **does more local bulk capacitance
hold V_OH steady enough to run 16 MHz?** A dynamic droop is what capacitance fixes;
a static one is a tuning constant and could simply be calibrated out in firmware
instead. Those two have completely different fixes, and which one this is has never
been established, only assumed.

That distinction is the most valuable thing in this document and it costs one bench
capture to settle.

## Not started

No netlist, no ngspice run. `ngspice 46` and `skidl 2.2.3` are both present and the
board is already a skidl design, so nothing needs installing. Held pending CORE's
shared method (claim-034) so the 101's model is built in the fleet's shape rather
than a private one, which is the same core-plus-quirk rule as everywhere else.
