# G13 target one for the 101: the pitch rail

Status: **scoped. The first draft of this document was wrong and is corrected
below, before any model was built.** Nothing here is simulated. The numbers are
either arithmetic from the design and the firmware constants, or measured on this
unit on 2026-08-15 and labelled as such.

> ## Correction, same day, before anything was built on it
>
> I wrote this file predicting that the pitch error would be **proportional to
> pitch and zero at the bottom note**, and said that a roughly constant error
> across the keyboard would falsify the whole story. Then I checked it against
> rail data already measured on this unit on 2026-08-15, and **it falsifies it**,
> on three counts:
>
> | | my first draft | measured |
> |---|---|---|
> | direction | lower rail -> **flat** | lower rail -> **sharp** |
> | sensitivity | 10.4 mV of rail = 5 cents | **~57 mV of rail = 1 semitone**, so 5 cents = **2.85 mV** |
> | shape across the keyboard | proportional to pitch, zero at note 36 | a **constant** semitone offset, i.e. a transposition |
>
> The measured sensitivity is self-consistent: 4.98 V at 4 MHz down to ~4.57 V at
> 16 MHz is 410 mV, which at 57 mV/semitone is 7.2 semitones, and the symptom
> recorded at the time was "C4 played ~G4", 7 semitones. Two independent numbers
> from the same session agreeing is what makes this measurement trustworthy and
> my arithmetic not.
>
> **What was wrong with the reasoning.** I derived the mechanism from the CPU
> board's schematic alone, got a plausible ratiometric story out of it (V_OH
> scales the ladder, so error scales with CV), and did not check it against the
> bench data that already existed. It is exactly LESSONS L4: a partial explanation
> that reads well enough to stop the search. The falsifiable prediction did its
> job faster than I expected, which is the argument for writing one.
>
> The corrected mechanism is below and it is **better news**, because it splits
> one symptom into two problems with different fixes.

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

**The measured sensitivity says this ratiometric story is not the whole mechanism,
or not the right one.** It predicts the wrong sign. If the CV were simply a
fraction of V_OH, a sagging rail would give a smaller CV and a flatter note; the
unit goes *sharp*. So something between the ladder and the VCO inverts the sense,
or the expo converter's reference is itself derived from the same +5 V rail and
moves with it. A uniform semitone offset across the keyboard is the signature of a
reference or bias shift, not of a full-scale error. Which of those it is has not
been established, and it is the first thing the model must reproduce, because a
model that gets the sign wrong is not a model.

Resolving that needs the SH-101 service manual's CV path (p.7 for the supply, the
expo converter and summing on the control board), which is in `docs/` and has not
been read for this purpose.

## The two problems this actually is

The correction above splits the symptom, and this is the useful part:

| | what it is | size, measured | fix |
|---|---|---|---|
| **static droop** | a constant transposition: rail sits lower at a higher clock | 410 mV at 16 MHz (7.2 semitones), ~20 mV at 8 MHz (~0.35 semitones) | it is a **tuning constant**. Calibrate it out in firmware, or lower the average current |
| **dynamic wobble** | rail ripples with scan/DAC/USB activity, so pitch moves *while playing* | the "~5 cents at 8 MHz" the project quotes, i.e. only about **2.85 mV** of ripple | **capacitance and supply impedance**. This is the one a model can answer |

Reconciling the two 8 MHz numbers, which had looked contradictory: a 20 mV static
droop is 35 cents, far more than the "5 cents" everyone quotes. They are not the
same measurement. 35 cents is a constant transposition, which is inaudible as
wrongness because everything is transposed together and it tunes out; 5 cents is
the *variation*, which is audible as instability. The project has been quoting the
wobble and reasoning about the droop.

That distinction changes what is worth doing. **If the wobble is the only real
problem, the fix is local decoupling and 16 MHz may be reachable without touching
the synth**, which is a far cheaper answer than the 606's harness surgery. And
2.85 mV is the number a model has to resolve, which is brutal: it is well inside
the noise of a casual measurement and it means probe placement and the regulator's
output impedance dominate.

## The budget the model has to resolve

From the firmware constants: `MIDI_CV_OFFSET` 36 and `CV_MAX` 63, so code 0 is
note 36 and code 63 is note 99. That is 63 semitones, 5.25 octaves, at the
SH-101's 1 V/octave.

Pitch error is proportional to the *fractional* change in V_OH, so for a
fractional droop d at a CV of V volts:

Use the **measured** sensitivity, not my derivation:

    57 mV of rail = 1 semitone = 100 cents
    so 1 cent  = 0.57 mV
       5 cents = 2.85 mV

**The model must resolve under 3 mV on a 5 V rail.** That is the number to design
the simulation around and it is harsh: it is inside the noise of a casual
measurement, and it means probe placement, connector resistance and the
regulator's output impedance all dominate. A model good to tens of millivolts
cannot answer this question. My first draft asked for 10 mV and was 3.6x too
generous, in the direction that would have made a useless model look adequate.

Sanity check on plausibility, not a result: the AT90USB1286 draws roughly 25 mA
more at 16 MHz than at 4 MHz. The measured 410 mV droop over that step implies
about **16 ohm** of effective source impedance, which is far too high for a
harness and a socket, and points at the regulator itself running out of headroom
rather than at wiring resistance. That is consistent with what the supply is: not
a modern LDO but a discrete pass regulator, op-amp 1b driving PNP TR3 through R2
8.2k with the reference at D4/VR-1 (service manual p.7). A discrete pass stage
with a weak base drive has a soft current limit and a rising output impedance as
it approaches it, which is exactly the shape of "fine at 4 MHz, sags a little at
8, collapses at 16".

That is the model's first job: reproduce 4.98 / 4.96 / ~4.57 V at the three clock
loads from the real regulator topology. Three measured points is a genuine
validation set, which is the luxury G13 says to exploit.

## The prediction that can still falsify it

The first draft's prediction (error proportional to pitch) is already refuted by
the measured constant sensitivity, so it is retired. What replaces it is sharper,
because it discriminates between the two problems above rather than between two
mechanisms:

**Is the 8 MHz wobble dynamic or static?** Play one sustained note at 8 MHz and
compare pitch with the emulator idle against the emulator running flat out (or
USB detached against configured, which the `0x3E` bench modes already toggle).

* pitch **moves** with CPU load -> dynamic, it is a decoupling and source-impedance
  problem, a model is worth building, and 16 MHz may be reachable with local
  capacitance alone
* pitch is **stable** but simply offset -> static, it is a tuning constant, the
  honest fix is calibration or a supply change, and SPICE tells us nothing we
  cannot get from a DC load line

Nobody has run that comparison, and the whole value of a model hangs on it. It is
headless, and the firmware already has the instrumentation for it: `0x3B` reports
VCC in mV, and `0x3E <mode> <s>` runs the rail-load experiment with the parked
sample landing in the `0x3C` reply. So the rail half needs **no audio lease at
all**. Only the pitch half needs a capture, and only to confirm what the millivolts
already imply.

## What the model needs, and what would be a substitute

Per G13, cite the part or say it is a stand-in:

* **AT90USB1286 supply current vs frequency**: datasheet active-mode figures.
  A stand-in until measured: the real modulation is not DC, it is switching
  current at the scan and DAC-write cadence, so the DC figure sets the offset and
  says nothing about the dynamic part.
* **The synth's +5 V regulator**: **known, and it is the likely culprit.** Not an
  LDO: a discrete pass regulator, op-amp 1b driving PNP TR3 through R2 8.2k,
  reference D4/VR-1, service manual p.7. Its large-signal behaviour near its
  current limit is what the model has to get right, and it is the reason the
  droop is 410 mV rather than the tens of millivolts plain wiring resistance
  would give. (My first draft called this "the dominant unknown"; it was recorded
  in the project's own memory from the 2026-08-15 session all along, which is
  L7's "already there" recurring.)
* **Validation set, measured on this unit 2026-08-15**: VCC at the chip is 4.98 V
  at 4 MHz, 4.96 V at 8 MHz and 4.49-4.65 V at 16 MHz, all with USB configured.
  The model reproduces those three points before it is asked anything new.
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

If the model reproduces the three measured VCC points from the real discrete
regulator, it can be asked the question that matters: **what does it take to hold
the rail inside 3 mV of ripple at 16 MHz?** Local bulk capacitance, a series
ferrite, or powering the AVR from somewhere other than this regulator (the board
has USB VBUS present, currently sense-only) are all testable in simulation before
anyone touches an iron, which is the whole point of G13.

The prize is specific: 16 MHz is 4x the emulator throughput, which takes the 101's
G5 from 25.3 % to comfortably over real time and removes the one per-machine
exception in that gate.

## What this document got wrong, kept deliberately

The correction at the top is left in place rather than edited away, per claim-035's
method. The first draft's mechanism was derived from one schematic, was internally
plausible, predicted the wrong sign, and asked for a tolerance 3.6x looser than the
real one. Had it not carried an explicit falsifiable prediction it would have gone
into a model and the sign error would have been discovered after the work, not
before it. The prediction is the only reason this cost twenty minutes instead of a
day.

## Not started

No netlist, no ngspice run. `ngspice 46` and `skidl 2.2.3` are both present and the
board is already a skidl design, so nothing needs installing. Held pending CORE's
shared method (claim-034) so the 101's model is built in the fleet's shape rather
than a private one, which is the same core-plus-quirk rule as everywhere else.
