# claim-038 G13: the 63:39:0 sign test, run

> **CORRECTION, same day, by the author.** The first version of this document
> said 63:39:0 was **refuted as a ratio** at 5.4 sigma. **That was wrong and it
> is withdrawn.** Two defects in my own analysis, both found by running the
> control I had named in my handoff as the weakest link:
>
> 1. **It compared two notes measured through DIFFERENT harmonics** (note 36 via
>    n=64, note 60 via n=16). Those two harmonics disagree on ABSOLUTE pitch at
>    that note by ~6 c, far outside their quoted errors, so at least one carries
>    a systematic. It largely cancels in a single note's shift, which is why the
>    shifts looked consistent, but it does NOT cancel in a ratio between notes.
> 2. **The error bars counted only within-run scatter.** Repeating the same cell
>    with the same harmonic on a different run moves it by more than its own
>    quoted sem: note 36 via n=16 gave +3.6355 +/- 0.3169 and then +2.9814 +/-
>    0.1975. The machine is analog and drifts between runs; the sem over
>    triggers inside one run is not the total uncertainty.
>
> Corrected below. **The ratio is consistent with the prediction and 63:39:0 is
> neither confirmed nor refuted at the precision reached.** The near-vanishing
> at note 99 survives both defects and is the part I would still defend.


Instrument `tools/diag/g13_sign.py run` at `e46bbc4`. Bench unit `0.2 1a6010b`,
unchanged, no reflash. Artifacts `/tmp/g13-run-<note>-<mode>-<i>.wav`, 16 per
cell. Capture device `:2` (SSL 18), verified against the live device list by the
operator session, which matters because `:2` is the same literal that produced
finding 25's muted-microphone capture.

## The result

8 MHz arm (`0x3E` mode 1) against the 4 MHz control (mode 0), same note, same
onset-locked window, harmonic-referred pitch, 40 triggers per cell. The
published quantity is the difference, so the machine's absolute tuning, the
estimator's bias at that harmonic and the envelope shape all cancel.

**Notes 36 and 60 are measured through the SAME harmonic (n=16).** That matters:
see the correction at the top.

| note | cv | 63 - cv | shift vs control | +/- (sem) |
|---|---|---|---|---|
| 36 | 0  | 63 | +2.9814 c | 0.1975 |
| 60 | 24 | 39 | +2.3758 c | 0.1031 |
| 99 | 63 | 0  | +0.1872 c | 0.0781 |

Note 99 is via n=7, the highest harmonic that fits under Nyquist at 2489 Hz. It
is compared against ZERO rather than entering a ratio, so it does not need to
match the others' harmonic.

### The ratio, which is the actual test

    note60 / note36   measured  0.797 +/- 0.063   (within-run sem only)
                                0.797 +/- 0.110   (errors inflated 1.75x for
                                                   the observed between-run drift)
    predicted                   0.619

**1.6 sigma with the honest error bar. That is consistent, not a refutation.**
The 1.75x inflation is not a fudge chosen to reach a conclusion: it is the
factor by which same-harmonic repeats of the same cell actually disagree
(note 36 at n=16 moved 0.65 c between runs against a combined error of 0.37;
note 36 at n=64 moved 0.21 c against 0.14).

There is a hint of an excess (the central value sits high, at 0.80 against
0.62), and it has now leaned the same way in every pairing I have measured. But
a 1.6-sigma lean is a reason to measure the drift properly, not to publish a
refutation. **I published one, and this document is the retraction.**

### What survives cleanly

**The shift is real and it is monotonic in cv.** ~+3 c at note 36, ~+2.4 c at
note 60, consistent with zero at note 99.

**The near-vanishing at the top is the load-bearing observation.** +0.187 +/-
0.078 c, which is 2.4 sigma from zero on the within-run error and 1.4 sigma on
the inflated one, against +2.98 c at note 36. No common-mode confounder (clock
error, capture-rate error, tuning drift, temperature) produces that: those move
every note together. This survives both of the defects that sank the ratio
claim, because it involves one note compared against zero rather than two notes
compared against each other.

## What is NOT claimed

**There is no rail-versus-load attribution here.** Only the 8 MHz arm measured.
Mode 4 (idle-sleep: lower load at a FIXED clock) is the arm that separates the
two mechanisms, and mode 8 (16 MHz) changes clock, load and loop rate together
so a shift under it is not attributable anyway. Both failed to measure, for an
instrument reason given below rather than a machine one.

**Only the 8 MHz arm's pitch channel.** Nothing here measures the rail directly;
that was deliberate, because the AVR's bandgap-against-AVCC read steps ~22 mV
near 5 V and stalled this claim twice.

## Why the other two arms did not measure

The harmonic search band is centred on the **nominal** pitch and is `+/-0.4/n`
wide, so it holds only while the machine's real pitch stays inside that: 0.625 %
at n=64. The 8 MHz shift is 0.19 % and is safely inside. The 16 MHz shift is
large enough to sit on the edge, so the estimator intermittently locks onto a
neighbouring harmonic: **sd 6.56 c against the control's 0.32 c in the same
run.**

A ladder that re-centres each rung on the previous one was tried and is
**worse** (sd 74 c at 16 MHz, 42.9 c at idle-sleep), because its first rung is
the fundamental and the noisy fundamental is the exact thing the harmonic
referral exists to avoid. The fix is a first rung that is both wide and quiet;
it is host-side and it is the next job on this claim.

## Two machine facts that were assumptions before this run

**The 101 does not sustain a note, and the firmware is not at fault.** The GATE
is asserted continuously for the whole hold and released on note-off (probe
`0x3B -> 0x3C` `raw[97]`, P2 bit 7, which is 0 when the gate is ON; held ON
across 12 s of sampling). The **VCA envelope** decays to the noise floor about
2.2 s after onset. That is analog, set by physical sliders, and cannot be
changed headless. The method therefore retriggers per sample instead of holding
one note.

This costs the confounder argument that motivated holding a note: at a clock the
app was not compiled for, `hw.h`'s `IdleWaitUs` dwells come out short, S/H
acquisition is incomplete, and for a HELD note the S/H is refreshed to the same
value every pass so the error converges to zero. **Retriggering does not get
that for free**, and it is a live confounder for the 8 MHz arm rather than a
closed one.

**The machine's own low-note pitch instability is the dominant noise.**
Per-trigger repeat spread on the real signal, on the fundamental:

| note | f0 | spread | as absolute Hz |
|---|---|---|---|
| 36 | 65.4 Hz | 11.67 c | ~0.44 Hz |
| 60 | 261.6 Hz | 3.41 c | ~0.51 Hz |
| 99 | 2489 Hz | 0.87 c | ~1.25 Hz |

against the instrument's **synthetic floor of 0.026 c**. Roughly constant in
absolute Hz, which is why it reads enormous at the bottom of the range.
Measuring harmonic N and referring back to f0 divides the cents error by N
(0.41 / 0.12 / 0.08 c), and that is what made 16 triggers per cell enough
instead of the ~13,700 the raw fundamental would have needed.

Three explanations were raised and **killed**, rather than left standing:

* **symmetric AM from the LFO** -- sidebands are symmetric and their pull
  cancels exactly: sd 0.000 c synthetic, at every note.
* **the measured envelope** -- the real envelope applied to a tone of exactly
  known f0 gives 0.73 c at note 36, under a tenth of the observed scatter.
* **60 Hz mains hum**, which was attractive because note 36 is 65.4 Hz -- there
  is no 60 Hz component in the floor capture (the strongest 40-200 Hz
  components are 124-163 Hz), and even 30 % hum gives only 0.62 c.

## Four instrument defects found on the way

Each would have published a confident wrong number.

1. **`sustain` reported HELD for a note that had decayed 56 dB.** It judged the
   tail's flatness, and a noise floor is perfectly flat. It now measures a real
   floor with nothing sounding and requires level and SNR as well. It is RED on
   this machine, which is the true answer.
2. **`capture()` starts ffmpeg AFTER note-on.** ffmpeg's startup latency is of
   order a second and varies, so the analysis window landed somewhere different
   in the envelope on every run: two runs minutes apart reported peak levels
   4.5x apart, and that was first misread as the machine decaying faster than it
   does. Notes are now fired DURING the capture and the window is locked to the
   detected onset. **This is not specific to this tool: anything on this bench
   that sends a stimulus and then starts a capture has it.**
3. **`selftest` analysed +1.0 to +4.0 s of a held note**, which here is entirely
   noise floor. Given an `expect` band it locks onto the loudest noise bin and
   prints a respectable repeat spread for a measurement of silence.
4. **The harmonic band was fixed at +/-6 %, which at n=16 spans +/-3.8
   harmonics.** n=64 at note 36 returned sd 0.14 c with the mean shifted +57 c:
   tight, confident, and locked onto the wrong harmonic. The band now shrinks as
   `0.4/n`.

## The generalisable lesson

**An instrument validated only against synthetic signals was 450x worse on the
real one** (0.026 c to 11.67 c) with nothing in the synthetic suite looking
wrong. The synthetic floor was honest and useless: the suite lacked the one
property the real signal had. A synthetic validation bounds the estimator, not
the measurement, and the two were quoted as if they were the same number for
four rotations.
