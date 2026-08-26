# claim-038 G13: the 63:39:0 sign test, run

> **SECOND CORRECTION, 2026-08-26, by the author. THE ONE SENTENCE THIS
> DOCUMENT MOST WANTS YOU TO BELIEVE IS NOW REFUTED.** The block below ends
> "the near-vanishing at note 99 survives both defects and is the part I would
> still defend". **Note 99 does not vanish.** Re-taken as five screened
> repeats it is **+0.5988 +/- 0.0073 c**, and the old +0.1872 came from
> reading h7 at -54 dB, the weakest partial in the capture. Everything in this
> document about note 99 below this line is superseded.
>
> **63:39:0 is REFUTED at its control point.** Four rows now exist:
>
>     63-cv 63 (note 36)  +4.0224 +/- 0.0754 stat  0.1226 harmonic-systematic
>     63-cv 39 (note 60)  +2.9062 +/- 0.0918 stat  0.0213
>     63-cv 27 (note 72)  +1.7122 +/- 0.0286 stat  0.0241
>     63-cv  0 (note 99)  +0.5988 +/- 0.0073 stat  0.0005
>
> **NO low-order law in (63-cv) fits them**: proportional chi2 6714 on 3 dof,
> affine 52.6 on 2, quadratic 25.3 on 1. **An affine law WAS proposed here
> earlier today and is also refuted**, by the note-72 row taken specifically to
> test it: it predicted +2.07 c against a measured +1.71, a 4.9 sigma miss once
> the prediction's own error is propagated (NOT the 9.5 sigma first written,
> which divided by the measurement error alone; CORE caught that and it is the
> same class of fault this claim already carries).
>
> **What survives is the SHAPE**: the shift is monotone decreasing in cv across
> four notes spanning the full DAC range and is largest at cv 0. What fails is
> every simple law written for it so far.
>
> Current: `artifacts/g13/101-four-rows-and-the-law.txt`,
> `101-note99-and-the-law.txt`, `101-harmonic-level-guard.txt`, and the ledger
> entry at `claim-038` (2026-08-26). Still open and unchanged: no
> rail-versus-load attribution, because modes 4 and 8 have never produced a
> usable cell.


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

## Where the scatter comes from, and a second correction to note 99 (2026-08-26)

Instrument `tools/diag/g13_sign.py reanalyse` at `6bbd87c`, artifact
`artifacts/g13/101-reanalyse-harmonics.txt`, over the 226 preserved captures in
`artifacts/g13/sign-2026-08-26/`. Host-side, no hardware, no lease. Bench unit
still `0.2 1a6010b`, nothing flashed.

**Method.** One capture read at two harmonics is one physical event read two
ways. Anything the MACHINE did is common to both readings and cancels in their
difference; what survives is the estimator. So `sd(h_a - h_b)` against the
`sqrt(s_a^2 + s_b^2)` expected from independent estimator noise splits a cell's
scatter into its two causes without needing to spot an outlier.

### The scatter at note 36 is NOT the estimator, and I predicted that it was

    note 36 mode 0   sd 0.669 c at h16, sd(h16-h32) 0.121 c   ratio 0.13
    note 36 mode 1   sd 1.055 c at h16, sd(h16-h32) 0.464 c   ratio 0.30
    note 60          ratio 1.08 and 1.02                      estimator
    note 99          both requests clamp to harmonic 7        NOT TESTABLE so

At note 36, the note the claim rests on, about 87 % of the within-cell scatter
is common to both readings. **My handoff put ~70 % on the estimator
intermittently losing the harmonic. That is wrong.**

**Stated at the precision the test supports:** the scatter is upstream of the
harmonic referral. Real pitch, window placement and capture are all upstream of
it, and this test cannot separate them, because both readings share one onset
window. "The machine" is the likely member of that set, not the measured one.

**What it changes.** The run-quality cut is rejecting runs in which the MACHINE
was more variable, not runs the estimator fumbled. It is still required, because
the between-repeat error requirement stands either way, but it must not be
described as an estimator-failure filter.

### The shift is robust to the harmonic, which kills one confounder

    note 36 mode 1   +2.6523 (h8)  +2.9814 (h16)  +3.1048 (h32)
    note 60 mode 1   +2.3761       +2.3758        +2.3586
    note 99 mode 1   +0.5368 (h4)  +0.1872 (h7)

Note 60 agrees to 0.018 c across a harmonic pair whose ABSOLUTE readings sit
5.04 c apart. A difference measurement is supposed to do exactly that, and here
it demonstrably does.

### Note 99, tightened again and for an independent reason

**Its shift moves 0.35 c with the harmonic**, against a quoted within-run sem of
0.0781 c. So the note-99 error must be inflated to at least 0.35 c on estimator
choice alone, before any between-run inflation, and +0.1872 c sits inside that.

CORE tightened this row once already, from "vanishes" to "consistent with zero".
It is now consistent with zero for a second, independent reason. **"Not resolved
by this instrument" is defensible; "measured small" is not.** It must never be
quoted as a measured null.

### The blocked arms are still blocked, and here is the number

Modes 8 (16 MHz) and 4 (idle-sleep) swing +16.16 c to -2.14 c with the harmonic
at note 36. Those shifts are estimator-generated at these harmonics, which is
the band limit already documented in `estimate_f0_harmonic`. **No
rail-versus-load attribution exists and none is available by this route.**

### Instrument defect, found by the instrument's own output

`pick_harmonic` clamps to Nyquist, so at note 99 a request for h16 and one for
h32 BOTH resolve to harmonic 7. The decomposition was subtracting a number from
itself: sd exactly 0.0000, ratio 0.00, and the verdict printed "MACHINE
dominates" for a comparison that was never made. It now refuses to compare when
both requests clamp to the same harmonic. This is the fifth defect on this claim
found by the instrument disagreeing with itself rather than by eyeballing.

## The run-quality cut (2026-08-26)

`tools/diag/g13_sign.py` at `6bbd87c`. **No shift on this claim is publishable
except through `pool`.**

* a cell whose within-run sd exceeds 3x the median cell sd for its note is
  REJECTED and printed with its numbers. **Rejection is on spread alone, never
  on the mean**, so the cut cannot know which way it would move the answer.
* under 4 cells the cut REFUSES rather than running. A median over two cells is
  their mean, and one bad cell drags the reference by half its own excess. An
  underpowered cut is no result, not a weak one (COMMON rule 1).
* **3 surviving repeats are required** before any shift is printed. Fewer is
  "not measured", and is not a null.
* **the error is `sd(per-repeat shifts)/sqrt(k)`**, the between-repeat spread.
  The within-run sem understates it by about 3x: five note-36 h16 measurements
  of one fixed configuration gave +3.6355 +2.9814 +3.9122 +2.3749 +4.0067, sd
  0.691 c, against quoted sems of 0.10 to 0.32 c.
* `pool` prints the UNCUT shift beside the cut one. **If they disagree by more
  than the quoted error, the cut is deciding the answer** and the run needs
  explaining rather than publishing. That is the residual risk the cut cannot
  rule out on its own: if the shift and the scatter share a cause, dropping the
  scattered cells biases the survivors.
* runs write a JSON sidecar; `pool` combines them. Separate invocations because
  a tool call caps at 10 minutes here and one repeat is ~3.5 minutes.

**`cutcheck`: 8 cases, no hardware, and 3 of them are injections that MUST go
red** (L9: a suite that has never failed proves nothing). Disabling the cut
moves case 3 from +3.0000 to +2.6400 c; a wide cell whose MEAN matches the good
ones is still rejected; two runs at different harmonics refuse to pool.

## Note 36, measured properly (2026-08-26)

Instrument `tools/diag/g13_sign.py {run,screen,pool}` at `599357d`. Artifact
`artifacts/g13/101-note36-pooled.txt`. Bench unit `0.2 1a6010b`, not reflashed.
Five repeats of one fixed configuration: note 36, modes 0 and 1, n=20,
harmonic 16 throughout, arms interleaved, each screened per capture by the
uniform host-side pass, pooled with the error from the between-repeat spread.

    cut-r1   +4.2444 c   mode 0 dropped 1 of 20
    cut-r2   +3.9887 c
    cut-r3   +3.8583 c
    cut-r4   +4.0738 c
    cut-r5   +3.7540 c
    SHIFT    +3.9838 +/- 0.0850 c   (between-repeat sd 0.1901, k=5)

**The cell-level cut rejected nothing.** All ten cells came in under 3x the
median cell sd, so the cut and uncut numbers agree to four decimal places and
this row is the PER-CAPTURE screen's work alone. The cell cut was what the
handoff asked for and it was not what was needed.

### The between-run error I handed myself was wrong too

**This corrects the second numbered defect in the correction at the top of this
document.** That note says error bars counted only within-run scatter and that
the between-run spread is about 3x the within-run sem, from five measurements
spanning +2.3749 to +4.0067 c with sd 0.691 c.

**Measured on five clean repeats: the between-repeat sd is 0.1901 c, SMALLER
than the typical within-run sem of 0.21 to 0.30 c.** There is no resolvable
extra between-run component at all.

**The 0.691 c was contaminated by its own bad runs.** Two of those five were
taken in sequential order, which puts ~1.75 min between the halves of a
difference. One (+2.3749) announced itself with a sem of 1.7501, 6x its
neighbours. The two that were interleaved and not internally scattered were
+3.9122 and +4.0067, which sit on top of today's five; those two pooled with
today's five give 4.014 +/- sd 0.136.

**What still stands, and what does not.** The retraction of the 5.4 sigma
stands: the mixed harmonics alone are sufficient for it and are not in doubt.
**What does not stand is "inflate a within-run error by ~3x".** Anyone doing
that on this document's authority should stop.

### What is NOT claimed here

**Not that the screen is why the spread collapsed.** Runs r2 through r5 had no
captures dropped at all and are just as tight as r1 is after screening, so the
screen cannot be what tightened them. The screen is real and it saved r1 (1.6 c
from a single capture), and that is the whole of its demonstrated worth.

The honest account is narrower than either story this claim has told: **the bad
runs were identifiable from their own internal evidence, which is what a cut is
for**, and at least one of them was a per-capture estimator failure. No cause is
claimed for the other two.

**Still no ratio.** It needs note 60 at the same precision, which is running.
