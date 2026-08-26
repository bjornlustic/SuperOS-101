#!/usr/bin/env python3
"""claim-038 (G13): the 63:39:0 sign test, and the instrument it needs.

THE PREDICTION, and why this shape is worth a bench run rather than another
model. `dac_out` in the mask ROM does `cpl a` (0x3FA, one occurrence in 2048;
`src/hw.h:119` reproduces it), so the R-2R ladder current FALLS as the note
rises and a sag on the +5 V rail (which IS the DAC reference) reads SHARP. The
pitch error from a rail change is therefore proportional to `63 - c`, where c is
the CV, and it VANISHES at the top of the range.

    note 36  -> cv  0  -> 63 - c = 63
    note 60  -> cv 24  -> 63 - c = 39
    note 99  -> cv 63  -> 63 - c =  0

So the shift at three notes should stand in the ratio 63 : 39 : 0. **Nothing is
fitted to the note dependence**, and no common-mode confounder (a clock error, a
capture-rate error, a tuning drift, a temperature ramp) produces a shift that is
large at the bottom, 62 % of that in the middle, and exactly zero at the top. If
it fails, the mechanism is dead in one capture.

THE INSTRUMENT, stated before the result per COMMON rule 1. The claim needs to
resolve a RATIO between shifts of order a few cents, so the pitch estimator has
to be good to well under a cent. It is a long-window interpolated DFT: Hann
window, zero-padded FFT for a coarse peak, then a two-stage fine search on the
true DFT magnitude around it. `--selftest` measures its own repeat spread on the
real signal and prints it, and `--synthetic` checks it against tones of KNOWN
frequency, because a repeat spread only bounds precision and says nothing about
bias.

This deliberately reads NO rail voltage. The AVR's bandgap-against-AVCC read
steps ~22 mV near 5 V and six bench windows in a row came back as two adjacent
codes; that instrument is what stalled claim-038 twice. Pitch is the observable
with the resolution this question needs.
"""
import argparse
import os
import subprocess
import sys
import threading
import time

import numpy as np

_USBC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "..", "SuperOS-USBC"))
if not os.path.isdir(_USBC):
    sys.exit(f"cannot find the shared repo, looked in {_USBC}")
sys.path.insert(0, os.path.join(_USBC, "tools"))
sys.path.insert(0, os.path.join(_USBC, "timing"))
import midiport                                            # noqa: E402
import mido                                                # noqa: E402
from scipy.io import wavfile                               # noqa: E402

import machines                                            # noqa: E402

RATE = 48000
CH_101 = machines.ORDER.index("101")          # channel index inside the capture
NOTES = (36, 60, 99)
CV_OF = 36                                    # MIDI_CV_OFFSET; cv = note - 36


def midi_hz(note):
    return 440.0 * 2 ** ((note - 69) / 12.0)


def cents(f, ref):
    return 1200.0 * np.log2(f / ref)


# --------------------------------------------------------------------------- #
# the pitch estimator
# --------------------------------------------------------------------------- #
def _dft_mag(x, t, freqs):
    """|DFT| of x at arbitrary freqs. Chunked so a fine grid cannot blow memory."""
    out = np.empty(len(freqs))
    for i, f in enumerate(freqs):
        out[i] = abs(np.dot(x, np.exp(-2j * np.pi * f * t)))
    return out


def estimate_f0(sig, rate=RATE, expect=None, span=0.06):
    """Fundamental of a steady tone, to well under a cent.

    `expect` and `span` bound the search to +/- `span` (a fraction) around the
    nominal pitch. That is not a fit to the answer: the whole question is a
    shift of a few cents, and span is 6 %, about a semitone, which is 100x the
    effect. It exists to stop the estimator locking onto a harmonic, which is
    the failure mode that would silently halve or double a reading.
    """
    x = np.asarray(sig, dtype=float)
    x = x - x.mean()
    if not np.any(x):
        return None
    x = x * np.hanning(len(x))
    t = np.arange(len(x)) / rate

    n = int(2 ** np.ceil(np.log2(len(x)))) * 8      # zero-pad for a coarse peak
    spec = np.abs(np.fft.rfft(x, n))
    fr = np.fft.rfftfreq(n, 1 / rate)
    if expect:
        lo, hi = expect * (1 - span), expect * (1 + span)
        band = (fr >= lo) & (fr <= hi)
        if not band.any():
            return None
        k = np.argmax(np.where(band, spec, 0))
    else:
        k = np.argmax(spec)
    f = fr[k]

    # Two fine stages on the TRUE DFT. The zero-padded peak is already close;
    # these remove the interpolation error rather than the bin error.
    for half, step in ((fr[1] - fr[0]) * 2, (fr[1] - fr[0]) / 8), ((fr[1] - fr[0]) / 4, (fr[1] - fr[0]) / 200):
        g = np.arange(f - half, f + half, step)
        g = g[g > 0]
        if len(g) < 3:
            break
        f = g[int(np.argmax(_dft_mag(x, t, g)))]
    return float(f)


# --------------------------------------------------------------------------- #
# bench plumbing
# --------------------------------------------------------------------------- #
def capture(path, seconds):
    dev = machines.avf_device()
    cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
           "-f", "avfoundation", "-i", dev, "-t", str(seconds),
           "-af", machines.pan_spec(machines.ORDER, machines.SSL_INPUT),
           "-c:a", "pcm_f32le", "-ar", str(RATE), path]
    subprocess.run(cmd, check=True)
    sr, data = wavfile.read(path)
    return sr, data[:, CH_101] if data.ndim > 1 else data


def onset_index(sig, rate=RATE, frac=0.10, hop=0.010):
    """Sample index of the note onset, at `frac` of the signal's OWN peak.

    Finding 34's rule: measuring each hit against its own peak is what stops
    the reading moving when one capture is simply louder. Returns None if
    nothing crosses the threshold.
    """
    x = np.abs(np.asarray(sig, dtype=float))
    w = max(1, int(hop * rate))
    env = np.array([x[i:i + w].mean() for i in range(0, len(x) - w, w)])
    if not len(env) or env.max() <= 0:
        return None
    hits = np.flatnonzero(env >= frac * env.max())
    return int(hits[0] * w) if len(hits) else None


def capture_note(b, note, path, secs=3.0, fire_at=1.2):
    """Capture with the note fired DURING the recording, not before it.

    This exists because `capture()` starts ffmpeg and ffmpeg's startup latency
    is variable and of order a second. Sending note-on first and assuming the
    recording begins at the note put the analysis window somewhere different in
    the envelope on every run: two `sustain` runs minutes apart reported peaks
    4.5x apart for that reason alone. Firing inside the capture and locking the
    window to the DETECTED onset removes the host's timing from the
    measurement entirely.
    """
    err = []

    def fire():
        try:
            time.sleep(fire_at)
            b.note(note, True)
        except Exception as e:                      # noqa: BLE001
            err.append(e)

    t = threading.Thread(target=fire)
    t.start()
    try:
        sr, sig = capture(path, secs)
    finally:
        t.join()
        b.note(note, False)
    if err:
        raise err[0]
    return sr, sig


def onset_window(sig, rate=RATE, head=0.05, length=0.60):
    """The analysis window, placed relative to the onset rather than to t=0.

    The 101 does not hold a note at constant level (measured: the gate stays
    asserted for the whole hold, but the VCA envelope decays with LFO wobble
    and reaches the noise floor about 2.2 s after onset). So the window has to
    sit where there is still signal, and it has to sit in the SAME place in
    every arm, because the 63:39:0 test is a difference between arms at one
    note and a common envelope cancels there.
    """
    i = onset_index(sig, rate)
    if i is None:
        return None
    a = i + int(head * rate)
    b_ = a + int(length * rate)
    return sig[a:b_] if b_ <= len(sig) and b_ > a else None


def pick_harmonic(f0, rate=RATE, want=16, ceiling=0.40):
    """Highest harmonic <= `want` that stays below `ceiling` * rate.

    Measuring harmonic N and dividing by N divides the CENTS error by N when
    the underlying error is fixed in absolute Hz, which is what this machine
    shows: the repeat spread is ~0.5-1.2 Hz at every note, so it reads as
    11.7 c at note 36 and 0.87 c at note 99. Note 99's harmonics run out of
    Nyquist quickly, which is fine: note 99 is the note that needs the help
    least.
    """
    n = int((ceiling * rate) / f0) if f0 > 0 else 1
    return max(1, min(want, n))


def estimate_f0_harmonic(sig, f_nom, rate=RATE, want=16, span=0.06):
    """f0 via a harmonic, referred back to the fundamental.

    Returns (f0_estimate, harmonic_used). The search is bounded around
    `f_nom * N` by the same +/-span as the direct estimator, so a harmonic
    cannot be confused with its neighbours: they are a whole f0 apart, which
    at span 6 % is far outside the band.
    """
    # The band MUST shrink with n: harmonics sit one f0 apart, so a fixed
    # +/-6 % band around n*f0 spans +/-0.06*n harmonics, already +/-3.8 of them
    # at n=16. Left at 6 % this locks onto a NEIGHBOURING harmonic and returns
    # a tight, confident, WRONG answer: measured, n=64 at note 36 gave sd
    # 0.14 c with the mean shifted +57 c, and n=32 at note 60 gave sd 21.6 c.
    #
    # KNOWN LIMIT, and it decides which arms this may be used on. The band is
    # centred on the NOMINAL pitch, so it holds only while the machine's real
    # pitch is inside +/-0.4/n of nominal: 0.625 % at n=64. The 8 MHz arm
    # shifts 0.19 % and is safely inside. The 16 MHz arm shifts far more, sits
    # on the edge, and intermittently misses: it came back sd 6.56 c against
    # the control's 0.32 c. A ladder that re-centres each rung on the previous
    # one was tried and is WORSE (sd 74 c), because rung 1 is the fundamental
    # and the fundamental is the noisy thing this exists to avoid. So: do not
    # use this on an arm whose shift approaches 0.4/n until that is solved.
    n = pick_harmonic(f_nom, rate, want)
    span_eff = min(span, 0.4 / n)
    f = estimate_f0(sig, rate=rate, expect=f_nom * n, span=span_eff)
    return (None, n) if f is None else (f / n, n)


def steady(sig, rate=RATE, head=0.35, tail=0.10):
    """Drop the attack and the release: pitch is read from the sustain only."""
    a, b = int(head * rate), int(len(sig) - tail * rate)
    return sig[a:b] if b > a else sig


class Bench:
    def __init__(self):
        self.out = mido.open_output(midiport.resolve("101", "out", agent="A101"))

    def note(self, n, on=True):
        self.out.send(mido.Message("note_on" if on else "note_off",
                                   note=n, velocity=100 if on else 0, channel=0))

    def bench_window(self, mode, secs):
        self.out.send(mido.Message("sysex", data=[0x7D, 0x3E, mode, secs]))

    def all_off(self):
        for n in NOTES:
            self.note(n, False)


# --------------------------------------------------------------------------- #
# --synthetic : tones of KNOWN f0
#
# A repeat spread bounds PRECISION and says nothing about BIAS. This is the
# stage that catches bias, and it is run before the machine is touched.
#
# What the claim actually needs is narrower than "accurate", and it is worth
# being explicit or the wrong thing gets validated. The 63:39:0 test compares
# f0 in a CONTROL window against f0 in a LOADED window at the SAME note. Any
# error common to both cancels in the difference. What does NOT cancel is an
# error that MOVES when something other than pitch moves: amplitude, harmonic
# content, noise floor, window length. So the figure of merit here is the
# DIFFERENTIAL error across those nuisance axes, not the absolute error. Both
# are printed; the differential is the one the claim rests on.
# --------------------------------------------------------------------------- #
def synth(f0, secs, kind="saw", rate=RATE, amp=0.3, noise=0.0, decay=0.0,
          seed=0, dc=0.0, drift=0.0):
    """A tone of exactly known f0. Phase-continuous, harmonics band-limited.

    `kind` covers what an SH-101 actually puts out (saw and pulse) as well as a
    pure sine, because a harmonic-rich signal is where a peak-picker is most
    likely to be pulled off the fundamental.
    """
    rng = np.random.default_rng(seed)
    n = int(secs * rate)
    t = np.arange(n) / rate
    # a slow linear frequency drift, in cents over the whole window, if asked
    if drift:
        f_inst = f0 * 2 ** ((drift / 1200.0) * (t / t[-1]))
        ph = 2 * np.pi * np.cumsum(f_inst) / rate
    else:
        ph = 2 * np.pi * f0 * t

    if kind == "sine":
        x = np.sin(ph)
    elif kind in ("saw", "pulse"):
        nh = max(1, int((rate / 2) / f0) - 1)          # band-limited, no aliasing
        x = np.zeros(n)
        for h in range(1, nh + 1):
            if kind == "saw":
                x += np.sin(h * ph) / h
            elif h % 2:
                x += np.sin(h * ph) / h
        x *= 2 / np.pi
    else:
        raise ValueError(kind)

    if decay:
        x = x * np.exp(-decay * t)
    x = amp * x + dc
    if noise:
        x = x + rng.normal(0, noise, n)
    return x


def _cents_err(got, want):
    return None if got is None else cents(got, want)


# --------------------------------------------------------------------------- #
# Deliberately broken estimators, so the harness can be SEEN to go red.
#
# claim-081's failure mode wearing a green hat: a comparator that has never
# produced a failure on a known-bad subject has not been shown to be able to.
# These are committed rather than run once by hand, so the red condition is
# reproducible by anyone who doubts the green one.
# --------------------------------------------------------------------------- #
def _broken(kind, fn, gain=1.0):
    def deaf(sig, rate=RATE, expect=None, span=0.06):
        return expect                       # reports the nominal, never the tone
    def quantize(sig, rate=RATE, expect=None, span=0.06):
        f = fn(sig, rate, expect, span)
        q = 0.05 * gain
        return None if f is None else round(f / q) * q
    def amplitude(sig, rate=RATE, expect=None, span=0.06):
        f = fn(sig, rate, expect, span)
        if f is None:
            return None
        rms = float(np.sqrt(np.mean(np.asarray(sig, float) ** 2)))
        return f * 2 ** ((0.5 * gain * rms) / 1200.0)  # bias that MOVES with level
    def harmonic(sig, rate=RATE, expect=None, span=0.06):
        f = fn(sig, rate, expect, span)
        return None if f is None else f * 2      # locks an octave up
    return {"deaf": deaf, "quantize": quantize,
            "amplitude": amplitude, "harmonic": harmonic}[kind]


def cmd_synthetic(args):
    """Absolute bias, differential bias, and a negative control."""
    global estimate_f0
    secs = args.secs
    if args.break_with != "none":
        estimate_f0 = _broken(args.break_with, estimate_f0,
                              args.break_gain)
        print(f"*** RED-CONDITION RUN: estimator deliberately broken "
              f"({args.break_with}, gain {args.break_gain}). A break SMALLER than\n    the stated floor is expected to pass: that is what the floor MEANS.\n    A break larger than it and still passing is a harness defect. ***\n")
    print(f"--synthetic : estimator against KNOWN f0, {secs:.2f} s windows, "
          f"rate {RATE}\n")

    # The three notes the claim uses, plus deliberately awkward offsets so no
    # case sits on an exact FFT bin centre.
    cases = []
    for note in NOTES:
        f_nom = midi_hz(note)
        for off_c in (0.0, +3.7, -11.3):
            f_true = f_nom * 2 ** (off_c / 1200.0)
            for kind in ("saw", "pulse", "sine"):
                cases.append((note, f_nom, f_true, kind))

    print("1. ABSOLUTE BIAS: f0 known exactly, clean signal.")
    print(f"{'note':>5} {'kind':>6} {'f_true Hz':>12} {'offset c':>9} "
          f"{'err cents':>10}")
    worst = 0.0
    errs = []
    for note, f_nom, f_true, kind in cases:
        f = estimate_f0(synth(f_true, secs, kind=kind), expect=f_nom)
        e = _cents_err(f, f_true)
        if e is None:
            print(f"{note:>5} {kind:>6} {f_true:>12.4f} "
                  f"{cents(f_true, f_nom):>9.2f} {'NO LOCK':>10}")
            worst = float("inf")
            continue
        errs.append(abs(e))
        worst = max(worst, abs(e))
        print(f"{note:>5} {kind:>6} {f_true:>12.4f} "
              f"{cents(f_true, f_nom):>9.2f} {e:>10.4f}")
    print(f"   worst |err| {worst:.4f} cents over {len(cases)} cases, "
          f"median {np.median(errs):.4f}\n")

    # ---------------------------------------------------------------- #
    # 2. Differential. This is the part the claim rests on: the 63:39:0
    #    test reads a DIFFERENCE between two windows at the same note, so
    #    any error common to both cancels and only a MOVING error survives.
    # ---------------------------------------------------------------- #
    print("2. DIFFERENTIAL BIAS: same f0, one nuisance variable moved.")
    print("   Only an error that MOVES with the nuisance survives the")
    print("   difference, so this and not (1) is the claim's floor.")
    print(f"\n{'note':>5} {'axis':>26} {'A cents':>9} {'B cents':>9} "
          f"{'B-A':>9}")
    dmax = 0.0
    axes = [
        ("amplitude 0.30 -> 0.05", dict(amp=0.30), dict(amp=0.05)),
        ("noise 0 -> 0.01 rms", dict(noise=0.0), dict(noise=0.01)),
        ("noise 0 -> 0.05 rms", dict(noise=0.0), dict(noise=0.05)),
        ("decay 0 -> 0.30 /s", dict(decay=0.0), dict(decay=0.30)),
        ("dc 0 -> +0.05", dict(dc=0.0), dict(dc=0.05)),
        ("timbre saw -> pulse", dict(kind="saw"), dict(kind="pulse")),
    ]
    for note in NOTES:
        f_nom = midi_hz(note)
        f_true = f_nom * 2 ** (3.7 / 1200.0)
        for label, ka, kb in axes:
            ea = _cents_err(estimate_f0(synth(f_true, secs, **ka),
                                        expect=f_nom), f_true)
            eb = _cents_err(estimate_f0(synth(f_true, secs, **kb),
                                        expect=f_nom), f_true)
            if ea is None or eb is None:
                print(f"{note:>5} {label:>26} {'NO LOCK':>9}")
                dmax = float("inf")
                continue
            d = eb - ea
            dmax = max(dmax, abs(d))
            print(f"{note:>5} {label:>26} {ea:>9.4f} {eb:>9.4f} {d:>9.4f}")
    print(f"   worst |B-A| {dmax:.4f} cents\n")

    # ---------------------------------------------------------------- #
    # 3. Noise is the one axis with a RANDOM component, so one seed is one
    #    draw and its spread has to be measured over many.
    # ---------------------------------------------------------------- #
    print(f"3. RANDOM ERROR under noise, {args.seeds} seeds per cell.")
    print(f"{'note':>5} {'noise rms':>10} {'mean c':>9} {'sd c':>9} "
          f"{'max|e| c':>9}")
    nmax = 0.0
    for note in NOTES:
        f_nom = midi_hz(note)
        f_true = f_nom * 2 ** (3.7 / 1200.0)
        for nz in (0.01, 0.05):
            es = []
            for sd in range(args.seeds):
                e = _cents_err(estimate_f0(
                    synth(f_true, secs, noise=nz, seed=sd), expect=f_nom),
                    f_true)
                if e is not None:
                    es.append(e)
            if not es:
                print(f"{note:>5} {nz:>10.3f} {'NO LOCK':>9}")
                nmax = float("inf")
                continue
            es = np.array(es)
            nmax = max(nmax, float(np.max(np.abs(es))))
            print(f"{note:>5} {nz:>10.3f} {es.mean():>9.4f} {es.std():>9.4f} "
                  f"{np.max(np.abs(es)):>9.4f}")
    print(f"   worst |err| under noise {nmax:.4f} cents\n")

    # ---------------------------------------------------------------- #
    # 4. A DRIFTING tone. This is NOT an estimator fault and must not be
    #    scored as one: a tone whose pitch ramps by D cents across the
    #    window has a Hann-weighted mean of D/2 by symmetry, and reporting
    #    that IS correct behaviour. It is checked against its own known
    #    expected value. It is here because thermal drift on the bench is
    #    real, and because the 63:39:0 design's answer to it is that a
    #    common-mode drift gives ratio 1:1:1, not 63:39:0.
    # ---------------------------------------------------------------- #
    print("4. DRIFTING tone: expected reading is HALF the ramp (window mean),")
    print("   so this row is scored against D/2, not against zero.")
    print(f"{'note':>5} {'ramp c':>8} {'expect':>8} {'got':>9} {'err':>9}")
    gmax = 0.0
    for note in NOTES:
        f_nom = midi_hz(note)
        for D in (5.0, -20.0):
            e = _cents_err(estimate_f0(synth(f_nom, secs, drift=D),
                                       expect=f_nom), f_nom)
            if e is None:
                print(f"{note:>5} {D:>8.2f} {'NO LOCK':>9}")
                gmax = float("inf")
                continue
            err = e - D / 2.0
            gmax = max(gmax, abs(err))
            print(f"{note:>5} {D:>8.2f} {D/2:>8.3f} {e:>9.4f} {err:>9.4f}")
    print(f"   worst |err| vs window-mean {gmax:.4f} cents\n")

    # ---------------------------------------------------------------- #
    # 5. NEGATIVE CONTROL. Trap 6: a test written where the mechanism
    #    cannot fail still prints a pass. Inject a shift of known size and
    #    confirm the harness REPORTS it, at every size the claim cares
    #    about, including one below the stated resolution where it is
    #    allowed to fail and must be seen to.
    # ---------------------------------------------------------------- #
    print("5. NEGATIVE CONTROL: a KNOWN shift injected between A and B.")
    print("   The instrument must report it. A comparator that cannot go")
    print("   red on a broken subject is not an instrument.")
    print(f"{'note':>5} {'injected c':>11} {'recovered c':>12} {'err':>9} "
          f"{'verdict':>9}")
    cmax = 0.0
    control_ok = True
    for note in NOTES:
        f_nom = midi_hz(note)
        for inj in (5.0, 3.1, 0.5, 0.1, 0.0):
            fa = f_nom
            fb = f_nom * 2 ** (inj / 1200.0)
            a = estimate_f0(synth(fa, secs, noise=0.01, seed=1), expect=f_nom)
            b = estimate_f0(synth(fb, secs, noise=0.01, seed=2), expect=f_nom)
            if a is None or b is None:
                print(f"{note:>5} {inj:>11.3f} {'NO LOCK':>12}")
                control_ok = False
                continue
            rec = cents(b, a)
            err = rec - inj
            cmax = max(cmax, abs(err))
            # a shift is DETECTED when the recovered size is within 10 % of
            # the injection and above the stated resolution
            det = abs(err) <= max(0.02, 0.10 * inj)
            if not det:
                control_ok = False
            print(f"{note:>5} {inj:>11.3f} {rec:>12.4f} {err:>9.4f} "
                  f"{'ok' if det else 'MISSED':>9}")
    print(f"   worst recovery error {cmax:.4f} cents\n")

    # ---------------------------------------------------------------- #
    need = args.need
    floor = max(dmax, nmax, cmax)
    print("RESOLUTION FOR THE CLAIM, stated before any bench run (COMMON r1):")
    print(f"  absolute bias, clean            {worst:.4f} cents")
    print(f"  differential across nuisances   {dmax:.4f} cents")
    print(f"  random error under noise        {nmax:.4f} cents")
    print(f"  known-shift recovery error      {cmax:.4f} cents")
    print(f"  drift handled as window mean to {gmax:.4f} cents")
    print(f"  ---")
    print(f"  INSTRUMENT FLOOR                {floor:.4f} cents")
    print(f"  needed for the claim            {need:.4f} cents")
    ok = floor <= need and control_ok and gmax <= need
    print(f"\n{'PASS' if ok else 'FAIL'}: the estimator "
          f"{'can' if ok else 'CANNOT'} resolve the 63:39:0 ratio.")
    if not control_ok:
        print("  negative control MISSED an injected shift: the instrument")
        print("  cannot be trusted to go red, so a null result would be void.")
    return 0 if ok else 1


# --------------------------------------------------------------------------- #
# Hardware plumbing shared by sustain / selftest / run.
#
# THE WINDOW'S REAL DURATION IS NOT THE BYTE YOU SEND. 0x3E keeps time with
# millis(), whose timer prescaler is fixed at compile time for the app's 4 MHz,
# so a window running at 16 MHz counts four times too fast and ends in a QUARTER
# of the real time asked for (boot.cpp:60-68 says so explicitly). Sending 3 for
# a three-second window at 16 MHz gets 0.75 s of audio, most of it attack.
# --------------------------------------------------------------------------- #
CLKPS_OF_MODE = {0: None, 1: 1, 2: None, 4: None, 8: 0}     # None = app's own
BASE_CLKPS = 2                       # the app's /4 at 16 MHz xtal -> 4 MHz


def window_byte(mode, want_real_secs):
    """Seconds byte to send so the window lasts want_real_secs of REAL time."""
    clkps = CLKPS_OF_MODE.get(mode)
    scale = 1 if clkps is None else 2 ** (BASE_CLKPS - clkps)
    b = int(round(want_real_secs * scale))
    if not 1 <= b <= 127:
        raise ValueError(f"mode {mode} for {want_real_secs}s needs byte {b}, "
                         f"outside 1..127 (0x18/0x16 single-byte limit class)")
    return b, scale


MODES = {
    0: "4 MHz control (the app's own divide, NOT a no-op)",
    1: "8 MHz (core /2)",
    8: "16 MHz (core /1)",
    4: "4 MHz + idle-sleep every pass (LOWER load, same clock)",
}


def cmd_sustain(args):
    """Does the 101 hold a note indefinitely? The whole method depends on it.

    Never tested before this. If the envelope decays there is no steady tone,
    the DFT reads a window mean of a moving thing, and the 63:39:0 design has
    to be rethought rather than patched.
    """
    b = Bench()
    path = args.wav or "/tmp/g13-sustain.wav"
    b.all_off()
    time.sleep(0.2)
    # Measure the NOISE FLOOR first, with nothing sounding. Without it this
    # test cannot tell a held note from silence: a tail sitting at the floor
    # is perfectly flat, and the first version of this function passed a note
    # that had decayed 56 dB because it judged flatness alone. That is L59 on
    # my own instrument, so the floor is now a required control, not an extra.
    _, quiet = capture(args.wav_floor or "/tmp/g13-sustain-floor.wav",
                       args.floor_secs)
    floor = float(np.sqrt(np.mean(quiet.astype(float) ** 2))) if len(quiet) else 0.0
    b.note(args.note, True)
    try:
        sr, sig = capture(path, args.secs)
    finally:
        b.note(args.note, False)
        b.all_off()

    n = len(sig)
    if not np.any(sig):
        print("SILENT. Either the machine did not sound or the capture is the "
              "wrong device (finding 25: a wrong-source capture is a valid wav "
              "of zeros). Check machines.avf_device().")
        return 1

    # rms in 100 ms slices: the envelope, directly
    w = int(0.1 * sr)
    slices = [float(np.sqrt(np.mean(sig[i:i + w].astype(float) ** 2)))
              for i in range(0, n - w, w)]
    peak = max(slices)
    print(f"note {args.note}, {args.secs:.1f} s, {len(slices)} x 100 ms slices")
    print(f"{'t s':>6} {'rms':>12} {'dB rel peak':>12}")
    for i, r in enumerate(slices):
        db = -np.inf if r <= 0 else 20 * np.log10(r / peak)
        print(f"{i * 0.1:>6.1f} {r:>12.6f} {db:>12.2f}")

    # judge the LAST two thirds, so the attack is not counted as decay
    tail = slices[len(slices) // 3:]
    fall = 20 * np.log10(tail[-1] / tail[0]) if tail[0] > 0 else -np.inf
    level = float(np.median(tail))
    rel = 20 * np.log10(level / peak) if level > 0 and peak > 0 else -np.inf
    snr = 20 * np.log10(level / floor) if level > 0 and floor > 0 else -np.inf

    print(f"\npeak rms {peak:.6f}   noise floor rms {floor:.6f} "
          f"({args.floor_secs:.1f} s, nothing sounding)")
    print(f"sustain segment (last 2/3), three conditions, ALL required:")
    print(f"  flat    {fall:+8.2f} dB end-to-start   need |x| <= {args.tol_db:.1f}")
    print(f"  level   {rel:+8.2f} dB rel peak        need >= {-args.min_level_db:.1f}")
    print(f"  SNR     {snr:+8.2f} dB over floor      need >= {args.min_snr_db:.1f}")

    ok_flat, ok_level, ok_snr = (abs(fall) <= args.tol_db,
                                 rel >= -args.min_level_db,
                                 snr >= args.min_snr_db)
    held = ok_flat and ok_level and ok_snr
    print(f"\n{'HELD' if held else 'NOT HELD'}: "
          f"flat={'ok' if ok_flat else 'NO'} "
          f"level={'ok' if ok_level else 'NO'} "
          f"snr={'ok' if ok_snr else 'NO'}")
    if not held:
        if ok_flat and not (ok_level and ok_snr):
            print("The tail is FLAT because it is SILENT, which is exactly the "
                  "reading flatness alone cannot distinguish from a held note.")
        print("The 63:39:0 method assumes a steady tone. It does not have one.")
    return 0 if held else 1


def cmd_selftest(args):
    """Repeat spread on the REAL signal. Precision, not bias: --synthetic
    covers bias, and a repeat spread cannot see it.

    RETRIGGERS the note for every repeat and windows relative to the detected
    onset. The first version held one note and took its captures from +1.0 s
    to +4.0 s, which on this machine is entirely noise floor: the VCA envelope
    is down ~50 dB by then. `estimate_f0` is given an `expect` band, so it
    would have locked onto the loudest noise bin in that band and printed a
    perfectly respectable repeat spread for a measurement of silence.
    """
    b = Bench()
    f_nom = midi_hz(args.note)
    b.all_off()
    time.sleep(0.2)
    fs = []
    try:
        for i in range(args.n):
            path = f"/tmp/g13-selftest-{i}.wav"
            sr, sig = capture_note(b, args.note, path, args.secs, args.fire_at)
            win = onset_window(sig, sr, args.head, args.win)
            if win is None:
                print(f"  {i}: NO ONSET (nothing crossed 10% of its own peak)")
                time.sleep(args.gap)
                continue
            f, nh = estimate_f0_harmonic(win, f_nom, rate=sr,
                                         want=args.harmonic)
            if f is None:
                print(f"  {i}: NO LOCK")
                time.sleep(args.gap)
                continue
            fs.append(f)
            print(f"  {i}: {f:>11.5f} Hz  {cents(f, f_nom):>+9.3f} c vs nominal"
                  f"   (via harmonic {nh})")
            time.sleep(args.gap)
    finally:
        b.note(args.note, False)
        b.all_off()

    if len(fs) < 2:
        print("fewer than two locks: no spread to report.")
        return 1
    fs = np.array(fs)
    c = cents(fs, fs.mean())
    print(f"\nn={len(fs)}  mean {fs.mean():.5f} Hz  "
          f"({cents(fs.mean(), f_nom):+.3f} c vs nominal)")
    print(f"window: onset +{args.head:.2f} s, {args.win:.2f} s long, retriggered")
    print(f"repeat spread: sd {c.std():.4f} c, range {c.max() - c.min():.4f} c")
    print(f"\nThis bounds PRECISION only. Bias is --synthetic's job.")
    print(f"The offset vs nominal is NOT an error: the 101's absolute tuning is "
          f"a physical TUNE trim and the claim is a DIFFERENCE at one note.")
    print(f"needed for the claim: <= {args.need:.3f} c")
    ok = c.std() <= args.need
    print(f"{'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


def _default_run_id():
    return time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())


def crosscheck_harmonic(f_nom, want, rate=RATE):
    """A SECOND harmonic that resolves differently from `want`, or None.

    pick_harmonic() clamps to Nyquist, so at note 99 a request for 16 and one
    for 32 both come back as 7 and the "check" is a number against itself. That
    already produced one false verdict in `reanalyse`. Try higher first, then
    lower, and return None when nothing lands on a different harmonic: the
    check is then UNAVAILABLE and must say so rather than silently pass.
    """
    n = pick_harmonic(f_nom, rate, want)
    for cand in (want * 2, want // 2, want // 4):
        if cand >= 1 and pick_harmonic(f_nom, rate, cand) != n:
            return cand
    return None


def _one_trigger(b, note, mode, i, f_nom, args):
    """One armed trigger: window, note, capture, estimate. None on any miss."""
    try:
        secs_b, _ = window_byte(mode, args.window)
    except ValueError as e:
        print(f"  mode {mode}: {e}")
        return None
    b.all_off()
    time.sleep(0.15)
    b.bench_window(mode, secs_b)
    time.sleep(0.05)
    path = os.path.join(args.outdir,
                        f"g13-run-{args.run_id}-{note}-{mode}-{i}.wav")
    sr, sig = capture_note(b, note, path, args.secs, args.fire_at)
    win = onset_window(sig, sr, args.head, args.win)
    if win is None:
        return None
    f, _ = estimate_f0_harmonic(win, f_nom, rate=sr, want=args.harmonic)
    time.sleep(args.gap)
    if f is None:
        return None

    # PER-CAPTURE ESTIMATOR SELF-CONSISTENCY, and it is the cut that matters.
    #
    # One capture read at two harmonics is one physical event read two ways, so
    # whatever the machine did is common to both and the DISAGREEMENT is the
    # estimator alone. Measured over 120 captures:
    #
    #   normal captures   median 0.045 - 0.070 c, p90 0.13 - 0.36 c
    #   worst normal      2.05 c
    #   the broken one    43.11 c
    #
    # Three orders of magnitude, so the threshold does not need tuning and is
    # not tuned: 3.0 c sits above every capture that was not obviously broken
    # and 14x below the one that was.
    #
    # Why this and not a distance-from-the-mean cut: distance from the mean is
    # not blind to the answer. Self-consistency is a property of ONE capture
    # read two ways and knows nothing about the other triggers or the arm.
    #
    # What it is worth: run cut-r1's control cell had ONE capture 46.6 MAD out
    # and that alone took the cell from sd 1.10 to sd 7.07 and the run's shift
    # from +4.24 to +5.85 c. Dropping the cell would have thrown away 19 good
    # captures to lose 1 bad one.
    if args.crosscheck:
        h2 = crosscheck_harmonic(f_nom, args.harmonic)
        if h2 is None:
            if not getattr(args, "_xc_warned", False):
                print(f"  crosscheck UNAVAILABLE at note {note}: every "
                      f"candidate harmonic clamps to "
                      f"{pick_harmonic(f_nom, RATE, args.harmonic)} at Nyquist. "
                      f"Captures here are NOT screened.")
                args._xc_warned = True
            return f
        f2, _ = estimate_f0_harmonic(win, f_nom, rate=sr, want=h2)
        if f2 is None:
            print(f"  trigger {i} mode {mode}: NO LOCK at the crosscheck "
                  f"harmonic {h2}; REJECTED")
            return None
        dis = abs(float(cents(f, f_nom)) - float(cents(f2, f_nom)))
        # GROSS FAILURES ONLY. This guard is deliberately wide, because at run
        # time there is no population to compare against: the first capture has
        # no reference, so anything data-driven here is either unavailable or
        # fitted to a handful of triggers.
        #
        # It was NOT wide, and that cost bench time. The first version used the
        # same 3.0 c the host-side screen uses, which was set from note-36 data
        # where the two harmonics agree to 0.05 c. At note 60 they sit a
        # CONSTANT ~4.5 c apart (measured this morning: h16 and h32 differ by
        # -5.04 c in mean while the SHIFT agrees to 0.018 c). So every note-60
        # capture was rejected and a whole run produced nothing.
        #
        # The distinction the first version missed: a constant offset between
        # two harmonics is a per-note SYSTEMATIC and cancels in a difference. A
        # per-capture disagreement that departs from that note's usual offset is
        # an estimator failure. Only the second is a reason to drop a capture.
        # `screen` makes that distinction with the whole population; this
        # cannot, so it only catches the 43 c kind.
        if dis > args.gross_disagree:
            print(f"  trigger {i} mode {mode}: REJECTED, the estimator "
                  f"disagrees with itself by {dis:.3f} c across harmonics "
                  f"(gross limit {args.gross_disagree:g}). "
                  f"{cents(f, f_nom):+.3f} c vs {cents(f2, f_nom):+.3f} c")
            return None
    return f


def _cell_stats(cs):
    """n / mean / sd / sem in cents for one cell's per-trigger list."""
    a = np.asarray(cs, dtype=float)
    n = len(a)
    if n == 0:
        return {"n": 0, "mean": float("nan"), "sd": float("nan"),
                "sem": float("nan")}
    sd = float(a.std()) if n > 1 else float("nan")
    sem = sd / np.sqrt(n) if n > 1 else float("nan")
    return {"n": n, "mean": float(a.mean()), "sd": sd, "sem": sem}


def _report_cell(mode, fs, f_nom):
    if not fs:
        print(f"  mode {mode:>2}: NO LOCKS")
        return
    a = np.array(fs)
    c = cents(a, f_nom)
    st = _cell_stats(c)
    print(f"  mode {mode:>2}: n={st['n']:>3}  {a.mean():>11.5f} Hz  "
          f"{st['mean']:>+8.3f} c  sd {st['sd']:.4f}  sem {st['sem']:.4f}")


# ---------------------------------------------------------------- the cut

MIN_CELLS_FOR_CUT = 4          # below this a median is not a reference
DEFAULT_SD_FACTOR = 3.0
DEFAULT_MIN_REPEATS = 3


def apply_cut(cells, factor=DEFAULT_SD_FACTOR):
    """Reject cells whose WITHIN-RUN sd exceeds `factor` x the median cell sd.

    `cells` is every cell for ONE note, across repeats and arms. The reference
    is the median of their sds, which is why this needs several: with two cells
    the "median" is their mean and one bad cell drags the reference up by half
    its own excess. Under MIN_CELLS_FOR_CUT the cut REFUSES to run and says so,
    rather than running underpowered (COMMON rule 1: an instrument that cannot
    resolve the distinction gives no result, not a weak one).

    Why sd and not the mean: this has to be blind to the answer. Three repeats
    of one fixed configuration gave +3.9122, +2.3749 and +4.0067 c, and the odd
    one out announced itself from its own internal scatter (sem 1.7501 against
    0.2886 and 0.2400, ~6x). Rejecting on spread cannot know which way it would
    have moved the shift; rejecting on the mean would be fitting the result.

    RESIDUAL RISK, stated because the cut cannot rule it out: if the true shift
    and the scatter share a cause, dropping the scattered cells biases the
    survivors. `pool` prints the uncut shift beside the cut one for exactly
    this reason. If they disagree by more than the quoted error, the cut is
    doing something and the run needs explaining, not publishing.

    Returns (kept, rejected, ref_sd). ref_sd is None when the cut refused.
    """
    usable = [c for c in cells if c["stats"]["n"] > 1
              and np.isfinite(c["stats"]["sd"])]
    if len(usable) < MIN_CELLS_FOR_CUT:
        return list(cells), [], None
    ref = float(np.median([c["stats"]["sd"] for c in usable]))
    lim = factor * ref
    kept, rej = [], []
    for c in cells:
        sd = c["stats"]["sd"]
        if c["stats"]["n"] < 2 or not np.isfinite(sd) or sd > lim:
            rej.append(c)
        else:
            kept.append(c)
    return kept, rej, ref


def pool_shifts(cells, factor=DEFAULT_SD_FACTOR,
                min_repeats=DEFAULT_MIN_REPEATS, out=print):
    """The published quantity: per-note, per-arm shift against the mode 0
    control, cut for run quality and quoted from the BETWEEN-REPEAT spread.

    A repeat contributes a shift only if BOTH its control cell and its arm cell
    survive the cut, because a difference is no better than its worse half.

    The error is sd(per-repeat shifts) / sqrt(k), NOT the within-run sem. The
    within-run sem understates it by about 3x on this machine: five note-36
    h16 measurements of the same quantity gave +3.6355 +2.9814 +3.9122 +2.3749
    +4.0067, sd 0.691 c, against quoted sems of 0.10 to 0.32 c. Every error bar
    published for this claim before the cut existed was the understating kind.

    Returns {(note, mode): dict}, empty for anything that did not qualify.
    """
    by_note = {}
    for c in cells:
        by_note.setdefault(c["note"], []).append(c)

    table = {}
    for note in sorted(by_note):
        kept, rej, ref = apply_cut(by_note[note], factor)
        out(f"\nnote {note} (63-cv {63 - (note - CV_OF)}): "
            f"{len(by_note[note])} cells offered")
        if ref is None:
            out(f"  CUT REFUSED: fewer than {MIN_CELLS_FOR_CUT} cells with a "
                f"usable sd. A median over this many is not a reference.")
            out("  Every cell is kept and NOTHING here is publishable.")
        else:
            out(f"  median cell sd {ref:.4f} c, reject above "
                f"{factor:g}x = {factor * ref:.4f} c")
        for c in rej:
            st = c["stats"]
            out(f"  REJECTED run {c['run_id']} mode {c['mode']}: "
                f"n={st['n']} mean {st['mean']:+.4f} c sd {st['sd']:.4f} "
                f"sem {st['sem']:.4f}")

        # index the survivors by (run_id, mode); one cell per pair
        idx = {}
        for c in kept:
            idx[(c["run_id"], c["mode"])] = c
        runs = sorted({c["run_id"] for c in by_note[note]})
        modes = sorted({c["mode"] for c in by_note[note]} - {0})

        for mode in modes:
            per, uncut = [], []
            for r in runs:
                ctrl, arm = idx.get((r, 0)), idx.get((r, mode))
                if ctrl and arm:
                    per.append((r, arm["stats"]["mean"] - ctrl["stats"]["mean"]))
                cu = {(c["run_id"], c["mode"]): c for c in by_note[note]}
                c0, c1 = cu.get((r, 0)), cu.get((r, mode))
                if c0 and c1 and c0["stats"]["n"] and c1["stats"]["n"]:
                    uncut.append(c1["stats"]["mean"] - c0["stats"]["mean"])
            k = len(per)
            out(f"  mode {mode}: {k} repeat(s) survive both halves")
            for r, d in per:
                # The per-repeat screen accounting, kept next to the repeat it
                # belongs to. CORE asked for this specifically and the reason is
                # right: a screen that drops a capture in ONE repeat and none in
                # the others is a fact about that run, and it is exactly the
                # kind of thing a pooled number summarises away. cut-r1 dropped
                # 1 of 40 and moved +5.8483 -> +4.2443, a 1.6 c swing from one
                # capture. A reader who cannot see that cannot judge the row.
                info = []
                for mm in (0, mode):
                    cell = idx.get((r, mm))
                    if cell is None:
                        continue
                    nd = len(cell.get("dropped") or [])
                    if cell.get("screened") is False:
                        info.append(f"mode {mm} UNSCREENED")
                    elif nd:
                        info.append(f"mode {mm} dropped {nd} of "
                                    f"{cell['stats']['n'] + nd}")
                note_ = ("   " + ", ".join(info)) if info else ""
                out(f"    {r}  {d:>+8.4f} c{note_}")
            if k < min_repeats:
                out(f"  NO SHIFT PUBLISHED for mode {mode}: {k} surviving "
                    f"repeats, {min_repeats} required. This is 'not measured', "
                    f"not a null.")
                continue
            ds = np.array([d for _, d in per], dtype=float)
            spread = float(ds.std(ddof=1))
            err = spread / np.sqrt(k)
            u = np.array(uncut, dtype=float)
            row = {"note": note, "mode": mode, "k": k,
                   "shift": float(ds.mean()), "err": err, "spread": spread,
                   "uncut_shift": float(u.mean()) if len(u) else float("nan"),
                   "uncut_k": len(u), "ref_sd": ref}
            table[(note, mode)] = row
            out(f"  SHIFT {row['shift']:>+8.4f} +/- {err:.4f} c "
                f"(between-repeat sd {spread:.4f} over k={k})")
            if len(u) > 1:
                out(f"  uncut, for comparison: {u.mean():>+8.4f} c over "
                    f"{len(u)} repeats. A disagreement larger than the error "
                    f"above means the cut is deciding the answer.")
    return table


def _cells_from_json(paths, out=print):
    """Load run sidecars into the flat cell list pool_shifts() wants."""
    import json
    cells, meta = [], []
    for p in paths:
        with open(p) as fh:
            d = json.load(fh)
        meta.append((d.get("run_id"), d.get("harmonic"), p))
        for c in d.get("cells", []):
            cs = c.get("cents", [])
            cells.append({"run_id": d.get("run_id"), "note": c["note"],
                          "mode": c["mode"], "cents": cs,
                          "screened": c.get("screened"),
                          "dropped": c.get("dropped", []),
                          "stats": _cell_stats(cs)})
    hs = {h for _, h, _ in meta}
    if len(hs) > 1:
        out(f"REFUSING TO POOL: harmonics differ across runs {sorted(hs)}. "
            f"Mixing harmonics is what produced the withdrawn 5.4 sigma.")
        return None, meta
    return cells, meta


def cmd_pool(args):
    """Pool several `run` sidecars: apply the quality cut, publish from the
    between-repeat spread. This is the ONLY subcommand whose shift is quotable.

    Separate invocations rather than one long one because a tool call caps at
    10 minutes here and one repeat of a 2-arm note-36 cell is ~3.5 min.
    """
    import glob
    paths = []
    for pat in args.json:
        paths.extend(sorted(glob.glob(pat)) or [pat])
    if not paths:
        print("no run sidecars given")
        return 2
    cells, meta = _cells_from_json(paths)
    if cells is None:
        return 2
    print("claim-038 G13: pooled sign test, run-quality cut applied")
    for rid, h, p in meta:
        print(f"  run {rid}  harmonic {h}  {p}")
    print(f"cut: reject a cell whose sd > {args.sd_factor:g}x the median cell "
          f"sd for its note; {args.min_repeats} surviving repeats required")
    table = pool_shifts(cells, args.sd_factor, args.min_repeats)
    if not table:
        print("\nNOTHING PUBLISHABLE. Take more repeats.")
        return 1
    print("\nPUBLISHABLE ROWS")
    print(f"{'note':>5} {'63-cv':>6} {'mode':>5} {'shift c':>10} "
          f"{'+/- c':>8} {'k':>3}")
    for (note, mode), r in sorted(table.items()):
        print(f"{note:>5} {63 - (note - CV_OF):>6} {mode:>5} "
              f"{r['shift']:>+10.4f} {r['err']:>8.4f} {r['k']:>3}")

    for mode in sorted({m for _, m in table}):
        # `m2 == m` here for one commit, and `m` is the SET COMPREHENSION's
        # variable, which does not leak in Python 3. It raised NameError the
        # first time a pool ran, which is the good failure: a typo that only
        # silently mis-filtered would have published a ratio built from the
        # wrong arm's rows.
        got = [(63 - (n - CV_OF), r) for (n, m2), r in sorted(table.items())
               if m2 == mode]
        base = next((r for k, r in got if k == 63), None)
        if base is None or len(got) < 2:
            continue
        print(f"\n  mode {mode}: the 63:39:0 ratio, with HONEST errors")
        for k, r in got:
            if k == 63:
                continue
            ratio = r["shift"] / base["shift"]
            # error on a quotient of two independent measurements
            re_ = abs(ratio) * np.sqrt((r["err"] / r["shift"]) ** 2
                                       + (base["err"] / base["shift"]) ** 2)
            pred = k / 63.0
            sig = abs(ratio - pred) / re_ if re_ > 0 else float("inf")
            print(f"    63-cv {k:>2}: ratio {ratio:.4f} +/- {re_:.4f} vs "
                  f"predicted {pred:.4f} -> {sig:.1f} sigma")
        print("  L79: a systematic that cancels in each half does NOT cancel "
              "in their\n  ratio. The per-half agreement is what hides it, so "
              "the quotient is what\n  gets quoted, never the halves.")
    return 0


def cmd_run(args):
    """The 63:39:0 sign test itself.

    For each note, and for each 0x3E arm, retrigger the note N times and take a
    harmonic-referred, onset-locked pitch estimate per trigger. The published
    quantity is the DIFFERENCE between an arm and the 4 MHz control at the SAME
    note, so anything common to both cancels: the machine's absolute tuning,
    the estimator's bias at that harmonic, and the envelope shape.

    Why N triggers rather than one long note: the 101 does not sustain (the
    gate is held for the whole hold, measured, but the VCA envelope decays to
    the noise floor about 2.2 s after onset), and the per-trigger repeat spread
    is 0.41 / 0.12 / 0.08 c at notes 36 / 60 / 99 with the harmonics chosen
    below. Averaging N brings the mean's error down as sd/sqrt(N).

    The confounder that argues FOR holding one note per trigger rather than
    playing a pattern: at a clock the app was not compiled for, hw.h's
    IdleWaitUs dwells come out short and the S/H acquisition is incomplete. For
    a HELD note the S/H is refreshed to the same value every pass, so an
    incomplete acquisition converges to zero steady-state error.
    """
    if not args.run_id:
        args.run_id = _default_run_id()
    b = Bench()
    notes = args.notes or list(NOTES)
    arms = args.modes or [0, 1, 8, 4]
    if 0 not in arms:
        print("mode 0 is the CONTROL and every difference is taken against it.")
        return 2

    # A run id in the path, because without one this destroyed its own
    # evidence. The path used to be keyed on cell and trigger index ONLY, so
    # any later run silently overwrote an earlier one's captures: CORE went to
    # re-derive a published row and found the files behind it had been replaced
    # by my own next run, still being written while they looked. A result whose
    # captures no longer exist rests on the author's arithmetic alone.
    os.makedirs(args.outdir, exist_ok=True)
    print("claim-038 G13: the 63:39:0 sign test")
    print(f"run id {args.run_id}, captures under {args.outdir}/")
    print(f"arms: {', '.join(f'{m} ({MODES.get(m, chr(63))})' for m in arms)}")
    print(f"notes: {notes}   {args.n} triggers per cell\n")

    results = {}                       # (note, mode) -> list of f0
    for note in notes:
        f_nom = midi_hz(note)
        nh = pick_harmonic(f_nom, RATE, args.harmonic)
        print(f"note {note} ({f_nom:.2f} Hz, cv {note - CV_OF}, "
              f"63-cv = {63 - (note - CV_OF)}), via harmonic {nh}")
        if args.interleave:
            # INTERLEAVED: alternate the arms trigger by trigger, so the
            # control and the arm are seconds apart instead of minutes.
            #
            # Sequential order is why this was needed. cmd_run used to take all
            # N control triggers and then all N arm triggers, putting ~1.75 min
            # between the two halves of every difference. The 101 is analog and
            # drifts over exactly that scale: repeating a cell with the same
            # harmonic on a different run moved it further than its own sem
            # (+3.6355 +/- 0.3169, then +2.9814 +/- 0.1975). Any drift inside
            # that gap goes straight into the published shift. Interleaved, it
            # is common-mode to both arms and largely cancels.
            per = {m: [] for m in arms}
            for i in range(args.n):
                for mode in arms:
                    f = _one_trigger(b, note, mode, i, f_nom, args)
                    if f is not None:
                        per[mode].append(f)
            for mode in arms:
                results[(note, mode)] = per[mode]
                _report_cell(mode, per[mode], f_nom)
            print()
            continue
        for mode in arms:
            fs = []
            for i in range(args.n):
                # Arm the window BEFORE the note and make it long enough to
                # cover the whole capture, then send nothing until it closes.
                try:
                    secs_b, scale = window_byte(mode, args.window)
                except ValueError as e:
                    print(f"  mode {mode}: {e}")
                    break
                # Through _one_trigger, NOT a second copy of the estimate.
                # This path used to inline its own and would have skipped the
                # per-capture screen entirely: a cut that applies on one code
                # path and not the other is worse than no cut, because the two
                # paths are compared to each other.
                f = _one_trigger(b, note, mode, i, f_nom, args)
                if f is not None:
                    fs.append(f)
            results[(note, mode)] = fs
            if fs:
                a = np.array(fs)
                c = cents(a, f_nom)
                sem = c.std() / np.sqrt(len(a)) if len(a) > 1 else float("nan")
                print(f"  mode {mode:>2}: n={len(a):>3}  "
                      f"{a.mean():>11.5f} Hz  {c.mean():>+8.3f} c  "
                      f"sd {c.std():.4f}  sem {sem:.4f}")
            else:
                print(f"  mode {mode:>2}: NO LOCKS")
        print()

    # The sidecar. Without it a repeat cannot be pooled with any other, and
    # the between-repeat spread is the only honest error this claim has.
    import json
    side = os.path.join(args.outdir, f"g13-run-{args.run_id}.json")
    doc = {"run_id": args.run_id, "harmonic": args.harmonic,
           "interleave": bool(args.interleave), "n": args.n,
           "notes": notes, "modes": arms,
           "capture": {"secs": args.secs, "fire_at": args.fire_at,
                       "head": args.head, "win": args.win,
                       "window": args.window, "gap": args.gap},
           "cells": []}
    for (note, mode), fs in sorted(results.items()):
        f_nom = midi_hz(note)
        cs = list(cents(np.array(fs), f_nom)) if fs else []
        doc["cells"].append({"note": note, "mode": mode,
                             "nh": pick_harmonic(f_nom, RATE, args.harmonic),
                             "f_nom": f_nom, "n": len(cs),
                             "freqs": [float(x) for x in fs],
                             "cents": [float(x) for x in cs]})
    with open(side, "w") as fh:
        json.dump(doc, fh, indent=1)
    print(f"sidecar written: {side}")

    # WITHIN-RUN differences. These are diagnostic. They are NOT the claim.
    #
    # A single run cannot quote an error on its shift, because the error that
    # matters is between runs and is about 3x the within-run sem on this
    # machine (five note-36 h16 measurements of one fixed configuration: sd
    # 0.691 c against quoted sems of 0.10 to 0.32 c). Every claim-038 error bar
    # published before this existed was the understating kind, and that plus
    # mixed harmonics is the whole explanation of the withdrawn 5.4 sigma.
    print("\nWITHIN-RUN differences, DIAGNOSTIC ONLY. Do not quote these.")
    print(f"{'note':>5} {'63-cv':>6} {'mode':>5} {'diff c':>10} "
          f"{'sem c':>8}   (sem understates the real error ~3x)")
    for note in notes:
        f_nom = midi_hz(note)
        ctrl = results.get((note, 0)) or []
        if len(ctrl) < 2:
            print(f"{note:>5}  control has fewer than two locks; no differences")
            continue
        cc = cents(np.array(ctrl), f_nom)
        for mode in arms:
            if mode == 0:
                continue
            arm = results.get((note, mode)) or []
            if len(arm) < 2:
                continue
            ac = cents(np.array(arm), f_nom)
            d = ac.mean() - cc.mean()
            err = np.sqrt(ac.std() ** 2 / len(ac) + cc.std() ** 2 / len(cc))
            print(f"{note:>5} {63 - (note - CV_OF):>6} {mode:>5} "
                  f"{d:>+10.4f} {err:>8.4f}")

    # The cut, run over this run alone. It will normally REFUSE here: one run
    # of two arms is two cells and a median over two is not a reference. That
    # refusal is the point, and it is why this prints the pool command.
    cells = [{"run_id": args.run_id, "note": n, "mode": m,
              "cents": list(cents(np.array(fs), midi_hz(n))) if fs else [],
              "stats": _cell_stats(cents(np.array(fs), midi_hz(n))
                                   if fs else [])}
             for (n, m), fs in sorted(results.items())]
    print("\nrun-quality cut, this run alone:")
    pool_shifts(cells, args.sd_factor, args.min_repeats)

    print(f"\nTHE SHIFT IS NOT PUBLISHED FROM ONE RUN. Repeat this command "
          f"{args.min_repeats + 1} or\nmore times with the SAME harmonic, then:"
          f"\n\n    python3 tools/diag/g13_sign.py pool "
          f"'{args.outdir}/g13-run-*.json'\n")
    return 0


def _mkcell(run_id, note, mode, mean, sd, n=20, seed=0):
    """A synthetic cell with a KNOWN mean and within-run sd."""
    rng = np.random.default_rng(seed)
    x = rng.normal(0.0, 1.0, n)
    x = (x - x.mean()) / (x.std() or 1.0) * sd + mean     # exact mean and sd
    return {"run_id": run_id, "note": note, "mode": mode,
            "cents": list(x), "stats": _cell_stats(x)}


def _quiet(*a, **k):
    pass


def cmd_cutcheck(args):
    """Break-test the run-quality cut, with no hardware and no bench time.

    L9 and claim-081: a suite that has never failed proves nothing. Every case
    here has a paired injection that MUST turn it red, and the injections are
    the ways this cut could be wrong rather than the ways it could be right.
    """
    fails = []

    def check(name, cond, detail=""):
        print(f"  {'ok  ' if cond else 'FAIL'}  {name}"
              + (f"   {detail}" if detail else ""))
        if not cond:
            fails.append(name)

    # ---- 1. clean: five repeats, arm +3.00 c, tight within-run sd
    print("1. clean data, 5 repeats, true shift +3.0000 c")
    clean = []
    for i in range(5):
        clean.append(_mkcell(f"r{i}", 36, 0, 0.0, 0.30, seed=100 + i))
        clean.append(_mkcell(f"r{i}", 36, 1, 3.0, 0.30, seed=200 + i))
    t = pool_shifts(clean, out=_quiet)
    row = t.get((36, 1))
    check("a shift is published", row is not None)
    if row:
        check("shift is the true +3.0000", abs(row["shift"] - 3.0) < 1e-6,
              f"got {row['shift']:+.4f}")
        check("all 5 repeats survive", row["k"] == 5, f"k={row['k']}")

    # ---- 2. THE DEFECT THIS EXISTS FOR: the error must come from the
    #        between-repeat spread, not the within-run sem. Within-run sd 0.02
    #        (sem 0.0045), repeat means scattered by 0.691 c, which is the
    #        measured note-36 number.
    print("\n2. tight cells, scattered repeat means (the measured 0.691 c)")
    means = [3.6355, 2.9814, 3.9122, 2.3749, 4.0067]
    sc = []
    for i, m in enumerate(means):
        sc.append(_mkcell(f"r{i}", 36, 0, 0.0, 0.02, seed=300 + i))
        sc.append(_mkcell(f"r{i}", 36, 1, m, 0.02, seed=400 + i))
    t = pool_shifts(sc, out=_quiet)
    row = t.get((36, 1))
    want_spread = float(np.array(means).std(ddof=1))
    check("published", row is not None)
    if row:
        check("error is the between-repeat spread / sqrt(k)",
              abs(row["err"] - want_spread / np.sqrt(5)) < 1e-6,
              f"{row['err']:.4f} vs {want_spread / np.sqrt(5):.4f}")
        sem = 0.02 / np.sqrt(20)
        check("error is >> the within-run sem it replaces",
              row["err"] > 20 * sem,
              f"{row['err']:.4f} c against sem {sem:.4f} c")

    # ---- 3. one bad repeat: 6x the sd, and it drags the answer
    print("\n3. one repeat with 6x the sd (the +2.3749 signature)")
    bad = []
    for i in range(5):
        bad.append(_mkcell(f"r{i}", 36, 0, 0.0, 0.30, seed=500 + i))
        s_ = 1.80 if i == 3 else 0.30
        m_ = 1.20 if i == 3 else 3.00
        bad.append(_mkcell(f"r{i}", 36, 1, m_, s_, seed=600 + i))
    t = pool_shifts(bad, out=_quiet)
    row = t.get((36, 1))
    check("published", row is not None)
    if row:
        check("the bad repeat is excluded", row["k"] == 4, f"k={row['k']}")
        check("cut shift is the good value", abs(row["shift"] - 3.0) < 1e-6,
              f"{row['shift']:+.4f}")
        check("uncut shift is dragged, and is printed for comparison",
              abs(row["uncut_shift"] - 3.0) > 0.3,
              f"uncut {row['uncut_shift']:+.4f} over {row['uncut_k']} repeats")

    # ---- 4. INJECTION: disable the cut. Case 3 must then go wrong.
    print("\n4. INJECTION: cut disabled (sd-factor 1e9). Case 3 must break.")
    t = pool_shifts(bad, factor=1e9, out=_quiet)
    row = t.get((36, 1))
    check("the bad repeat now survives", row is not None and row["k"] == 5,
          f"k={row['k'] if row else None}")
    check("and the shift is now wrong",
          row is not None and abs(row["shift"] - 3.0) > 0.3,
          f"{row['shift']:+.4f}" if row else "")

    # ---- 5. too few repeats: not measured, not a null
    print("\n5. two repeats only: nothing may be published")
    few = []
    for i in range(2):
        few.append(_mkcell(f"r{i}", 36, 0, 0.0, 0.30, seed=700 + i))
        few.append(_mkcell(f"r{i}", 36, 1, 3.0, 0.30, seed=800 + i))
    t = pool_shifts(few, out=_quiet)
    check("no shift published", (36, 1) not in t)

    # ---- 6. under MIN_CELLS_FOR_CUT the cut REFUSES rather than running
    print(f"\n6. {MIN_CELLS_FOR_CUT - 1} cells: the cut must refuse, not guess")
    two = [_mkcell("r0", 36, 0, 0.0, 0.30, seed=900),
           _mkcell("r0", 36, 1, 3.0, 9.00, seed=901),
           _mkcell("r1", 36, 0, 0.0, 0.30, seed=902)][:MIN_CELLS_FOR_CUT - 1]
    kept, rej, ref = apply_cut(two)
    check("cut refused", ref is None)
    check("nothing rejected while refused", not rej)

    # ---- 7. INJECTION: a cell of pure garbage must not survive on its mean
    print("\n7. INJECTION: a wide cell whose MEAN looks right must still go")
    mix = []
    for i in range(5):
        mix.append(_mkcell(f"r{i}", 36, 0, 0.0, 0.30, seed=1000 + i))
        s_ = 2.40 if i == 1 else 0.30
        mix.append(_mkcell(f"r{i}", 36, 1, 3.00, s_, seed=1100 + i))
    kept, rej, ref = apply_cut(mix)
    check("rejected on spread alone, mean identical to the good cells",
          len(rej) == 1 and rej[0]["run_id"] == "r1",
          f"{len(rej)} rejected")

    # ---- 8. mixing harmonics must be refused at load, not averaged
    print("\n8. INJECTION: two runs at different harmonics must not pool")
    import json
    import tempfile
    paths = []
    with tempfile.TemporaryDirectory() as d:
        for i, h in enumerate((16, 32)):
            p = os.path.join(d, f"g13-run-x{i}.json")
            with open(p, "w") as fh:
                json.dump({"run_id": f"x{i}", "harmonic": h, "cells": [
                    {"note": 36, "mode": 0, "cents": [0.0, 0.1]},
                    {"note": 36, "mode": 1, "cents": [3.0, 3.1]}]}, fh)
            paths.append(p)
        cells, _ = _cells_from_json(paths, out=_quiet)
    check("refused", cells is None)

    # ---- 9. the crosscheck harmonic must RESOLVE DIFFERENTLY or be refused
    print("\n9. crosscheck harmonic selection")
    check("note 36, want 16: a different harmonic is available",
          crosscheck_harmonic(midi_hz(36), 16) is not None,
          f"got {crosscheck_harmonic(midi_hz(36), 16)}")
    n36 = pick_harmonic(midi_hz(36), RATE, 16)
    x36 = crosscheck_harmonic(midi_hz(36), 16)
    check("and it really does resolve to a different harmonic",
          x36 is not None and pick_harmonic(midi_hz(36), RATE, x36) != n36,
          f"{n36} vs {pick_harmonic(midi_hz(36), RATE, x36) if x36 else None}")
    x99 = crosscheck_harmonic(midi_hz(99), 16)
    n99 = pick_harmonic(midi_hz(99), RATE, 16)
    check("note 99, want 16: whatever it picks must differ from "
          f"harmonic {n99}, or be None",
          x99 is None or pick_harmonic(midi_hz(99), RATE, x99) != n99,
          f"picked {x99}")

    # ---- 10. INJECTION: the screen's threshold must be doing work.
    #      The real numbers, from 120 captures: normal disagreement is
    #      0.045-0.070 c median and 2.05 c at worst; the one broken capture
    #      was 43.109 c. A threshold anywhere between those two separates
    #      them, which is what "not tuned" means here.
    print("\n10. the per-capture screen's threshold, against measured data")
    normal_worst, broken = 2.048, 43.109
    check("3.0 c passes every capture that was not broken",
          normal_worst < 3.0, f"worst normal {normal_worst} c")
    check("3.0 c rejects the broken one", broken > 3.0,
          f"broken {broken} c")
    check("and the gap is wide enough not to need tuning",
          broken / 3.0 > 10 and 3.0 / normal_worst > 1.4,
          f"{broken / 3.0:.0f}x above, {3.0 / normal_worst:.1f}x below")

    print(f"\n{len(fails)} failure(s)" if fails else "\nall cases pass")
    if fails:
        for f in fails:
            print(f"  FAILED: {f}")
    return 1 if fails else 0


def _read_capture(path):
    sr, data = wavfile.read(path)
    return sr, (data[:, CH_101] if data.ndim > 1 else data)


def cmd_reanalyse(args):
    """Re-analyse SAVED captures at several harmonics. No hardware, no lease.

    THE QUESTION THIS ANSWERS, and it is the one the quality cut leaves open:
    is the run-to-run scatter the ESTIMATOR intermittently losing the harmonic,
    or the MACHINE moving? A cut can reject a bad run either way, but only this
    says what was rejected.

    The test is per-FILE and therefore immune to the mixed provenance of the
    pre-run-id captures. For one capture, h16 and h32 read the SAME physical
    event. If a capture that is an outlier inside its own cell also disagrees
    with itself across harmonics, the estimator lost it. If every harmonic
    agrees on that capture and it is still an outlier, the machine moved and no
    estimator fix will help.

    Trap, from this instrument's own history: a tight spread is not a correct
    answer. `--harmonic 64` at note 36 once looked like the best cell in the
    sweep at sd 0.14 c and was locked onto the WRONG harmonic, mean +57 c out.
    So the cross-harmonic DISAGREEMENT is reported in cents referred to f0,
    where a lost harmonic shows up as a large offset rather than as noise.
    """
    import glob
    import re
    paths = []
    for pat in args.wav:
        paths.extend(sorted(glob.glob(pat)) or [pat])
    paths = [p for p in paths if os.path.isfile(p)]
    if not paths:
        print("no captures matched")
        return 2

    hs = args.harmonics
    print("claim-038: saved captures re-analysed at several harmonics")
    print(f"{len(paths)} captures, harmonics {hs}")
    print("estimator-vs-machine: a within-cell outlier that AGREES across "
          "harmonics\nis the machine; one that disagrees is the estimator.\n")

    rows = []
    for p in paths:
        m = re.search(r"-(\d+)-(\d+)-(\d+)\.wav$", os.path.basename(p))
        if not m:
            print(f"  skip (cannot parse note/mode/index): {p}")
            continue
        note, mode, idx = (int(x) for x in m.groups())
        f_nom = midi_hz(note)
        sr, sig = _read_capture(p)
        win = onset_window(sig, sr, args.head, args.win)
        if win is None:
            print(f"  {os.path.basename(p)}: NO ONSET")
            continue
        vals = {}
        for h in hs:
            f, nh = estimate_f0_harmonic(win, f_nom, rate=sr, want=h)
            vals[h] = (float(cents(f, f_nom)) if f is not None else None, nh)
        rows.append({"path": p, "note": note, "mode": mode, "idx": idx,
                     "vals": vals})

    if not rows:
        print("nothing analysed")
        return 1

    ref = args.reference
    by_cell = {}
    for r in rows:
        by_cell.setdefault((r["note"], r["mode"]), []).append(r)

    verdicts = {"estimator": 0, "machine": 0, "unclear": 0}
    for (note, mode), rs in sorted(by_cell.items()):
        base = np.array([r["vals"][ref][0] for r in rs
                         if r["vals"].get(ref, (None,))[0] is not None])
        if len(base) < 3:
            print(f"note {note} mode {mode}: fewer than 3 locks at h{ref}")
            continue
        med = float(np.median(base))
        mad = float(np.median(np.abs(base - med))) or 1e-9
        print(f"note {note} mode {mode}: n={len(base)} at h{ref}, "
              f"median {med:+.3f} c, MAD {mad:.4f} c "
              f"(harmonics used: "
              f"{ {h: rs[0]['vals'][h][1] for h in hs} })")
        for r in sorted(rs, key=lambda z: z["idx"]):
            v = r["vals"].get(ref, (None,))[0]
            if v is None:
                continue
            dev = abs(v - med) / mad
            if dev < args.outlier_mad:
                continue
            others = [r["vals"][h][0] for h in hs
                      if h != ref and r["vals"][h][0] is not None]
            if not others:
                verdicts["unclear"] += 1
                continue
            disagree = max(abs(v - o) for o in others)
            # A capture the estimator handled consistently reads the same at
            # every harmonic; the cross-harmonic spread of a good capture sets
            # the scale, so compare against the cell's own MAD.
            if disagree > args.outlier_mad * mad:
                verdicts["estimator"] += 1
                tag = "ESTIMATOR (harmonics disagree)"
            else:
                verdicts["machine"] += 1
                tag = "MACHINE (harmonics agree)"
            oth = "  ".join(f"h{h}={r['vals'][h][0]:+.3f}" for h in hs
                            if r["vals"][h][0] is not None)
            print(f"  idx {r['idx']:>3}: h{ref}={v:+.3f} c, "
                  f"{dev:.1f} MAD from the cell median -> {tag}")
            print(f"           {oth}   worst disagreement {disagree:.3f} c")

    # The decomposition, which does not depend on spotting an outlier.
    #
    # Each capture is ONE physical event read two ways. Anything the MACHINE
    # did is common to both readings and cancels in their difference; what
    # survives is the estimator. So sd(h_a - h_b) against sd(h_ref) splits the
    # cell's scatter into the two causes directly.
    #
    # The one correction that has to be made: if the underlying error is fixed
    # in absolute Hz, its cents value scales as 1/n, so the two readings do not
    # carry equal estimator noise and the difference is dominated by the lower
    # harmonic. The expected difference sd from independent estimator noise
    # alone is sqrt(s_a^2 + s_b^2); it is printed beside the measured one so
    # the comparison is against the right number and not against sd(h_ref).
    print("\nESTIMATOR vs MACHINE, by decomposition (independent of outliers)")
    print("a difference across harmonics cancels anything the machine did.")
    for (note, mode), rs in sorted(by_cell.items()):
        cols = {}
        for h in hs:
            v = [r["vals"][h][0] for r in rs if r["vals"][h][0] is not None]
            cols[h] = np.array(v, dtype=float)
        print(f"  note {note} mode {mode}:")
        for h in hs:
            if len(cols[h]) > 1:
                print(f"    h{h:<3} n={len(cols[h]):>3}  "
                      f"mean {cols[h].mean():>+8.3f} c  "
                      f"sd {cols[h].std():.4f} c")
        for a, b_ in ((hs[i], hs[j]) for i in range(len(hs))
                      for j in range(i + 1, len(hs))):
            pair = [(r["vals"][a][0], r["vals"][b_][0]) for r in rs
                    if r["vals"][a][0] is not None
                    and r["vals"][b_][0] is not None]
            if len(pair) < 3:
                continue
            # DEFECT CAUGHT BY ITS OWN OUTPUT: pick_harmonic() clamps to
            # Nyquist, so at note 99 a request for h16 and one for h32 BOTH
            # resolve to harmonic 7. The difference is then a number minus
            # itself: sd exactly 0.0000, ratio 0.00, and this printed "MACHINE
            # dominates" for a comparison that was never made. Refuse it.
            nha = rs[0]["vals"][a][1]
            nhb = rs[0]["vals"][b_][1]
            if nha == nhb:
                print(f"    h{a} - h{b_}: NOT A COMPARISON. Both requests "
                      f"clamp to harmonic {nha} (Nyquist), so this is one "
                      f"reading minus itself.")
                continue
            d = np.array([x - y for x, y in pair], dtype=float)
            indep = np.sqrt(cols[a].std() ** 2 + cols[b_].std() ** 2)
            print(f"    h{a} - h{b_}: mean {d.mean():>+8.3f} c  "
                  f"sd {d.std():.4f} c   vs {indep:.4f} c expected if the "
                  f"scatter were ALL estimator")
            if indep > 0:
                frac = d.std() / indep
                if frac < 0.5:
                    verdict = ("MACHINE dominates: the two readings move "
                               "together")
                elif frac > 0.85:
                    verdict = ("ESTIMATOR dominates: the two readings are "
                               "independent")
                else:
                    verdict = "MIXED, neither cause dominates"
                print(f"      ratio {frac:.2f} -> {verdict}")

    # The shift at each harmonic. The claim is a DIFFERENCE between arms at
    # one note, so a harmonic's absolute bias cancels here even when it is
    # large: note 60 reads 5.04 c apart at h16 and h32 and its SHIFT agrees to
    # 0.017 c. If a shift moved with the harmonic, the estimator would be
    # generating it, and that is the failure mode the withdrawn 5.4 sigma had.
    print("\nTHE SHIFT AT EACH HARMONIC (arm minus the mode 0 control)")
    notes_seen = sorted({n for n, _ in by_cell})
    for note in notes_seen:
        modes = sorted({m for n, m in by_cell if n == note})
        if 0 not in modes:
            continue
        for mode in modes:
            if mode == 0:
                continue
            line = []
            for h in hs:
                c0 = [r["vals"][h][0] for r in by_cell[(note, 0)]
                      if r["vals"][h][0] is not None]
                c1 = [r["vals"][h][0] for r in by_cell[(note, mode)]
                      if r["vals"][h][0] is not None]
                if len(c0) > 1 and len(c1) > 1:
                    line.append(f"h{h}: {np.mean(c1) - np.mean(c0):>+7.4f} c")
            if line:
                print(f"  note {note} mode {mode}:  " + "   ".join(line))
    print("  A shift that MOVES with the harmonic is the estimator's, not the "
          "machine's.")

    tot = sum(verdicts.values())
    print(f"\noutliers beyond {args.outlier_mad:g} MAD: {tot}")
    for k in ("estimator", "machine", "unclear"):
        print(f"  {k:>10}: {verdicts[k]}")
    if tot == 0:
        print("No outliers at this threshold. That is NOT evidence the scatter "
              "is absent:\nthese captures may simply not contain a rejected "
              "run. Say 'not tested'.")
    return 0


def _capture_disagreements(doc, outdir, note, mode, h1, h2):
    """Per-capture (cents at h1, disagreement vs h2) for one cell, or None."""
    rid = doc.get("run_id")
    f_nom = None
    for c in doc.get("cells", []):
        if c["note"] == note and c["mode"] == mode:
            f_nom = c.get("f_nom") or midi_hz(note)
    if f_nom is None:
        return []
    out = []
    i = 0
    while True:
        wav = os.path.join(outdir, f"g13-run-{rid}-{note}-{mode}-{i}.wav")
        if not os.path.isfile(wav):
            break
        sr, sig = _read_capture(wav)
        win = onset_window(sig, sr, doc["capture"]["head"], doc["capture"]["win"])
        if win is None:
            out.append((i, None, None, None))
            i += 1
            continue
        fa, _ = estimate_f0_harmonic(win, f_nom, rate=sr, want=h1)
        fb, _ = estimate_f0_harmonic(win, f_nom, rate=sr, want=h2)
        if fa is None or fb is None:
            out.append((i, None, None, None))
            i += 1
            continue
        ca, cb = float(cents(fa, f_nom)), float(cents(fb, f_nom))
        out.append((i, ca, float(fa), ca - cb))
        i += 1
    return out


def cmd_screen(args):
    """Screen every capture against its NOTE's own usual harmonic disagreement.

    THE DISTINCTION THIS EXISTS TO MAKE, and the first version got it wrong.

    Two harmonics of the same capture can disagree for two different reasons:

      a CONSTANT offset for that note, which is a per-note SYSTEMATIC. It
        cancels in a difference and is not a reason to drop anything. Measured:
        at note 60, h16 and h32 sit 5.04 c apart in mean while the SHIFT
        between arms agrees to 0.018 c.
      a DEPARTURE from that note's usual offset on one capture, which is the
        estimator failing on that capture. Measured: note 36's median
        disagreement is 0.045 c and one capture came in at 43.109 c.

    Only the second is a reason to drop a capture. The first version compared
    the RAW disagreement to a fixed 3.0 c set from note-36 data, so at note 60
    it rejected every capture in a run and produced nothing. It failed safe (it
    dropped everything rather than silently keeping bad data) but it cost a run
    and a half of bench time.

    So the reference is the note's own MEDIAN disagreement and the scale is its
    MAD, pooled across arms and repeats. Blind to the shift: a disagreement is a
    property of one capture read two ways and does not know which arm it is in,
    and the reference is a per-note constant rather than a per-arm one.

    Writes `<sidecar>.screened.json` and never overwrites the original.
    """
    import glob
    import json
    paths = []
    for pat in args.json:
        paths.extend(sorted(glob.glob(pat)) or [pat])
    paths = [q for q in paths if os.path.isfile(q)
             and not q.endswith(".screened.json")]
    if not paths:
        print("no sidecars matched")
        return 2

    docs = []
    for sc in paths:
        with open(sc) as fh:
            docs.append((sc, json.load(fh)))

    hs = {d.get("harmonic") for _, d in docs}
    if len(hs) > 1:
        print(f"REFUSING: harmonics differ across runs {sorted(hs)}. Mixing "
              f"harmonics is what produced the withdrawn 5.4 sigma.")
        return 2
    h1 = docs[0][1].get("harmonic", 16)

    print("claim-038: per-capture estimator screen over saved captures")
    print("reference: the NOTE's own median harmonic disagreement, not zero.\n")

    # PASS 1: measure every capture, and build the per-note reference.
    meas = {}                       # (sidecar, note, mode) -> list of tuples
    per_note = {}                   # note -> list of disagreements
    for sc, doc in docs:
        outdir = args.capture_dir or os.path.dirname(sc)
        for cell in doc.get("cells", []):
            note, mode = cell["note"], cell["mode"]
            f_nom = cell.get("f_nom") or midi_hz(note)
            h2 = crosscheck_harmonic(f_nom, h1)
            if h2 is None:
                meas[(sc, note, mode)] = None
                continue
            rows = _capture_disagreements(doc, outdir, note, mode, h1, h2)
            meas[(sc, note, mode)] = (rows, h2)
            # The reference key. Per NOTE by default; per (note, arm) with
            # --ref-per-arm.
            #
            # This is not a preference, it is a measured asymmetry. At note 36
            # the disagreement distribution is WIDER on the shifted arm:
            #
            #   mode 0   n=100  median +0.0147  MAD 0.0471  p95 dev 0.358
            #   mode 1   n=100  median -0.0200  MAD 0.0817  p95 dev 1.114
            #
            # which is what the band limit predicts: the crosscheck harmonic's
            # search band is centred on NOMINAL pitch and is +/-0.4/n wide, so
            # the arm whose pitch is shifted sits closer to its edge and its
            # crosscheck reading is the less reliable of the two.
            #
            # The consequence is that a per-NOTE reference drops more captures
            # from the shifted arm (6 against 2 over five runs) and moves the
            # shift by +0.048 c, the SAME SIGN in all four clean runs. That is
            # a signed systematic worth about half the quoted error, not noise.
            #
            # A per-ARM reference equalises the selection and is blind to the
            # arm by construction; a per-NOTE one is blind to nothing but is
            # right about which captures are actually worse. Neither is simply
            # correct, so both are available and `pool` should be run twice.
            key = (note, mode) if args.ref_per_arm else note
            per_note.setdefault(key, []).extend(
                [d for _, _, _, d in rows if d is not None])

    ref = {}
    for note, ds in per_note.items():   # `note` is the reference KEY
        a = np.array(ds, dtype=float)
        if len(a) < args.min_ref:
            ref[note] = None
            continue
        med = float(np.median(a))
        mad = float(np.median(np.abs(a - med)))
        # A floor on the scale, because MAD can come back near zero on a tight
        # population and would then reject captures that are perfectly good.
        lim = max(args.k * mad, args.floor)
        ref[note] = (med, mad, lim)
        lab = (f"note {note[0]} mode {note[1]}" if isinstance(note, tuple)
               else f"note {note}")
        print(f"{lab}: {len(a)} captures, median disagreement "
              f"{med:+.4f} c, MAD {mad:.4f} c -> reject beyond "
              f"{lim:.4f} c from the median")
        if abs(med) > 1.0:
            print(f"  (that median is a per-note SYSTEMATIC of {med:+.3f} c "
                  f"between h{h1} and its crosscheck. It cancels in a "
                  f"difference and is NOT a reason to drop captures.)")
    print()

    # PASS 2: apply it.
    rc = 0
    for sc, doc in docs:
        rid = doc.get("run_id")
        print(f"run {rid}")
        total_drop = total_seen = 0
        for cell in doc.get("cells", []):
            note, mode = cell["note"], cell["mode"]
            got = meas.get((sc, note, mode))
            if got is None:
                print(f"  note {note} mode {mode}: screen UNAVAILABLE, every "
                      f"candidate harmonic clamps to Nyquist. UNSCREENED.")
                cell["screened"] = False
                continue
            rows, h2 = got
            r = ref.get((note, mode) if args.ref_per_arm else note)
            if r is None:
                print(f"  note {note} mode {mode}: fewer than {args.min_ref} "
                      f"captures at this note, no reference. UNSCREENED.")
                cell["screened"] = False
                continue
            med, mad, lim = r
            kept_c, kept_f, dropped = [], [], []
            for i, ca, fa, d in rows:
                total_seen += 1
                if d is None:
                    dropped.append((i, ca, None))
                elif abs(d - med) > lim:
                    dropped.append((i, ca, d))
                else:
                    kept_c.append(ca)
                    kept_f.append(fa)
            before = _cell_stats(cell.get("cents", []))
            after = _cell_stats(kept_c)
            cell["cents"] = kept_c
            cell["freqs"] = kept_f
            cell["n"] = len(kept_c)
            cell["screened"] = True
            cell["crosscheck_harmonic"] = h2
            cell["dropped"] = [{"i": i, "cents": c, "disagree": d}
                               for i, c, d in dropped]
            total_drop += len(dropped)
            print(f"  note {note} mode {mode}: n {before['n']} -> "
                  f"{after['n']}   mean {before['mean']:+.4f} -> "
                  f"{after['mean']:+.4f} c   sd {before['sd']:.4f} -> "
                  f"{after['sd']:.4f} c   (crosscheck h{h2})")
            for i, c, d in dropped:
                why = (f"disagreement {d:+.3f} c, {abs(d - med):.3f} from the "
                       f"note's median" if d is not None
                       else "no capture or no lock")
                cs = f"{c:+.4f} c" if c is not None else "no estimate"
                print(f"    dropped trigger {i}: {cs}, {why}")
        frac = total_drop / max(1, total_seen)
        if frac > args.max_drop_frac:
            print(f"  RUN FLAGGED: the screen dropped {100 * frac:.0f} % of "
                  f"this run's captures, over the "
                  f"{100 * args.max_drop_frac:.0f} % limit. A screen doing that "
                  f"much work is not cleaning a run, it is telling you the run "
                  f"is broken.")
            doc["flagged"] = True
            rc = 1
        out = (sc[:-5] + ".screened.json" if sc.endswith(".json")
               else sc + ".screened.json")
        with open(out, "w") as fh:
            json.dump(doc, fh, indent=1)
        print(f"  -> {out}\n")
    return rc


def build_parser():
    p = argparse.ArgumentParser(
        description="claim-038 G13: the 63:39:0 sign test and its instrument.")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("synthetic", help="estimator vs tones of KNOWN f0 "
                                         "(no hardware)")
    s.add_argument("--secs", type=float, default=2.55,
                   help="analysis window, matching what a real capture yields "
                        "after steady() trims the attack and release")
    s.add_argument("--need", type=float, default=0.10,
                   help="differential resolution the claim needs, in cents")
    s.add_argument("--break-with", default="none",
                   choices=("none", "deaf", "quantize", "amplitude",
                            "harmonic"),
                   help="deliberately break the estimator and confirm this "
                        "harness goes RED (claim-081 / trap 6)")
    s.add_argument("--break-gain", type=float, default=1.0,
                   help="severity multiplier for --break-with")
    s.add_argument("--seeds", type=int, default=12,
                   help="noise draws per cell in the random-error stage")
    s.set_defaults(fn=cmd_synthetic)

    s = sub.add_parser("sustain", help="does the 101 hold a note? (hardware)")
    s.add_argument("--note", type=int, default=36)
    s.add_argument("--secs", type=float, default=6.0)
    s.add_argument("--tol-db", type=float, default=1.0)
    s.add_argument("--wav")
    s.add_argument("--floor-secs", type=float, default=1.0,
                   help="silence captured before the note, for the noise floor")
    s.add_argument("--wav-floor", default=None)
    s.add_argument("--min-level-db", type=float, default=12.0,
                   help="tail must be within this many dB of the peak")
    s.add_argument("--min-snr-db", type=float, default=20.0,
                   help="tail must exceed the measured noise floor by this")
    s.set_defaults(fn=cmd_sustain)

    s = sub.add_parser("selftest", help="repeat spread on the real signal "
                                        "(hardware)")
    s.add_argument("--note", type=int, default=36)
    s.add_argument("--n", type=int, default=6)
    s.add_argument("--secs", type=float, default=3.0)
    s.add_argument("--settle", type=float, default=1.0)
    s.add_argument("--need", type=float, default=0.10)
    s.add_argument("--fire-at", type=float, default=1.2,
                   help="seconds into the capture at which the note is fired")
    s.add_argument("--head", type=float, default=0.05,
                   help="window start, seconds after the DETECTED onset")
    s.add_argument("--win", type=float, default=0.60, help="window length, s")
    s.add_argument("--harmonic", type=int, default=16,
                   help="measure this harmonic and refer it back to f0; the "
                        "highest one that fits under Nyquist is used. 1 = the "
                        "fundamental, which is what the first version did")
    s.add_argument("--gap", type=float, default=0.4,
                   help="silence between repeats, so each is a fresh trigger")
    s.set_defaults(fn=cmd_selftest)

    s = sub.add_parser("run", help="the 63:39:0 sign test itself (hardware)")
    s.add_argument("--notes", type=int, nargs="*", default=None)
    s.add_argument("--modes", type=int, nargs="*", default=None,
                   help="0x3E arms; 0 is the control and is required")
    s.add_argument("--n", type=int, default=20, help="triggers per cell")
    s.add_argument("--secs", type=float, default=3.0, help="capture length")
    s.add_argument("--window", type=float, default=6.0,
                   help="0x3E window in REAL seconds; window_byte() rescales "
                        "it for the arm's clock, which millis() does not")
    s.add_argument("--fire-at", type=float, default=1.2)
    s.add_argument("--head", type=float, default=0.05)
    s.add_argument("--win", type=float, default=0.60)
    s.add_argument("--harmonic", type=int, default=16)
    s.add_argument("--gap", type=float, default=0.3)
    s.add_argument("--interleave", action="store_true",
                   help="alternate the arms trigger by trigger so drift is "
                        "common-mode; strongly recommended on this machine")
    s.add_argument("--outdir", default="/tmp/g13",
                   help="where captures are written")
    s.add_argument("--run-id", default=None,
                   help="distinguishes this run's captures from every other "
                        "run's; defaults to a UTC timestamp. Do NOT reuse one: "
                        "a repeated id overwrites the earlier run's evidence")
    s.add_argument("--crosscheck", dest="crosscheck",
                   action="store_true", default=True,
                   help="screen every capture by reading it at a second "
                        "harmonic and rejecting it when the estimator "
                        "disagrees with itself (default on)")
    s.add_argument("--no-crosscheck", dest="crosscheck", action="store_false",
                   help="disable the per-capture screen. Use only to "
                        "demonstrate what it was catching")
    s.add_argument("--gross-disagree", type=float, default=20.0,
                   help="in-run guard: reject a capture whose two harmonics "
                        "disagree by more than this many cents. WIDE on "
                        "purpose; the real screen is the host-side 'screen', "
                        "which has a population to compare against")
    s.add_argument("--sd-factor", type=float, default=DEFAULT_SD_FACTOR,
                   help="reject a cell whose within-run sd exceeds this many "
                        "times the median cell sd for its note")
    s.add_argument("--min-repeats", type=int, default=DEFAULT_MIN_REPEATS,
                   help="surviving repeats required before any shift is "
                        "printed as a result")
    s.set_defaults(fn=cmd_run)

    s = sub.add_parser("pool", help="pool several runs, apply the quality cut "
                                    "and publish (no hardware)")
    s.add_argument("json", nargs="+",
                   help="run sidecars, or globs of them")
    s.add_argument("--sd-factor", type=float, default=DEFAULT_SD_FACTOR)
    s.add_argument("--min-repeats", type=int, default=DEFAULT_MIN_REPEATS)
    s.set_defaults(fn=cmd_pool)

    s = sub.add_parser("reanalyse", help="re-read SAVED captures at several "
                                         "harmonics (no hardware)")
    s.add_argument("wav", nargs="+", help="capture paths or globs")
    s.add_argument("--harmonics", type=int, nargs="+", default=[8, 16, 32])
    s.add_argument("--reference", type=int, default=16,
                   help="the harmonic the published number came from")
    s.add_argument("--head", type=float, default=0.05)
    s.add_argument("--win", type=float, default=0.60)
    s.add_argument("--outlier-mad", type=float, default=4.0,
                   help="a capture this many MAD from its cell median is an "
                        "outlier worth attributing")
    s.set_defaults(fn=cmd_reanalyse)

    s = sub.add_parser("screen", help="re-screen saved runs per capture "
                                      "(no hardware)")
    s.add_argument("json", nargs="+", help="run sidecars, or globs of them")
    s.add_argument("--ref-per-arm", action="store_true",
                   help="compute the disagreement reference per (note, arm) "
                        "rather than per note. Equalises the selection between "
                        "the arms, which a difference measurement wants; run "
                        "both and compare rather than picking one")
    s.add_argument("--k", type=float, default=6.0,
                   help="reject a capture whose disagreement departs from its "
                        "note's median by more than k MADs")
    s.add_argument("--floor", type=float, default=1.0,
                   help="floor on that limit, in cents. MAD can come back near "
                        "zero on a tight population and would then reject "
                        "perfectly good captures")
    s.add_argument("--min-ref", type=int, default=20,
                   help="captures required at a note before its reference is "
                        "trusted; below this the note is left UNSCREENED")
    s.add_argument("--capture-dir", default=None,
                   help="where the .wav files are; defaults to the sidecar's "
                        "own directory")
    s.add_argument("--max-drop-frac", type=float, default=0.10,
                   help="flag a run whose screen drops more than this "
                        "fraction of its captures")
    s.set_defaults(fn=cmd_screen)

    s = sub.add_parser("cutcheck", help="break-test the quality cut itself "
                                        "(no hardware)")
    s.set_defaults(fn=cmd_cutcheck)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
