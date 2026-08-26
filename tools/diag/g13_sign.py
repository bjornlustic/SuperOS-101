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
    return f


def _report_cell(mode, fs, f_nom):
    if not fs:
        print(f"  mode {mode:>2}: NO LOCKS")
        return
    a = np.array(fs)
    c = cents(a, f_nom)
    sem = c.std() / np.sqrt(len(a)) if len(a) > 1 else float("nan")
    print(f"  mode {mode:>2}: n={len(a):>3}  {a.mean():>11.5f} Hz  "
          f"{c.mean():>+8.3f} c  sd {c.std():.4f}  sem {sem:.4f}")


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
                b.all_off()
                time.sleep(0.15)
                b.bench_window(mode, secs_b)
                time.sleep(0.05)
                path = os.path.join(args.outdir,
                                    f"g13-run-{args.run_id}-{note}-{mode}-{i}.wav")
                sr, sig = capture_note(b, note, path, args.secs, args.fire_at)
                win = onset_window(sig, sr, args.head, args.win)
                if win is None:
                    continue
                f, _ = estimate_f0_harmonic(win, f_nom, rate=sr,
                                            want=args.harmonic)
                if f is not None:
                    fs.append(f)
                time.sleep(args.gap)
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

    # the differences, which are the actual claim
    print("SHIFT vs the 4 MHz control, per note. This is the published quantity.")
    print(f"{'note':>5} {'63-cv':>6} {'mode':>5} {'shift c':>10} {'+/- c':>8}")
    table = {}
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
            table[(note, mode)] = (d, err)
            print(f"{note:>5} {63 - (note - CV_OF):>6} {mode:>5} "
                  f"{d:>+10.4f} {err:>8.4f}")

    print("\nTHE PREDICTION: the shift is proportional to (63 - cv), so the "
          "three notes\nshould stand as 63 : 39 : 0, and note 99 should show "
          "NO shift at all.")
    for mode in arms:
        if mode == 0:
            continue
        got = [(63 - (n - CV_OF), table.get((n, mode))) for n in notes]
        got = [(k, v) for k, v in got if v]
        if len(got) < 2:
            continue
        base = next((v[0] for k, v in got if k == 63), None)
        print(f"\n  mode {mode}:")
        for k, (d, err) in got:
            rel = "" if not base or base == 0 else f"   {100 * d / base:>6.1f} % of the 63 arm"
            pred = "" if base is None else f"   predicted {(k / 63.0) * (base or 0):>+7.4f} c"
            print(f"    63-cv {k:>2}: {d:>+8.4f} +/- {err:.4f} c{rel}{pred}")
    print("\nA shift that is large at 63, about 62 % of it at 39 and consistent "
          "with zero at 0\nis the mechanism. Anything else falsifies it, and a "
          "common-mode confounder\n(clock error, capture-rate error, tuning "
          "drift, temperature) cannot make that shape.")
    return 0


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
    s.set_defaults(fn=cmd_run)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
