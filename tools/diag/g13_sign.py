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
    print(f"\npeak rms {peak:.6f}")
    print(f"sustain segment (last 2/3): {fall:+.2f} dB end-to-start")
    held = abs(fall) <= args.tol_db
    print(f"\n{'HELD' if held else 'DECAYS'}: envelope moves {fall:+.2f} dB "
          f"over {len(tail) * 0.1:.1f} s, tolerance +/-{args.tol_db:.1f} dB.")
    if not held:
        print("The 63:39:0 method assumes a steady tone. It does not have one.")
    return 0 if held else 1


def cmd_selftest(args):
    """Repeat spread on the REAL signal. Precision, not bias: --synthetic
    covers bias, and a repeat spread cannot see it."""
    b = Bench()
    f_nom = midi_hz(args.note)
    b.all_off()
    time.sleep(0.2)
    b.note(args.note, True)
    fs = []
    try:
        time.sleep(args.settle)
        for i in range(args.n):
            path = f"/tmp/g13-selftest-{i}.wav"
            sr, sig = capture(path, args.secs)
            f = estimate_f0(steady(sig, sr), rate=sr, expect=f_nom)
            if f is None:
                print(f"  {i}: NO LOCK")
                continue
            fs.append(f)
            print(f"  {i}: {f:>11.5f} Hz  {cents(f, f_nom):>+9.3f} c vs nominal")
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
    print(f"repeat spread: sd {c.std():.4f} c, range {c.max() - c.min():.4f} c")
    print(f"\nThis bounds PRECISION only. Bias is --synthetic's job.")
    print(f"needed for the claim: <= {args.need:.3f} c")
    ok = c.std() <= args.need
    print(f"{'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


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
    s.set_defaults(fn=cmd_sustain)

    s = sub.add_parser("selftest", help="repeat spread on the real signal "
                                        "(hardware)")
    s.add_argument("--note", type=int, default=36)
    s.add_argument("--n", type=int, default=6)
    s.add_argument("--secs", type=float, default=3.0)
    s.add_argument("--settle", type=float, default=1.0)
    s.add_argument("--need", type=float, default=0.10)
    s.set_defaults(fn=cmd_selftest)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
