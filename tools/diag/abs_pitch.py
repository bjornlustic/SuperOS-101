#!/usr/bin/env python3
"""Absolute pitch of a 101 capture, for the keyboard-tracking question.

WHY A SECOND ESTIMATOR EXISTS. `g13_sign.estimate_f0_harmonic` reads ONE
harmonic and divides. That is right for claim-038, which is a difference
between two arms at the same note and the same harmonic, where a per-(note,
harmonic) bias cancels exactly. It is WRONG for absolute pitch: the same
mode-0 captures read at h1/h2/h4/h8/h16/h32 disagree with themselves by 5 to
135 cents, so any single harmonic's absolute number is that bias plus the
pitch and nothing separates them.

This reads the PERIOD instead, from the autocorrelation of the whole
waveform, so every partial contributes and no harmonic has to be chosen. It
is checked against synthetic tones of KNOWN frequency before it is believed
(COMMON rule 1: state the instrument's resolution next to the tolerance the
claim needs). The claim needs ~1 cent across the keyboard.
"""
import sys, os, glob, statistics as st
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import importlib.util
spec = importlib.util.spec_from_file_location(
    "g13", os.path.join(os.path.dirname(os.path.abspath(__file__)), "g13_sign.py"))
g = importlib.util.module_from_spec(spec); spec.loader.exec_module(g)


def f0_coarse(sig, rate, f_hint, span=0.06):
    """Coarse period from the autocorrelation. Good to ~10 c and no better;
    it exists only to tell the fine stage WHICH partial is which."""
    x = np.asarray(sig, dtype=float)
    x = x - x.mean()
    if not np.any(x):
        return None
    n = len(x)
    ac = np.correlate(x, x, mode="full")[n - 1:]
    lag0 = rate / f_hint
    lo, hi = int(lag0 * (1 - span)), int(np.ceil(lag0 * (1 + span))) + 1
    lo, hi = max(2, lo), min(len(ac) - 2, hi)
    if hi <= lo:
        return None
    k = lo + int(np.argmax(ac[lo:hi]))
    a, b, c = ac[k - 1], ac[k], ac[k + 1]
    den = a - 2 * b + c
    k = k + (0.5 * (a - c) / den) if den else k
    return rate / k if k > 0 else None


def f0_consensus(sig, rate, f_hint, harmonics=(1, 2, 3, 4, 5, 6, 7, 8)):
    """Absolute f0 from SEVERAL partials, each located by the validated fine
    search, and the spread between them returned as the honest error.

    The coarse stage says where partial n is to ~10 c; the fine stage then
    measures it to well under a cent. Dividing each by n gives n independent
    readings of the SAME f0, so their spread is the estimator's own error on
    THIS capture rather than an assumed one. That is the whole point: the
    single-harmonic estimator could not do this, which is why its absolute
    numbers disagreed by up to 135 c with nothing to flag it.
    """
    c0 = f0_coarse(sig, rate, f_hint)
    if not c0:
        return None, None, 0
    est = []
    for n in harmonics:
        if n * c0 > 0.40 * rate:                    # Nyquist, same as pick_harmonic
            break
        span = min(0.02, 0.40 / n)                  # cannot reach partial n+-1
        f = g.estimate_f0(sig, rate, expect=n * c0, span=span)
        if f:
            est.append(f / n)
    if not est:
        return None, None, 0
    med = float(np.median(est))
    spread = 1200 * np.log2(max(est) / min(est)) if len(est) > 1 else float("nan")
    return med, spread, len(est)


def selftest():
    """Synthetic sawtooths of KNOWN f0, with the 101's decay and noise."""
    rate = 48000
    print("SELFTEST: sawtooth of known f0, 0.60 s, decaying, with noise")
    print(f"{'f0 true':>10} {'f0 read':>10} {'error c':>9} {'spread c':>9} {'k':>3}")
    rng = np.random.default_rng(101)
    worst = 0.0
    for f in (246.94165, 261.6256, 277.18263, 523.2511, 2489.0159):
        t = np.arange(int(0.60 * rate)) / rate
        sig = 2 * ((f * t) % 1.0) - 1.0                # sawtooth: every partial
        sig *= np.exp(-1.1 * t)                        # the 101's own decay
        sig += rng.normal(0, 0.004, len(sig))
        got, spread, k = f0_consensus(sig, rate, f)
        e = 1200 * np.log2(got / f)
        worst = max(worst, abs(e))
        print(f"{f:>10.4f} {got:>10.4f} {e:>+9.4f} {spread:>9.4f} {k:>3}")
    need = 1.0
    print(f"worst {worst:.4f} c against the {need:.1f} c the keyboard question needs: "
          f"{'USABLE' if worst < need else 'NOT USABLE, do not quote it'}")
    return worst < need


if __name__ == "__main__":
    if not selftest():
        sys.exit(1)
    print()
    runs = {36: ["cut-r?"], 59: ["n59-r?"], 60: ["n60-r?"],
            61: ["n61-r?"], 72: ["n72-r?"], 99: ["n99-r?"]}
    print("101 ABSOLUTE PITCH vs equal temperament, mode 0 = SHIPPING 4 MHz divide")
    print(f"{'note':>4} {'cv':>3} {'Hz nom':>9} {'n':>4} {'cents':>9} "
          f"{'sd':>7} {'sem':>7}")
    out = {}
    for note in sorted(runs):
        wavs = []
        for pat in runs[note]:
            wavs += sorted(glob.glob(f"/tmp/g13cut/g13-run-{pat}-{note}-0-*.wav"))
        vals = []
        for w in wavs:
            sr, sig = g._read_capture(w)
            win = g.onset_window(sig, sr, 0.05, 0.60)
            if win is None:
                continue
            f_nom = g.midi_hz(note)
            f, spread, k = f0_consensus(win, sr, f_nom)
            if f and k >= 3 and spread < 2.0:
                vals.append(1200 * np.log2(f / f_nom))
        if not vals:
            print(f"{note:>4}   no captures")
            continue
        m, sd = st.mean(vals), st.pstdev(vals)
        print(f"{note:>4} {note-36:>3} {g.midi_hz(note):>9.2f} {len(vals):>4} "
              f"{m:>+9.3f} {sd:>7.3f} {sd/len(vals)**0.5:>7.3f}")
        out[note] = m
    if len(out) > 1:
        print(f"\nKEYBOARD SPREAD, worst note to worst note: "
              f"{max(out.values()) - min(out.values()):.3f} cents")
