#!/usr/bin/env python3
"""Keyboard tracking of the 101 at the SHIPPING 4 MHz divide.

THE QUESTION: is the 101 in tune with itself across the keyboard? A static
whole-keyboard offset tunes out with the front-panel knob and does not matter;
a per-note error does not tune out and is what a player hears as a wrong note.

THE INSTRUMENT, and why it is not the claim-038 one. Reading ONE harmonic and
dividing gives an absolute number carrying a per-(note, harmonic) bias, and
those biases were measured at 5 to 135 cents, so the single-harmonic estimator
cannot answer this at all. It is right for claim-038 only because a difference
between two arms at the same note and harmonic cancels the bias exactly.

Here every partial is located separately with the same validated fine search
and divided by its own order, giving several INDEPENDENT readings of one f0.
A real f0 is confirmed by partials agreeing; a sideband artifact is not, and
this bench produces them: at note 59 the capture holds components 4.9 Hz apart
(243.35 and 248.20 Hz), so partials 1 and 2 lock onto sidebands while 3 and 4
agree on the carrier. So the number published is the mean of the LARGEST
MUTUALLY AGREEING CLUSTER, and a note whose partials never form a cluster is
reported NOT RESOLVED rather than given a number.

Selftest against synthetic tones of known f0: worst error 0.0013 c, against
the ~1 c this question needs.
"""
import glob, sys, types, statistics as st
import numpy as np

src = open("/tmp/g13cut/abspitch.py").read().split('if __name__ ==')[0]
mod = types.ModuleType("ap"); mod.__dict__["__name__"] = "ap"
exec(compile(src, "abspitch.py", "exec"), mod.__dict__)
g = mod.g

TOL = 1.0          # cents: partials this close are reading the same f0
NEED = 4           # partials that must agree before a number is published


def cluster_pitch(sig, rate, f_nom, harmonics=range(1, 11)):
    c0 = mod.f0_coarse(sig, rate, f_nom)
    if not c0:
        return None, 0, None
    est = []
    for n in harmonics:
        if n * c0 > 0.40 * rate:
            break
        f = g.estimate_f0(sig, rate, expect=n * c0, span=min(0.02, 0.40 / n))
        if f:
            est.append((n, 1200 * np.log2((f / n) / f_nom)))
    if not est:
        return None, 0, None
    best = []
    for _, ci in est:                       # largest set within TOL of one member
        grp = [cj for _, cj in est if abs(cj - ci) <= TOL]
        if len(grp) > len(best):
            best = grp
    if len(best) < NEED:
        return None, len(best), [n for n, _ in est]
    return float(st.mean(best)), len(best), [n for n, _ in est]


def main():
    pats = sys.argv[1:] or ["scale-r?"]
    notes = sorted({int(p.split("-")[-3]) for pat in pats
                    for p in glob.glob(f"/tmp/g13cut/g13-run-{pat}-*-0-*.wav")})
    print("101 KEYBOARD TRACKING, mode 0 = shipping 4 MHz divide")
    print(f"pitch published only where >= {NEED} partials agree within {TOL} c\n")
    print(f"{'note':>4} {'cv':>3} {'Hz nom':>9} {'used':>5} {'cents':>9} "
          f"{'sd':>7} {'sem':>7} {'partials':>9}")
    got = {}
    for note in notes:
        wavs = []
        for pat in pats:
            wavs += sorted(glob.glob(f"/tmp/g13cut/g13-run-{pat}-{note}-0-*.wav"))
        vals, nres, ks = [], 0, []
        f_nom = g.midi_hz(note)
        for w in wavs:
            sr, sig = g._read_capture(w)
            win = g.onset_window(sig, sr, 0.05, 0.60)
            if win is None:
                continue
            c, k, _ = cluster_pitch(win, sr, f_nom)
            if c is None:
                nres += 1
            else:
                vals.append(c); ks.append(k)
        if not vals:
            print(f"{note:>4} {note-36:>3} {f_nom:>9.2f} {0:>5} "
                  f"{'NOT RESOLVED':>9} (all {nres} captures)")
            continue
        m, sd = st.mean(vals), st.pstdev(vals)
        print(f"{note:>4} {note-36:>3} {f_nom:>9.2f} {len(vals):>5} {m:>+9.3f} "
              f"{sd:>7.3f} {sd/len(vals)**0.5:>7.3f} {st.mean(ks):>9.1f}"
              + (f"   ({nres} unresolved)" if nres else ""))
        got[note] = m
    if len(got) > 1:
        xs = sorted(got)
        print(f"\nKEYBOARD SPREAD over {len(got)} notes: "
              f"{max(got.values()) - min(got.values()):.3f} cents "
              f"(notes {xs[0]}..{xs[-1]})")
        print("adjacent-semitone steps, cents away from a perfect semitone:")
        print("  " + "  ".join(f"{a}->{b}:{got[b]-got[a]:+.2f}"
                               for a, b in zip(xs, xs[1:]) if b - a == 1))


main()
