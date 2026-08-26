"""Retrospective harmonic-level audit of the six PUBLISHED claim-038 rows.

Why: the level guard (cc9a70e, 11:37) is a RUN-TIME check. Every row captured
before it existed was never asked whether the selected partial had energy in
it, and mc-r3 showed the failure mode is real and gross (note 51 h16 fell
-14 dB -> -43 dB). This re-asks the guard's question of captures already on
disk, at zero bench cost.

Samples captures per cell rather than measuring all of them: the fault being
looked for is ~30 dB, not 1 dB, and the in-flight bracketed run has the CPU.
"""
import sys, os, glob, json
sys.path.insert(0, "/Users/bjornlustic/Desktop/Projects/SH101/SuperOS-101FIRMWARE/tools/diag")
import numpy as np
import g13_sign as G

SIDE = "/Users/bjornlustic/Desktop/Projects/SH101/SuperOS-101FIRMWARE/artifacts/g13/sidecars"
CAPS = "/tmp/g13cut"
STRIDE = int(os.environ.get("STRIDE", "5"))

rows = []
for sc in sorted(glob.glob(os.path.join(SIDE, "*.json"))):
    if "screened" in sc:
        continue
    doc = json.load(open(sc))
    rid = doc.get("run_id")
    h_asked = doc.get("harmonic", 16)
    for cell in doc.get("cells", []):
        note, mode = cell["note"], cell["mode"]
        f_nom = cell.get("f_nom") or G.midi_hz(note)
        h_used = G.pick_harmonic(f_nom, G.RATE, h_asked)
        lv_used, lv_i = [], []
        i = 0
        while True:
            wav = os.path.join(CAPS, f"g13-run-{rid}-{note}-{mode}-{i}.wav")
            if not os.path.isfile(wav):
                break
            if i % STRIDE == 0:
                sr, sig = G._read_capture(wav)
                win = G.onset_window(sig, sr, doc["capture"]["head"],
                                     doc["capture"]["win"])
                if win is not None:
                    lv = G.harmonic_levels(win, f_nom, rate=sr)
                    if h_used in lv:
                        lv_used.append(lv[h_used]); lv_i.append(i)
            i += 1
        if not lv_used:
            rows.append((os.path.basename(sc), rid, note, mode, h_asked,
                         h_used, None, None, 0, i))
            continue
        a = np.array(lv_used)
        rows.append((os.path.basename(sc), rid, note, mode, h_asked, h_used,
                     float(np.median(a)), float(a.min()), len(a), i))

print(f"retro level audit, stride {STRIDE}, guard {G.MIN_HARMONIC_DB:+.0f} dB\n")
print(f"{'sidecar':26s} {'note':>4s} {'md':>2s} {'ask':>3s} {'use':>3s} "
      f"{'median dB':>9s} {'min dB':>8s} {'n':>3s}/{'cap':<4s} verdict")
bad = []
for sc, rid, note, mode, ha, hu, med, mn, n, tot in rows:
    if med is None:
        v = "NO WINDOW"
    elif mn < G.MIN_HARMONIC_DB:
        v = "BELOW GUARD"; bad.append((sc, note, mode, mn))
    elif mn < G.MIN_HARMONIC_DB + 10:
        v = "within 10 dB"
    else:
        v = "ok"
    print(f"{sc:26s} {note:4d} {mode:2d} {ha:3d} {hu:3d} "
          f"{'--' if med is None else f'{med:+9.1f}'} "
          f"{'--' if mn is None else f'{mn:+8.1f}'} {n:3d}/{tot:<4d} {v}")
print()
if bad:
    print(f"{len(bad)} cells have at least one sampled capture below the guard:")
    for b in bad:
        print(f"  {b[0]} note {b[1]} mode {b[2]}  min {b[3]:+.1f} dB")
else:
    print("NO sampled capture in any published row falls below the guard.")
