"""Full harmonic level table for the two MARGINAL published rows, note 61 and
note 72, both arms. Question: is h16 low because the whole capture is weak
(harmless, cancels in a difference) or because h16 SPECIFICALLY is a null
(dangerous, the estimator reads noise on one arm only)?"""
import sys, os, json, glob
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, g13_sign as G
SIDE=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "artifacts", "g13", "sidecars")
for tag, runs in (("n61", ["g13-run-n61-r1","g13-run-n61-r3"]),
                  ("n72", ["n72-r1-h16-raw"])):
    for rn in runs:
        doc = json.load(open(os.path.join(SIDE, rn + ".json")))
        rid = doc["run_id"]
        for cell in doc["cells"]:
            note, mode = cell["note"], cell["mode"]
            f_nom = cell.get("f_nom") or G.midi_hz(note)
            acc = {}
            n = 0
            for i in range(0, 20, 4):
                wav = f"/tmp/g13cut/g13-run-{rid}-{note}-{mode}-{i}.wav"
                if not os.path.isfile(wav): continue
                sr, sig = G._read_capture(wav)
                win = G.onset_window(sig, sr, doc["capture"]["head"], doc["capture"]["win"])
                if win is None: continue
                lv = G.harmonic_levels(win, f_nom, rate=sr)
                for k, v in lv.items(): acc.setdefault(k, []).append(v)
                n += 1
            med = {k: float(np.median(v)) for k, v in acc.items()}
            top = sorted(med.items(), key=lambda kv: -kv[1])[:6]
            print(f"{rn:20s} note {note} mode {mode}  n={n}")
            print("   strongest: " + "  ".join(f"h{k}:{v:+.1f}" for k, v in top))
            print("   of interest: " + "  ".join(
                f"h{k}:{med[k]:+.1f}" for k in (2,4,6,8,12,16,24,32) if k in med))
