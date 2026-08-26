"""Why did h12 return a -9.38 c shift at note 61 with the screen dropping
NOTHING, while at note 59 the same harmonic dropped 19 captures and flagged 2?

If the estimator fails the SAME way on every capture, a screen referenced to
the note's OWN median disagreement moves with it and drops nothing. A uniform
failure would be invisible to it. Testing that directly."""
import sys, os, json
sys.path.insert(0, "/Users/bjornlustic/Desktop/Projects/SH101/SuperOS-101FIRMWARE/tools/diag")
import numpy as np, g13_sign as G
SIDE = "/Users/bjornlustic/Desktop/Projects/SH101/SuperOS-101FIRMWARE/artifacts/g13/sidecars"
for h in (8, 12, 16):
    print(f"\n=== note 61, harmonic asked {h} ===")
    per_mode = {}
    for rn in ("g13-run-n61-r1", "g13-run-n61-r2", "g13-run-n61-r3"):
        doc = json.load(open(os.path.join(SIDE, rn + ".json")))
        rid = doc["run_id"]
        for cell in doc["cells"]:
            note, mode = cell["note"], cell["mode"]
            f_nom = cell.get("f_nom") or G.midi_hz(note)
            hu = G.pick_harmonic(f_nom, G.RATE, h)
            cs, nhs = [], set()
            for i in range(0, 20, 2):
                wav = f"/tmp/g13cut/g13-run-{rid}-{note}-{mode}-{i}.wav"
                if not os.path.isfile(wav): continue
                sr, sig = G._read_capture(wav)
                win = G.onset_window(sig, sr, doc["capture"]["head"], doc["capture"]["win"])
                if win is None: continue
                f, nh = G.estimate_f0_harmonic(win, f_nom, rate=sr, want=h)
                if f is None: continue
                cs.append(float(G.cents(f, f_nom))); nhs.add(nh)
            a = np.array(cs)
            per_mode.setdefault(mode, []).extend(cs)
            print(f"  {rn} mode {mode}: n={len(a)} mean {a.mean():+8.3f} c "
                  f"sd {a.std(ddof=1):6.3f} c  range {a.max()-a.min():6.3f} c  nh={sorted(nhs)}")
    if 0 in per_mode and 1 in per_mode:
        m0, m1 = np.array(per_mode[0]), np.array(per_mode[1])
        print(f"  POOLED shift (mode1 - mode0) = {m1.mean()-m0.mean():+.4f} c   "
              f"sd0 {m0.std(ddof=1):.3f}  sd1 {m1.std(ddof=1):.3f}")
