#!/bin/bash
# claim-038 major-carry test: harmonic and window systematics for every note in
# the mc-* runs, re-derived from those captures at zero bench cost. Same recipe
# as the note 36/59/60/61/72/99 rows, so the numbers are comparable to them.
set -u
SD="$1"; shift
RUNS="${@:-mc-r1 mc-r2 mc-r5 mc-r6}"   # r3/r4 EXCLUDED BY NAME: bad data, see handoff s4
REPO=/Users/bjornlustic/Desktop/Projects/SH101/SuperOS-101FIRMWARE
export SUPEROS_AGENT=A101
cd "$REPO" || exit 1
variant() {   # label harmonic head win
  d="$SD/mc-$1"; rm -rf "$d"; mkdir -p "$d"
  for r in $RUNS; do
    f="/tmp/g13cut/g13-run-$r.json"; [ -f "$f" ] || { echo "MISSING $f" >&2; continue; }
    python3 - "$f" "$d/$(basename $f)" "$2" "$3" "$4" <<'PY'
import json,sys
src,dst,h,head,win = sys.argv[1],sys.argv[2],int(sys.argv[3]),float(sys.argv[4]),float(sys.argv[5])
d=json.load(open(src)); d["harmonic"]=h
for c in d["cells"]: c["nh"]=h
d["capture"]["head"]=head; d["capture"]["win"]=win
json.dump(d,open(dst,"w"),indent=1)
PY
  done
  python3 tools/diag/g13_sign.py screen $(for r in $RUNS; do echo "$d/g13-run-$r.json"; done) --capture-dir /tmp/g13cut > "$d/screen.log" 2>&1
  python3 tools/diag/g13_sign.py pool $(for r in $RUNS; do echo "$d/g13-run-$r.screened.json"; done) > "$d/pool.log" 2>&1
  echo "== $1 (h$2 head $3 win $4)  dropped $(grep -c 'dropped trigger' $d/screen.log) flagged $(grep -c 'RUN FLAGGED' $d/screen.log)"
  sed -n '/PUBLISHABLE ROWS/,$p' "$d/pool.log" | grep -E "^ +[0-9]+ +[0-9]+ +1 "
}
variant ref   16 0.05 0.60
variant h8     8 0.05 0.60
variant h32   32 0.05 0.60
variant early 16 0.05 0.30
variant late  16 0.35 0.30
variant vlate 16 0.65 0.30
