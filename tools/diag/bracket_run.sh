#!/bin/bash
# Run one or more g13_sign repeats, each BRACKETED by a boot-counter probe read,
# so a reset DURING a run is attributed to that run rather than inferred from
# the audio afterwards (L136). That is the check that was missing when mc-r3
# came back with both arms reading identically.
#
# THE BUG THIS VERSION FIXES, found 2026-08-26 in the middle of a live run.
# The previous script piped the probe through `grep -E ... 2>&1`. A probe that
# FAILS then has its error swallowed by the grep and leaves an EMPTY section
# under its heading, which reads exactly like a clean read to anyone skimming.
# mc-r5's after-probe did precisely that, and the bracket survived only because
# the next run's before-probe happened to serve the same purpose. An empty
# bracket is NO MEASUREMENT, not good news, and it must say so.
#
#   tools/diag/bracket_run.sh 7 8       runs mc-r7 then mc-r8
#
# Run ids must have NO existing captures: a reused id overwrites evidence.
set -u
REPO=$(cd "$(dirname "$0")/../.." && pwd)
OUT=${OUT:-/tmp/g13cut}
NOTES=${NOTES:-"67 68 51 52 56 57"}
export SUPEROS_AGENT=A101
cd "$REPO" || exit 1

probe() {   # $1 = label. Prints the three fields, or says FAILED. Never silent.
  local label="$1" tmp rc
  tmp=$(mktemp)
  python3 "$OUT/probe_raw.py" > "$tmp" 2> "$tmp.err"
  rc=$?
  echo "=== $label"
  if [ $rc -ne 0 ]; then
    echo "  PROBE FAILED rc=$rc -- THIS BRACKET IS NO MEASUREMENT, not a clean read"
    sed 's/^/  | /' "$tmp.err" | head -5
  else
    grep -E "raw\[  0\]|raw\[  7\]|vcc" "$tmp" \
      || echo "  PROBE RETURNED NO MATCHING FIELDS -- NO MEASUREMENT"
  fi
  rm -f "$tmp" "$tmp.err"
}

for r in "$@"; do
  if compgen -G "$OUT/g13-run-mc-r$r-*.wav" > /dev/null; then
    echo "REFUSING mc-r$r: captures already exist under that id. Pick an unused one."
    continue
  fi
  probe "before mc-r$r"
  python3 -u tools/diag/g13_sign.py run --notes $NOTES --modes 0 1 \
    --n 20 --harmonic 16 --interleave --outdir "$OUT" --run-id "mc-r$r" \
    > "$OUT/mc-r$r.log" 2>&1
  echo "=== run mc-r$r exited rc=$?"
  probe "after mc-r$r"
done
