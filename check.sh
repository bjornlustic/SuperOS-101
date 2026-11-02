#!/bin/sh
# SH101-Firmware: every machine check, in one command. `sh check.sh`
#
# Exits 0 only when nothing failed. A check that cannot run prints SKIP and the
# reason; it is never reported as PASS. Nothing here touches hardware: no flash,
# fuses, upload, MIDI port or tools/diag. Everything builds in a scratch copy
# under $TMPDIR, so the checkout gets no new .syx, .hex, binary or log.
# What each line proves, and what only the bench can prove: verify/README.md.

ROOT=$(cd "$(dirname "$0")" && pwd)
PASSES=0; FAILS=0; SKIPS=0

# The 8 cases where engine.h and the mask ROM differ (claim-096, G14 Phase 0):
# arp and sequencer CV or gate, and event order on the keyboard case. A ninth
# fails this check; so does a fix, so update the pin together with the fix.
G14_PIN="1 10 11 12 14 15 18 19"
G14_TOTAL=20
FLASH_CEILING=0x16000   # tools/makesyx.py FLASH_ARENA_BASE

pass() { PASSES=$((PASSES + 1)); printf 'PASS  %s\n' "$*"; }
skip() { SKIPS=$((SKIPS + 1)); printf 'SKIP  %s\n' "$*"; }
fail() { FAILS=$((FAILS + 1)); printf 'FAIL  %s\n' "$*"; }
tail_of() { sed 's/^/        /' "$1" | tail -n 15; }

S=$(mktemp -d "${TMPDIR:-/tmp}/sh101-check.XXXXXX") || exit 1
trap 'rm -rf "$S"' EXIT INT TERM
W=$S/w

# The mask ROM dump is git-ignored. A worktree has none, so fall back to the
# main checkout's copy. It is only read and copied into $S, never into git.
REF=${SH101_REF:-}
if [ -z "$REF" ] && [ -f "$ROOT/SH101-reference/80C49_SH101.bin" ]; then REF=$ROOT/SH101-reference; fi
if [ -z "$REF" ]; then
    common=$(git -C "$ROOT" rev-parse --git-common-dir 2>/dev/null)
    if [ -n "$common" ]; then
        main=$(cd "$ROOT" && cd "$common/.." 2>/dev/null && pwd)
        if [ -f "$main/SH101-reference/80C49_SH101.bin" ]; then REF=$main/SH101-reference; fi
    fi
fi

[ -f "$REF/80C49_SH101.bin" ] || REF=

mkdir -p "$W" || exit 1
rsync -a --exclude .git --exclude .pio --exclude artifacts --exclude docs --exclude old \
    --exclude SH101-reference --exclude '*.syx' --exclude '*.hex' \
    --exclude 'sim/superos101-sim' --exclude 'sim/*.bin' "$ROOT/" "$W/" || { echo "FAIL  scratch copy"; exit 1; }

# sim/Makefile stamps `git rev-parse` into the build, and the self-test fails
# on the "unknown" fallback, so the copy needs a repo of its own.
(cd "$W" && git init -q . && git add -A &&
    git -c user.name=check -c user.email=check@invalid commit -qm scratch) >/dev/null 2>&1

echo "SH101-Firmware check ($(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo no-git))"

# 1. Firmware images and flash margin. Needs the AVR toolchain to run here.
PIO=${PIO:-}
if [ -z "$PIO" ]; then
    PIO=$(command -v pio 2>/dev/null)
    [ -n "$PIO" ] || PIO=$(ls "$HOME"/Library/Python/*/bin/pio 2>/dev/null | head -n 1)
fi
AVRGCC=$HOME/.platformio/packages/toolchain-atmelavr/bin/avr-gcc
if [ -z "$PIO" ]; then
    skip "firmware build, bytes free: pio not found (last recorded: 26690 free of 0x16000, T0068 recipe 2026-11-01)"
elif ! "$AVRGCC" --version >/dev/null 2>&1; then
    skip "firmware build, bytes free: the AVR toolchain is missing or does not run on this machine (last recorded: 26690 free of 0x16000, T0068 recipe 2026-11-01)"
else
    if (cd "$W" && "$PIO" run -e app -e bootloader) >"$S/pio.log" 2>&1; then
        pass "pio run -e app -e bootloader"
        if out=$(python3 -I "$ROOT/tools/hexfree.py" "$W/.pio/build/app/firmware.hex" "$FLASH_CEILING" 2>&1); then
            pass "app flash: $out (ceiling $FLASH_CEILING)"
        else
            fail "app flash over the ceiling: $out (ceiling $FLASH_CEILING)"
        fi
        ram=$(grep 'RAM:' "$S/pio.log" | head -n 1 | sed 's/.*used \([0-9]*\) bytes from \([0-9]*\).*/\1 of \2/')
        [ -n "$ram" ] && echo "      app RAM used $ram (the rest is stack)"
    else
        fail "pio run -e app -e bootloader"; tail_of "$S/pio.log"
    fi
fi

# 2. Host simulator: real firmware sources plus the real mask ROM.
if [ -z "$REF" ]; then
    skip "sim self-test, G1, G14, G14 red test: mask ROM dump not found (SH101-reference/80C49_SH101.bin; set SH101_REF)"
else
    mkdir -p "$W/SH101-reference"
    cp "$REF/80C49_SH101.bin" "$W/SH101-reference/"
    if ! (cd "$W" && python3 tools/mkrom.py) >"$S/mkrom.log" 2>&1; then
        fail "tools/mkrom.py"; tail_of "$S/mkrom.log"
    else
        rm -f "$W/sim/superos101-sim"   # a stale binary must not be what gets tested
        if (cd "$W/sim" && make test) >"$S/sim.log" 2>&1 && grep -q 'self-test passed (0 failures)' "$S/sim.log"; then
            pass "sim self-test: $(grep -c '^  PASS' "$S/sim.log") checks, 0 failures"
        else
            fail "sim self-test"; tail_of "$S/sim.log"
        fi
    fi
fi

if [ -x "$W/sim/superos101-sim" ]; then
    # 3. Panel functions through the sim.
    if (cd "$W/sim" && python3 g1_test.py) >"$S/g1.log" 2>&1 && grep -q 'all G1 panel checks passed' "$S/g1.log"; then
        pass "G1 panel functions: $(grep -c '^  ok ' "$S/g1.log") of $(grep -c '^  \(ok \|FAIL\)' "$S/g1.log")"
    else
        fail "G1 panel functions"; tail_of "$S/g1.log"
    fi

    # 4. G14: SuperOS engine against the mask ROM. Exit 0 carries no verdict, so
    # the diverged list is compared with the pin.
    g14_summary() { # $1 = log; prints "matched M diverged D untested U"
        sed -n 's/^matched \([0-9]*\) *diverged \([0-9]*\) *UNTESTED \([0-9]*\).*/matched \1 diverged \2 untested \3/p' "$1"
    }
    g14_diverged() { # a row with anything but four "ok" in its last four columns
        awk '/^[0-9]+ / && !/UNTESTED/ { bad = 0; for (i = NF - 3; i <= NF; i++) if ($i != "ok") bad = 1; if (bad) printf "%s ", $1 }' "$1" | sed 's/ $//'
    }
    (cd "$W/sim" && ./superos101-sim --g14) >"$S/g14.log" 2>&1; rc=$?
    got=$(g14_summary "$S/g14.log"); list=$(g14_diverged "$S/g14.log")
    set -- $G14_PIN; npin=$#
    want="matched $((G14_TOTAL - npin)) diverged $npin untested 0"
    if [ "$rc" -ne 0 ]; then
        fail "G14 exited $rc"; tail_of "$S/g14.log"
    elif [ "$got" != "$want" ]; then
        fail "G14 count: got '$got', pinned '$want'"; tail_of "$S/g14.log"
    elif [ "$list" != "$G14_PIN" ]; then
        fail "G14 diverged cases: got '$list', pinned '$G14_PIN'"
    else
        pass "G14: $got; diverged cases $list as pinned"
    fi

    # 5. Red test (claim-081): a deliberately damaged engine must turn all of G14 red.
    for b in 1 2 3; do
        (cd "$W/sim" && G14_BREAK=$b ./superos101-sim --g14) >"$S/g14b$b.log" 2>&1
        got=$(g14_summary "$S/g14b$b.log")
        if grep -q 'RED-CONDITION RUN' "$S/g14b$b.log" && [ "$got" = "matched 0 diverged $G14_TOTAL untested 0" ]; then
            pass "G14_BREAK=$b goes red: $got"
        else
            fail "G14_BREAK=$b did not go red: got '$got'"; tail_of "$S/g14b$b.log"
        fi
    done
fi

echo "---"
echo "$PASSES passed, $FAILS failed, $SKIPS skipped"
[ "$SKIPS" -eq 0 ] || echo "Skipped checks were NOT run. Their reasons are above."
if [ "$PASSES" -eq 0 ]; then echo "Nothing ran, so nothing is verified."; exit 2; fi
[ "$FAILS" -eq 0 ]
