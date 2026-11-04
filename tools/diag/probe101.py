#!/usr/bin/env python3
"""Read the 101's 0x3B -> 0x3C probe, by FIELD, with a lease check.

Why this exists rather than timing/probe101.py (CORE's): that one prefix-matches
the port name and has no lease check, so it will happily talk to a machine
another agent holds. It also indexes the 0x21 reply by offset, which is how it
came to print `ver=1` for a machine running 0.2. Both traps are mine to avoid
here: `midiport.resolve` does the lease check, and the version string is read as
the trailing ASCII tail rather than as a byte at a fixed index.

The probe body is two nibbles per raw byte after the 0x3C opcode, so a reply of
N data bytes carries (N - 2) / 2 raw bytes. Print that count: the bench unit's
97 against main's 127 is a behavioural difference, and raw[125..126] (G5's
key-scan word) simply does not exist below 127.

    python3 tools/diag/probe101.py            # one probe
    python3 tools/diag/probe101.py -n 8       # eight, for the pass-max spread
"""
import argparse
import os
import sys
import time

import mido

# The shared tools (midiport.py) live in SuperOS-Tools/USB-Bootloader/tools.
# Walk up from this file to the first folder that has SuperOS-Tools, so the
# result does not depend on how deep this checkout or a worktree sits.
# $SUPEROS_USBC overrides (a folder holding tools/). Assert, so a miss names
# the paths tried.
def _find_usbc_tools():
    env = os.environ.get("SUPEROS_USBC")
    if env:
        return os.path.join(env, "tools")
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        t = os.path.join(d, "SuperOS-Tools", "USB-Bootloader", "tools")
        if os.path.isfile(os.path.join(t, "midiport.py")):
            return t
        if os.path.dirname(d) == d:
            return t
        d = os.path.dirname(d)


_USBC = _find_usbc_tools()
if not os.path.isfile(os.path.join(_USBC, "midiport.py")):
    sys.exit(f"cannot find the shared tools dir, looked in {_USBC}")
sys.path.insert(0, _USBC)
import midiport


def txn(pin, pout, body, want, timeout=2.0):
    for _ in pin.iter_pending():
        pass
    pout.send(mido.Message("sysex", data=body))
    t0 = time.time()
    while time.time() - t0 < timeout:
        for m in pin.iter_pending():
            if m.type == "sysex" and len(m.data) >= 2 and m.data[1] == want:
                return list(m.data)
        time.sleep(0.002)
    return None


def raw_of(reply):
    nib = reply[2:]
    return [(nib[i] << 4) | nib[i + 1] for i in range(0, len(nib) - 1, 2)]


def w(raw, i):
    return (raw[i] << 8) | raw[i + 1] if i + 1 < len(raw) else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", type=int, default=1, help="number of probes")
    ap.add_argument("--gap", type=float, default=1.1,
                    help="seconds between probes; the per-second counters and "
                         "the key-scan max both roll over at 1 s, so a gap "
                         "under that reports a partial window")
    midiport.add_lease_arg(ap)
    a = ap.parse_args()

    nin = midiport.resolve("101", "in", agent=os.environ.get("SUPEROS_AGENT"),
                           ignore_lease=a.ignore_lease)
    nout = midiport.resolve("101", "out", agent=os.environ.get("SUPEROS_AGENT"),
                            ignore_lease=a.ignore_lease)
    pin, pout = mido.open_input(nin), mido.open_output(nout)

    cfg = txn(pin, pout, [0x7D, 0x20], 0x21)
    if cfg:
        d = cfg[2:]
        ver = "".join(chr(b) for b in d[6:] if 32 <= b < 127).strip() or "?"
        print(f"0x21  fw={d[0]} ({'SuperOS' if d[0] == 0 else 'emulator'}) "
              f"chan={d[1]} sync={d[2]} div={d[3]} pat={d[4]} persist={d[5]} "
              f"ver={ver!r}")
    else:
        print("0x21  NO REPLY")

    ident = txn(pin, pout, [0x7D, 0x34], 0x35)
    if ident:
        print("0x35  " + "".join(chr(b) for b in ident[2:] if 32 <= b < 127))
    else:
        print("0x35  NO REPLY  (build identity absent: unit is behind b473647)")

    for k in range(a.n):
        if k:
            time.sleep(a.gap)
        rep = txn(pin, pout, [0x7D, 0x3B], 0x3C)
        if not rep:
            print("0x3C  NO REPLY")
            continue
        r = raw_of(rep)
        kg = w(r, 125) if len(r) > 126 else None
        print(f"0x3C  rawlen={len(r):3d}  pass last={w(r,9):6d} us  "
              f"max={w(r,11):6d} us  vcc={w(r,13)} mV  "
              f"cyc/s={w(r,17)*100 if len(r)>18 else '?'}  "
              f"pass/s={w(r,19) if len(r)>20 else '?'}  "
              f"keyToGate max={kg if kg is not None else 'ABSENT'}")


if __name__ == "__main__":
    main()
