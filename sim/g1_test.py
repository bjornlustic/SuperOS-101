#!/usr/bin/env python3
"""Automated G1 panel-function harness for the 101 (headless, no hardware).

Drives the sim (which runs the REAL firmware) through the owner's-manual panel
functions and asserts the audible result, so G1 is reproducible instead of a
one-off eyeball. Rule 7: the agents test everything in the computer via firmware.

Run:  ./superos101-sim must be built first (make), then  python3 g1_test.py
Exit 0 = all pass. Each check names the manual function it covers.
"""
import os
import re
import subprocess
import sys

SIM = os.path.join(os.path.dirname(os.path.abspath(__file__)), "superos101-sim")


def run(script):
    """Feed CLI lines to the sim, return the list of [sos]/[emu] state dicts."""
    p = subprocess.run([SIM], input=script, capture_output=True, text=True, timeout=60)
    states = []
    for line in p.stdout.splitlines():
        m = re.search(r"\[(sos|emu)\]\s+cv=\s*(\d+)\s+gate=(ON|off)\s+vco-range=\s*(\d+)"
                      r".*?p2=([0-9A-Fa-f]{2})\s+keys=([0-9A-Fa-f]+)", line)
        if m:
            states.append(dict(fw=m.group(1), cv=int(m.group(2)), gate=(m.group(3) == "ON"),
                               vco=int(m.group(4)), p2=int(m.group(5), 16), keys=int(m.group(6), 16)))
    return states


fails = 0
def check(name, cond, detail=""):
    global fails
    print(f"  {'ok  ' if cond else 'FAIL'}  {name}" + (f"   {detail}" if detail and not cond else ""))
    if not cond:
        fails += 1


# --- 1. keyboard note priority (TRIG=last-note) --------------------------------
# GATE+TRIG = last-note priority: the most recently pressed key sounds.
s = run("mode superos\nrun 20\nsw adsr trig\nkey 2 down\nrun 3\nkey 8 down\nrun 3\nstate\nquit\n")
last = s[-1] if s else {}
# key k -> cv (17 + k at oct-mid default); key 8 -> 25
check("note priority TRIG=last-note (manual: Playing/Keyboard)",
      bool(last) and last.get("gate") and last.get("cv") == 25, str(last))

# --- 2. RANGE switch changes the VCO-range CV ----------------------------------
s = run("mode superos\nrun 20\nsw range 16\nkey 0 down\nrun 6\nstate\nsw range 8\nkey 0 up\nrun 3\n"
        "key 0 down\nrun 6\nstate\nquit\n")
vcos = [st["vco"] for st in s if st["gate"]]
check("RANGE switch alters VCO-range CV (manual: Panel Switches/RANGE)",
      len(set(vcos)) >= 2 or (len(s) >= 2 and s[0]["vco"] != s[-1]["vco"]), str(vcos))

# --- 3. TRANSPOSE shifts one octave (CV +/- 12) --------------------------------
s = run("mode superos\nrun 20\nsw oct mid\nkey 0 down\nrun 6\nstate\nkey 0 up\nrun 3\n"
        "sw oct down\nkey 0 down\nrun 6\nstate\nquit\n")
ons = [st["cv"] for st in s if st["gate"]]
check("TRANSPOSE down = one octave lower (manual: Panel Switches/TRANSPOSE)",
      len(ons) >= 2 and (max(ons) - min(ons)) == 12, str(ons))

# --- 4. arpeggiator UP steps across held keys ----------------------------------
s = run("mode superos\nrun 20\nmidi sync on\nmidi div 6\nbtn up tap\nrun 100\nkey 0 down\nkey 4 down\nrun 3\n"
        "midi start\nmidi clock 6\nstate\nmidi clock 6\nstate\nmidi clock 6\nstate\nmidi clock 6\nstate\nquit\n")
arp_cvs = {st["cv"] for st in s}
check("arpeggiator UP alternates held keys (manual: The Arpeggiator)",
      len(arp_cvs & {17, 21}) == 2, str(sorted(arp_cvs)))

# --- 5. sequencer record + play under clock ------------------------------------
s = run("mode superos\nrun 20\nbtn load tap\nrun 100\nkey 0 down\nrun 2\nkey 0 up\nrun 2\nkey 4 down\nrun 2\n"
        "key 4 up\nrun 2\nkey 7 down\nrun 2\nkey 7 up\nrun 2\nbtn load tap\nmidi sync on\nmidi div 6\n"
        "btn play tap\nmidi start\nmidi clock 6\nstate\nmidi clock 6\nstate\nmidi clock 6\nstate\nquit\n")
play_cvs = [st["cv"] for st in s]
check("sequencer records + plays back the entered notes (manual: The Sequencer)",
      all(v in play_cvs for v in (17, 21, 24)), str(play_cvs))

print()
if fails:
    print(f"{fails} G1 checks FAILED")
    sys.exit(1)
print("all G1 panel checks passed (headless, real firmware via sim)")
