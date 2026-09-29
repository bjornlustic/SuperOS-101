#!/usr/bin/env python3
"""Runs each scene of the guide in the host simulator (sim/superos101-sim, the
real firmware) and reports what the seven panel LEDs show.

A scene is a list of tokens, played on a fresh machine (empty EEPROM and flash):
  d:ID / u:ID / t:ID   press and hold / release / tap (ID = LOAD PLAY DOWN UD UP
                       HOLD KT, or key1..key32, key1 = lowest F)
  w:MS                 wait MS milliseconds of virtual time
  c:N                  N clock pulses on the LFO/EXT CLK line (T1)
  m:start / m:stop     MIDI Start / Stop;  m:clock:N  N MIDI clocks (24 ppqn, ~120 BPM)
  sync:on / sync:off   MIDI clock sync, as the config menu sets it
  sw:oct:up|mid|down   TRANSPOSE switch (H / M / L)
  p:on / p:off         pedal (HOLD jack) pressed / released
A scene starts 500 ms after power-on. Every press and release is followed by 40 ms (the firmware ignores new keys
for 30 ms after button activity, as a human's timing naturally allows).
Keys still held at the end stay held while the LEDs are sampled.

The LEDs are sampled every 16 ms for 1 s from the P2 port image: bits 0-2
one-hot function LED through the 4556 (1 LOAD 2 PLAY 3 DOWN 4 U&D 5 UP),
bit 5 = 0 HOLD lit, bit 6 = 1 KEY TRANSPOSE lit. Lit in every sample = "on",
in a quarter or more = "blink" (the firmware blinks at 128 ms).

    python3 capture.py < scenes.json   # [{name, keys}] -> {name: {LED: on|blink}}
"""
import json
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.join(os.path.dirname(os.path.dirname(HERE)), "sim")
SIM = os.path.join(SIM_DIR, "superos101-sim")
BTN = {"LOAD": "load", "PLAY": "play", "DOWN": "down", "UD": "ud", "UP": "up", "HOLD": "hold", "KT": "ktrans"}
FN = {1: "LOAD", 2: "PLAY", 3: "DOWN", 4: "UD", 5: "UP"}
N, STEP_MS, GAP_MS = 64, 16, 40
MARK = "__sample__"


def commands(keys):
    out = ["wait 500"]                      # power-on settle

    def press(i, act):
        if i.startswith("key"):
            out.append(f"key {int(i[3:]) - 1} {act}")
        else:
            out.append(f"btn {BTN[i]} {act}")
        out.append(f"wait {GAP_MS}")
    for k in keys:
        op, _, arg = k.partition(":")
        if op in ("d", "u"):
            press(arg, "down" if op == "d" else "up")
        elif op == "t":
            press(arg, "down")
            press(arg, "up")
        elif op == "w":
            out.append(f"wait {int(arg)}")
        elif op == "c":
            for _ in range(int(arg)):
                out += ["tick", "wait 60"]
        elif op == "m":
            sub, _, n = arg.partition(":")
            if sub == "clock":
                out += ["midi clock 1", "wait 20"] * int(n)
            else:
                out.append(f"midi {sub}")
        elif op == "p":
            out += [f"pedal {arg}", f"wait {GAP_MS}"]
        elif op == "sync":
            out.append(f"midi sync {arg}")
        elif op == "sw":
            which, _, val = arg.partition(":")
            out.append(f"sw {which} {val}")
        else:
            raise SystemExit(f"bad token {k}")
    out.append("wait 300")
    out.append(MARK)
    out += [f"wait {STEP_MS}"] * N
    out.append("quit")
    return out


def run(keys):
    with tempfile.TemporaryDirectory() as tmp:
        env = dict(os.environ, SIM_EEPROM=os.path.join(tmp, "ee.bin"), SIM_FLASH=os.path.join(tmp, "fl.bin"))
        res = subprocess.run([SIM], input="\n".join(commands(keys)) + "\n", capture_output=True,
                             text=True, env=env, cwd=tmp, check=True)
    tail = res.stdout.split("unknown command", 1)[1]
    p2s = [int(m, 16) for m in re.findall(r"p2=([0-9A-F]{2})", tail)]
    if len(p2s) != N:
        raise SystemExit(f"sim gave {len(p2s)} samples, expected {N}:\n{res.stdout[-2000:]}")
    count = {}
    for p in p2s:
        lit = []
        if p & 7 in FN:
            lit.append(FN[p & 7])
        if not p & 0x20:
            lit.append("HOLD")
        if p & 0x40:
            lit.append("KT")
        for led in lit:
            count[led] = count.get(led, 0) + 1
    return {led: ("on" if n >= N - 1 else "blink") for led, n in count.items() if n >= N / 4}


def main():
    if not os.path.exists(SIM):
        subprocess.run(["make"], cwd=SIM_DIR, check=True, capture_output=True)
    scenes = json.load(sys.stdin)
    json.dump({s["name"]: run(s["keys"]) for s in scenes}, sys.stdout)


if __name__ == "__main__":
    main()
