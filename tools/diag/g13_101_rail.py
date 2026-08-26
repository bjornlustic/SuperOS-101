#!/usr/bin/env python3
"""claim-038 (G13): is the 101's pitch-rail droop DYNAMIC or STATIC?

The distinction decides whether a SPICE model is worth building at all:

  dynamic  the rail moves WITH CPU load at a fixed clock -> a decoupling and
           source-impedance problem, capacitance can fix it, 16 MHz may be
           reachable with no synth surgery, and the 101's G5 exception goes away
  static   the rail sits lower at a higher clock but does not move with load
           -> a tuning constant, calibrate it out; a DC load line tells us
           everything and SPICE adds nothing

Second edition. The first one RAN and could not answer, for two reasons that
were both properties of the instrument, and both are now fixed in firmware
(89c823a). Kept in the file because a tool that hides why it was rebuilt invites
the next reader to rebuild the first version:

  1. The published number was a MEAN IN MILLIVOLTS, from 8 samples of the 1.1 V
     bandgap against AVCC. Near 5 V one ADC code is 22 mV, which on this machine
     is 39 cents (~57 mV/semitone, measured 2026-08-15), against a question
     about 5 cents. Six windows returned 4984 mV except one 4962: two adjacent
     codes, i.e. dither, not a signal. And no mean at any precision can show
     ripple, which is what "dynamic" MEANS.
     Now: the probe publishes the RAW SET, raw[114..121], sum of 256 codes plus
     min and max. Min/max is the ripple, directly.
  2. The window could not produce the load. 0x3E changes the CLOCK, but the
     SuperOS loop sits in DAC settle waits (~350 us a pass, hw.h) and is not
     CPU-bound, so the 410 mV droop did not reproduce at all. The 2026-08-15
     data says the variable is LOAD and not clock outright: at 8 MHz the
     EMULATOR read 4.96 V while SuperOS read 4.98 V.
     Now: --emu runs the sweep in emulator mode, which is the real load and the
     configuration the 2026-08-15 numbers came from.

And the window itself was wrong about its own clock: it "restored" CLKPR to /1
rather than to the app's /4, so a /2 window left the machine at 16 MHz with the
UART divisor still computed for 4 MHz, and nothing could request 16 MHz at all.
Two of the six rows in the last sweep were therefore labelled with a clock the
machine was not running. The probe now reports the prescale it APPLIED
(raw[122]), and this tool prints that rather than the mode it asked for.

  0x3E <mode> <secs>  bench window:
                      bit0 core /2  -> 8 MHz
                      bit1 USB engine + PLL fully off
                      bit2 idle-sleep at the end of every pass (LOWERS load)
                      bit3 core /1  -> 16 MHz (wins over bit0)
                      mode 0 keeps the app's own 4 MHz: the CONTROL, not a no-op
  0x3B -> 0x3C        probe; the parked set is raw[114..121], live mV raw[13..14]

Read the parked set AFTER the window: during it the core runs at a clock the
UART divisor was not computed for, so MIDI on the wire is garbled. That is why
the firmware parks the samples instead of replying inside the window.

Holding one clock and toggling ONLY bit2 changes the load and nothing else,
which is the comparison the claim turns on.

  python3 tools/diag/g13_101_rail.py           # SuperOS (the low-load control)
  python3 tools/diag/g13_101_rail.py --emu     # emulator: the real load
"""
import sys
import time

import mido

PORT = "SuperOS-101"
MFR = 0x7D
MV_PER_SEMITONE = 57.0          # measured on this unit, 2026-08-15
N_SAMPLES = 256                 # the firmware's parked set size


def send(out, body):
    out.send(mido.Message("sysex", data=(MFR,) + tuple(body)))


def probe(inp, out, timeout=3.0):
    """0x3B -> 0x3C. Returns the decoded raw[] byte list, or None."""
    for _ in inp.iter_pending():
        pass
    send(out, (0x3B,))
    end = time.time() + timeout
    while time.time() < end:
        for m in inp.iter_pending():
            d = list(m.data)
            if len(d) > 3 and d[0] == MFR and d[1] == 0x3C:
                nib = d[2:]
                return [(nib[i] << 4) | nib[i + 1]
                        for i in range(0, len(nib) - 1, 2)]
        time.sleep(0.01)
    return None


def w16(raw, i):
    return (raw[i] << 8) | raw[i + 1]


def code_to_mv(code):
    return (1100.0 * 1024.0) / code if code else 0.0


class RailSet:
    """The parked raw ADC set, and the honest reading of it."""

    def __init__(self, raw):
        self.n = raw[121] * 16
        self.sum = (raw[114] << 16) | (raw[115] << 8) | raw[116]
        self.min = w16(raw, 117)
        self.max = w16(raw, 119)
        self.clkps = raw[122]
        self.mode = raw[123]

    @property
    def ok(self):
        return self.n > 0

    @property
    def mean_code(self):
        return self.sum / self.n if self.n else 0.0

    @property
    def mean_mv(self):
        return code_to_mv(self.mean_code)

    @property
    def lsb_mv(self):
        """One ADC code in millivolts at this rail. The instrument's step."""
        c = self.mean_code
        return abs(code_to_mv(c) - code_to_mv(c + 1)) if c else 0.0

    @property
    def spread(self):
        return self.max - self.min

    @property
    def ripple_mv(self):
        """min/max in millivolts. A LOWER code is a HIGHER voltage, so the
        extremes swap: max code is the minimum rail."""
        return code_to_mv(self.min) - code_to_mv(self.max)

    def clock_mhz(self):
        return {0: 16.0, 1: 8.0, 2: 4.0}.get(self.clkps, 16.0 / (1 << self.clkps))

    def line(self):
        if not self.ok:
            return "NO SET (the window never completed a parked set)"
        if self.spread == 0:
            detail = (f"min==max, so the ripple is BOUNDED under one code "
                      f"({self.lsb_mv:.1f} mV = {self.lsb_mv / MV_PER_SEMITONE * 100:.0f} "
                      f"cents), NOT measured")
        else:
            detail = (f"spread {self.spread} codes = {self.ripple_mv:.1f} mV = "
                      f"{self.ripple_mv / MV_PER_SEMITONE * 100:.1f} cents")
        return (f"{self.mean_mv:8.1f} mV mean over {self.n:3d} samples  "
                f"[{self.min}..{self.max}]  {detail}")


def bench(inp, out, mode, secs=3):
    """Run a 0x3E window, wait it out, then read the parked set back."""
    send(out, (0x3E, mode & 0x7F, secs & 0x7F))
    time.sleep(secs + 1.5)          # window + re-settle at the app's own divide
    raw = probe(inp, out)
    if raw is None or len(raw) < 125:
        return None
    return RailSet(raw)


# (label, mode). bit3 is the explicit 16 MHz; mode 0 is the 4 MHz control.
RUNS = [
    (" 4 MHz  loaded   (app clock, the control)", 0b0000),
    (" 4 MHz  idle     (sleep every pass)      ", 0b0100),
    (" 8 MHz  loaded                           ", 0b0001),
    (" 8 MHz  idle     (sleep every pass)      ", 0b0101),
    ("16 MHz  loaded                           ", 0b1000),
    ("16 MHz  idle     (sleep every pass)      ", 0b1100),
    ("16 MHz  loaded, USB engine off           ", 0b1010),
]


def main():
    emu = "--emu" in sys.argv
    ins = mido.get_input_names()
    if not any(PORT in n for n in ins):
        print(f"FAIL: {PORT} not enumerated. Ports: {ins}")
        print("If it just re-enumerated, run: killall MIDIServer (finding 8)")
        return 1
    name = next(n for n in ins if PORT in n)

    with mido.open_input(name) as inp, mido.open_output(name) as out:
        raw = probe(inp, out)
        if raw is None:
            print("FAIL: no 0x3C reply. Try: killall MIDIServer")
            return 1
        print(f"probe ok, {len(raw)} bytes")
        if len(raw) < 125:
            print(f"STOP: {len(raw)} bytes means this unit predates the rail set "
                  "(89c823a). Reflash before reading anything below as a "
                  "measurement; the old probe cannot answer this question.")
            return 1
        print(f"  live VCC (mean, coarse) : {w16(raw, 13)} mV")
        print(f"  store health            : 0x{raw[124]:02X} "
              f"(bit0 service, bit1 mounted-attempted, bit2 live)")
        print(f"  usb_configuration={raw[6]} boot_count={raw[7]}")
        if emu:
            print("\nSwitching to the 80C49 emulator: SuperOS is not CPU-bound "
                  "(DAC settle waits, hw.h) and cannot produce the aggressor.")
            send(out, (0x2F,))
            print("  sent 0x2F, waiting for re-enumeration...")
            return 2   # caller re-runs after killall MIDIServer; see the note below

        print(f"\nbench windows ({N_SAMPLES} raw ADC samples parked in the last "
              "second of each):")
        res = {}
        for label, mode in RUNS:
            s = bench(inp, out, mode)
            if s is None:
                print(f"  {label}  -> NO REPLY")
                continue
            res[mode] = s
            # Print the clock the machine APPLIED, not the one we asked for.
            print(f"  [{s.clock_mhz():4.0f} MHz applied] {label}  {s.line()}")

        print("\n--- the question: does the rail MOVE with load? ---")
        for clk, loaded, idle in (("4 MHz", 0b0000, 0b0100),
                                  ("8 MHz", 0b0001, 0b0101),
                                  ("16 MHz", 0b1000, 0b1100)):
            a, b = res.get(loaded), res.get(idle)
            if not (a and b and a.ok and b.ok):
                print(f"{clk}: incomplete")
                continue
            d = b.mean_mv - a.mean_mv
            resolvable = a.lsb_mv / (N_SAMPLES ** 0.5)
            verdict = ("DYNAMIC" if abs(d) > max(3.0, resolvable)
                       else "no load dependence resolvable")
            print(f"{clk}: loaded {a.mean_mv:.1f} mV, idle {b.mean_mv:.1f} mV, "
                  f"swing {d:+.1f} mV = {d / MV_PER_SEMITONE * 100:+.1f} cents "
                  f"-> {verdict}")
            if a.spread == 0 and b.spread == 0:
                print(f"       both sets have min==max, so the sum carries no "
                      f"dither and the mean above is NOT better than one code "
                      f"({a.lsb_mv:.1f} mV). Report this as a BOUND.")

        print("\nSTATIC droop, for the record (a transposition, not a wobble):")
        base = res.get(0b0000)
        for clk, mode in (("8 MHz", 0b0001), ("16 MHz", 0b1000)):
            s = res.get(mode)
            if base and s and base.ok and s.ok:
                d = s.mean_mv - base.mean_mv
                print(f"  {clk} vs the 4 MHz control: {d:+.1f} mV = "
                      f"{d / MV_PER_SEMITONE:+.2f} semitones")
    return 0


if __name__ == "__main__":
    # --emu is two runs on purpose: the switch re-enumerates the device and
    # CoreMIDI will not rebind the port without killall MIDIServer (finding 8),
    # which cannot be done from inside an open port. Run:
    #   python3 tools/diag/g13_101_rail.py --emu   (switches, exits 2)
    #   killall MIDIServer && sleep 3
    #   python3 tools/diag/g13_101_rail.py         (sweeps, now in the emulator)
    #   then 0x4D to come back to SuperOS.
    sys.exit(main())
