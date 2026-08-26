#!/usr/bin/env python3
"""claim-038 (G13): the 63:39:0 sign test, and the instrument it needs.

THE PREDICTION, and why this shape is worth a bench run rather than another
model. `dac_out` in the mask ROM does `cpl a` (0x3FA, one occurrence in 2048;
`src/hw.h:119` reproduces it), so the R-2R ladder current FALLS as the note
rises and a sag on the +5 V rail (which IS the DAC reference) reads SHARP. The
pitch error from a rail change is therefore proportional to `63 - c`, where c is
the CV, and it VANISHES at the top of the range.

    note 36  -> cv  0  -> 63 - c = 63
    note 60  -> cv 24  -> 63 - c = 39
    note 99  -> cv 63  -> 63 - c =  0

So the shift at three notes should stand in the ratio 63 : 39 : 0. **Nothing is
fitted to the note dependence**, and no common-mode confounder (a clock error, a
capture-rate error, a tuning drift, a temperature ramp) produces a shift that is
large at the bottom, 62 % of that in the middle, and exactly zero at the top. If
it fails, the mechanism is dead in one capture.

THE INSTRUMENT, stated before the result per COMMON rule 1. The claim needs to
resolve a RATIO between shifts of order a few cents, so the pitch estimator has
to be good to well under a cent. It is a long-window interpolated DFT: Hann
window, zero-padded FFT for a coarse peak, then a two-stage fine search on the
true DFT magnitude around it. `--selftest` measures its own repeat spread on the
real signal and prints it, and `--synthetic` checks it against tones of KNOWN
frequency, because a repeat spread only bounds precision and says nothing about
bias.

This deliberately reads NO rail voltage. The AVR's bandgap-against-AVCC read
steps ~22 mV near 5 V and six bench windows in a row came back as two adjacent
codes; that instrument is what stalled claim-038 twice. Pitch is the observable
with the resolution this question needs.
"""
import argparse
import os
import subprocess
import sys
import time

import numpy as np

_USBC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "..", "SuperOS-USBC"))
if not os.path.isdir(_USBC):
    sys.exit(f"cannot find the shared repo, looked in {_USBC}")
sys.path.insert(0, os.path.join(_USBC, "tools"))
sys.path.insert(0, os.path.join(_USBC, "timing"))
import midiport                                            # noqa: E402
import mido                                                # noqa: E402
from scipy.io import wavfile                               # noqa: E402

import machines                                            # noqa: E402

RATE = 48000
CH_101 = machines.ORDER.index("101")          # channel index inside the capture
NOTES = (36, 60, 99)
CV_OF = 36                                    # MIDI_CV_OFFSET; cv = note - 36


def midi_hz(note):
    return 440.0 * 2 ** ((note - 69) / 12.0)


def cents(f, ref):
    return 1200.0 * np.log2(f / ref)


# --------------------------------------------------------------------------- #
# the pitch estimator
# --------------------------------------------------------------------------- #
def _dft_mag(x, t, freqs):
    """|DFT| of x at arbitrary freqs. Chunked so a fine grid cannot blow memory."""
    out = np.empty(len(freqs))
    for i, f in enumerate(freqs):
        out[i] = abs(np.dot(x, np.exp(-2j * np.pi * f * t)))
    return out


def estimate_f0(sig, rate=RATE, expect=None, span=0.06):
    """Fundamental of a steady tone, to well under a cent.

    `expect` and `span` bound the search to +/- `span` (a fraction) around the
    nominal pitch. That is not a fit to the answer: the whole question is a
    shift of a few cents, and span is 6 %, about a semitone, which is 100x the
    effect. It exists to stop the estimator locking onto a harmonic, which is
    the failure mode that would silently halve or double a reading.
    """
    x = np.asarray(sig, dtype=float)
    x = x - x.mean()
    if not np.any(x):
        return None
    x = x * np.hanning(len(x))
    t = np.arange(len(x)) / rate

    n = int(2 ** np.ceil(np.log2(len(x)))) * 8      # zero-pad for a coarse peak
    spec = np.abs(np.fft.rfft(x, n))
    fr = np.fft.rfftfreq(n, 1 / rate)
    if expect:
        lo, hi = expect * (1 - span), expect * (1 + span)
        band = (fr >= lo) & (fr <= hi)
        if not band.any():
            return None
        k = np.argmax(np.where(band, spec, 0))
    else:
        k = np.argmax(spec)
    f = fr[k]

    # Two fine stages on the TRUE DFT. The zero-padded peak is already close;
    # these remove the interpolation error rather than the bin error.
    for half, step in ((fr[1] - fr[0]) * 2, (fr[1] - fr[0]) / 8), ((fr[1] - fr[0]) / 4, (fr[1] - fr[0]) / 200):
        g = np.arange(f - half, f + half, step)
        g = g[g > 0]
        if len(g) < 3:
            break
        f = g[int(np.argmax(_dft_mag(x, t, g)))]
    return float(f)


# --------------------------------------------------------------------------- #
# bench plumbing
# --------------------------------------------------------------------------- #
def capture(path, seconds):
    dev = machines.avf_device()
    cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
           "-f", "avfoundation", "-i", dev, "-t", str(seconds),
           "-af", machines.pan_spec(machines.ORDER, machines.SSL_INPUT),
           "-c:a", "pcm_f32le", "-ar", str(RATE), path]
    subprocess.run(cmd, check=True)
    sr, data = wavfile.read(path)
    return sr, data[:, CH_101] if data.ndim > 1 else data


def steady(sig, rate=RATE, head=0.35, tail=0.10):
    """Drop the attack and the release: pitch is read from the sustain only."""
    a, b = int(head * rate), int(len(sig) - tail * rate)
    return sig[a:b] if b > a else sig


class Bench:
    def __init__(self):
        self.out = mido.open_output(midiport.resolve("101", "out", agent="A101"))

    def note(self, n, on=True):
        self.out.send(mido.Message("note_on" if on else "note_off",
                                   note=n, velocity=100 if on else 0, channel=0))

    def bench_window(self, mode, secs):
        self.out.send(mido.Message("sysex", data=[0x7D, 0x3E, mode, secs]))

    def all_off(self):
        for n in NOTES:
            self.note(n, False)
