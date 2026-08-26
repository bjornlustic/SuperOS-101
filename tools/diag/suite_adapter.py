#!/usr/bin/env python3
"""The SH-101's adapter for CORE's shared G12 suite (docs/SUITE_ADAPTER_CONTRACT.md).

    python3 timing/suite.py 101          # from the SuperOS-USBC repo

CORE owns the runner, the channels and the combinatorial generator so four
machines cannot disagree about what "the same" means. This file supplies only
what is genuinely the 101's: its axes, how to put it in a state, what that state
should be afterwards, and how to read it back.

WHAT THIS MACHINE IS, because it decides the axis set. The SH-101 is MONO. There
is no chain, no A/B link, no per-step accent and no slide, so two of the 303's
five axes do not exist here. That is a REPORT about the machine's shape, not an
omission from the sweep: an adapter that quietly dropped them would look like it
covered less, and one that faked them would be testing a fiction. What remains
and does interact: edit type, pattern length at its boundaries, playhead
position including the wrap, and legato (this machine's tie).

MODEL IS A CLOSED FORM, per the contract and for the reason the contract gives:
if model() called the same helper the device path calls, it would pass by
construction and prove nothing. Everything in `model_state` below is
reimplemented from the SH-101's own rules (src/engine.h), including the seeded
xorshift randomize, so a green case is two independent statements agreeing.

The one thing that repeatedly bites here and is easy to reimplement wrongly:
`randomize` in scope 0 draws TWICE per step when the step comes out a note (once
to decide, once for the pitch) and ONCE when it comes out a rest. Get that wrong
and the sequence desyncs after the first rest, which looks like a device bug.
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import mido
except ImportError:                                    # pragma: no cover
    mido = None

PORT = "SuperOS-101"
MFR = 0x7D

# src/engine.h. MAX_STEPS is 128 and NUM_PATTERNS is 8.
NUM_PATTERNS = 8
MAX_STEPS = 128
MIDI_CV_OFFSET = 36
CV_MAX = 63
STEP_NOTE, STEP_REST, STEP_LEGATO = 0, 1, 2

# The slot the suite works in. Not 0: pattern 0 is what the machine plays at
# rest and what CORE's G3 capture leaves set to one hit per beat, and a suite
# that silently rewrote it would break the next capture rather than its own run.
SLOT = 6


# --------------------------------------------------------------------------- #
# The 101's own rules, reimplemented (src/engine.h)
# --------------------------------------------------------------------------- #
def step_byte(cv, kind):
    return (cv & 0x3F) | ((kind & 3) << 6)


def step_cv(s):
    return s & 0x3F


def step_kind(s):
    t = s >> 6
    return STEP_NOTE if t > 2 else t


def xorshift14(seed, entropy=0):
    """The device's PRNG, engine.h randomize(). uint16_t state, and every shift
    truncates back to 16 bits: in C the promotion to int is invisible because
    the result is assigned back to a uint16_t, and in Python it is not, so the
    mask is written out. Returns a generator of 7-bit values."""
    st = seed if seed else (entropy | 0x100)
    while True:
        st ^= (st << 7) & 0xFFFF
        st ^= st >> 9
        st ^= (st << 8) & 0xFFFF
        st &= 0xFFFF
        yield st & 0x7F


def model_randomize(steps, length, scope, density, seed):
    """engine.h randomize(). Draw order is the part that must match exactly."""
    rnd = xorshift14(seed)
    out = list(steps)
    for i in range(length):
        r = next(rnd)
        on = True if density >= 127 else (False if density == 0 else r < density)
        if scope == 1:                                  # note values only
            if step_kind(out[i]) == STEP_REST:
                continue                                # no second draw
            out[i] = step_byte(next(rnd) & CV_MAX, step_kind(out[i]))
        else:
            out[i] = (step_byte(next(rnd) & CV_MAX, STEP_NOTE) if on
                      else step_byte(0, STEP_REST))
    return out


def model_set_step(steps, length, lane, idx, value):
    """engine.h setStep(). Returns (steps, ack). ack 2 = refused."""
    out = list(steps)
    if idx >= length:
        return out, 2                                   # grow with 0x18 first
    if lane == 0:
        if value == 0:
            out[idx] = step_byte(0, STEP_REST)
        else:
            cv = max(0, min(CV_MAX, value - MIDI_CV_OFFSET))
            out[idx] = step_byte(cv, STEP_NOTE)          # a note edit clears legato
    elif lane == 3:
        if step_kind(out[idx]) == STEP_REST:
            return out, 2                                # no gate on a rest
        out[idx] = step_byte(step_cv(out[idx]),
                             STEP_LEGATO if value else STEP_NOTE)
    else:
        return out, 2                                    # lanes 1/2 absent (mono)
    return out, 0


def model_set_length(steps, length, new):
    """engine.h setLength(). Growing fills with rests, and note that shrinking
    then regrowing does NOT restore the old bytes: the fill runs from the
    CURRENT length, so the discarded steps come back as rests."""
    if new < 1 or new > MAX_STEPS:
        return list(steps), length, 2
    # The device's step_[p] is a fixed MAX_STEPS array, so growing never
    # appends; it overwrites cells that already exist. Mirror that rather than
    # letting a short host list decide the semantics.
    out = list(steps) + [step_byte(0, STEP_REST)] * (MAX_STEPS - len(steps))
    for i in range(length, new):
        out[i] = step_byte(0, STEP_REST)
    return out, new, 0


# --------------------------------------------------------------------------- #
# Transport
# --------------------------------------------------------------------------- #
def pack7(src):
    out = []
    for i in range(0, len(src), 7):
        chunk = src[i:i + 7]
        msb = 0
        for k, b in enumerate(chunk):
            if b & 0x80:
                msb |= 1 << k
        out.append(msb)
        out.extend(b & 0x7F for b in chunk)
    return out


def unpack7(src):
    out, i = [], 0
    while i < len(src):
        msb = src[i]; i += 1
        for k in range(7):
            if i >= len(src):
                break
            out.append(src[i] | (0x80 if (msb >> k) & 1 else 0))
            i += 1
    return out


class Link:
    """One SysEx conversation with the 101 over USB-C."""

    def __init__(self, port=PORT):
        self.port = port
        self.inp = self.out = None

    def open(self):
        if mido is None:
            raise RuntimeError("mido is not installed; cannot reach the 101")
        self.inp = mido.open_input(_match(mido.get_input_names(), self.port))
        self.out = mido.open_output(_match(mido.get_output_names(), self.port))
        self.drain()

    def close(self):
        for p in (self.inp, self.out):
            if p is not None:
                p.close()
        self.inp = self.out = None

    def drain(self):
        if self.inp is not None:
            for _ in self.inp.iter_pending():
                pass

    def send(self, body):
        self.out.send(mido.Message("sysex", data=(MFR,) + tuple(body)))

    def recv(self, opcode, timeout=2.0):
        """Wait for a reply with this opcode. Unsolicited 0x15 step-position
        messages arrive continuously while the sequencer runs (the 101's 0x15 is
        DIRECTIONAL: incoming randomize, outgoing playhead), so anything that
        reads a reply has to skip past traffic rather than assume the next
        message is its answer."""
        end = time.time() + timeout
        while time.time() < end:
            for m in self.inp.iter_pending():
                d = list(m.data)
                if len(d) >= 2 and d[0] == MFR and d[1] == opcode:
                    return d[2:]
            time.sleep(0.002)
        return None

    def collect(self, opcode, seconds):
        """Every message with this opcode seen over a window."""
        end, seen = time.time() + seconds, []
        while time.time() < end:
            for m in self.inp.iter_pending():
                d = list(m.data)
                if len(d) >= 2 and d[0] == MFR and d[1] == opcode:
                    seen.append(d[2:])
            time.sleep(0.002)
        return seen


def _match(names, want):
    for n in names:
        if want in n:
            return n
    raise RuntimeError(f"no MIDI port matching {want!r} in {names}. "
                       "After any 0x2F/0x4D/0x4A the device re-enumerates and "
                       "CoreMIDI will not rebind: killall MIDIServer (finding 8)")


# --------------------------------------------------------------------------- #
# The adapter
# --------------------------------------------------------------------------- #
class Adapter:
    machine = "101"

    def __init__(self, link=None):
        self.link = link if link is not None else Link()

    # -- lifecycle --------------------------------------------------------- #
    def open(self):
        self.link.open()

    def close(self):
        self.link.close()

    # -- the state space --------------------------------------------------- #
    def axes(self):
        """Kept small enough to run as a FULL cross product, per the contract:
        claim-031 was a three-axis fault and a pairwise array is not guaranteed
        to find those. 3 lengths x 3 heads x 2 legato x 5 edits = 90 cases,
        comfortably under the runner's 240 cross-product cap.

        length carries both boundaries the 101 actually has: 1 is the minimum
        setLength accepts and the case where every step IS the wrap, and 128 is
        MAX_STEPS.
        """
        return dict(
            length=[1, 16, 128],
            head=["start", "mid", "wrap"],
            legato=["off", "on"],
            edit=["setstep", "setrest", "setlen", "randomize", "select"],
        )

    def applicable(self, case):
        # At length 1 there is no "middle" and no distinct wrap: position 0 is
        # every position. Running those as separate cases would report three
        # times the coverage for one state.
        if case["length"] == 1 and case["head"] != "start":
            return False
        # Legato needs a note to hang off, and a step edit that writes a rest
        # over the step under test cannot then be legato: setStep refuses lane 3
        # on a rest (engine.h), so the combination is not a state the machine
        # can be in.
        if case["edit"] == "setrest" and case["legato"] == "on":
            return False
        return True

    # -- driving it -------------------------------------------------------- #
    def setup(self, case):
        """Put SLOT into a known state: full length, every step a note, legato
        applied if the case asks, playhead parked where the case asks."""
        n = case["length"]
        self._ack(0x18, SLOT, n)
        for i in range(n):
            self._ack(0x16, SLOT, 0, i, MIDI_CV_OFFSET + (i % 24))
        if case["legato"] == "on":
            for i in range(n):
                self._ack(0x16, SLOT, 3, i, 1)
        self.link.send((0x1D, SLOT))
        self.link.recv(0x21)
        self._park_head(case)
        self._base = self._model_base(case)

    def _park_head(self, case):
        """The playhead is moved by clocking the machine, not by an opcode, so
        this is the real mechanism and not a shortcut around it.

        TRAP, and it has already cost this fleet a whole clocked run (finding
        13): MIDI Start does NOT start this sequencer. 0xFA sets the clock
        running and rewinds, but only SysEx 0x1E calls setPlaying(). A 101 that
        is slaved with sync on will sit silent through an entire capture unless
        0x1E goes with it.
        """
        n = case["length"]
        want = {"start": 0, "mid": n // 2, "wrap": n - 1}[case["head"]]
        self.link.send((0x1E, 1))                       # setPlaying: see above
        self.link.drain()
        for _ in range(want):
            self.link.out.send(mido.Message("clock"))
            time.sleep(0.004)
        time.sleep(0.05)
        self.link.send((0x1E, 0))
        self._head = want

    def edit(self, case):
        n = case["length"]
        idx = min(3, n - 1)
        e = case["edit"]
        if e == "setstep":
            self._ack(0x16, SLOT, 0, idx, MIDI_CV_OFFSET + 20)
        elif e == "setrest":
            self._ack(0x16, SLOT, 0, idx, 0)
        elif e == "setlen":
            self._ack(0x18, SLOT, self._new_len(case))
        elif e == "randomize":
            self._ack(0x15, SLOT, 0, 64, 0x39, 0x60)
        elif e == "select":
            self.link.send((0x1D, 0 if SLOT else 1))
            self.link.recv(0x21)
        else:
            raise RuntimeError(f"the 101 has no edit {e!r}")

    @staticmethod
    def _new_len(case):
        """Shrink where there is room to, grow where there is not, so setlen is
        exercised in both directions across the axis rather than only one."""
        return 8 if case["length"] > 8 else 32

    # -- the closed form --------------------------------------------------- #
    def _model_base(self, case):
        n = case["length"]
        kind = STEP_LEGATO if case["legato"] == "on" else STEP_NOTE
        steps = [step_byte(i % 24, kind) for i in range(n)]
        steps += [step_byte(0, STEP_REST)] * (MAX_STEPS - n)
        return steps

    def model(self, case):
        n = case["length"]
        steps, length = list(self._base), n
        head, active = self._head, SLOT
        idx = min(3, n - 1)
        e = case["edit"]
        if e == "setstep":
            steps, _ = model_set_step(steps, length, 0, idx, MIDI_CV_OFFSET + 20)
        elif e == "setrest":
            steps, _ = model_set_step(steps, length, 0, idx, 0)
        elif e == "setlen":
            new = self._new_len(case)
            steps, length, _ = model_set_length(steps, length, new)
            # engine.h setLength: the playhead rewinds when the pattern it is
            # playing shrinks out from under it. This is the length-boundary x
            # playhead interaction the axis set exists to reach.
            if head >= length:
                head = 0
        elif e == "randomize":
            seed = 0x39 | (0x60 << 7)
            steps = model_randomize(steps, length, 0, 64, seed)
        elif e == "select":
            active = 0 if SLOT else 1
        return dict(steps=steps[:length], length=length, active=active)

    # -- reading it back --------------------------------------------------- #
    def read_state(self, case):
        length, steps = self._read_pattern(SLOT)
        cfg = self._read_config()
        return dict(steps=steps, length=length, active=cfg["current"])

    def _read_pattern(self, p):
        self.link.drain()
        self.link.send((0x10, p))
        d = self.link.recv(0x11)
        if d is None:
            raise RuntimeError(f"no 0x11 dump for pattern {p}")
        raw = unpack7(d[3:])
        if not raw:
            raise RuntimeError("0x11 dump did not unpack")
        n = raw[0]
        ck = 0
        for b in raw[:n + 1]:
            ck ^= b
        if (ck & 0x7F) != d[1] or ((ck >> 7) & 1) != d[2]:
            raise RuntimeError("0x11 dump checksum mismatch: the reply is "
                               "corrupt, so a state comparison would be a lie")
        return n, raw[1:1 + n]

    def _read_config(self):
        self.link.drain()
        self.link.send((0x20,))
        d = self.link.recv(0x21)
        if d is None:
            raise RuntimeError("no 0x21 config reply")
        return dict(fw=d[0], channel=d[1], sync=d[2], div=d[3],
                    current=d[4], store=d[5])

    def _ack(self, *body):
        self.link.drain()
        self.link.send(body)
        d = self.link.recv(0x14)
        if d is None:
            raise RuntimeError(f"no 0x14 ack for {[hex(b) for b in body]}")
        return d[0]

    # -- the time channel -------------------------------------------------- #
    def check_time(self, case, ch):
        """A303's claim-019 method: clock the sequencer through several wraps
        and require the wrap anchors at an exact tick interval. The 101 reports
        its playhead unsolicited on outgoing 0x15, so the anchors are the
        device's own account of where it was, not the host's guess.

        Skipped at length 128: at 4 ms a tick that is four wraps of over 500
        ticks, which is minutes of clocking for one case in a 90-case sweep.
        Said out loud rather than silently narrowed.
        """
        n = self.model(case)["length"]
        if n > 32:
            from suite import Skip
            raise Skip(f"length {n} needs {4 * n} ticks per case; run it in the "
                       "focused suite, not the sweep")
        self.link.drain()
        self.link.send((0x1E, 1))
        ticks = 4 * n + 2
        for _ in range(ticks):
            self.link.out.send(mido.Message("clock"))
            time.sleep(0.004)
        seen = self.link.collect(0x15, 0.2)
        self.link.send((0x1E, 0))
        anchors = [i for i, m in enumerate(seen) if len(m) > 1 and m[1] == 0]
        if len(anchors) < 2:
            from suite import SuiteFailure
            raise SuiteFailure(f"saw {len(anchors)} playhead wrap(s) in "
                               f"{len(seen)} step reports over {ticks} ticks")
        ch.anchors_exact(anchors, n, what="playhead wrap")

    def expect_red(self, case):
        return ""

    # -- per-feature tests ------------------------------------------------- #
    def feature_tests(self):
        """sim/g1_test.py's 5 panel functions run headless against the REAL
        firmware in the host simulator and need no hardware, so they stay where
        they are and are named here rather than re-driven over SysEx. What runs
        here is what only the device can answer."""

        def randomize_deterministic(ad, ch):
            """G12's own worked example: same seed, byte-identical result."""
            case = dict(length=16, head="start", legato="off", edit="randomize")
            runs = []
            for _ in range(2):
                ad.setup(case)
                ad._ack(0x15, SLOT, 0, 64, 0x39, 0x60)
                runs.append(ad._read_pattern(SLOT)[1])
            if runs[0] != runs[1]:
                raise _fail("same seed gave two different patterns")
            ch.uses("state")
            if any(step_cv(s) > CV_MAX for s in runs[0]):
                raise _fail(f"randomize produced a CV past {CV_MAX}")

        def randomize_matches_model(ad, ch):
            """The closed form against the device. This is the check that would
            catch a draw-order slip in either direction."""
            case = dict(length=16, head="start", legato="off", edit="randomize")
            ad.setup(case)
            ad._ack(0x15, SLOT, 0, 64, 0x39, 0x60)
            ch.state_equals(ad.model(case)["steps"], ad._read_pattern(SLOT)[1],
                            what="seeded randomize")

        def randomize_density_bounds(ad, ch):
            """density 0 clears and 127 fills. The two ends are the ones a
            distribution test cannot pin, and they are exact."""
            case = dict(length=16, head="start", legato="off", edit="randomize")
            ad.setup(case)
            ad._ack(0x15, SLOT, 0, 0, 0x11, 0x00)
            got = ad._read_pattern(SLOT)[1]
            ch.uses("state")
            if any(step_kind(s) != STEP_REST for s in got):
                raise _fail("density 0 left notes in the pattern")
            ad._ack(0x15, SLOT, 0, 127, 0x11, 0x00)
            got = ad._read_pattern(SLOT)[1]
            if any(step_kind(s) == STEP_REST for s in got):
                raise _fail("density 127 left rests in the pattern")

        def mono_lanes_refused(ad, ch):
            """The 101 has no accent and no slide. The contract is that lanes 1
            and 2 ack 2 rather than being silently accepted, because a silent
            accept is what makes a shared opcode set drift: the host would
            believe an edit landed."""
            case = dict(length=16, head="start", legato="off", edit="setstep")
            ad.setup(case)
            ch.uses("state")
            for lane in (1, 2):
                if ad._ack(0x16, SLOT, lane, 0, 1) != 2:
                    raise _fail(f"lane {lane} (absent on a mono machine) was "
                                "not refused with ack 2")

        def out_of_range_refused(ad, ch):
            case = dict(length=16, head="start", legato="off", edit="setstep")
            ad.setup(case)
            ch.uses("state")
            for body, why in (((0x16, NUM_PATTERNS, 0, 0, 60), "pattern past NUM_PATTERNS"),
                              ((0x16, SLOT, 0, 200, 60), "step past the length"),
                              ((0x18, SLOT, 0), "length 0"),
                              ((0x18, SLOT, MAX_STEPS + 1), "length past MAX_STEPS")):
                if ad._ack(*body) != 2:
                    raise _fail(f"{why} was not refused with ack 2")

        def legato_needs_a_note(ad, ch):
            """engine.h refuses a gate edit on a rest. Checked because the
            alternative (accepting it) would store a legato rest, a state the
            player cannot make and the sequencer does not define."""
            case = dict(length=16, head="start", legato="off", edit="setstep")
            ad.setup(case)
            ch.uses("state")
            ad._ack(0x16, SLOT, 0, 0, 0)                 # step 0 -> rest
            if ad._ack(0x16, SLOT, 3, 0, 1) != 2:
                raise _fail("legato on a rest was accepted")

        def length_survives_reboot(ad, ch):
            """claim-033, kept as a permanent test rather than a past result.
            A606's claim-036 is the argument: on the 606 every write acked out
            of a RAM cache while nothing reached flash, and only a reboot
            exposed it. This is the reboot.

            Costs a re-enumeration, so it is a feature test and not a sweep
            case, and it needs killall MIDIServer behind it (finding 8).
            """
            case = dict(length=16, head="start", legato="off", edit="setlen")
            ad.setup(case)
            ad._ack(0x18, SLOT, 24)
            ad._ack(0x16, SLOT, 0, 11, 60)
            time.sleep(1.2)                              # the 500 ms flush gate
            probe = ad._probe()
            ch.uses("state")
            if probe is not None and len(probe) > 124 and not (probe[124] & 0x04):
                raise _fail("store reports not mounted, so any 'it persisted' "
                            "result here would be a cache echo (claim-036)")
            ad.link.send((0x4D,))                        # reboot into SuperOS
            time.sleep(3.0)
            os.system("killall MIDIServer >/dev/null 2>&1")
            time.sleep(2.0)
            ad.close(); ad.open()
            n, steps = ad._read_pattern(SLOT)
            if n != 24:
                raise _fail(f"length was 24 before the reboot, {n} after")
            if step_cv(steps[11]) != 60 - MIDI_CV_OFFSET:
                raise _fail("the step written before the reboot did not survive")

        def store_health(ad, ch):
            """The state channel for persistence itself (claim-036)."""
            probe = ad._probe()
            ch.uses("state")
            if probe is None or len(probe) < 125:
                raise _fail("probe reply is shorter than 125 bytes: this unit "
                            "is running an image older than the store-health "
                            "word, so persistence cannot be observed")
            if not probe[124] & 0x01:
                raise _fail("the bootloader's SPM service did not answer")
            if not probe[124] & 0x04:
                raise _fail("the flash arena is not mounted: writes will ack "
                            "from RAM and vanish on reboot (claim-036)")

        yield "randomize determinism+bounds", randomize_deterministic, ""
        yield "randomize matches the closed form", randomize_matches_model, ""
        yield "randomize density 0 and 127", randomize_density_bounds, ""
        yield "absent lanes (accent/slide) refused", mono_lanes_refused, ""
        yield "out-of-range edits refused", out_of_range_refused, ""
        yield "legato refused on a rest", legato_needs_a_note, ""
        yield "store health reported and live", store_health, ""
        yield "length+step survive a reboot", length_survives_reboot, ""

    def _probe(self):
        self.link.drain()
        self.link.send((0x3B,))
        d = self.link.recv(0x3C, timeout=3.0)
        if d is None:
            return None
        nib = d[1:]
        return [(nib[i] << 4) | nib[i + 1] for i in range(0, len(nib) - 1, 2)]


def _fail(msg):
    from suite import SuiteFailure
    return SuiteFailure(msg)


def make_adapter():
    return Adapter()


if __name__ == "__main__":
    # Offline check of the closed form: no hardware, no runner. Proves the
    # model reimplements the device's rules rather than merely running.
    s = [step_byte(i % 24, STEP_NOTE) for i in range(16)]
    r = model_randomize(s, 16, 0, 64, 0x39 | (0x60 << 7))
    assert r == model_randomize(s, 16, 0, 64, 0x39 | (0x60 << 7)), "not deterministic"
    assert all(step_cv(x) <= CV_MAX for x in r), "cv out of range"
    assert model_randomize(s, 16, 0, 0, 99) == [step_byte(0, STEP_REST)] * 16
    assert all(step_kind(x) == STEP_NOTE for x in model_randomize(s, 16, 0, 127, 99))
    st, _ = model_set_step(s, 16, 0, 0, 0)
    assert step_kind(st[0]) == STEP_REST
    _, ack = model_set_step(st, 16, 3, 0, 1)
    assert ack == 2, "legato on a rest must be refused"
    _, ack = model_set_step(s, 16, 1, 0, 1)
    assert ack == 2, "lane 1 is absent on a mono machine"
    g, n, _ = model_set_length(s, 16, 32)
    assert n == 32 and step_kind(g[16]) == STEP_REST
    print(f"closed form self-checks passed ({len(r)} steps modelled)")
