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

# src/midiclock.h DIV_TICKS, integer tick counts at 24 ppqn (never rate-based:
# integer counts are why this machine's division cannot accumulate drift).
#   0 1/4  1 1/4d  2 1/4t  3 1/8  4 1/8d  5 1/8t  6 1/16  7 1/16d  8 1/16t  9 1/32
DIV_TICKS = (24, 36, 16, 12, 18, 8, 6, 9, 4, 3)

# The suite clocks at 1/32: 3 ticks a step is the cheapest division in the
# table, and a sweep case that parks the playhead at step 126 costs 378 clock
# bytes instead of 3024 at the 1/16 default. It is a real table entry, not a
# special value.
DIV_IDX = 9
TICKS_PER_STEP = DIV_TICKS[DIV_IDX]

MIDI_CHANNEL = 0
MIDI_CV_OFFSET = 36
CV_MAX = 63
STEP_NOTE, STEP_REST, STEP_LEGATO = 0, 1, 2

# engine.h edit lanes. 1 and 2 (accent, slide) do not exist on this mono
# machine and must be REFUSED rather than reinterpreted.
LANE_NOTE, LANE_ACCENT, LANE_SLIDE, LANE_GATE, LANE_ALL = 0, 1, 2, 3, 0x7F

# The slot the suite works in. Not 0: pattern 0 is what the machine plays at
# rest and what CORE's G3 capture leaves set to one hit per beat, and a suite
# that silently rewrote it would break the next capture rather than its own run.
SLOT = 6

# Where the `select` edit switches TO. Also not 0, and for a second reason
# beyond leaving CORE's capture pattern alone: `select` changes which pattern
# the sequencer plays, so the time channel's expectation (a wrap every `length`
# steps) becomes a statement about the DESTINATION pattern, not the one the
# case edited. Pointing select at slot 0 made that expectation depend on
# whatever length slot 0 happened to hold: on this bench 3, against the case's
# 16, which reads as a machine fault and is not one. The suite owns both slots
# and sets both lengths, so the expectation is defined.
SLOT_ALT = 5
SLOT_ALT_LEN = 4


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


def model_randomize(steps, length, lane, density, seed):
    """engine.h randomize(). Addressed by LANE since claim-057 retired `scope`.

    Draw order is the part that must match exactly: one draw per step for the
    density decision, plus a SECOND draw only where a pitch is written. A rest
    costs one draw and a note costs two. LANE_GATE writes no pitch, so it is
    one draw per step throughout.
    """
    rnd = xorshift14(seed)
    out = list(steps)
    for i in range(length):
        r = next(rnd)
        on = True if density >= 127 else (False if density == 0 else r < density)
        if lane == LANE_NOTE:                           # pitches only
            if step_kind(out[i]) == STEP_REST:
                continue                                # no second draw
            out[i] = step_byte(next(rnd) & CV_MAX, step_kind(out[i]))
        elif lane == LANE_GATE:                         # legato only, no draw
            if step_kind(out[i]) == STEP_REST:
                continue
            out[i] = step_byte(step_cv(out[i]),
                               STEP_LEGATO if on else STEP_NOTE)
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

    # What this transport can honestly answer for. `persists` is the one that
    # matters: it gates the two tests that would otherwise pass VACUOUSLY off
    # the bench. See SimLink.persists for why that is not a formality.
    kind = "device"
    persists = True
    # Wall-clock cost of one MIDI clock byte. On the wire it is real time,
    # because the device's own divider is what has to see it.
    tick_cost = 0.004

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

    def clock(self, n, per=0.004):
        """n MIDI clock bytes. Real time on the wire, because that is what the
        device's own divider sees."""
        for _ in range(n):
            self.out.send(mido.Message("clock"))
            time.sleep(per)

    def transport(self, msg):
        """0xFA / 0xFC. See Adapter._park_head for why this is not optional."""
        self.out.send(mido.Message(msg))
        time.sleep(0.01)


class SimLink:
    """The same conversation, against sim/superos101-sim instead of hardware.

    Why this exists: the G12 sweep gates G8/G9 for this machine, and the 101's
    hardware lease is shared with three other agents and CORE's audio captures,
    so a suite that can ONLY run on the bench is a suite that runs rarely. The
    sim links the REAL firmware (src/main.cpp, src/engine.h, the actual 80C49
    emulator), so a sweep through here exercises the true OnSysex dispatch and
    the true edit ops, not a stand-in for them.

    What it CANNOT show, said plainly because a green run here is easy to
    over-read: anything electrical, anything about the device's real timing,
    and anything about the flash part. `sim_flash_program` is a host file, and
    `ServicePresent()` is a constant true. So a sim sweep is necessary and NOT
    sufficient, and the adapter reports which transport produced a run.
    """

    kind = "sim"

    # THE PERSISTENCE CHANNEL DOES NOT EXIST HERE, and this flag is the whole
    # reason the class carries one. `ServicePresent()` is a compile-time true
    # in the host build and `sim_flash_program` writes a host file that the
    # process keeps across a reboot, so `store_health` would report a healthy
    # store and `length_survives_reboot` would report a survived length NO
    # MATTER WHAT THE FLASH CODE DID. Two green lines asserting exactly the
    # thing the 606 was wrong about (claim-036), proving nothing.
    #
    # That is A808's trap in another costume: a check written where the
    # mechanism cannot fail switches itself off and still prints a pass. So
    # these two skip here WITH A REASON rather than passing.
    persists = False

    # A tick is a loop pass here, not 1/24 of a beat of real time, so clocking
    # is effectively free and the long-pattern time cases can actually run.
    tick_cost = 0.0

    def __init__(self, exe=None):
        self.exe = exe or os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "..", "sim", "superos101-sim")
        self.p = None
        self._pending = []

    def open(self):
        import subprocess
        import tempfile
        if not os.path.exists(self.exe):
            raise RuntimeError(f"no sim binary at {self.exe}; run `make` in sim/")
        # A PRIVATE EEPROM PER RUN, for two separate reasons.
        #
        # Hygiene: the sim writes `sim-eeprom.bin` into its CURRENT DIRECTORY
        # (sim.cpp EepromPath). The runner lives in SuperOS-USBC, so driving
        # the sim from there drops a stray binary into CORE's shared repo, and
        # driving it from here leaves this tree dirty after every sweep. A
        # dirty tree is not cosmetic in this project: a flash carries the whole
        # tree (LESSONS L12).
        #
        # Correctness, which matters more: a persistent EEPROM makes the sweep
        # depend on what the PREVIOUS sweep left behind. Config and patterns
        # both live there, so a case could pass only because an earlier case
        # set something it never sets itself. That is the same defect as
        # inheriting the bench machine's sync setting, one layer down, and a
        # fresh file per run removes the whole class rather than the instance.
        self._ee = tempfile.NamedTemporaryFile(prefix="sim-eeprom-", suffix=".bin",
                                               delete=False)
        self._ee.close()
        env = dict(os.environ, SIM_EEPROM=self._ee.name)
        self.p = subprocess.Popen([self.exe], stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, text=True, bufsize=1,
                                  cwd=os.path.dirname(self.exe), env=env)
        self._until_ok(greeting=True)

    def close(self):
        if self.p is not None:
            try:
                self.p.stdin.write("quit\n"); self.p.stdin.flush()
                self.p.wait(timeout=5)
            except Exception:
                self.p.kill()
            self.p = None
        ee = getattr(self, "_ee", None)
        if ee is not None:
            try:
                os.unlink(ee.name)
            except OSError:
                pass
            self._ee = None

    def _until_ok(self, greeting=False):
        """Read sim output up to the next [ok], collecting `<- ` SysEx lines."""
        got = []
        while True:
            line = self.p.stdout.readline()
            if not line:
                raise RuntimeError("the sim exited unexpectedly")
            line = line.rstrip("\n")
            if line.startswith("<-"):
                got.append([int(b, 16) for b in line[2:].split()])
            elif line == "[ok]":
                return got
            elif line == "[reboot]":
                # A REBOOT IS ALSO A COMPLETE RESPONSE, and missing this is a
                # hang rather than an error. Any opcode that reboots (0x2F,
                # 0x4D, 0x4A) reaches sim_reboot(), which longjmps to the top
                # of Repl() (sim.cpp:288). That jump leaves the `syx` handler
                # WITHOUT running its puts("[ok]"), so a reader waiting only
                # for [ok] waits forever while the sim sits waiting for the
                # next command: two idle processes, no error, no timeout.
                # Cost 7 minutes of a 45 s suite run before faulthandler
                # named the line.
                return got
            elif greeting and line.startswith("SuperOS-101 sim"):
                # The banner has no [ok]; one prompt-free settle is enough.
                return got

    def drain(self):
        self._pending = []

    def send(self, body):
        hexs = " ".join(f"{b:02X}" for b in body)
        self.p.stdin.write(f"syx {hexs}\n"); self.p.stdin.flush()
        self._pending.extend(self._until_ok())

    def recv(self, opcode, timeout=2.0):
        for i, m in enumerate(self._pending):
            if len(m) >= 2 and m[0] == MFR and m[1] == opcode:
                return self._pending.pop(i)[2:]
        return None

    def collect(self, opcode, seconds=0.0):
        out = [m[2:] for m in self._pending
               if len(m) >= 2 and m[0] == MFR and m[1] == opcode]
        self._pending = [m for m in self._pending
                         if not (len(m) >= 2 and m[0] == MFR and m[1] == opcode)]
        return out

    def clock(self, n, per=0.0):
        # One clock byte per loop pass, which is what the sim's own `midi clock`
        # does; no wall-clock delay, so a 128-step sweep is seconds not minutes.
        if n <= 0:
            return
        self.p.stdin.write(f"midi clock {n}\n"); self.p.stdin.flush()
        self.p.stdin.write("runq 2\n"); self.p.stdin.flush()
        self._pending.extend(self._until_ok())

    def transport(self, msg):
        """0xFA / 0xFC. See Adapter._park_head for why this is not optional."""
        self.p.stdin.write(f"midi {msg}\n"); self.p.stdin.flush()
        self.p.stdin.write("runq 2\n"); self.p.stdin.flush()
        self._pending.extend(self._until_ok())


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

        length carries both boundaries REACHABLE OVER SYSEX: 1 is the minimum
        setLength accepts and the case where every step IS the wrap, and 127 is
        the largest a 7-bit SysEx data byte can carry.

        NOT 128, and this is a real protocol limit rather than a choice:
        engine.h MAX_STEPS is 128 and setLength accepts 1..128, but 0x18's
        length byte is one MIDI data byte, so 128 sets the status bit and
        terminates the message. The top step of this machine's longest pattern
        is unreachable over SysEx, and so is any step index above 127. Logged
        as claim-061; the panel can still reach it, so it is a protocol gap and
        not a dead feature.
        """
        return dict(
            length=[1, 16, 127],
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
        # 0x22 [chan, sync, div]. ESTABLISH the clock config, do not inherit it.
        # Without this the suite silently depended on how the machine happened
        # to be left: the bench 101 has had sync ON since I set it up for CORE's
        # G3 capture, so the device path would have worked by luck while the sim
        # (cold EEPROM, sync OFF) advanced nothing. A test that passes because
        # of a setting another agent made is not measuring what it says.
        self.link.send((0x22, MIDI_CHANNEL, 1, DIV_IDX))
        self.link.recv(0x21)
        # The `select` destination is put in a KNOWN state too, so the time
        # channel after a select is checking a length this suite set rather
        # than one it inherited. See SLOT_ALT.
        self._ack(0x18, SLOT_ALT, SLOT_ALT_LEN)
        for i in range(SLOT_ALT_LEN):
            self._ack(0x16, SLOT_ALT, 0, i, MIDI_CV_OFFSET + i)
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

        AND THE CONVERSE, which is the half I got wrong and which the paragraph
        above talks you straight past: **0x1E is not sufficient either.** The
        two are both necessary. `midiclk::OnRealtime` ignores every 0xF8 unless
        `S().running`, and only 0xFA sets that (midiclock.h:71). So an adapter
        that sends 0x1E and then clocks, as this one did, advances the playhead
        exactly zero steps and every time-channel case reports "0 step reports"
        with the sequencer sitting armed and idle. Knowing the trap in one
        direction is what stopped me seeing it in the other.

        Ticks are not steps. One step is DIV_TICKS[div] clock bytes at 24 ppqn
        (3 at the 1/32 this suite sets), so a step count has to be scaled or
        nothing reaches the next division boundary.
        """
        n = case["length"]
        want = {"start": 0, "mid": n // 2, "wrap": n - 1}[case["head"]]
        self.link.transport("start")                    # 0xFA: clock running
        self.link.send((0x1E, 1))                       # setPlaying: see above
        self.link.drain()
        self.link.clock(want * TICKS_PER_STEP)
        self.link.send((0x1E, 0))
        self.link.transport("stop")
        self.link.drain()
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
            self._ack(0x2E, SLOT, LANE_ALL, 64, 0x39, 0x60)
        elif e == "select":
            self.link.send((0x1D, SLOT_ALT))
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
            steps = model_randomize(steps, length, LANE_ALL, 64, seed)
        elif e == "select":
            active = SLOT_ALT
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

        Skipped for LONG patterns ON THE DEVICE ONLY, and the "only" is the
        point. At 4 ms a tick, two wraps of a 127-step pattern is 765 clock
        bytes and about 3 s per case, which is minutes across the sweep; in the
        sim a tick is a loop pass and the same cases cost nothing. The bound
        that matters is wall clock, so it is read off the transport rather than
        hard-coded, and the long cases run in full wherever they are affordable.

        This matters more than a speed tweak because of a runner bug A808
        reported to CORE and which is still open: **a Skip from check_time
        aborts the WHOLE case**, so a skipped time channel silently throws away
        a state result that passed. Skipping 15 long cases here did not report
        "no time channel", it reported "could not test", and the state coverage
        those cases represent vanished from the count.
        """
        # The wrap interval belongs to the pattern that is PLAYING, which after
        # a select is the destination, not the slot the case edited. Reading it
        # off model()["length"] unconditionally asserted the wrong pattern's
        # length and reported the machine red for behaving correctly.
        n = (SLOT_ALT_LEN if case["edit"] == "select"
             else self.model(case)["length"])
        ticks = (2 * n + 1) * TICKS_PER_STEP
        budget = ticks * self.link.tick_cost
        if budget > 1.0:
            raise _skip(f"length {n} needs {ticks} ticks = {budget:.1f} s on "
                        f"the {self.link.kind} transport; run it in the focused "
                        "suite, not the sweep")
        self.link.drain()
        self.link.transport("start")                    # 0xFA: see _park_head
        self.link.send((0x1E, 1))
        # `ticks` is computed above with the budget: enough for at least two
        # wraps, so there are two anchors to take an interval between, and
        # scaled by the divider because ticks are not steps.
        self.link.clock(ticks)
        seen = self.link.collect(0x15, 0.2)
        self.link.send((0x1E, 0))
        self.link.transport("stop")
        anchors = [i for i, m in enumerate(seen) if len(m) > 1 and m[1] == 0]
        if len(anchors) < 2:
            raise _fail(f"saw {len(anchors)} playhead wrap(s) in "
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
                ad._ack(0x2E, SLOT, LANE_ALL, 64, 0x39, 0x60)
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
            ad._ack(0x2E, SLOT, LANE_ALL, 64, 0x39, 0x60)
            # state_equals diffs DICTS field by field, so the comparison has to
            # be handed named fields, not bare lists.
            ch.state_equals({"steps": ad.model(case)["steps"]},
                            {"steps": ad._read_pattern(SLOT)[1]},
                            what="seeded randomize")

        def randomize_density_bounds(ad, ch):
            """density 0 clears and 127 fills. The two ends are the ones a
            distribution test cannot pin, and they are exact."""
            case = dict(length=16, head="start", legato="off", edit="randomize")
            ad.setup(case)
            ad._ack(0x2E, SLOT, LANE_ALL, 0, 0x11, 0x00)
            got = ad._read_pattern(SLOT)[1]
            ch.uses("state")
            if any(step_kind(s) != STEP_REST for s in got):
                raise _fail("density 0 left notes in the pattern")
            ad._ack(0x2E, SLOT, LANE_ALL, 127, 0x11, 0x00)
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
            # Every value here has to fit in 7 bits or it is not a SysEx data
            # byte at all: it would set the status bit and truncate the
            # message, which is a malformed send rather than a refused edit.
            # That is why there is no "length past MAX_STEPS" case (129 is
            # unsendable) and why the step case uses 127, which is a legal byte
            # and still past a length-16 pattern.
            for body, why in (((0x16, NUM_PATTERNS, 0, 0, 60), "pattern past NUM_PATTERNS"),
                              ((0x16, SLOT, 0, 127, 60), "step past the length"),
                              ((0x18, SLOT, 0), "length 0")):
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
            if not ad.link.persists:
                raise _skip("no persistence channel on the "
                            f"{ad.link.kind} transport: sim_flash_program is a "
                            "host file that survives the reboot regardless, so "
                            "a pass here would be guaranteed and would mean "
                            "nothing. Run with SUPEROS_101_TRANSPORT=device.")
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
            if not ad.link.persists:
                raise _skip(f"ServicePresent() is a compile-time true on the "
                            f"{ad.link.kind} transport, so this test cannot "
                            "fail here and its passing would be an artifact of "
                            "the host build. Run with "
                            "SUPEROS_101_TRANSPORT=device.")
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
        # recv() already strips the manufacturer byte AND the opcode, so `d` is
        # the nibble stream itself. Taking d[1:] here dropped the first nibble
        # and shifted every decoded byte by half a byte, which showed up as a
        # short reply rather than as wrong values.
        return [(d[i] << 4) | d[i + 1] for i in range(0, len(d) - 1, 2)]


def _runner_exc(name):
    """Return the runner's OWN exception class by that name.

    `from suite import Skip` is WRONG here and looks right, which is why A808
    lost time to it and reported it. suite.py executes as `__main__`, so
    importing `suite` loads a SECOND copy of the module: the class the adapter
    raises is then a different object from the one `run_one` catches, the
    `except Skip` handler does not match, and the whole run dies with a
    traceback reading `suite.Skip` next to a handler reading `Skip`. Resolving
    off sys.modules["__main__"] gets the class the runner will actually catch.

    Falls back to a plain import so this file still works when driven directly
    (the __main__ block below) rather than by the runner.
    """
    mod = sys.modules.get("__main__")
    cls = getattr(mod, name, None)
    if isinstance(cls, type) and issubclass(cls, BaseException):
        return cls
    import suite
    return getattr(suite, name)


def _fail(msg):
    return _runner_exc("SuiteFailure")(msg)


def _skip(msg):
    return _runner_exc("Skip")(msg)


def make_adapter():
    """Transport chosen EXPLICITLY by SUPEROS_101_TRANSPORT: `sim` or `device`.

    THERE IS NO AUTODETECT AND THAT IS THE POINT. The previous version fell
    back to the device whenever `SuperOS-101` happened to be enumerated, which
    on this bench it always is. Two ways that is dangerous, both of them
    somebody else's problem rather than mine:

    * the 101's lease is usually held by another agent, and a capture takes all
      26 SSL channels at once, so a sweep that quietly went to the wire would
      land in CORE's exclusive audio capture as a few hundred extra onsets and
      corrupt a measurement nobody would think to re-run;
    * a device run and a sim run prove very different things, and a default
      that picks between them by what is plugged in makes "which one produced
      this result" an accident.

    So: no variable, no run. An explicit choice costs one word and removes an
    entire class of silent wrong answer.
    """
    want = os.environ.get("SUPEROS_101_TRANSPORT", "").lower()
    if want == "device":
        return Adapter(link=Link())
    if want == "sim":
        return Adapter(link=SimLink())
    raise RuntimeError(
        "set SUPEROS_101_TRANSPORT=sim or =device. There is deliberately no "
        "default: the 101 is usually leased by another agent, and a sweep that "
        "guessed 'device' would push MIDI into whatever capture is running.")


if __name__ == "__main__":
    # Offline check of the closed form: no hardware, no runner. Proves the
    # model reimplements the device's rules rather than merely running.
    s = [step_byte(i % 24, STEP_NOTE) for i in range(16)]
    r = model_randomize(s, 16, LANE_ALL, 64, 0x39 | (0x60 << 7))
    assert r == model_randomize(s, 16, LANE_ALL, 64, 0x39 | (0x60 << 7)), "not deterministic"
    assert all(step_cv(x) <= CV_MAX for x in r), "cv out of range"
    assert model_randomize(s, 16, LANE_ALL, 0, 99) == [step_byte(0, STEP_REST)] * 16
    assert all(step_kind(x) == STEP_NOTE for x in model_randomize(s, 16, LANE_ALL, 127, 99))
    st, _ = model_set_step(s, 16, 0, 0, 0)
    assert step_kind(st[0]) == STEP_REST
    _, ack = model_set_step(st, 16, 3, 0, 1)
    assert ack == 2, "legato on a rest must be refused"
    _, ack = model_set_step(s, 16, 1, 0, 1)
    assert ack == 2, "lane 1 is absent on a mono machine"
    g, n, _ = model_set_length(s, 16, 32)
    assert n == 32 and step_kind(g[16]) == STEP_REST
    print(f"closed form self-checks passed ({len(r)} steps modelled)")
