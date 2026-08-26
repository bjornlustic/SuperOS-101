# G5: the 101's key-scan bound is right, and the instrument that produced it cannot measure it

**I am correcting my own work.** `artifacts/101-baseline.md` (shared repo, cited
by finding 24) states the 101's G5 key-scan bound and marks G5 "done". The number
is approximately right. **The method stated for it cannot produce it**, and the
agreement is a numerical coincidence.

Serves **G5** (this machine's bar is a stated key-scan bound) and **G7**.

## What the artifact says

    :180  "advances 2.5 us per emulated cycle, i.e. the real 6 MHz chip's
           timescale. Key-scan latency is measured in virt, then scaled to the
           4 MHz emulator by the measured throughput"
    :191  "bound roughly 6-15 ms on the emulator"
    :205  "a ~9 ms key-scan response (imperceptible)"
    :247  "G5 done (25.3% + key-scan bound)"

## The sim's virtual clock does not advance per emulated cycle. It never did.

`sim/sim.cpp` has exactly one writer:

    :133  void delayMicroseconds(uint32_t us) { sim::g_virtualUs += us; }

`grep -rn g_virtualUs sim/` returns that line, the declaration, and two readers.
`git log -S "g_virtualUs +=" -- sim/sim.cpp` returns **only the initial commit**,
so this is not drift: the per-cycle advance the artifact describes has never
existed.

**virt is the sum of the settle delays traversed** (`SETTLE_LADDER_US` 10,
`SETTLE_ACQUIRE_US` 100, `SETTLE_SCAN_US` 3, plus `ShRefresh`). Interpretation is
free. That is the one cost G5 exists to bound on this machine.

## Measured, not argued

Same firmware, same ROM, the probe's own counters (`raw[17..20]`, `sysex.h:227`):

| | cyc/s | ROM passes/s | ms/pass |
|---|---|---|---|
| sim (`mode emu`, `run 4000`, `syx 3b`) | **367,800** | **565** | 1.77 |
| hardware, 4 MHz (`artifacts/101-baseline.md:13-14`) | **101,100** | **145** | 6.9 |
| the real 80C49 at 6 MHz | 400,000 | ~300-660 | 1.5-3.5 |

The sim runs the identical interpreter over the identical ROM. It reports **3.6x
the throughput** because virt charges nothing for running it.

**And note which row the sim lands on.** 367,800 cyc/s is 92% of the real chip,
and 1.77 ms/pass sits inside the real chip's 1.5-3.5 ms. So the sim reproduces
**the original 80C49's timescale**, by accident: the settle delays are real and
identical on both, and the term that separates them is the one virt omits. That
is why the artifact's claim looked true. **The number it produced was checkable
and checked out.**

## Why the answer came out right anyway, and why that is the worrying part

The artifact scales the virt latency by the throughput ratio, 1/0.253 = **3.95**.
The true sim-to-hardware pass ratio is 6.9/1.77 = **3.90**. Within 1%, so the
bound lands about right.

It is a coincidence of two unrelated quantities:

* **the settle component does not scale with clock at all.** `delayMicroseconds`
  and `IdleWaitUs` are wall-clock; they are identical at 4, 8 and 16 MHz. The
  artifact scales it anyway.
* **the interpretation component is absent from virt entirely.** It is 5.1 of the
  6.9 ms hardware pass and it is the only part that responds to the clock or to
  the interpreter.

Scaling a term that must not be scaled, by a factor that happens to equal the
ratio of a term that is missing, gives the right answer here and nowhere else.

## The consequence that actually matters

**The instrument is blind to the improvement G5 asks for.** My brief's item 2 is
"the emulator: more efficient". Make the interpreter twice as fast and:

* hardware passes/s rises, key-scan latency falls, G5 improves;
* **the sim's virt does not move by one microsecond**, because virt never counted
  interpretation;
* so the scaled "key-scan bound" does not move either.

A measurement that cannot move when the thing it measures improves is not a
measurement. This is the fleet's characteristic failure (COMMON rule 1, LESSONS
L16) in a form the rule does not currently catch: **not a simulation quoted as a
measurement, but a simulation that agrees with the measurement for the wrong
reason.** The usual defence, "does the number look right", is exactly the defence
that fails here, because the number does look right.

## What the bound actually rests on, and it is sound

The structural part is independent of the sim and survives:

From the ROM, one pass runs the numbered steps in order, and the two that matter
are **step 5 KEYBOARD READ** (`SH101.asm:491`) and **step 13 GATE & LED DATA
OUTPUT** (`:1381`), in that order, once each per pass. `g_passAcc`
(`emu101.cpp:178`) ticks on the step-12 CV-OUT route write, one per pass: the
only other `anl a,#3fh ; outl bus,a` site in the ROM is inside `testloop`, not the
main program.

So, with `f` = the step-5-to-step-13 fraction of a pass:

    key-to-gate latency = (0 to 1 pass of waiting) + f x T_pass
    bound = (1 + f) x T_pass  <  2 x T_pass

`T_pass` is **measured on hardware**: 6.9 ms (145/s). So:

| firmware | T_pass | key-to-gate bound |
|---|---|---|
| emulator @ 4 MHz | 6.9 ms (measured) | **< 13.8 ms** |
| SuperOS @ 4 MHz | 0.83 ms (measured, `101-baseline.md:62`) | **< 1.7 ms** |
| the real SH-101 | 1.5-3.5 ms | < 3.0-7.0 ms |

**The SuperOS firmware beats the original machine. The emulator firmware is ~4x
the original**, which is the 4 MHz divide showing up where the manual says it
would.

**`f` is the only unmeasured term**, and it is the difference between "< 13.8 ms"
and a real number. It should be measured on hardware, not modelled: the emulator
already hooks both events (`mcs48_p1_read` counts keyboard-bank reads,
`mcs48_bus_write` sees the gate write), so it is a probe word, which is what
`CLAUDE.md` and COMMON rule 7 require instead of asking for hands.

## `f` is now a probe word, and it is NOT yet a measurement

`emu101::keyToGateUs(true)`, published at **`raw[125..126]`** of the `0x3B ->
0x3C` probe. It times the ROM's own structure, not a stimulus: the ROM reads the
keyboard (step 5) and writes P2 (step 13) every pass whether or not a key is
held, so **no key injection opcode, no `0x4B`, and no hands are needed**. It is
armed at a keyboard-bank read (DB0..3 low, so the button column cannot arm it)
and disarmed at the first P2 write after it, so the later LED writes in the same
pass cannot stretch it. Max over the last second, because the gate wants a bound
and not a sample.

**Appended at the end deliberately**: every existing probe offset is unchanged,
so offset-indexing readers keep working. That was the failure mode of the `0x21`
config reply (`DEVICES.md` documented five fields where the 101 sends seven, and
`timing/probe101.py` has been printing `ver=1` for a machine running 0.2 ever
since). **It also takes the last free word**: `raw` is now 127 and the
`static_assert` ceiling is 127, so the next addition must change the body
encoding from nibbles to 7-bit packing. That is marked at the declaration.

**What the sim run does and does not establish.** It reads 1878 us against a
mean pass of 1770 us, which confirms the word is wired, armed, disarmed and
published, and that it moves. **It is not a value for `f`.** Two reasons, and the
first is the one this whole document is about:

* the sim's clock is settle-delays-only, which is precisely why it cannot stand
  in for hardware here. **I am not entitled to use it for `f` having just shown
  it cannot measure this class of quantity.**
* 1878 us is a **max** over a second and 1770 us is a **mean**. Their ratio is
  not `f` and the apparent `f > 1` is that mismatch, not a physical result. On
  hardware the same care applies: pair the max with a max.

Verified no regression: self-test 0 failures, `g1_test.py` all green, G12 suite
still 69 pass / 0 fail / 2 skip on the sim transport.

## Status

* **The bound "< 13.8 ms on the emulator, < 1.7 ms on SuperOS" stands**, on the
  hardware-measured pass rate and the ROM's structure. Neither input is the sim.
* **"~9 ms" should not be quoted**: it is the sim number times a factor that has
  no derivation. The honest statement today is the bound, not a point value.
* **G5 for the 101 is NOT done.** It needs `f` measured on the unit. Everything
  else it needs is already measured.
* The sim should either charge virt per emulated cycle or stop being used for
  latency. It remains correct for everything it is actually used for elsewhere
  (state, protocol, the G12 matrix), none of which reads the clock.
