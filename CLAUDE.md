# SuperOS-101: what a session here needs to know before it starts

Facts about **this machine** that have already cost someone a measurement. Each
one is here because it was discovered the expensive way, or because it is
routinely assumed wrong. Verified against the source at the time of writing; if
one contradicts the code, the code wins and this file is the bug.

Fleet context lives in `~/Desktop/Projects/SuperOS-USBC/`: `coord/GOAL.md` (the
gates), `docs/FINDINGS.md` (measured, permanent), `coord/LESSONS.md` (what the
fleet got wrong), `coord/briefs/A101.md` (this machine's brief).

## The traps

**MIDI Start DOES start the sequencer now, but only with sync ON and only from
idle.** Since 2026-08-27 sync ON is strict transport slaving: `0xFA` calls
`playFromHost()` (SuperOS) or taps the ROM's PLAY button (80C49 mode), `0xFC`
stops it, and with sync ON nothing but the divided clock ever steps the machine
(no LFO fallback; a silent wire = a silent machine, and a local PLAY press only
ARMS it). The old trap survives in two forms: with sync OFF, `0xFA` still only
rewinds (finding 13 behaviour), and `0xFA` never interrupts an active arp or
LOAD mode. `timing/sync_test.py` still sends `F0 7D 1E 01 F7`, which remains
correct in every mode.

**`0x15` is directional, and `EDIT_OPCODES.md` is wrong about it.** The shared
spec lists `0x15` as free for RANDOMIZE. On the 101 it is a get/set pair:
**incoming** `0x15` is randomize, **outgoing** `0x15` is the sequencer step
position, emitted on every step change while playing (`main.cpp SendStep`). A
host reading the 101 under clock will see unsolicited `0x15` traffic that is not
a reply to anything. Do not treat it as a protocol error.

**`0x13` is REQUEST-ALL here, not SET STEP.** SET STEP is `0x16`. CORE's first
opcode draft was 808-based and said `0x13`; it was withdrawn precisely because
`0x13` is request-all on both the 101 and the 303. If a doc tells you to
length-dispatch `0x13`, that doc is stale.

**Re-enumeration needs `killall MIDIServer` on this host.** After any `0x2F`,
`0x4D` or `0x4A` the device re-enumerates and CoreMIDI will not rebind the port,
sometimes for over 30 s. It is a host cache, not a firmware fault (finding 8),
and it has eaten hours of diagnosis across this fleet.

## Things that look like defects and are not

**8 MHz (was 4) is a supply-rail limit, not power, not USB, not EMI.** The SH-101's
+5 V rail is the pitch DAC's reference, so it *is* the pitch scale, and its small
discrete regulator has a knee. Re-measured 2026-09-30 after the R63 ground strap
(0x3E bench + 0x3B probe, and pitch recorded through the MPC's Input R): 4 and
8 MHz both hold 4855 mV and pitch within 0.3 cents, even in the CPU-bound
emulator; 16 MHz sags to ~4570 mV with a 90 mV swing (+350 cents, sd 71 cents).
The ground strap cannot change that: it lowers ground-return resistance, not the
regulator's current capacity. 16 MHz needs the CPU board on its own supply.
Every other machine in the fleet divides for a different physical reason (the
606 for EMI, the 808 not at all), so "why is this divided" has four answers.

**The emulator running below real time is ACCEPTED, not a bug.** G5 was
rewritten by the human specifically for this machine: the 80C49 interpreter is at
its computed-goto ceiling with the hot loop already native (~25 % of real time at
4 MHz, 171k cycles/s ~ 50 % at 8 MHz since 2026-09-30), and the clock is the rail's
limit (above), not slack code.
The same interpreter clears real time at 16 MHz. The 101's G5 bar is instead that
the musical path is unaffected and key-scan response stays inside a stated bound.
Do not "optimize" this; measure the bound.

**This machine keeps its own bootloader.** claim-005 is withdrawn. There is no
fourth boot chord, no shared-bootloader port, and reading `BOOT_MAP.md` means you
are in the wrong job. Its bootloader is **ISP-only for flashing** and the bench is
USB-C, so bootloader changes cannot be tested headless and G11 is deferred on that
ground, not on difficulty.

## Shape of the machine, for anyone writing tests

Mono. **No chain, no A/B link, no slide.** Lane 2 of the shared edit protocol
does not exist here and acks status 2 rather than being silently accepted. Lane 1
(**accent**) exists since 2026-09-30 (the CV DAC mod): 0x16 lane 1 sets it (ack 2
on a rest), RANDOMIZE still refuses it, and 0x11/0x12 and the flash record carry
a packed ceil(len/8)-byte accent tail after the ratchet tail (flash: after the 2
scale bytes). Lane 4 is the per-step **filter CV** (0..127), which is NOT in the
pattern record or 0x11: it lives in its own flash blocks (FB_FILTER_BASE + p) and
travels as 0x2C (sent after every 0x11) / 0x2D. So the G12 combination axes that apply are length boundary,
legato/tie, accent, playhead position and edit type; chain and A/B are a
*report*, not an omission, when you describe coverage.

**The accent lasts one note.** It ends with the note's gate or when the next step
starts unaccented (the MC-202's CPU accent line), 7 ms release. A test expecting
an accent tail into the next step is testing the pre-56a13bc bug.

**Flash the 101 over DIN, not over the MPC's USB.** That link drops bootloader
messages; `AkaiMPCLIVE2/tools/device/sh101_update.py --din` sends through MPC
Port A and fails unless the build identity (0x35) changed.

**MIDI OUT vs THRU is a setting, not a cable.** 0x65 id 11 (0x66's 12th field,
panel menu key 32, saved): THRU mirrors MIDI IN to OUT and suppresses the 101's
own notes. A host that sees no notes from the 101 should check this first, and
then `0x65 id 12` (notes out: DIN+USB / DIN / USB / off).

`NUM_PATTERNS` 8, `MAX_STEPS` 128, `MIDI_CV_OFFSET` 36 (cv = note - 36, clamped
0..63). Edit lanes: 0 = note (value is the MIDI note, 0 = rest), 3 = gate (0
normal, 1 legato). Per-step **ratchets** (2 bits: 0 = single, 1 = 2x, 2 = 3x)
exist since 2026-08-26: SysEx `0x1B <p><step><val>`, panel UP/DOWN in LOAD mode
cycles the last recorded step, and `0x11`/`0x12` and the flash record carry a
packed ceil(len/4)-byte ratchet tail after the step bytes. The tail is optional
on input (old hosts omit it, checksum is over what was sent) but always present
on output, so a host that hard-codes the old `1+len` payload size will misparse
`0x11`.

**But SysEx cannot reach step 128 or length 128** (claim-061). `setLength` accepts
1..128 and `MAX_STEPS` is 128, yet the length and step-index fields of `0x18` and
`0x16` are each a single MIDI data byte: 128 sets the status bit and truncates the
message. So the top step of the longest pattern is panel-only, and any test that
writes 128 is sending a malformed message rather than exercising a boundary. Use
127 as the largest reachable index. This is a protocol gap, not a dead feature.

**P2 has two inversions and they are easy to get backwards.** GATE (P27) is **0**
when the gate is ON. HOLD (P25) is **0** when its LED is LIT. KEY-TRANSPOSE (P26)
is **1** when lit (an extra inverter, TR10). CLOCK RESET (P24) idles high and is
pulsed low. P20-22 are a one-hot function-LED select through IC8, a **4556**:
0 none, 1 LOAD, 2 PLAY, 3 DOWN, 4 U&D, 5 UP.

**Pattern length is stored twice**, in the flash record header (`FE_OFF_LEN`) and
in the payload `buf[0]`, and `loadAll` reads it from `buf[0]`. That redundancy is
why the 101 is immune to the 606's length-loss bug (claim-029/claim-033). Do not
"tidy" it away without reading that claim.

## How to test without hands

Everything here is headless and takes one command:

    cd sim && make && ./superos101-sim --test && python3 g1_test.py

The sim links the **real** firmware (`src/main.cpp`, `src/engine.h`, the actual
emulator), not a model of it, so a self-test failure is a firmware failure. It
covers the panel, the engine, MIDI, SysEx, the 80C49 emulator against the genuine
mask ROM, the G4 opcodes and a G12 combination matrix.

On hardware, the `0x3B -> 0x3C` probe is the state channel: it reports the reset
cause, USB registers, loop timing, VCC in mV (the pitch rail, per the 4 MHz note
above), the emulator's throughput counters, and since `e3e3336` the SuperOS P2
image and per-bit LED/gate transition rates. If you need to observe something on
hardware that it does not report, **add a word to the probe** rather than asking
for hands: that is what rule 7 in `coord/COMMON.md` requires.

## House rules that have bitten here specifically

**Check `git status` before building anything you intend to flash.** This repo has
had another session's uncommitted work sitting in the tree, and a flash carries
the whole tree, not your diff. A hardware result attributed to your change is
wrong if the image also contained someone else's (`coord/LESSONS.md` L12).

**An emulator change that is not compared in recorded audio before and after is
not done.** The commits under `8c0a865` set that standard here and it is higher
than the other machines'. Every signal moved behind a queue must stop being
observable raw at the same instant, or you reproduce `8c0a865`'s double-triggered
steps.

**Do not replace the MIDI clock divider table with anything rate-based.** It is
integer tick counts at 24 ppqn (`midiclock.h`) so division cannot accumulate
error. That is deliberate.
