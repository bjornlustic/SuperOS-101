# SH-101 firmware behaviour (reverse-engineered, for SuperOS-101)

Derived from the original mask ROM: `SH101-reference/SH101.asm` (Joe Britt's commented
disassembly of `80C49_SH101.bin`, 2 KB). Line numbers below (`asm:NNN`) refer to that file.
This is the behavioural spec our AVR firmware must reproduce. See `docs/HARDWARE.md` for the
pin/port map and RAM layout.

---

## Execution model

- **One big main loop** (`MAIN_LOOP`, `asm:341`) running steps 3–13 of the service-manual
  flowchart, plus exactly **4 leaf subroutines**: `keyboard_read`, `cv_dac_val`, `dac_out`,
  `delay2`. None nest beyond 1 level (2-byte stack).
- **No timer, no counter, no interrupts, no register-bank switching.** The 8049's `T`
  (timer) register is abused as an 8-bit **flag byte** ("T flags").
- **Loop period ≈ 1.5–3.5 ms** (service manual p.3) — set by code volume, *not* delay
  loops (`delay2` is only ~3–5 µs, `asm:1000`). Each pass refreshes all three sample-&-hold
  CVs, so the loop must stay fast enough that the S/H caps don't droop audibly. **Our
  firmware should keep the per-pass rate in the same ballpark (~300–600 Hz refresh).**

### State

- **T flags** (the `T` register, `asm:200–219`): the main mutable state byte.
  - b6 = LFO current level · b5 = LFO H→L edge (1-pass pulse) · b4 = HOLD level
  - b3, b2 = key-scanner state · b1 = function-switch-read state · b0 = arp/seq state
  - b7 = scratch / PORTAMENTO-OFF timing
- **User flags F0/F1** (`asm:189–197`):
  - F1: key priority — **1 = lowest-note** (S7 = LFO or GATE), **0 = last-note**
    (S7 = GATE+TRIG). Reused as the HOLD-requested signal.
  - F0: "found the low/last key" + reused as a general scratch flag.
- **Working regs R0–R7** and the **128-byte RAM map** are documented in `docs/HARDWARE.md`
  (sequencer = `0x1B–0x7F`, key-bank states `0x0A–0x0D`, raw key# `0x0E`, random `0x13`,
  key-shift/range `0x16/0x17`, etc.).

### Numeric model (CONFIRMED exactly)

The DAC is **6-bit (0–63)** and the firmware works in **semitone units**: **1 count =
1/12 V = one semitone, 1 V/octave**, full scale 63 → 5.25 V. Proof from Test Mode
(`asm:266–298`) cross-checked with the service-manual calibration voltages (p.3):

| Button | reg value | ÷12 | manual says |
|---|---|---|---|
| PLAY | `0x21` = 33 | 2.75 V | 2.75 V ✓ |
| DOWN | `0x1E` = 30 | 2.50 V | 2.5 V ✓ |
| U&D | `0x39` = 57 | 4.75 V | 4.75 V ✓ |
| UP (Range) | `0x39` = 57 | 4.75 V | 4.75 V ✓ |

Range octave offsets are 12 counts apart: 16'=`0x0C`, 8'=`0x18`, 4'=`0x24`, 2'=`0x30`
(`asm:407–413`). Octave transpose R6 = `0x00`/`0x0C`/`0x18` (−12/0/+12, `asm:427–435`).

---

## The DAC + mux protocol  ⚠ get this exact

This is the part the new hardware must reproduce bit- and order-faithfully. The 6-bit value
sits on **DB0–DB5**; **DB6/DB7 select the 4052 route** (which S/H cap gets it):

| DB7 DB6 | route | carries |
|---|---|---|
| 0 0 | **CV OUT** | note pitch (KCV) → CV-out jack + VCO pitch |
| 0 1 | **VCO** | range + key-shift (octave offset) → VCO |
| 1 0 | **RANDOM** | random waveform → random-LFO / S&H |
| 1 1 | **disconnected** | settle / scan-safe (no S/H driven) |

`dac_out` (`asm:1366`):
```
dac_out:  cpl a            ; A = ~value  → DB0–5 are ACTIVE-LOW (output one's-complement!)
          orl a,#0C0h      ; force route = 11 (disconnected)
          outl bus,a       ; drive value with NO S/H connected, let ladder settle
          ret              ; returns A = (~value | 0xC0)
```
The caller then connects the target S/H **without changing the data bits**:
```
anl a,#3Fh ; route 00 → CV OUT       (or)
anl a,#7Fh ; route 01 → VCO          (or)
anl a,#0BFh; route 10 → RANDOM
outl bus,a ; same ~value still on DB0–5, now charging the selected S/H
```
…usually followed by `call delay2` for acquisition.

**Two invariants to honour:**
1. **Output the one's-complement** of the logical 0–63 value on DB0–5 (the ladder/buffer is
   active-low). The *logical* value still follows value/12 V.
2. **Route must be `11` (disconnected) during every scan.** All keyboard/switch/button
   reads drive DB as active-low column commons with DB6=DB7=1 (`0xFE/0xFD/0xFB/0xF7`,
   `0xEF`, `0xDF`) — so the garbage ladder voltage never corrupts a held CV. Never connect
   a S/H while a scan column is low.

### Per-loop DAC sequence (3 writes/pass)
1. **RANGE → VCO** (`asm:473–484`): `dac_out(range + keyshift)` then route 01.
2. `keyboard_read` (returns random byte), then **RANDOM** (`asm:495–498`): `dac_out(random)`
   then route 10, `delay2`.
3. `cv_dac_val` (returns note CV), then **CV OUT** (`asm:506–508`): `dac_out(note)`
   then route 00.
4. `update_outputs` (P2: gate/LED/clk-rst/porta).

---

## Step-by-step (one main-loop pass)

**3. RANGE/OCT/ADSR read** (`asm:344–449`): drive `BUS=0xEF` (DB4 low), read P1 (active-low):
RANGE switch → R5 octave offset; OCT up/down → R6; S7 ADSR mode → F1 priority + R6 bit6
(LFO-trigger flag).

**4. RANGE DATA OUTPUT** (`asm:452–484`): R5 + key-shift → VCO S/H (route 01).

**5. KEYBOARD READ** (`keyboard_read`, `asm:545`): march a single `0` across DB0–DB3 to
select each of 4 banks; read 8 keys on P1 each (invert → 1=down); store bank states at
`0x0A–0x0D`. Priority per F1: **low-note** = first key found wins; **last-note** = XOR vs
previous scan to find newly-pressed. Raw key# (0–31) → `0x0E`.

**6. CLOCK CHECK** (`asm:750–793`): read **T1** (LFO/EXT clock). Track level (T-flag b6) and
detect **H→L edge** (T-flag b5, 1 pass). On edge → clock the arp/sequencer + stir random.

**7. RANDOM DATA OUTPUT** (`asm:807–815` + caller): random byte (via a `movp` program-memory
table + accumulation) → RANDOM S/H (route 10).

**8. FUNCTION SWITCH READ** (`cv_dac_val`, `asm:833`): drive `BUS=0xDF` (DB5 low), read P1
buttons (active-low, edge-detected). Handle **KEY TRANSPOSE** (compute key-shift → `0x16/17`)
and **HOLD** (external pedal on **T0** + panel button).

**9/10/11. LOAD / PLAY / ARPEGGIO** (`handle_load asm:1007`, `handle_play asm:1097`, arp
`asm:1155–1357`): see below. Produces the final note CV.

**12. CV OUTPUT** (`asm:506`): note CV → CV OUT S/H (route 00). `cv_dac_val` returns the
value via `add a,#5; ret` (`asm:992`).

**13. GATE & LED OUTPUT** (`update_outputs`, `asm:1381`): build the P2 byte in R6 and
`outl p2,a` (`asm:1490`). Bit meaning (P2 / R6, see `docs/HARDWARE.md` for pins):
GATE = b7 (**inverted**: `anl #7F`=on, `orl #80`=off); function-LED select via P20–P22
(4556) one-hot among LOAD/PLAY/DOWN/U&D/UP; HOLD LED b5; KEY-TRANSPOSE LED b6; CLOCK RESET
P24; PORTAMENTO OFF P23 (gated by T-flag b7).

---

## Sequencer (LOAD / PLAY)

- **Storage:** `0x1B–0x7F` = up to **100 step bytes**, one byte/step. Step byte = 6-bit note
  (0–63) + 2 flag bits (b7/b6) used for **tie/rest/gate**; `0x1B` holds the base/marker.
  End/empty handled via marker bytes; pointer wraps `0x7F → 0x1C` (`asm:1059`, `asm:1128`).
- **LOAD** (`handle_load`): append the current key (or rest/tie) to the step buffer, advance
  the write pointer, manage the flag bits. (Clearing the sequence zeroes `0x1C…` over 0x64
  bytes, `asm:1029–1033`.)
- **PLAY** (`handle_play`): on each LFO/EXT clock H→L edge, advance the read pointer, fetch
  the step, emit note CV + GATE; honour tie/rest flags; loop at end marker.

## Arpeggiator (`asm:1155–1357`)

Walks the held-key bank states (`0x0A–0x0D`) in order. **UP** uses the forward scan
(`X0352`, rlc/inc), **DOWN** uses the reverse scan (`X03A6`, rrc/dec), **U&D** alternates
(T-flag b0 tracks direction). Advances on the clock edge like the sequencer. Emits the
selected key as the note CV.

> The sequencer step-packing and the arp scan inner-loops are intricate and lightly
> commented (the `X0xxx` labels). **When implementing, follow `SH101.asm` directly** for
> these two; the rest of the loop is well understood above.

---

## Reimplementation notes (AVR / SuperOS-101)

- **DB0–5 active-low:** emit `~value` (mask to 6 bits). Logical value = volts × 12.
- **Route = 11 during all scans** — make this structural (e.g. a `scan()` helper that always
  sets DB6=DB7=1), so a held CV can never be corrupted.
- **Ordering per CV:** set data (route 11) → settle → set route (00/01/10) → hold/acquire.
  Measure the real S/H acquisition time on the analog board and make our settle ≥ that
  (original `delay2` ≈ 3–5 µs is just CPU settle; the cap RC dominates).
- **Keep the loop rate ~1.5–3.5 ms** (refresh ≈ 300–600 Hz) so the three S/H caps don't
  droop — don't over-optimise into a much faster or slower loop.
- **T1 clock:** put on an AVR INT/PCINT pin and detect H→L edges (matches T-flag b5);
  **T0 HOLD** on a plain input (active-low/closed = HOLD).
- **GATE is inverted** at P27 (b7=1 → gate off).
- **Sequencer persistence:** the original keeps `0x1B–0x7F` in battery-backed RAM; we move it
  to AVR flash/EEPROM (survives battery removal).
- We can re-implement behaviourally (cleaner, extensible) — we are **not** bound to the 8049
  instruction-for-instruction, only to the pin-level protocol + observable behaviour.

## Implementation status (SuperOS-101 firmware)

Implemented (clean-room, behavioural — see `src/`):
- DAC/mux protocol + scan invariants (`hw.h`), panel RANGE/OCT/ADSR decode (`controls.h`,
  ported from asm step 3), keyboard mono (lowest/last-note), arpeggiator (up/down/u&d),
  and a 100-step sequencer with **AVR-EEPROM** persistence (`engine.h`).

Still stubbed / to refine:
- KEY-TRANSPOSE shift value (`keyShift_`), exact clock-reset + portamento timing, and
  the mode-button latch semantics (verify on HW).
- ~~function-LED 4552 one-hot multiplex~~ **Retired 2026-08-25: it is implemented and
  now measured, not stubbed.** `engine.h ledSelect()` maps the mode to the one-hot
  select and `main.cpp` drives it into the P2 image every pass. Asserted headless in
  the sim self-test by reading the select back through the 0x3B probe (`raw[97]` bits
  0-2): UP -> 5, DOWN -> 3, U&D -> 4, keyboard -> 0. The part is the **4556** (as
  `pins.h`, `HARDWARE.md` and line 132 of this file all say); "4552" here was a typo
  that survived because it was the only copy that disagreed.

## Open questions

- [ ] Confirm the analog **destination of each route** (CV-OUT vs VCO summing) on the
      schematic (p.5/p.7) — naming is from the firmware, not the board.
- [ ] Decode the exact **sequencer step flag bits** (b7/b6 = tie vs rest vs gate) precisely
      (we currently use our own note/rest/tie encoding, not the original byte packing).
- [ ] Confirm **U&D turn-around** behaviour (does it repeat top/bottom notes?).
- [ ] Measure real **S/H acquisition + droop** to set loop rate and settle delay.

---

## CV DAC: MIDI filter cutoff + accent (`src/cvdac.h`, board v1.1+)

Hardware: U5 MCP4822 on PORTA (PA0 ~CS, PA1 SCK, PA2 SDI), 1 LSB = 1 mV. Two
board variants, `hw` in the settings (SysEx 0x25 byte 12, default 1 = bipolar; set 0 on an unreworked first-batch board):

| `hw` | board | VCF path | cutoff range | VCA path |
|---|---|---|---|---|
| 0 | first-batch v1.1 (fabbed 2026-08-20) before rework: R8 100k, R9 4.7k, no bias | additive only, center 0 V, 0.55 throw max | CC 0..127 = 0..+0.55 throw | 4.7k: up to 0.94x envelope |
| 1 | **bipolar (default)**: board v1.2 (R8 27k, R9 2.2k, R10 100k from +15 V), or a v1.1 board with its bias wire on +15 V (R10 68k or 100k) | center = Vref - (V15 - Vref) x 27k / R10: ~2300 mV for 100k at a 5.00 V node, 850 on the first unit (68k, 4.87 V node, measured); -113..+89 % throw with 100k, about -40..+150 % with 68k | CC 64 = slider, 0/127 = ∓/±1 throw | 2.2k: up to 2x envelope |

VOUTA -> R8 -> VCF cutoff summing node (IC18b pin 6, held at the SH-101's +5 V
D/A reference, not 0 V; the v1.1 -5 V bias pinned it and silenced the synth). VOUTB -> R9 ->
BA662 pin 1 (VCA current node). At rest VOUTB is shut down (MCP4822 500k
pull-down) so it takes no envelope current whatever that node's voltage; it is
driven only during an accent, from `vcaPark` (default 700 mV, measured; SysEx 0x25 bytes
12-13) upward. Accent depths are the same in
fader terms on both variants (the unipolar path is scaled by 100/27), clamped
by the DAC's 0.55-throw ceiling on `hw` 0.

Per main-loop pass: `VOUTA = center + cutoff(CC, one-pole glide tau ~16 ms) +
vcfAccent(t)`. Cutoff accepts 14-bit CC (MSB on CC 74, LSB on CC 106); a 7-bit
CC step (32 LSB, ~2 semitones of cutoff) is glided over ~50 ms so sweeps have
no audible stepping, while a full-range jump still lands in under 100 ms.
`VOUTA = ... +`, `VOUTB = vcaPark + env(t) * vcaAmount` (shut down when env = 0). The DAC is only written when
a code changes (~40 us per channel at 4 MHz, bit-banged).

Accent trigger: a fresh note (gate rising, or a non-legato retrigger flagged by
the engine's PORTAMENTO-OFF pulse) whose MIDI velocity >= `velThr`
(default 100), or a sequencer step marked accented (it fires as a velocity-127
note at the step's start, legato steps and each ratchet hit included, and goes
out on MIDI at velocity 127). Steps are marked in LOAD by holding **U&D** while
entering the note (U&D is exempt from the scan-bleed guard while held in LOAD;
its edges still are), with SysEx 0x16 lane 1, or by the editors (the packed
accent tail on 0x11/0x12, 1 bit a step after the ratchets; the flash record
carries it after the scale bytes). Other panel notes carry no velocity; the
`ACCENT_ALL` flag accents every note. Two models, selectable (menu keys 11/12, SysEx 0x25):

| | MC-202 | TB-303 |
|---|---|---|
| envelope | 7 ms one-pole attack, exponential decay `decay10ms` (default 300 ms) | same attack, **fixed 200 ms** decay (the 303 overrides DECAY on accents) |
| VCF | only the part above a 15 % threshold passes (the MC-202's two diodes), max ~0.4 fader throw | env/2 + **build-up** (leaky integrator, +`buildup` per hit, 400 ms bleed = C13), scaled by `resDepth` (stands in for the 303's second RESONANCE pot gang; the SH-101's RES slider is a passive audio attenuator and cannot be read), max ~0.8 fader throw |
| VCA | `vcaAmount` 64 = +1 envelope (the MC-202's 3.3k‖3.3k) | `vcaAmount` up to the cap of **100 = +1.6 envelopes** (the 303's 1k vs 2.2k would be +2.2; capped because BA662 pin 1 is rated ~2 mA and the envelope alone is ~0.78 mA: 0.78 + 1.23 = 2.0 mA at amount 100) |

Settings persist in internal EEPROM 0x010..0x01B (`combined.h`). SysEx 0x23 /
0x24 / 0x25 carry the 11-byte config (`cvdac::Pack` order) plus the live DAC
codes for metering. Headroom note: at cutoff CC near the top the VCF accent
saturates (the filter is wide open there anyway); R8 22k would buy +50 % more
range if ever wanted.
