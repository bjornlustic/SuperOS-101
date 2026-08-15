# SH-101 hardware notes (for SuperOS-101)

Source of truth:
- `docs/Roland SH101 Service Manual.pdf` (Roland, Nov 1st 1982, First Edition). Page
  references below are to that PDF.
- Reference firmware: `SH101-reference/` — Joe Britt's dump + commented disassembly of
  the original mask ROM (`80C49_SH101.bin`, 2048 bytes; `SH101.asm`). This is the
  authoritative description of the CPU's pin behaviour and is cited as `asm` below.

Goal: a **new replacement CPU board + our own firmware** (a "SuperOS-101"), in the spirit
of SuperOS-303 / SuperOS-606 — but note this is a **clean-sheet design**, not a port (see
below).

---

## Why this is NOT the µPD650 platform

The 303 / 606 / 808 all share the NEC **µPD650** (4-bit µCOM-4). The MSW V0X / OS-303
board emulates *that* chip, which is why one board drops into all three.

The **SH-101 uses a completely different CPU**: IC6 = **Toshiba `TMP80C49P-6-7301`**
(Roland P/N `15179136`) — an **Intel MCS-48 family** part (CMOS **8049**), 40-pin DIP.

| | µPD650 (303/606/808) | TMP80C49 (SH-101) |
|---|---|---|
| Family | NEC µCOM-4, 4-bit | Intel MCS-48, 8-bit |
| Package | 42-pin DIP | 40-pin DIP |
| Internal ROM | mask | **2 KB mask** |
| Internal RAM | — | **128 B** (battery-backed) |
| Clock | — | **6 MHz** ceramic resonator (`CSA 6MHz`) |

**Consequence:** the V0X/OS-303 board does **not** fit, and **no OS-303 firmware logic
ports over**. Only our build/packaging/tooling experience carries. The advantage:
the original firmware is **dumped and fully disassembled** (`SH101-reference/`), so unlike
the 303 we have an exact reference implementation to match.

---

## What the CPU does (it's a small role over an all-analog voice)

The SH-101 is "basically a fully analog synthesizer" (block diagram, p.2). The audio path
is all analog ICs and the CPU never touches it:

- **VCO:** CEM3340 (IC13) · **VCF:** IR3109 (IC14) · **VCA:** BA662A (IC15)
- Sub-osc, noise, LFO, ENV (ADSR), bender — all analog.

The CPU (block diagram p.2; CPU Program p.3, `asm`) only does digital housekeeping:

1. **Key assigner** — scans the 32-key keyboard (4×8 matrix), priority, legato/non-legato.
2. **Sequencer** — up to **100 steps**, stored in internal RAM (battery-backed).
3. **Arpeggiator** — UP / DOWN / U&D.
4. Reads function buttons + RANGE / TRANSPOSE / GATE-TRIG switches.
5. Drives a **6-bit DAC** (R-2R, buffered by IC7 4050) whose output is **demuxed** by
   IC10 (4052) to one of several sample-&-hold lines (CV-OUT, VCO CV, RANDOM).
6. Outputs **GATE**, **CLOCK RESET**, **PORTAMENTO-OFF**, and **function-LED** drive.
7. Reads the **clock** (LFO / EXT-CLK) and generates **random** CV (random LFO / S&H).

Program loop runs every **1.5–3.5 ms** (p.3). Per `asm`: a single main loop + only 4 leaf
subroutines (`dac_out`, `delay2`, `keyboard_read`, `cv_dac_val`); **no timer, no
interrupts, no register-bank switching**. This makes a behavioural reimplementation
very tractable.

---

## Pin map — 40-pin DIP (TMP80C49P)

Physical pinout from the manual (p.3, PIN CONNECTION); functions from `asm`. Standard
MCS-48 control pins that the firmware does **not** use are marked *(free)* — useful as
debug/UART pins during bring-up.

| Pin | MCS-48 name | Dir | SH-101 function |
|----:|---|---|---|
| 1 | T0 | IN | **HOLD** external input (pedal/panel) |
| 2 | XTAL1 | — | 6 MHz ceramic resonator |
| 3 | XTAL2 | — | 6 MHz ceramic resonator |
| 4 | /RESET | IN | reset |
| 5 | /SS | — | *(free)* single-step, tied inactive |
| 6 | /INT | — | *(free)* no external interrupt |
| 7 | EA | IN | =0 → run internal ROM |
| 8 | /RD | — | *(free)* no external memory bus |
| 9 | /PSEN | — | *(free)* |
| 10 | /WR | — | *(free)* |
| 11 | ALE | — | *(free)* |
| 12 | DB0 | I/O | DAC LSB · kbd bank0 select (F3–C4) |
| 13 | DB1 | I/O | DAC · kbd bank1 (C#4–G#4) |
| 14 | DB2 | I/O | DAC · kbd bank2 (A4–E5) |
| 15 | DB3 | I/O | DAC · kbd bank3 (F5–C6) |
| 16 | DB4 | I/O | DAC · **OCT COM** / S3 RANGE COM / S7 ADSR COM |
| 17 | DB5 | I/O | DAC MSB · **BUTTONS COM** |
| 18 | DB6 | OUT | **DAC ROUTE** bit0 |
| 19 | DB7 | OUT | **DAC ROUTE** bit1 |
| 20 | Vss | — | ground |
| 21 | P20 | OUT | LED select (seq/arp) |
| 22 | P21 | OUT | LED select |
| 23 | P22 | OUT | LED select |
| 24 | P23 | OUT | **PORTAMENTO OFF** |
| 25 | /PROG | — | *(free)* expander strobe |
| 26 | Vdd | PWR | **+5 V RAM standby** — battery-backs internal RAM (⚠ confirm vs schematic) |
| 27 | P10 | IN | scan input (see matrix) |
| 28 | P11 | IN | scan input |
| 29 | P12 | IN | scan input |
| 30 | P13 | IN | scan input |
| 31 | P14 | IN | scan input |
| 32 | P15 | IN | scan input |
| 33 | P16 | IN | scan input |
| 34 | P17 | IN | scan input |
| 35 | P24 | OUT | **CLOCK RESET** |
| 36 | P25 | OUT | LED select (HOLD) |
| 37 | P26 | OUT | LED select (KEY TRANSPOSE) |
| 38 | P27 | OUT | **GATE** |
| 39 | T1 | IN | **clock** input (LFO trig / EXT CLK) |
| 40 | Vcc | PWR | +5 V |

> Pin-26 caveat: the manual's pin sheet prints an unusual label here; on every standard
> MCS-48 part pin 26 is **Vdd** (low-power standby supply that keeps internal data RAM
> alive). That is almost certainly how the SH-101 retains the 100-step sequencer on
> battery. **Verify against the power-supply section of the schematic (p.5/p.7) before
> committing the board.**

---

## The multiplexed I/O scheme (this is the part to get exactly right)

Everything hangs off the 8-bit **BUS port (DB0–DB7)** plus the **P1 input port** and
**P2 output port**. There is **no external RAM/ROM** — the BUS is used as GPIO.

### DAC + route demux (DB0–DB7)
- **DB0–DB5 = 6-bit DAC** value (0–63) into the R-2R ladder (IC7 4050 buffer).
- **DB6–DB7 = DAC ROUTE** — which S/H the DAC voltage lands on (IC10 4052):

  | DB7 DB6 | route |
  |---|---|
  | 0 0 | **CV OUT** (the CV-out jack / KCV path) |
  | 0 1 | **VCO** |
  | 1 0 | **RANDOM** |
  | 1 1 | unused |

  Sequence per pass: write 6-bit value → set route → settle → next. Our firmware must
  reproduce the **write/route/settle ordering and dwell** or the S/H pitch voltages drift.

### Keyboard + switch scan (DB as column commons, P1 as rows)
The same DB low bits double as **active-low column commons**; the CPU pulls one low and
reads **P10–P17** (which have ~50 kΩ pull-ups, so a closed switch reads **0**, p. `asm`):

- **DB0..DB3 low, one at a time** → read one **8-key keyboard bank** on P10–P17
  (per bank: lowest key → P10, highest key → P17). 4 banks × 8 = 32 keys.
- **DB4 low** → read **RANGE (S3)**, **ADSR/GATE-TRIG (S7)**, **OCT UP/DOWN** on P1.
- **DB5 low** → read **function buttons** S8–S14
  (P10=LOAD, P11=PLAY, P12=DOWN, P13=U&D, P14=UP, P15=HOLD, P17=KEY-TRANSPOSE).

### Function LEDs (P2 via IC8 4556 dual 2→4 decoder)
P20–P22 drive the 4556 (→ TR17–TR21), and P25/P26 directly drive TR16/TR15 (via TR10),
turning **one** function LED on at a time (LOAD/PLAY/DOWN/U&D/UP mutually exclusive,
plus HOLD, KEY-TRANSPOSE). Full truth table in `asm` lines ~90–128.

### Gate / clock / portamento (P2 direct)
- **P27 = GATE**, **P24 = CLOCK RESET**, **P23 = PORTAMENTO OFF**.

---

## Internal RAM / sequencer layout (`asm` lines ~131–170)

128 bytes, indirect via R0/R1. Bank-1 working regs are **not used**.

| Addr | Use |
|---|---|
| `0x00–0x07` | R0–R7 (bank 0). R6 = OCTAVE (DOWN=0x00/NORM=0x0C/UP=0x18); bit7 of `0x07` = GATE state |
| `0x08–0x09` | call/return stack (2 levels max used) |
| `0x0A–0x0D` | music-keyboard banks 0–3 (current key states) |
| `0x0E–0x17` | arp/seq state, last random number, key-shift/range, octave+LFO-ADSR trig |
| `0x18–0x1A` | (bank-1 regs region, unused) |
| `0x1B–0x7F` | **100 sequencer steps** (≈1 byte/step) |

Implication for our design: the sequence is just ~101 bytes. We can keep it in our MCU's
own **flash/EEPROM/FRAM** (survives battery removal — a strict upgrade over the original
Vdd-standby scheme) instead of relying on the battery-backed RAM standby pin.

---

## Test Mode & calibration (must preserve)

Hold **LOAD + KEY-TRANSPOSE** at power-on → Test Mode (p.3). The CPU forces specific
KCV / RANGE voltages and GATE states per function button (p.3 table), which the service
**D/A Converter / VCO / VCF adjustment** procedures (p.4) depend on (VR-1 +5V, VR-2 D/A
Tune, VR-3 D/A Linear, VR-5 Range Width, VR-7 VCO Tune, etc.). **Our firmware should
reproduce Test Mode exactly**, or the documented bench-calibration procedure breaks.

Test-mode voltages to reproduce (p.3):

| Button | KCV | Range | Gate |
|---|---|---|---|
| PLAY | 2.75 V | 0 V | Off |
| ARP DOWN | 2.5 V | 0 V | Off |
| ARP U&D | 4.75 V | 0 V | On |
| ARP UP | 0 V | 4.75 V | On |
| LOAD | 0 V | 0 V | Off |

---

## Replacement-board plan (Path 2: our MCU + our firmware)

1. **Form factor.** Desolder the original 80C49 (single-sided through-hole — confirmed
   easy, per Tubbutec) and fit a **40-pin DIP socket**. Our board is a small PCB with a
   DIP-40 pin-header footprint that plugs into that socket. (Proven viable: Tubbutec
   SH-1oh1, Din Sync RE-101's "101 CPU".)
2. **MCU: AT90USB1286** — same part as SuperOS-303/606 (see next section). A new DIP-40
   mainboard is required (the 303 board is a DIP-42 µPD650 footprint), but the breakout
   (FFC → USB + TRS MIDI) and Tag-Connect ISP carry over, and the firmware's whole-port
   driver style maps directly. The AVR's **5 V** I/O is a perfect match for the SH-101's
   5 V CMOS ladder/4556/4052/pull-ups — **no level shifting**.
3. **Clock.** We don't need the 6 MHz resonator electrically — the AVR runs its own
   on-board crystal; socket pins 2/3 (XTAL1/2) go unused. Reproduce *behavioural* pin
   timing (the 1.5–3.5 ms loop, DAC settle dwell) in firmware.
4. **Sequencer persistence.** Use on-MCU NVM (flash/EEPROM/FRAM); decide whether to keep
   wiring Vdd/pin-26 standby at all.
5. **Must-reproduce behaviours.** DAC value→route→settle ordering; keyboard/switch scan
   timing & active-low sense; LED multiplex; GATE/CLOCK-RESET/PORTAMENTO outputs; T0 HOLD;
   T1 clock detection + random generation; **Test Mode**.

---

## MCU & pin mapping — AT90USB1286 (DECIDED)

Same MCU as SuperOS-303/606. The 80C49 has three 8-bit groups, which map one-to-one onto
the same three AVR ports the 303 firmware already uses for its three 8-bit groups — so the
whole-port read/write driver style transfers; only the *meaning* of each port changes.

| 80C49 function | bits | dir | AVR port | 303 used this port for |
|---|---|---|---|---|
| DB0–DB7 (6-bit DAC + DB6/7 route + DB0–5 column commons) | 8 | OUT | **PORTC** | CV data out |
| P10–P17 (keyboard/switch scan rows) | 8 | IN | **PORTB** | switch-matrix read |
| P20–P27 (LED selects, GATE, CLOCK-RESET, PORTA-OFF) | 8 | OUT | **PORTF** | PH selects + PG LEDs |
| T0 (HOLD in) | 1 | IN | PORTD spare | — |
| T1 (clock in) | 1 | IN | PD0/PD1 (INT-capable for edge detect) | — |
| MIDI (new feature) | 2 | UART | PD2/PD3 | MIDI |
| ISP | 3 | — | PB1/2/3 (shared, programming only) | ISP |

Note: BUS (DB0–7) is **output-only** in this firmware — DAC value, route, and the active-
low scan column commons are all writes (`OUTL BUS,A`); the keyboard/switch *reading* is on
P1. So PORTC = pure output, no bus-turnaround handling needed.

### Fuses
`lfuse 0xDF, hfuse 0xDA, efuse 0xF8`. Only lfuse differs from the 303/606 (CKOUT off,
because CLKO is PC7 = DB7 here); see `[env:fuses]` in platformio.ini.

### Board-level deltas vs the SuperOS-303 board
- **New DIP-40 footprint** (303 is DIP-42). Reuse the FFC breakout + Tag-Connect ISP.
- Power from socket **pin 40 (Vcc) / pin 20 (Vss)** (303 took +5 V from J4.21).
- Socket **pins 2/3 (XTAL1/2) unused** — AVR has its own crystal; leave the 6 MHz
  resonator pins unconnected.
- **Pin 26 (Vdd RAM-standby):** sequence persistence moves to the AVR's flash/EEPROM
  (survives battery removal). Just tie/accept +5 V here; don't depend on RAM standby.

## Bring-up plan (diagnostic-first, like SuperOS-606)

The SH-101 has **no MIDI**, so we can't report over MIDI as the 606 did. Options:
- **Debug UART on the free control pins** (RD/WR/PSEN/ALE/PROG/INT/SS — none used by the
  app) while developing on the bench.
- **Report via the function LEDs** and/or the **CV-OUT jack** (drive known DAC values and
  measure) once in-socket.
- Develop on a **breakout** first (MCU + DIP-40 header) before trusting the real unit.

Suggested probe stages: (1) read keyboard+switch matrix → confirm row/column/active-low
map on a real unit; (2) drive the DAC at each route and measure CV-OUT/VCO/RANDOM S/H;
(3) GATE/CLOCK-RESET timing; (4) function-LED multiplex; (5) Test-Mode voltages vs p.3.

---

## Open questions / TODO

- [x] **MCU = AT90USB1286** (5 V → no level-shifting; whole-port mapping mirrors the 303).
- [x] **Sequencer NVM** = AVR flash/EEPROM (don't rely on battery-backed RAM standby).
- [ ] Confirm **pin 26 = Vdd RAM-standby** and how the battery backs it (schematic p.5/7).
- [ ] Confirm whether **EA (pin 7)** is hard-tied low on the board.
- [ ] **Current budget:** verify the SH-101 +5 V rail (S1671140 DC/DC) has headroom for
      the AVR (more than the original CMOS 80C49). Evidence it is marginal (2026-08-15):
      with the BOD fuse at 4.3 V (efuse 0xF8) every SPM flash write and the USB PLL
      start reset the chip (BORF via SysEx 0x3B); at 3.5 V (efuse 0xF9, now the
      standard) it is clean. Meter VCC at the AVR with USB plugged and during a
      pattern save; consider more bulk capacitance on the CPU board if it sits
      below ~4.6 V.
- [ ] Measure the real **DAC settle time** the analog ladder/4052 S/H needs.
- [ ] Lay out the **new DIP-40 mainboard** (reuse FFC breakout + Tag-Connect ISP).
- [x] Read `SH101.asm` in full → behaviour captured in [`docs/FIRMWARE.md`](FIRMWARE.md)
      (DAC/mux protocol, numeric model, loop steps, seq/arp). Arp + seq-packing inner-loops
      still to be copied verbatim from the asm at implementation time.
- [x] **USB-C breakout** carries the CC pull-downs (C-to-C and C-to-A hosts both
      supply VBUS; confirmed working). UVCC is on the board +5 V and VBUS goes to
      the AVR's VBUS pin only, so the port neither powers nor back-feeds the unit;
      the firmware's VBUS gate (src/usb_gate.h) runs the USB engine only with a
      cable in.
- [ ] **USB engine noise on the panel nets:** the 606 saw the 48 MHz USB clock
      charge-pump floating status nets behind diodes. On the 101, P1 rows sit on the
      AVR's internal pull-ups and the LEDs are static (no scan-multiplexed LEDs), so
      no clamp scheme is expected to be needed. Bench 2026-08-15 with a cable in: no
      phantom keys/buttons observed over 30 firmware swaps and 3 update cycles (the
      config menu combo never fired); still worth a keyboard/T0/T1 pass by hand.
- [ ] Verify the **MC-202 relationship** (shares the analog voice; different/larger digital
      section — likely a *different* CPU, so probably not a shared board like the µPD650 trio).
