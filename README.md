# SuperOS-101

Open firmware + a replacement CPU for the **Roland SH-101**, running on an
**AT90USB1286** (Teensy++ 2.0 silicon) — the same MCU as SuperOS-303 / SuperOS-606.

**This is a clean-sheet design, not a port.** The SH-101 CPU is a Toshiba
**TMP80C49P** (Intel MCS-48 / 8049), *not* the NEC µPD650 shared by the TB-303 /
TR-606 / TR-808. So the V0X / OS-303 board does **not** drop in and none of the OS-303
firmware logic applies. What carries over: the AT90USB1286, the breakout (FFC → USB +
TRS MIDI), the Tag-Connect ISP, the PlatformIO toolchain, and the whole-port driver
style. The advantage over the 303 work: the original SH-101 mask ROM is **fully dumped
and disassembled**, so we re-implement against a known-good reference.

## Status

🟡 **Bring-up.** Combined firmware builds (≈12.7 KB flash): **SuperOS** plus an
**80C49 emulator mode** that runs the original Roland mask ROM (see below), selected
per boot from a config menu. The SuperOS side: faithful DAC/mux protocol and scan
invariants, **panel decode** (RANGE/OCTAVE/ADSR, ported from the disassembly), a
**mono keyboard** (lowest/last-note priority), an **arpeggiator** (up/down/up&down),
a **100-step sequencer** with EEPROM persistence, and **MIDI clock sync**
(`src/midiclock.h`): the arp/sequencer clock, LFO hard-sync via CLOCK RESET, and a
beat-stepped random S&H, at ten divisions (1/4 … 1/32 incl. dotted/triplet), in both
firmwares (the emulator gets the divided clock on its T1 pin). Remaining stubs: the
KEY-TRANSPOSE shift value, the function-LED 4552 multiplex, and exact
clock-reset/portamento timing. The CPU board (SKiDL netlist) lives in
[`../SuperOS-101CPU`](../SuperOS-101CPU) — PCB layout still to do.

## Config menu & 80C49 emulator mode

Like the SuperOS-303 combined build (SuperOS + D650C emulator), one image carries
two firmwares; internal-EEPROM byte 0x000 selects which runs at boot (map in
[`src/combined.h`](src/combined.h)):

- **SuperOS** — the clean-sheet firmware above.
- **80C49 emulator** — an MCS-48 interpreter ([`src/mcs48.h`](src/mcs48.h)) running
  the original 2 KB SH-101 mask ROM. The AVR ports map 1:1 onto the 80C49 socket,
  so the emulated BUS/P1/P2/T0/T1 pass straight through ([`src/emu101.cpp`](src/emu101.cpp)):
  the original scan patterns, active-low DAC writes, and settle loops hit the analog
  board unchanged. Paced to the original 6 MHz resonator (400 k machine cycles/s).
  The original's power-on Test Mode (LOAD + KEY-TRANSPOSE) works in this mode.

**Config menu**: hold **HOLD + KEY-TRANSPOSE for 2 s** (while running, either mode)
or at power-on. In the menu the HOLD LED lights, the gate is forced off, and the
function LED shows the running firmware (LOAD lit = emulator, PLAY lit = SuperOS).
Then: **LOAD** = reboot into the emulator, **PLAY** = reboot into SuperOS,
**UP&DOWN** = toggle MIDI clock sync (KEY-TRANSP LED lit = on), **UP**/**DOWN** =
cycle the clock division, the 10 lowest keys = pick the division directly
(1/4, 1/4d, 1/4t, 1/8, 1/8d, 1/8t, 1/16, 1/16d, 1/16t, 1/32),
**KEY-TRANSPOSE** = exit. Sync and division persist in EEPROM.

**Mask-ROM policy** (same as the 303): the ROM dump is Roland's copyrighted code and
is **never distributed**. `tools/mkrom.py` embeds it at build time from
`SH101-reference/80C49_SH101.bin` (git-ignored) into the generated, git-ignored
`src/rom101_data.h`. Without the dump the build still succeeds; the menu simply
refuses emulator mode and the unit always boots SuperOS.

## Docs

- [`docs/HARDWARE.md`](docs/HARDWARE.md) — the CPU, the 40-pin pin map, the AT90USB1286
  port mapping, the multiplexed I/O scheme, RAM layout, Test Mode, board plan, bring-up.
- [`docs/FIRMWARE.md`](docs/FIRMWARE.md) — reverse-engineered behaviour: the DAC/mux
  protocol, the (confirmed) 1 V/oct numeric model, every main-loop step, sequencer + arp.
- [`docs/Roland SH101 Service Manual.pdf`](docs/) — Roland, Nov 1982 (source of truth).
- `SH101-reference/` — Joe Britt's 2 KB ROM dump + commented disassembly (git-ignored).

## Source layout

- `src/pins.h` — AVR↔80C49 pin map (PORTC=BUS, PORTB=P1, PORTF=P2, PD0/1=T1/T0).
- `src/hw.h` — low-level driver enforcing the two invariants (active-low DB0–5;
  route = disconnected during every scan).
- `src/controls.h` — panel switch + function-button decode (ported from `SH101.asm`).
- `src/engine.h` — keyboard / arpeggiator / 100-step sequencer (EEPROM-backed).
- `src/main.cpp` — the SuperOS main loop (steps 3–13; entry points `superos_setup/loop`).
- `src/boot.cpp` / `src/combined.h` — combined-build dispatch + EEPROM map.
- `src/menu.h` — the config menu (firmware select).
- `src/mcs48.h` / `src/emu101.cpp` — MCS-48 core + 80C49-emulator host layer.
- `sim/` — host-side simulator (see below); runs both firmwares, including the
  emulated mask ROM (`mode emu` / `mode superos` in the CLI, or the menu combo).

## Host simulator

`sim/` builds the **real firmware sources natively** against a shimmed AVR layer
(`sim/shim/`) and models the analog-board side of the 80C49 socket at pin level:
the scan matrix (keyboard banks / switches / buttons, active-low), the DAC bus with
4052 route latching into the three sample/holds, GATE / CLOCK-RESET on P2, the T1
clock and T0 HOLD inputs, and file-backed EEPROM. No hardware needed to exercise
panel decode, the keyboard, the arpeggiator, the sequencer, the config menu, or the
80C49 emulator running the original mask ROM (the self-test verifies the emulated
ROM against the SuperOS behaviour model).

```sh
cd sim
make test        # build + scripted self-test (25 checks incl. the mask ROM)
./superos101-sim # interactive CLI: key/btn/sw/pedal/tick/run/mode/state, `help` lists all
```

Example session: `key 0 down` (gate on, CV 17), `btn up tap` then `tick 3`
(arpeggiate held keys on the simulated clock), `mode emu` (reboot into the original
ROM). EEPROM persists to `sim-eeprom.bin` (`SIM_EEPROM` overrides the path), so a
saved sequence and the selected firmware survive restarts.

## Build

Requires [PlatformIO](https://platformio.org/).

```sh
pio run                 # build the app -> .pio/build/app/firmware.hex
pio run -e app-debug    # app + USB serial + DEBUG for bench testing
```

## Flashing

Bench rig: Atmel-ICE on the Tag-Connect TC2030 pads (`avrdude -c atmelice_isp`). The
board is powered from the SH-101 socket (pin 40); the USB port does not power it. Factory
chips arrive with lock bits set, so the first write needs a chip erase (`-e`).

**Fuses: `lfuse 0xDF, hfuse 0xDA, efuse 0xF8`.** Burn them once per chip:

```bash
pio run -e fuses
```

Do not reuse the SuperOS-303/606 low fuse 0x9F on this board (CKOUT drives PC7 = DB7);
the reason is documented in `[env:fuses]` in `platformio.ini`.

Then flash the app (chip erase) followed by the bootloader (no erase, `-D`):

```bash
pio run -e app-nousb -e bootloader-nousb
```

```bash
avrdude -c atmelice_isp -p at90usb1286 -B 4 -e -U flash:w:.pio/build/app-nousb/firmware.hex:i
```

```bash
avrdude -c atmelice_isp -p at90usb1286 -B 4 -D -U flash:w:.pio/build/bootloader-nousb/firmware.hex:i
```

`app-nousb` is the bring-up build for the first board rev (USB-C hardware issues); use
`app` / `bootloader` once USB works. Later updates can go over DIN MIDI SysEx through the
bootloader (`tools/makesyx.py` emits `app-update.syx`, send with `tools/sendsyx`).

For hardware bring-up of a new board, `pio run -e diag` builds a diagnostic that streams
a per-pin BUS self-test over MIDI OUT (decode with `tools/recvsyx`); see
[`src/diag/diag.cpp`](src/diag/diag.cpp).

## Credits

- SH-101 ROM dump + disassembly: Joe Britt ([joebritt/SH101_Disassembled](https://github.com/joebritt/SH101_Disassembled)).
- AT90USB1286 replacement-CPU lineage: Nicholas J. Michalek (Phazerville) &
  [Michigan Synth Works](https://michigansynthworks.com/).
- SH-101 service manual (1982, Roland) under `docs/`.
