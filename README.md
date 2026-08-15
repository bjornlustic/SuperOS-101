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

🟢 **Running on the first hardware unit** (fuses, DAC path, both firmwares verified
2026-08-15). Combined firmware (about 18 KB): **SuperOS** plus an **80C49 emulator
mode** that runs the original Roland mask ROM (see below), selected per boot from a
config menu. The SuperOS side: faithful DAC/mux protocol and scan invariants,
**panel decode** (RANGE/OCTAVE/ADSR, ported from the disassembly), a **mono
keyboard** (lowest/last-note priority), **HOLD**, an **arpeggiator** (up/down/up&down)
with the mode shown on the function LEDs, **8 patterns of up to 100 steps** stored
in a flash arena ("emulated EEPROM", same scheme as SuperOS-303), **DIN MIDI in/out**
(notes, program change, SysEx editor protocol) and **MIDI clock sync**
(`src/midiclock.h`): the arp/sequencer clock, LFO hard-sync via CLOCK RESET, and a
beat-stepped random S&H, at ten divisions (1/4 ... 1/32 incl. dotted/triplet), in
both firmwares (the emulator gets the divided clock on its T1 pin). A **web editor**
(`tools/web-editor`) edits patterns and config over MIDI. Remaining stubs: the
KEY-TRANSPOSE shift value and exact portamento timing. The CPU board lives in
[`../SuperOS-101CPU`](../SuperOS-101CPU).

## Panel (SuperOS mode)

- **UP / DOWN / UP&DOWN**: arpeggiator on/off (LED shows the mode). **HOLD**: latch keys.
- **LOAD**: start recording the current pattern (LOAD LED); keys = notes, HOLD held while
  pressing a key = tie, KEY-TRANSPOSE = rest; **LOAD** again stores it.
- **PLAY**: sequencer play/stop (PLAY LED). **PLAY held + one of the 8 lowest keys**
  selects pattern 1..8 (also MIDI Program Change 0..7 or the editor).
- **HOLD + KEY-TRANSPOSE 2 s**: config menu (below).

## MIDI (DIN, both firmwares)

Notes in on the configured channel (default 1): MIDI 53..84 act as the SH-101's own
keys F3..C6 (so they arpeggiate and record like keys), other notes 36..99 play as a
mono override. Everything the engine plays is echoed on MIDI OUT (note = CV + 36).
Realtime clock/start/stop drive the sync (config menu or editor). SysEx protocol
`F0 7D <cmd> ... F7` is documented at the top of [`src/midi.h`](src/midi.h): pattern
read/write (0x10..0x14), step position (0x15), select/transport (0x1D/0x1E), config
(0x20..0x22), firmware switch (0x2F/0x4D), emulator status (0x40/0x41), bootloader
(0x4A). Host tools: `tools/sendsyx` / `tools/recvsyx` (macOS CoreMIDI, build from
the `.c` files).

## Pattern storage (flash arena)

Patterns persist in a wear-leveled record store carved out of internal flash
(pages 0x160..0x1DF, 32 KB), ported from SuperOS-303 (`src/flash_eeprom.h`,
`src/flash_persist.h`). The SPM page writer lives in the bootloader image
(`src/bootloader/flash_service.c`, entry 0x1FE00), because only boot-section code
can write flash. Writes are deferred (about 0.5 s after the last edit) and
power-loss safe (write-then-erase). `tools/makesyx.py` refuses an app that grows into
the arena. A chip erase over ISP wipes the arena; SysEx OS updates do not.

## Web editor

`tools/web-editor/index.html` is a single-file Web MIDI editor in the style of the
SuperOS-303 editor: pattern grid (note / rest / tie, keyboard shortcuts), read/write
patterns, pattern select and transport, live playhead, config (channel, sync,
division), firmware switch, OS update from a `*-update.syx`, on-screen MIDI keyboard,
JSON save/load. Web MIDI needs Chrome/Edge over http or localhost. In Cursor / VS
Code press **F5** ("SuperOS-101 Web Editor (Chrome)", `.vscode/launch.json`); it
serves the folder on port 3101 and opens Chrome. Or:

```bash
python3 -m http.server 3101 --directory tools/web-editor
```

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
- `src/engine.h` — keyboard / hold / arpeggiator / 8 x 100-step sequencer.
- `src/midi.h` — DIN MIDI driver (UART1 rings + ISRs), parser, SysEx protocol spec.
- `src/midiclock.h` — MIDI clock divider; `src/sysex.h` — commands common to both firmwares.
- `src/flash_eeprom.h` / `flash_store.h` / `flash_persist.h` — flash-arena pattern store.
- `src/main.cpp` — the SuperOS main loop (steps 3–13; entry points `superos_setup/loop`).
- `src/boot.cpp` / `src/combined.h` — combined-build dispatch + EEPROM map.
- `src/menu.h` — the config menu (firmware select).
- `src/mcs48.h` / `src/emu101.cpp` — MCS-48 core + 80C49-emulator host layer.
- `src/bootloader/` — SysEx bootloader + `flash_service.c` (SPM page writer).
- `tools/web-editor/` — the Web MIDI editor; `tools/flash.sh` — ISP build+flash.
- `sim/` — host-side simulator (see below); runs both firmwares, including the
  emulated mask ROM (`mode emu` / `mode superos` in the CLI, or the menu combo).

## Host simulator

`sim/` builds the **real firmware sources natively** against a shimmed AVR layer
(`sim/shim/`) and models the analog-board side of the 80C49 socket at pin level:
the scan matrix (keyboard banks / switches / buttons, active-low), the DAC bus with
4052 route latching into the three sample/holds, GATE / CLOCK-RESET on P2, the T1
clock and T0 HOLD inputs, MIDI in/out queues, and file-backed EEPROM + flash arena. No hardware needed to exercise
panel decode, the keyboard, the arpeggiator, the sequencer, the config menu, or the
80C49 emulator running the original mask ROM (the self-test verifies the emulated
ROM against the SuperOS behaviour model).

```sh
cd sim
make test        # build + scripted self-test (incl. the mask ROM, MIDI, flash store)
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
