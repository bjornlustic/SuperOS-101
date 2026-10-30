Corrected 2026-10-13: firmware switch is in process (bb0307f), not a reboot; 16 patterns not 8; KEY TRANSPOSE no longer a stub.

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
with the mode shown on the function LEDs, **16 patterns of up to 128 steps** stored
in a flash arena ("emulated EEPROM", same scheme as SuperOS-303), **MIDI in/out on
DIN and USB-C** (notes, program change, SysEx editor protocol) and **MIDI clock sync**
(`src/midiclock.h`): the arp/sequencer clock, LFO hard-sync via CLOCK RESET, and a
beat-stepped random S&H, at ten divisions (1/4 ... 1/32 incl. dotted/triplet), in
both firmwares (the emulator gets the divided clock on its T1 pin). A **web editor**
(`tools/web-editor`) edits patterns and config over MIDI. KEY TRANSPOSE is
implemented (see Performance features); exact portamento timing is unverified. The CPU board lives in
[`../SuperOS-101CPU`](../SuperOS-101CPU).

## Panel (SuperOS mode)

- **UP / DOWN / UP&DOWN**: arpeggiator on/off (LED shows the mode). **HOLD**: latch keys.
- **LOAD**: start recording the current pattern (LOAD LED); keys = notes, HOLD held while
  pressing a key = legato (no gate gap before it, glides with PORTAMENTO on AUTO),
  KEY-TRANSPOSE = rest; **LOAD** again stores it.
- **PORTAMENTO AUTO** works as on the original: the firmware pulses PORTAMENTO OFF on
  every fresh note and leaves it alone on legato notes (keyboard legato, legato steps),
  so only those glide. ON / OFF ignore the line.
- **PLAY**: sequencer play/stop (PLAY LED). **PLAY held + one of the 8 lowest keys**
  selects pattern 1..16 (also MIDI Program Change 0..15 or the editor). While playing,
  the switch waits for the end of the loop pass.
- **HOLD + KEY-TRANSPOSE 2 s**: config menu (below).

Performance features (SuperOS only; SysEx 0x65 / 0x67 -> 0x66 for hosts). Stock
behavior from the Roland owner's manual is kept; the additions follow the Tubbutec
SH-1oh1 conventions where it has the same feature, so SH-1oh1 players feel at home.

- **KEY TRANSPOSE (stock)**: hold it and press a key: everything that plays is transposed
  by the distance from the lowest C, and it stays (LED lit). KEY TRANSPOSE + lowest C =
  normal. Unlike stock, the transpose also reaches CV OUT and MIDI OUT.
- **Transpose mode (new)**: **double-tap KEY TRANSPOSE** (LED blinks). While the sequencer
  plays (or a HOLD-latched arp runs) plain key presses transpose, no button held.
  Double-tap again to leave it.
- **Direction while playing (SH-1oh1)**: **UP** forward, **DOWN** backward, **U&D**
  ping-pong, **DOWN + U&D together** random. Persists.
- **Shift (SH-1oh1)**: while playing, **KEY TRANSPOSE + UP / DOWN** plays the next /
  previous step now (move the pattern against the clock).
- **Random arp (SH-1oh1)**: **DOWN + U&D together** (U&D LED blinks); again = off.
- **Randomize (SuperOS-303 style)**: **PLAY held + U&D** = new pattern (rhythm + pitch),
  **PLAY held + UP** = new pitches, **PLAY held + DOWN** = mutate (2 to 4 small edits). Written
  notes land in the pattern's scale. SysEx 0x68 [pat, op 0 full / 1 pitch / 2 rhythm / 3 mutate /
  4 octaves / 5 legato, density (0x7F = panel weights), seed lo, hi]; the fleet 0x2E is unchanged.
- **Scale (per pattern, the 303's 35 scales)**: **PLAY held + KEY TRANSPOSE + a key** = that
  note as the root, Major; the same key again steps through the scales. **PLAY held + KEY
  TRANSPOSE tap** = scale on / off. Non-destructive: the output snaps to the nearest scale note
  (lower on a tie) before any transpose. SysEx 0x2A [pat] reads, [pat, mask lo7, hi5, on] sets.
- **Nudge**: while playing, **HOLD button + UP / DOWN** moves the notes one step right / left
  (editor: nudge buttons; SysEx 0x69 [pat, 0 left / 1 right, count]). The playhead and clock are
  untouched; on the playing pattern the move lands at the next half-step, after the sounding
  note has started and before the next is chosen, so nothing is cut, doubled or retriggered.
- **Resync**: SysEx 0x65 id 9 (the editor's Resync) puts the sequence back on the one.
  Direction, loop length, pattern switches and clock-division changes never move it off the
  grid; ping-pong repeats its end steps so a round trip stays on the bar.
- **Clock source**: SysEx 0x65 id 10 (saved; 0x66's 11th field): 0 DIN and USB (the default),
  1 DIN only, 2 USB only. Realtime bytes (clock, Start, Stop, Continue) from the other jack are
  not acted on (soft thru still forwards them). A host that clocks both jacks, such as an MPC
  with its clock on the USB port and on the DIN cable, otherwise steps the machine twice a tick.
- **MIDI OUT / THRU**: SysEx 0x65 id 11 (saved; 0x66's 12th field; panel menu key 32): 0 OUT
  sends the notes the 101 plays (keys, arp, sequencer; accented steps at velocity 127), 1 THRU
  mirrors MIDI IN to OUT and sends none of its own notes, so a sequence does not spam the chain.
- **Notes out**: SysEx 0x65 id 12 (saved; 0x66's 13th field): the notes the 101 plays go to DIN
  and USB, DIN only, USB only, or nowhere. SysEx sync is unaffected.
- **Filter CV per step** (CV DAC mod): 0 = none, 1..127 on the cutoff CC's scale (64 = the FREQ
  slider). Set from the editors (0x16 lane 4, 0x2B/0x2C/0x2D); ties hold their note's value.
  See docs/FIRMWARE.md "Filter CV per step".
- **Accent per step** (CV DAC mod): in LOAD, hold **U&D** while entering a note (with HOLD too
  for an accented legato step). The step fires the accent envelope like a velocity-127 note.
  SysEx 0x16 lane 1 (0/1), and a packed accent tail after the ratchets on 0x11/0x12.
- **Arp octave range**: in arp mode, **KEY TRANSPOSE + UP / DOWN** = one octave more / less
  (1 to 3). UP climbs octave by octave, DOWN starts at the top, U&D bounces across the whole
  span, random picks an octave too. Persists. (Our addition; the SH-1oh1 has no range.)
- **Patterns (SH-1oh1)**: **PLAY held + key 1-16**: from stop, starts that pattern on
  release; while playing, it follows at the end of the current pass. **PLAY held + several
  keys** in order = a song (up to 16, looping). A **key 1-16 alone while playing** switches
  now, keeping the position.
- **Loop length, live**: while playing, **HOLD button + key n** loops the first n steps
  (lowest key = 1 step); the same key again = full pattern. Non-destructive.
- **LOAD while playing (stock "Correcting and Adding")**: keeps the pattern and re-loads
  from the step after the one sounding. From stop, LOAD starts a new pattern (clears).
  In LOAD, **DOWN** takes back the last step (SH-1oh1), **UP** cycles its ratchet 1x/2x/3x.
- **Real-time record**: while playing, **PLAY held + LOAD** (PLAY/LOAD LEDs alternate).
  Keys overwrite the step they land on (the next step once the step is half over), a key
  held past the half-step ties into the next step, KEY TRANSPOSE writes a rest.
  **PLAY held + LOAD** again ends it; the edits persist when playback stops.
- **Pedal as transpose**: config menu key 19 routes the HOLD pedal jack to act as the
  KEY TRANSPOSE button.
- **Sync delay**: config menu keys 20-31 = 0, 2, 4, 6, 8, 10, 12, 15, 18, 21, 25, 30 ms
  (SysEx 0x65 id 7: 0-60 ms). With MIDI sync on, the whole machine plays that much later
  than the incoming clock, to line the 101 up with slower gear or listening paths. Persists.
- **Tune**: **KEY TRANSPOSE held + LOAD** (from keyboard mode) holds an A with the gate
  open: 440 Hz at 8' with TRANSPOSE in the middle, and MIDI OUT note 69 for a tuner.
  Any function button leaves it (LOAD LED blinks while tuning).
- **CV / GATE OUT** carry whatever the voice plays, sequencer and arp included (the
  CPU's CV sample/hold feeds both the VCO and the rear CV jack).

## MIDI (DIN + USB-C, both firmwares)

Notes in on the configured channel (default 1): MIDI 53..84 act as the SH-101's own
keys F3..C6 (so they arpeggiate and record like keys), other notes 36..99 play as a
mono override. Everything the engine plays is echoed on MIDI OUT (note = CV + 36).
Realtime clock/start/stop drive the sync (config menu or editor). SysEx protocol
`F0 7D <cmd> ... F7` is documented at the top of [`src/midi.h`](src/midi.h): pattern
read/write (0x10..0x14), step position (0x15), select/transport (0x1D/0x1E), config
(0x20..0x22), firmware switch (0x2F/0x4D), bench probe (0x3B/0x3C: reset cause,
USB registers, boots since power-on, where the previous boot ended, loop pass time,
VCC in mV plus the raw ADC set behind it) and bench windows (0x3D park USB,
0x3E rail-load experiment: bit0 8 MHz, bit1 USB engine off, bit2 idle sleep,
bit3 16 MHz, mode 0 = the app's own 4 MHz as the control), emulator status
(0x40/0x41), bootloader (0x4A). Host tools: `tools/sendsyx` / `tools/recvsyx` (macOS CoreMIDI, build from
the `.c` files).

**USB-C** (`app` build): the unit enumerates as a class-compliant USB-MIDI device
"SuperOS-101" (VID/PID 16C0:0485, serial SOS101-0001), one virtual cable, in both
firmwares. Both ports run at once and carry the same traffic: notes and realtime go
in from either and out to both; SysEx replies and editor telemetry go to the port(s)
an editor has spoken on, so a USB session never waits on the 31250-baud DIN ring
(`src/midi.h`, one parser per port). The USB engine only runs while a cable is
plugged in (VBUS gate, `src/usb_gate.h`), every USB write is non-blocking and gated
on the bus being alive, and the firmware detaches cleanly before any reset or
bootloader jump; a 2 s pre-main watchdog plus an 8 s loop watchdog turn the
303-era freezes (PLL lock at boot, USB core against a bad bus after a fast firmware
swap) into recovery reboots, and the Teensy core's EP0 control-transfer spins are
bounded (`tools/patch_usb_core.py`) so a bus reset mid-enumeration cannot wedge the
ISR. Bench-verified 2026-08-15 over USB-C: enumeration, SysEx both ways, pattern
write + flash-arena persistence, 30/30 clean firmware swaps (one boot each, port back
in 1.2 s), 3/3 bootloader update round trips. The breakout carries the CC pull-downs;
C-to-C and C-to-A cables both work. The first bring-up day's "USB-C hardware issues"
were the 4.3 V brown-out fuse (see Flashing), not the port.

**Tuning and the +5 V rail.** The SH-101's +5 V regulator is also the DAC reference,
and it cannot supply the AVR at 16 MHz plus the USB engine: with both, VCC at the
chip sagged to 4.5-4.6 V and every note played about 7 semitones off, moving with the
load. The firmware therefore runs the core at 4 MHz (`platformio.ini`) and sleeps
through the S/H settle waits (`hw.h`); measured on the first unit the rail then holds
4.98 V with USB-MIDI running, in both firmwares, and the pitch is the same as with
the USB engine off (8 MHz was enough for SuperOS but left the CPU-bound emulator
~5 cents load-sensitive). The USB engine keeps its clock when unplugged so plugging
a cable in never moves the tuning (details in `docs/HARDWARE.md`, "Current
budget"). The side effect is on the 80C49 emulator: the interpreter cannot keep
up with the real 6 MHz chip. Measured on the unit (SysEx 0x3B probe): about
107k emulated machine cycles/s against the chip's 400k, so the ROM's program
pass, which is its S/H refresh and its clock-sample rate, takes ~6.6 ms instead
of ~1.6 ms (152 passes/s), and key/arp latency stretches with it (not the DAC
values). Three things keep the pitch and the sequencer usable at that speed
(`src/emu101.cpp`, `src/emu101_hle.h`, `src/mcs48.h`): the host re-writes the two
pitch S/H (CV-OUT, VCO range) with the ROM's last values every 2 ms between
emulated instructions, transparently to the ROM (`ShRefresh`), so the caps do not
droop 4x further than on the original; LFO / EXT CLK edges are captured by INT0
and fed to the ROM one per read as a virtual T1 (`mcs48_t1`), so its
once-per-pass sample never misses one (max followable clock is about half the
pass rate, ~75 Hz, and a step can land up to one pass late; the virtual line
only ever falls through that queue, never by showing the raw pin, or an edge
still inside INT0's bounce qualification stepped the ROM twice); and the ROM's
per-key scan loop (40% of a pass) runs natively (`emu101_hle.h`, verified
cycle- and state-exact against the interpreter); and since the ROM retriggers
each sequencer/arp step by dropping GATE for exactly one pass (~2 ms on the
chip, ~8 ms here, an audible re-attack), and the ROM writes the new step's CV
in that next pass just before raising the gate again, so on the chip the hole
covers the pitch change. The host reproduces that at the chip's timing: when the
ROM's own state says a retrigger follows (PLAY mode, current step is a note) the
GATE pin stays high at the ROM's drop and a 2 ms hole opens the instant the ROM
connects the changed CV (or at the raise, for a repeated note), the raise inside
it deferred to its end (`GateOnRomChange`, `GateOnCvChange`). Rests, releases
and stop follow the ROM exactly. Placing the hole elsewhere is audible: a hole
ending before the CV change re-attacks on the old pitch (stutter), and a CV
change under a high gate is legato to the analog PORTAMENTO AUTO logic (glide;
the ROM never raises PORTAMENTO OFF here). The MIDI echo follows the ROM's own
transitions. And since the analog PORTAMENTO AUTO logic keys on the
PORTAMENTO OFF line (verified: a 4 ms pulse makes the pitch S/H jump, without
it every change glides at the portamento setting) and the ROM never raises it
in sequencer/arp/keyboard use, the host pulses it as SuperOS does: on every
fresh note, every retriggered step and every arp pitch change; tied steps and
legato keys still glide. The host's own BUS traffic obeys the S/H invariants in `hw.h`
(present, then connect; a single-write restore after the menu button poll put a
blip on the pitch S/H every time it landed while the ROM held CV-OUT connected,
and the analog trigger detector turned that into envelope retriggers on some
notes: the per-step "stutter", found by recording the audio and bisecting). The interpreter itself keeps
its PC as a pointer into the 2 KB-aligned ROM image and dispatches through a
flash jump table (`mcs48.h`). The SuperOS loop pass is ~0.85 ms.

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
SuperOS-303 editor: pattern grid (note / rest / legato, per-step semitone/octave, length control), read/write
patterns, pattern select and transport, live playhead, config (channel, sync,
division), firmware switch, OS update from a `*-update.syx`, on-screen MIDI keyboard,
JSON save/load. It talks to the unit directly over USB-C (port "SuperOS-101") or via
any DIN interface. Web MIDI needs Chrome/Edge over http or localhost. In Cursor / VS
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
  MIDI notes 53..84 are pressed into the emulated key matrix (the ROM sees keys), the
  ROM's CV/GATE is echoed on MIDI OUT, and the ROM's battery RAM (0x1C..0x7F, the
  100 sequencer steps) is mirrored to internal EEPROM (`combined.h` map, `emu101.cpp`):
  restored before the ROM runs, written back a second after the last change.

**Config menu**: hold **HOLD + KEY-TRANSPOSE for 2 s** (while running, either mode)
or at power-on. In the menu the HOLD LED lights, the gate is forced off, and the
function LED shows the running firmware (LOAD lit = emulator, PLAY lit = SuperOS).
Then: **LOAD** = switch to the emulator, **PLAY** = switch to SuperOS,
**UP&DOWN** = toggle MIDI clock sync (KEY-TRANSP LED lit = on), **UP**/**DOWN** =
cycle the clock division, the 10 lowest keys = pick the division directly
(1/4, 1/4d, 1/4t, 1/8, 1/8d, 1/8t, 1/16, 1/16d, 1/16t, 1/32),
keys 11/12 = accent model, key 13 = MIDI cutoff on/off, key 14 = LFO hard-sync,
**keys 15-18** = sequencer direction (forward, backward, ping-pong, random),
**key 19** = pedal route (HOLD / KEY TRANSPOSE),
**KEY-TRANSPOSE** = exit. All of it persists in EEPROM.

A firmware switch (menu, or SysEx 0x2F / 0x4D from the editor) runs in process
since bb0307f (2026-08-27): no MCU reset and no USB detach. The request is latched,
the next loop pass persists the EEPROM select byte, tears the leaving side down
(SuperOS flushes dirty patterns; the emulator saves its battery RAM), applies the
input lockout (all keys and buttons released for 250 ms, 4 s cap), and runs the
other side's setup with its state reset (`src/boot.cpp` SwitchTo). Only the
bootloader entry (0x4A) still detaches and reboots.

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
- `src/engine.h` — keyboard / hold / arpeggiator / 16 x 128-step sequencer.
- `src/midi.h` — MIDI driver: DIN (UART1 rings + ISRs) + USB-MIDI endpoints, one
  parser per port, SysEx protocol spec; `src/usbmidi_codec.h` — USB-MIDI event codec;
  `src/usb_gate.h` — USB VBUS/SOF gating (from the 303/606).
- `src/midiclock.h` — MIDI clock divider; `src/sysex.h` — commands common to both firmwares.
- `src/flash_eeprom.h` / `flash_store.h` / `flash_persist.h` — flash-arena pattern store.
- `src/main.cpp` — the SuperOS main loop (steps 3–13; entry points `superos_setup/loop`).
- `src/boot.cpp` / `src/combined.h` — combined-build dispatch, watchdogs, firmware
  switch / bootloader entry, EEPROM map.
- `src/menu.h` — the config menu (firmware select).
- `src/mcs48.h` / `src/emu101.cpp` — MCS-48 core + 80C49-emulator host layer.
- `src/bootloader/` — SysEx bootloader + `flash_service.c` (SPM page writer).
- `tools/web-editor/` — the Web MIDI editor; `tools/flash.sh` — ISP build+flash;
  `tools/patch_usb_core.py` — makes the Teensy usb_midi core's USB identity overridable.
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
pio run                 # build the app (USB-MIDI + DIN) -> .pio/build/app/firmware.hex + app-update.syx
pio run -e app-nousb    # DIN-only app (USB engine never enabled)
pio run -e app-debug    # app + USB serial + DEBUG for bench testing (no USB-MIDI)
```

## Flashing

Bench rig: Atmel-ICE on the Tag-Connect TC2030 pads (`avrdude -c atmelice_isp`). The
board is powered from the SH-101 socket (pin 40); the USB port does not power it. Factory
chips arrive with lock bits set, so the first write needs a chip erase (`-e`).

**Fuses: `lfuse 0xDF, hfuse 0xDA, efuse 0xF9`.** Burn them once per chip:

```bash
pio run -e fuses
```

Do not reuse the SuperOS-303/606 low fuse 0x9F (CKOUT drives PC7 = DB7) or extended
fuse 0xF8 (4.3 V brown-out; the SH-101's +5 V rail dips below it on every flash write
and on USB start, resetting the chip); both reasons are documented in `[env:fuses]` in
`platformio.ini`.

Then flash the app (chip erase) followed by the bootloader (no erase, `-D`), or just
run `tools/flash.sh`:

```bash
pio run -e app -e bootloader
```

```bash
avrdude -c atmelice_isp -p at90usb1286 -B 4 -e -U flash:w:.pio/build/app/firmware.hex:i
```

```bash
avrdude -c atmelice_isp -p at90usb1286 -B 4 -D -U flash:w:.pio/build/bootloader/firmware.hex:i
```

`app-nousb` / `bootloader-nousb` (`tools/flash.sh nousb`) are DIN-only fallbacks that
never enable the USB engine.

**Updates without the ISP rig** go over MIDI SysEx, DIN or USB-C, through the
bootloader: every `pio run` emits `app-update.syx`. Send `F0 7D 4A F7` to the running
app (the web editor's *Update OS* button does this, `sendsyx` users send it first) and
the unit jumps into the bootloader without a reset, HOLD LED blinking then lit; over
USB it re-enumerates as "SuperOS-101 Bootloader". Then send the file
(`tools/sendsyx "SuperOS-101 Bootloader" app-update.syx`, or the editor sends the
pages itself, following the unit onto the bootloader port). The final `F0 7D 02 F7`
in the file makes the bootloader reset into the new app. The panel fallback is
holding **LOAD + PLAY** at power-on. The bootloader itself is only ever written over
ISP; SysEx updates cannot touch it or the pattern arena.

For hardware bring-up of a new board, `pio run -e diag` builds a diagnostic that streams
a per-pin BUS self-test over MIDI OUT (decode with `tools/recvsyx`); see
[`src/diag/diag.cpp`](src/diag/diag.cpp).

## Credits

- SH-101 ROM dump + disassembly: Joe Britt ([joebritt/SH101_Disassembled](https://github.com/joebritt/SH101_Disassembled)).
- AT90USB1286 replacement-CPU lineage: Nicholas J. Michalek (Phazerville) &
  [Michigan Synth Works](https://michigansynthworks.com/).
- SH-101 service manual (1982, Roland) under `docs/`.

## CV DAC (MIDI filter cutoff + accent)

Board v1.1+ carries an MCP4822 wired into the SH-101's cutoff summing node and
VCA current node. MIDI CC 74 sets the cutoff anywhere in the FREQ slider's
range from any slider position (bipolar, center-calibrated), and accented
notes (MIDI velocity >= threshold) get an MC-202- or TB-303-style accent on
the filter and the VCA, selectable in the config menu (keys 11/12) or by SysEx
0x25. Details: `docs/FIRMWARE.md` "CV DAC", `src/cvdac.h`.
