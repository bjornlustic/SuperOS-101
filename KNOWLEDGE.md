# SuperOS-101 firmware: knowledge base

Consolidated 2026-10-13 from the code, git history (148 commits, 2026-08-14 to 2026-10-09), the docs in this repo, and the archived CLAUDE.md (`~/SuperOS303/Backlog/Archive/claude-md/SH101/SuperOS-101FIRMWARE/CLAUDE.md`). Where they disagree, the code wins. Superseded notes were moved to `~/SuperOS303/Backlog/Archive/notes/SH101/SuperOS-101FIRMWARE/` (list in `old/notes/MANIFEST-roland-drums.txt`).

## What this is

Open firmware for the Roland SH-101, running on the AT90USB1286 replacement CPU board in `../../hardware/cpu-board` (the board plugs into the 40-pin socket of the original Toshiba TMP80C49P, an MCS-48/8049; not the uPD650 of the 303/606/808). One combined image (PlatformIO, Teensy++ 2.0 core) carries two firmwares selected by internal-EEPROM byte 0x000:

- **SuperOS**: clean-sheet SH-101 behaviour plus 16 patterns x up to 128 steps, arp, MIDI (DIN + USB-C), MIDI clock sync, performance features, CV DAC (filter cutoff + accent), SysEx editor protocol.
- **80C49 emulator**: an MCS-48 interpreter running the original 2 KB Roland mask ROM against the real analog board.

Part of the SuperOS fleet (SuperOS-303/606/808/909); fleet-wide docs live in `~/SuperOS303/SuperOS-Tools/USB-Bootloader/docs/` (`FINDINGS.md`, `EDIT_OPCODES.md`, `G9_INVENTORY.md`). The fleet `coord/` folder was archived to `~/SuperOS303/Backlog/Archive/coord/`.

## Current status (2026-10-13)

- Running on the first hardware unit since 2026-08-15 (fuses, DAC path, both firmwares, USB-C, SysEx bootloader verified that day).
- Last commit `d0363de` 2026-10-09: armed start mode (0x65 id 15). Before that: key notes / local control (`5bca66c`, 2026-10-08), clock source, soft thru, CV DAC + per-step accent + per-step filter CV (2026-09-29/30), 16 patterns, scales, randomizers, nudge, MIDI file import in the web editor (2026-09-28).
- Sim self-test: `./superos101-sim --test` passed with 0 failures on 2026-10-13 (existing binary, which `make -q` reports as older than the sources; rebuild before relying on it).
- Working tree: `CLAUDE.md` shows deleted (archived on purpose); `docs/mods/*.png` untracked and intended to stay.
- Version string: `SUPEROS_VERSION` 0.2 (`platformio.ini`).

## Layout

- `src/` firmware: `main.cpp` (SuperOS loop), `engine.h` (keyboard/arp/sequencer), `controls.h` (panel decode), `hw.h` (DAC/mux driver and invariants), `pins.h`, `midi.h` (DIN+USB MIDI, SysEx protocol comment at the top), `sysex.h`, `midiclock.h`, `cvdac.h`, `flash_eeprom.h`/`flash_store.h`/`flash_persist.h` (flash-arena store), `combined.h` (EEPROM map), `boot.cpp` (dispatch, watchdogs, in-process firmware switch), `menu.h`, `mcs48.h`/`emu101.cpp`/`emu101_hle.h` (emulator), `bootloader/` (SysEx bootloader + `flash_service.c` SPM writer), `diag/` (bring-up diagnostic).
- `sim/` host simulator linking the real firmware sources; `sim.cpp` self-test, `g1_test.py` panel-function harness, `Makefile`.
- `tools/`: `flash.sh` (ISP), `makesyx.py`/`hex2sysex.py` (update .syx), `sendsyx.c`/`recvsyx.c` (CoreMIDI tools), `mkrom.py` (embeds mask ROM), `buildrev.py` (build sha), `patch_usb_core.py`, `web-editor/` (single-file Web MIDI editor + Playwright tests), `diag/` (G13 pitch/rail measurement scripts; three of them hard-code `artifacts/g13/sidecars`).
- `docs/`: `HARDWARE.md` (pin map, port map, rail budget), `FIRMWARE.md` (ROM behaviour spec, CV DAC, filter CV, note output, key notes, start mode), `G10-tempo-blink-draft.md` (original G10 spec, now implemented), `superos-101-manual-*.html` (user manual), `shortcut-guide/` (generated PDF guide + its build scripts), `mods/` (CV mod install guide HTML + photos), `Roland SH101 Service Manual.pdf`.
- `SH101-reference/` (git-ignored): Joe Britt's ROM dump `80C49_SH101.bin`, `SH101.asm`, manuals. Never distribute the ROM.
- `artifacts/g13/`: G13 capture data (`sidecars/*.json`, `.wav`); the text result dumps were moved to old/notes.
- Root `*.syx`, `sim-*.bin`: build/run outputs (git-ignored).

## Build / run / flash / test

Verified to exist (platformio.ini envs: `app` default, `app-debug`, `app-nousb`, `diag`, `fuses`, `bootloader`, `bootloader-nousb`; `pio` is at `~/Library/Python/3.9/bin/pio`; `avrdude` at `/opt/homebrew/bin/avrdude`).

```sh
pio run                          # app -> .pio/build/app/firmware.hex + app-update.syx
pio run -e app-nousb             # DIN-only app
pio run -e app -e bootloader
pio run -e fuses -t upload       # once per chip: lfuse 0xDF hfuse 0xDA efuse 0xF9 (chip erase first if locked)
tools/flash.sh [app|nousb|diag|fuses]   # Atmel-ICE on the TC2030 pads; app with -e, then bootloader with -D
cd sim && make test              # build + self-test (real firmware, real mask ROM)
cd sim && ./superos101-sim --test && python3 g1_test.py
cd sim && ./superos101-sim --g14 # firmware vs mask-ROM divergence comparator
python3 tools/web-editor/test_import.py      # needs playwright
python3 -m http.server 3101 --directory tools/web-editor   # editor (Chrome/Edge)
python3 docs/shortcut-guide/build_guide.py   # regenerates the shortcut PDF (needs Chrome)
```

Field updates: send `F0 7D 4A F7`, then `tools/sendsyx "SuperOS-101 Bootloader" app-update.syx` (or the editor's Update OS). Panel fallback: hold LOAD + PLAY at power-on. The bootloader itself is ISP-only. On the bench, flash the 101 over its USB-C through the MPC Live II (Ethernet link host 169.254.55.1): `~/SuperOS303/Products/Samplers/Akai-MPC-Live-II/tools/device/sh101_update.py --host 169.254.55.1 app-update.syx`. The script resends every page until a pass has no USB endpoint halts, then starts the app; it fails unless the build identity 0x35 changed. If the 101 drops off USB (no app, no bootloader in the MPC's `dmesg`): hold LOAD + PLAY at power-on and run the script again. `--din` (MPC MIDI Port A cabled to the 101's MIDI IN) is a fallback only; that cable is not normally connected. Check `git status` before building an image to flash: a flash carries the whole tree.

## Key technical facts and findings

**Clock and rail**
- App core clock is **4 MHz** (`board_build.f_cpu = 4000000L`, `platformio.ini [env:app]`); bootloader runs at 16 MHz. The SH-101 +5 V rail is also the pitch DAC reference; about 57 mV of rail = 1 semitone, and lower rail = sharper pitch.
- 2026-08-15: 16 MHz + USB sagged to 4.49-4.65 V (about 7 semitones off); 8 MHz showed about 5 cents load wobble in the emulator; 4 MHz held 4.98 V in both firmwares.
- 2026-09-30, after the R63 ground strap: 4 and 8 MHz both hold 4855 mV and pitch within 0.3 cents (8 MHz verified safe); 16 MHz sags to about 4570 mV (+350 cents, sd 71 cents). 16 MHz needs the CPU board on its own supply. Commits `4efb48e` (8 MHz), `3a42203` (back to 4 MHz), comment block in `platformio.ini`.
- efuse 0xF9 (BOD 3.5 V): at 4.3 V every SPM flash write and USB PLL start reset the chip. lfuse 0xDF: the 303/606 value 0x9F drives CLKO onto PC7 = DB7 (DAC route bit).

**Emulator**
- Runs at about 25 % of the real 80C49 at 4 MHz (about 101-107k of 400k machine cycles/s, ROM pass about 6.6-6.9 ms vs 1.5-3.5 ms on the chip). Accepted, not a bug: the interpreter is at its computed-goto ceiling with the key scan already native (`emu101_hle.h`, cycle-exact).
- Host compensations in `src/emu101.cpp`: S/H refresh every 2 ms (`ShRefresh`), INT0-captured T1 edges fed one per read as a virtual T1 (max followable clock about 75 Hz), a 2 ms gate hole timed to the ROM's CV change (`GateOnRomChange`/`GateOnCvChange`), PORTAMENTO OFF pulses on fresh notes. The virtual T1 must only fall through the edge queue, never show the raw pin (`7c515a2`, double-triggered steps otherwise).
- Emulator changes must be compared in recorded audio before and after (standard set by the 2026-08-15/16 emulator commits, e.g. `71d2d11`, `7c515a2`). Any signal moved behind a queue must stop being observable raw at the same instant.
- Key-to-gate bound (from hardware pass time and ROM structure): emulator < 13.8 ms, SuperOS < 1.7 ms, original 3.0-7.0 ms. The sim's virtual clock only counts settle delays, so it cannot measure latency.

**Protocol (`src/midi.h` header is canonical)**
- Manufacturer 0x7D. `0x13` is REQUEST-ALL (no length dispatch). SET STEP is `0x16 <p><lane><step><value>`: lane 0 note (0 = rest), 1 accent (refused on a rest, ack 2), 2 absent (ack 2: no slide/chain on the 101), 3 gate (0 normal, 1 legato), 4 filter CV (refused on a rest, ack 2).
- `0x15` is **outbound only** (step position every step while playing). Inbound RANDOMIZE moved to `0x2E` (claim-049/057); `0x68` is the SuperOS-303-style randomizer. The archived CLAUDE.md's "incoming 0x15 is randomize" is stale.
- `0x18` length and `0x16` step index are one 7-bit byte: SysEx reaches 1..127 only; step/length 128 is panel-only (claim-061).
- `0x11`/`0x12` carry a packed ratchet tail (ceil(len/4) bytes) then an accent tail (ceil(len/8)); optional on input, always present on output. Filter CV lanes travel separately as `0x2B`/`0x2C`/`0x2D`; every `0x11` the unit sends is followed by `0x2C`.
- Settings: `0x65 <id> <v>` set, `0x67` request, `0x66` reply. ids include 7 sync delay, 9 resync, 10 clock source, 11 OUT/THRU, 12 notes out, 13 key notes, 14 local control, 15 start mode. Nudge `0x69`, scale `0x2A`, CV DAC config `0x23/0x24/0x25`, probe `0x3B -> 0x3C`, rail bench `0x3E`, build id `0x34 -> 0x35`, bootloader `0x4A`, firmware switch `0x2F`/`0x4D`.
- Sync ON is strict transport slaving: `0xFA` starts from idle, `0xFC` stops, only the divided clock steps; a silent wire = a silent machine. Sync OFF: `0xFA` only rewinds. Start never interrupts an active arp or LOAD. Armed start mode (id 15): Start never starts an idle 101; local PLAY queues it.
- MIDI clock divisions are integer tick counts at 24 ppqn (`midiclock.h DIV_TICKS`); do not replace with anything rate-based. Clock model is the fleet's uClock owed-tick model (`5a075c3`).
- After `0x2F`/`0x4D`/`0x4A` on this Mac, CoreMIDI may not rebind for 30 s+: `killall MIDIServer` (host cache, finding 8). Since `bb0307f` the firmware switch is in process (no USB drop); only `0x4A` re-enumerates.

**Engine and storage**
- `NUM_PATTERNS` 16 (`src/flash_persist.h:19`), `MAX_STEPS` 128, `MIDI_CV_OFFSET` 36 (cv = note - 36, clamped 0..63). Ratchets 2 bits/step (single/2x/3x).
- Patterns persist in a wear-levelled flash arena (pages 0x160..0x1DF, 32 KB), deferred about 0.5 s, write-then-erase. SPM writer lives in the bootloader image at 0x1FE00. Filter lanes are separate blocks `FB_FILTER_BASE (17) + p`, `FE_MAX_BLOCKS` 40. Chip erase over ISP wipes the arena; SysEx updates do not.
- Pattern length lives in payload byte 0 (`flash_persist.h:8`), separate from the record header's payload length; this redundancy made the 101 immune to the 606's length-loss bug (claim-029/033). Do not merge them.
- EEPROM map in `src/combined.h` (0x000 firmware select through 0x00C start mode, 0x010..0x01E CV DAC, 0x020..0x023 features, 0x030 MC-202 attack, 0x100+ emulator battery RAM), bounds enforced at compile time (`723c048`).
- Note priority: lowest-note when `lastNotePriority` is false (`engine.h pickKey`; fixed in `c738a15`, it was first-note).
- Accent lasts one note (MC-202 model, `56a13bc`): ends with the gate or when the next step starts unaccented, 7 ms release.

**Hardware interface**
- P2: GATE (P27) is 0 when ON; HOLD LED (P25) 0 when lit; KEY-TRANSPOSE LED (P26) 1 when lit; CLOCK RESET (P24) idles high, pulsed low; P20-22 one-hot LED select via IC8 4556 (0 none, 1 LOAD, 2 PLAY, 3 DOWN, 4 U&D, 5 UP). DAC writes are the one's complement on DB0-5 (`dac_out: cpl a`); route must be 11 during every scan (`hw.h`).
- CV DAC: MCP4822 on PA0-2, VOUTA to the VCF summing node (IC18b pin 6, sits at the +5 V D/A reference), VOUTB to BA662 pin 1 via TR28 collector. Board variant `hw` 1 (bipolar, default). First unit: v1.1 board, bias on +15 V with R10 68k, center measured 850 mV. Details `docs/FIRMWARE.md` "CV DAC".
- 0x3B probe is the hardware state channel (reset cause, USB regs, loop timing, VCC mV, emulator counters, P2 image and per-bit rates at raw[97..113], key-to-gate word at raw[125..126]). `raw` is full at 127 words; the next field needs a 7-bit packing change.

## Decisions and why

- **4 MHz app clock** (owner's choice, `3a42203`, 2026-09-30): rail and pitch are identical at 4 and 8 MHz after the ground strap; 8 MHz would double emulator speed and is one line in `platformio.ini`. 16 MHz is ruled out by the regulator's knee, not EMI or USB.
- **Own bootloader, not the fleet shared one**: claim-005 withdrawn; ISP-only for flashing the bootloader; G11 deferred because the bench cannot test it headless.
- **Emulator below real time is accepted** (G5 rewritten for this machine): the bar is that the musical path is unaffected and key-scan response has a stated bound.
- **Accent one note long** (MC-202 CPU accent line), filter CV refused on rests (`8d43d8e`), per-step filter lanes stored outside the pattern record to keep the record format unchanged.
- **Firmware switch in process** (`bb0307f`): both firmwares share one USB identity, so a reboot only cost host bindings.
- **Mask ROM is never distributed**: `tools/mkrom.py` embeds `SH101-reference/80C49_SH101.bin` into git-ignored `src/rom101_data.h`; without it the build succeeds and emulator mode is refused.
- **G4 edit opcodes** (0x16/0x18, randomize now 0x2E): shipped in `5bc9e58` (2026-08-25); the retired draft was moved to old/notes.

## Known issues

- G14 divergences vs the original ROM (2026-08-26, `sim --g14`): D1 SuperOS writes CV and gate in the same pass, the ROM writes CV a pass before the gate; D3 the ROM does not retrigger the gate between arp steps, SuperOS does; D4 the ROM's first arp step gates the previous pitch. D2 (priority) is fixed. D3/D4 want a hardware capture before changing `engine.h`. No later commit addresses D1/D3/D4.
- G5 for the 101 is not closed: the step-5-to-step-13 fraction `f` of a pass has a probe word but no hardware measurement.
- Stale code comments: `src/engine.h:31` says NUM_PATTERNS is 8 (it is 16); `src/main.cpp:7-8` lists KEY-TRANSPOSE as a stub (implemented); `src/midi.h` says an emulator unit ignores 0x20 (it answers since `54e52ee`, `emu101.cpp:575`).
- Portamento timing is not verified against the original.
- The G13 rail study (August, 8 MHz vs 4 MHz pitch shift per note: +4.02 c at note 36, +2.91 c at 60, +1.71 c at 72, +0.60 c at 99; no simple law fits; DAC carry boundaries step) predates the R63 ground strap and is moot at 4 MHz. Full write-ups are in old/notes.
- Low notes have intrinsic per-trigger pitch spread (about 11.7 c at note 36, about 0.44 Hz absolute); the VCA envelope decays to the floor about 2.2 s after onset even with the gate held. Both are analog, not firmware.

## Open questions for Bjorn

1. (Resolved 2026-10-15) G10 tempo blink is implemented on the KEY-TRANSPOSE LED: it blinks on the beat while `midiclk::Driving`, and transpose mode's fast blink keeps priority (`main.cpp`, `midiclock.h BeatLed`). Built privately in the 101's clock, not in the shared fleet clock core.
2. Is the v1.2 CPU board (100k bias) fabbed or installed anywhere, or is the reworked v1.1 on the first unit still the only hardware?
3. Keep 4 MHz, or switch to 8 MHz for emulator speed now that it is verified safe?
4. G14 D1/D3/D4: match the original (changes how the arp sounds) or keep SuperOS behaviour as a deliberate difference?
5. The +5 V regulator: `docs/HARDWARE.md` describes a discrete op-amp + PNP (TR3) regulator from schematic p.7; the CPU board README once called it an "S1671140 DC/DC". Which is right (affects any future supply fix)?
6. `docs/mods/*.png` are untracked: commit them alongside `SH101-Accent-Filter-Ground-Mod.html`?
