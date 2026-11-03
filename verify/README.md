# Verifying SH101-Firmware

One command from the repo root: `sh check.sh`. It exits 0 only when nothing failed and at least one check ran. It builds in a scratch copy under `$TMPDIR`, so the checkout gets no new files. It never touches hardware. The nightly run (`nightly()` in HQ) calls it.

Setup: `python3`, `rsync`, `git`, a C++ compiler, and the mask ROM dump `SH101-reference/80C49_SH101.bin` (git-ignored, never committed). In a git worktree the script reads the dump from the main checkout; `SH101_REF=<dir>` overrides. For the firmware build also `pio` and a working AVR toolchain (`PIO=<path>` if `pio` is not on PATH).

## What each line proves

| Line | Proves | When it is a SKIP |
|---|---|---|
| `pio run -e app -e bootloader` | Both images build | `pio` not found, or `~/.platformio/packages/toolchain-atmelavr/bin/avr-gcc` missing or not runnable (an x86 toolchain on an arm64 Mac without Rosetta) |
| `app flash: end 0x... free N` | Bytes free below 0x16000, the flash arena (`tools/makesyx.py`). PlatformIO's percentage is against the whole part, `makesyx` only refuses an overrun. Fails when the image reaches the arena. RAM used is printed under it; the rest of the 8192 is stack | same as above |
| `sim self-test` | Real firmware sources plus the real mask ROM in the host simulator. The stale binary is removed first | mask ROM dump not found |
| `G1 panel functions` | Panel functions driven headless through the sim | no sim binary |
| `G14` | SuperOS engine against the mask ROM under emulation. Exit 0 proves nothing, so the divergence list is pinned: cases 1, 10, 11, 12, 14, 15, 18, 19 (arp and sequencer CV or gate, event order on the keyboard case). A ninth divergence fails. So does a fix: update `G14_PIN` in `check.sh` with it | no sim binary |
| `G14_BREAK=1, 2, 3` | The red test (claim-081): a damaged engine must turn all 20 cases red | no sim binary |

A SKIP means the check did not run. It is never a pass, and a run where nothing passed exits 2.

Last measured (T0068 recipe, 2026-11-01): app ends at 0xF7BE, 26690 bytes free, RAM 5510 of 8192.

Not covered by `check.sh`: the parity line (`../parity`), the plug-in (`../../software/plugin`, `sh101_check`, `auval`) and the web editor tests (need playwright). Their commands are in HQ `verify/recipes/sh101.md`.

## Bjorn only

No agent runs these.

1. Flash: check `git status` first (a flash carries the whole tree), then `Akai-MPC-Live-II/tools/device/sh101_update.py --host 169.254.55.1 app-update.syx`, or `F0 7D 4A F7` then `tools/sendsyx "SuperOS-101 Bootloader" app-update.syx`.
2. Pitch: rail, DAC reference and the G13 scripts on a unit. +5 V is the pitch DAC reference; about 57 mV of rail is a semitone. No model replaces this.
3. Emulator latency: the sim's virtual clock cannot measure it. Key to gate needs the 0x3B probe on hardware.
4. Emulator changes need audio recorded before and after (`KNOWLEDGE.md`). `artifacts/g13/` holds past captures; nothing compares against them automatically.
5. CV DAC variants (`hw` 0 and 1), USB-C enumeration, the analog board by ear.

## Never run from an agent

`tools/flash.sh`, `pio run -e fuses`, any `-t upload`, `tools/sendsyx`, `tools/diag/`, `Akai-MPC-Live-II/tools/device/`.
