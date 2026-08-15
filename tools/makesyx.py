# SuperOS-101: post-build script — emit <env>-update.syx next to the project
# so every `pio run` produces a bootloader-flashable update stream.
Import("env")
import os
import sys

sys.path.insert(0, os.path.join(env["PROJECT_DIR"], "tools"))
import hex2sysex
from intelhex import IntelHex

# The bootloader owns 0x1F000+. An app that reaches it cannot be shipped:
# hex2sysex refuses those pages and the image would be truncated.
BOOT_BASE = 0x1F000
# The flash-arena pattern store (src/flash_eeprom.h FE_ARENA_FIRST_PAGE) starts
# here; an app image that grows into it would overwrite saved patterns on
# update. Keep in sync with flash_eeprom.h.
FLASH_ARENA_BASE = 0x16000

syx_path = os.path.join(env["PROJECT_DIR"], "%s-update.syx" % env["PIOENV"])


def make_syx(target, source, env):
    hex_path = str(source[0])
    out_path = str(target[0])
    ih = IntelHex(hex_path)
    if ih.maxaddr() >= FLASH_ARENA_BASE:
        sys.stderr.write(
            "makesyx: FATAL: app reaches 0x%X, into the flash-arena pattern "
            "store (0x%X) / boot section (0x%X). Shrink the app or move the "
            "arena (flash_eeprom.h).\n" % (ih.maxaddr(), FLASH_ARENA_BASE, BOOT_BASE))
        env.Exit(1)
        return
    with open(out_path, "wb") as f:
        hex2sysex.emit(ih, f)
    print("makesyx: wrote %s (%d bytes)"
          % (os.path.basename(out_path), os.path.getsize(out_path)))


syx = env.Command(syx_path, "$BUILD_DIR/${PROGNAME}.hex", make_syx)
env.AlwaysBuild(syx)
env.Default(syx)
