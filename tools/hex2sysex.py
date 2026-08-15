#!/usr/bin/env python
# SuperOS-101: firmware.hex -> MIDI SysEx update stream for the bootloader
# (src/bootloader/bootload.c). Ported from SuperOS-303; same protocol:
#   F0 7D 01 <page-hi> <page-lo> <len-hi> <len-lo> <ck-hi> <ck-lo> <7bit> F7
#   F0 7D 02 F7   run the app

import sys
from intelhex import IntelHex

PAGE_SIZE = 256          # SPM_PAGESIZE, AT90USB1286
SYSEX_START = 0xF0
SYSEX_END = 0xF7
MFR_ID = 0x7D

CMD_WRITE_PAGE = 0x01
CMD_EXECUTE = 0x02


def pack_7bit(data):
    """Convert 8-bit data into MIDI 7-bit packed format (7 bytes -> 8)."""
    out = []
    i = 0
    while i < len(data):
        chunk = data[i:i + 7]
        msb = 0
        for bit, b in enumerate(chunk):
            if b & 0x80:
                msb |= (1 << bit)
        out.append(msb)
        for b in chunk:
            out.append(b & 0x7F)
        i += 7
    return out


def build_sysex(page, page_data):
    encoded = pack_7bit(page_data)
    length = len(encoded)
    checksum = 0
    for b in page_data:
        checksum ^= b

    msg = [
        SYSEX_START,
        MFR_ID,
        CMD_WRITE_PAGE,
        (page >> 7) & 0x7F,
        page & 0x7F,
        (length >> 7) & 0x7F,
        length & 0x7F,
        (checksum >> 4) & 0x0F,
        checksum & 0x0F,
    ]
    msg.extend(encoded)
    msg.append(SYSEX_END)
    return bytes(msg)


# The whole boot section (0x1F000-0x1FFFF) belongs to the RUNNING bootloader.
# Writing any of it from the bootloader bricks the device (recovery = ISP rig).
BOOT_FIRST_PAGE = 0x1F000 // PAGE_SIZE
BOOT_LAST_PAGE = 0x1FFFF // PAGE_SIZE


def emit(ih, outbuf):
    """Emit write-page messages for only the pages the image actually covers;
    gap pages are never sent (writing them as 0xFF would erase EEPROM-free
    flash some future feature may claim)."""
    pages = set()
    for start, stop in ih.segments():
        pages.update(range(start // PAGE_SIZE, (stop - 1) // PAGE_SIZE + 1))

    for page in sorted(pages):
        if BOOT_FIRST_PAGE <= page <= BOOT_LAST_PAGE:
            sys.stderr.write(
                "hex2sysex: skipping page 0x%X (running bootloader; writing "
                "it over MIDI would brick the device)\n" % page)
            continue
        data = ih.tobinarray(start=page * PAGE_SIZE, size=PAGE_SIZE)
        outbuf.write(build_sysex(page, data))

    outbuf.write(bytes([SYSEX_START, MFR_ID, CMD_EXECUTE, SYSEX_END]))


def main():
    if len(sys.argv) < 2:
        print("usage: hex2sysex firmware.hex > update.syx")
        return
    emit(IntelHex(sys.argv[1]), sys.stdout.buffer)


if __name__ == "__main__":
    main()
