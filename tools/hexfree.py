#!/usr/bin/env python3
"""Flash bytes free below a ceiling, from an Intel hex file. Stdlib only.

    python3 hexfree.py firmware.hex [ceiling]     (ceiling default 0x16000)

Prints `end 0x<next free byte> free <bytes>` and exits 1 when the image reaches
the ceiling. The ceiling is the flash arena (tools/makesyx.py FLASH_ARENA_BASE),
not the size of the part, which is what PlatformIO's percentage is against.
makesyx only refuses an overrun; it never prints the margin.
"""
import sys


def image_end(path):
    base, end = 0, 0
    with open(path) as f:
        for n, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            if line[0] != ":":
                raise SystemExit("%s:%d: not an Intel hex record" % (path, n))
            b = bytes.fromhex(line[1:])
            if sum(b) & 0xFF:
                raise SystemExit("%s:%d: bad checksum" % (path, n))
            size, addr, kind = b[0], (b[1] << 8) | b[2], b[3]
            if kind == 0 and size:
                end = max(end, base + addr + size)
            elif kind == 2:
                base = ((b[4] << 8) | b[5]) << 4
            elif kind == 4:
                base = ((b[4] << 8) | b[5]) << 16
    return end


if __name__ == "__main__":
    if len(sys.argv) not in (2, 3):
        raise SystemExit(__doc__)
    ceiling = int(sys.argv[2], 0) if len(sys.argv) == 3 else 0x16000
    end = image_end(sys.argv[1])
    print("end 0x%X free %d" % (end, ceiling - end))
    sys.exit(0 if end <= ceiling else 1)
