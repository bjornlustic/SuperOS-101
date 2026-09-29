#!/usr/bin/env python3
"""sh101-photo-wikimedia.jpg -> panel-photo.jpg: level, crop to the unit, even out exposure.

The source (Wikimedia Commons, "SH-101.jpg" by Wikipedia user 909 bd, CC BY-SA 3.0)
is tilted by about 0.25 degrees (the button strip's top rule runs 523 -> 530 px and
its lower ledge 683 -> 693 px across the panel). It is rotated level, cropped to the
panel edge, and given a mild per-channel levels stretch (half strength), nothing else.

    python3 prepare_photo.py
"""
import os

from PIL import Image, ImageOps

HERE = os.path.dirname(os.path.abspath(__file__))
ANGLE = 0.25                     # degrees, counter-clockwise
CROP = (4, 6, 2338, 1302)        # after rotation: the unit's left, top, right edge and the key fronts


def main():
    im = Image.open(os.path.join(HERE, "sh101-photo-wikimedia.jpg")).convert("RGB")
    im = im.rotate(ANGLE, resample=Image.BICUBIC, fillcolor=(40, 40, 40)).crop(CROP)
    stretched = Image.merge("RGB", [ImageOps.autocontrast(c, cutoff=0.4) for c in im.split()])
    im = Image.blend(im, stretched, 0.5)
    im.save(os.path.join(HERE, "panel-photo.jpg"), quality=85, optimize=True, progressive=True)
    print("panel-photo.jpg", im.size)


if __name__ == "__main__":
    main()
