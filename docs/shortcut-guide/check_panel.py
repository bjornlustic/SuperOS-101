#!/usr/bin/env python3
"""Alignment proof: every measured box of panel.py drawn over panel-photo.jpg.

Buttons, keys and knobs in red, the LED windows in yellow, each labelled with
its id. Writes check/overlay.png and three zoomed thirds of the lower half;
anything measured wrong shows as a box off its control.

    python3 check_panel.py [outdir]
"""
import os
import sys

from PIL import Image, ImageDraw

import panel

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "check")
    os.makedirs(out, exist_ok=True)
    im = Image.open(os.path.join(HERE, panel.PHOTO)).convert("RGB")
    d = ImageDraw.Draw(im)
    for k, (cx, cy, w, h) in panel.centres().items():
        box = (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)
        if panel.KEYS[k][4] == "knob":
            d.ellipse(box, outline=(255, 30, 30), width=2)
        else:
            d.rectangle(box, outline=(255, 30, 30), width=2)
        d.text((box[0] + 3, box[1] + 3), k.replace("key", ""), fill=(255, 255, 0))
    for k, x in panel.LED_X.items():
        d.rectangle((x - panel.LED_W / 2, panel.LED_Y - panel.LED_H / 2,
                     x + panel.LED_W / 2, panel.LED_Y + panel.LED_H / 2), outline=(255, 230, 0), width=2)
    im.save(os.path.join(out, "overlay.png"))
    w, h = im.size
    for i in range(3):
        x0 = i * w // 3
        z = im.crop((x0, 480, x0 + w // 3, h))
        z.resize((z.width * 2, z.height * 2), Image.LANCZOS).save(os.path.join(out, f"overlay_{i}.png"))
    print(os.path.join(out, "overlay.png"))


if __name__ == "__main__":
    main()
