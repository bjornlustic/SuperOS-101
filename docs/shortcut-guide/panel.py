#!/usr/bin/env python3
"""The Roland SH-101 panel as black-and-white line art (SVG).

Laid out after the owner's manual drawings (SH101-reference/SH101_OM.pdf: p.3
panel overview, p.31 controllers, p.33 function buttons) in a 1000-unit wide
frame. Upper panel: TUNE, MODULATOR, VCO, SOURCE MIXER, VCF, VCA, ENV. Middle
strip: the seven function buttons with their LEDs. Lower: the controller
section (VOLUME, PORTAMENTO, bender, PORTAMENTO and TRANSPOSE switches) and
the 32-key keyboard, F to C (key1 = lowest F, key8 = lowest C).

svg(state, crop): state = {"LOAD": "on" | "blink", ...} as capture.py reports
it (LED ids LOAD PLAY DOWN UD UP HOLD KT), crop "full" or "play" (strip,
controllers and keyboard).
centres(): id -> (cx, cy, w, h) of every control a callout can point at.
"""
FONT = "Helvetica Neue, Helvetica, Arial, sans-serif"
WEIGHT = 500                                  # Helvetica Neue Medium
X0, X1, Y0, Y1 = 0, 1000, 0, 500
CROPS = {"full": (Y0, Y1), "play": (208, Y1)}
LED_ON = "#e3001b"
LEDS = ["LOAD", "PLAY", "DOWN", "UD", "UP", "HOLD", "KT"]
LED_NAME = {"LOAD": "LOAD", "PLAY": "PLAY", "DOWN": "DOWN", "UD": "U&D", "UP": "UP", "HOLD": "HOLD",
            "KT": "KEY TRANSP."}

# keyboard: 32 keys F..C
KB_X0, KB_X1, KB_Y0, KB_Y1 = 200, 990, 292, 490
WHITE_W = (KB_X1 - KB_X0) / 19
BLACK_W, BLACK_H = 24, 118
BLACK = {1, 3, 5, 8, 10, 13, 15, 17, 20, 22, 25, 27, 29}   # 0-based semitones from F that are black

KEYS = {}                                   # id: (cx, cy, w, h, kind)
_w = 0
_white_x = {}
for k in range(32):
    if k in BLACK:
        continue
    _white_x[k] = KB_X0 + _w * WHITE_W
    _w += 1
for k in range(32):
    if k in BLACK:
        x = _white_x[k - 1] + WHITE_W - BLACK_W / 2
        KEYS[f"key{k + 1}"] = (x + BLACK_W / 2, KB_Y0 + BLACK_H / 2, BLACK_W, BLACK_H, "black")
    else:
        x = _white_x[k]                     # callout box: the part below the black keys
        y0 = KB_Y0 + BLACK_H + 2
        KEYS[f"key{k + 1}"] = (x + WHITE_W / 2, (y0 + KB_Y1) / 2, WHITE_W, KB_Y1 - y0, "white")

# function buttons, over the left of the keyboard (OM p.33)
BTN_Y, BTN_W, BTN_H, LED_Y = 241, 38, 17, 219
BTN_X = {"LOAD": 232, "PLAY": 278, "DOWN": 352, "UD": 398, "UP": 444, "HOLD": 518, "KT": 580}
for b, x in BTN_X.items():
    KEYS[b] = (x, BTN_Y, BTN_W, BTN_H, "button")
KEYS["POWER"] = (60, 243, 34, 17, "button")
# controller section (OM p.31)
KEYS["VOLUME"] = (55, 318, 40, 40, "knob")
KEYS["PORTAKNOB"] = (140, 318, 40, 40, "knob")
KEYS["PORTASW"] = (150, 386, 56, 14, "switch3")
KEYS["TRANSPOSE"] = (150, 428, 56, 14, "switch3")
KEYS["BENDER"] = (100, 470, 150, 16, "bender")
# upper panel
KEYS["TUNE"] = (45, 118, 40, 40, "knob")
KEYS["RATE"] = (110, 118, 16, 110, "slider")


def esc(s):
    return str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def T(x, y, s, size=9.0, anchor="middle", fill="#000"):
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" font-weight="{WEIGHT}" '
            f'text-anchor="{anchor}" fill="{fill}">{esc(s)}</text>')


def led(x, y, w, h, st, rx=1.0):
    """An LED window: off = outline, on = filled, blink = lower half filled."""
    base = f'<rect x="{x - w / 2:.1f}" y="{y - h / 2:.1f}" width="{w}" height="{h}" rx="{rx}" '
    if st == "on":
        return base + f'fill="{LED_ON}" stroke="{LED_ON}" stroke-width="1"/>'
    if st == "blink":
        return (base + f'fill="#fff" stroke="{LED_ON}" stroke-width="1.2"/>'
                f'<rect x="{x - w / 2:.1f}" y="{y:.1f}" width="{w}" height="{h / 2:.1f}" fill="{LED_ON}"/>')
    return base + 'fill="#fff" stroke="#000" stroke-width="1"/>'


def knob(cx, cy, r, ticks=11):
    import math
    out = [f'<circle cx="{cx}" cy="{cy}" r="{r}" fill="#fff" stroke="#000" stroke-width="1.5"/>']
    for i in range(ticks):
        a = math.radians(-225 + 270 * i / (ticks - 1))
        out.append(f'<line x1="{cx + (r + 2) * math.cos(a):.1f}" y1="{cy + (r + 2) * math.sin(a):.1f}" '
                   f'x2="{cx + (r + 6) * math.cos(a):.1f}" y2="{cy + (r + 6) * math.sin(a):.1f}" '
                   'stroke="#000" stroke-width="0.8"/>')
    out.append(f'<line x1="{cx}" y1="{cy}" x2="{cx}" y2="{cy - r + 2}" stroke="#000" stroke-width="1.6"/>')
    return "".join(out)


def slider(x, y0, y1, label, capy=None, size=7):
    """A slider slot with a scale and a cap; label above."""
    capy = (y0 + y1) / 2 if capy is None else capy
    o = [T(x, y0 - 7, label, size)]
    for i in range(11):
        yy = y0 + (y1 - y0) * i / 10
        o.append(f'<line x1="{x - 11}" y1="{yy:.1f}" x2="{x - 6}" y2="{yy:.1f}" stroke="#000" stroke-width="0.6"/>')
    o.append(f'<rect x="{x - 2.5}" y="{y0}" width="5" height="{y1 - y0}" rx="2" fill="#fff" stroke="#000" stroke-width="1"/>')
    o.append(f'<rect x="{x - 8}" y="{capy - 6}" width="16" height="12" rx="1.5" fill="#fff" stroke="#000" stroke-width="1.4"/>'
             f'<line x1="{x - 6}" y1="{capy}" x2="{x + 6}" y2="{capy}" stroke="#000" stroke-width="1"/>')
    return "".join(o)


def vswitch(x, y, labels, pos, size=6.3):
    """Vertical 3-way lever switch with labels to the right, top to bottom."""
    o = [f'<rect x="{x - 4}" y="{y - 16}" width="8" height="32" rx="2" fill="#fff" stroke="#000" stroke-width="1"/>',
         f'<rect x="{x - 6}" y="{y - 16 + pos * 13:.1f}" width="12" height="6" rx="1" fill="#000"/>']
    for i, s in enumerate(labels):
        o.append(T(x + 8, y - 11 + i * 13, s, size, "start"))
    return "".join(o)


def hswitch(cx, cy, labels, pos, title):
    """Horizontal 3-way slide switch (PORTAMENTO, TRANSPOSE), labels above, title below."""
    w, h = KEYS["PORTASW"][2], KEYS["PORTASW"][3]
    o = [f'<rect x="{cx - w / 2}" y="{cy - h / 2}" width="{w}" height="{h}" rx="{h / 2}" fill="#fff" '
         'stroke="#000" stroke-width="1.3"/>']
    px = cx - w / 2 + h / 2 + pos * (w - h) / 2
    o.append(f'<circle cx="{px:.1f}" cy="{cy}" r="{h / 2 - 1.5}" fill="#000"/>')
    for i, s in enumerate(labels):
        o.append(T(cx - w / 2 + h / 2 + i * (w - h) / 2, cy - h / 2 - 3, s, 6.3))
    o.append(T(cx, cy + h / 2 + 9, title, 6.8))
    return "".join(o)


def section(x0, x1, title):
    return (f'<rect x="{x0}" y="16" width="{x1 - x0}" height="13" fill="#000"/>'
            + T((x0 + x1) / 2, 25.8, title, 8.2, fill="#fff"))


def upper():
    o = []
    o.append(f'<text x="14" y="11" font-size="9" font-weight="{WEIGHT}">Roland</text>')
    o.append(T(45, 70, "TUNE", 7) + knob(45, 118, 16))
    o.append(section(80, 200, "MODULATOR"))
    o.append(slider(110, 60, 175, "RATE"))
    o.append(T(160, 70, "WAVEFORM", 6.5) + knob(160, 118, 15, 4))
    o.append(section(205, 380, "VCO"))
    o.append(slider(228, 60, 175, "MOD", 150))
    o.append(T(272, 70, "RANGE", 6.5) + knob(272, 118, 15, 4))
    o.append(slider(318, 60, 175, "PW", 160))
    o.append(T(357, 70, "PWM", 6.5) + vswitch(350, 118, ["LFO", "MAN", "ENV"], 1))
    o.append(section(385, 560, "SOURCE MIXER"))
    for i, s in enumerate(["PULSE", "SAW", "SUB", "NOISE"]):
        o.append(slider(405 + i * 34, 60, 175, s, [70, 110, 160, 170][i]))
    o.append(T(537, 70, "SUB", 6.5) + vswitch(530, 118, ["1", "2", "3"], 0))
    o.append(section(565, 760, "VCF"))
    for i, s in enumerate(["FREQ", "RES", "ENV", "MOD", "KYBD"]):
        o.append(slider(587 + i * 38, 60, 175, s, [100, 140, 120, 170, 90][i]))
    o.append(section(765, 815, "VCA"))
    o.append(T(790, 70, "ENV", 6.3) + vswitch(784, 118, ["ENV", "", "GATE"], 0))
    o.append(section(820, 990, "ENV"))
    o.append(T(846, 70, "TRIG", 6.3) + vswitch(836, 118, ["G+T", "GATE", "LFO"], 1))
    for i, s in enumerate(["A", "D", "S", "R"]):
        o.append(slider(882 + i * 30, 60, 175, s, [170, 130, 90, 150][i]))
    for x in [202.5, 382.5, 562.5, 762.5, 817.5]:
        o.append(f'<line x1="{x}" y1="16" x2="{x}" y2="190" stroke="#000" stroke-width="0.8"/>')
    o.append('<line x1="6" y1="200" x2="994" y2="200" stroke="#000" stroke-width="1.4"/>')
    return "".join(o)


def strip(state):
    o = []
    o.append(T(60, 227, "POWER", 6.8))
    x, y, w, h = KEYS["POWER"][:4]
    o.append(f'<rect x="{x - w / 2}" y="{y - h / 2}" width="{w}" height="{h}" rx="2" fill="#fff" stroke="#000" stroke-width="1.3"/>'
             f'<rect x="{x - w / 2 + 3}" y="{y - h / 2 + 3}" width="{w / 2 - 3}" height="{h - 6}" fill="#000"/>')
    for b, x in BTN_X.items():
        o.append(led(x, LED_Y, 11, 6, state.get(b), 1))
        o.append(f'<rect x="{x - BTN_W / 2}" y="{BTN_Y - BTN_H / 2}" width="{BTN_W}" height="{BTN_H}" rx="2" '
                 'fill="#fff" stroke="#000" stroke-width="1.6"/>'
                 f'<rect x="{x - BTN_W / 2 + 3.5}" y="{BTN_Y - BTN_H / 2 + 3.5}" width="{BTN_W - 7}" '
                 f'height="{BTN_H - 7}" rx="1.5" fill="none" stroke="#000" stroke-width="0.8"/>')
    ly = BTN_Y + BTN_H / 2 + 16
    for b, s in [("LOAD", "LOAD"), ("PLAY", "PLAY"), ("DOWN", "DOWN"), ("UD", "U&D"), ("UP", "UP"),
                 ("HOLD", "HOLD")]:
        o.append(T(BTN_X[b], ly, s, 8))
    o.append(T(BTN_X["KT"], ly, "KEY TRANSPOSE", 7.6))
    gy = ly + 12
    for a, b, s in [("LOAD", "PLAY", "SEQUENCER"), ("DOWN", "UP", "ARPEGGIO"), ("HOLD", "HOLD", "LEGATO"),
                    ("KT", "KT", "REST")]:
        x0, x1 = BTN_X[a] - BTN_W / 2, BTN_X[b] + BTN_W / 2
        o.append(f'<path d="M{x0},{gy - 7} L{x0},{gy - 4} L{x1},{gy - 4} L{x1},{gy - 7}" fill="none" '
                 'stroke="#000" stroke-width="0.8"/>' + T((x0 + x1) / 2, gy + 4, s, 7.6))
    o.append(f'<text x="985" y="258" font-size="30" font-weight="{WEIGHT}" text-anchor="end" '
             'letter-spacing="1">SH-101</text>')
    o.append(T(985, 272, "SYNTHESIZER", 7.5, "end"))
    return "".join(o)


def controllers():
    o = []
    o.append(T(55, 290, "VOLUME", 7) + knob(55, 318, 16))
    o.append(T(140, 290, "PORTAMENTO", 7) + knob(140, 318, 16))
    for i, s in enumerate(["VCO", "VCF", "MOD"]):
        o.append(slider(30 + i * 34, 372, 440, s, 430, 6.3))
    o.append(hswitch(150, 386, ["AUTO", "OFF", "ON"], 0, "PORTAMENTO"))
    o.append(hswitch(150, 428, ["L", "M", "H"], 1, "TRANSPOSE"))
    x, y, w, h = KEYS["BENDER"][:4]
    o.append(f'<rect x="{x - w / 2}" y="{y - h / 2}" width="{w}" height="{h}" rx="3" fill="#fff" stroke="#000" stroke-width="1.4"/>'
             f'<rect x="{x - 9}" y="{y - h / 2 + 3}" width="18" height="{h - 6}" rx="2" fill="#000"/>'
             + T(x, y + h / 2 + 9, "BENDER", 6.8))
    return "".join(o)


def keyboard():
    o = [f'<rect x="{KB_X0 - 4}" y="{KB_Y0 - 4}" width="{KB_X1 - KB_X0 + 8}" height="{KB_Y1 - KB_Y0 + 6}" '
         'fill="#fff" stroke="#000" stroke-width="1.2"/>']
    for k in range(32):
        if k in BLACK:
            continue
        x = _white_x[k]
        o.append(f'<rect x="{x:.1f}" y="{KB_Y0}" width="{WHITE_W:.1f}" height="{KB_Y1 - KB_Y0}" rx="1.5" '
                 'fill="#fff" stroke="#000" stroke-width="1.2"/>')
        o.append(T(x + WHITE_W / 2, KB_Y1 - 8, str(k + 1), 12, fill="#707070"))
        if k == 7:
            o.append(T(x + WHITE_W / 2, KB_Y1 - 24, "C", 10, fill="#707070"))
    for k in range(32):
        if k not in BLACK:
            continue
        cx, cy, w, h, _ = KEYS[f"key{k + 1}"]
        o.append(f'<rect x="{cx - w / 2:.1f}" y="{KB_Y0}" width="{w}" height="{h}" rx="1.5" fill="#000"/>'
                 + T(cx, KB_Y0 + h - 20, str(k + 1), 10, fill="#fff"))
    return "".join(o)


def body(state, crop):
    o = [f'<rect x="{X0 + 1}" y="{Y0 + 1}" width="{X1 - X0 - 2}" height="{Y1 - Y0 - 2}" fill="#fff" '
         'stroke="#000" stroke-width="2"/>']
    if crop == "full":
        o.append(upper())
    o += [strip(state), controllers(), keyboard()]
    return "".join(o)


def svg(state, crop="play"):
    y0, y1 = CROPS[crop]
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{X0 - 2} {y0 - 2} {X1 - X0 + 4} {y1 - y0 + 4}" '
            f'font-family="{FONT}">{body(state, crop)}</svg>')


def centres():
    return {k: v[:4] for k, v in KEYS.items()}


if __name__ == "__main__":                    # writes panel.svg for a look
    import os
    open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "panel.svg"), "w").write(
        svg({"PLAY": "on", "HOLD": "on", "KT": "blink"}, "full"))
