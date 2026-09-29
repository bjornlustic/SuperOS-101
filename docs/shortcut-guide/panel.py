#!/usr/bin/env python3
"""The Roland SH-101 panel: the real photo, with the LED states drawn on it.

panel-photo.jpg (2334 x 1296) is prepare_photo.py's levelled crop of
sh101-photo-wikimedia.jpg (Wikimedia Commons, "SH-101.jpg" by Wikipedia user
909 bd, CC BY-SA 3.0). Every position below is measured on panel-photo.jpg,
in its pixels; check_panel.py draws the boxes over the photo to prove it.

svg(state, crop): the photo (as an <image> referencing panel-photo.jpg, so
the HTML that inlines it must sit next to the file) with each LED of state
({"LOAD": "on" | "blink", ...}, ids LOAD PLAY DOWN UD UP HOLD KT, as
capture.py reports) lit on the photo. crop "full", "play" (button strip,
controllers and keyboard) or "left" (the same, POWER to key 15).
centres(): id -> (cx, cy, w, h) of every control a callout can point at.
"""
FONT = "Helvetica Neue, Helvetica, Arial, sans-serif"
WEIGHT = 500                                  # Helvetica Neue Medium
PHOTO = "panel-photo.jpg"
X0, X1, Y0, Y1 = 0, 2334, 0, 1296
# crop: (x0, y0, x1, y1). "left" = POWER to key 15 with the controllers, for scenes that stay there.
CROPS = {"full": (X0, Y0, X1, Y1), "play": (X0, 508, X1, Y1), "left": (X0, 508, 1262, Y1)}
LED_ON = "#e3001b"
LEDS = ["LOAD", "PLAY", "DOWN", "UD", "UP", "HOLD", "KT"]
LED_NAME = {"LOAD": "LOAD", "PLAY": "PLAY", "DOWN": "DOWN", "UD": "U&D", "UP": "UP", "HOLD": "HOLD",
            "KT": "KEY TRANSP."}

# id: (cx, cy, w, h, kind)
KEYS = {}
# function buttons (outer bezel) and their LED windows
BTN_X = {"LOAD": 438, "PLAY": 507, "DOWN": 600, "UD": 669, "UP": 737, "HOLD": 830, "KT": 924}
LED_X = {"LOAD": 438, "PLAY": 507, "DOWN": 600, "UD": 669, "UP": 737, "HOLD": 831, "KT": 923}
BTN_Y, BTN_W, BTN_H = 582, 54, 30
LED_Y, LED_W, LED_H = 544, 23, 11
for b, x in BTN_X.items():
    KEYS[b] = (x, BTN_Y, BTN_W, BTN_H, "button")
KEYS["POWER"] = (165, 580, 69, 40, "button")

# keyboard: 19 white keys (spans at the key fronts) and 13 black keys
WHITE = [(376, 470), (475, 569), (573, 666), (672, 765), (770, 863), (868, 960), (967, 1059),
         (1066, 1158), (1164, 1255), (1262, 1354), (1360, 1452), (1457, 1549), (1556, 1648),
         (1654, 1747), (1754, 1846), (1854, 1947), (1952, 2046), (2052, 2146), (2150, 2242)]
BLACKS = [(432, 491), (543, 603), (654, 713), (825, 884), (947, 1008), (1119, 1178), (1228, 1290),
          (1352, 1400), (1524, 1571), (1648, 1695), (1823, 1870), (1934, 1982), (2048, 2095)]
KB_TOP, BLACK_BOT, KB_BOT = 703, 1075, 1287
BLACK = {1, 3, 5, 8, 10, 13, 15, 17, 20, 22, 25, 27, 29}   # semitones from F that are black
_w = iter(WHITE)
_b = iter(BLACKS)
for k in range(32):
    if k in BLACK:
        a, b = next(_b)
        KEYS[f"key{k + 1}"] = ((a + b) / 2, (KB_TOP + BLACK_BOT) / 2, b - a, BLACK_BOT - KB_TOP, "black")
    else:
        a, b = next(_w)                     # callout box: the key front below the black keys
        y0 = BLACK_BOT + 6
        KEYS[f"key{k + 1}"] = ((a + b) / 2, (y0 + KB_BOT) / 2, b - a, KB_BOT - y0, "white")

# controller section
KEYS["VOLUME"] = (89, 817, 86, 86, "knob")
KEYS["PORTAKNOB"] = (282, 817, 86, 86, "knob")
KEYS["PORTASW"] = (287, 959, 92, 30, "switch")
KEYS["TRANSPOSE"] = (287, 1061, 92, 32, "switch")
KEYS["BENDER"] = (182, 1215, 184, 48, "bender")
for name, x, top in [("BEND_VCO", 58, 920), ("BEND_VCF", 120, 920), ("BEND_MOD", 183, 919)]:
    KEYS[name] = (x, (top + 1104) / 2, 40, 1104 - top, "slider")
# upper panel, for orientation
KEYS["TUNE"] = (167, 365, 112, 112, "knob")
KEYS["RATE"] = (296, 385, 40, 196, "slider")
for name, a, b in [("MODULATOR", 252, 500), ("VCO", 502, 936), ("SOURCEMIXER", 938, 1365),
                   ("VCF", 1367, 1756), ("VCA", 1758, 1868), ("ENV", 1871, 2323)]:
    KEYS[name] = ((a + b) / 2, 334, b - a, 368, "section")


def esc(s):
    return str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def T(x, y, s, size=9.0, anchor="middle", fill="#000"):
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" font-weight="{WEIGHT}" '
            f'text-anchor="{anchor}" fill="{fill}">{esc(s)}</text>')


def led(x, y, w, h, st, rx=1.0):
    """A flat LED window for the large readout: off = outline, on = filled, blink = lower half."""
    base = f'<rect x="{x - w / 2:.1f}" y="{y - h / 2:.1f}" width="{w}" height="{h}" rx="{rx}" '
    if st == "on":
        return base + f'fill="{LED_ON}" stroke="{LED_ON}" stroke-width="1"/>'
    if st == "blink":
        return (base + f'fill="#fff" stroke="{LED_ON}" stroke-width="1.2"/>'
                f'<rect x="{x - w / 2:.1f}" y="{y:.1f}" width="{w}" height="{h / 2:.1f}" fill="{LED_ON}"/>')
    return base + 'fill="#fff" stroke="#000" stroke-width="1"/>'


DEFS = ('<defs><radialGradient id="sh101glow"><stop offset="0" stop-color="#ff5a3c" stop-opacity="0.95"/>'
        '<stop offset="0.35" stop-color="#ff2a14" stop-opacity="0.6"/>'
        '<stop offset="1" stop-color="#ff2a14" stop-opacity="0"/></radialGradient>'
        '<linearGradient id="sh101core" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#ffb3a0"/>'
        '<stop offset="0.45" stop-color="#ff3a22"/><stop offset="1" stop-color="#d81008"/></linearGradient></defs>')


def photo_led(x, y, st):
    """A lit LED on the photo: a glowing red window. Blink: a dimmer window, the glow on
    its lower half only, and a dashed red ring around it."""
    w, h = LED_W, LED_H
    core = (f'<rect x="{x - w / 2:.1f}" y="{y - h / 2:.1f}" width="{w}" height="{h}" rx="2" '
            'fill="url(#sh101core)" stroke="#7a0000" stroke-width="0.8"')
    if st == "on":
        return (f'<ellipse cx="{x}" cy="{y}" rx="{w * 1.9}" ry="{h * 3.2}" fill="url(#sh101glow)"/>'
                + core + '/>')
    return (f'<ellipse cx="{x}" cy="{y + h * 0.35:.1f}" rx="{w * 1.3}" ry="{h * 1.8}" fill="url(#sh101glow)" '
            'opacity="0.7"/>'
            + core + ' opacity="0.75"/>'
            f'<rect x="{x - w / 2:.1f}" y="{y - h / 2:.1f}" width="{w}" height="{h / 2:.1f}" rx="1.5" '
            'fill="#3a0806" opacity="0.7"/>'
            f'<rect x="{x - w / 2 - 8:.1f}" y="{y - h / 2 - 7:.1f}" width="{w + 16}" height="{h + 14}" rx="6" '
            f'fill="none" stroke="#ff5a40" stroke-width="3" stroke-dasharray="5 4"/>')


def key_numbers():
    """Small key numbers, so a key can be named (key1 = lowest F, key8 = lowest C)."""
    o = []
    for n in range(1, 33):
        cx, cy, w, h, kind = KEYS[f"key{n}"]
        if kind == "white":
            o.append(T(cx, KB_BOT - 14, str(n), 22, fill="#8c8c8c"))
            if n == 8:
                o.append(T(cx, KB_BOT - 44, "C", 18, fill="#8c8c8c"))
        else:
            o.append(T(cx, BLACK_BOT - 48, str(n), 19, fill="#d8d8d8"))
    return "".join(o)


def leds(state):
    """The lit and blinking LEDs, in photo coordinates (draw them last, over any callout)."""
    return DEFS + "".join(photo_led(LED_X[k], LED_Y, state[k]) for k in LEDS if state.get(k))


def body(state, with_leds=True, numbers=True):
    o = [f'<image href="{PHOTO}" x="0" y="0" width="{X1}" height="{Y1}" preserveAspectRatio="none"/>',
         key_numbers() if numbers else ""]
    if with_leds:
        o.append(leds(state))
    return "".join(o)


def svg(state, crop="play", with_leds=True, numbers=True):
    x0, y0, x1, y1 = CROPS[crop] if isinstance(crop, str) else crop
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{x0} {y0} {x1 - x0} {y1 - y0}" '
            f'font-family="{FONT}">{body(state, with_leds, numbers)}</svg>')


def centres():
    return {k: v[:4] for k, v in KEYS.items()}
