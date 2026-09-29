#!/usr/bin/env python3
"""Build the SuperOS-101 Shortcut Guide (HTML + PDF).

One drawing of the SH-101 panel per function, with the LEDs the firmware
shows at that moment, and the keys in order underneath. Same shape as the
TR-909, TB-303, S900 and S950 guides.

Panel art: panel.py. States: capture.py runs every scene of content.py in the
host simulator (sim/superos101-sim, the real firmware); the build stops if a
scene does not show exactly the LEDs its expect list says. Text: content.py.

    python3 docs/shortcut-guide/build_guide.py
"""
import base64
import datetime
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import panel                        # noqa: E402
from content import CHAPTERS        # noqa: E402

CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
OUT_HTML = os.path.join(HERE, "guide.html")
OUT_PDF = os.path.join(HERE, "SuperOS-101-Shortcut-Guide.pdf")
REV = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "buildrev.py"), "--print"],
                     capture_output=True, text=True).stdout.strip() or "unknown"
VERSION = f"Build {datetime.date.today().isoformat()} ({REV})"
RED, BLUE = "#e3001b", "#2f7fd0"

# ---- one drawing with callouts (the S900 guide's diagram.py, sized for this panel) ----
PRINT_MM = 134.0                    # printed width of a diagram
KEYS = panel.centres()


def esc(s):
    return str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def bbox(btn):
    ids = btn.split("|")
    for i in ids:
        if i not in KEYS:
            raise KeyError(f"no such control: {i}")
    x0 = min(KEYS[i][0] - KEYS[i][2] / 2 for i in ids)
    y0 = min(KEYS[i][1] - KEYS[i][3] / 2 for i in ids)
    x1 = max(KEYS[i][0] + KEYS[i][2] / 2 for i in ids)
    y1 = max(KEYS[i][1] + KEYS[i][3] / 2 for i in ids)
    return x0, y0, x1, y1


def draw(callouts, state, crop="play"):
    X0, X1 = panel.X0 - 2, panel.X1 + 2
    Y0, Y1 = panel.CROPS[crop][0] - 2, panel.CROPS[crop][1] + 2
    vw = X1 - X0
    mm = vw / PRINT_MM
    FS, LH, PADX, GAP = 1.85 * mm, 3.6 * mm, 1.3 * mm, 1.0 * mm
    BAND, RING, LEAD = LH + 4.6 * mm, 0.45 * mm, 0.28 * mm
    top = BAND if any(c["side"] == "t" for c in callouts) else 1.5 * mm
    bot = BAND if any(c["side"] == "b" for c in callouts) else 1.5 * mm
    rows = {"t": [], "b": []}
    for c in callouts:
        x0, y0, x1, y1 = bbox(c["btn"])
        c.update(_cx=(x0 + x1) / 2, _cy=(y0 + y1) / 2, _cw=x1 - x0, _ch=y1 - y0,
                 _w=len(c["label"]) * (FS * 0.64 + 0.08 * mm) + 2 * PADX)
        c["_x"] = c["_cx"] - c["_w"] / 2
        rows[c["side"]].append(c)
    for side, g in rows.items():
        g.sort(key=lambda c: c["_cx"])
        if sum(c["_w"] + GAP for c in g) > vw:
            raise SystemExit(f"callout row {side} too wide: {[c['label'] for c in g]}")
        lo = X0 + GAP
        for c in g:
            c["_x"] = max(c["_x"], lo)
            lo = c["_x"] + c["_w"] + GAP
        hi = X1 - GAP
        for c in reversed(g):
            c["_x"] = min(c["_x"], hi - c["_w"])
            hi = c["_x"] - GAP
    H = Y1 - Y0 + top + bot
    body = panel.svg(state, crop)
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{X0} {Y0 - top:.1f} {vw} {H:.1f}" '
           f'font-family="{panel.FONT}" stroke-linejoin="round" stroke-linecap="round">',
           f'<rect x="{X0}" y="{Y0 - top:.1f}" width="{vw}" height="{H:.1f}" fill="#fff"/>',
           body.replace("<svg ", f'<svg x="{X0}" y="{Y0}" width="{vw}" height="{Y1 - Y0}" ', 1)]   # nested: clips the cut
    lines, rings, pills = [], [], []
    for c in callouts:
        col = RED if c["hold"] else (BLUE if c["box"] else "#000")
        cx, cy, cw, ch = c["_cx"], c["_cy"], c["_cw"], c["_ch"]
        m = (1.1 if c["box"] else 0.9) * mm
        dash = f' stroke-dasharray="{1.6 * mm:.1f} {1.0 * mm:.1f}"' if c["box"] else ""
        rx = m if c["box"] else min(cw, ch) / 2 + m if c["btn"] in ("VOLUME", "PORTAKNOB", "TUNE") else 3 + m
        rings.append(f'<rect x="{cx - cw / 2 - m:.1f}" y="{cy - ch / 2 - m:.1f}" width="{cw + 2 * m:.1f}" '
                     f'height="{ch + 2 * m:.1f}" rx="{rx:.1f}" fill="none" stroke="{col}" '
                     f'stroke-width="{RING:.2f}"{dash}/>')
        edge_top, edge_bot = cy - ch / 2 - m, cy + ch / 2 + m
        px, w = c["_x"], c["_w"]
        if c["side"] == "t":
            py = Y0 - top + 0.6 * mm
            sx, sy, ey = px + w / 2, py + LH, edge_top
        else:
            py = Y1 + bot - LH - 0.6 * mm
            sx, sy, ey = px + w / 2, py, edge_bot
        knee = sy + (1.8 * mm if c["side"] == "t" else -1.8 * mm)
        tx = min(max(sx, cx - cw / 2 + 6), cx + cw / 2 - 6)   # land inside a wide group, not only at its centre
        lines.append(f'<path d="M{sx:.1f},{sy:.1f} L{sx:.1f},{knee:.1f} L{tx:.1f},{ey:.1f}" fill="none" '
                     f'stroke="{col}" stroke-width="{LEAD:.2f}"/>')
        fill = RED if c["hold"] else "#fff"
        tcol = "#fff" if c["hold"] else (BLUE if c["box"] else "#000")
        pills.append(f'<rect x="{px:.1f}" y="{py:.1f}" width="{w:.1f}" height="{LH:.1f}" rx="{LH / 2:.1f}" '
                     f'fill="{fill}" stroke="{col}" stroke-width="{LEAD:.2f}"/>'
                     f'<text x="{px + w / 2:.1f}" y="{py + LH / 2 + FS * 0.36:.1f}" text-anchor="middle" '
                     f'font-size="{FS:.1f}" font-weight="{panel.WEIGHT}" letter-spacing="{0.08 * mm:.2f}" '
                     f'fill="{tcol}">{esc(c["label"])}</text>')
    halos = [re.sub(r'stroke="[^"]*"', 'stroke="#fff"', r, 1)
             .replace(f'stroke-width="{RING:.2f}"', f'stroke-width="{RING * 1.8:.2f}"')
             .replace(" stroke-dasharray", " data-d") for r in rings]
    out += lines + halos + rings + pills + ["</svg>"]
    return "".join(out)


def leds_svg(state):
    """The seven LEDs large, with their names: the state at a glance."""
    out = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 150 44" '
           f'font-family="{panel.FONT}">']
    for i, k in enumerate(panel.LEDS):
        x = 11 + i * 21.3
        out.append(panel.led(x, 12, 13, 7.5, state.get(k), 1.2))
        name = {"KT": "KEY TR."}.get(k, panel.LED_NAME[k])
        out.append(f'<text x="{x:.1f}" y="27" font-size="5.6" font-weight="{panel.WEIGHT}" '
                   f'text-anchor="middle">{esc(name)}</text>')
        st = state.get(k)
        out.append(f'<text x="{x:.1f}" y="36" font-size="5" font-weight="{panel.WEIGHT}" text-anchor="middle" '
                   f'fill="{"#e3001b" if st else "#8a8a8a"}">{esc((st or "off").upper())}</text>')
    return "".join(out) + "</svg>"


# ---- capture and checks ----
def fmt(state):
    return ", ".join(f"{v}={k}" for k, v in sorted(state.items())) or "all off"


def capture():
    scenes = []
    for _, fns in CHAPTERS:
        for f in fns:
            scenes.append({"name": f["title"], "keys": f["keys"]})
            for i, (keys, _) in enumerate(f["also"]):
                scenes.append({"name": f'{f["title"]} #{i + 1}', "keys": keys})
    res = subprocess.run([sys.executable, os.path.join(HERE, "capture.py")], input=json.dumps(scenes),
                         capture_output=True, text=True, check=True)
    states = json.loads(res.stdout)
    bad = []
    for _, fns in CHAPTERS:
        for f in fns:
            checks = [(f["title"], f["expect"])] + [(f'{f["title"]} #{i + 1}', e)
                                                   for i, (_, e) in enumerate(f["also"])]
            for name, expect in checks:
                want = dict(reversed(e.split("=", 1)) for e in expect)
                if states[name] != want:
                    bad.append(f"{name}: expected {fmt(want)}, got {fmt(states[name])}")
            f["_state"] = states[f["title"]]
    if bad:
        raise SystemExit("capture check failed:\n" + "\n".join(bad))
    return states


def svg_data(svg):
    return "data:image/svg+xml;base64," + base64.b64encode(svg.encode()).decode()


def chip(t, filled=False):
    return f'<span class="chip{" f" if filled else ""}">{esc(t)}</span>'


def block(n, f):
    dia = svg_data(draw(f["callouts"], f["_state"]))
    shot = (f'<div class="shot"><div class="lcd"><img src="{svg_data(leds_svg(f["_state"]))}" alt="">'
            f'</div><p>{esc(f["caption"])}</p></div>')
    steps = "".join(f"<li>{esc(s)}</li>" for s in f["steps"])
    note = f'<div class="note"><strong>Note</strong>{esc(f["note"])}</div>' if f["note"] else ""
    return (f'<section class="fn"><div class="head"><span class="num">{n}</span>'
            f'<h2>{esc(f["title"])}</h2></div><div class="ctx">{chip(f["mode"], True)}{chip(f["when"])}</div>'
            f'<div class="row"><div class="dia"><img src="{dia}" alt=""></div>{shot}</div>'
            f'<ol class="steps">{steps}</ol>{note}</section>')


CSS = """
@page { size: A4; margin: 10mm 10mm 12mm; }
* { box-sizing: border-box; }
html, body { margin: 0; padding: 0; background: #fff; }
body { font-family: 'Helvetica Neue', Helvetica, Arial, sans-serif; font-weight: 500; /* Helvetica Neue Medium */
  color: #000; font-size: 9pt; line-height: 1.42;
  -webkit-print-color-adjust: exact; print-color-adjust: exact;
  font-feature-settings: "kern" 1, "tnum" 1; }
strong { font-weight: 500; }
.brand { display: flex; justify-content: space-between; align-items: baseline; }
.brand-note { font-size: 7pt; font-weight: 500; letter-spacing: 0.28em; text-transform: uppercase; }
.cover h1 { margin: 4mm 0 0; font-size: 34pt; font-weight: 500; letter-spacing: -0.01em; line-height: 1; }
.cover h1 small { display: block; font-size: 12pt; font-weight: 500; letter-spacing: 0.3em;
  text-transform: uppercase; margin-top: 3mm; }
.meta { margin-top: 4mm; padding-top: 3mm; border-top: 3px solid #000; }
.meta p { margin: 0; font-size: 7.4pt; font-weight: 500; letter-spacing: 0.22em;
  text-transform: uppercase; line-height: 1.8; }
.intro { margin-top: 4mm; font-size: 9.5pt; line-height: 1.5; }
.coverart { margin: 8mm 0 0; }
.coverart img { display: block; width: 100%; height: auto; }
.legend { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 3mm; margin-top: 8mm; }
.legend div { border: 1.5px solid #000; border-radius: 8px; padding: 2.5mm 3mm;
  font-size: 8pt; line-height: 1.45; }
.legend strong { display: block; font-size: 7pt; letter-spacing: 0.2em;
  text-transform: uppercase; margin-bottom: 1mm; }
.legend i { display: inline-block; width: 3.6mm; height: 3.6mm; border-radius: 50%;
  border: 2px solid #000; vertical-align: -0.8mm; margin-right: 1.2mm; }
.legend i.h { border-color: #e3001b; background: #e3001b; }
.legend i.on { width: 2.6mm; height: 2.6mm; border: 1px solid #e3001b; background: #e3001b; }
.legend i.bl { width: 2.6mm; height: 2.6mm; border: 1px solid #e3001b;
  background: linear-gradient(#fff 50%, #e3001b 50%); }
.legend i.off { width: 2.6mm; height: 2.6mm; border: 1px solid #9a9a9a; }
.legend b { display: inline-block; width: 7mm; border-top: 2.5px dashed #2f7fd0;
  vertical-align: 1.2mm; margin-right: 1.2mm; }
.part { margin: 0 0 3mm; background: #000; color: #fff; font-size: 10pt; font-weight: 500;
  letter-spacing: 0.35em; text-transform: uppercase; text-align: center;
  padding: 2mm 4mm 2mm 5mm; border-radius: 999px; break-after: avoid; }
.fn { margin: 0 0 6mm; break-inside: avoid; page-break-inside: avoid; }
.head { display: flex; align-items: baseline; gap: 2.5mm; padding-bottom: 1mm;
  border-bottom: 2.4px solid #000; }
.num { flex-shrink: 0; font-size: 9pt; font-weight: 500; border: 1.8px solid #000;
  padding: 0 2mm; line-height: 1.5; }
.head h2 { margin: 0; font-size: 14pt; font-weight: 500; text-transform: uppercase;
  letter-spacing: 0.01em; line-height: 1.1; }
.ctx { display: flex; gap: 1.5mm; margin: 1.2mm 0 1.2mm; flex-wrap: wrap; }
.chip { display: inline-block; font-size: 6.6pt; font-weight: 500; letter-spacing: 0.14em;
  text-transform: uppercase; border: 1.5px solid #000; padding: 0.4mm 2mm; line-height: 1.5;
  white-space: nowrap; }
.chip.f { background: #000; color: #fff; }
.dia img { display: block; width: 100%; height: auto; }
.row { display: grid; grid-template-columns: 136fr 50fr; gap: 4mm; align-items: center; margin-top: 1mm; }
.shot { border: 1.8px solid #000; border-radius: 8px; padding: 2mm; }
.lcd img { display: block; width: 100%; height: auto; }
.shot p { margin: 1.8mm 0 0; font-size: 8pt; line-height: 1.4; }
.steps { margin: 2mm 0 0; padding-left: 5mm; }
.steps li { margin: 0 0 0.8mm; padding-left: 1mm; }
.note { margin: 2mm 0 0; padding: 1.8mm 3mm; border: 1.5px solid #000; border-radius: 8px;
  font-size: 8.2pt; line-height: 1.42; }
.note strong { display: block; font-size: 6.6pt; letter-spacing: 0.2em;
  text-transform: uppercase; margin-bottom: 0.5mm; }
.pg { height: 275mm; display: flex; flex-direction: column; break-after: page; overflow: hidden; }
.pg:last-child { break-after: auto; }
.pfoot { margin-top: auto; padding-top: 1.5mm; border-top: 1.2px solid #000;
  display: flex; justify-content: space-between; font-size: 7pt; font-weight: 500;
  letter-spacing: 0.16em; text-transform: uppercase; }
.toccols { columns: 2; column-gap: 8mm; font-size: 8.8pt; }
.tocpart { break-inside: avoid; break-after: avoid; margin: 3mm 0 1.2mm; font-size: 7.4pt; font-weight: 500;
  letter-spacing: 0.22em; text-transform: uppercase; border-bottom: 2px solid #000;
  padding-bottom: 0.8mm; }
.tocpart:first-child { margin-top: 0; }
.tocrow { display: flex; align-items: baseline; gap: 2mm; margin: 0 0 1.1mm; break-inside: avoid; }
.tocnum { flex-shrink: 0; width: 6mm; font-size: 7pt; font-weight: 500; border: 1.2px solid #000;
  text-align: center; line-height: 1.5; }
.toctitle { font-weight: 500; }
.tocleader { flex: 1; min-width: 3mm; border-bottom: 1.5px dotted #999; transform: translateY(-2px); }
.tocpage { font-weight: 500; font-variant-numeric: tabular-nums; }
.ref { columns: 2; column-gap: 6mm; font-size: 7.6pt; line-height: 1.3; }
.ref h3 { margin: 0 0 1mm; font-size: 8.4pt; font-weight: 500; text-transform: uppercase;
  letter-spacing: 0.04em; break-after: avoid; }
.ref p { margin: 0 0 2.5mm; break-inside: avoid; }
.ref table { width: 100%; border-collapse: collapse; margin: 0 0 2.5mm; font-size: 7.2pt; }
.ref tr { break-inside: avoid; }
.ref th { background: #000; color: #fff; text-align: left; font-size: 6.2pt; letter-spacing: 0.12em;
  text-transform: uppercase; padding: 0.5mm 1.2mm; }
.ref td { border-bottom: 0.8px solid #000; padding: 0.45mm 1.2mm; vertical-align: top; }
.ref td:first-child { width: 48%; }
"""

def ref():
    rows = []
    for name, fns in CHAPTERS:
        rows.append(f'<h3>{esc(name)}</h3><table><tr><th>Gesture</th><th>Function</th></tr>')
        for f in fns:
            gest, what = f["g"].split("|")
            rows.append(f"<tr><td>{esc(gest)}</td><td>{esc(what)}</td></tr>")
        rows.append("</table>")
    return ('<div class="part">Reference</div><div class="ref">'
            '<h3>LEDs</h3><table><tr><th>LED</th><th>Meaning</th></tr>'
            '<tr><td>LOAD lit</td><td>Recording (LOAD); in the menu: 80C49 emulator running</td></tr>'
            '<tr><td>LOAD blinks</td><td>Tune</td></tr>'
            '<tr><td>PLAY lit</td><td>Sequencer playing (or armed); in the menu: SuperOS running</td></tr>'
            '<tr><td>PLAY + LOAD alternate</td><td>Real-time record</td></tr>'
            '<tr><td>DOWN / U&amp;D / UP lit</td><td>Arpeggio direction</td></tr>'
            '<tr><td>U&amp;D blinks</td><td>Random arpeggio</td></tr>'
            '<tr><td>HOLD lit</td><td>HOLD button or pedal down; in the menu: the menu is open</td></tr>'
            '<tr><td>KEY TRANSPOSE lit</td><td>A transpose is active; in the menu: MIDI clock sync on</td></tr>'
            '<tr><td>KEY TRANSPOSE blinks</td><td>Transpose mode</td></tr></table>'
            + "".join(rows) +
            '<h3>MIDI</h3><p>Notes 53 to 84 on the receive channel act as keys 1 to 32 (they arpeggiate and '
            'record); other notes 36 to 99 play the voice. Program change 0 to 7 selects pattern 1 to 8. '
            'Everything played goes out on MIDI OUT. The web editor (tools/web-editor) edits patterns and '
            'settings over USB-C or DIN.</p>'
            '<h3>About these pages</h3><p>The LEDs in every drawing are what the SuperOS-101 firmware '
            f'(rev {esc(REV)}) shows in the host simulator (sim/superos101-sim, the real firmware source) '
            'after the keys of that function, on a fresh machine. The panel drawing follows the Roland '
            'owner\'s manual; most drawings show the button strip, controllers and keyboard. Details: '
            'README.md.</p></div>')


def cover():
    art = svg_data(panel.svg({"PLAY": "on", "HOLD": "on", "KT": "blink"}, "full"))
    return f"""<div class="cover">
<div class="brand"><span class="brand-note">Firmware Documentation</span>
<span class="brand-note">Panel Reference</span></div>
<h1>SuperOS-101<small>Shortcut Guide</small></h1>
<div class="meta"><p>{VERSION} &middot; Custom firmware for the Roland SH-101<br>
AT90USB1286 CPU board &middot; SuperOS + 80C49 emulator &middot; MIDI on DIN and USB-C</p></div>
<p class="intro">Every SuperOS function on one drawing each: the panel, the controls involved, the order to
press them, and the LEDs the SH-101 shows while you do it. The LED states are real: the firmware ran each
sequence in the host simulator. Find the function, look at the picture, follow the numbered steps.</p>
<div class="coverart"><img src="{art}" alt=""></div>
<div class="legend">
<div><strong>Reading the drawings</strong><i></i>Black ring: press this.<br>
<i class="h"></i>Red ring and red label: hold it while you do the rest.<br>
<b></b>Blue dashed box: any key in that group.</div>
<div><strong>LEDs</strong><i class="on"></i>lit &nbsp;<i class="bl"></i>blinking &nbsp;<i class="off"></i>off<br>
The box beside each drawing repeats the seven LEDs large. Keys are numbered 1 to 32 from the lowest F;
key 8 is the lowest C.</div>
<div><strong>Words</strong>A + B: hold A, then press B. Tap: press and release. Playing: the sequencer
runs. Clock: the LFO / CLK RATE, or MIDI clock with sync on. The header gives the mode and when it
works.</div>
</div>
</div>"""


def write_pdf(html, out):
    with open(OUT_HTML, "w", encoding="utf-8") as f:
        f.write(html)
    subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--no-pdf-header-footer",
                    "--print-to-pdf=" + out, "file://" + OUT_HTML], check=True, capture_output=True)


def measure(fragments):
    """Printed height in mm of each HTML fragment, measured by Chrome at page width."""
    items = "".join(f'<div class="m">{frag}</div>' for frag in fragments)
    html = ('<!doctype html><html><head><meta charset="utf-8">'
            f'<style>{CSS} body{{width:190mm}} .m{{display:flow-root}}</style></head><body>{items}'
            '<script>const px=96/25.4;document.title=JSON.stringify([...document.querySelectorAll(".m")]'
            '.map(e=>{const c=e.firstElementChild,st=getComputedStyle(c);'
            'return (c.getBoundingClientRect().height+parseFloat(st.marginTop)+parseFloat(st.marginBottom))/px}))'
            '</script></body></html>')
    tmp = os.path.join(HERE, "_measure.html")
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(html)
    dom = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--dump-dom", "file://" + tmp],
                         check=True, capture_output=True, text=True).stdout
    os.remove(tmp)
    return json.loads(re.search(r"<title>(.*?)</title>", dom, re.S).group(1))


def main():
    capture()
    LIMIT = 267.0
    seq, n = [], 0
    for name, fns in CHAPTERS:
        seq.append(("part", name, f'<div class="part">{esc(name)}</div>'))
        for f in fns:
            n += 1
            seq.append(("fn", f, block(n, f)))
    heights = measure([frag for _, _, frag in seq])
    pages, cur, used = [], [], 0.0
    for i, (kind, obj, frag) in enumerate(seq):
        h = heights[i] + (heights[i + 1] if kind == "part" and i + 1 < len(seq) else 0)
        if cur and used + h > LIMIT:
            pages.append(cur)
            cur, used = [], 0.0
        cur.append(frag)
        used += heights[i]
        if kind == "fn":
            obj["_page"] = len(pages) + 3
    pages.append(cur)
    pages.append([ref()])

    toc = ['<div class="part">Contents</div><div class="toccols">']
    n = 0
    for name, fns in CHAPTERS:
        toc.append(f'<div class="tocpart"><span>{esc(name)}</span></div>')
        for f in fns:
            n += 1
            toc.append(f'<div class="tocrow"><span class="tocnum">{n}</span><span class="toctitle">'
                       f'{esc(f["title"])}</span><span class="tocleader"></span>'
                       f'<span class="tocpage">{f["_page"]}</span></div>')
    toc.append('<div class="tocpart"><span>Reference</span></div>'
               '<div class="tocrow"><span class="tocnum">&nbsp;</span><span class="toctitle">LEDs, every gesture, '
               f'MIDI</span><span class="tocleader"></span><span class="tocpage">{len(pages) + 2}'
               '</span></div></div>')

    def foot(pg):
        return f'<div class="pfoot"><span>SuperOS-101 &middot; Shortcut Guide</span><span>{pg}</span></div>'

    body = [f'<div class="pg">{cover()}{foot(1)}</div>', f'<div class="pg">{"".join(toc)}{foot(2)}</div>']
    body += [f'<div class="pg">{"".join(pg)}{foot(i + 3)}</div>' for i, pg in enumerate(pages)]
    head = ('<!doctype html><html lang="en"><head><meta charset="utf-8"><title>SuperOS-101 Shortcut Guide</title>'
            f'<style>{CSS}</style></head><body>')
    write_pdf(head + "".join(body) + "</body></html>", OUT_PDF)
    print(OUT_PDF, "functions:", n, "pages:", len(pages) + 2)


if __name__ == "__main__":
    main()
