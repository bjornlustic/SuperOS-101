#!/usr/bin/env python3
"""Headless check of the editor's Performance section against a fake device.

A fake navigator.requestMIDIAccess stands in for the unit: it answers 0x20
with a 0x21 config and 0x65/0x67 with a 0x66 built by the same rules as
src/main.cpp OnSysex (out-of-range values ignored, tune only in keyboard mode,
real-time record only while playing, tmode off clears the transpose). No real
MIDI port is touched.

    python3 tools/web-editor/test_performance.py
"""
import pathlib, sys
from playwright.sync_api import sync_playwright

PAGE = (pathlib.Path(__file__).parent / "index.html").resolve().as_uri()

FAKE = r"""
(() => {
  const st = {fw: 0, dir: 0, pedal: 0, tr: 0, tmode: 0, loop: 0, rt: 0, tune: 0, queued: 0x7F,
              delay: 0, arp: 1, playing: false, keyboard: true, featLen: 10};
  const sent = []; window.__sent = sent; window.__dev = st;
  const input = {id: 'in1', name: 'SuperOS-101', state: 'connected', onmidimessage: null};
  const reply = body => setTimeout(() => input.onmidimessage &&
    input.onmidimessage({data: new Uint8Array([0xF0, 0x7D, ...body, 0xF7])}), 5);
  window.__reply = reply;
  const feats = () => [0x66, st.dir, st.pedal, st.tr + 64, st.tmode, st.loop, st.rt, st.tune, st.queued, st.delay, st.arp].slice(0, 1 + st.featLen);
  const output = {id: 'out1', name: 'SuperOS-101', state: 'connected', send(d) {
    const m = Array.from(d); sent.push(m);
    if (m[0] !== 0xF0) return;
    const c = m[2], id = m[3], v = m[4];
    if (c === 0x20) reply([0x21, st.fw, 0, 0, 6, 0, 1, 1, 0, ...'1.0'.split('').map(ch => ch.charCodeAt(0))]);
    if (st.fw !== 0) return;   // the emulator never answers 0x67
    if (c === 0x65) {
      if (id === 0 && v < 4) st.dir = v;
      if (id === 1 && v <= 1) st.pedal = v;
      if (id === 2 && v >= 28 && v <= 100) st.tr = v - 64;
      if (id === 3) { st.tr = v ? st.tr : 0; st.tmode = v ? 1 : 0; }
      if (id === 4 && v <= 128) st.loop = v;
      if (id === 5 && st.playing) st.rt = v ? 1 : 0;
      if (id === 6) st.tune = v && st.keyboard ? 1 : 0;
      if (id === 7 && v <= 60) st.delay = v;
      if (id === 8 && v >= 1 && v <= 3) st.arp = v;
      reply(feats());
    }
    if (c === 0x67) reply(feats());
  }};
  const access = {inputs: new Map([['in1', input]]), outputs: new Map([['out1', output]]), onstatechange: null};
  navigator.requestMIDIAccess = async () => access;
})();
"""

fails = []
def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond: fails.append(what)

def last65(page):
    m = [s for s in page.evaluate("window.__sent") if s[:3] == [0xF0, 0x7D, 0x65]]
    return m[-1][3:5] if m else None

def count67(page):
    return sum(1 for s in page.evaluate("window.__sent") if s == [0xF0, 0x7D, 0x67, 0xF7])

with sync_playwright() as p:
    b = p.chromium.launch(channel="chrome")
    page = b.new_context().new_page()
    page.add_init_script(FAKE)
    errs = []
    page.on("pageerror", lambda e: errs.append(str(e)))
    page.goto(PAGE)
    page.click("#btnMidi")
    page.wait_for_timeout(400)
    vis = lambda sel: page.is_visible(sel)
    check(count67(page) >= 1, "0x67 sent on connect")
    check(vis("#perfPanel"), "performance panel visible in SuperOS mode")
    check(page.inner_text("#perfStatus").startswith("dir FWD"), "status shows the 0x66 reply")

    def act(fn, want, label, settle=150):
        fn(); page.wait_for_timeout(settle)
        check(last65(page) == want, f"{label}: sent 65 {want} (got {last65(page)})")

    act(lambda: page.click("#dirGrid .div-btn[data-idx='2']"), [0, 2], "direction ping-pong")
    check("is-on" in page.get_attribute("#dirGrid .div-btn[data-idx='2']", "class"), "direction renders P-P")
    act(lambda: page.click("#arpGrid .div-btn[data-idx='3']"), [8, 3], "arp range 3")
    check("is-on" in page.get_attribute("#arpGrid .div-btn[data-idx='3']", "class"), "arp renders 3")
    act(lambda: page.click("#btnTrUp"), [2, 65], "transpose +1")
    act(lambda: page.click("#btnTrUp"), [2, 66], "transpose +2")
    check(page.input_value("#inTr") == "2", "transpose renders +2")
    act(lambda: page.click("#btnTrDn"), [2, 65], "transpose -1")
    act(lambda: (page.fill("#inTr", "-12"), page.press("#inTr", "Tab")), [2, 52], "transpose typed -12")
    act(lambda: (page.fill("#inTr", "99"), page.press("#inTr", "Tab")), [2, 100], "transpose clamps to +36")
    act(lambda: page.click("#btnTrReset"), [2, 64], "transpose reset")
    check(page.input_value("#inTr") == "0", "transpose renders 0")
    act(lambda: page.click("#btnTrMode"), [3, 1], "transpose mode on")
    check(page.text_content("#btnTrMode") == "On", "transpose mode renders On")
    act(lambda: page.select_option("#selLoop", "8"), [4, 8], "loop 8")
    check(page.input_value("#selLoop") == "8", "loop renders 8")
    act(lambda: page.select_option("#selLoop", "0"), [4, 0], "loop full")
    act(lambda: (page.fill("#inDelay", "25"), page.press("#inDelay", "Tab")), [7, 25], "sync delay 25 typed")
    check(page.input_value("#rngDelay") == "25", "slider follows reply")
    page.eval_on_selector("#rngDelay", "e => { e.value = 40; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }")
    page.wait_for_timeout(150)
    check(last65(page) == [7, 40], "sync delay slider sends 40")
    act(lambda: page.select_option("#selPedal", "1"), [1, 1], "pedal key transpose")
    act(lambda: page.click("#btnTune"), [6, 1], "tune on")
    check(page.text_content("#btnTune") == "On", "tune renders On")
    act(lambda: page.click("#btnTune"), [6, 0], "tune off")
    act(lambda: page.click("#btnRtRec"), [5, 1], "rt record on (stopped)")
    check(page.text_content("#btnRtRec") == "Off", "rt record refused while stopped: UI shows device truth")
    page.evaluate("window.__dev.playing = true; window.__dev.keyboard = false")
    act(lambda: page.click("#btnRtRec"), [5, 1], "rt record on (playing)")
    check(page.text_content("#btnRtRec") == "On", "rt record renders On")
    act(lambda: page.click("#btnTune"), [6, 1], "tune on outside keyboard mode")
    check(page.text_content("#btnTune") == "Off", "tune refused: UI shows Off")
    check(page.inner_text("#queued") == "none", "queued none")

    # Refresh rate: a flood of 0x15 step messages plus the keepalive gives at most ~1 read per second.
    page.evaluate("window.__dev.queued = 4")
    n0 = count67(page)
    for i in range(30):
        page.evaluate(f"window.__reply([0x15, 0, {i % 16}])")
        page.wait_for_timeout(100)
    n = count67(page) - n0
    check(1 <= n <= 4, f"0x67 refresh throttled under 0x15 traffic ({n} in ~3 s)")
    check(page.inner_text("#queued").startswith("Pattern 5"), "queued pattern indicator updates")

    # The real unit on 2026-09-28 (firmware before the arp-range commit) sent
    # exactly this 9-field reply: the arp range control must hide, the rest stay.
    page.evaluate("window.__reply([0x66, 0, 0, 64, 0, 0, 0, 0, 127, 0])")
    page.wait_for_timeout(100)
    check(not vis("[data-f='9']") and vis("[data-f='8']") and vis("[data-f='7']"), "9-field reply: arp range hidden, sync delay shown")

    # Older firmware: a short 0x66 hides the controls whose bytes are missing.
    page.evaluate("window.__reply([0x66, 1, 0, 64])")
    page.wait_for_timeout(100)
    check(vis("[data-f='0']") and vis("[data-f='2']"), "short reply: present fields shown")
    check(not vis("[data-f='9']") and not vis("[data-f='8']") and not vis("[data-f='3']"), "short reply: missing fields hidden")

    # Emulator mode: the section goes away.
    page.evaluate("window.__dev.fw = 1")
    page.wait_for_timeout(2300)   # next keepalive 0x20 -> 0x21 with fw byte 1
    check(not vis("#perfPanel"), "performance panel hidden in 80C49 mode")

    # Menu map lists the new config-menu keys.
    mm = page.inner_text("#mmBody")
    check("Keys 15" in mm and "Key 19" in mm and "Keys 20" in mm, "menu legend lists keys 15-31")
    check(page.locator("#mmKbd .mm-key").count() == 31, "menu keyboard shows 31 keys")
    check(not errs, "no page errors " + "; ".join(errs))
    b.close()

print(f"\n{'FAILED ' + str(len(fails)) if fails else 'all passed'}")
sys.exit(1 if fails else 0)
