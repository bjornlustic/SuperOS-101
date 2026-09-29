#!/usr/bin/env python3
"""Headless check of the editor's Performance section against a fake device.

A fake navigator.requestMIDIAccess stands in for the unit: it answers 0x20
with a 0x21 config and 0x65/0x67 with a 0x66 built by the same rules as
src/main.cpp OnSysex (out-of-range values ignored, tune only in keyboard mode,
real-time record only while playing), holds 16 patterns and their scales
(0x10/0x13 -> 0x11, 0x2A, 0x68 -> 0x14 + 0x11). No real
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
  const NP = 16;   // patterns on the unit (firmware e770f60)
  st.pats = Array.from({length: NP}, (_, p) => p === 0 ? Array.from({length: 16}, (_, i) => [12, 13, 14, 15][i % 4] | (i === 5 ? 1 << 6 : 0) | (i === 9 ? 2 << 6 : 0)) : []);
  st.scales = Array.from({length: NP}, () => ({mask: 0xFFF, on: 0}));
  st.rnd = [];
  const pack7 = src => { const out = []; for (let i = 0; i < src.length; i += 7) { const ch = src.slice(i, i + 7); let msb = 0;
    ch.forEach((b, k) => { if (b & 0x80) msb |= 1 << k; }); out.push(msb, ...ch.map(b => b & 0x7F)); } return out; };
  const sendPat = p => {
    const steps = st.pats[p], raw = [steps.length, ...steps, ...new Array(Math.ceil(steps.length / 4)).fill(0)];
    let x = 0; raw.forEach(v => x ^= v);
    reply([0x11, p, x & 0x7F, (x >> 7) & 1, ...pack7(raw)]);
  };
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
      if (id === 3) st.tmode = v ? 1 : 0;   // leaving the mode keeps the transpose
      if (id === 4 && v <= 128) st.loop = v;
      if (id === 5 && st.playing) st.rt = v ? 1 : 0;
      if (id === 6) st.tune = v && st.keyboard ? 1 : 0;
      if (id === 7 && v <= 60) st.delay = v;
      if (id === 8 && v >= 1 && v <= 3) st.arp = v;
      if (id === 9) st.resyncs = (st.resyncs || 0) + 1;
      reply(feats());
    }
    if (c === 0x67) reply(feats());
    if (c === 0x13) for (let p = 0; p < NP; ++p) sendPat(p);
    if (c === 0x10 && id < NP) sendPat(id);
    if (c === 0x2A && id < NP) {
      if (m.length >= 8) st.scales[id] = {mask: m[4] | (m[5] & 0x1F) << 7, on: m[6] ? 1 : 0};
      const sc = st.scales[id]; reply([0x2A, id, sc.mask & 0x7F, sc.mask >> 7, sc.on]);
    }
    if (c === 0x68) {
      const [p, op, dens, lo, hi] = m.slice(3, 8);
      st.rnd.push({p, op, dens, seed: lo | hi << 7});
      if (p >= NP || op > 5) { reply([0x14, 2]); return; }
      st.pats[p] = Array.from({length: 16}, (_, i) => (i % 3 === 2 ? 1 << 6 : 0) | ((op * 5 + i * 7) % 48));
      reply([0x14, 0]); sendPat(p);
    }
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

    # ---- step grid: no add tile, no per-step insert/delete, no Enter/Del hints
    check(page.locator(".add-card").count() == 0, "no add-step tile")
    check(page.locator("[data-a='insa'], [data-a='del']").count() == 0, "no per-step + / x buttons")
    hint = page.inner_text(".seq-toolbar")
    check("Enter" not in hint and "inserts" not in hint and "deletes" not in hint, "no insert/delete help or key hints")
    n = page.locator("#steps .step-card").count()
    page.click("#steps .step-card[data-i='2'] .step-num"); page.keyboard.press("Enter"); page.keyboard.press("Delete")
    page.wait_for_timeout(200)
    check(page.locator("#steps .step-card").count() == n == 16, "Enter / Delete no longer change the length")

    def sent_since(k, keep_polls=False):   # drops the background keepalive / feature polls unless asked
        return [m[2:-1] for m in page.evaluate("window.__sent")[k:]
                if m[:2] == [0xF0, 0x7D] and (keep_polls or m[2:-1] not in ([0x20], [0x40], [0x67]))]
    mark = lambda: len(page.evaluate("window.__sent"))

    # ---- Resync = back on the one, nothing else
    k = mark(); page.click("#btnResync"); page.wait_for_timeout(150)
    check(sent_since(k) == [[0x65, 9, 0]], f"Resync sends only 65 09 00 (got {sent_since(k)})")
    check(page.evaluate("window.__dev.resyncs") == 1, "fake unit saw one resync")
    # ---- Pull all = the old full re-read
    k = mark(); page.click("#btnPull"); page.wait_for_timeout(300)
    cmds = [m[0] for m in sent_since(k, True)]
    check(all(c in cmds for c in (0x20, 0x23, 0x67, 0x13)) and cmds.count(0x2A) == 16, f"Pull all re-reads config, perf, patterns, 16 scales ({cmds})")

    # ---- Scale
    check(page.text_content("#btnScaleOn") == "Off" and page.input_value("#scaleSelect") == "", "scale starts off, all notes (custom)")
    k = mark(); page.click("#scaleRootBtns [data-pc='2']"); page.wait_for_timeout(200)
    dmaj = ((0xAB5 << 2) | (0xAB5 >> 10)) & 0xFFF
    check(sent_since(k) == [[0x2A, 0, dmaj & 0x7F, dmaj >> 7, 1]], f"root D sends D Major on (got {sent_since(k)})")
    check(page.input_value("#scaleSelect") == "0" and page.text_content("#btnScaleOn") == "On", "renders D Major, On (not C# Locrian)")
    check("active" in page.get_attribute("#scaleRootBtns [data-pc='2']", "class"), "root D highlighted")
    offs = page.locator("#steps .step-card.is-offscale").count()
    check(offs == 8, f"C3 and D#3 steps marked out of scale ({offs})")
    check("B2" in page.inner_text("#steps .step-card[data-i='0'] .step-name"), "C3 shows it plays B2 (lower on a tie)")
    k = mark(); page.select_option("#scaleSelect", "11"); page.wait_for_timeout(200)
    mpent = ((0x4A9 << 2) | (0x4A9 >> 10)) & 0xFFF
    check(sent_since(k) == [[0x2A, 0, mpent & 0x7F, mpent >> 7, 1]], "scale menu sends D Minor Pentatonic")
    k = mark(); page.click("#scaleNoteBtns [data-pc='1']"); page.wait_for_timeout(200)
    check(sent_since(k) == [[0x2A, 0, (mpent | 2) & 0x7F, (mpent | 2) >> 7, 1]], f"note toggle C# adds bit 1 ({sent_since(k)} vs {mpent|2:#x})")
    check(page.input_value("#scaleSelect") == "", "a toggled mask shows Custom")
    k = mark(); page.click("#btnScaleOn"); page.wait_for_timeout(200)
    check(sent_since(k) == [[0x2A, 0, (mpent | 2) & 0x7F, (mpent | 2) >> 7, 0]], "Off keeps the mask, on=0")
    check(page.locator("#steps .step-card.is-offscale").count() == 0, "scale off: no out-of-scale marks")
    amin = ((0x9AD << 9) | (0x9AD >> 3)) & 0xFFF   # panel: A Harmonic Minor, pushed unsolicited
    page.evaluate(f"window.__reply([0x2A, 0, {amin & 0x7F}, {amin >> 7}, 1])"); page.wait_for_timeout(150)
    check(page.input_value("#scaleSelect") == "13" and "active" in page.get_attribute("#scaleRootBtns [data-pc='9']", "class"), "unsolicited panel scale renders A Harmonic Minor")
    k = mark(); page.click(".pat-pill:nth-child(3)"); page.wait_for_timeout(200)
    check([0x2A, 2] in sent_since(k), "switching pattern reads its scale")
    check(page.text_content("#btnScaleOn") == "Off", "pattern 3 shows its own scale")
    check(page.locator(".pat-pill").count() == 16, "16 pattern tabs")
    k = mark(); page.click(".pat-pill:nth-child(12)"); page.wait_for_timeout(200)
    check([0x1D, 11] in sent_since(k) and [0x2A, 11] in sent_since(k), "tab 12 selects pattern 12 and reads its scale")
    check(page.inner_text("#seqTitle").lower() == "pattern 12", "tab 12 shows pattern 12")
    page.click(".pat-pill:nth-child(1)"); page.wait_for_timeout(200)

    # ---- Randomize
    k = mark(); page.click("#randBtns .btn:has-text('Mutate')"); page.wait_for_timeout(250)
    check(sent_since(k) == [[0x68, 0, 3, 0x7F, 0, 0]], f"Mutate, panel weights, seed 0 (got {sent_since(k)})")
    names = page.locator("#steps .step-name").all_inner_texts()
    check(len(names) == 16 and names[2] == "\u2014", "grid replaced by the unit's 0x11 reply")
    page.select_option("#selDensity", label="50% notes"); page.fill("#inSeed", "300")
    k = mark(); page.click("#randBtns .btn:has-text('Full')"); page.wait_for_timeout(250)
    check(sent_since(k) == [[0x68, 0, 0, 63, 300 & 0x7F, 300 >> 7]], f"Full, 50%, seed 300 (got {sent_since(k)})")
    # unsent local edits do not block the randomize reply
    page.click("#btnLive")
    page.click("#steps .step-card[data-i='0'] [data-a='oup']"); page.wait_for_timeout(150)
    before = page.inner_text("#steps .step-card[data-i='0'] .step-name")
    page.click("#randBtns .btn:has-text('Pitches')"); page.wait_for_timeout(250)
    after = page.inner_text("#steps .step-card[data-i='0'] .step-name")
    check(before != after and "is-dirty" not in page.get_attribute(".pat-pill:nth-child(1)", "class"), "randomize reply replaces unsent edits")
    page.click("#btnUndo"); page.wait_for_timeout(150)
    check(page.inner_text("#steps .step-card[data-i='0'] .step-name") == before, "Undo restores the pre-randomize notes")
    page.click("#btnLive")
    page.evaluate("window.__reply([0x14, 2])"); page.wait_for_timeout(100)
    check(page.is_visible("#errorToast"), "a rejected randomize toasts")

    # Replies captured from the real unit on 2026-09-28 (read-only session): its
    # 0x11 for pattern 1 (8 steps + ratchet tail), 0x2A (all notes, off), 10-field 0x66.
    page.evaluate("window.__reply([0x11, 0, 39, 0, 68, 8, 18, 37, 27, 22, 20, 30, 0, 17, 15, 0, 1])"); page.wait_for_timeout(100)
    page.evaluate("window.__reply([0x2A, 0, 127, 31, 0]); window.__reply([0x66, 0, 0, 64, 0, 0, 0, 0, 127, 0, 1])"); page.wait_for_timeout(150)
    check(page.locator("#steps .step-card").count() == 8 and page.text_content("#btnScaleOn") == "Off", "real-unit capture: 8-step pattern, scale off")
    check(page.is_visible("[data-f='9']"), "real-unit capture: 10-field 0x66 shows arp range")

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
