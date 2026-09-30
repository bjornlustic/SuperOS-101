#!/usr/bin/env python3
"""Headless check of the editor's Performance section against a fake device.

A fake navigator.requestMIDIAccess stands in for the unit: it answers 0x20
with a 0x21 config and 0x65/0x67 with a 0x66 built by the same rules as
src/main.cpp OnSysex (out-of-range values ignored, tune only in keyboard mode,
real-time record only while playing), holds 16 patterns and their scales
(0x10/0x13 -> 0x11, 0x12 -> 0x14, 0x2A, 0x68 and 0x69 -> 0x14 + 0x11). No real
MIDI port is touched.

    python3 tools/web-editor/test_performance.py
"""
import pathlib, sys
from playwright.sync_api import sync_playwright

PAGE = (pathlib.Path(__file__).parent / "index.html").resolve().as_uri()

FAKE = r"""
(() => {
  const st = {fw: 0, dir: 0, pedal: 0, tr: 0, tmode: 0, loop: 0, rt: 0, tune: 0, queued: 0x7F,
              delay: 0, arp: 1, clk: 0, thru: 0, nout: 0, playing: false, keyboard: true, featLen: 13};
  const sent = []; window.__sent = sent; window.__dev = st;
  const input = {id: 'in1', name: 'SuperOS-101', state: 'connected', onmidimessage: null};
  const reply = body => setTimeout(() => input.onmidimessage &&
    input.onmidimessage({data: new Uint8Array([0xF0, 0x7D, ...body, 0xF7])}), 5);
  window.__reply = reply;
  const feats = () => [0x66, st.dir, st.pedal, st.tr + 64, st.tmode, st.loop, st.rt, st.tune, st.queued, st.delay, st.arp, st.clk, st.thru, st.nout].slice(0, 1 + st.featLen);
  const NP = 16;   // patterns on the unit (firmware e770f60)
  st.pats = Array.from({length: NP}, (_, p) => p === 0 ? Array.from({length: 16}, (_, i) => [12, 13, 14, 15][i % 4] | (i === 5 ? 1 << 6 : 0) | (i === 9 ? 2 << 6 : 0)) : []);
  st.rats = Array.from({length: NP}, () => []);   // ratchet per step, 0..2
  st.accs = Array.from({length: NP}, () => []);   // accent per step, 0/1
  st.flts = Array.from({length: NP}, () => []);   // filter CV per step, 0..127
  // CV DAC config as src/cvdac.h Pack(): center, cc, flags, mode, vcf, vca, decay, velThr, res, build, hw, park
  st.cv = {center: 2300, cc: 74, flags: 7, mode: 0, vcf: 64, vca: 64, decay: 30, thr: 100, res: 64, build: 64, hw: 1, park: 700};
  const cvReply = () => { const c = st.cv; reply([0x24, c.center & 0x7F, c.center >> 7, c.cc, c.flags, c.mode, c.vcf, c.vca, c.decay, c.thr, c.res, c.build, c.hw,
    c.center & 0x7F, c.center >> 7, c.park & 0x7F, c.park >> 7, c.park & 0x7F, c.park >> 7, c.sus || 0]); };
  st.scales = Array.from({length: NP}, () => ({mask: 0xFFF, on: 0}));
  st.rnd = [];
  const pack7 = src => { const out = []; for (let i = 0; i < src.length; i += 7) { const ch = src.slice(i, i + 7); let msb = 0;
    ch.forEach((b, k) => { if (b & 0x80) msb |= 1 << k; }); out.push(msb, ...ch.map(b => b & 0x7F)); } return out; };
  const sendPat = p => {
    const steps = st.pats[p], tail = new Array(Math.ceil(steps.length / 4)).fill(0), acc = new Array(Math.ceil(steps.length / 8)).fill(0);
    steps.forEach((_, i) => tail[i >> 2] |= (st.rats[p][i] || 0) << ((i & 3) * 2));
    steps.forEach((_, i) => acc[i >> 3] |= (st.accs[p][i] || 0) << (i & 7));
    const raw = [steps.length, ...steps, ...tail, ...acc];
    let x = 0; raw.forEach(v => x ^= v);
    reply([0x11, p, x & 0x7F, (x >> 7) & 1, ...pack7(raw)]);
    reply([0x2C, p, ...steps.map((_, i) => st.flts[p][i] || 0)]);   // the filter lane follows every 0x11
  };
  window.__sendPat = sendPat;
  const unpack7 = src => { const out = []; let i = 0;
    while (i < src.length) { const msb = src[i++]; for (let k = 0; k < 7 && i < src.length; ++k) out.push(src[i++] | ((msb >> k) & 1 ? 0x80 : 0)); } return out; };
  const output = {id: 'out1', name: 'SuperOS-101', state: 'connected', send(d) {
    const m = Array.from(d); sent.push(m);
    if (m[0] !== 0xF0) return;
    const c = m[2], id = m[3], v = m[4];
    if (c === 0x1D && id < NP) st.cur = id;   // the unit follows the selection, as src/main.cpp does
    if (c === 0x20) reply([0x21, st.fw, 0, 0, 6, st.cur || 0, 1, 1, 0, ...'1.0'.split('').map(ch => ch.charCodeAt(0))]);
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
      if (id === 11 && v <= 1) st.thru = v;
      if (id === 12 && v <= 3) st.nout = v;
      if (id === 9) st.resyncs = (st.resyncs || 0) + 1;
      reply(feats());
    }
    if (c === 0x67 && !st.quiet) reply(feats());   // quiet: the injected replies below stay the truth
    if (c === 0x13) for (let p = 0; p < NP; ++p) sendPat(p);
    if (c === 0x10 && id < NP) sendPat(id);
    if (c === 0x12 && id < NP) {   // pattern write: steps + packed ratchet tail
      const raw = unpack7(m.slice(6, -1)), len = raw[0];
      st.pats[id] = raw.slice(1, 1 + len);
      st.rats[id] = st.pats[id].map((_, i) => (raw[1 + len + (i >> 2)] >> ((i & 3) * 2)) & 3);
      st.accs[id] = st.pats[id].map((_, i) => ((raw[1 + len + Math.ceil(len / 4) + (i >> 3)] || 0) >> (i & 7)) & 1);
      reply([0x14, 0]);
    }
    if (c === 0x16 && m[4] === 1) {   // accent lane: refused on a rest (ack 2)
      const [p, , i, val] = m.slice(3, 7);
      if (p >= NP || i >= st.pats[p].length || (st.pats[p][i] >> 6) === 1 || val > 1) { reply([0x14, 2]); return; }
      st.accs[p][i] = val; reply([0x14, 0]);
    }
    if (c === 0x16 && m[4] === 4) {   // filter lane: any step inside the pattern, rests too
      const [p, , i, val] = m.slice(3, 7);
      if (p >= NP || i >= st.pats[p].length || val > 127) { reply([0x14, 2]); return; }
      st.flts[p][i] = val; reply([0x14, 0]);
    }
    if (c === 0x2B && id < NP) reply([0x2C, id, ...st.pats[id].map((_, i) => st.flts[id][i] || 0)]);
    if (c === 0x2D && id < NP) { st.flts[id] = m.slice(4, -1); reply([0x14, 0]); }
    if (c === 0x23) cvReply();
    if (c === 0x25) {
      const b = m.slice(3, -1);
      Object.assign(st.cv, {center: b[0] | b[1] << 7, cc: b[2], flags: b[3], mode: b[4], vcf: b[5], vca: Math.min(100, b[6]), decay: b[7],
                            thr: b[8], res: b[9], build: b[10], hw: b[11]});
      if (b.length >= 14) st.cv.park = b[12] | b[13] << 7;
      if (b.length >= 15) st.cv.sus = b[14];
      st.cv25 = b; cvReply();
    }
    if (c === 0x69) {   // nudge: rotate the notes, ratchets move with their steps
      const [p, dir, n] = m.slice(3, 6);
      if (p >= NP || !n) { reply([0x14, 2]); return; }
      const len = st.pats[p].length, k = len ? (dir ? n : -n) % len : 0;
      const rot = a => a.map((_, i) => a[((i - k) % len + len) % len]);
      if (len) { st.pats[p] = rot(st.pats[p]); st.rats[p] = rot(st.pats[p].map((_, i) => st.rats[p][i] || 0)); }
      reply([0x14, 0]); if (len) setTimeout(() => sendPat(p), 20);   // playing: the 0x11 can trail the ack
    }
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

    check(vis("[data-f='11']") and page.input_value("#selThru") == "0", "MIDI OUT/THRU shown, OUT")
    act(lambda: page.select_option("#selThru", "1"), [11, 1], "MIDI THRU")
    check(page.input_value("#selThru") == "1" and "MIDI THRU" in page.inner_text("#perfStatus"), "THRU renders from the 0x66 reply")
    act(lambda: page.select_option("#selThru", "0"), [11, 0], "MIDI OUT")
    act(lambda: page.select_option("#selNoteOut", "1"), [12, 1], "notes out DIN only")
    check(page.input_value("#selNoteOut") == "1" and "notes DIN only" in page.inner_text("#perfStatus"), "notes out renders from 0x66")
    act(lambda: page.select_option("#selNoteOut", "0"), [12, 0], "notes out both")
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
    check(page.inner_text("#seqTitle").lower() == "pattern 12", "tab 12 shows pattern 12 (" + page.inner_text("#seqTitle") + ")")
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

    # ---- Nudge on the unit: 0x69 -> ack + 0x11; ratchets travel with their steps
    orig = [(10 + i) | (2 << 6 if i == 2 else 0) | (1 << 6 if i == 5 else 0) for i in range(8)]
    orats = [1, 0, 2, 0, 0, 0, 1, 0]
    page.evaluate(f"window.__dev.pats[0] = {orig}; window.__dev.rats[0] = {orats}; window.__sendPat(0)"); page.wait_for_timeout(150)
    local = lambda: page.evaluate("patterns[0].map(s => [(s.cv & 0x3F) | (s.type << 6), s.rat])")
    as_pairs = lambda st, rt: [[a, b] for a, b in zip(st, rt)]
    right = lambda a: a[-1:] + a[:-1]
    left = lambda a: a[1:] + a[:1]
    check(local() == as_pairs(orig, orats), "nudge setup: grid holds the unit's 8 steps")
    k = mark(); page.click("#btnNudgeR"); page.wait_for_timeout(250)
    check(sent_since(k) == [[0x69, 0, 1, 1]], f"Nudge right sends 69 00 01 01 (got {sent_since(k)})")
    check(local() == as_pairs(right(orig), right(orats)), f"grid follows the 0x11 reply, step 1 = old step 8 ({local()})")
    check(page.inner_text("#steps .step-card[data-i='3'] .step-name") != "" and "is-tie" in page.get_attribute("#steps .step-card[data-i='3']", "class"),
          "the legato step moved from 3 to 4 on screen")
    check("is-dirty" not in page.get_attribute(".pat-pill:nth-child(1)", "class"), "nudged pattern is not dirty (the unit holds it)")
    raw = [8] + orig + [orats[0] | orats[1] << 2 | orats[2] << 4 | orats[3] << 6, orats[4] | orats[5] << 2 | orats[6] << 4 | orats[7] << 6, 0]
    x = 0
    for v in raw: x ^= v
    pk = []
    for i in range(0, len(raw), 7):
        ch = raw[i:i + 7]; pk += [sum(1 << j for j, v in enumerate(ch) if v & 0x80)] + [v & 0x7F for v in ch]
    k = mark(); page.click("#btnUndo"); page.wait_for_timeout(300)
    check(local() == as_pairs(orig, orats), "Undo restores the pre-nudge pattern")
    check([0x12, 0, x & 0x7F, (x >> 7) & 1] + pk in sent_since(k), f"Undo writes the old pattern with 0x12 ({sent_since(k)})")
    check(page.evaluate("window.__dev.pats[0]") == orig and page.evaluate("window.__dev.rats[0]") == orats, "the fake unit holds the old pattern again")

    # Shift+Left / Shift+Right nudge; plain arrows still select
    [page.keyboard.press("ArrowLeft") for _ in range(8)]; [page.keyboard.press("ArrowRight") for _ in range(3)]
    check(page.evaluate("selStep") == 3, "arrows select step 4")
    k = mark(); page.keyboard.press("Shift+ArrowLeft"); page.wait_for_timeout(250)
    check(sent_since(k) == [[0x69, 0, 0, 1]], f"Shift+Left sends 69 00 00 01 (got {sent_since(k)})")
    check(local() == as_pairs(left(orig), left(orats)), "Shift+Left: step 1 = old step 2")
    check(page.evaluate("selStep") == 3, "Shift+Left does not move the selection")
    k = mark(); page.keyboard.press("Shift+ArrowRight"); page.wait_for_timeout(250)
    check(sent_since(k) == [[0x69, 0, 1, 1]] and local() == as_pairs(orig, orats), "Shift+Right nudges back")
    page.keyboard.press("ArrowRight"); page.keyboard.press("ArrowRight"); page.keyboard.press("ArrowLeft")
    check(page.evaluate("selStep") == 4 and "is-sel" in page.get_attribute("#steps .step-card[data-i='4']", "class"), "plain arrows still select steps")
    page.keyboard.press("Shift+ArrowUp"); page.wait_for_timeout(200)
    check(local()[4][0] == orig[4] + 12, "Shift+Up still moves the selected step an octave")
    page.click("#btnUndo"); page.wait_for_timeout(250)

    # Edit live off: nudge is a local edit (dirty, nothing sent) and Write pushes it
    page.click("#btnLive")
    k = mark(); page.click("#btnNudgeL"); page.wait_for_timeout(200)
    check(sent_since(k) == [] and local() == as_pairs(left(orig), left(orats)), "live off: rotated locally, nothing sent")
    check("is-dirty" in page.get_attribute(".pat-pill:nth-child(1)", "class"), "live off: pattern marked dirty")
    k = mark(); page.click("#btnWrite"); page.wait_for_timeout(200)
    check(any(m[0] == 0x12 for m in sent_since(k)) and page.evaluate("window.__dev.pats[0]") == left(orig)
          and page.evaluate("window.__dev.rats[0]") == left(orats), "Write pushes the nudged pattern (ratchets included)")
    page.click("#btnLive")
    check("HOLD + UP / DOWN" in page.inner_text("#perfBody") and "nudges right / left" in page.inner_text("#perfBody"), "panel gestures mention HOLD + UP / DOWN nudge")
    check(page.get_attribute("#btnNudgeR", "title").startswith("Moves the notes, not the timing; on the unit it lands between notes"), "nudge tooltip")

    # ---- Per-step accent: ACC button / A key -> 0x16 lane 1; the 0x11 accent tail renders
    page.evaluate("window.__dev.pats[0] = [12, 13, 1 << 6, 15]; window.__dev.rats[0] = [0, 0, 0, 0]; window.__dev.accs[0] = [0, 1, 0, 0]; window.__sendPat(0)")
    page.wait_for_timeout(150)
    card = lambda i: page.get_attribute(f"#steps .step-card[data-i='{i}']", "class")
    check("is-acc" in card(1) and "is-acc" not in card(0), "0x11 accent tail: step 2 shows accented")
    k = mark(); page.click("#steps .step-card[data-i='0'] [data-a='acc']"); page.wait_for_timeout(250)
    check([0x16, 0, 1, 0, 1] in sent_since(k) and page.evaluate("window.__dev.accs[0][0]") == 1, f"ACC on step 1 sends 16 00 01 00 01 ({sent_since(k)})")
    check("is-acc" in card(0), "step 1 renders accented")
    k = mark(); page.keyboard.press("a"); page.wait_for_timeout(250)
    check([0x16, 0, 1, 0, 0] in sent_since(k) and page.evaluate("window.__dev.accs[0][0]") == 0, "A key toggles the accent off on the unit")
    check(not page.is_visible("#steps .step-card[data-i='2'] [data-a='acc']"), "a rest has no ACC button")
    page.click("#btnLive")
    page.click("#steps .step-card[data-i='3'] [data-a='acc']")
    k = mark(); page.click("#btnWrite"); page.wait_for_timeout(200)
    check(page.evaluate("window.__dev.accs[0]") == [0, 1, 0, 1], f"Write carries the accent tail on 0x12 ({page.evaluate('window.__dev.accs[0]')})")
    page.click("#btnLive")

    # ---- Per-step filter CV: FLT slider -> 0x16 lane 4; 0x2C after 0x11 renders; Write sends 0x2D
    page.evaluate("window.__dev.flts[0] = [0, 90, 0, 0]; window.__sendPat(0)"); page.wait_for_timeout(150)
    check(page.input_value("#steps .step-card[data-i='1'] .flt-rng") == "90" and page.inner_text("#steps .step-card[data-i='1'] .flt-val") == "+41",
          "0x2C after 0x11: step 2 shows filter 90 (+41)")
    k = mark()
    page.fill("#steps .step-card[data-i='0'] .flt-rng", "30"); page.dispatch_event("#steps .step-card[data-i='0'] .flt-rng", "change")
    page.wait_for_timeout(250)
    check([0x16, 0, 4, 0, 30] in sent_since(k) and page.evaluate("window.__dev.flts[0][0]") == 30, f"FLT on step 1 sends 16 00 04 00 1E ({sent_since(k)})")
    page.click("#steps .step-card[data-i='0'] .step-num"); page.keyboard.press("]"); page.wait_for_timeout(250)
    check(page.evaluate("window.__dev.flts[0][0]") == 38, "] raises the selected step's filter by 8")
    page.keyboard.press("\\"); page.wait_for_timeout(250)
    check(page.evaluate("window.__dev.flts[0][0]") == 0 and page.inner_text("#steps .step-card[data-i='0'] .flt-val") == "OFF", "backslash clears it (OFF)")
    page.click("#btnLive")
    page.fill("#steps .step-card[data-i='2'] .flt-rng", "100"); page.dispatch_event("#steps .step-card[data-i='2'] .flt-rng", "change")
    k = mark(); page.click("#btnWrite"); page.wait_for_timeout(250)
    check(page.evaluate("window.__dev.flts[0]") == [0, 90, 100, 0], f"Write sends the lane whole with 0x2D ({page.evaluate('window.__dev.flts[0]')})")
    page.click("#btnLive")

    # ---- Filter CV / accent panel (0x23 on connect -> 0x24; edits -> 0x25)
    check(vis("#cvPanel") and page.input_value("#inCenter") == "2300", "CV panel shows the unit's filter CV offset (2300)")
    check("offset 2300 mV" in page.inner_text("#cvStatus"), "CV status line")
    page.fill("#inCenter", "2250"); page.press("#inCenter", "Tab"); page.wait_for_timeout(150)
    b25 = page.evaluate("window.__dev.cv25")
    check(b25 and len(b25) == 15 and (b25[0] | b25[1] << 7) == 2250 and (b25[12] | b25[13] << 7) == 700,
          f"offset 2250 -> 0x25 with 15 bytes, park and sustain kept ({b25})")
    page.fill("#rngSus", "40"); page.dispatch_event("#rngSus", "change"); page.wait_for_timeout(150)
    check(page.evaluate("window.__dev.cv.sus") == 40 and page.inner_text("#susVal") == "31 %", "accent sustain 40 -> 0x25, shown as 31 %")
    check(page.inner_text("#vcaAmtVal") == "+100 %", "VCA accent 64 shows as +100 % of the envelope")
    check(page.input_value("#rngCenter") == "2250", "slider follows the 0x24 reply")
    page.click("#btnCal"); page.wait_for_timeout(150)
    check(page.evaluate("window.__dev.cv.center") == 848, "calculator: 4870 / 15000 mV with 68k -> 848")
    page.click("#btnVcaAcc"); page.wait_for_timeout(150)
    check(page.evaluate("window.__dev.cv.flags") == 3, "VCA accent toggle clears flag bit 2")
    page.select_option("#selCvMode", "1"); page.wait_for_timeout(150)
    check(page.evaluate("window.__dev.cv.mode") == 1, "accent model TB-303")
    k = mark(); page.fill("#rngCutoff", "100"); page.dispatch_event("#rngCutoff", "input"); page.wait_for_timeout(50)
    check([0xB0, 74, 100] in page.evaluate('window.__sent')[k:], "cutoff slider sends CC 74 on the unit's channel")

    page.evaluate("window.__dev.quiet = true")
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

    # ---- Nudge offline (never connected): local rotation, ratchets included
    page = b.new_context().new_page(); page.add_init_script(FAKE)
    page.on("pageerror", lambda e: errs.append(str(e)))
    page.goto(PAGE)
    page.evaluate("patterns[0] = [0,1,2,3,4].map(i => ({cv: 20 + i, type: i === 3 ? 2 : 0, rat: [2,0,1,0,0][i]})); renderSteps()")
    rows = lambda: page.evaluate("patterns[0].map(s => [s.cv, s.type, s.rat])")
    o = rows()
    page.click("#btnNudgeR")
    check(rows() == o[-1:] + o[:-1], f"offline nudge right rotates steps and ratchets ({rows()})")
    check("is-dirty" in page.get_attribute(".pat-pill:nth-child(1)", "class"), "offline nudge marks the pattern dirty")
    check(page.inner_text("#steps .step-card[data-i='0'] .step-name") == "C4" and "is-on" in page.get_attribute("#steps .step-card[data-i='1'] [data-a='r2']", "class"),
          "offline grid shows the rotation (note and 3x ratchet)")
    page.keyboard.press("Shift+ArrowLeft"); page.keyboard.press("Shift+ArrowLeft")
    check(rows() == o[1:] + o[:1], "offline Shift+Left twice")
    page.click("#btnUndo"); page.click("#btnUndo")
    check(rows() == o[-1:] + o[:-1], "offline Undo steps back through the nudges")
    check(page.evaluate("window.__sent.length") == 0, "offline: nothing sent")
    check(not errs, "no page errors (offline) " + "; ".join(errs))
    b.close()

print(f"\n{'FAILED ' + str(len(fails)) if fails else 'all passed'}")
sys.exit(1 if fails else 0)
