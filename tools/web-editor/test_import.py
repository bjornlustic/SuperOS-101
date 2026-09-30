#!/usr/bin/env python3
"""Headless check of the editor's MIDI file import (parseSMF / convertNotes / the
import panel), on synthetic .mid files built here. Uses the fake unit from
test_performance.py; no real MIDI port is touched.

    python3 tools/web-editor/test_import.py
"""
import pathlib, re, struct, sys, tempfile
from playwright.sync_api import sync_playwright

HERE = pathlib.Path(__file__).parent
PAGE = (HERE / "index.html").resolve().as_uri()
FAKE = re.search(r'FAKE = r"""(.*?)"""', (HERE / "test_performance.py").read_text(), re.S).group(1)

def vlq(n):
    out = [n & 0x7F]; n >>= 7
    while n: out.insert(0, (n & 0x7F) | 0x80); n >>= 7
    return bytes(out)

def track(events):
    """events: [(abs_tick, raw bytes)] -> MTrk chunk with delta times, end-of-track appended."""
    body, t = b"", 0
    for tick, raw in sorted(events, key=lambda e: e[0]):
        body += vlq(tick - t) + raw; t = tick
    body += vlq(0) + b"\xFF\x2F\x00"
    return b"MTrk" + struct.pack(">I", len(body)) + body

def smf(fmt, tpq, tracks):
    return b"MThd" + struct.pack(">IHHH", 6, fmt, len(tracks), tpq) + b"".join(tracks)

def notes_track(notes, ch=0, running=True, vel0_off=True):
    """notes: [(start, end, note)]. Running status + velocity-0 note-offs by default."""
    ev = []
    for s, e, n in notes:
        ev.append((s, ("on", n))); ev.append((e, ("off", n)))
    ev.sort(key=lambda x: (x[0], x[1][0] == "on"))   # offs before ons on the same tick
    out, last = [], None
    for tick, (kind, n) in ev:
        st = (0x90 if kind == "on" or vel0_off else 0x80) | ch
        data = bytes([n, 100 if kind == "on" else (0 if vel0_off else 64)])
        raw = data if (running and st == last) else bytes([st]) + data
        last = st
        out.append((tick, raw))
    return track(out)

fails = []
def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond: fails.append(what)

def pack7(src):
    out = []
    for i in range(0, len(src), 7):
        ch = src[i:i + 7]; msb = 0
        for k, b in enumerate(ch):
            if b & 0x80: msb |= 1 << k
        out += [msb] + [b & 0x7F for b in ch]
    return out

with sync_playwright() as p:
    b = p.chromium.launch(channel="chrome")
    page = b.new_context().new_page()
    page.add_init_script(FAKE)
    errs = []
    page.on("pageerror", lambda e: errs.append(str(e)))
    page.goto(PAGE)
    page.click("#btnMidi")
    page.wait_for_timeout(500)

    def parse(data):
        return page.evaluate("a => { const r = parseSMF(new Uint8Array(a)); return {format: r.format, tpq: r.tpq, ts: r.timeSig, "
                             "tracks: r.tracks.map(t => ({name: t.name, notes: t.notes.map(n => [n.start, n.end, n.note, n.ch])}))}; }", list(data))

    def conv(notes, **o):
        opts = dict(tpq=96, perBeat=4, barTicks=384, offsetTicks=0, len=0, mono="high", oct="auto", split=False); opts.update(o)
        return page.evaluate("([n, o]) => { const r = convertNotes(n.map(([start, end, note]) => ({start, end, note})), o);"
                             " r.steps = r.chunks.map(c => c.map(s => [s.type, s.cv])); delete r.chunks; return r; }", [notes, opts])

    # ---- parser: type 0, running status, velocity-0 note-off, meta (name, time sig) and sysex
    notes = [(0, 20, 60), (24, 48, 62), (46, 70, 64)]
    ev = notes_track(notes)[8:-4]   # the note events without chunk header and end-of-track
    meta = b"\x00\xFF\x03\x04Lead" + b"\x00\xFF\x58\x04\x03\x02\x18\x08" + b"\x00\xF0\x03\x7E\x7F\xF7"
    tr = meta + ev + b"\x00\xFF\x2F\x00"
    f0 = smf(0, 96, [b"MTrk" + struct.pack(">I", len(tr)) + tr])
    r = parse(f0)
    check(r["format"] == 0 and r["tpq"] == 96, "type 0 header")
    check(r["tracks"][0]["name"] == "Lead" and r["ts"] == [3, 4], "track name and 3/4 time signature read")
    check([n[:3] for n in r["tracks"][0]["notes"]] == [[0, 20, 60], [24, 48, 62], [46, 70, 64]],
          f"running status + velocity-0 offs pair up ({r['tracks'][0]['notes']})")
    check(ev.count(0x90) == 1, "the file really uses running status (one 0x90 for six events)")

    # ---- parser: type 1, two note tracks on different channels, explicit 0x80 offs, a long delta
    f1 = smf(1, 480, [track([(0, b"\xFF\x51\x03\x07\xA1\x20")]),
                      notes_track([(0, 240, 48), (20000, 20100, 50)], ch=0, running=False, vel0_off=False),
                      notes_track([(0, 120, 72), (120, 240, 74), (240, 360, 76)], ch=2)])
    r = parse(f1)
    check(r["format"] == 1 and len(r["tracks"]) == 3, "type 1: three tracks")
    check([n[:3] for n in r["tracks"][1]["notes"]] == [[0, 240, 48], [20000, 20100, 50]], "0x80 offs and a multi-byte delta")
    check([n[3] for n in r["tracks"][2]["notes"]] == [2, 2, 2], "channel kept")
    check(page.evaluate("() => { try { parseSMF(new Uint8Array([1,2,3,4,5,6,7,8])); return false; } catch (e) { return true; } }"), "garbage is refused")

    # ---- conversion (1/16 at 96 tpq = 24 ticks a step)
    r = conv([[94, 110, 60], [190, 210, 62]])   # 94/24 = 3.9 -> step 4, 190/24 = 7.9 -> step 8
    s = r["steps"][0]
    check(s[4] == [0, 24] and s[8] == [0, 26] and len(s) == 16, f"early notes land on steps 5 and 9, 16 steps ({s[:10]})")
    r = conv([[0, 40, 60], [24, 48, 62]])
    check(r["steps"][0][0] == [0, 24] and r["steps"][0][1] == [2, 26], "overlap: the next note is legato")
    r = conv([[0, 20, 60], [24, 48, 62]])
    check(r["steps"][0][1] == [0, 26], "no overlap: a plain note")
    r = conv([[0, 72, 60], [96, 110, 62]])
    check(r["steps"][0][:5] == [[0, 24], [2, 24], [2, 24], [1, 0], [0, 26]], f"held 3 steps: two ties, then a rest ({r['steps'][0][:5]})")
    check(r["legatos"] == 2 and r["notes"] == 2, "summary counts ties and notes")
    chord = [[0, 20, 60], [0, 20, 67], [2, 20, 64]]
    check(conv(chord, mono="high")["steps"][0][0] == [0, 31] and conv(chord, mono="high")["dropped"] == 2, "mono Highest keeps G, drops 2")
    check(conv(chord, mono="low")["steps"][0][0] == [0, 24], "mono Lowest keeps C")
    check(conv(chord, mono="last")["steps"][0][0] == [0, 28], "mono Last keeps the latest start (E)")
    check(conv([[0, 20, 20], [24, 40, 30]])["shift"] == 2, "auto-fit: 20..30 moves up 2 octaves")
    check(conv([[0, 20, 100], [24, 40, 110]])["shift"] == -1, "auto-fit: 100..110 moves down 1 octave")
    check(conv([[0, 20, 60]])["shift"] == 0, "auto-fit: in range stays put")
    r = conv([[0, 20, 20], [24, 40, 120]])
    check(r["clamped"] == 1 and r["steps"][0][0][1] in (0, 63), f"span wider than the range: 1 clamped ({r['clamped']})")
    r = conv([[0, 20, 50]], oct="-2")
    check(r["shift"] == -2 and r["clamped"] == 1 and r["steps"][0][0] == [0, 0], "manual -2: clamped to the bottom")
    full = [[i * 24, i * 24 + 20, 48 + i % 12] for i in range(128)]
    r = conv(full)
    check(len(r["steps"]) == 1 and len(r["steps"][0]) == 128 and r["notes"] == 128 and r["rests"] == 0, "128 notes fill 128 steps")
    long = [[i * 24, i * 24 + 20, 60] for i in range(200)]
    r = conv(long)
    check(len(r["steps"]) == 1 and r["cut"] == 72, "200 steps, Cut: 128 kept, 72 notes reported cut")
    r = conv(long, split=True)
    check([len(c) for c in r["steps"]] == [128, 80], f"Split: 128 + 80 (the rest, whole bars) ({[len(c) for c in r['steps']]})")
    r = conv(long, split=True, len=64)
    check([len(c) for c in r["steps"]] == [64, 64, 64, 64], "Split at 64: four full patterns")
    r = conv([[0, 20, 60], [96, 110, 62]], offsetTicks=96)
    check(r["steps"][0][0] == [0, 26] and r["before"] == 1, "start offset skips earlier notes")
    r = conv([[0, 20, 60], [48, 70, 62]], perBeat=2)
    check(r["steps"][0][:2] == [[0, 24], [0, 26]] and len(r["steps"][0]) == 8, "1/8 grid")
    r = conv([[0, 20, 60], [32, 50, 62]], perBeat=6)
    check(r["steps"][0][:3] == [[0, 24], [1, 0], [0, 26]] and len(r["steps"][0]) == 24, "1/16T grid, 24 steps a bar")

    # ---- UI: import via the file picker, preview, write (0x12 bytes), Undo
    notes = [(i * 24, i * 24 + 20, [40, 52, 55, 59][i % 4]) for i in range(16)] + [(15 * 24, 16 * 24 + 10, 64)]
    mid = smf(0, 96, [notes_track(notes)])
    tmpdir = tempfile.TemporaryDirectory(); path = pathlib.Path(tmpdir.name) / "import_test.mid"
    path.write_bytes(mid)
    before = page.locator("#steps .step-name").all_inner_texts()
    page.set_input_files("#fileMid", str(path)); page.wait_for_timeout(300)
    check(page.is_visible("#impPanel") and "17 notes" in page.inner_text("#impSrc"), "panel opens, source lists 17 notes")
    summ = page.inner_text("#impSummary")
    check("octave shift 0" in summ, f"summary shows the shift ({summ})")
    check("1 dropped by the mono rule" in summ, "summary counts the chord note dropped")
    check("PREVIEW" in page.inner_text("#seqTitle").upper() and page.locator("#steps .step-card").count() == 16, "grid shows the 16-step preview")
    check(page.evaluate("patterns[0].length") == 16 and page.locator("#steps .step-name").all_inner_texts() != before, "preview does not touch the pattern data")
    k = len(page.evaluate("window.__sent"))
    page.click("#impWrite"); page.wait_for_timeout(400)
    sent = [m for m in page.evaluate("window.__sent")[k:] if m[:3] == [0xF0, 0x7D, 0x12]]
    steps = page.evaluate("patterns[0].map(s => (s.cv & 0x3F) | (s.type << 6))")
    raw = [len(steps)] + steps + [0] * ((len(steps) + 3) // 4) + [0] * ((len(steps) + 7) // 8)   # ratchet + accent tails
    x = 0
    for v in raw: x ^= v
    want = [0xF0, 0x7D, 0x12, 0, x & 0x7F, (x >> 7) & 1] + pack7(raw) + [0xF7]
    check(len(sent) == 1 and sent[0] == want, "Write sends the exact 0x12 push")
    check(steps[15] >> 6 == 0 and (steps[15] & 0x3F) == 64 - 36, "the chord at step 16 kept the highest note (E4)")
    check(not page.is_visible("#impPanel") and "PREVIEW" not in page.inner_text("#seqTitle").upper(), "panel closes after Write")
    page.click("#btnUndo"); page.wait_for_timeout(300)
    check(page.locator("#steps .step-name").all_inner_texts() == before, "Undo brings the old pattern back")

    # 128-step pattern through the 0x12 push (the length byte is 7-bit packed)
    mid128 = smf(0, 96, [notes_track([(i * 24, i * 24 + 20, 48 + i % 24) for i in range(128)])])
    path.write_bytes(mid128)
    page.set_input_files("#fileMid", str(path)); page.wait_for_timeout(300)
    k = len(page.evaluate("window.__sent"))
    page.click("#impWrite"); page.wait_for_timeout(400)
    sent = [m for m in page.evaluate("window.__sent")[k:] if m[:3] == [0xF0, 0x7D, 0x12]]
    raw = [128] + page.evaluate("patterns[0].map(s => (s.cv & 0x3F) | (s.type << 6))") + [0] * 32 + [0] * 16
    x = 0
    for v in raw: x ^= v
    check(len(sent) == 1 and sent[0] == [0xF0, 0x7D, 0x12, 0, x & 0x7F, (x >> 7) & 1] + pack7(raw) + [0xF7] and all(v < 0x80 for v in sent[0][1:-1]),
          "128 steps: one valid 7-bit 0x12 push")

    # drag and drop, Split into patterns 15 and 16, Cancel leaves everything alone
    page.evaluate("a => { const dt = new DataTransfer(); dt.items.add(new File([new Uint8Array(a)], 'drop.mid'));"
                  " document.body.dispatchEvent(new DragEvent('drop', {dataTransfer: dt, bubbles: true, cancelable: true})); }",
                  list(smf(1, 96, [track([]), notes_track([(i * 24, i * 24 + 20, 60) for i in range(200)])])))
    page.wait_for_timeout(300)
    check(page.is_visible("#impPanel") and page.text_content("#impFile") == "drop.mid", "drop a .mid opens the importer")
    page.select_option("#impPat", index=14); page.select_option("#impSplit", index=1); page.wait_for_timeout(200)
    summ = page.inner_text("#impSummary")
    check("15, 16" in summ and "hold PLAY and press keys 15, 16" in summ, f"Split fills 15 and 16 and names the song gesture ({summ})")
    page.click("#impCancel"); page.wait_for_timeout(200)
    check(not page.is_visible("#impPanel") and page.evaluate("patterns[14].length + patterns[15].length") == 0, "Cancel writes nothing")
    check(page.evaluate("curPat") == 0, "Cancel returns to the pattern that was open")
    check(not errs, "no page errors " + "; ".join(errs))
    tmpdir.cleanup()
    b.close()

print(f"\n{'FAILED ' + str(len(fails)) if fails else 'all passed'}")
sys.exit(1 if fails else 0)
