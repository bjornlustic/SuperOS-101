// SuperOS-101 - main loop
//
// Mirrors the original SH-101 program loop (steps 3-13, see docs/FIRMWARE.md).
// The DAC/mux protocol and scan invariants are faithful (src/hw.h). Panel decode
// (src/controls.h), the keyboard/arp/sequencer engine (src/engine.h), DIN MIDI
// (src/midi.h: notes in/out, clock sync, SysEx editor protocol) and the flash
// pattern store (src/flash_persist.h) are wired in. Remaining stubs:
// KEY-TRANSPOSE shift value and exact portamento timing (marked at their sites).
#include <Arduino.h>
#include "pins.h"
#include "hw.h"
#include "controls.h"
#include "engine.h"
#include "menu.h"
#include "midi.h"
#include "midiclock.h"
#include "sysex.h"
#include "cvdac.h"

static engine::NoteEngine    g_engine;
static uint8_t               g_lastVel = 0;       // velocity of the latest MIDI note-on (accent trigger)
static bool                  g_prevGate = false;  // for the fresh-note edge into cvdac
static controls::Buttons     g_buttons;
static bool                  g_lastClock = false;
static uint8_t               g_rnd = 0xA5;
static uint8_t               g_rndHeld = 0;       // synced S&H latch (MIDI sync on)
static uint32_t              g_rstUntilMs = 0;    // CLOCK-RESET pulse stretcher
static bool                  g_clkResync = false; // re-prime the clock edge detector after a reset pulse
static uint32_t              g_portaUntilMs = 0;  // PORTAMENTO OFF pulse (non-legato notes)
static uint32_t              g_lastEditMs = 0;    // for the deferred flash flush
static uint32_t              g_lastRawFallMs = 0; // raw T1 fall (AUTO LFO div measurement)
static bool                  g_rstSinceFall = false; // a reset pulse polluted this interval

// --- MIDI note input ----------------------------------------------------------
// In-range notes (53..84 = keys F3..C6 at the middle octave) become keyboard
// bits; others go on a small last-note stack and play as a mono override.
static uint32_t g_midiMask = 0;
static uint8_t  g_extStack[8]; static uint8_t g_extN = 0;

static void MidiNoteOn(uint8_t note, uint8_t vel) {
  g_lastVel = vel;   // consumed by the accent trigger when the engine starts the note
  int16_t key = (int16_t)note - (engine::MIDI_CV_OFFSET + 0x0C + engine::CV_BASE);
  if (key >= 0 && key < 32) { g_midiMask |= (1UL << key); return; }
  for (uint8_t i = 0; i < g_extN; ++i) if (g_extStack[i] == note) return;
  if (g_extN == sizeof(g_extStack)) { memmove(g_extStack, g_extStack + 1, --g_extN); }
  g_extStack[g_extN++] = note;
}
static void MidiNoteOff(uint8_t note) {
  int16_t key = (int16_t)note - (engine::MIDI_CV_OFFSET + 0x0C + engine::CV_BASE);
  if (key >= 0 && key < 32) { g_midiMask &= ~(1UL << key); return; }
  for (uint8_t i = 0; i < g_extN; ++i)
    if (g_extStack[i] == note) { memmove(g_extStack + i, g_extStack + i + 1, g_extN - i - 1); --g_extN; return; }
}
static int16_t ExtCv() {
  if (!g_extN) return -1;
  int16_t cv = (int16_t)g_extStack[g_extN - 1] - engine::MIDI_CV_OFFSET;
  return cv < 0 ? 0 : (cv > engine::CV_MAX ? engine::CV_MAX : cv);
}
static void MidiProgram(uint8_t pc) { g_engine.selectPattern(pc & 7); }
static void MidiRealtime(uint8_t b) { midiclk::OnRealtime(b); }
static void MidiCC(uint8_t num, uint8_t val) { cvdac::OnCc(num, val); }   // filter cutoff (CC 74 default)

// --- MIDI note output --------------------------------------------------------
static int16_t g_outNote = -1;
static void MidiOut(uint8_t cv, bool gate) {
  int16_t note = gate ? (int16_t)cv + engine::MIDI_CV_OFFSET : -1;
  if (note == g_outNote) return;
  if (note >= 0) midi::NoteOn((uint8_t)note);          // new note first: legato-friendly
  if (g_outNote >= 0) midi::NoteOff((uint8_t)g_outNote);
  g_outNote = note;
}

// --- SysEx (editor protocol, see midi.h) -----------------------------------
// 0x11 payload: [0]=len, [1..len]=steps, then ceil(len/4) packed ratchet
// bytes (2 bits/step). The checksum XORs every payload byte, so hosts that
// predate ratchets fail the checksum only if they also parse the tail; ours
// sends/expects the tail both ways, and 0x12 accepts payloads without it.
static void SendPattern(uint8_t p) {
  if (p >= engine::NUM_PATTERNS) return;
  engine::Sequencer& sq = g_engine.seq();
  uint8_t raw[engine::MAX_STEPS + 1 + engine::MAX_STEPS / 4];
  raw[0] = sq.length(p);
  memcpy(raw + 1, sq.steps(p), raw[0]);
  uint8_t rb = (uint8_t)((raw[0] + 3) >> 2);
  memcpy(raw + 1 + raw[0], sq.ratchets(p), rb);
  uint8_t total = (uint8_t)(1 + raw[0] + rb);
  uint8_t body[4 + 188];   // 4 + PackedLen(161)
  body[0] = 0x11; body[1] = p;
  uint8_t x = 0; for (uint8_t i = 0; i < total; ++i) x ^= raw[i];
  body[2] = x & 0x7F; body[3] = (x >> 7) & 1;
  uint8_t n = midi::Pack7(raw, total, body + 4);
  midi::SendSysex(body, (uint8_t)(4 + n));
}
static void SendConfig() {
  uint8_t body[8 + 24];
  body[0] = 0x21; body[1] = 0;                       // fw 0 = SuperOS
  body[2] = midi::Channel();
  body[3] = midiclk::SyncOn() ? 1 : 0;
  body[4] = midiclk::DivIdx();
  body[5] = g_engine.seq().current();
  body[6] = persist::Available() ? 1 : 0;
  body[7] = midiclk::LfoSyncOn() ? 1 : 0;
  body[8] = midiclk::LfoDivIdx();
  uint8_t n = 9;
  for (const char* v = SUPEROS_VERSION; *v && n < sizeof(body); ++v) body[n++] = (uint8_t)(*v & 0x7F);
  midi::SendSysex(body, n);
}
static void SendAck(uint8_t code) { uint8_t b[2] = {0x14, code}; midi::SendSysex(b, 2); }
// 0x24: CV DAC config (cvdac.h Pack order) + live codes for the editor's meters.
static void SendCvConfig() {
  uint8_t body[1 + cvdac::CFG_LEN + 4];
  body[0] = 0x24;
  cvdac::Pack(cvdac::Get(), body + 1);
  uint16_t a = cvdac::CodeA(), b = cvdac::CodeB();
  body[1 + cvdac::CFG_LEN + 0] = a & 0x7F; body[1 + cvdac::CFG_LEN + 1] = (a >> 7) & 0x1F;
  body[1 + cvdac::CFG_LEN + 2] = b & 0x7F; body[1 + cvdac::CFG_LEN + 3] = (b >> 7) & 0x1F;
  midi::SendSysex(body, sizeof(body));
}
static void SendStep() {
  uint8_t b[3] = {0x15, g_engine.seq().current(), g_engine.seq().position()};
  midi::TrySendSysex(b, 3);
}

static void OnSysex(const uint8_t* body, uint8_t len) {
  if (sysex::HandleCommon(body, len)) return;
  switch (body[0]) {
    case 0x10: if (len >= 2) SendPattern(body[1]); break;
    case 0x13: for (uint8_t p = 0; p < engine::NUM_PATTERNS; ++p) SendPattern(p); break;
    case 0x12: {
      if (len < 4) { SendAck(2); break; }
      uint8_t raw[engine::MAX_STEPS + 1 + engine::MAX_STEPS / 4];
      uint8_t n = midi::Unpack7(body + 4, (uint8_t)(len - 4), raw, sizeof(raw));
      if (n < 1 || raw[0] > engine::MAX_STEPS || n < (uint8_t)(raw[0] + 1) || body[1] >= engine::NUM_PATTERNS) { SendAck(2); break; }
      // Checksum over everything received: old hosts send 1+len bytes, new
      // ones append the packed ratchet tail; both XOR what they sent.
      uint8_t x = 0; for (uint8_t i = 0; i < n; ++i) x ^= raw[i];
      if ((x & 0x7F) != body[2] || ((x >> 7) & 1) != body[3]) { SendAck(1); break; }
      uint8_t rb = (uint8_t)(n - 1 - raw[0]);   // 0 for pre-ratchet hosts
      g_engine.seq().set(body[1], raw + 1, raw[0], rb ? raw + 1 + raw[0] : nullptr, rb);
      g_lastEditMs = millis();
      SendAck(0);
      break;
    }
    case 0x1D: if (len >= 2) { g_engine.selectPattern(body[1]); SendConfig(); } break;
    case 0x1E: if (len >= 2) { g_engine.setPlaying(body[1] != 0); } break;   // transport from the editor
    // Live-edit opcodes (docs/EDIT_OPCODES.md, gate G4). Atomic in place; 0x13
    // stays REQUEST-ALL (above). ack 0x14: 0 ok, 1 bad length, 2 out of range.
    case 0x16:   // SET STEP [pat, lane, step, value]
      if (len >= 5) { bool ok = g_engine.seq().setStep(body[1], body[2], body[3], body[4]);
                      if (ok) g_lastEditMs = millis(); SendAck(ok ? 0 : 2); }
      else SendAck(1);
      break;
    case 0x18:   // SET LENGTH [pat, length]
      if (len >= 3) { bool ok = g_engine.seq().setLength(body[1], body[2]);
                      if (ok) g_lastEditMs = millis(); SendAck(ok ? 0 : 2); }
      else SendAck(1);
      break;
    case 0x1B:   // SET RATCHET [pat, step, 0..2] (0 = single, 1 = 2x, 2 = 3x)
      if (len >= 4) { bool ok = g_engine.seq().setRatchet(body[1], body[2], body[3]);
                      if (ok) g_lastEditMs = millis(); SendAck(ok ? 0 : 2); }
      else SendAck(1);
      break;
    // RANDOMIZE [pat, lane, density, seed_lo, seed_hi]. On 0x2E, NOT 0x15.
    //
    // 0x15 inbound is deliberately GONE and must not come back. This machine
    // EMITS 0x15 as its playhead position on every step change (SendStep
    // above), and direction is not a property a MIDI cable preserves: with
    // thru on, a DAW echo, or any host that re-emits unrecognised SysEx, the
    // 101's own playhead notification arrives back at its input. CORE's call
    // after three machines reported the same collision.
    //
    // WHAT IT ACTUALLY COST ON THIS MACHINE, because the fleet record
    // overstates it here: SendStep emits THREE bytes, and the old inbound
    // handler required len >= 6, answering ack 1 below that (see 5e20bf0^).
    // So an echo could not reach randomize() at all -- it produced one 0x14
    // ack per step on the editor's own wire. An ACK STORM, not data loss. The
    // pattern-destroying path needed a SIX-byte inbound 0x15, which this
    // machine's own three-byte notification cannot produce. A808's "8
    // randomizes per second at 120 BPM" is the 808's number on the 808's
    // payload; do not quote it for the 101.
    //
    // The move is still right, and NOT because the damage was small: an ack
    // per step is its own fault, and "safe because the payload lengths
    // differ" is not a protocol property. It is a coincidence of two
    // independently chosen layouts, and it stops holding the moment either
    // one changes.
    //
    // 0x16 SET STEP keeps its number despite being dual-direction too, and the
    // difference is the blast radius rather than the collision: an echoed 0x16
    // re-sets a step to the value it already holds and is idempotent, while an
    // echoed RANDOMIZE destroys the pattern.
    //
    // It was briefly 0x38 here (5e20bf0) and that was withdrawn one
    // commit later: 0x38 is CMD_FRAM_PROBE inbound on the 606. Both 0x15 and
    // 0x38 were proposed as "free" and both were taken, each caught by an
    // agent reading its own source AFTER publication. 0x2E is the first byte
    // taken from `tools/opcode_free.py` against the registry rather than
    // proposed and then checked, and the 101's full both-directions table is
    // published in docs/OPCODES.tsv so the allocator can see this machine.
    //
    // The payload's second byte is now a LANE, not the retired `scope`
    // (claim-057); see engine::randomize for why scope could not be salvaged.
    case 0x2E:
      if (len >= 6) { uint16_t seed = (uint16_t)(body[4] | ((uint16_t)body[5] << 7));
                      bool ok = g_engine.seq().randomize(body[1], body[2], body[3], seed, (uint8_t)(millis() & 0xFF));
                      if (ok) g_lastEditMs = millis(); SendAck(ok ? 0 : 2); }
      else SendAck(1);
      break;
    case 0x20: SendConfig(); break;
    case 0x23: SendCvConfig(); break;
    case 0x25: {   // set CV DAC config; the editor gets the applied values back
      cvdac::Settings s = cvdac::Get();
      if (len >= 1 + cvdac::CFG_LEN && cvdac::Unpack(body + 1, s)) cvdac::Set(s);
      SendCvConfig();
      break;
    }
    case 0x22:
      // <chan><sync><div>[<lfoSync><lfoDiv>] — the LFO pair is optional so
      // pre-LFO-sync hosts keep working (their 4-byte form leaves it alone).
      if (len >= 4) {
        midi::SetChannel(body[1]);
        sysex::SaveChannel(body[1]);
        midiclk::SetSync(body[2] != 0);
        midiclk::SetDiv(body[3]);
        if (len >= 6) { midiclk::SetLfoSync(body[4] != 0); midiclk::SetLfoDiv(body[5]); }
        SendConfig();
      }
      break;
    default: break;
  }
}

// Scan the 4x8 keyboard into a 32-bit held-key mask (bit 0 = lowest key).
static uint32_t ReadKeyboard() {
  uint32_t mask = 0;
  for (uint8_t bank = 0; bank < 4; ++bank) {
    uint8_t keys = hw::ScanRead((ScanCol)(COL_KBD_BANK0 + bank)); // 1 = down
    mask |= (uint32_t)keys << (bank * 8);
  }
  return mask;
}

// LFO/EXT-CLK H->L edge (SH101.asm:768-789) = step; L->H edge = half-step
// (gate-off point), plus a placeholder random byte.
static uint8_t ClockAndRandom(bool &edge, bool &halfEdge) {
  bool now = hw::ClockLevel();
  edge = (g_lastClock && !now);
  halfEdge = (!g_lastClock && now);
  g_lastClock = now;
  uint8_t x = g_rnd; x ^= x << 3; x ^= x >> 5; x ^= x << 1;
  g_rnd = x ? x : 0xA5;
  return g_rnd & 0x3F;
}

// Entry points renamed for the combined build; src/boot.cpp owns the real
// setup()/loop() and dispatches here or to the 80C49 emulator (emu101.cpp).
void superos_setup() {
  hw::Init();
  // Re-entry (in-process firmware switch, combined.h SwitchTo): every piece
  // of loop state back to its boot value, so a stint in the other firmware
  // cannot leak into this one (a stale MODE_SEQ_PLAY would start the
  // sequencer at the switch).
  g_engine.reset();
  g_lastVel = 0; g_prevGate = false; g_midiMask = 0; g_extN = 0; g_outNote = -1;
  g_lastClock = false; g_rstUntilMs = 0; g_clkResync = false; g_portaUntilMs = 0;
  g_lastEditMs = 0; g_lastRawFallMs = 0; g_rstSinceFall = false; g_rndHeld = 0;
  g_engine.begin();
  // Prime the button edge detector with the current state so a button that
  // is still held from a firmware switch / power-on does not register as a
  // fresh press (which would toggle PLAY/LOAD/arp modes at boot).
  g_buttons.Scan();
  g_buttons.Scan();
  midi::Handlers h;
  h.noteOn = MidiNoteOn; h.noteOff = MidiNoteOff; h.program = MidiProgram;
  h.sysex = OnSysex; h.realtime = MidiRealtime; h.cc = MidiCC;
  midi::SetHandlers(h);
  cvdac::Begin();   // MCP4822: VOUTA at the cutoff center, VOUTB parked
#ifdef USB_SERIAL
  Serial.begin(115200);
#endif
}

// In-process firmware switch (combined.h SwitchTo): edits made here must
// survive the switch, and no note may hang on MIDI OUT.
void superos_teardown() {
  g_engine.seq().flushAll();
  if (g_outNote >= 0) { midi::NoteOff((uint8_t)g_outNote); g_outNote = -1; }
}

void superos_loop() {
  // MIDI: notes, SysEx, realtime (divided clock + transport via midiclock.h)
  midi::Poll();
  bool divEdge = midiclk::ConsumeEdge();
  // Sync ON makes the 101 a real transport slave: the host's Start STARTS
  // the sequencer (play in the DAW and the machine plays; nobody has to
  // also press PLAY here or send 0x1E), and Stop stops it. Sync OFF keeps
  // the old behaviour: Start only aligns an already-playing sequencer.
  if (midiclk::ConsumeStart()) {
    g_engine.restart();
    if (midiclk::SyncOn()) g_engine.playFromHost();   // step 0 sounds on the downbeat tick
  }
  if (midiclk::ConsumeStop() && midiclk::SyncOn()) g_engine.setPlaying(false);
  // Sync ON only rules while the host's clock is actually on the wire
  // (midiclock.h Driving); otherwise the LFO/EXT-CLK steps the machine so a
  // PLAY press is never silent.
  const uint32_t nowMs = millis();
  const bool driving = midiclk::Driving(nowMs);

  // 3. RANGE / OCTAVE / ADSR
  controls::Panel panel = controls::ReadPanel();

  // 4. RANGE DATA OUTPUT -> VCO sample/hold (octave-select CV)
  hw::DacWrite(panel.rangeOffset, ROUTE_VCO);

  // 5. KEYBOARD READ + 8. FUNCTION SWITCH READ
  uint32_t keys = ReadKeyboard() | g_midiMask;
  g_buttons.Scan();
  menu::NotifyButtons(g_buttons.now, millis());  // HOLD+KEY-TRANSP 2 s -> config menu

  // 6/7. CLOCK CHECK + RANDOM DATA OUTPUT -> RANDOM sample/hold
  // Sync ON: the divided MIDI clock replaces the T1 (LFO/EXT-CLK) edge, and
  // the random S&H steps once per division instead of free-running.
  bool clockEdge, halfEdge;
  uint8_t random = ClockAndRandom(clockEdge, halfEdge);
  // A CLOCK-RESET pulse restarts the analog LFO, and the level transitions
  // the restart causes must not read as clock edges: the stale detector state
  // turned them into a spurious step right after play start. One pass of
  // grace after the pulse proved too short on hardware: the release glitch
  // landed a pass later, advanced the sequencer off the just-played step 0,
  // and the first step sounded skipped. Swallow edges while the pulse is
  // active and for a CLK_SETTLE_MS settle window after it ends
  // (ClockAndRandom keeps g_lastClock primed throughout); 3 ms is well under
  // the shortest real clock half-period. Only the LFO path needs this; with
  // sync ON the edges below are MIDI-derived.
  static const uint8_t CLK_SETTLE_MS = 3;
  // Raw T1 fall bookkeeping for the AUTO LFO division (midiclock.h): a
  // fall-to-fall interval is a slider measurement only if no CLOCK-RESET
  // pulse landed inside it (a hard-synced LFO's forced transitions measure
  // the reset rate, not the slider).
  {
    bool inRst = (int32_t)(g_rstUntilMs + CLK_SETTLE_MS - nowMs) > 0;
    if (inRst) g_rstSinceFall = true;
    if (clockEdge && !inRst) {
      if (g_lastRawFallMs && !g_rstSinceFall) {
        uint32_t iv = nowMs - g_lastRawFallMs;
        midiclk::LfoPeriodSample(iv > 30000 ? 30000 : (uint16_t)iv, nowMs);
      }
      g_lastRawFallMs = nowMs;
      g_rstSinceFall = false;
    }
  }
  if (!driving) {
    if ((int32_t)(g_rstUntilMs + CLK_SETTLE_MS - millis()) > 0) {
      clockEdge = false; halfEdge = false; g_clkResync = true;
    } else if (g_clkResync) {
      clockEdge = false; halfEdge = false; g_clkResync = false;
    }
  }
  if (driving) {
    clockEdge = divEdge;
    halfEdge = midiclk::ConsumeHalfEdge();
    if (divEdge) g_rndHeld = random;
    random = g_rndHeld;
  }
  hw::DacWrite(random, ROUTE_RANDOM);

  // 9-12. LOAD/PLAY/ARP/keyboard -> note CV, then CV OUTPUT -> CV-OUT sample/hold
  engine::Output out = g_engine.update(keys, ExtCv(), g_buttons.hold, panel, g_buttons, clockEdge, halfEdge);
  hw::DacWrite(out.cv, ROUTE_CV_OUT);
  MidiOut(out.cv, out.gate);

  // CV DAC (cvdac.h): a fresh note = gate rising from silence, or a non-legato
  // retrigger (the engine flags those with the PORTAMENTO-OFF pulse). It fires
  // the accent envelope when the note's MIDI velocity clears the threshold;
  // panel and sequencer notes carry no velocity (ACCENT_ALL flag covers them).
  // Then one tick of the envelope / cutoff slew and the DAC writes.
  {
    bool fresh = out.gate && (!g_prevGate || out.portaOff);
    g_prevGate = out.gate;
    if (fresh) { cvdac::NoteOn(g_lastVel); g_lastVel = 0; }
    cvdac::Tick();
  }
  if (midi::HostSeen()) {
    if (g_engine.seq().consumeStepChanged()) SendStep();
    // Panel edits/selection mirror to the editor: pattern data after LOAD,
    // config (current pattern) after PLAY+key.
    uint8_t ed = g_engine.seq().consumeEdited();
    for (uint8_t p = 0; ed && p < engine::NUM_PATTERNS; ++p) if (ed & (1u << p)) SendPattern(p);
    if (g_engine.seq().consumeSelChanged()) SendConfig();
  } else { g_engine.seq().consumeStepChanged(); g_engine.seq().consumeEdited(); g_engine.seq().consumeSelChanged(); }

  // Deferred pattern persistence: SysEx writes and panel edits land in RAM;
  // flush one dirty pattern per pass once things have been quiet for 500 ms
  // and we are neither recording nor CLOCKED (a page write halts the CPU for
  // a few ms, which under playback/arp was an audible timing hitch whenever
  // the editor's live edits went dirty mid-run). Edits made while playing
  // persist at stop; flushAll on LOAD exit is unchanged.
  {
    engine::Mode m = g_engine.mode();
    bool clocked = (m == engine::MODE_SEQ_PLAY || m == engine::MODE_SEQ_LOAD ||
                    m == engine::MODE_ARP_UP || m == engine::MODE_ARP_DOWN ||
                    m == engine::MODE_ARP_UPDOWN);
    if (g_engine.seq().dirty() && !clocked && millis() - g_lastEditMs > 500) {
      g_engine.seq().flushOne();
      g_lastEditMs = millis();
    }
  }

  // 13. GATE / CLOCK-RESET / PORTAMENTO OFF / LEDs
  // CLOCK RESET fires on engine events and, with sync ON + LFO sync ON, on
  // every LFO-division boundary (hard-syncs the analog LFO to the clock at
  // its own musical rate, midiclock.h lfoDiv). With LFO sync OFF the LFO
  // free-runs for modulation while the sequencer still steps from the
  // divided clock, so the engine's play-arm reset is suppressed too (only
  // in play mode: the ADSR=LFO keyboard retrigger stays). Stretched to
  // >=2 ms: our loop pass is far shorter than the original's 3.5 ms, so a
  // 1-pass pulse would be too narrow for the LFO reset circuit.
  {
    bool lfoEdge = midiclk::ConsumeLfoEdge();
    bool engineReset = out.clockReset &&
      !(driving && !midiclk::LfoSyncOn() && g_engine.mode() == engine::MODE_SEQ_PLAY);
    if (engineReset ||
        (driving && midiclk::LfoSyncOn() && lfoEdge && !midiclk::AutoListenSkip(nowMs))) {
      g_rstUntilMs = millis() + 2;
      g_rstSinceFall = true;
    }
  }
  // PORTAMENTO OFF (P23, active-high; the ROM idles it low and raises it
  // briefly on a fresh, non-legato note): the S/H jumps instead of gliding.
  // With the panel switch on AUTO this is what makes only legato notes glide;
  // ON / OFF ignore the line. Held a few ms so the RC has settled on the jump.
  if (out.portaOff) g_portaUntilMs = millis() + 4;
#ifdef DEBUG
  static uint32_t s_dbgNext = 0;
  if (Serial && millis() >= s_dbgNext) {
    s_dbgNext = millis() + 500;
    Serial.print(F("PIND=0x"));  Serial.print(PIND, HEX);
    Serial.print(F(" btnraw=0x")); Serial.print(g_buttons.now, HEX);
    Serial.print(F(" keys=0x")); Serial.println(keys, HEX);
  }
#endif
  // LED drive (SH101.asm:90-128): the two direct LEDs have OPPOSITE
  // polarities: HOLD is lit by P25=0, KEY-TRANSPOSE by P26=1 (extra inverter
  // TR10). Function-LED select 0 = all off, 1-5 = exactly one of
  // LOAD/PLAY/DOWN/U&D/UP. CLOCK RESET is ACTIVE-LOW (the ROM idles P24 high
  // and pulses it low; holding it low freezes the LFO).
  uint8_t p2 = (uint8_t)(P2_IDLE | g_engine.ledSelect());
  if (g_buttons.hold)  p2 &= (uint8_t)~(1u << P2_LED_HOLD);  // 0 -> HOLD lit
  if (out.gate)        p2 &= (uint8_t)~(1u << P2_GATE);      // bit=0 -> gate ON
  if ((int32_t)(g_rstUntilMs - millis()) > 0)
    p2 &= (uint8_t)~(1u << P2_CLOCK_RST);                    // pulse LOW
  if ((int32_t)(g_portaUntilMs - millis()) > 0)
    p2 |= (uint8_t)(1u << P2_PORTA_OFF);                     // pulse HIGH
  hw::P2Write(p2);
  hw::P2Obs().tick(millis());   // roll the LED/gate rate window (probe [97..113])
}
