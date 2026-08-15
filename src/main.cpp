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

static engine::NoteEngine    g_engine;
static controls::Buttons     g_buttons;
static bool                  g_lastClock = false;
static uint8_t               g_rnd = 0xA5;
static uint8_t               g_rndHeld = 0;       // synced S&H latch (MIDI sync on)
static uint32_t              g_rstUntilMs = 0;    // CLOCK-RESET pulse stretcher
static uint32_t              g_lastEditMs = 0;    // for the deferred flash flush

// --- MIDI note input ----------------------------------------------------------
// In-range notes (53..84 = keys F3..C6 at the middle octave) become keyboard
// bits; others go on a small last-note stack and play as a mono override.
static uint32_t g_midiMask = 0;
static uint8_t  g_extStack[8]; static uint8_t g_extN = 0;

static void MidiNoteOn(uint8_t note, uint8_t) {
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
static void SendPattern(uint8_t p) {
  if (p >= engine::NUM_PATTERNS) return;
  engine::Sequencer& sq = g_engine.seq();
  uint8_t raw[engine::MAX_STEPS + 1];
  raw[0] = sq.length(p);
  memcpy(raw + 1, sq.steps(p), raw[0]);
  uint8_t body[4 + 128];
  body[0] = 0x11; body[1] = p;
  uint8_t x = 0; for (uint8_t i = 0; i <= raw[0]; ++i) x ^= raw[i];
  body[2] = x & 0x7F; body[3] = (x >> 7) & 1;
  uint8_t n = midi::Pack7(raw, (uint8_t)(raw[0] + 1), body + 4);
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
  uint8_t n = 7;
  for (const char* v = SUPEROS_VERSION; *v && n < sizeof(body); ++v) body[n++] = (uint8_t)(*v & 0x7F);
  midi::SendSysex(body, n);
}
static void SendAck(uint8_t code) { uint8_t b[2] = {0x14, code}; midi::SendSysex(b, 2); }
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
      uint8_t raw[engine::MAX_STEPS + 1];
      uint8_t n = midi::Unpack7(body + 4, (uint8_t)(len - 4), raw, sizeof(raw));
      if (n < 1 || raw[0] > engine::MAX_STEPS || n < (uint8_t)(raw[0] + 1) || body[1] >= engine::NUM_PATTERNS) { SendAck(2); break; }
      uint8_t x = 0; for (uint8_t i = 0; i <= raw[0]; ++i) x ^= raw[i];
      if ((x & 0x7F) != body[2] || ((x >> 7) & 1) != body[3]) { SendAck(1); break; }
      g_engine.seq().set(body[1], raw + 1, raw[0]);
      g_lastEditMs = millis();
      SendAck(0);
      break;
    }
    case 0x1D: if (len >= 2) { g_engine.selectPattern(body[1]); SendConfig(); } break;
    case 0x1E: if (len >= 2) { g_engine.setPlaying(body[1] != 0); } break;   // transport from the editor
    case 0x20: SendConfig(); break;
    case 0x22:
      if (len >= 4) {
        midi::SetChannel(body[1]);
        sysex::SaveChannel(body[1]);
        midiclk::SetSync(body[2] != 0);
        midiclk::SetDiv(body[3]);
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

// LFO/EXT-CLK H->L edge (SH101.asm:768-789) + a placeholder random byte.
static uint8_t ClockAndRandom(bool &edge) {
  bool now = hw::ClockLevel();
  edge = (g_lastClock && !now);
  g_lastClock = now;
  uint8_t x = g_rnd; x ^= x << 3; x ^= x >> 5; x ^= x << 1;
  g_rnd = x ? x : 0xA5;
  return g_rnd & 0x3F;
}

// Entry points renamed for the combined build; src/boot.cpp owns the real
// setup()/loop() and dispatches here or to the 80C49 emulator (emu101.cpp).
void superos_setup() {
  hw::Init();
  g_engine.begin();
  // Prime the button edge detector with the current state so a button that
  // is still held from a firmware switch / power-on does not register as a
  // fresh press (which would toggle PLAY/LOAD/arp modes at boot).
  g_buttons.Scan();
  g_buttons.Scan();
  midi::Handlers h;
  h.noteOn = MidiNoteOn; h.noteOff = MidiNoteOff; h.program = MidiProgram;
  h.sysex = OnSysex; h.realtime = MidiRealtime;
  midi::SetHandlers(h);
#ifdef USB_SERIAL
  Serial.begin(115200);
#endif
}

void superos_loop() {
  // MIDI: notes, SysEx, realtime (divided clock + transport via midiclock.h)
  midi::Poll();
  bool divEdge = midiclk::ConsumeEdge();
  if (midiclk::ConsumeStart()) g_engine.restart();

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
  bool clockEdge;
  uint8_t random = ClockAndRandom(clockEdge);
  if (midiclk::SyncOn()) {
    clockEdge = divEdge;
    if (divEdge) g_rndHeld = random;
    random = g_rndHeld;
  }
  hw::DacWrite(random, ROUTE_RANDOM);

  // 9-12. LOAD/PLAY/ARP/keyboard -> note CV, then CV OUTPUT -> CV-OUT sample/hold
  engine::Output out = g_engine.update(keys, ExtCv(), g_buttons.hold, panel, g_buttons, clockEdge);
  hw::DacWrite(out.cv, ROUTE_CV_OUT);
  MidiOut(out.cv, out.gate);
  if (g_engine.seq().consumeStepChanged() && midi::HostSeen()) SendStep();

  // Deferred pattern persistence: SysEx writes and panel edits land in RAM;
  // flush one dirty pattern per pass once things have been quiet for 500 ms
  // and we are not recording (a page write halts the CPU for a few ms).
  if (g_engine.seq().dirty() && g_engine.mode() != engine::MODE_SEQ_LOAD &&
      millis() - g_lastEditMs > 500) {
    g_engine.seq().flushOne();
    g_lastEditMs = millis();
  }

  // 13. GATE / CLOCK-RESET / LEDs (PORTAMENTO OFF: TODO)
  // CLOCK RESET fires on engine events and, with sync ON, on every division
  // boundary (hard-syncs the analog LFO). Stretched to >=2 ms: our loop pass
  // is far shorter than the original's 3.5 ms, so a 1-pass pulse would be
  // too narrow for the LFO reset circuit.
  if (out.clockReset || (midiclk::SyncOn() && divEdge))
    g_rstUntilMs = millis() + 2;
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
  hw::P2Write(p2);
}
