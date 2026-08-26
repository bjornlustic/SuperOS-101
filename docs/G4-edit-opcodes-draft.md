# G4 live-edit opcodes: 101 implementation draft (NOT YET APPLIED)

Spec: SuperOS-USBC/docs/EDIT_OPCODES.md, CORE revised numbering 2026-08-25:
SET STEP = 0x16, SET LENGTH 0x18, SELECT 0x1D (exists), REQUEST-ALL 0x13
(unchanged), RANDOMIZE 0x15, ack 0x14 <status>, 0x19 reserved (303 step-lock).
A101 adds incoming 0x16, 0x18, 0x15. The 0x13 collision is dissolved (SET STEP
moved off 0x13), so NO length-dispatch is needed.

Gated behind the 101's certified G3 (sequencing rule). This is the ready-to-apply
diff so the moment G3 lands: apply -> build -> flash -> run edit_test.py under
clock -> A/B against the clean G3. Edits are ATOMIC in place (single step / length
field), NOT a whole-pattern 0x12 rewrite, which is the point of G4 (claim-014).

Params: MAX_STEPS 128, 8 slots, mono. lane 0 = note (value MIDI note, 0 = rest),
lane 3 = gate (0 normal/retrigger, 1 legato/tie); lanes 1/2 absent -> status 2.
cv = note - 36, clamped 0..63.

## engine.h -- add to class Sequencer (public)

```cpp
// --- live host edits (SysEx 0x13/0x18/0x15), atomic in place under clock ---
bool setStep(uint8_t p, uint8_t lane, uint8_t idx, uint8_t value) {
  if (p >= NUM_PATTERNS || idx >= len_[p]) return false;      // grow with 0x18 first
  uint8_t s = step_[p][idx];
  if (lane == 0) {                                            // note field
    if (value == 0) s = StepByte(0, STEP_REST);
    else { int16_t cv = (int16_t)value - MIDI_CV_OFFSET;
           if (cv < 0) cv = 0; else if (cv > CV_MAX) cv = CV_MAX;
           s = StepByte((uint8_t)cv, STEP_NOTE); }            // a note edit clears legato
  } else if (lane == 3) {                                     // gate/legato field
    if (StepKind(s) == STEP_REST) return false;               // no gate on a rest
    s = StepByte(StepCv(s), value ? STEP_LEGATO : STEP_NOTE);
  } else return false;                                        // lanes 1/2 absent on the 101
  step_[p][idx] = s; dirty_ |= (uint8_t)(1u << p);
  if (p == cur_) stepChanged_ = true;
  return true;
}
bool setLength(uint8_t p, uint8_t len) {
  if (p >= NUM_PATTERNS || len < 1 || len > MAX_STEPS) return false;
  for (uint8_t i = len_[p]; i < len; ++i) step_[p][i] = StepByte(0, STEP_REST);
  len_[p] = len; dirty_ |= (uint8_t)(1u << p);
  if (p == cur_ && pos_ >= len) rewind();
  return true;
}
// scope 0 = whole pattern (note+rest), 1 = note VALUES only (keep rest/note/legato
// layout). density 0 clears, 127 fills, else P(note)=density/127. seed 14-bit:
// 0 = entropy, nonzero = deterministic (seeded xorshift) so a G4 capture repeats.
bool randomize(uint8_t p, uint8_t scope, uint8_t density, uint16_t seed, uint8_t entropy) {
  if (p >= NUM_PATTERNS || len_[p] == 0) return false;
  uint16_t st = seed ? seed : (uint16_t)(entropy | 0x100);           // never 0
  auto rnd = [&st]() -> uint8_t { st ^= st << 7; st ^= st >> 9; st ^= st << 8; return (uint8_t)(st & 0x7F); };
  for (uint8_t i = 0; i < len_[p]; ++i) {
    uint8_t r = rnd();
    bool on = density >= 127 ? true : (density == 0 ? false : (r < density));
    if (scope == 1) {                                                // note values only
      if (StepKind(step_[p][i]) == STEP_REST) continue;
      step_[p][i] = StepByte(rnd() & CV_MAX, StepKind(step_[p][i]));
    } else {
      step_[p][i] = on ? StepByte(rnd() & CV_MAX, STEP_NOTE) : StepByte(0, STEP_REST);
    }
  }
  dirty_ |= (uint8_t)(1u << p);
  if (p == cur_) stepChanged_ = true;
  return true;
}
```

## main.cpp -- OnSysex, replace `case 0x13` and add 0x18/0x15

```cpp
// 0x13 stays REQUEST-ALL unchanged (CORE revised numbering: SET STEP is 0x16,
// so the earlier collision is dissolved; no length-dispatch needed).
case 0x16:                                   // SET STEP [pat, lane, step, value]
  if (len >= 5) { bool ok = g_engine.seq().setStep(body[1], body[2], body[3], body[4]);
                  if (ok) g_lastEditMs = millis(); SendAck(ok ? 0 : 2); }
  else SendAck(1);
  break;
case 0x18:                                   // SET LENGTH [pat, length]
  if (len >= 3) { bool ok = g_engine.seq().setLength(body[1], body[2]);
                  if (ok) g_lastEditMs = millis(); SendAck(ok ? 0 : 2); }
  else SendAck(1);
  break;
case 0x15:                                   // RANDOMIZE [pat, scope, density, seed_lo, seed_hi]
  if (len >= 6) { uint16_t seed = body[4] | ((uint16_t)body[5] << 7);
                  bool ok = g_engine.seq().randomize(body[1], body[2], body[3], seed, (uint8_t)(millis() & 0xFF));
                  if (ok) g_lastEditMs = millis(); SendAck(ok ? 0 : 2); }
  else SendAck(1);
  break;
```

## Also in the SAME commit (G8: docs match code)

* Manual editor/SysEx section: document 0x13/0x18/0x15 as incoming edits, and note
  0x13 is length-dispatched (len 1 = dump-all, len 5 = set step).
* claim-014 / claim-101-a in the ledger: mark the 101's edit surface complete.

## Verification plan (post-flash, needs CORE audio for the clock A/B)

1. Headless read-back: each opcode acks 0, then 0x10 read-back shows the edit.
   Out-of-range pat/lane/step -> ack 2. Determinism: same nonzero seed twice ->
   identical read-back.
2. G4 proper: run under clock, drive these edits, compare jitter/drift to the
   clean G3. Pass = indistinguishable, no step lands late (that capture is CORE's).
```
