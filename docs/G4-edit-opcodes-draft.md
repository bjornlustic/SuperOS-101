# G4 live-edit opcodes: 101 implementation draft -- RETIRED 2026-08-25

**This draft is superseded. The work it planned is shipped and verified. Do not
implement from this file.**

Replaced by, in order of authority:

1. **The code.** `src/engine.h` (`setStep` / `setLength` / `randomize`, next to
   `set()`) and `src/main.cpp` (`OnSysex` cases 0x16 / 0x18 / 0x15). Committed in
   `abbcba1`, which also records the hardware verification.
2. **The protocol comment** at the top of `src/midi.h`, which is the canonical
   statement of the 101's opcode meanings and lane map.
3. **The spec**, `SuperOS-USBC/docs/EDIT_OPCODES.md` (claim-014), for the
   fleet-wide numbering.
4. **The tests**, `sim/sim.cpp` G4 block and G12 matrix, for the behaviour that is
   actually asserted.

## Why it is retired rather than kept as a record

Two reasons, both of which are the point of claim-035:

* **It duplicated the source.** The body was a verbatim copy of the three engine
  methods and the three dispatch cases. Two copies of the same code drift, and the
  copy nobody compiles drifts silently. The source is now the single statement.
* **It had gone actively wrong.** Its closing section still instructed the reader
  to "document 0x13/0x18/0x15 as incoming edits, and note 0x13 is
  length-dispatched (len 1 = dump-all, len 5 = set step)". That is CORE's first,
  **withdrawn** 808-based numbering. CORE revised SET STEP to 0x16 precisely
  because 0x13 is REQUEST-ALL on both the 303 and the 101, and the shipped code
  keeps 0x13 as REQUEST-ALL with no length dispatch. A reader following this file
  today would have implemented the collision the revision existed to remove.

The history is in the commits, which is where history belongs. What the draft got
right is now in the code and its comments; what it got wrong is recorded above so
the mistake is not repeatable rather than merely deleted.

## The one thing here that is still open

The other half of G4: "editing under clock disturbs nothing" needs the clocked
audio A/B against finding 23. That is CORE's capture, and the 101's edit schedule
is delivered at `SuperOS-USBC/timing/101.edits.json`. Nothing in this repo blocks
it.
