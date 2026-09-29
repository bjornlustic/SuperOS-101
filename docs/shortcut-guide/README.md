# SuperOS-101 Shortcut Guide

`SuperOS-101-Shortcut-Guide.pdf`: one drawing of the SH-101 panel per function, with the LEDs the
firmware shows at that moment, and the keys in order underneath. Same shape as the TR-909, TB-303,
S900 and S950 guides. The build stamps the date and the `tools/buildrev.py` revision on the cover.

    python3 docs/shortcut-guide/build_guide.py    # capture + HTML + PDF (needs Chrome; builds the sim if missing)

| file | what it is |
| --- | --- |
| `panel.py` | the SH-101 panel as black-and-white line art, laid out after the owner's manual drawings (`SH101-reference/SH101_OM.pdf` p.3, 31, 33). Helvetica Neue Medium. `centres()` names the controls: LOAD PLAY DOWN UD UP HOLD KT, key1..key32 (key1 = lowest F, key8 = lowest C), PORTASW, TRANSPOSE, PORTAKNOB, VOLUME, BENDER. |
| `content.py` | one `F(...)` per function: callouts, the scene to play, the text, the exact LED state the capture must show, extra checks (`also`), and the reference-page line. |
| `capture.py` | plays each scene on a fresh machine (empty EEPROM and flash) in the host simulator `sim/superos101-sim`, which links the real firmware, and samples the P2 port image for 1 s: each LED is on, blinking or off. |
| `build_guide.py` | callouts (red = held, blue dashed = group), the capture checks, page packing, contents, reference page, PDF via headless Chrome. Stops if any LED differs from what `content.py` expects. |

The sim's REPL gained one command for this, `wait <ms>` (run the loop for that much virtual time),
so blinking LEDs and the 2 s menu combo can be timed. Firmware (`src/`) is untouched.
