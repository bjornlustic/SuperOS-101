# SuperOS-101 Shortcut Guide

`SuperOS-101-Shortcut-Guide.pdf`: one picture of the real SH-101 panel per function, with the
controls to press ringed, the LEDs the firmware shows at that moment lit on the photo, and the keys
in order underneath. Same shape as the TR-909, TB-303, S900 and S950 guides. The build stamps the
date and the `tools/buildrev.py` revision on the cover.

    python3 docs/shortcut-guide/build_guide.py    # capture + HTML + PDF (needs Chrome; builds the sim if missing)

| file | what it is |
| --- | --- |
| `sh101-photo-wikimedia.jpg` | the source photo, unchanged: [SH-101.jpg](https://commons.wikimedia.org/wiki/File:SH-101.jpg) by Wikipedia user "909 bd", Wikimedia Commons, CC BY-SA 3.0. |
| `prepare_photo.py` | makes `panel-photo.jpg` from it: rotated level (0.25 degrees), cropped to the unit, a half-strength per-channel levels stretch. |
| `panel-photo.jpg` | the prepared photo (2334 x 1296), the base of every drawing. |
| `panel.py` | every control measured in `panel-photo.jpg` pixels: LOAD PLAY DOWN UD UP HOLD KT and their LEDs, POWER, key1..key32 (key1 = lowest F, key8 = lowest C), PORTASW, TRANSPOSE, PORTAKNOB, VOLUME, BENDER, the bender sliders, TUNE, RATE and the upper sections. Draws lit LEDs as a red glow on the photo, blinking ones half lit in a dashed ring, and small key numbers. |
| `check_panel.py` | alignment proof: draws every measured box over the photo (`check/overlay.png` plus zoomed thirds). |
| `content.py` | one `F(...)` per function: callouts, the scene to play, the text, the exact LED state the capture must show, extra checks (`also`), and the reference-page line. |
| `capture.py` | plays each scene on a fresh machine (empty EEPROM and flash) in the host simulator `sim/superos101-sim`, which links the real firmware, and samples the P2 port image for 1 s: each LED is on, blinking or off. |
| `build_guide.py` | callouts (black = press, red = hold, blue dashed = group), a crop per scene (the whole strip and keyboard, or zoomed to POWER..key 15, or to the button strip), the capture checks, page packing, contents, reference page, PDF via headless Chrome. Stops if any LED differs from what `content.py` expects. `guide.html` references `panel-photo.jpg`, so keep them together. |

**License.** The panel photo is CC BY-SA 3.0 (Wikipedia user 909 bd, Wikimedia Commons). The
panel images in this guide (`panel-photo.jpg` and every drawing built on it, in `guide.html` and the
PDF) are adaptations of that photo and are distributed under the same license, CC BY-SA 3.0. The
guide credits the photo on the cover and on the reference page.

The sim's REPL gained one command for this, `wait <ms>` (run the loop for that much virtual time),
so blinking LEDs and the 2 s menu combo can be timed. Firmware (`src/`) is untouched.
