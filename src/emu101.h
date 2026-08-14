// SuperOS-101 — 80C49 emulator firmware side (original mask ROM on the AVR)
#pragma once

namespace emu101 {
bool available();   // true when the mask ROM is embedded (rom101_data.h)
void setup();
void loop();
}
