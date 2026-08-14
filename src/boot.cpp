// SuperOS-101 — combined-build boot dispatch (see combined.h)
//
// setup()/loop() live here; the SuperOS firmware's entry points are renamed
// superos_setup()/superos_loop() (src/main.cpp) and the emulator side is
// emu101::setup()/loop(). One firmware runs per boot, chosen by EE_FW_SELECT;
// the config menu (menu.h) writes the byte and reboots to switch.
#include <Arduino.h>
#include "combined.h"
#include "hw.h"
#include "menu.h"
#include "emu101.h"
#include "midiclock.h"

#ifdef __AVR__
#include <avr/wdt.h>
#include <avr/eeprom.h>
#else
#include <avr/eeprom.h>          // sim shim
extern "C" void sim_reboot();    // sim: longjmp back to boot (sim.cpp)
#endif

void superos_setup();
void superos_loop();

static uint8_t g_fw = FW_SUPEROS;

uint8_t combined_current_fw() { return g_fw; }

void RebootTo(uint8_t fw) {
  eeprom_update_byte(EE_FW_SELECT, fw);
  menu::Exit();
#ifdef __AVR__
  wdt_enable(WDTO_15MS);
  for (;;) {}
#else
  sim_reboot();
#endif
}

void setup() {
#ifdef __AVR__
  // A watchdog reset (RebootTo) leaves the WDT armed at 15 ms — disarm first.
  MCUSR = 0;
  wdt_disable();
#endif
  hw::Init();
  midiclk::Begin();   // UART + saved sync/division settings (both firmwares)
  uint8_t sel = eeprom_read_byte(EE_FW_SELECT);
  g_fw = (sel == FW_EMU && emu101::available()) ? FW_EMU : FW_SUPEROS;
  if (menu::PowerOnComboHeld()) menu::Enter();
  if (g_fw == FW_EMU) emu101::setup(); else superos_setup();
}

void loop() {
  if (menu::Active()) { menu::Poll(); return; }
  if (g_fw == FW_EMU) emu101::loop(); else superos_loop();
}
