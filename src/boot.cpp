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
#include <util/delay.h>
#else
#include <avr/eeprom.h>          // sim shim
extern "C" void sim_reboot();    // sim: longjmp back to boot (sim.cpp)
#endif

void superos_setup();
void superos_loop();

#if defined(__AVR__) && defined(USB_DISABLED)
// app-nousb: USB_DISABLED makes the Teensy core compile no USB code, but
// pins_teensy.c still calls usb_init() unconditionally. With the module
// never enabled, the AVR's USB controller stays in its reset state
// (detached, clock frozen), so VBUS events cannot disturb the loop.
extern "C" void usb_init(void) {}
#endif

static uint8_t g_fw = FW_SUPEROS;

uint8_t combined_current_fw() { return g_fw; }

// SysEx F0 7D 4A F7 (midiclock.h): jump to the boot section without a reset
// so GPIOR0 survives; the bootloader sees BOOT_MAGIC and stays resident.
void EnterBootloader() {
#ifdef __AVR__
  cli();
  UDCON |= (1 << DETACH);        // drop USB cleanly if the core brought it up
  _delay_ms(30);                 // host must register the detach first
  USBCON = 0;
  PLLCSR = 0;
  GPIOR0 = 0xB7;                 // BOOT_MAGIC, keep in sync with bootloader/bootload.c
  asm volatile("jmp 0x1F000");   // boot section (BOOTRST, BOOTSZ=01)
#else
  sim_reboot();                  // sim has no bootloader; behave like a reset
#endif
}

void RebootTo(uint8_t fw) {
  eeprom_update_byte(EE_FW_SELECT, fw);
  menu::Exit();

  // Everything off during the switch: gate off, portamento off, no LED lit
  // (HOLD LED off = P25 high, KT LED off = P26 low, function select 0), so
  // nothing stale shows while the other firmware comes up.
  hw::P2Write(P2_IDLE);
  BUS_PORT = 0xFF;

  // Input lockout (same as SuperOS-303): the finger that pressed LOAD/PLAY is
  // still on the button when the reset lands, and the other firmware would
  // boot with it held (the ROM enters step-LOAD mode, SuperOS toggles into
  // sequencer PLAY). Wait until every button and key has read released for
  // 250 ms straight; hard 4 s timeout so a stuck line cannot block the
  // switch (a floating P1 reads all-pressed).
  {
    const uint32_t t0 = millis();
    uint32_t idleSince = millis();
    for (;;) {
      bool any = hw::ScanRead(COL_BUTTONS) != 0;
      for (uint8_t bank = 0; bank < 4 && !any; ++bank)
        any = hw::ScanRead((ScanCol)(COL_KBD_BANK0 + bank)) != 0;
      BUS_PORT = 0xFF;
      const uint32_t now = millis();
      if (any) idleSince = now;
      else if (now - idleSince >= 250) break;
      if (now - t0 >= 4000) break;
      delayMicroseconds(2000);
    }
  }
#ifdef __AVR__
  // Hardware reset via watchdog: the bootloader wdt_disable()s on entry and
  // falls straight through to the app, which reads the new select byte.
  cli();
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
