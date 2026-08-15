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
#include "midi.h"
#include "sysex.h"
#include "usb_gate.h"

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

// Drop off the USB bus (if this build has USB at all) before a reset or a
// bootloader jump: the host registers the detach, so the next identity
// (bootloader, or the app again after a firmware switch) enumerates cleanly
// instead of confusing the host with a device that changed under it.
static void UsbDetach() {
#if defined(__AVR__) && defined(SUPEROS_USB_MIDI)
  usb_gate_detach();
#elif defined(__AVR__) && !defined(USB_DISABLED)
  UDCON |= (1 << DETACH);        // drop USB cleanly if the core brought it up
  _delay_ms(30);                 // host must register the detach first
  USBCON = 0;
  PLLCSR = 0;
#endif
}

static uint8_t g_fw = FW_SUPEROS;
#if defined(__AVR__) && defined(SUPEROS_USB_MIDI)
uint32_t g_usbHoldUntil = 0;   // usb_gate.h bench hold-off
#endif
uint16_t g_vccParkedMv = 0;    // sysex.h probe: VCC seen while USB was parked

#ifdef __AVR__
// Boot counter that survives watchdog/brown-out resets (RAM is not cleared by
// a reset; .noinit is not zeroed by crt0). Reported by SysEx 0x3B (sysex.h):
// a firmware switch or bootloader round trip is exactly one boot; anything
// more means the watchdog had to recover the unit. Magic guards power-on.
static uint16_t g_bootMagic __attribute__((section(".noinit")));
static uint8_t  g_bootCount __attribute__((section(".noinit")));
// Boot-stage breadcrumb of the PREVIOUS boot (where it was when it ended):
// 1 setup entered, 2 setup done, 3 loop running, 4 loop ran > 1 s,
// 0x10 firmware switch, 0x20 bootloader entry. A watchdog recovery leaves
// the stage the hung boot last reached.
static uint8_t  g_stage __attribute__((section(".noinit")));
static uint8_t  g_prevStage __attribute__((section(".noinit")));
uint8_t combined_boot_count() { return g_bootCount; }
uint8_t combined_prev_stage() { return g_prevStage; }
static inline void Stage(uint8_t s) { g_stage = s; }
// Loop pass time (us): last pass, and the longest since the probe last read
// it. The S/H caps are refreshed once per pass, so pass length is pitch
// stability; the probe (SysEx 0x3B) reports both and clears the max.
static uint16_t g_passLastUs, g_passMaxUs;
static uint32_t g_passT0;
uint16_t combined_pass_us(bool clearMax) {
  uint16_t m = g_passMaxUs; if (clearMax) g_passMaxUs = 0;
  return clearMax ? m : g_passLastUs;
}
#else
uint8_t combined_boot_count() { return 0; }
uint8_t combined_prev_stage() { return 0; }
static inline void Stage(uint8_t) {}
uint16_t combined_pass_us(bool) { return 0; }
#endif

#ifdef __AVR__
ISR(USART1_RX_vect)  { midi::HalIsrRx(); }
ISR(USART1_UDRE_vect){ midi::HalIsrTx(); }

// Pre-main boot guard (ported from SuperOS-303 combined.cpp). The Teensy
// core's usb_init() runs BEFORE setup() and busy-waits FOREVER on USB PLL
// lock (`while (!(PLLCSR & PLOCK));`). If the PLL misses lock, seen on the
// 303 intermittently when a watchdog reboot landed while the previous
// firmware's USB/PLL was still running, the unit wedges dead until power-off.
// Arm a 2 s watchdog from .init3 (before crt0 finishes, long before
// usb_init) so ANY pre-setup hang retries the boot instead of freezing;
// setup() re-arms it as the permanent loop watchdog below. The bootloader is
// unaffected: it clears MCUSR and disables the WDT on entry, and this code
// only runs once it has jumped to the app.
extern "C" __attribute__((naked, used, section(".init3")))
void combined_boot_guard(void) {
  MCUSR = 0;
  wdt_enable(WDTO_2S);
}
#endif

uint8_t combined_current_fw() { return g_fw; }

// SysEx F0 7D 4A F7 (midiclock.h): jump to the boot section without a reset
// so GPIOR0 survives; the bootloader sees BOOT_MAGIC and stays resident.
void EnterBootloader() {
#ifdef __AVR__
  cli();
  Stage(0x20);
  UsbDetach();
  // Quiet the panel and the UART the same way the bootloader expects to find
  // them: gate off, no LED, BUS idle. The bootloader re-inits its own I/O.
  hw::P2Write(P2_IDLE);
  BUS_PORT = 0xFF;
  UCSR1B = 0;                    // stop the UART ISR sources before the jump
  GPIOR0 = 0xB7;                 // BOOT_MAGIC, keep in sync with bootloader/bootload.c
  asm volatile("jmp 0x1F000");   // boot section (BOOTRST, BOOTSZ=01)
#else
  sim_reboot();                  // sim has no bootloader; behave like a reset
#endif
}

void RebootTo(uint8_t fw) {
  Stage(0x10);
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
#ifdef __AVR__
      wdt_reset();
#endif
    }
  }
#ifdef __AVR__
  // Hardware reset via watchdog: the bootloader wdt_disable()s on entry and
  // falls straight through to the app, which reads the new select byte.
  cli();
  UsbDetach();
  wdt_enable(WDTO_15MS);
  for (;;) {}
#else
  sim_reboot();
#endif
}

void setup() {
#ifdef __AVR__
  // Permanent 8 s watchdog, petted every loop() pass (also from the switch
  // lockout in RebootTo). 303 root-cause evidence: after a fast firmware swap
  // the host's USB port can come up hosed and the CPU intermittently froze a
  // few seconds in, somewhere below our code (the USB core against the
  // pathological bus). The watchdog turns any such freeze into a recovery
  // reboot; the fresh attach afterwards always enumerated cleanly. 8 s clears
  // every legitimate stall here (flash-arena page writes are ms, the switch
  // lockout is 4 s max and pets). The bootloader (jump entry, SysEx 0x4A)
  // wdt_disable()s on its first lines.
  MCUSR = 0;
  wdt_enable(WDTO_8S);
  if (g_bootMagic != 0xB007) { g_bootMagic = 0xB007; g_bootCount = 0; g_stage = 0; }
  ++g_bootCount;
  g_prevStage = g_stage;
  Stage(1);
#endif
#if defined(__AVR__) && defined(SUPEROS_USB_MIDI)
  // The core's pre-main usb_init() left the USB engine live and attached;
  // park it now, before any panel I/O, and let the VBUS gate in loop()
  // (usb_gate.h) bring it up only with a cable in.
  usb_gate_begin();
#endif
#ifdef __AVR__
  // Every unused peripheral off: the SH-101's +5 V regulator is the DAC
  // reference and runs at its limit with this board, so each mA counts.
  // Timer0 stays (millis), USART1 stays (DIN MIDI), USB stays.
  PRR0 = (1 << PRTWI) | (1 << PRTIM2) | (1 << PRTIM1) | (1 << PRSPI) | (1 << PRADC);
  PRR1 = (1 << PRTIM3);
#endif
  hw::Init();
  midi::Begin(sysex::LoadChannel());   // UART1 rings + ISRs (both firmwares)
  midiclk::Begin();                    // saved sync/division settings
  uint8_t sel = eeprom_read_byte(EE_FW_SELECT);
  g_fw = (sel == FW_EMU && emu101::available()) ? FW_EMU : FW_SUPEROS;
  if (menu::PowerOnComboHeld()) menu::Enter();
  if (g_fw == FW_EMU) emu101::setup(); else superos_setup();
  Stage(2);
}

void loop() {
#ifdef __AVR__
  wdt_reset();       // pet the freeze-recovery watchdog (see setup)
  Stage(millis() > 1000 ? 4 : 3);
  {
    uint32_t now = micros();
    uint32_t d = now - g_passT0; g_passT0 = now;
    g_passLastUs = d > 0xFFFF ? 0xFFFF : (uint16_t)d;
    if (g_passLastUs > g_passMaxUs) g_passMaxUs = g_passLastUs;
  }
#endif
#if defined(__AVR__) && defined(SUPEROS_USB_MIDI)
  usb_vbus_gate();   // USB engine runs only while a cable is present
  if (g_usbHoldUntil && (int32_t)(millis() - (g_usbHoldUntil - 1000)) > 0 &&
      (int32_t)(millis() - g_usbHoldUntil) < 0)
    g_vccParkedMv = sysex::MeasureVccMv();   // last second of a hold-off
#endif
  if (menu::Active()) { menu::Poll(); return; }
  if (g_fw == FW_EMU) emu101::loop(); else superos_loop();
}
