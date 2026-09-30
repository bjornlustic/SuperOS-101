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
#include <avr/sleep.h>
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

// Drop off the USB bus before a bootloader jump: the bootloader IS a new
// identity, so the host must register the detach and enumerate it cleanly.
// A firmware switch no longer detaches (SwitchTo below runs in process).
#ifdef __AVR__
static void UsbDetach() {
#if defined(SUPEROS_USB_MIDI)
  usb_gate_detach();
#elif !defined(USB_DISABLED)
  UDCON |= (1 << DETACH);        // drop USB cleanly if the core brought it up
  _delay_ms(30);                 // host must register the detach first
  USBCON = 0;
  PLLCSR = 0;
#endif
}
#endif

static uint8_t g_fw = FW_SUPEROS;
#if defined(__AVR__) && defined(SUPEROS_USB_MIDI)
uint32_t g_usbHoldUntil = 0;   // usb_gate.h bench hold-off
bool     g_usbBenchOff = false; // usb_gate.h: bench engine-off (SysEx 0x3E bit1)
#endif
uint16_t g_vccParkedMv = 0;    // sysex.h probe: VCC seen while USB was parked
sysex::VccSamples g_vccParkedSet = {0, 0, 0, 0};   // the 0x3E rail set
// Bench load experiment (SysEx 0x3E <mode> <s>, sysex.h): for s seconds run
// with bit0 core /2 (8 MHz), bit1 USB engine fully off (PLL stopped, clock
// frozen), bit2 idle-sleep at the end of every pass, bit3 core /1 (16 MHz,
// wins over bit0). The rail is sampled through the last second into
// g_vccParkedSet. Timekeeping runs off millis(), so a window at a clock other
// than the app's own runs correspondingly longer or shorter in real time, and
// hw.h's IdleWaitUs scales its S/H dwells from the compile-time F_CPU, so
// those dwells come out short at 8 and 16 MHz. Both are part of the load the
// window is there to create, not defects in it.
uint8_t  g_benchMode = 0;
uint8_t  g_benchApplied = 0;   // what is in force, for the probe to report
uint8_t  g_benchClkps = 0;     // set to the app's own divide in setup()
uint32_t g_benchUntil = 0;
#ifdef __AVR__
static uint8_t g_baseClkps = 0;
#endif

#ifdef __AVR__
// Boot counter that survives watchdog/brown-out resets (RAM is not cleared by
// a reset; .noinit is not zeroed by crt0). Reported by SysEx 0x3B (sysex.h):
// a bootloader round trip is exactly one boot, a firmware switch is ZERO (it
// runs in process since SwitchTo); anything more means the watchdog had to
// recover the unit. Magic guards power-on.
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
static bool     g_passSkip;    // exclude the next recorded pass from the max
uint16_t combined_pass_us(bool clearMax) {
  uint16_t m = g_passMaxUs; if (clearMax) g_passMaxUs = 0;
  return clearMax ? m : g_passLastUs;
}
// THE PROBE USED TO REPORT ITSELF, and it took a wrong G7 defect report to CORE
// to notice. The pass that answers a 0x3B sends 257 bytes through the BLOCKING
// SendSysex (HalWrite spins when the TX ring fills, midi.h:187), so it is by far
// the longest pass this firmware runs. combined_pass_us(true) clears the max on
// read, so every probe returned the cost of the PREVIOUS probe's own reply: a
// rock-steady 14768 +/- 16 us that reads exactly like a periodic hitch in the
// machine, on a unit that was serving as a capture control at the time.
//
// Clearing the max inside SendProbe does NOT fix it, and that was my first
// attempt: the duration is recorded at the TOP of the next loop(), not at the
// end of the pass it describes, so the probe's own pass is folded in after any
// clear SendProbe could do. It has to be suppressed at the point of record.
//
// This drops a cost that exists only because someone asked for telemetry. It
// hides no real hitch and it makes the question decidable: after this, a large
// max IS the machine.
void combined_pass_skip_next() { g_passSkip = true; }
#else
uint8_t combined_boot_count() { return 0; }
uint8_t combined_prev_stage() { return 0; }
static inline void Stage(uint8_t) {}
uint16_t combined_pass_us(bool) { return 0; }
void combined_pass_skip_next() {}
#endif

#ifdef __AVR__
ISR(USART1_RX_vect)  { midi::HalIsrRx(); }
ISR(USART1_UDRE_vect){ midi::HalIsrTx(); }
EMPTY_INTERRUPT(TIMER1_COMPA_vect)   // hw::IdleWaitUs wake alarm

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
  cvdac::Park();                 // CV DAC neutral: it holds this through the update
  GPIOR0 = 0xB7;                // BOOT_MAGIC, keep in sync with bootloader/bootload.c
  CLKPR = 0x80; CLKPR = 0x00;    // bootloader is built for the full 16 MHz
  asm volatile("jmp 0x1F000");   // boot section (BOOTRST, BOOTSZ=01)
#else
  sim_reboot();                  // sim has no bootloader; behave like a reset
#endif
}

// In-process firmware switch (see combined.h). Requested from inside the
// running firmware's own dispatch (SysEx handler, config menu), so it is only
// LATCHED here and performed at the top of the next loop() pass, where neither
// firmware is mid-update.
static uint8_t g_fwPending = 0xFF;
void superos_teardown();     // main.cpp: flush dirty patterns, release notes

void SwitchTo(uint8_t fw) { g_fwPending = fw; }

static void DoSwitch(uint8_t fw) {
  eeprom_update_byte(EE_FW_SELECT, fw);   // a power cycle keeps the choice
  menu::Exit();

  // The side being left cleans up after itself: dirty patterns / battery RAM
  // persist NOW (a switch must not lose edits), hanging MIDI notes end, and
  // the emulator's INT0 edge source goes quiet.
  if (g_fw == FW_EMU) emu101::teardown(); else superos_teardown();
  cvdac::Park();   // no accent frozen mid-decay into the emulator, which never writes the DAC

  // Everything off during the switch: gate off, portamento off, no LED lit
  // (HOLD LED off = P25 high, KT LED off = P26 low, function select 0), so
  // nothing stale shows while the other firmware comes up.
  hw::P2Write(P2_IDLE);
  BUS_PORT = 0xFF;

  // Input lockout (same as SuperOS-303): the finger that pressed LOAD/PLAY is
  // still on the button, and the other firmware would come up with it held
  // (the ROM enters step-LOAD mode, SuperOS toggles into sequencer PLAY).
  // Wait until every button and key has read released for 250 ms straight;
  // hard 4 s timeout so a stuck line cannot block the switch (a floating P1
  // reads all-pressed). USB stays alive throughout: control transfers are
  // ISR-driven and bulk data just waits in the endpoint banks.
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
  g_fw = (fw == FW_EMU && emu101::available()) ? FW_EMU : FW_SUPEROS;
  if (g_fw == FW_EMU) emu101::setup(); else superos_setup();
}

void setup() {
#ifdef __AVR__
  // The divide the app itself runs at, read from the hardware rather than
  // assumed (the Teensy core sets CLKPR from F_CPU before setup(), so /4 here).
  // The bench window restores exactly this; assuming /1 instead is what used to
  // leave the machine at 16 MHz, with the UART divisor still computed for
  // 4 MHz, after every /2 window. 16 MHz is the condition the divide exists to
  // avoid (platformio.ini [env:app]), so that leak was not cosmetic.
  g_baseClkps = CLKPR & 0x0F;
  g_benchClkps = g_baseClkps;
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
  // Timer1 is re-enabled by hw::Init() as the idle-sleep wake alarm.
  PRR0 = (1 << PRTWI) | (1 << PRTIM2) | (1 << PRTIM1) | (1 << PRSPI) | (1 << PRADC);
  PRR1 = (1 << PRTIM3);
#endif
  hw::Init();
  midi::Begin(sysex::LoadChannel());   // UART1 rings + ISRs (both firmwares)
  midi::SetThru(sysex::LoadThru());
  midi::SetClockFrom(sysex::LoadClockFrom());
  midi::SetNoteOut(sysex::LoadNoteOut());
  midiclk::Begin();                    // saved sync/division settings
  cvdac::Begin();                      // CV DAC neutral in both firmwares (the emulator never writes it)
  uint8_t sel = eeprom_read_byte(EE_FW_SELECT);
  g_fw = (sel == FW_EMU && emu101::available()) ? FW_EMU : FW_SUPEROS;
  if (menu::PowerOnComboHeld()) menu::Enter();
  if (g_fw == FW_EMU) emu101::setup(); else superos_setup();
  Stage(2);
#ifdef __AVR__
  // Prime the pass timer, or the FIRST recorded pass is the whole of setup().
  // g_passT0 is zero-initialised, so loop()'s first `micros() - g_passT0` is
  // time-since-boot, which saturates the uint16_t and makes every first probe
  // after a boot report a 65535 us max. Observed on the unit on each of three
  // reflashes today. That is a third reading in the same family as the two
  // probe-reports-itself defects, and it is the same failure to ask what the
  // instrument costs: 65535 reads as a 65 ms hitch and there is no hitch.
  // Skip the first pass as well as priming, because the interval from here to
  // the top of loop() is not a pass either.
  g_passT0 = micros();
  g_passSkip = true;
#endif
}

void loop() {
#ifdef __AVR__
  wdt_reset();       // pet the freeze-recovery watchdog (see setup)
  Stage(millis() > 1000 ? 4 : 3);
  {
    uint32_t now = micros();
    uint32_t d = now - g_passT0; g_passT0 = now;
    g_passLastUs = d > 0xFFFF ? 0xFFFF : (uint16_t)d;
    if (g_passSkip) g_passSkip = false;   // the pass that answered a probe
    else if (g_passLastUs > g_passMaxUs) g_passMaxUs = g_passLastUs;
  }
#endif
#if defined(__AVR__) && defined(SUPEROS_USB_MIDI)
  usb_vbus_gate();   // USB engine runs only while a cable is present
  if (g_usbHoldUntil && (int32_t)(millis() - (g_usbHoldUntil - 1000)) > 0 &&
      (int32_t)(millis() - g_usbHoldUntil) < 0)
    g_vccParkedMv = sysex::MeasureVccMv();   // last second of a hold-off
#endif
#ifdef __AVR__
  // Bench load experiment window (see g_benchMode).
  {
    const uint8_t base = g_baseClkps;   // the app's own divide (setup())
    static uint8_t applied = 0;
    static bool collecting = false;
    const bool active = g_benchUntil && (int32_t)(millis() - g_benchUntil) < 0;
    const uint8_t want = active ? g_benchMode : 0;
    if (want != applied) {
      const uint8_t clkps = sysex::BenchClkps(want, base);
      const uint32_t hz = sysex::BenchCoreHz(F_CPU, base, clkps);
      cli();
      if (clkps != g_benchClkps) {
        CLKPR = 0x80; CLKPR = clkps;
        UBRR1L = sysex::BenchUbrr(hz);
      }
      sei();
      g_benchClkps = clkps;
#ifdef SUPEROS_USB_MIDI
      g_usbBenchOff = (want & 2) != 0;   // gate parks the engine / brings it back
#endif
      applied = want;
      g_benchApplied = want;
    }
    if (active) {
      // Gather the rail set across the last second's passes. A full 256-sample
      // set is ~27 ms of ADC time; taking it in one burst would stall the pass
      // that refreshes the S/H caps and move the rail we are trying to read.
      if ((int32_t)(millis() - (g_benchUntil - 1000)) > 0) {
        if (!collecting) {
          g_vccParkedSet = sysex::VccSamples{0, 0, 0, 0};
          sysex::VccSampleBegin(sysex::BenchCoreHz(F_CPU, base, g_benchClkps));
          collecting = true;
        }
        if (g_vccParkedSet.n < 256) {
          sysex::VccSampleStep(g_vccParkedSet, 4);
          if (g_vccParkedSet.n >= 256) {
            sysex::VccSampleEnd();
            g_vccParkedMv = sysex::VccCodeToMv((uint16_t)(g_vccParkedSet.sum / g_vccParkedSet.n));
          }
        }
      }
    } else if (g_benchUntil) {
      if (collecting) { sysex::VccSampleEnd(); collecting = false; }
      g_benchUntil = 0;
    }
  }
#endif
  if (g_fwPending != 0xFF) { uint8_t fw = g_fwPending; g_fwPending = 0xFF; DoSwitch(fw); }
  if (menu::Active()) { menu::Poll(); return; }
  if (g_fw == FW_EMU) emu101::loop(); else superos_loop();
#ifdef __AVR__
  if (g_benchUntil && (g_benchMode & 4)) { set_sleep_mode(SLEEP_MODE_IDLE); sleep_mode(); }
#endif
}
