/*
 * SuperOS-101 MIDI SysEx Bootloader (DIN serial + USB-MIDI)
 * Target: AT90USB1286 @ 16 MHz, boot section 0x1F000 (BOOTRST, BOOTSZ=01)
 *
 * Ported from SuperOS-303 (same MCU, same fuses, same protocol):
 *   F0 7D 01 <page-hi> <page-lo> <len-hi> <len-lo> <ck-hi> <ck-lo> <7bit data> F7
 *   F0 7D 02 F7                          run the app
 * The same stream is accepted from USART1 (DIN MIDI IN via the panel
 * breakout) or the polled USB-MIDI device in usb_boot.c.
 *
 * Entry: every reset lands here (BOOTRST) and jumps straight to the app
 * unless (a) the app parked BOOT_MAGIC in GPIOR0 and jumped here (SysEx
 * F0 7D 4A F7 in the app, no reset, so GPIOR0 survives), (b) LOAD + PLAY
 * are held at power-on (HOLD + KEY-TRANSPOSE is taken: it is the app's
 * config-menu combo, and the bootloader runs first), or (c) the app
 * section is blank.
 *
 * Board differences from the 303 (see src/pins.h):
 *   PORTC = BUS (DB0-7, active-low data + 4052 route + scan commons)
 *   PORTB = P1 scan rows (input, external pull-ups, 0 = pressed)
 *   PORTF = P2 (LED selects, GATE bit7 = 1 -> gate OFF)
 * Bootloader indicator: HOLD LED on (PF5 = 0 -> lit). Page writes cycle
 * the function-LED select (PF0-2, via IC8 4556).
 */

#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdint.h>

/* NO_USB (env:bootloader-nousb): DIN MIDI is the only transport; usb_boot.c
 * is excluded from the build and its entry points become no-ops. */
#ifdef NO_USB
#define usb_boot_init()
#define usb_boot_task()
#define usb_boot_detach()
#else
#include "usb_boot.h"
#endif

#define APP_ADDRESS 0x0000
#define PAGE_SIZE SPM_PAGESIZE
#define SYSEX_MAX (PAGE_SIZE*2)

/* App-requested bootloader entry: survives a jmp (no reset), and GPIOR0
 * resets to 0 on any real reset, so it cannot false-positive at power-on.
 * Keep in sync with the app's SysEx 0x4A handler (src/boot.cpp). */
#define BOOT_MAGIC 0xB7

/* P2 (PORTF) bits, mirrored from src/pins.h (this is C, no C++ headers) */
#define P2_PORTA_OFF 3
#define P2_LED_HOLD 5
#define P2_GATE 7

/* BUS scan column: DB5 low = function buttons; DB6/7 high = route 11
 * (disconnected), so the scan can never corrupt a held S/H CV. */
#define SCAN_BUTTONS 0xDF
/* P1 rows (PINB), 0 = pressed: LOAD = bit 0, PLAY = bit 1 */
#define BTN_LOAD_BIT 0
#define BTN_PLAY_BIT 1

/* Hardware enters the boot section at 0x1F000. This stub is pinned there by
 * the linker (.boot_entry); it establishes the C environment the app or
 * reset left behind and jumps to main. Built with -nostartfiles. */
int main(void);
__attribute__((naked, used, section(".boot_entry")))
void boot_entry(void) {
  asm volatile(
    "eor  r1, r1            \n\t"
    "ldi  r28, %0           \n\t"
    "ldi  r29, %1           \n\t"
    "out  __SP_L__, r28     \n\t"
    "out  __SP_H__, r29     \n\t"
    "jmp  main              \n\t"
    :: "M" (RAMEND & 0xFF), "M" (RAMEND >> 8));
}

static uint8_t page_buffer[PAGE_SIZE];
static uint8_t sysex_buf[SYSEX_MAX];

static volatile uint16_t sysex_index = 0;
static volatile uint8_t in_sysex = 0;

static void uart_init(void) {
  // 31250 baud @ 16 MHz -> UBRR = 31
  UBRR1H = 0;
  UBRR1L = 31;

  UCSR1A = 0;
  UCSR1B = (1 << RXEN1);                  // RX only
  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // 8N1
}

/* returns checksum of decoded bytes */
static uint8_t decode_7bit(uint8_t *in, uint16_t len, uint8_t *out) {
  uint16_t i = 0, o = 0;
  uint8_t check = 0;

  while (i < len) {
    uint8_t msb = in[i++];

    for (uint8_t b = 0; b < 7 && i < len; b++) {
      uint8_t v = in[i++];
      out[o] = v | (((msb >> b) & 1) << 7);
      check ^= out[o++];
      if (o >= PAGE_SIZE)
        return check;
    }
  }
  return check;
}

static void flash_write_page(uint16_t page) {
  uint32_t addr = (uint32_t)page * PAGE_SIZE;

  uint8_t sreg = SREG;
  cli();

  eeprom_busy_wait();

  boot_page_erase(addr);
  boot_spm_busy_wait();

  for (uint16_t i = 0; i < PAGE_SIZE; i += 2) {
    uint16_t w = (uint16_t)(page_buffer[i]) | ((uint16_t)(page_buffer[i + 1]) << 8);
    boot_page_fill(addr + i, w);
  }

  boot_page_write(addr);
  boot_spm_busy_wait();
  boot_rww_enable();

  SREG = sreg;
}

static void jump_to_app(void) {
  cli();

  // Returns to the caller when the app section is blank.
  if (pgm_read_word(APP_ADDRESS) != 0xFFFF) {
    usb_boot_detach(); // no-op unless USB was brought up
    ((void (*)(void))APP_ADDRESS)();
  }
}

static uint8_t count = 0;
static void process_sysex(uint8_t *data, uint16_t len) {
  if (len < 2)
    return;

  if (data[0] != 0x7D)
    return;

  uint8_t cmd = data[1];

  if (cmd == 0x01 && len > 9) {
    uint16_t page = ((uint16_t)data[2] << 7) | data[3];
    uint16_t packed_len = ((uint16_t)data[4] << 7) | data[5];
    uint8_t cksum = (data[6] << 4) | data[7];

    if (packed_len + 8 > len)
      return;

    // cycle the function-LED select (PF0-2) while writing
    PORTF = (PORTF & 0xF8) | (count++ & 0x07);

    // if they match, cksum will become zero
    cksum ^= decode_7bit(&data[8], packed_len, page_buffer);
    if (cksum) {
      // bad checksum: blink the HOLD LED forever (PF5 = 0 -> lit)
      while (1) {
        PORTF ^= (1 << P2_LED_HOLD);
        _delay_ms(200);
      }
    }
    flash_write_page(page);
  } else if (cmd == 0x02) {
    jump_to_app();
  }
}

// One byte of MIDI from either transport (USART1 or USB); non-static so
// usb_boot.c can feed it.
void midi_byte(uint8_t b) {
  if (b == 0xF0) {
    in_sysex = 1;
    sysex_index = 0;
    return;
  }

  if (!in_sysex)
    return;

  if (b == 0xF7) {
    process_sysex(sysex_buf, sysex_index);
    in_sysex = 0;
    return;
  }

  if (b < 0x80 && sysex_index < SYSEX_MAX)
    sysex_buf[sysex_index++] = b;
}

int main(void) {
  // After a watchdog reset, WDRF forces WDE on and wdt_disable() alone cannot
  // clear it. Stash the reset cause in GPIOR1 so the app can tell a watchdog
  // recovery (RebootTo uses the WDT) from a normal power-on.
  GPIOR1 = MCUSR;
  MCUSR = 0;
  wdt_disable();

  // -nostartfiles: .bss is never zeroed, so parser state is set explicitly
  // (this also covers entry by jump from the app, where RAM is app leftovers)
  sysex_index = 0;
  in_sysex = 0;
  count = 0;

  // BUS output, idle 0xFF (data 0 active-low, route 11); P1 input with
  // internal pull-ups (the original's "50k pullups" were inside the 80C49 —
  // the board has none); P2 output, gate OFF + portamento off.
  DDRC = 0xFF; PORTC = 0xFF;
  DDRB = 0x00; PORTB = 0xFF;
  DDRF = 0xFF; PORTF = (1 << P2_GATE) | (1 << P2_PORTA_OFF);
  uart_init();

  // app-requested entry (SysEx 0x4A jumped here with the magic parked)
  uint8_t stay = 0;
  if (GPIOR0 == BOOT_MAGIC) {
    GPIOR0 = 0;
    stay = 1;
  }

  // hold LOAD + PLAY at power-on to stay in the bootloader (pull-ups are
  // already on). Every other row must read released: a floating or
  // unplugged P1 reads all-pressed, which must never trap the unit here.
  PORTC = SCAN_BUTTONS;
  _delay_ms(1);
  {
    uint8_t combo = (1 << BTN_LOAD_BIT) | (1 << BTN_PLAY_BIT);
    if ((uint8_t)(PINB ^ combo) == 0xFF)   /* combo low, all others high */
      stay = 1;
  }
  PORTC = 0xFF;

  if (!stay)
    jump_to_app(); // returns only if the app section is blank

  // attach early so USB enumeration overlaps the LED show (control
  // requests just wait in the FIFO until the loop below services them)
  usb_boot_init();

  // indicate bootloader mode: HOLD LED on (PF5 = 0 -> lit), blink 4x
  for (uint8_t i = 0; i < 8; ++i) {
    PORTF ^= (1 << P2_LED_HOLD);
    _delay_ms(150);
  }
  PORTF &= (uint8_t)~(1 << P2_LED_HOLD);   // leave it lit

  while (1) {
    if (UCSR1A & (1 << RXC1))
      midi_byte(UDR1);
    usb_boot_task();
  }
}
