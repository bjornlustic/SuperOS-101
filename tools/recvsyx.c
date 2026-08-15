/*
 * recvsyx: monitor SysEx from a CoreMIDI source (macOS) and decode the
 * SuperOS-101 diag firmware's telemetry.
 *
 * Part of the bring-up workflow: the env:diag build streams its boot
 * electrical self-test every 2 s and probe-mode loopback events as
 *   F0 7D 44 <68 nibbles>  F7   (self-test report:
 *       allLow allHigh rdLow[0..7] rdHigh[0..7] rise[0..7] fall[0..7])
 *   F0 7D 45 <2 nibbles>   F7   (loopback row mask, 0 = lost)
 *
 * Build: clang -O2 -o recvsyx recvsyx.c -framework CoreMIDI -framework CoreFoundation
 * Usage: recvsyx list | recvsyx [source-substring]
 *        (no argument: connect to every source)
 */
#include <CoreMIDI/CoreMIDI.h>
#include <CoreFoundation/CoreFoundation.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void src_name(MIDIEndpointRef ep, char *out, size_t n) {
  CFStringRef s = NULL;
  out[0] = 0;
  if (MIDIObjectGetStringProperty(ep, kMIDIPropertyDisplayName, &s) == noErr && s) {
    CFStringGetCString(s, out, (CFIndex)n, kCFStringEncodingUTF8);
    CFRelease(s);
  }
}

/* ---- diag decode ---------------------------------------------------------- */

/* 80C49 socket pin for each DB line (DB0..DB7 = pins 12..19). */
static const int db_socket_pin[8] = {12, 13, 14, 15, 16, 17, 18, 19};
/* 80C49 socket pin for each P1 row (P10..P17 = pins 27..34). */
static const int p1_socket_pin[8] = {27, 28, 29, 30, 31, 32, 33, 34};

static void print_mask(const char *label, unsigned mask) {
  printf("  %s:", label);
  if (!mask) { printf(" none\n"); return; }
  for (int i = 0; i < 8; i++)
    if (mask & (1u << i)) printf(" DB%d(pin %d)", i, db_socket_pin[i]);
  printf("\n");
}

static void decode_report(const unsigned char *d, size_t n) {
  if (n != 88) { printf("  (bad report length %zu, want 88 nibbles)\n", n); return; }
  unsigned char b[44];
  for (int i = 0; i < 44; i++) b[i] = (unsigned char)((d[i * 2] << 4) | (d[i * 2 + 1] & 0x0F));

  unsigned bootPinc = b[0], mcusr = b[1], allLow = b[2], allHigh = b[3];
  const unsigned char *rdLow = b + 4, *rdHigh = b + 12;
  const unsigned char *rise = b + 20, *fall = b + 28, *floatRise = b + 36;

  printf("reset cause: %02X (%s%s%s%s%s)\n", mcusr,
         mcusr & 0x01 ? "power-on " : "",
         mcusr & 0x02 ? "external/ISP " : "",
         mcusr & 0x04 ? "BROWN-OUT " : "",
         mcusr & 0x08 ? "watchdog " : "",
         mcusr ? "" : "none recorded");

  printf("boot sample (pins tri-state, before anything driven): PINC=%02X  DB6=%u DB7=%u\n",
         bootPinc, (bootPinc >> 6) & 1, (bootPinc >> 7) & 1);
  printf("  only meaningful after a REAL power-off (a programmer reset keeps the\n"
         "  net charged). After >= 60 s off: DB7=0 here means the net floats (no\n"
         "  external hold: PC7's low-side driver is dead, fault is the AVR pin);\n"
         "  DB7=1 after a long off means the net is held high externally.\n");

  unsigned failLow = 0, failHigh = 0;
  for (int i = 0; i < 8; i++) {
    if (rdLow[i] & (1u << i)) failLow |= 1u << i;
    if (!(rdHigh[i] & (1u << i))) failHigh |= 1u << i;
  }

  printf("diag self-test report\n");
  print_mask("stuck (driven LOW, read HIGH) ", failLow);
  print_mask("stuck (driven HIGH, read LOW) ", failHigh);

  /* localize each stuck pin: rail short vs bridge to another bus pin */
  for (int i = 0; i < 8; i++) {
    unsigned m = 1u << i;
    if (failLow & m) {
      if (allLow & m)
        printf("  -> DB%d stays HIGH even with the whole port driven low:\n"
               "     tied to +5V or an external high net (NOT a bus-pin bridge).\n"
               "     Could be a solder short to a 5V pad on the CPU board, or a\n"
               "     damaged mainboard input (e.g. 4052 protection diode) holding\n"
               "     the net at the rail.\n", i);
      else
        printf("  -> DB%d goes LOW when the whole port is driven low: it was\n"
               "     fighting another bus pin's driver, i.e. a solder bridge\n"
               "     between two bus lines. See the follower analysis below.\n", i);
    }
    if (failHigh & m) {
      if (!(allHigh & m))
        printf("  -> DB%d stays LOW even with the whole port driven high:\n"
               "     tied to GND or an external low net.\n", i);
    }
  }

  /* follower analysis: driving one pin drags a different (driven) pin along.
   * Pins already known stuck are excluded: they "follow" everything. */
  unsigned stuck = failLow | failHigh;
  for (int i = 0; i < 8; i++) {
    unsigned m = 1u << i;
    if (stuck & m) continue;
    unsigned followLow  = (unsigned)(~rdLow[i]) & ~m & ~stuck & 0xFF;
    unsigned followHigh = (unsigned)( rdHigh[i]) & ~m & ~stuck & 0xFF;
    if (followLow)
      for (int q = 0; q < 8; q++)
        if (followLow & (1u << q))
          printf("  -> DB%d low drags DB%d low: bridge DB%d-DB%d suspected\n", i, q, i, q);
    if (followHigh)
      for (int q = 0; q < 8; q++)
        if (followHigh & (1u << q))
          printf("  -> DB%d high drags DB%d high: bridge DB%d-DB%d suspected\n", i, q, i, q);
  }

  printf("  pin        socket  rise  fall  floatRise  (loop counts; 255 = timeout)\n");
  for (int i = 0; i < 8; i++)
    printf("  DB%d %s   %2d     %3u   %3u     %3u\n",
           i, i >= 6 ? "(route)" : "       ", db_socket_pin[i], rise[i], fall[i], floatRise[i]);
  printf("  floatRise: pin driven low then released with NO pull-up; only an\n"
         "  external (mainboard) source can lift it back. Finite value on DB6 =\n"
         "  the select nets carry an external pull-up; a weak pull-up cannot\n"
         "  defeat the AVR's driver, so DB7 failing readback then means PC7's\n"
         "  driver is dead. 255 on DB6 = no external pull on the select nets,\n"
         "  making DB7's held-high state a hard external short instead.\n");
  printf("  reading it: rise is how long the released pad took to rise through\n"
         "  its pull-up (measures attached capacitance/load), fall is how fast\n"
         "  an external DC load dragged the released pad down (255 = none).\n"
         "  If DB6/DB7 rise in ~0-2 counts while DB0-5 take clearly longer,\n"
         "  those pads are likely floating: open joint on the CPU board\n"
         "  (TQFP pin 41/42 or edge-header pin 18/19). If DB6/DB7 look like\n"
         "  the others, the wiring is probably intact; confirm at the socket\n"
         "  with the probe-mode wire test, then suspect the mainboard 4052.\n");
}

static void decode_duel(const unsigned char *d, size_t n) {
  if (n != 4) { printf("  (bad duel length %zu)\n", n); return; }
  unsigned a = (unsigned)((d[0] << 4) | (d[1] & 0x0F));
  unsigned b = (unsigned)((d[2] << 4) | (d[3] & 0x0F));
  printf("driver duel (jumper socket pin 18 <-> 19 required)\n");
  printf("  A: PC7 floated, PC6 driven LOW : PINC=%02X  DB6=%u DB7=%u\n",
         a, (a >> 6) & 1, (a >> 7) & 1);
  printf("  B: PC6 floated, PC7 driven LOW : PINC=%02X  DB6=%u DB7=%u\n",
         b, (b >> 6) & 1, (b >> 7) & 1);
  if (!((a >> 7) & 1))
    printf("  verdict: DB6's driver pulled the DB7 net LOW. The net is fine;\n"
           "  PC7's own output driver is dead. Fault is the AVR pin (TQFP 42):\n"
           "  rework/replace the chip, or bodge DB7 onto the adjacent free PE2\n"
           "  pad (TQFP 43) with a deliberate solder bridge + firmware change.\n");
  else if ((a >> 6) & 1)
    printf("  verdict: even DB6's good driver could not pull the net low: the\n"
           "  DB7 net is held hard at +5V externally. Inspect the PC7 trace on\n"
           "  the CPU board for a short to 5V, then the SH-101 mainboard side\n"
           "  (4052 select input / level shifter sourcing into the net).\n");
  else
    printf("  verdict: DB6 went low but DB7 did not follow: the jumper was not\n"
           "  making contact (or there is an open between a pad and its socket\n"
           "  pin). Press the wire firmly on both pins and press PLAY again.\n");
}

static void decode_event(const unsigned char *d, size_t n) {
  if (n != 2) { printf("  (bad event length %zu)\n", n); return; }
  unsigned mask = (unsigned)((d[0] << 4) | (d[1] & 0x0F));
  if (!mask) { printf("loopback LOST\n"); return; }
  printf("loopback DETECTED on");
  for (int i = 0; i < 8; i++)
    if (mask & (1u << i)) printf(" P1%d(socket pin %d)", i, p1_socket_pin[i]);
  printf(": the square wave reaches the socket. CPU board side OK.\n");
}

/* ---- sysex reassembly ----------------------------------------------------- */

static unsigned char g_buf[1024];
static size_t g_len;
static int g_in_sysex;

static void handle_sysex(const unsigned char *msg, size_t len) {
  printf("[%zu bytes]", len);
  for (size_t i = 0; i < len && i < 64; i++) printf(" %02X", msg[i]);
  if (len > 64) printf(" ...");
  printf("\n");
  if (len >= 3 && msg[0] == 0xF0 && msg[1] == 0x7D) {
    if (msg[2] == 0x44) decode_report(msg + 3, len - 4);
    if (msg[2] == 0x45) decode_event(msg + 3, len - 4);
    if (msg[2] == 0x46) decode_duel(msg + 3, len - 4);
  }
  fflush(stdout);
}

static void read_proc(const MIDIPacketList *pl, void *ref, void *src) {
  (void)ref; (void)src;
  const MIDIPacket *p = &pl->packet[0];
  for (UInt32 i = 0; i < pl->numPackets; i++) {
    for (UInt16 j = 0; j < p->length; j++) {
      unsigned char c = p->data[j];
      if (c >= 0xF8) continue;                       /* realtime: ignore */
      if (c == 0xF0) { g_in_sysex = 1; g_len = 0; }
      if (!g_in_sysex) {                             /* channel messages: one line each */
        static unsigned char st = 0, d[2]; static int nd = 0;
        if (c & 0x80) { st = c; nd = 0; }
        else if (st) {
          d[nd++] = c;
          int need = ((st & 0xF0) == 0xC0 || (st & 0xF0) == 0xD0) ? 1 : 2;
          if (nd >= need) {
            if (need == 2) printf("[midi] %02X %02X %02X\n", st, d[0], d[1]);
            else           printf("[midi] %02X %02X\n", st, d[0]);
            fflush(stdout); nd = 0;
          }
        }
        continue;
      }
      if (g_len < sizeof g_buf) g_buf[g_len++] = c;
      if (c == 0xF7) { g_in_sysex = 0; handle_sysex(g_buf, g_len); }
      else if (c >= 0x80 && c != 0xF0) g_in_sysex = 0;  /* interrupted */
    }
    p = MIDIPacketNext(p);
  }
}

int main(int argc, char **argv) {
  MIDIClientRef client;
  if (MIDIClientCreate(CFSTR("recvsyx"), NULL, NULL, &client) != noErr) {
    fprintf(stderr, "recvsyx: MIDIClientCreate failed\n");
    return 1;
  }

  ItemCount n = MIDIGetNumberOfSources();
  if (argc > 1 && !strcmp(argv[1], "list")) {
    for (ItemCount i = 0; i < n; i++) {
      char name[256];
      src_name(MIDIGetSource(i), name, sizeof name);
      printf("%lu: %s\n", (unsigned long)i, name);
    }
    return 0;
  }

  MIDIPortRef port;
  if (MIDIInputPortCreate(client, CFSTR("in"), read_proc, NULL, &port) != noErr) {
    fprintf(stderr, "recvsyx: MIDIInputPortCreate failed\n");
    return 1;
  }

  int connected = 0;
  for (ItemCount i = 0; i < n; i++) {
    MIDIEndpointRef ep = MIDIGetSource(i);
    char name[256];
    src_name(ep, name, sizeof name);
    if (argc > 1 && !strcasestr(name, argv[1])) continue;
    if (MIDIPortConnectSource(port, ep, NULL) == noErr) {
      printf("listening on \"%s\"\n", name);
      connected++;
    }
  }
  if (!connected) {
    fprintf(stderr, "recvsyx: no matching MIDI source (try: recvsyx list)\n");
    return 1;
  }
  fflush(stdout);
  CFRunLoopRun();
  return 0;
}
