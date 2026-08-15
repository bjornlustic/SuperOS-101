/*
 * sendsyx — send a .syx file to a CoreMIDI destination (macOS).
 *
 * Part of the SuperOS-101 update workflow: put the unit in the SysEx
 * bootloader (LOAD+PLAY at power-on, or SysEx F0 7D 4A F7 to the app),
 * then:  sendsyx list
 *        sendsyx "SuperOS-101 Bootloader" app-update.syx
 *
 * Build: clang -O2 -o sendsyx sendsyx.c -framework CoreMIDI -framework CoreFoundation
 *
 * USB-MIDI flow control does the pacing (the bootloader NAKs while a flash
 * page writes), so the file is sent as one MIDISendSysex request.
 */
#include <CoreMIDI/CoreMIDI.h>
#include <CoreFoundation/CoreFoundation.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

static void dest_name(MIDIEndpointRef ep, char *out, size_t n) {
  CFStringRef s = NULL;
  out[0] = 0;
  if (MIDIObjectGetStringProperty(ep, kMIDIPropertyDisplayName, &s) == noErr && s) {
    CFStringGetCString(s, out, (CFIndex)n, kCFStringEncodingUTF8);
    CFRelease(s);
  }
}

static volatile int g_done = 0;
static void sysex_done(MIDISysexSendRequest *req) { (void)req; g_done = 1; }

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: sendsyx list | sendsyx <dest-substring> <file.syx>\n");
    return 2;
  }

  MIDIClientRef client;
  if (MIDIClientCreate(CFSTR("sendsyx"), NULL, NULL, &client) != noErr) {
    fprintf(stderr, "sendsyx: MIDIClientCreate failed\n");
    return 1;
  }

  ItemCount n = MIDIGetNumberOfDestinations();
  if (!strcmp(argv[1], "list")) {
    for (ItemCount i = 0; i < n; i++) {
      char name[256];
      dest_name(MIDIGetDestination(i), name, sizeof name);
      printf("%lu: %s\n", (unsigned long)i, name);
    }
    return 0;
  }

  if (argc < 3) {
    fprintf(stderr, "usage: sendsyx <dest-substring> <file.syx>\n");
    return 2;
  }

  MIDIEndpointRef dest = 0;
  char name[256];
  for (ItemCount i = 0; i < n; i++) {
    MIDIEndpointRef ep = MIDIGetDestination(i);
    dest_name(ep, name, sizeof name);
    if (strcasestr(name, argv[1])) { dest = ep; break; }
  }
  if (!dest) {
    fprintf(stderr, "sendsyx: no destination matching \"%s\" (try: sendsyx list)\n", argv[1]);
    return 1;
  }

  FILE *f = fopen(argv[2], "rb");
  if (!f) { perror(argv[2]); return 1; }
  fseek(f, 0, SEEK_END);
  long len = ftell(f);
  fseek(f, 0, SEEK_SET);
  unsigned char *buf = malloc((size_t)len);
  if (fread(buf, 1, (size_t)len, f) != (size_t)len) {
    fprintf(stderr, "sendsyx: short read\n");
    return 1;
  }
  fclose(f);

  printf("sending %ld bytes to \"%s\"...\n", len, name);
  /* Send one F0..F7 message at a time with a gap between messages: the SysEx
   * bootloader spends ~10 ms in SPM after each page with the UART unread, so a
   * back-to-back stream (as MIDISendSysex would produce for a whole file) loses
   * the head of the next page. Files without F0 (raw channel bytes) go as one
   * chunk. */
  long pos = 0;
  int nmsg = 0;
  while (pos < len) {
    long end = pos;
    if (buf[pos] == 0xF0) {
      while (end < len && buf[end] != 0xF7) end++;
      if (end < len) end++;            /* include F7 */
    } else {
      end = len;
    }
    g_done = 0;
    MIDISysexSendRequest req = {
      .destination = dest,
      .data = buf + pos,
      .bytesToSend = (UInt32)(end - pos),
      .complete = false,
      .completionProc = sysex_done,
      .completionRefCon = NULL,
    };
    if (MIDISendSysex(&req) != noErr) {
      fprintf(stderr, "sendsyx: MIDISendSysex failed\n");
      return 1;
    }
    while (!g_done)
      CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.05, false);
    if (req.bytesToSend) { fprintf(stderr, "sendsyx: %u bytes unsent\n", (unsigned)req.bytesToSend); return 1; }
    nmsg++;
    pos = end;
    if (pos < len) usleep(60000);   /* 60 ms gap */
  }
  printf("done (%d message%s)\n", nmsg, nmsg == 1 ? "" : "s");
  return 0;
}
