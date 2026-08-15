#!/bin/sh
# Build and flash the SuperOS-101 over ISP (Atmel-ICE on the Tag-Connect pads).
#   tools/flash.sh            app + bootloader (USB-MIDI + DIN)
#   tools/flash.sh nousb      app-nousb + bootloader-nousb (DIN only, for a
#                             board whose USB-C path is unusable)
#   tools/flash.sh diag       diagnostic firmware only (no bootloader)
#   tools/flash.sh fuses      burn the fuses (once per chip)
# Order matters: the app is written with a chip erase (-e), the bootloader
# afterwards without one (-D), otherwise the second write wipes the first.
set -e
cd "$(dirname "$0")/.."
PROG="avrdude -c atmelice_isp -p at90usb1286"
case "${1:-app}" in
  fuses)
    pio run -e fuses -t upload ;;
  diag)
    pio run -e diag
    $PROG -B 4 -e -U flash:w:.pio/build/diag/firmware.hex:i ;;
  nousb)
    pio run -e app-nousb -e bootloader-nousb
    $PROG -B 4 -e -U flash:w:.pio/build/app-nousb/firmware.hex:i
    $PROG -B 4 -D -U flash:w:.pio/build/bootloader-nousb/firmware.hex:i ;;
  app|*)
    pio run -e app -e bootloader
    $PROG -B 4 -e -U flash:w:.pio/build/app/firmware.hex:i
    $PROG -B 4 -D -U flash:w:.pio/build/bootloader/firmware.hex:i ;;
esac
