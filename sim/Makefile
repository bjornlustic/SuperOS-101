# SuperOS-101 host emulator. `make` builds, `make test` runs the self-test.
CXX      ?= c++
CXXFLAGS ?= -std=c++17 -O1 -Wall -Wextra

SRCS = sim.cpp ../src/main.cpp ../src/boot.cpp ../src/emu101.cpp
DEPS = shim/Arduino.h shim/avr/eeprom.h ../src/pins.h ../src/hw.h \
       ../src/controls.h ../src/engine.h ../src/combined.h ../src/menu.h \
       ../src/emu101.h ../src/emu101_hle.h ../src/mcs48.h ../src/rom101_data.h ../src/midi.h ../src/usbmidi_codec.h ../src/midiclock.h ../src/sysex.h ../src/flash_eeprom.h ../src/flash_persist.h ../src/cvdac.h

# The sim carries the same build identity the firmware does, so the 0x34 reply
# is exercised with a populated SUPEROS_BUILD_REV rather than only its fallback.
# The self-test asserts the reply's SHAPE, never this value: make does not track
# flag changes, so a rebuild is not forced when the sha moves, and asserting the
# value would fail for a reason that has nothing to do with the firmware.
BUILD_REV := $(shell python3 ../tools/buildrev.py --print)

superos101-sim: $(SRCS) $(DEPS)
	$(CXX) $(CXXFLAGS) -DSUPEROS_SIM -DSUPEROS_BUILD_REV='"$(BUILD_REV)"' -I shim -I ../src $(SRCS) -o $@

../src/rom101_data.h: ../tools/mkrom.py
	python3 ../tools/mkrom.py

.PHONY: test clean
test: superos101-sim
	./superos101-sim --test

clean:
	rm -f superos101-sim sim-eeprom.bin sim-selftest-eeprom.bin
