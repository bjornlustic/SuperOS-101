# SuperOS-101 host emulator. `make` builds, `make test` runs the self-test.
CXX      ?= c++
CXXFLAGS ?= -std=c++17 -O1 -Wall -Wextra

SRCS = sim.cpp ../src/main.cpp ../src/boot.cpp ../src/emu101.cpp
DEPS = shim/Arduino.h shim/avr/eeprom.h ../src/pins.h ../src/hw.h \
       ../src/controls.h ../src/engine.h ../src/combined.h ../src/menu.h \
       ../src/emu101.h ../src/emu101_hle.h ../src/mcs48.h ../src/rom101_data.h ../src/midi.h ../src/usbmidi_codec.h ../src/midiclock.h ../src/sysex.h ../src/flash_eeprom.h ../src/flash_persist.h ../src/cvdac.h

superos101-sim: $(SRCS) $(DEPS)
	$(CXX) $(CXXFLAGS) -DSUPEROS_SIM -I shim -I ../src $(SRCS) -o $@

../src/rom101_data.h: ../tools/mkrom.py
	python3 ../tools/mkrom.py

.PHONY: test clean
test: superos101-sim
	./superos101-sim --test

clean:
	rm -f superos101-sim sim-eeprom.bin sim-selftest-eeprom.bin
