// SuperOS-101 emulator — avr/eeprom.h shim (host build only)
//
// EEPROM is a 4 KB byte array persisted to a file (default ./sim-eeprom.bin,
// override with SIM_EEPROM env var), so sequencer save/load survives runs just
// like the real chip.
#pragma once
#include <stdint.h>

uint8_t eeprom_read_byte(const uint8_t* addr);
void    eeprom_update_byte(uint8_t* addr, uint8_t value);
