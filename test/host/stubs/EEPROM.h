#pragma once
#include <string.h>
#include <stdint.h>
struct HostEEPROM {
  uint8_t mem[4284];
  HostEEPROM() { memset(mem, 0xFF, sizeof(mem)); }
  template <typename T> void get(int addr, T& out) { memcpy(&out, mem + addr, sizeof(T)); }
  template <typename T> void put(int addr, const T& in) { memcpy(mem + addr, &in, sizeof(T)); }
};
extern HostEEPROM EEPROM;
