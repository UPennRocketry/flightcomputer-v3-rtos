// Host stub of the Teensy Arduino core: just enough to compile the
// hardware-independent flight modules on a PC for tests.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

namespace arduino { enum { LOW = 0, HIGH = 1, INPUT = 0, OUTPUT = 1, MSBFIRST = 1 }; }

extern uint32_t g_host_millis;
inline uint32_t millis() { return g_host_millis; }
inline uint32_t micros() { return g_host_millis * 1000UL; }
inline void delay(uint32_t ms) { g_host_millis += ms; }
inline void delayMicroseconds(uint32_t) {}

extern int g_host_pin_state[64];
inline void pinMode(uint8_t, int) {}
inline void digitalWrite(uint8_t pin, int v) { if (pin < 64) g_host_pin_state[pin] = v; }

struct HostSerial {
  void begin(unsigned long) {}
  void print(const char* s) { fputs(s, stdout); }
  void println(const char* s) { puts(s); }
  void printf(const char* f, ...) { va_list a; va_start(a, f); vprintf(f, a); va_end(a); }
  int available() { return 0; }
  int read() { return -1; }
};
extern HostSerial Serial;

// i.MX RT registers touched by the firmware.
extern uint32_t g_host_SRC_SRSR;
extern uint16_t g_host_WDOG1_WCR;
extern uint16_t g_host_WDOG1_WSR;
#define SRC_SRSR g_host_SRC_SRSR
#define WDOG1_WCR g_host_WDOG1_WCR
#define WDOG1_WSR g_host_WDOG1_WSR
