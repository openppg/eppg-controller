// Arduino stub for the OTA emulation. Shadows test/native_stubs/Arduino.h
// (it comes first on the include path) to give a virtual clock, a captured
// USBSerial and a recording ESP.restart().
#pragma once

#include <stdint.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>

#include "emu_ota_harness.h"

#define EMU_OTA_ARDUINO_STUB 1

inline unsigned long millis() { return emu::nowMs; }
inline void delay(unsigned long ms) { emu::advanceMs(ms); }

class EmuSerial {
 public:
  void begin(unsigned long) {}
  void flush() {}
  void print(const char* s) { emu::serialLog.push_back(s ? s : ""); }
  void println(const char* s) { print(s); }
  void println() { print(""); }
  void printf(const char* fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    emu::serialLog.push_back(buf);
  }
};

inline EmuSerial USBSerial;
inline EmuSerial Serial;

class EspClass {
 public:
  void restart() {
    ++emu::restartCount;
    emu::restartAtMs = emu::nowMs;
    emu::recordEvent("restart");
  }
};

inline EspClass ESP;
