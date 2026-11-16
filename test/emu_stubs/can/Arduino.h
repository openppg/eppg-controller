// Arduino surface used by esc.cpp, the SINE-ESC-CAN CanardAdapter, the
// Adafruit MCP2515 driver and BMS_CAN, for the CAN emulation tests. Time comes
// from the virtual clock in emu_clock.h; serial output is discarded.
#pragma once

#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <string>

#include "emu_clock.h"
#include "freertos/FreeRTOS.h"

typedef uint8_t byte;

#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x01
#define OUTPUT 0x03
#define FALLING 0x02
#define DEC 10
#define HEX 16

inline unsigned long micros() { return static_cast<unsigned long>(emuMicrosRef()); }
inline unsigned long millis() { return static_cast<unsigned long>(emuMicrosRef() / 1000ULL); }
inline void delay(uint32_t ms) { emuAdvanceMillis(ms); }
inline void delayMicroseconds(uint32_t us) { emuAdvanceMicros(us); }
inline void yield() {}

inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return HIGH; }
inline int digitalPinToInterrupt(int pin) { return pin; }
inline void attachInterrupt(int, void (*)(void), int) {}
inline void detachInterrupt(int) {}

class String {
 public:
  String() = default;
  String(const char* s) : s_(s ? s : "") {}  // NOLINT(runtime/explicit)
  String& operator+=(const char* s) {
    s_ += s ? s : "";
    return *this;
  }
  String& operator+=(const String& o) {
    s_ += o.s_;
    return *this;
  }
  bool operator==(const char* s) const { return s_ == (s ? s : ""); }
  const char* c_str() const { return s_.c_str(); }
  size_t length() const { return s_.size(); }

 private:
  std::string s_;
};

class Print {
 public:
  virtual ~Print() = default;
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(const uint8_t* buffer, size_t size) {
    size_t n = 0;
    while (size--) n += write(*buffer++);
    return n;
  }
  template <typename T>
  size_t print(const T&, int = DEC) { return 0; }
  template <typename T>
  size_t println(const T&, int = DEC) { return 0; }
  size_t println() { return 0; }
  size_t printf(const char*, ...) { return 0; }
};

class Stream : public Print {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
  virtual void flush() {}
};

class EmuSerial : public Stream {
 public:
  size_t write(uint8_t) override { return 1; }
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
};

inline EmuSerial Serial;
inline EmuSerial USBSerial;
