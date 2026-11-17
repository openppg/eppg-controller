// Host stub of the Arduino-ESP32 2.0.17 core API used by the settings / QC
// code: String, Print/Stream, a capturing USBSerial, virtual-clock
// millis()/delay(), hookable digitalRead()/analogRead(), ESP.restart().
#pragma once

#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <functional>
#include <string>
#include <type_traits>

#include "emu_clock.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

using std::max;
using std::min;

#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x01
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05
#define DEC 10
#define HEX 16

#ifndef PROGMEM
#define PROGMEM
#endif
#define F(string_literal) (string_literal)

#define log_e(...) ((void)0)
#define log_w(...) ((void)0)
#define log_i(...) ((void)0)
#define log_d(...) ((void)0)
#define log_v(...) ((void)0)

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

typedef uint16_t word;

inline uint8_t pgm_read_byte(const void* p) { return *reinterpret_cast<const uint8_t*>(p); }

// map() exactly as cores/esp32/WMath.cpp (2.0.17).
inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
  const long run = in_max - in_min;
  if (run == 0) {
    return -1;
  }
  const long rise = out_max - out_min;
  const long delta = x - in_min;
  return (delta * rise) / run + out_min;
}

inline unsigned long millis() { return emuMillisRef(); }
inline unsigned long micros() { return emuMillisRef() * 1000UL; }
inline void delay(uint32_t ms) { emuAdvanceMillis(ms); }
inline void yield() {}

// --- String -----------------------------------------------------------------

class String {
 public:
  String() = default;
  String(const char* s) : data_(s ? s : "") {}  // NOLINT(runtime/explicit)
  String(const std::string& s) : data_(s) {}    // NOLINT(runtime/explicit)

  bool concat(char c) {
    data_ += c;
    return true;
  }
  bool concat(const char* s) {
    data_ += s ? s : "";
    return true;
  }
  const char* c_str() const { return data_.c_str(); }
  size_t length() const { return data_.size(); }
  bool operator==(const String& o) const { return data_ == o.data_; }
  bool operator==(const char* o) const { return data_ == (o ? o : ""); }
  bool operator!=(const String& o) const { return !(*this == o); }
  bool operator!=(const char* o) const { return !(*this == o); }

 private:
  std::string data_;
};

// --- Print / Stream ---------------------------------------------------------

struct __FlashStringHelper;

class Print {
 public:
  virtual ~Print() = default;
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(const uint8_t* buffer, size_t size) {
    size_t n = 0;
    while (size--) n += write(*buffer++);
    return n;
  }
  size_t write(const char* s) {
    return s ? write(reinterpret_cast<const uint8_t*>(s), strlen(s)) : 0;
  }
  size_t write(const char* buffer, size_t size) {
    return write(reinterpret_cast<const uint8_t*>(buffer), size);
  }

  size_t print(const char* s) { return write(s); }
  size_t print(const String& s) { return write(s.c_str()); }
  size_t print(char c) { return write(static_cast<uint8_t>(c)); }
  template <typename T, typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
  size_t print(T v, int base = DEC) {
    char buf[40];
    if (base == HEX) {
      snprintf(buf, sizeof(buf), "%llX", static_cast<unsigned long long>(v));
    } else if (std::is_signed<T>::value) {
      snprintf(buf, sizeof(buf), "%lld", static_cast<long long>(v));
    } else {
      snprintf(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(v));
    }
    return write(buf);
  }
  template <typename T, typename std::enable_if<std::is_floating_point<T>::value, int>::type = 0>
  size_t print(T v, int digits = 2) {
    char buf[48];
    snprintf(buf, sizeof(buf), "%.*f", digits, static_cast<double>(v));
    return write(buf);
  }

  size_t println() { return write("\r\n"); }
  template <typename T>
  size_t println(const T& v) {
    size_t n = print(v);
    return n + println();
  }
  template <typename T>
  size_t println(const T& v, int fmt) {
    size_t n = print(v, fmt);
    return n + println();
  }

  size_t printf(const char* fmt, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return 0;
    return write(buf);
  }
};

class Printable {
 public:
  virtual ~Printable() = default;
  virtual size_t printTo(Print& p) const = 0;
};

class Stream : public Print {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() { return -1; }
  virtual void flush() {}
  size_t readBytes(char* buffer, size_t length) {
    size_t n = 0;
    while (n < length && available() > 0) buffer[n++] = static_cast<char>(read());
    return n;
  }
};

// Captures everything written; read()/available() serve a scripted input.
class EmuSerial : public Stream {
 public:
  void begin(unsigned long) {}
  void end() {}
  operator bool() const { return true; }

  size_t write(uint8_t c) override {
    out_ += static_cast<char>(c);
    return 1;
  }
  using Print::write;

  int available() override { return static_cast<int>(in_.size() - inPos_); }
  int read() override { return inPos_ < in_.size() ? static_cast<uint8_t>(in_[inPos_++]) : -1; }
  int peek() override { return inPos_ < in_.size() ? static_cast<uint8_t>(in_[inPos_]) : -1; }

  // Test side
  const std::string& output() const { return out_; }
  void clearOutput() { out_.clear(); }
  void feed(const std::string& s) { in_ += s; }
  void reset() {
    out_.clear();
    in_.clear();
    inPos_ = 0;
  }

 private:
  std::string out_;
  std::string in_;
  size_t inPos_ = 0;
};

inline EmuSerial USBSerial;
inline EmuSerial Serial;

// --- GPIO / ADC -------------------------------------------------------------

inline std::function<int(uint8_t)>& emuDigitalReadHook() {
  static std::function<int(uint8_t)> hook;
  return hook;
}

inline std::function<uint16_t(uint8_t)>& emuAnalogReadHook() {
  static std::function<uint16_t(uint8_t)> hook;
  return hook;
}

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t pin) {
  return emuDigitalReadHook() ? emuDigitalReadHook()(pin) : HIGH;  // pull-up idle
}
inline uint16_t analogRead(uint8_t pin) {
  return emuAnalogReadHook() ? emuAnalogReadHook()(pin) : 0;
}
inline void analogReadResolution(uint8_t) {}

// --- ESP --------------------------------------------------------------------

class EspClass {
 public:
  // On target this never returns; here it is recorded and the caller resumes.
  void restart() { restarts_++; }
  uint32_t getFreeHeap() { return 200000; }
  int restartCount() const { return restarts_; }
  void resetRestartCount() { restarts_ = 0; }

 private:
  int restarts_ = 0;
};

inline EspClass ESP;
