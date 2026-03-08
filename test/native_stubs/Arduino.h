#pragma once
#include <stdint.h>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <string>
#include <thread>

// Basic Arduino types and helpers for native unit tests
using std::chrono::steady_clock;
using std::chrono::milliseconds;

class String {
 public:
  String() = default;
  String(const char* s) : data_(s ? s : "") {}

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

 private:
  std::string data_;
};

struct __FlashStringHelper;
class Print;
class Printable {
 public:
  virtual ~Printable() = default;
  virtual size_t printTo(Print&) const { return 0; }
};

#ifndef PROGMEM
#define PROGMEM
#endif

#ifndef pgm_read_byte
inline uint8_t pgm_read_byte(const void* p) {
  return *reinterpret_cast<const uint8_t*>(p);
}
#endif

inline unsigned long millis() {
  static auto start = steady_clock::now();
  return (unsigned long)std::chrono::duration_cast<milliseconds>(steady_clock::now() - start).count();
}

// Dummy Serial replacement
class Print {
 public:
  virtual ~Print() = default;
  virtual size_t write(uint8_t) { return 1; }
  virtual size_t write(const uint8_t* buffer, size_t size) {
    (void)buffer;
    return size;
  }
  size_t write(const char* s) {
    return write(reinterpret_cast<const uint8_t*>(s), s ? strlen(s) : 0);
  }

  template <typename... Args>
  void printf(const char*, Args...) {}
  template <typename T>
  void print(const T&) {}
  template <typename T>
  void println(const T&) {}
  void println() {}
};

class Stream : public Print {
 public:
  virtual int available() { return 0; }
  virtual int read() { return -1; }
  virtual size_t readBytes(char* buffer, size_t length) {
    if (buffer == nullptr || length == 0) {
      return 0;
    }
    return 0;
  }
};

struct DummySerial : public Stream {
  inline void begin(unsigned long) {}
};

static DummySerial USBSerial;
static DummySerial Serial;

// Arduino word type
typedef uint16_t word;

// FreeRTOS queue stub for unit tests
typedef void* QueueHandle_t;

inline void delay(unsigned long ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Minimal GPIO/ADC stubs for native builds
#ifndef INPUT
#define INPUT 0
#endif

#ifndef F
#define F(x) x
#endif

inline void pinMode(int, int) {}
inline int g_testAnalogReadValue = 0;
inline void setTestAnalogReadValue(int value) { g_testAnalogReadValue = value; }
inline int analogRead(int) { return g_testAnalogReadValue; }
inline void analogReadResolution(int) {}

// Math helpers
#ifndef constrain
template <typename T>
inline T constrain(T x, T a, T b) { return x < a ? a : (x > b ? b : x); }
#endif

#ifndef min
template <typename T>
inline T min(T a, T b) { return a < b ? a : b; }
#endif

#ifndef max
template <typename T>
inline T max(T a, T b) { return a > b ? a : b; }
#endif

// Arduino-style map helper
inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
