#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

// Host compatibility surface used by the production throttle module.  Time is
// supplied by the deterministic simulator rather than wall-clock time.
namespace eppg::sim {
extern uint64_t g_virtual_time_ms;
extern int g_analog_read_value;
}  // namespace eppg::sim

inline unsigned long millis() {
  return static_cast<unsigned long>(eppg::sim::g_virtual_time_ms);
}

inline unsigned long micros() {
  return static_cast<unsigned long>(eppg::sim::g_virtual_time_ms * 1000ULL);
}

class SPIClass {};

class String {
 public:
  String() = default;
  String(const char* value) : value_(value == nullptr ? "" : value) {}
  const char* c_str() const { return value_.c_str(); }
  std::size_t length() const { return value_.length(); }

 private:
  std::string value_;
};

class HostSerial {
 public:
  template <typename T>
  void print(const T&) {}
  template <typename T>
  void println(const T&) {}
  void println() {}
  template <typename... Args>
  void printf(const char*, Args...) {}
};

inline HostSerial Serial;
inline HostSerial USBSerial;

inline void pinMode(int, int) {}
inline int analogRead(int) { return eppg::sim::g_analog_read_value; }
inline void analogReadResolution(int) {}

#ifndef INPUT
#define INPUT 0
#endif

template <typename T>
inline T constrain(T value, T minimum, T maximum) {
  return std::min(maximum, std::max(minimum, value));
}

template <typename T>
inline T min(T lhs, T rhs) {
  return std::min(lhs, rhs);
}

template <typename T>
inline T max(T lhs, T rhs) {
  return std::max(lhs, rhs);
}

inline long map(long value, long in_min, long in_max, long out_min,
                long out_max) {
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) +
         out_min;
}
