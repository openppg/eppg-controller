#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

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

inline unsigned long millis() {
  using namespace std::chrono;
  static const auto epoch = steady_clock::now();
  return static_cast<unsigned long>(
      duration_cast<milliseconds>(steady_clock::now() - epoch).count());
}

inline unsigned long micros() {
  using namespace std::chrono;
  static const auto epoch = steady_clock::now();
  return static_cast<unsigned long>(
      duration_cast<microseconds>(steady_clock::now() - epoch).count());
}

