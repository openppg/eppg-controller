#pragma once

#include "platform/peripheral.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include "esp_system.h"
#include "esp_rom_sys.h"
#include "esp_heap_caps.h"
#include <type_traits>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace platform {

inline uint32_t millis32() { return eppg_millis(); }
inline uint32_t micros32() { return static_cast<uint32_t>(eppg_micros()); }
inline void delayMs(uint32_t ms) { eppg_delay_ms(ms); }

// Integer truncation is deliberate: the control transfer function is part of
// the product interface and must match the original firmware at every ADC code.
inline long scaleInteger(long value, long from_min, long from_max,
                         long to_min, long to_max) {
  return (value - from_min) * (to_max - to_min) /
             (from_max - from_min) + to_min;
}
template<class T, class A, class B>
inline T clamp(T value, A low, B high) {
  return std::min<T>(std::max<T>(value, static_cast<T>(low)), static_cast<T>(high));
}

// Application console; actual IO uses the ESP32-S3 USB Serial/JTAG driver.
// No Arduino Stream/Print or Arduino USB device runtime is linked.
class Console {
 public:
  void init() { eppg_usb_init(); }
  size_t write(const uint8_t* data, size_t length) {
    const int count = eppg_usb_write(data, length);
    return count > 0 ? static_cast<size_t>(count) : 0;
  }
  size_t write(const char* text) {
    return write(reinterpret_cast<const uint8_t*>(text), strlen(text));
  }
  void text(const char* value) { if (value) write(value); }
  void text(const std::string& value) { write(reinterpret_cast<const uint8_t*>(value.data()), value.size()); }
  template<class T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
  void text(T value) { format("%lld", static_cast<long long>(value)); }
  void text(double value, int digits = 2) { format("%.*f", digits, value); }
  template<class T> void line(const T& value) { text(value); write("\r\n"); }
  void line() { write("\r\n"); }
  void line(double value, int digits) { text(value, digits); write("\r\n"); }
  template<class... Args> void format(const char* format_string, Args... args) {
    char data[512];
    const int n = snprintf(data, sizeof(data), format_string, args...);
    if (n > 0) write(reinterpret_cast<const uint8_t*>(data), std::min<size_t>(n, sizeof(data) - 1));
  }
  bool hasInput();
  int readByte();
 private:
  int pending_ = -1;
};
extern Console console;
}
