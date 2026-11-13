#pragma once
// Test-only native implementation of the controller platform interface.
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <cstdint>
#include <string>
namespace platform {
inline uint32_t millis32() { return static_cast<uint32_t>(millis()); }
inline uint32_t micros32() { return millis32() * 1000u; }
inline void delayMs(uint32_t value) { delay(value); }
inline long scaleInteger(long value,long low,long high,long out_low,long out_high) {
  return (value-low)*(out_high-out_low)/(high-low)+out_low;
}
template<class T,class A,class B> T clamp(T value,A low,B high) {
  return std::min<T>(std::max<T>(value,static_cast<T>(low)),static_cast<T>(high));
}
class Console {
 public:
  void init() {}
  template<class... Args> void text(Args&&...) {}
  template<class... Args> void line(Args&&...) {}
  template<class... Args> void format(Args&&...) {}
  size_t write(const uint8_t*,size_t length) { return length; }
  size_t write(const char* value) { return value ? std::strlen(value) : 0; }
  bool hasInput() { return false; }
  int readByte() { return -1; }
};
inline Console console;
}
