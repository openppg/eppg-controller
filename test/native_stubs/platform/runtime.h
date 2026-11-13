#pragma once
// Host-only platform implementation for native tests, without an Arduino SDK.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
using QueueHandle_t = void*;
using TaskHandle_t = void*;
inline int g_testAnalogReadValue = 0;
inline void setTestAnalogReadValue(int value) { g_testAnalogReadValue = value; }
extern "C" int eppg_gpio_config(int,bool,bool);
extern "C" int eppg_adc_read(int);
namespace platform {
inline uint32_t millis32() {
  static auto start=std::chrono::steady_clock::now();
  return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now()-start).count());
}
inline uint32_t micros32() { return millis32()*1000u; }
inline void delayMs(uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
inline long scaleInteger(long v,long low,long high,long out_low,long out_high) {
  return (v-low)*(out_high-out_low)/(high-low)+out_low;
}
template<class T,class A,class B> inline T clamp(T v,A low,B high) {
  return std::min<T>(std::max<T>(v,static_cast<T>(low)),static_cast<T>(high));
}
class Console {
 public:
  void init() {}
  template<class...Args> void text(Args&&...) {}
  template<class...Args> void line(Args&&...) {}
  template<class...Args> void format(Args&&...) {}
  size_t write(const uint8_t*,size_t length) { return length; }
  size_t write(const char* text) { return text ? std::strlen(text) : 0; }
  bool hasInput() { return false; }
  int readByte() { return -1; }
};
inline Console console;
}
