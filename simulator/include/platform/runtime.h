#pragma once
// Deterministic host implementation of the native-IDF platform boundary.
// Legacy reference wrappers use Arduino.h only as a test transport interface.
#include "Arduino.h"
#include <algorithm>
#include <cstdint>
#include <string>
namespace platform {
inline uint32_t millis32() { return static_cast<uint32_t>(eppg::sim::g_virtual_time_ms); }
inline uint32_t micros32() { return millis32() * 1000u; }
inline void delayMs(uint32_t ms) { eppg::sim::g_virtual_time_ms += ms; }
inline long scaleInteger(long v,long lo,long hi,long out_lo,long out_hi) {
  return (v-lo)*(out_hi-out_lo)/(hi-lo)+out_lo;
}
template<class T,class A,class B> inline T clamp(T value,A lo,B hi) {
  return std::min<T>(std::max<T>(value,static_cast<T>(lo)),static_cast<T>(hi));
}
class Console {
 public:
  template<class...Args> void text(Args&&...) {}
  template<class...Args> void line(Args&&...) {}
  template<class...Args> void format(Args&&...) {}
};
inline Console console;
}
extern "C" inline int eppg_gpio_config(int,bool,bool) { return 0; }
extern "C" inline int eppg_adc_read(int) { return eppg::sim::g_analog_read_value; }
