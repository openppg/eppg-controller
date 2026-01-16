#pragma once
#include <stdint.h>
#include <chrono>
#include <cstdio>
#include <thread>

// Basic Arduino types and helpers for native unit tests
using std::chrono::steady_clock;
using std::chrono::milliseconds;

inline unsigned long millis() {
  static auto start = steady_clock::now();
  return (unsigned long)std::chrono::duration_cast<milliseconds>(steady_clock::now() - start).count();
}

// Dummy Serial replacement
struct DummySerial {
  template <typename... Args>
  void printf(const char*, Args...) {}
  template <typename T>
  void print(const T&) {}
  template <typename T>
  void println(const T&) {}
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

inline void pinMode(int, int) {}
inline int analogRead(int) { return 0; }
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
