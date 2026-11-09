#pragma once
#include <cstdint>
#include <cstdio>
#include <functional>
inline uint32_t fakeMs = 0;
inline std::function<void()> onDelay;
inline unsigned long millis() { return fakeMs; }
inline void delay(unsigned long ms) { fakeMs += ms; if (onDelay) onDelay(); }
struct SerialStub {
  template <typename... Args> void printf(const char*, Args...) {}
  void println(const char*) {}
};
inline SerialStub USBSerial;
struct EspStub { unsigned restarts = 0; void restart() { ++restarts; } };
inline EspStub ESP;
