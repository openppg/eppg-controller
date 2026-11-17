#ifndef INC_SP140_ESC_TELEMETRY_LOGIC_H_
#define INC_SP140_ESC_TELEMETRY_LOGIC_H_

// Pure ESC telemetry decisions used by readESCTelemetry() in esc.cpp.
// No SineEsc/TWAI/Arduino dependency, so these are host-tested
// (test/test_esc_telemetry). Transport and protocol parsing stay in the
// SINE-ESC-CAN library.

#include <math.h>
#include <stdint.h>

// Motor temp validity range (disconnected/invalid readings are represented as NaN)
constexpr float MOTOR_TEMP_VALID_MIN_C = -20.0f;
constexpr float MOTOR_TEMP_VALID_MAX_C = 140.0f;

inline bool isMotorTempValidC(float tempC) {
  return tempC > MOTOR_TEMP_VALID_MIN_C && tempC <= MOTOR_TEMP_VALID_MAX_C;
}

// A disconnected motor thermistor reads ~149 °C (pulled high). Store anything
// outside the valid range as NaN so downstream consumers skip it.
inline float sanitizeMotorTempC(float rawTempC) {
  return isMotorTempValidC(rawTempC) ? rawTempC : NAN;
}

// Tracks ESC runtime from the ESC's uint16 time_10ms counter (~10.9 min
// period) and doubles as the stale-data guard: the SINE library keeps
// returning its last response, so an unchanged counter means no new frame.
struct EscRuntimeTracker {
  uint16_t lastTime10ms = 0;
  uint32_t accumulatedMs = 0;
  bool firstUpdate = true;
};

// Feed the latest response counter. Returns true when it is a new frame
// (telemetry should be applied), false when it is stale. Unsigned uint16
// subtraction keeps the elapsed time correct across counter rollover, e.g.
// last=65000, current=100 -> 636 ticks = 6360 ms.
inline bool escRuntimeOnFrame(EscRuntimeTracker& tracker, uint16_t rawTime10ms) {
  if (rawTime10ms == tracker.lastTime10ms) {
    return false;
  }
  if (!tracker.firstUpdate) {
    const uint16_t deltaTicks = static_cast<uint16_t>(rawTime10ms - tracker.lastTime10ms);
    tracker.accumulatedMs += static_cast<uint32_t>(deltaTicks) * 10UL;
  }
  tracker.firstUpdate = false;
  tracker.lastTime10ms = rawTime10ms;
  return true;
}

// On link timeout: restart runtime from zero on reconnect, but keep
// lastTime10ms so the stale guard still rejects the unchanged counter the
// library keeps returning.
inline void escRuntimeOnDisconnect(EscRuntimeTracker& tracker) {
  tracker.accumulatedMs = 0;
  tracker.firstUpdate = true;
}

// True when the ESC link should be treated as NOT_CONNECTED: never heard
// from (lastCommMs == 0) or silent for longer than timeoutMs. uint32_t math
// so the millis() wrap behaves the same on the host as on the ESP32.
inline bool escTelemetryTimedOut(uint32_t nowMs, uint32_t lastCommMs, uint32_t timeoutMs) {
  return lastCommMs == 0 || (nowMs - lastCommMs) > timeoutMs;
}

#endif  // INC_SP140_ESC_TELEMETRY_LOGIC_H_
