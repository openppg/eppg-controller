#ifndef INC_SP140_ESC_H_
#define INC_SP140_ESC_H_

#include <Arduino.h>

// Motor temp validity range (disconnected/invalid readings are represented as NaN)
constexpr float MOTOR_TEMP_VALID_MIN_C = -20.0f;
constexpr float MOTOR_TEMP_VALID_MAX_C = 140.0f;

inline bool isMotorTempValidC(float tempC) {
  return tempC > MOTOR_TEMP_VALID_MIN_C && tempC <= MOTOR_TEMP_VALID_MAX_C;
}
#include "sp140/structs.h"
#include "../../inc/sp140/esp32s3-config.h"
#include <SineEsc.h>
#include <CanardAdapter.h>
#include "sp140/esc_errors.h"

enum class EscStatusLightMode : uint8_t {
  OFF = 0,
  DISARMED_NOMINAL,
  DISARMED_WARNING,
  DISARMED_CRITICAL,
  FLIGHT_NOMINAL,
  FLIGHT_WARNING,
  FLIGHT_CRITICAL,
};

void initESC();
void setESCThrottle(int throttlePWM);
void readESCTelemetry();
bool setupTWAI();

// Accessor to the single shared CanardAdapter (the controller's sole CAN owner).
// Used by the ESC config relay so its requests go out on the same adapter the
// throttle task drives — keeping all CAN traffic on one task, no mutex needed.
CanardAdapter& escAdapter();

// Request ESC hardware info (HW ID, FW version, bootloader, serial number).
// Thread-safe: sets a flag consumed by readESCTelemetry() on its next tick.
// Also called automatically the first time the ESC connects.
void requestEscHardwareInfo();

// ESC LED control
void requestEscStatusLightMode(EscStatusLightMode mode);

// ESC motor beep
void queueEscMotorBeepArm();
void queueEscMotorBeepDisarm();

// External declaration of telemetry data structure
extern STR_ESC_TELEMETRY_140 escTelemetryData;

#endif  // INC_SP140_ESC_H_
