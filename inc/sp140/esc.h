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

void initESC();
void setESCThrottle(int throttlePWM);
void readESCTelemetry();
bool setupTWAI();

// Get the highest temperature from all ESC sensors
float getHighestTemp(const STR_ESC_TELEMETRY_140& telemetry);

// ESC Error Decoding Functions
String decodeRunningError(uint16_t errorCode);
String decodeSelfCheckError(uint16_t errorCode);
bool hasRunningError(uint16_t errorCode);
bool hasSelfCheckError(uint16_t errorCode);
bool hasCriticalRunningError(uint16_t errorCode);
bool hasWarningRunningError(uint16_t errorCode);
bool hasCriticalSelfCheckError(uint16_t errorCode);

// Individual running error bit checkers
bool hasOverCurrentError(uint16_t errorCode);
bool hasLockedRotorError(uint16_t errorCode);
bool hasOverTempError(uint16_t errorCode);
bool hasOverVoltError(uint16_t errorCode);
bool hasVoltagDropError(uint16_t errorCode);
bool hasThrottleSatWarning(uint16_t errorCode);

// Individual self-check error bit checkers
bool hasMotorCurrentOutError(uint16_t errorCode);
bool hasTotalCurrentOutError(uint16_t errorCode);
bool hasMotorVoltageOutError(uint16_t errorCode);
bool hasCapNTCError(uint16_t errorCode);
bool hasMosNTCError(uint16_t errorCode);
bool hasBusVoltRangeError(uint16_t errorCode);
bool hasBusVoltSampleError(uint16_t errorCode);
bool hasMotorZLowError(uint16_t errorCode);
bool hasMotorZHighError(uint16_t errorCode);
bool hasMotorVDet1Error(uint16_t errorCode);
bool hasMotorVDet2Error(uint16_t errorCode);
bool hasMotorIDet2Error(uint16_t errorCode);
bool hasSwHwIncompatError(uint16_t errorCode);
bool hasBootloaderBadError(uint16_t errorCode);

// for debugging
void dumpThrottleResponse(const sine_esc_SetThrottleSettings2Response *res);
void dumpESCMessages();  // dumps all messages to USBSerial

// External declaration of telemetry data structure
extern STR_ESC_TELEMETRY_140 escTelemetryData;

#endif  // INC_SP140_ESC_H_
