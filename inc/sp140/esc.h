#ifndef INC_SP140_ESC_H_
#define INC_SP140_ESC_H_

#include <Arduino.h>
#include "sp140/structs.h"
#include "../../inc/sp140/esp32s3-config.h"
#include <SineEsc.h>
#include <CanardAdapter.h>

// Temperature thresholds
#define ESC_MOS_WARN 90
#define ESC_MOS_CRIT 110
#define ESC_MCU_WARN 80
#define ESC_MCU_CRIT 95
#define ESC_CAP_WARN 85
#define ESC_CAP_CRIT 100
#define MOTOR_WARN 90
#define MOTOR_CRIT 110

bool initESC(int escPin);
void setESCThrottle(int throttlePWM);
void readESCTelemetry();
bool setupTWAI();

// Get the highest temperature from all ESC sensors
float getHighestTemp(const STR_ESC_TELEMETRY_140& telemetry);

// Function declarations
TempState checkTempState(float temp, TempComponent component);

// for debugging
void dumpThrottleResponse(const sine_esc_SetThrottleSettings2Response *res);
void dumpESCMessages();  // dumps all messages to USBSerial

// External declaration of telemetry data structure
extern STR_ESC_TELEMETRY_140 escTelemetryData;

#endif  // INC_SP140_ESC_H_
