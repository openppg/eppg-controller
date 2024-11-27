#ifndef INC_SP140_ESC_H_
#define INC_SP140_ESC_H_

#include <Arduino.h>
#include "sp140/structs.h"

#ifdef RP_PIO
  #include "../../inc/sp140/rp2040-config.h"
#elif CAN_PIO
  #include "../../inc/sp140/esp32s3-config.h"
  #include <SineEsc.h>
  #include <CanardAdapter.h>
#endif

#ifndef CAN_PIO
  #define ESC_BAUD_RATE         115200
  #define ESC_DATA_V2_SIZE      22
  #define ESC_TIMEOUT           15

  SerialESC.begin(ESC_BAUD_RATE);
  SerialESC.setTimeout(ESC_TIMEOUT);
#endif

bool initESC(int escPin);
void setESCThrottle(int throttlePWM);
void readESCTelemetry();
bool setupTWAI();
float getBatteryPercent(float voltage);

// for debugging
void dumpThrottleResponse(const sine_esc_SetThrottleSettings2Response *res);
void dumpESCMessages();  // dumps all messages to USBSerial

// External declaration of telemetry data structure
extern STR_ESC_TELEMETRY_140 escTelemetryData;

#endif  // INC_SP140_ESC_H_
