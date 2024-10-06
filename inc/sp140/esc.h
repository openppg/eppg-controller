#ifndef INC_SP140_ESC_H_
#define INC_SP140_ESC_H_

#include <Arduino.h>
#include "sp140/structs.h"

#ifdef M0_PIO
  #include "../../inc/sp140/m0-config.h"
#elif RP_PIO
  #include "../../inc/sp140/rp2040-config.h"
#elif CAN_PIO
  #include "../../inc/sp140/esp32s3-config.h"
  #include <SineEsc.h>
  #include <CanardAdapter.h>
#endif

// ESC communication parameters
#define ESC_BAUD_RATE         115200
#define ESC_DATA_V2_SIZE      22
#define READ_INTERVAL         0
#define ESC_TIMEOUT           15

bool initESC(int escPin);
void setupESCSerial();
void setESCThrottle(int throttlePWM);
void readESCTelemetry();
void handleESCSerialData(byte buffer[]);
void prepareESCSerialRead();
int checkFletcher16(byte byteBuffer[]);
bool setupTWAI();

// for debugging
void dumpThrottleResponse(const sine_esc_SetThrottleSettings2Response *res);
void dumpESCMessages();  // dumps all messages to USBSerial

// External declaration of telemetry data structure
extern STR_ESC_TELEMETRY_140 escTelemetryData;

#endif  // INC_SP140_ESC_H_
