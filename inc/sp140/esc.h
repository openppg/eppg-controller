#ifndef INC_SP140_ESC_H_
#define INC_SP140_ESC_H_

#include <Arduino.h>
#include "sp140/structs.h"

#ifdef M0_PIO
  #include "../../inc/sp140/m0-config.h"
#else
  #include "../../inc/sp140/rp2040-config.h"
#endif

void initESC(int escPin);
void setupESCSerial();
void setESCThrottle(int throttlePWM);
void readESCTelemetry();
void handleESCSerialData(byte buffer[]);
void prepareESCSerialRead();
int checkFletcher16(byte byteBuffer[]);

extern STR_ESC_TELEMETRY_140 escTelemetryData;

#endif  // INC_SP140_ESC_H_
