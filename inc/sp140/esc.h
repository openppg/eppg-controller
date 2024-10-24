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

#ifndef CAN_PIO
  #define ESC_BAUD_RATE         115200
  #define ESC_DATA_V2_SIZE      22
  #define ESC_TIMEOUT           15

  SerialESC.begin(ESC_BAUD_RATE);
  SerialESC.setTimeout(ESC_TIMEOUT);
#endif

bool initESC(int escPin);
void setupESCSerial();
void setESCThrottle(int throttlePWM);
void readESCTelemetry();
void handleESCSerialData(byte buffer[]);
void prepareESCSerialRead();
int checkFletcher16(byte byteBuffer[]);
bool setupTWAI();
float getBatteryPercent(float voltage);

// for debugging
void dumpThrottleResponse(const sine_esc_SetThrottleSettings2Response *res);
void dumpESCMessages();  // dumps all messages to USBSerial

// External declaration of telemetry data structure
extern STR_ESC_TELEMETRY_140 escTelemetryData;

// v2 ESC telemetry raw data structure
typedef struct  {
  // Voltage
  int V_HI;
  int V_LO;

  // Temperature
  int T_HI;
  int T_LO;

  // Current
  int I_HI;
  int I_LO;

  // Reserved
  int R0_HI;
  int R0_LO;

  // eRPM
  int RPM0;
  int RPM1;
  int RPM2;
  int RPM3;

  // Input Duty
  int DUTYIN_HI;
  int DUTYIN_LO;

  // Motor Duty
  int MOTORDUTY_HI;
  int MOTORDUTY_LO;

  // Reserved
  int R1;

  // Status Flags
  int statusFlag;

  // checksum
  int CSUM_HI;
  int CSUM_LO;
} telem_esc_t;

#endif  // INC_SP140_ESC_H_
