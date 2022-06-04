// OpenPPG

#ifndef __ESC_H__
#define __ESC_H__

// clang-format off
#include <Arduino.h>
// clang-format on

typedef struct {
  float volts;
  float temperatureC;
  float amps;
  float eRPM;
  float inPWM;
  float outPWM;
  uint8_t status_flags;
  word checksum;
}STR_ESC_TELEMETRY_140;

static STR_ESC_TELEMETRY_140 telemetryData;
uint8_t escData[ESC_DATA_SIZE];  // TODO move to private

class OpenPPG_ESC {
public:
  OpenPPG_ESC(HardwareSerial *hs);
  OpenPPG_ESC(Stream *serial);
  
  void begin(uint32_t baud);
  boolean validFletcher16(uint8_t *data, uint8_t size);


private:

Stream *mySerial;
HardwareSerial *hwSerial;

};

#endif // __ESC_H__
