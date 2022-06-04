
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

#endif // __ESC_H__
