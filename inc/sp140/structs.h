// Copyright 2020 <Zach Whitehead>
#ifndef INC_SP140_STRUCTS_H_
#define INC_SP140_STRUCTS_H_

#pragma pack(push, 1)

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

typedef struct {
  uint8_t version_major;  // 5
  uint8_t version_minor;  // 1
  uint16_t armed_time;    // minutes (think Hobbs)
  uint8_t screen_rotation;  // 1,2,3,4 (90 deg)
  float sea_pressure;  // 1013.25 mbar
  bool metric_temp;    // true
  bool metric_alt;     // false
  uint8_t performance_mode;  // 0,1,2
  uint16_t batt_size;     // 4000 (4kw) or 2000 (2kw)
  uint8_t btn_mode;     // for future use
  uint8_t unused;     // for future use
  uint16_t crc;        // error check
}STR_DEVICE_DATA_140_V1;

typedef struct {
  uint16_t freq;
  uint16_t duration;
}STR_NOTE;
#pragma pack(pop)

static STR_ESC_TELEMETRY_140 telemetryData;

#endif  // INC_SP140_STRUCTS_H_
