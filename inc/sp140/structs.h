// Copyright 2020 <Zach Whitehead>
#ifndef INC_SP140_STRUCTS_H_
#define INC_SP140_STRUCTS_H_

#pragma pack(push, 1)

// v1 ESC telemetry
typedef struct {
  float volts;
  float mos_temp;
  float cap_temp;
  float mcu_temp;
  float amps;
  float eRPM;
  float inPWM;
  float outPWM;
  uint8_t statusFlag;
  word checksum;
  unsigned long lastUpdateMs;  // Timestamp of last telemetry update
}STR_ESC_TELEMETRY_140;

// Internal device data
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
  uint8_t theme;     // 0,1 for light/dark
  uint8_t revision;     // 2040 board revision (1=original, 2=rev1, 3=esp32s3)
  uint16_t crc;        // error check
}STR_DEVICE_DATA_140_V1;

// Tone struct (passed between cores)
typedef struct {
  uint16_t freq;
  uint16_t duration;
}STR_NOTE;

struct BatteryVoltagePoint {
  float voltage;
  float percent;
};
#pragma pack(pop)


static BatteryVoltagePoint batteryLevels[] = {
  {99.6, 100},  // 99+ volts corresponds to 100%
  {94.8, 90}, {93.36, 80}, {91.68, 70}, {89.76, 60}, {87.6, 50},
  {85.2, 40}, {82.32, 30}, {80.16, 20}, {78, 10}, {60.96, 0}
};

#endif  // INC_SP140_STRUCTS_H_
