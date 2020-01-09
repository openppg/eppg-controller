// Copyright 2020 <Zach Whitehead>

#pragma pack(push, 1)
typedef struct {
  uint8_t version;  // packet struct version
  uint8_t id;       // packet ID (0x10)
  uint8_t length;   // packet length
  uint8_t armed;    // 0/1 (bool)
  uint16_t throttlePercent;  // 0 to 1000
  uint16_t crc;  // error check
}STR_CTRL2HUB_MSG;

typedef struct {
  uint8_t version;  // packet struct version
  uint8_t id;       // packet ID (0x20)
  uint8_t length;   // packet length
  uint8_t armed;    // 0/1 (bool)
  uint32_t voltage;
  uint32_t totalMah;
  uint32_t totalCurrent;
  uint16_t avgRpm;
  uint8_t avgCapTemp;
  uint8_t avgFetTemp;
  uint16_t crc;  // error check
}STR_HUB2CTRL_MSG_V1;

typedef struct {
  uint8_t version;  // packet struct version
  uint8_t id;       // packet ID (0x20)
  uint8_t length;   // packet length
  uint8_t armed;    // 0/1 (bool)
  uint32_t voltage;
  uint32_t totalMah;
  uint32_t totalCurrent;
  uint16_t avgRpm;
  uint8_t avgCapTemp;
  uint8_t avgFetTemp;
  int16_t baroTemp;       // degrees c
  uint32_t baroPressure;  // hpa/mbar
  uint16_t crc;           // error check
}STR_HUB2CTRL_MSG_V2;

typedef struct {
  uint8_t version_major;  // 4
  uint8_t version_minor;  // 1
  uint16_t armed_time;    // minutes (think Hobbs)
  uint16_t crc;  // error check
}STR_DEVICE_DATA_V1;

typedef struct {
  uint8_t version_major;  // 4
  uint8_t version_minor;  // 1
  uint16_t armed_time;    // minutes (think Hobbs)
  uint8_t screen_rotation;  // 1,2,3,4 (90 deg)
  float sea_pressure;  // 1013.25 mbar
  float min_batt_v;  // 47.2v
  float max_batt_v;  // 59.2v
  bool metric_temp;    // true
  bool metric_alt;     // false
  uint16_t unused;     // for future use
  uint16_t crc;        // error check
}STR_DEVICE_DATA_V2;
#pragma pack(pop)

static STR_CTRL2HUB_MSG controlData;
static STR_HUB2CTRL_MSG_V2 hubData;
static STR_DEVICE_DATA_V1 deviceDataV1;
static STR_DEVICE_DATA_V2 deviceData;
