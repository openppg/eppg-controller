#ifndef INC_SP140_BLE_BLE_IDS_H_
#define INC_SP140_BLE_BLE_IDS_H_

// Shared BLE UUID definitions. Keeping macros to match existing style.

// Configuration service
#define CONFIG_SERVICE_UUID      "1779A55B-DEB8-4482-A5D1-A12E62146138"
#define METRIC_ALT_UUID          "DF63F19E-7295-4A44-A0DC-184D1AFEDDF7"
#define ARMED_TIME_UUID          "58B29259-43EF-4593-B700-250EC839A2B2"
#define SCREEN_ROTATION_UUID     "9CBAB736-3705-4ECF-8086-FB7C5FB86282"
#define SEA_PRESSURE_UUID        "DB47E20E-D8C1-405A-971A-DA0A2DF7E0F6"
#define METRIC_TEMP_UUID         "D4962473-A3FB-4754-AD6A-90B079C3FB38"
#define PERFORMANCE_MODE_UUID    "D76C2E92-3547-4F5F-AFB4-515C5C08B06B"
#define THEME_UUID               "AD0E4309-1EB2-461A-B36C-697B2E1604D2"
#define HW_REVISION_UUID         "2A27"
#define FW_VERSION_UUID          "2A26"
#define UNIX_TIME_UUID           "E09FF0B7-5D02-4FD5-889E-C4251A58D9E7"
#define TIMEZONE_UUID            "CAE49D1A-7C21-4B0C-8520-416F3EF69DB1"
#define THROTTLE_VALUE_UUID      "50AB3859-9FBF-4D30-BF97-2516EE632FAD"
#define DEVICE_STATE_UUID        "8F80BCF5-B58F-4908-B079-E8AD6F5EE257"

// Device info service
#define DEVICE_INFO_SERVICE_UUID "180A"
#define MANUFACTURER_NAME_UUID   "2A29"
#define DEVICE_UNIQUE_ID_UUID    "B1571560-345F-4974-A14D-66E98740232F"

// BMS service
#define BMS_TELEMETRY_SERVICE_UUID "9E0F2FA3-3F2B-49C0-A6A3-3D8923062133"
#define BMS_SOC_UUID                "ACDEB138-3BD0-4BB3-B159-19F6F70871ED"
#define BMS_VOLTAGE_UUID            "AC0768DF-2F49-43D4-B23D-1DC82C90A9E9"
#define BMS_CURRENT_UUID            "6FEEC926-BA3C-4E65-BC71-5DB481811186"
#define BMS_POWER_UUID              "9DEA1343-434F-4555-A0A1-BB43FCBC68A6"
#define BMS_HIGH_CELL_UUID          "49267B41-560F-4CFF-ADC8-90EF85D2BE20"
#define BMS_LOW_CELL_UUID           "B9D01E5C-3751-4092-8B06-6D1FFF479E77"
#define BMS_HIGH_TEMP_UUID          "0EA08B6D-C905-4D9D-93F8-51E35DA096FC"
#define BMS_LOW_TEMP_UUID           "26CD6E8A-175D-4C8E-B487-DEFF0B034F2A"
#define BMS_FAILURE_LEVEL_UUID      "396C768B-F348-44CC-9D46-92388F25A557"
#define BMS_VOLTAGE_DIFF_UUID       "1C45825B-7C81-430B-8D5F-B644FFFC71BB"
#define BMS_CHARGE_MOS_UUID         "4D0C3E4D-E7E2-41EF-8DDD-38C4B0948F9E"
#define BMS_DISCHARGE_MOS_UUID      "175A2989-3442-4B69-9C84-0CE4F1BD4F4F"
#define BMS_CELL_VOLTAGES_UUID      "4337e58b-8462-49b2-b061-c3bf379ef4af"
#define BMS_TEMPERATURES_UUID       "C53898C0-C10D-435B-8B5D-B46A2E06EB53"

// ESC service
#define ESC_TELEMETRY_SERVICE_UUID "C154DAE9-1984-40EA-B20F-5B23F9CBA0A9"
#define ESC_VOLTAGE_UUID           "0528ecd8-9337-4249-95e4-9aba69f6c1f4"
#define ESC_CURRENT_UUID           "3889e01e-7d2d-4478-b5cc-a06b803e2788"
#define ESC_RPM_UUID               "24dc4a84-0be3-4eba-a8c3-ed9748daa599"
#define ESC_TEMPS_UUID             "d087f190-5450-4fea-b9ff-17133a0b6f64"

// ============================================================================
// Binary Packed Telemetry Characteristics (V1)
// High-efficiency binary telemetry for reduced BLE overhead
// ============================================================================

// Packed BMS telemetry characteristic (single binary packet per BMS)
// For multi-BMS, multiple notifications sent with different bms_id values
#define BMS_PACKED_TELEMETRY_UUID     "ABAB6342-D068-45A7-9D45-FBCE8E4D4DF1"

// Extended BMS data (cell voltages + temps array) - separate packet for detailed data
#define BMS_EXTENDED_TELEMETRY_UUID   "93431570-64A3-448D-AC97-4655B2D1458F"

// Packed ESC telemetry characteristic
#define ESC_PACKED_TELEMETRY_UUID     "D3C2B470-007F-494E-95C5-8E275A181A3A"

// Controller telemetry service and characteristic (ESP32 sensors: altitude, baro, vario, temps)
#define CONTROLLER_SERVICE_UUID       "01C63B60-0891-4655-BBCA-8E745C48A175"
#define CONTROLLER_TELEMETRY_UUID     "01C63B61-0891-4655-BBCA-8E745C48A176"

// ============================================================================
// OTA Service (Firmware Update)
// ============================================================================
#define OTA_SERVICE_UUID              "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define OTA_CONTROL_UUID              "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define OTA_DATA_UUID                 "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#endif  // INC_SP140_BLE_BLE_IDS_H_
