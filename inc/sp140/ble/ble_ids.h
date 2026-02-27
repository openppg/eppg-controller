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

// ESC service
#define ESC_TELEMETRY_SERVICE_UUID "C154DAE9-1984-40EA-B20F-5B23F9CBA0A9"

// Log sync service (historical backfill)
#define LOG_SYNC_SERVICE_UUID      "F21F9A40-7A3D-4E08-9A8E-2014C5207A10"
#define LOG_SYNC_CONTROL_UUID      "F21F9A41-7A3D-4E08-9A8E-2014C5207A10"
#define LOG_SYNC_DATA_UUID         "F21F9A42-7A3D-4E08-9A8E-2014C5207A10"
#define LOG_SYNC_META_UUID         "F21F9A43-7A3D-4E08-9A8E-2014C5207A10"
#define LOG_SYNC_STATUS_UUID       "F21F9A44-7A3D-4E08-9A8E-2014C5207A10"

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

#endif  // INC_SP140_BLE_BLE_IDS_H_
