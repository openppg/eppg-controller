#ifndef INC_SP140_BLE_BLE_IDS_H_
#define INC_SP140_BLE_BLE_IDS_H_

// Shared BLE UUID definitions. Keeping macros to match existing style.

// Configuration service
#define CONFIG_SERVICE_UUID "1779A55B-DEB8-4482-A5D1-A12E62146138"
#define METRIC_ALT_UUID "DF63F19E-7295-4A44-A0DC-184D1AFEDDF7"
#define ARMED_TIME_UUID "58B29259-43EF-4593-B700-250EC839A2B2"
#define SCREEN_ROTATION_UUID "9CBAB736-3705-4ECF-8086-FB7C5FB86282"
#define SEA_PRESSURE_UUID "DB47E20E-D8C1-405A-971A-DA0A2DF7E0F6"
#define METRIC_TEMP_UUID "D4962473-A3FB-4754-AD6A-90B079C3FB38"
#define PERFORMANCE_MODE_UUID "D76C2E92-3547-4F5F-AFB4-515C5C08B06B"
#define THEME_UUID "AD0E4309-1EB2-461A-B36C-697B2E1604D2"
#define HW_REVISION_UUID "2A27"
#define FW_VERSION_UUID "2A26"
#define UNIX_TIME_UUID "E09FF0B7-5D02-4FD5-889E-C4251A58D9E7"
#define TIMEZONE_UUID "CAE49D1A-7C21-4B0C-8520-416F3EF69DB1"
#define THROTTLE_VALUE_UUID "50AB3859-9FBF-4D30-BF97-2516EE632FAD"
#define DEVICE_STATE_UUID "8F80BCF5-B58F-4908-B079-E8AD6F5EE257"

// ESC config relay (phone -> controller -> ESC over CAN). CMD is written by the
// app (opcode-multiplexed); STATUS is read/notify for the async result.
// See: powerpack-flash-qc/configs/ESC-Config-Relay-Design.md
#define ESC_RELAY_CMD_UUID "E5C0C0DE-0001-4A5C-9B21-7E5C0F1A2B30"
#define ESC_RELAY_STATUS_UUID "E5C0C0DE-0002-4A5C-9B21-7E5C0F1A2B30"

// ESC firmware relay: CTRL is written with FW_START/FW_END/ABORT opcodes and
// read/notify for flasher status; DATA receives offset-addressed image chunks
// (write-without-response for throughput). See ESC-Config-Relay-Design.md §4.
#define ESC_FW_CTRL_UUID "E5C0C0DE-0003-4A5C-9B21-7E5C0F1A2B30"
#define ESC_FW_DATA_UUID "E5C0C0DE-0004-4A5C-9B21-7E5C0F1A2B30"

// ESC parameter read-all result blob: app writes [offset u32 LE] then reads back
// up to ~240 bytes of the result blob from that offset (paged fetch).
#define ESC_PARAM_DATA_UUID "E5C0C0DE-0005-4A5C-9B21-7E5C0F1A2B30"

// ESC relay NOTIFY: the controller pushes status + streams the result blob here,
// because GATT reads of the config service return null on this stack while
// notify is reliable. Frames: [0x01]=STATUS[code][phase][detail][len u16],
// [0x02]=DATA[offset u16][bytes]. See ESC-Config-Relay-Design.md.
#define ESC_RELAY_NOTIFY_UUID "E5C0C0DE-0006-4A5C-9B21-7E5C0F1A2B30"

// Factory QC record fetch (paged, same pattern as ESC_PARAM_DATA): app writes
// [offset u32 LE], then reads back up to ~240 bytes of the stored QC JSON
// record from that offset. Empty read = no record / past end. The app syncs
// this silently on connect and uploads to the cloud (fleet QC/cal data).
// See FIRST_BOOT_QC.md.
#define QC_RECORD_UUID "E5C0C0DE-0007-4A5C-9B21-7E5C0F1A2B30"

// Device info service
#define DEVICE_INFO_SERVICE_UUID "180A"
#define MANUFACTURER_NAME_UUID "2A29"
#define DEVICE_UNIQUE_ID_UUID "B1571560-345F-4974-A14D-66E98740232F"

// Fast-Link Unified Telemetry (V1)
#define FAST_LINK_TELEMETRY_SERVICE_UUID "45A17001-B73B-49E1-8B39-5E9ED5E1B930"
#define FAST_LINK_TELEMETRY_UUID "45A17002-B73B-49E1-8B39-5E9ED5E1B930"
// App writes 0x01 here to trigger a GetHwInfo request to the ESC
#define FAST_LINK_COMMAND_UUID "45A17003-B73B-49E1-8B39-5E9ED5E1B930"

// Espressif Standard OTA Service UUIDs (Android esp-ble-ota-android app)
static const NimBLEUUID OTA_SERVICE_UUID("00008018-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID OTA_RECV_FW_UUID("00008020-0000-1000-8000-00805f9b34fb");   // Firmware data (Write/Indicate)
static const NimBLEUUID OTA_PROGRESS_UUID("00008021-0000-1000-8000-00805f9b34fb");  // Progress (Indicate)
static const NimBLEUUID OTA_COMMAND_UUID("00008022-0000-1000-8000-00805f9b34fb");   // Command (Write/Indicate)
static const NimBLEUUID OTA_CUSTOMER_UUID("00008023-0000-1000-8000-00805f9b34fb");  // Customer (Indicate)

#endif  // INC_SP140_BLE_BLE_IDS_H_
