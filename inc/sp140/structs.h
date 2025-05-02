// Copyright 2020 <Zach Whitehead>
#ifndef INC_SP140_STRUCTS_H_
#define INC_SP140_STRUCTS_H_

#pragma pack(push, 1)

// Temperature component enum
enum TempComponent {
  COMP_ESC_MOS = 0,
  COMP_ESC_MCU = 1,
  COMP_ESC_CAP = 2,
  COMP_MOTOR = 3
};

enum TempState {
  TEMP_INVALID = -1,  // Sensor reading is invalid
  TEMP_NORMAL = 0,
  TEMP_WARNING = 1,
  TEMP_CRITICAL = 2
};

// Telemetry connection state
enum class TelemetryState : uint8_t {
  NOT_CONNECTED,  // Never received data since boot
  CONNECTED,      // Receiving data normally
  STALE          // Was connected but hasn't received data recently
};

// v1 ESC telemetry
typedef struct {
  float volts;
  float mos_temp;
  float cap_temp;
  float mcu_temp;
  float motor_temp;  // Motor temperature in °C
  float amps;
  float eRPM;
  float inPWM;
  float outPWM;
  uint8_t statusFlag;
  word checksum;
  unsigned long lastUpdateMs;  // Timestamp of last telemetry update
  TempState mos_state;         // MOS temperature state
  TempState mcu_state;        // MCU temperature state
  TempState cap_state;        // CAP temperature state
  TempState motor_state;      // Motor temperature state
  bool temp_sensor_error;     // True if any sensor is invalid
  TelemetryState escState;       // Current connection state
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
  uint8_t theme;     // 0,1 for light/dark
  uint8_t revision;     // 2040 board revision (1=original, 2=rev1, 3=esp32s3)
  int32_t timezone_offset;  // Timezone offset in seconds from UTC
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

// Used to hold data from either BMS or ESC
struct UnifiedBatteryData {
  float volts;
  float amps;
  float power;  // power in kW
  float soc;    // State of Charge
};
// BMS telemetry data
typedef struct {
  float soc;                    // State of Charge (%)
  float battery_voltage;        // Total battery voltage (V)
  float battery_current;        // Battery current (A)
  float power;                  // Power (kW)
  float highest_cell_voltage;   // Highest individual cell voltage (V)
  float lowest_cell_voltage;    // Lowest individual cell voltage (V)
  float highest_temperature;    // Highest temperature reading (°C)
  float lowest_temperature;     // Lowest temperature reading (°C)
  float energy_cycle;          // Energy per cycle (kWh)
  uint32_t battery_cycle;      // Battery cycle count
  uint8_t battery_failure_level;  // Battery failure status
  float voltage_differential;   // Highest cell minus lowest cell voltage (V)
  unsigned long lastUpdateMs;   // Timestamp of last telemetry update
  bool is_charging;
  TelemetryState bmsState;        // Current connection state
} STR_BMS_TELEMETRY_140;
#pragma pack(pop)


static BatteryVoltagePoint batteryLevels[] = {
  {99.6, 100},  // 99+ volts corresponds to 100%
  {94.8, 90}, {93.36, 80}, {91.68, 70}, {89.76, 60}, {87.6, 50},
  {85.2, 40}, {82.32, 30}, {80.16, 20}, {78, 10}, {60.96, 0}
};

// Add this struct definition near other structs
struct MelodyRequest {
  uint16_t* notes;
  uint8_t size;
  uint16_t duration;
};

#endif  // INC_SP140_STRUCTS_H_
