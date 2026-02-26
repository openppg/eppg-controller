// Copyright 2020 <Zach Whitehead>
#ifndef INC_SP140_STRUCTS_H_
#define INC_SP140_STRUCTS_H_

#include <cstdint>

#pragma pack(push, 1)

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
  TelemetryState escState;       // Current connection state
  uint16_t running_error;      // Runtime error bitmask
  uint16_t selfcheck_error;    // Self-check error bitmask
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

#define BMS_CELLS_NUM 24  // Maximum number of cells supported
#define BMS_TEMPERATURE_SENSORS_NUM 6  // MOS, Balance, T1-T4

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
  uint8_t battery_fail_level;  // Battery failure status
  float voltage_differential;   // Highest cell minus lowest cell voltage (V)
  unsigned long lastUpdateMs;   // Timestamp of last telemetry update
  bool is_charging;
  bool is_charge_mos;
  bool is_discharge_mos;
  TelemetryState bmsState;        // Current connection state
  float cell_voltages[BMS_CELLS_NUM];  // Individual cell voltages

  // Individual temperature sensors
  float mos_temperature;        // BMS MOSFET temperature (°C) - index 0
  float balance_temperature;    // BMS balance resistor temperature (°C) - index 1
  float t1_temperature;         // T1 cell temperature sensor (°C) - index 2
  float t2_temperature;         // T2 cell temperature sensor (°C) - index 3
  float t3_temperature;         // T3 cell temperature sensor (°C) - index 4
  float t4_temperature;         // T4 cell temperature sensor (°C) - index 5
} STR_BMS_TELEMETRY_140;
#pragma pack(pop)

// Add this struct definition near other structs
struct MelodyRequest {
  uint16_t* notes;
  uint8_t size;
  uint16_t duration;
};

// ============================================================================
// BLE Binary Packed Telemetry Structures (V1)
// These structures are designed for efficient BLE transmission with minimal
// overhead. All multi-byte values are little-endian (ESP32/ARM native).
// ============================================================================

#pragma pack(push, 1)

// Binary packed BMS telemetry for BLE transmission (~65 bytes)
// For multi-BMS systems, send separate notifications with different bms_id values
typedef struct {
  uint8_t version;              // Protocol version (1)
  uint8_t bms_id;               // BMS identifier (0-3 for multi-BMS)
  uint8_t connection_state;     // TelemetryState enum value
  float soc;                    // State of charge (%)
  float battery_voltage;        // Total battery voltage (V)
  float battery_current;        // Battery current (A)
  float power;                  // Power (kW)
  float highest_cell_voltage;   // Highest cell voltage (V)
  float lowest_cell_voltage;    // Lowest cell voltage (V)
  float highest_temperature;    // Highest temperature (°C)
  float lowest_temperature;     // Lowest temperature (°C)
  float voltage_differential;   // Cell voltage differential (V)
  uint8_t battery_fail_level;   // Battery failure status
  uint8_t is_charge_mos;        // Charge MOSFET state (0/1)
  uint8_t is_discharge_mos;     // Discharge MOSFET state (0/1)
  uint8_t is_charging;          // Charging state (0/1)
  uint32_t battery_cycle;       // Battery cycle count
  float energy_cycle;           // Energy per cycle (kWh)
  uint32_t lastUpdateMs;        // Timestamp of last update
} BLE_BMS_Telemetry_V1;

// Binary packed Extended BMS telemetry for BLE transmission (~175 bytes)
// Fixed-size payload with all summary fields + per-cell/per-probe values.
typedef struct {
  uint8_t version;              // Protocol version (1)
  uint8_t bms_id;               // BMS identifier (0-3 for multi-BMS)
  uint8_t connection_state;     // TelemetryState enum value
  float soc;                    // State of charge (%)
  float battery_voltage;        // Total battery voltage (V)
  float battery_current;        // Battery current (A)
  float power;                  // Power (kW)
  float highest_cell_voltage;   // Highest cell voltage (V)
  float lowest_cell_voltage;    // Lowest cell voltage (V)
  float highest_temperature;    // Highest temperature (deg C)
  float lowest_temperature;     // Lowest temperature (deg C)
  float voltage_differential;   // Cell voltage differential (V)
  uint8_t battery_fail_level;   // Battery failure status
  uint8_t is_charge_mos;        // Charge MOSFET state (0/1)
  uint8_t is_discharge_mos;     // Discharge MOSFET state (0/1)
  uint8_t is_charging;          // Charging state (0/1)
  uint32_t battery_cycle;       // Battery cycle count
  float energy_cycle;           // Energy per cycle (kWh)
  uint32_t lastUpdateMs;        // Timestamp of last update
  float cell_voltages[BMS_CELLS_NUM];  // Per-cell voltages (V)
  // Temperature ordering: [mos, balance, t1, t2, t3, t4]
  float temperatures[BMS_TEMPERATURE_SENSORS_NUM];  // Probe temperatures (deg C)
} BLE_BMS_Extended_Telemetry_V1;

// Binary packed ESC telemetry for BLE transmission (~46 bytes)
typedef struct {
  uint8_t version;              // Protocol version (1)
  uint8_t connection_state;     // TelemetryState enum value
  float volts;                  // ESC voltage (V)
  float amps;                   // ESC current (A)
  float mos_temp;               // MOSFET temperature (°C)
  float cap_temp;               // Capacitor temperature (°C)
  float mcu_temp;               // MCU temperature (°C)
  float motor_temp;             // Motor temperature (°C), NaN if sensor invalid/disconnected
  int32_t eRPM;                 // Electrical RPM
  uint16_t inPWM;               // Input PWM value
  uint16_t running_error;       // Runtime error bitmask
  uint16_t selfcheck_error;     // Self-check error bitmask
  uint32_t lastUpdateMs;        // Timestamp of last update
} BLE_ESC_Telemetry_V1;

// Binary packed Controller telemetry for BLE transmission (~23 bytes)
// Contains ESP32 sensor data and system status
typedef struct {
  uint8_t version;              // Protocol version (1)
  float altitude;               // Barometric altitude (m)
  float baro_temp;              // Barometric sensor temperature (°C)
  float vario;                  // Vertical speed (m/s)
  float mcu_temp;               // Internal ESP32 temperature (°C)
  uint16_t pot_raw;             // Raw potentiometer reading (0..4095)
  uint32_t uptime_ms;           // Time since boot (ms)
} BLE_Controller_Telemetry_V1;

static_assert(sizeof(BLE_BMS_Extended_Telemetry_V1) <= 182,
              "Extended BMS packet exceeds BLE payload budget for MTU 185");

#pragma pack(pop)

#endif  // INC_SP140_STRUCTS_H_
