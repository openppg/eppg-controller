// Copyright 2020 <Zach Whitehead>
#ifndef INC_SP140_STRUCTS_H_
#define INC_SP140_STRUCTS_H_

#include <cstdint>

#pragma pack(push, 1)

// Telemetry connection state
enum class TelemetryState : uint8_t {
  NOT_CONNECTED,  // Not receiving data
  CONNECTED,      // Receiving data normally
};

// v1 ESC telemetry
typedef struct {
  float volts;
  float mos_temp;
  float cap_temp;
  float mcu_temp;
  float motor_temp;    // Motor temperature in °C
  float amps;          // DC bus current (A)
  float phase_current;  // AC phase current into motor windings (A)
  float eRPM;
  float inPWM;
  float outPWM;
  uint16_t comm_pwm;     // Commutation PWM output to motor (raw ESC units)
  uint16_t v_modulation;  // Voltage modulation index (raw ESC units)
  uint8_t statusFlag;
  uint16_t checksum;
  unsigned long lastUpdateMs;  // Timestamp of last telemetry update
  TelemetryState escState;    // Current connection state
  uint16_t running_error;     // Runtime error bitmask
  uint16_t selfcheck_error;   // Self-check error bitmask
  uint32_t esc_runtime_ms;    // ESC internal runtime (ms) from time_10ms × 10
  // Static hardware info (populated from GetHwInfo response)
  uint16_t hardware_id;
  uint16_t fw_version;
  uint16_t bootloader_version;
  uint8_t sn_code[16];
} STR_ESC_TELEMETRY_140;

// Internal device data
typedef struct {
  uint8_t version_major;    // 5
  uint8_t version_minor;    // 1
  uint16_t armed_time;      // minutes (think Hobbs)
  uint8_t screen_rotation;  // 1,2,3,4 (90 deg)
  float sea_pressure;       // 1013.25 mbar
  bool metric_temp;         // true
  bool metric_alt;          // false
  uint8_t performance_mode;  // 0,1,2
  uint8_t theme;             // 0,1 for light/dark
  uint8_t revision;  // 2040 board revision (1=original, 2=rev1, 3=esp32s3)
  int32_t timezone_offset;  // Timezone offset in seconds from UTC
} STR_DEVICE_DATA_140_V1;

// Tone struct (passed between cores)
typedef struct {
  uint16_t freq;
  uint16_t duration;
} STR_NOTE;

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

// BMS telemetry data
typedef struct {
  float soc;                    // State of Charge (%)
  float battery_voltage;        // Total battery voltage (V)
  float battery_current;        // Battery current (A)
  float power;                  // Power (kW)
  float highest_cell_voltage;   // Highest individual cell voltage (V)
  float lowest_cell_voltage;    // Lowest individual cell voltage (V)
  float highest_temperature;    // Highest valid temperature reading (°C), NaN if unavailable
  float lowest_temperature;     // Lowest valid temperature reading (°C), NaN if unavailable
  float energy_cycle_ah;        // Energy per cycle (Ah)
  uint32_t battery_cycle;       // Battery cycle count
  uint8_t battery_fail_level;   // Battery failure status
  float voltage_differential;   // Highest cell minus lowest cell voltage (V)
  unsigned long lastUpdateMs;   // Timestamp of last telemetry update
  bool is_charging;
  bool is_charge_mos;
  bool is_discharge_mos;
  TelemetryState bmsState;            // Current connection state
  float cell_voltages[BMS_CELLS_NUM];  // Individual cell voltages

  // Individual temperature sensors
  float mos_temperature;        // BMS MOSFET temperature (°C) - index 0
  float balance_temperature;    // BMS balance resistor temperature (°C) - index 1
  float t1_temperature;         // T1 cell temperature sensor (°C), NaN if disconnected - index 2
  float t2_temperature;         // T2 cell temperature sensor (°C), NaN if disconnected - index 3
  float t3_temperature;         // T3 cell temperature sensor (°C), NaN if disconnected - index 4
  float t4_temperature;         // T4 cell temperature sensor (°C), NaN if disconnected - index 5

  // Additional status flags
  bool charge_wire_connected;  // Charge wire physically connected
  bool low_soc_warning;        // BMS low SOC warning active
  bool battery_ready;          // BMS reports battery ready for use

  // Static battery identity (populated when available)
  char battery_id[33];  // Null-terminated battery serial/ID string (up to 32
                        // chars)

  // BMS type detected by firmware (0=unknown, 1=Type A older, 2=Type B newer)
  // Note: library auto-detection currently maps big-endian cells -> Type A,
  // little-endian cells -> Type B.
  uint8_t bms_type;
} STR_BMS_TELEMETRY_140;
#pragma pack(pop)

// Add this struct definition near other structs
struct MelodyRequest {
  uint16_t *notes;
  uint8_t size;
  uint16_t duration;
};

static constexpr uint8_t FASTLINK_PROTOCOL_VERSION = 3;
static constexpr int16_t kNoTempSensorDC = INT16_MIN;  // Sentinel: NaN / disconnected

#pragma pack(push, 1)

// Fast-Link Unified Telemetry Struct (V3 - Fixed-Point Compressed)
// Designed for High-Bandwidth BLE 5.0 (2M PHY, DLE)
// All float fields converted to fixed-point integers matching native sensor resolution
typedef struct {
  uint8_t version;     // Protocol version (3)
  uint32_t packet_id;  // Sequential packet ID for drop detection
  uint32_t uptime_ms;  // Time since boot (ms)

  // Controller Data
  int32_t altitude_cm;         // Barometric altitude (cm, x100 from meters)
  int16_t baro_temp_dC;        // Barometric sensor temperature (deci-C, x10)
  uint16_t baro_pressure_dHPa;  // Barometric pressure (deci-hPa, x10)
  int16_t vario_cmps;          // Vertical speed (cm/s, x100 from m/s)
  int16_t mcu_temp_dC;         // ESP32 internal temperature (deci-C, x10)
  uint16_t pot_raw;            // Raw throttle potentiometer (0..4095)
  uint8_t device_state;        // 0=DISARMED, 1=ARMED, 2=ARMED_CRUISING

  // ESC Data
  uint8_t esc_status;              // TelemetryState enum
  uint16_t esc_volts_dV;           // ESC Voltage (deci-Volts, x10)
  int16_t esc_amps_dA;             // DC bus current (deci-Amps, x10)
  int16_t esc_phase_current_dA;    // AC phase current (deci-Amps, x10)
  int32_t esc_rpm;                 // Electrical RPM
  int16_t esc_temp_mos_dC;         // MOSFET Temp (deci-C, x10)
  int16_t esc_temp_cap_dC;         // Capacitor Temp (deci-C, x10)
  int16_t esc_temp_mcu_dC;         // MCU Temp (deci-C, x10)
  int16_t esc_temp_motor_dC;       // Motor Temp (deci-C, x10), INT16_MIN = no sensor
  uint16_t esc_inPWM;              // Input PWM command (recv_pwm, raw)
  uint16_t esc_outPWM;             // Commutation PWM output (comm_pwm, raw)
  uint16_t esc_v_modulation;       // Voltage modulation index (raw)
  uint16_t esc_error;              // Runtime error bitmask
  uint16_t esc_selfcheck;          // Self-check error bitmask
  // ESC static hardware info
  uint16_t esc_hardware_id;
  uint16_t esc_fw_version;
  uint16_t esc_bootloader_version;
  uint32_t esc_runtime_ms;         // ESC internal runtime (ms) from time_10ms x 10
  uint8_t esc_sn_code[16];         // ESC serial number (16 bytes)

  // BMS Data
  uint8_t bms_status;              // TelemetryState enum
  uint8_t bms_soc;                 // State of Charge (%, 0-100, native 1% resolution)
  uint16_t bms_volts_dV;           // Total Battery Voltage (deci-Volts, x10)
  int16_t bms_amps_dA;             // Battery Current (deci-Amps, x10)
  // bms_power REMOVED - app derives from bms_volts_dV * bms_amps_dA
  uint32_t bms_energy_cycle_mAh;   // Energy per cycle (mAh, native 1 mAh resolution)
  uint32_t bms_battery_cycle;      // Battery cycle count
  uint8_t bms_fail_level;          // Battery failure status
  uint8_t bms_is_charging;         // Charging state (0/1)
  uint8_t bms_is_charge_mos;       // Charge MOSFET state (0/1)
  uint8_t bms_is_discharge_mos;    // Discharge MOSFET state (0/1)
  uint8_t bms_charge_wire;         // Charge wire physically connected (0/1)
  uint8_t bms_low_soc_warning;     // Low SOC warning active (0/1)
  uint8_t bms_battery_ready;       // Battery ready for use (0/1)
  int8_t bms_highest_temp_C;       // Highest temperature (C, native 1C resolution)
  int8_t bms_lowest_temp_C;        // Lowest temperature (C, native 1C resolution)
  uint16_t bms_cell_max_mV;        // Highest cell voltage (mV, native 1 mV resolution)
  uint16_t bms_cell_min_mV;        // Lowest cell voltage (mV, native 1 mV resolution)
  uint16_t bms_voltage_diff_mV;    // Cell voltage differential (mV)
  char bms_battery_id[33];         // Battery serial/ID string (null-terminated)
  uint8_t bms_type;                // BMS type (0=unknown, 1=Type A older, 2=Type B newer)

  // Extended BMS arrays (compressed)
  uint16_t bms_cell_voltages_mV[BMS_CELLS_NUM];  // 24 cell voltages (mV, native resolution)
  int8_t bms_temp_sensors_C[8];    // 8 temps (C, MOS/BAL/T1-T4/unused/unused)
} BLE_FastLink_Telemetry;

#pragma pack(pop)

static_assert(sizeof(BLE_FastLink_Telemetry) == 198,
              "BLE_FastLink_Telemetry V3 size mismatch - update if struct changes");

#endif  // INC_SP140_STRUCTS_H_
