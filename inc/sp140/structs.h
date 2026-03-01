// Copyright 2020 <Zach Whitehead>
#ifndef INC_SP140_STRUCTS_H_
#define INC_SP140_STRUCTS_H_

#include <cstdint>

#pragma pack(push, 1)

// Telemetry connection state
enum class TelemetryState : uint8_t {
  NOT_CONNECTED, // Not receiving data
  CONNECTED,     // Receiving data normally
};

// v1 ESC telemetry
typedef struct {
  float volts;
  float mos_temp;
  float cap_temp;
  float mcu_temp;
  float motor_temp;      // Motor temperature in °C
  float amps;            // DC bus current (A)
  float phase_current;   // AC phase current into motor windings (A)
  float eRPM;
  float inPWM;
  float outPWM;
  uint16_t comm_pwm;     // Commutation PWM output to motor (raw ESC units)
  uint16_t v_modulation; // Voltage modulation index (raw ESC units)
  uint8_t statusFlag;
  uint16_t checksum;
  unsigned long lastUpdateMs; // Timestamp of last telemetry update
  TelemetryState escState;    // Current connection state
  uint16_t running_error;     // Runtime error bitmask
  uint16_t selfcheck_error;   // Self-check error bitmask
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
  uint8_t performance_mode; // 0,1,2
  uint8_t theme;            // 0,1 for light/dark
  uint8_t revision; // 2040 board revision (1=original, 2=rev1, 3=esp32s3)
  int32_t timezone_offset; // Timezone offset in seconds from UTC
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
  float power; // power in kW
  float soc;   // State of Charge
};

#define BMS_CELLS_NUM 24 // Maximum number of cells supported

// BMS telemetry data
typedef struct {
  float soc;                  // State of Charge (%)
  float battery_voltage;      // Total battery voltage (V)
  float battery_current;      // Battery current (A)
  float power;                // Power (kW)
  float highest_cell_voltage; // Highest individual cell voltage (V)
  float lowest_cell_voltage;  // Lowest individual cell voltage (V)
  float highest_temperature;  // Highest temperature reading (°C)
  float lowest_temperature;   // Lowest temperature reading (°C)
  float energy_cycle_ah;      // Energy per cycle (Ah)
  uint32_t battery_cycle;     // Battery cycle count
  uint8_t battery_fail_level; // Battery failure status
  float voltage_differential; // Highest cell minus lowest cell voltage (V)
  unsigned long lastUpdateMs; // Timestamp of last telemetry update
  bool is_charging;
  bool is_charge_mos;
  bool is_discharge_mos;
  TelemetryState bmsState;            // Current connection state
  float cell_voltages[BMS_CELLS_NUM]; // Individual cell voltages

  // Individual temperature sensors
  float mos_temperature;     // BMS MOSFET temperature (°C) - index 0
  float balance_temperature; // BMS balance resistor temperature (°C) - index 1
  float t1_temperature;      // T1 cell temperature sensor (°C) - index 2
  float t2_temperature;      // T2 cell temperature sensor (°C) - index 3
  float t3_temperature;      // T3 cell temperature sensor (°C) - index 4
  float t4_temperature;      // T4 cell temperature sensor (°C) - index 5

  // Additional status flags
  bool charge_wire_connected; // Charge wire physically connected
  bool low_soc_warning;       // BMS low SOC warning active
  bool battery_ready;         // BMS reports battery ready for use

  // Static battery identity (populated when available)
  char battery_id[33];        // Null-terminated battery serial/ID string (up to 32 chars)

  // BMS type detected by firmware (0=unknown, 1=Type A LE cells, 2=Type B BE cells)
  uint8_t bms_type;
} STR_BMS_TELEMETRY_140;
#pragma pack(pop)

// Add this struct definition near other structs
struct MelodyRequest {
  uint16_t *notes;
  uint8_t size;
  uint16_t duration;
};

#pragma pack(push, 1)

// Fast-Link Unified Telemetry Struct (V2 - Exhaustive)
// Designed for High-Bandwidth BLE 5.0 (2M PHY, DLE)
typedef struct {
  uint8_t version;    // Protocol version (2)
  uint32_t packet_id; // Sequential packet ID for drop detection
  uint32_t uptime_ms; // Time since boot (ms)

  // Controller Data
  float altitude;        // Barometric altitude (m)
  float baro_temp;       // Barometric sensor temperature (°C)
  float baro_pressure;   // Barometric pressure (hPa)
  float vario;           // Vertical speed (m/s)
  float mcu_temp;        // ESP32 internal temperature (°C)
  uint16_t pot_raw;      // Raw throttle potentiometer (0..4095)
  uint8_t device_state;  // 0=DISARMED, 1=ARMED, 2=ARMED_CRUISING

  // ESC Data
  uint8_t esc_status;         // TelemetryState enum
  float esc_volts;            // ESC Voltage (V)
  float esc_amps;             // DC bus current (A)
  float esc_phase_current;    // AC phase current into motor windings (A)
  int32_t esc_rpm;            // Electrical RPM
  float esc_temp_mos;         // MOSFET Temp (°C)
  float esc_temp_cap;         // Capacitor Temp (°C)
  float esc_temp_mcu;         // MCU Temp (°C)
  float esc_temp_motor;       // Motor Temp (°C)
  uint16_t esc_inPWM;         // Input PWM command (recv_pwm, raw)
  uint16_t esc_outPWM;        // Commutation PWM output (comm_pwm, raw)
  uint16_t esc_v_modulation;  // Voltage modulation index (raw)
  uint16_t esc_error;         // Runtime error bitmask
  uint16_t esc_selfcheck;     // Self-check error bitmask
  // ESC static hardware info
  uint16_t esc_hardware_id;
  uint16_t esc_fw_version;
  uint16_t esc_bootloader_version;
  uint8_t esc_sn_code[16];    // ESC serial number (16 bytes)

  // BMS Data
  uint8_t bms_status;              // TelemetryState enum
  float bms_soc;                   // State of Charge (%)
  float bms_volts;                 // Total Battery Voltage (V)
  float bms_amps;                  // Battery Current (A)
  float bms_power;                 // Power (kW)
  float bms_energy_cycle_ah;       // Energy per cycle (Ah)
  uint32_t bms_battery_cycle;      // Battery cycle count
  uint8_t bms_fail_level;          // Battery failure status
  uint8_t bms_is_charging;         // Charging state (0/1)
  uint8_t bms_is_charge_mos;       // Charge MOSFET state (0/1)
  uint8_t bms_is_discharge_mos;    // Discharge MOSFET state (0/1)
  uint8_t bms_charge_wire;         // Charge wire physically connected (0/1)
  uint8_t bms_low_soc_warning;     // Low SOC warning active (0/1)
  uint8_t bms_battery_ready;       // Battery ready for use (0/1)
  float bms_highest_temp;          // Highest temperature (°C)
  float bms_lowest_temp;           // Lowest temperature (°C)
  float bms_cell_max;              // Highest cell voltage (V)
  float bms_cell_min;              // Lowest cell voltage (V)
  float bms_voltage_diff;          // Cell voltage differential (V)
  char bms_battery_id[33];         // Battery serial/ID string (null-terminated)
  uint8_t bms_type;                // BMS type (0=unknown, 1=Type A LE, 2=Type B BE)

  // Extended BMS arrays
  float bms_cell_voltages[BMS_CELLS_NUM]; // Array of 24 cell voltages (V)

  // Array of 8 temperatures (MOS, BAL, T1-T4, unused, unused)
  float bms_temp_sensors[8];
} BLE_FastLink_Telemetry;

#pragma pack(pop)

#endif // INC_SP140_STRUCTS_H_
