#ifndef INC_SP140_MONITOR_CONFIG_H_
#define INC_SP140_MONITOR_CONFIG_H_

//=============================================================================
// CENTRALIZED MONITORING CONFIGURATION
// This file consolidates all thresholds and constants previously scattered
// across multiple files (esc.h, lvgl_display.cpp, etc.)
//=============================================================================

// ESC Temperature Thresholds (°C)
#define ESC_MOS_TEMP_WARN     90.0f
#define ESC_MOS_TEMP_CRIT     110.0f
#define ESC_MCU_TEMP_WARN     80.0f  
#define ESC_MCU_TEMP_CRIT     95.0f
#define ESC_CAP_TEMP_WARN     85.0f
#define ESC_CAP_TEMP_CRIT     100.0f
#define MOTOR_TEMP_WARN       90.0f
#define MOTOR_TEMP_CRIT       110.0f

// BMS Temperature Thresholds (°C)
#define BMS_TEMP_WARN         45.0f
#define BMS_TEMP_CRIT         55.0f

// BMS Voltage Thresholds (V)
#define BMS_CELL_VOLT_WARN    3.2f
#define BMS_CELL_VOLT_CRIT    2.95f
#define BMS_TOTAL_VOLT_MIN    75.0f    // 25S * 3.0V
#define BMS_TOTAL_VOLT_MAX    105.0f   // 25S * 4.2V

// BMS State of Charge Thresholds (%)
#define BMS_SOC_WARN          20.0f
#define BMS_SOC_CRIT          5.0f

// BMS Voltage Differential (V)
#define BMS_VOLT_DIFF_WARN    0.1f     // 100mV difference
#define BMS_VOLT_DIFF_CRIT    0.2f     // 200mV difference

// Altimeter/Barometric Temperature (°C)
#define BARO_TEMP_WARN        60.0f    // Operating limit
#define BARO_TEMP_CRIT        70.0f    // Absolute limit

// Internal System Temperature (°C)
#define CPU_TEMP_WARN         70.0f    // ESP32 operating temp
#define CPU_TEMP_CRIT         80.0f    // ESP32 critical temp

// Temperature Sensor Validation Limits (°C)
#define TEMP_SENSOR_MIN       -50.0f   // Below this = sensor error
#define TEMP_SENSOR_MAX       200.0f   // Above this = sensor error

// Monitoring System Configuration
#define MONITOR_UPDATE_RATE_MS     100    // How often to check sensors
#define MONITOR_ALERT_DEBOUNCE_MS  1000   // Debounce time for alerts
#define MONITOR_CLEAR_DEBOUNCE_MS  2000   // Time to clear alerts

// Alert System Configuration
#define ALERT_MAX_ACTIVE       10     // Maximum active alerts to track
#define ALERT_DISPLAY_TIME_MS  3000   // How long to show each alert
#define ALERT_CYCLE_TIME_MS    5000   // Time between cycling alerts

// ESC Error Bit Masks (from ESC documentation)
#define ESC_RUNNING_ERROR_OVERCURRENT    (1 << 0)
#define ESC_RUNNING_ERROR_LOCKED_ROTOR   (1 << 1)
#define ESC_RUNNING_ERROR_OVERTEMP       (1 << 2)
#define ESC_RUNNING_ERROR_OVERVOLT       (1 << 6)
#define ESC_RUNNING_ERROR_VOLTAGE_DROP   (1 << 7)
#define ESC_RUNNING_WARNING_THROTTLE_SAT (1 << 5)

#define ESC_SELFCHECK_ERROR_MOTOR_CURRENT_OUT (1 << 0)
#define ESC_SELFCHECK_ERROR_TOTAL_CURRENT_OUT (1 << 1)
#define ESC_SELFCHECK_ERROR_MOTOR_VOLTAGE_OUT (1 << 2)
#define ESC_SELFCHECK_ERROR_CAP_NTC           (1 << 3)
#define ESC_SELFCHECK_ERROR_MOS_NTC           (1 << 4)
#define ESC_SELFCHECK_ERROR_BUS_VOLT_RANGE    (1 << 5)
#define ESC_SELFCHECK_ERROR_BUS_VOLT_SAMPLE   (1 << 6)
#define ESC_SELFCHECK_ERROR_MOTOR_Z_LOW       (1 << 7)
#define ESC_SELFCHECK_ERROR_MOTOR_Z_HIGH      (1 << 8)
#define ESC_SELFCHECK_ERROR_MOTOR_V_DET1      (1 << 9)
#define ESC_SELFCHECK_ERROR_MOTOR_V_DET2      (1 << 10)
#define ESC_SELFCHECK_ERROR_MOTOR_I_DET2      (1 << 11)
#define ESC_SELFCHECK_ERROR_SW_HW_INCOMPAT    (1 << 13)
#define ESC_SELFCHECK_ERROR_BOOTLOADER_BAD    (1 << 14)

#endif  // INC_SP140_MONITOR_CONFIG_H_