#ifndef INC_SP140_TELEMETRY_HUB_H_
#define INC_SP140_TELEMETRY_HUB_H_

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstdint>

#include "sp140/structs.h"

// Logging mode state machine for RAM buffer / gap file management
enum class LoggingMode : uint8_t {
  STREAMING,      // BLE connected, phone is logging, RAM buffer is rolling
  GAP_RECORDING,  // BLE disconnected, writing to RAM + LittleFS
  GAP_SYNCING     // BLE reconnected, syncing gap file while also streaming live
};

// Central telemetry data hub. Producers (throttle task, BMS task) write into
// this struct; consumers (BLE notify task, data logger task, UI task) read from
// it. Each field has a monotonic sequence counter so consumers can detect new
// data without consuming it.
struct TelemetryHub {
  SemaphoreHandle_t mutex;

  // ESC data (written by throttle task at 50Hz)
  STR_ESC_TELEMETRY_140 esc;
  uint32_t escSeq;

  // BMS data (written by BMS task at ~2Hz)
  STR_BMS_TELEMETRY_140 bms;
  uint32_t bmsSeq;

  // Controller sensor data (written periodically)
  float altitude;
  float baro_temp;
  float baro_pressure;
  float vario;
  float mcu_temp;
  uint16_t pot_raw;
  uint32_t uptime_ms;
  uint32_t ctrlSeq;

  // Logging mode for gap file management
  LoggingMode loggingMode;
};

// Initialize the hub (creates mutex, zeros all fields)
void telemetryHubInit();

// --- Producer helpers (acquire mutex internally) ---

// Write new ESC telemetry. Increments escSeq.
void telemetryHubWriteEsc(const STR_ESC_TELEMETRY_140& data);

// Write new BMS telemetry. Increments bmsSeq.
void telemetryHubWriteBms(const STR_BMS_TELEMETRY_140& data);

// Write new controller sensor data. Increments ctrlSeq.
void telemetryHubWriteController(float altitude, float baro_temp,
                                  float baro_pressure, float vario,
                                  float mcu_temp, uint16_t pot_raw,
                                  uint32_t uptime_ms);

// --- Consumer helpers ---

// Read a snapshot of the entire hub. Returns true if mutex acquired.
bool telemetryHubRead(TelemetryHub* out, TickType_t timeout = pdMS_TO_TICKS(5));

// Set logging mode (called by BLE connect/disconnect handlers)
void telemetryHubSetLoggingMode(LoggingMode mode);

// Get current logging mode
LoggingMode telemetryHubGetLoggingMode();

// Global hub instance
extern TelemetryHub gTelemetryHub;

#endif  // INC_SP140_TELEMETRY_HUB_H_
