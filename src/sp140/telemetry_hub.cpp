#include "sp140/telemetry_hub.h"

#include <cstring>

TelemetryHub gTelemetryHub = {};

void telemetryHubInit() {
  memset(&gTelemetryHub, 0, sizeof(gTelemetryHub));
  gTelemetryHub.mutex = xSemaphoreCreateMutex();
  gTelemetryHub.loggingMode = LoggingMode::STREAMING;
}

void telemetryHubWriteEsc(const STR_ESC_TELEMETRY_140& data) {
  if (gTelemetryHub.mutex == nullptr) return;
  if (xSemaphoreTake(gTelemetryHub.mutex, pdMS_TO_TICKS(2)) != pdTRUE) return;
  gTelemetryHub.esc = data;
  gTelemetryHub.escSeq++;
  xSemaphoreGive(gTelemetryHub.mutex);
}

void telemetryHubWriteBms(const STR_BMS_TELEMETRY_140& data) {
  if (gTelemetryHub.mutex == nullptr) return;
  if (xSemaphoreTake(gTelemetryHub.mutex, pdMS_TO_TICKS(2)) != pdTRUE) return;
  gTelemetryHub.bms = data;
  gTelemetryHub.bmsSeq++;
  xSemaphoreGive(gTelemetryHub.mutex);
}

void telemetryHubWriteController(float altitude, float baro_temp,
                                  float vario, float mcu_temp,
                                  uint16_t pot_raw, uint32_t uptime_ms) {
  if (gTelemetryHub.mutex == nullptr) return;
  if (xSemaphoreTake(gTelemetryHub.mutex, pdMS_TO_TICKS(2)) != pdTRUE) return;
  gTelemetryHub.altitude = altitude;
  gTelemetryHub.baro_temp = baro_temp;
  gTelemetryHub.vario = vario;
  gTelemetryHub.mcu_temp = mcu_temp;
  gTelemetryHub.pot_raw = pot_raw;
  gTelemetryHub.uptime_ms = uptime_ms;
  gTelemetryHub.ctrlSeq++;
  xSemaphoreGive(gTelemetryHub.mutex);
}

bool telemetryHubRead(TelemetryHub* out, TickType_t timeout) {
  if (gTelemetryHub.mutex == nullptr || out == nullptr) return false;
  if (xSemaphoreTake(gTelemetryHub.mutex, timeout) != pdTRUE) return false;
  *out = gTelemetryHub;
  xSemaphoreGive(gTelemetryHub.mutex);
  return true;
}

void telemetryHubSetLoggingMode(LoggingMode mode) {
  if (gTelemetryHub.mutex == nullptr) return;
  if (xSemaphoreTake(gTelemetryHub.mutex, pdMS_TO_TICKS(5)) != pdTRUE) return;
  gTelemetryHub.loggingMode = mode;
  xSemaphoreGive(gTelemetryHub.mutex);
}

LoggingMode telemetryHubGetLoggingMode() {
  if (gTelemetryHub.mutex == nullptr) return LoggingMode::STREAMING;
  if (xSemaphoreTake(gTelemetryHub.mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
    return LoggingMode::STREAMING;
  }
  LoggingMode mode = gTelemetryHub.loggingMode;
  xSemaphoreGive(gTelemetryHub.mutex);
  return mode;
}
