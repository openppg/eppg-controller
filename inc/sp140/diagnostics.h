#ifndef INC_SP140_DIAGNOSTICS_H_
#define INC_SP140_DIAGNOSTICS_H_

#include <Arduino.h>
#include <ArduinoJson.h>
#include <stdint.h>
#include "sp140/device_state.h"

static constexpr uint8_t DIAG_SCHEMA_VERSION = 1;
static constexpr uint8_t DIAG_HISTORY_CAPACITY = 16;
static constexpr uint8_t DIAG_BACKTRACE_DEPTH = 6;
static constexpr uint32_t DIAG_SNAPSHOT_MAGIC = 0x44484731UL;

enum class DiagResetReason : uint8_t {
  UNKNOWN = 0,
  POWERON = 1,
  EXT_PIN = 2,
  SOFTWARE = 3,
  PANIC = 4,
  INT_WDT = 5,
  TASK_WDT = 6,
  WDT = 7,
  DEEPSLEEP = 8,
  BROWNOUT = 9,
  SDIO = 10,
  USB_UART = 11,
};

enum class PlannedRestartReason : uint8_t {
  NONE = 0,
  USB_COMMAND_REBOOT = 1,
  BLE_UNBOND_REBOOT = 2,
};

#pragma pack(push, 1)

struct RuntimeHeartbeatSnapshot {
  uint32_t magic;
  uint32_t uptime_ms;
  uint32_t snapshot_ms;
  uint32_t last_throttle_run_ms;
  uint32_t last_ui_run_ms;
  uint32_t last_bms_run_ms;
  uint32_t free_heap;
  uint32_t min_free_heap;
  uint8_t device_state;
  uint8_t esc_connected;
  uint8_t bms_connected;
  uint8_t planned_restart_reason;
  uint8_t reserved;
};

struct CoreDumpSummary {
  uint8_t present;
  uint8_t valid;
  uint8_t backtrace_depth;
  char task[16];
  uint32_t pc;
  uint32_t exc_cause;
  uint32_t exc_vaddr;
  uint32_t backtrace[DIAG_BACKTRACE_DEPTH];
};

struct BootDiagRecord {
  uint32_t boot_counter;
  uint32_t prev_uptime_ms;
  uint32_t prev_throttle_age_ms;
  uint32_t prev_ui_age_ms;
  uint32_t prev_bms_age_ms;
  uint32_t prev_free_heap;
  uint32_t prev_min_free_heap;
  uint8_t reset_reason;
  uint8_t raw_cpu0_reset_reason;
  uint8_t raw_cpu1_reset_reason;
  uint8_t prev_device_state;
  uint8_t planned_restart_reason;
  uint8_t snapshot_valid;
  uint8_t esc_connected;
  uint8_t bms_connected;
  uint8_t reserved;
  CoreDumpSummary coredump;
};

#pragma pack(pop)

struct BootDiagPacket {
  BootDiagRecord current;
  BootDiagRecord history[DIAG_HISTORY_CAPACITY];
  uint8_t history_count;
};

struct BootDiagBuildInput {
  uint32_t boot_counter;
  DiagResetReason reset_reason;
  uint8_t raw_cpu0_reset_reason;
  uint8_t raw_cpu1_reset_reason;
  PlannedRestartReason planned_restart_reason;
  RuntimeHeartbeatSnapshot snapshot;
  CoreDumpSummary coredump;
};

const char* diagResetReasonToString(DiagResetReason reason);
const char* plannedRestartReasonToString(PlannedRestartReason reason);
const char* diagDeviceStateToString(uint8_t state);

DiagResetReason diagResolveResetReason(DiagResetReason esp_reason,
                                       uint8_t raw_cpu0_reset_reason,
                                       uint8_t raw_cpu1_reset_reason);
PlannedRestartReason diagConsumePlannedRestartReason(PlannedRestartReason* marker);
BootDiagRecord diagBuildRecord(const BootDiagBuildInput& input);
void diagClearRetainedHistory(BootDiagPacket* packet);
void diagPushHistory(BootDiagPacket* packet, const BootDiagRecord& record);
void diagPacketToJson(JsonDocument& doc, const BootDiagPacket& packet);
bool diagPacketFromJson(JsonDocument& doc, BootDiagPacket* packet);

void diagnosticsInit();
void diagnosticsRefreshHeartbeat();
void diagnosticsMarkPlannedRestart(PlannedRestartReason reason);
void diagnosticsPopulatePacket(BootDiagPacket* packet);
void diagnosticsPopulateJson(JsonDocument& doc);
void diagnosticsSendJson(Stream& stream);
void diagnosticsClearPersistentData();

#ifdef PIO_UNIT_TESTING
void diagnosticsSetPacketForTest(const BootDiagPacket& packet);
#endif

#endif  // INC_SP140_DIAGNOSTICS_H_
