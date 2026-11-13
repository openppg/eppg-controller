#include "platform/runtime.h"
#include "sp140/diagnostics.h"

#include <new>
#include <stdio.h>
#include <string.h>

#include "sp140/structs.h"
#include "sp140/time_utils.h"

#ifndef PIO_UNIT_TESTING
#include "sp140/globals.h"
#include "platform/nvs_store.h"
#include <esp_attr.h>
#include <esp_core_dump.h>
#include <esp32s3/rom/rtc.h>
#include <esp_system.h>
#endif

extern volatile DeviceState currentState;
extern volatile uint32_t lastThrottleRunMs;
extern volatile uint32_t lastUiRunMs;
extern volatile uint32_t lastBmsRunMs;

namespace {

constexpr uint32_t kDiagMetaMagic = 0x44494147UL;
constexpr char kDiagNamespace[] = "openppg_diag";
constexpr char kDiagMetaKey[] = "meta";
constexpr char kDiagPlannedKey[] = "planned";
constexpr unsigned long kHeartbeatRefreshIntervalMs = 1000UL;
constexpr uint8_t kRawUsbUartChipResetReason = 21;
constexpr size_t kDiagSerialChunkSize = 96;

struct BootDiagMeta {
  uint32_t magic;
  uint32_t boot_counter;
  uint8_t next_slot;
  uint8_t count;
  uint16_t reserved;
};

BootDiagPacket gDiagPacket = {};

#ifndef PIO_UNIT_TESTING
platform::NvsStore gDiagPreferences;
RTC_NOINIT_ATTR RuntimeHeartbeatSnapshot gRtcHeartbeatSnapshot;
unsigned long gLastHeartbeatRefreshMs = 0;
#endif

uint8_t diagClampState(uint8_t state) {
  return state <= static_cast<uint8_t>(ARMED_CRUISING)
             ? state
             : static_cast<uint8_t>(DISARMED);
}

DiagResetReason diagResetReasonFromStoredValue(uint8_t value) {
  switch (static_cast<DiagResetReason>(value)) {
    case DiagResetReason::UNKNOWN:
    case DiagResetReason::POWERON:
    case DiagResetReason::EXT_PIN:
    case DiagResetReason::SOFTWARE:
    case DiagResetReason::PANIC:
    case DiagResetReason::INT_WDT:
    case DiagResetReason::TASK_WDT:
    case DiagResetReason::WDT:
    case DiagResetReason::DEEPSLEEP:
    case DiagResetReason::BROWNOUT:
    case DiagResetReason::SDIO:
    case DiagResetReason::USB_UART:
      return static_cast<DiagResetReason>(value);
  }
  return DiagResetReason::UNKNOWN;
}

PlannedRestartReason plannedRestartReasonFromValue(uint8_t value) {
  switch (static_cast<PlannedRestartReason>(value)) {
    case PlannedRestartReason::NONE:
    case PlannedRestartReason::USB_COMMAND_REBOOT:
    case PlannedRestartReason::BLE_UNBOND_REBOOT:
      return static_cast<PlannedRestartReason>(value);
  }
  return PlannedRestartReason::NONE;
}

uint32_t diagAgeMs(uint32_t uptime_ms, uint32_t last_run_ms) {
  if (last_run_ms > uptime_ms) {
    return 0;
  }
  return uptime_ms - last_run_ms;
}

void zeroPacket(BootDiagPacket* packet) {
  if (packet == nullptr) return;
  memset(packet, 0, sizeof(*packet));
}

void diagWriteStreamChunked(platform::Console& stream, const uint8_t* data, size_t length) {
  if (data == nullptr || length == 0) {
    return;
  }

  size_t offset = 0;
  while (offset < length) {
    const size_t remaining = length - offset;
    const size_t chunk_len =
        remaining < kDiagSerialChunkSize ? remaining : kDiagSerialChunkSize;
    const size_t written = stream.write(data + offset, chunk_len);
    if (written == 0) {
#ifndef PIO_UNIT_TESTING
      platform::delayMs(1);
#endif
      continue;
    }
    offset += written;
#ifndef PIO_UNIT_TESTING
    platform::delayMs(1);
#endif
  }
}

void diagRecordToJson(cJSON* obj, const BootDiagRecord& record) {
  platform::json::set(obj, "boot", record.boot_counter);
  platform::json::set(obj, "rst", diagResetReasonToString(
      diagResetReasonFromStoredValue(record.reset_reason)));
  platform::json::set(obj, "raw0", record.raw_cpu0_reset_reason);
  platform::json::set(obj, "raw1", record.raw_cpu1_reset_reason);
  platform::json::set(obj, "plan", plannedRestartReasonToString(
      plannedRestartReasonFromValue(record.planned_restart_reason)));
  platform::json::set(obj, "snap_ok", record.snapshot_valid == 1);
  platform::json::set(obj, "prev_state", diagDeviceStateToString(record.prev_device_state));
  platform::json::set(obj, "prev_up_ms", record.prev_uptime_ms);
  platform::json::set(obj, "thr_age_ms", record.prev_throttle_age_ms);
  platform::json::set(obj, "ui_age_ms", record.prev_ui_age_ms);
  platform::json::set(obj, "bms_age_ms", record.prev_bms_age_ms);
  platform::json::set(obj, "free_heap", record.prev_free_heap);
  platform::json::set(obj, "min_heap", record.prev_min_free_heap);
  platform::json::set(obj, "esc_ok", record.esc_connected == 1);
  platform::json::set(obj, "bms_ok", record.bms_connected == 1);

  cJSON* coredump = cJSON_AddObjectToObject(obj, "cdmp");
  platform::json::set(coredump, "present", record.coredump.present == 1);
  platform::json::set(coredump, "valid", record.coredump.valid == 1);
  if (record.coredump.present == 1) {
    platform::json::set(coredump, "task", record.coredump.task);
    platform::json::set(coredump, "pc", record.coredump.pc);
    platform::json::set(coredump, "cause", record.coredump.exc_cause);
    platform::json::set(coredump, "vaddr", record.coredump.exc_vaddr);

    cJSON* backtrace = cJSON_AddArrayToObject(coredump, "bt");
    const uint8_t depth = record.coredump.backtrace_depth <= DIAG_BACKTRACE_DEPTH
                              ? record.coredump.backtrace_depth
                              : DIAG_BACKTRACE_DEPTH;
    for (uint8_t i = 0; i < depth; ++i) {
      cJSON_AddItemToArray(backtrace, cJSON_CreateNumber(record.coredump.backtrace[i]));
    }
  }
}

bool diagRecordFromJsonObject(const cJSON* obj, BootDiagRecord* record) {
  if (record == nullptr || obj == nullptr) {
    return false;
  }

  BootDiagRecord parsed = {};
  parsed.boot_counter = platform::json::get<uint32_t>(obj, "boot", 0U);
  parsed.reset_reason = static_cast<uint8_t>(DiagResetReason::UNKNOWN);
  const char* reset_reason = platform::json::string(obj, "rst", "UNKNOWN");
  const char* planned_reason = platform::json::string(obj, "plan", "none");
  const char* prev_state = platform::json::string(obj, "prev_state", "DISARMED");

  for (uint8_t i = 0; i <= static_cast<uint8_t>(DiagResetReason::USB_UART);
       ++i) {
    if (strcmp(reset_reason,
               diagResetReasonToString(static_cast<DiagResetReason>(i))) == 0) {
      parsed.reset_reason = i;
      break;
    }
  }

  for (uint8_t i = 0;
       i <= static_cast<uint8_t>(PlannedRestartReason::BLE_UNBOND_REBOOT);
       ++i) {
    if (strcmp(planned_reason,
               plannedRestartReasonToString(
                   static_cast<PlannedRestartReason>(i))) == 0) {
      parsed.planned_restart_reason = i;
      break;
    }
  }

  if (strcmp(prev_state, "ARMED") == 0) {
    parsed.prev_device_state = static_cast<uint8_t>(ARMED);
  } else if (strcmp(prev_state, "ARMED_CRUISING") == 0) {
    parsed.prev_device_state = static_cast<uint8_t>(ARMED_CRUISING);
  } else {
    parsed.prev_device_state = static_cast<uint8_t>(DISARMED);
  }

  parsed.raw_cpu0_reset_reason = platform::json::get<uint32_t>(obj, "raw0", 0);
  parsed.raw_cpu1_reset_reason = platform::json::get<uint32_t>(obj, "raw1", 0);
  parsed.snapshot_valid = (platform::json::get<bool>(obj, "snap_ok", false)) ? 1 : 0;
  parsed.prev_uptime_ms = platform::json::get<uint32_t>(obj, "prev_up_ms", 0U);
  parsed.prev_throttle_age_ms = platform::json::get<uint32_t>(obj, "thr_age_ms", 0U);
  parsed.prev_ui_age_ms = platform::json::get<uint32_t>(obj, "ui_age_ms", 0U);
  parsed.prev_bms_age_ms = platform::json::get<uint32_t>(obj, "bms_age_ms", 0U);
  parsed.prev_free_heap = platform::json::get<uint32_t>(obj, "free_heap", 0U);
  parsed.prev_min_free_heap = platform::json::get<uint32_t>(obj, "min_heap", 0U);
  parsed.esc_connected = (platform::json::get<bool>(obj, "esc_ok", false)) ? 1 : 0;
  parsed.bms_connected = (platform::json::get<bool>(obj, "bms_ok", false)) ? 1 : 0;

  const cJSON* coredump = platform::json::field(obj, "cdmp");
  parsed.coredump.present = (platform::json::get<bool>(coredump, "present", false)) ? 1 : 0;
  parsed.coredump.valid = (platform::json::get<bool>(coredump, "valid", false)) ? 1 : 0;
  strncpy(parsed.coredump.task, platform::json::string(coredump, "task", ""),
          sizeof(parsed.coredump.task) - 1);
  parsed.coredump.pc = platform::json::get<uint32_t>(coredump, "pc", 0U);
  parsed.coredump.exc_cause = platform::json::get<uint32_t>(coredump, "cause", 0U);
  parsed.coredump.exc_vaddr = platform::json::get<uint32_t>(coredump, "vaddr", 0U);

  const cJSON* backtrace = platform::json::field(coredump, "bt");
  uint8_t depth = 0;
  for (const cJSON* value = backtrace ? backtrace->child : nullptr; value; value = value->next) {
    if (depth >= DIAG_BACKTRACE_DEPTH) break;
    parsed.coredump.backtrace[depth++] = static_cast<uint32_t>(value->valuedouble);
  }
  parsed.coredump.backtrace_depth = depth;

  *record = parsed;
  return true;
}

#ifndef PIO_UNIT_TESTING

DiagResetReason diagResetReasonFromEspReset(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:
      return DiagResetReason::POWERON;
    case ESP_RST_EXT:
      return DiagResetReason::EXT_PIN;
    case ESP_RST_SW:
      return DiagResetReason::SOFTWARE;
    case ESP_RST_PANIC:
      return DiagResetReason::PANIC;
    case ESP_RST_INT_WDT:
      return DiagResetReason::INT_WDT;
    case ESP_RST_TASK_WDT:
      return DiagResetReason::TASK_WDT;
    case ESP_RST_WDT:
      return DiagResetReason::WDT;
    case ESP_RST_DEEPSLEEP:
      return DiagResetReason::DEEPSLEEP;
    case ESP_RST_BROWNOUT:
      return DiagResetReason::BROWNOUT;
    case ESP_RST_SDIO:
      return DiagResetReason::SDIO;
    case ESP_RST_UNKNOWN:
    default:
      return DiagResetReason::UNKNOWN;
  }
}

void diagInitMeta(BootDiagMeta* meta) {
  if (meta == nullptr) return;
  memset(meta, 0, sizeof(*meta));
  meta->magic = kDiagMetaMagic;
}

void diagMakeSlotKey(uint8_t slot, char* key, size_t key_len) {
  snprintf(key, key_len, "r%02u", slot);
}

bool diagOpenPreferences() {
  return gDiagPreferences.open(kDiagNamespace, false);
}

void diagLoadMetaFromPreferences(BootDiagMeta* meta) {
  diagInitMeta(meta);
  if (meta == nullptr) return;

  const size_t stored_len = gDiagPreferences.blobSize(kDiagMetaKey);
  if (stored_len != sizeof(BootDiagMeta)) {
    return;
  }

  BootDiagMeta stored = {};
  if (gDiagPreferences.readBlob(kDiagMetaKey, &stored, sizeof(stored)) !=
      sizeof(stored)) {
    return;
  }

  if (stored.magic != kDiagMetaMagic ||
      stored.count > DIAG_HISTORY_CAPACITY ||
      stored.next_slot >= DIAG_HISTORY_CAPACITY) {
    return;
  }

  *meta = stored;
}

void diagStoreMetaToPreferences(const BootDiagMeta& meta) {
  gDiagPreferences.writeBlob(kDiagMetaKey, &meta, sizeof(meta));
}

void diagLoadHistoryFromPreferences(const BootDiagMeta& meta,
                                    BootDiagPacket* packet) {
  if (packet == nullptr) return;

  packet->history_count = 0;
  if (meta.count == 0) {
    return;
  }

  const uint8_t start_slot =
      meta.count < DIAG_HISTORY_CAPACITY ? 0 : meta.next_slot;
  for (uint8_t i = 0; i < meta.count; ++i) {
    const uint8_t slot = (start_slot + i) % DIAG_HISTORY_CAPACITY;
    char key[8] = {};
    diagMakeSlotKey(slot, key, sizeof(key));

    const size_t stored_len = gDiagPreferences.blobSize(key);
    if (stored_len != sizeof(BootDiagRecord)) {
      continue;
    }

    BootDiagRecord record = {};
    if (gDiagPreferences.readBlob(key, &record, sizeof(record)) !=
        sizeof(record)) {
      continue;
    }
    packet->history[packet->history_count++] = record;
  }
}

PlannedRestartReason diagLoadPlannedRestartFromPreferences() {
  return plannedRestartReasonFromValue(
      gDiagPreferences.get<uint8_t>(kDiagPlannedKey, 0));
}

void diagStorePlannedRestartToPreferences(PlannedRestartReason reason) {
  gDiagPreferences.set<uint8_t>(kDiagPlannedKey, static_cast<uint8_t>(reason));
}

void diagStoreRecordToPreferences(uint8_t slot, const BootDiagRecord& record) {
  char key[8] = {};
  diagMakeSlotKey(slot, key, sizeof(key));
  gDiagPreferences.writeBlob(key, &record, sizeof(record));
}

void diagCaptureHeartbeatSnapshot(PlannedRestartReason override_reason,
                                  bool force) {
  const unsigned long now = platform::millis32();
  if (!force && (now - gLastHeartbeatRefreshMs) < kHeartbeatRefreshIntervalMs) {
    return;
  }

  RuntimeHeartbeatSnapshot snapshot = {};
  snapshot.magic = DIAG_SNAPSHOT_MAGIC;
  snapshot.uptime_ms = now;
  snapshot.snapshot_ms = now;
  snapshot.last_throttle_run_ms = lastThrottleRunMs;
  snapshot.last_ui_run_ms = lastUiRunMs;
  snapshot.last_bms_run_ms = lastBmsRunMs;
  snapshot.free_heap = esp_get_free_heap_size();
  snapshot.min_free_heap = esp_get_minimum_free_heap_size();
  snapshot.device_state = diagClampState(static_cast<uint8_t>(currentState));
  snapshot.esc_connected =
      escTelemetryData.escState == TelemetryState::CONNECTED ? 1 : 0;
  snapshot.bms_connected =
      bmsTelemetryData.bmsState == TelemetryState::CONNECTED ? 1 : 0;
  const uint8_t existing_planned_restart =
      gRtcHeartbeatSnapshot.magic == DIAG_SNAPSHOT_MAGIC
          ? gRtcHeartbeatSnapshot.planned_restart_reason
          : 0;
  snapshot.planned_restart_reason =
      override_reason == PlannedRestartReason::NONE
          ? existing_planned_restart
          : static_cast<uint8_t>(override_reason);

  gRtcHeartbeatSnapshot = snapshot;
  gLastHeartbeatRefreshMs = now;
}

RuntimeHeartbeatSnapshot diagReadHeartbeatSnapshot() {
  if (gRtcHeartbeatSnapshot.magic != DIAG_SNAPSHOT_MAGIC) {
    RuntimeHeartbeatSnapshot empty = {};
    return empty;
  }
  return gRtcHeartbeatSnapshot;
}

void diagPrintBootSummary(const BootDiagRecord& record) {
  platform::console.format(
      "[DIAG] boot=%lu rst=%s plan=%s prev=%s up=%lums age=%lu/%lu/%lums "
      "esc=%u bms=%u heap=%lu/%lu cdmp=%u\n",
      static_cast<unsigned long>(record.boot_counter),
      diagResetReasonToString(diagResetReasonFromStoredValue(record.reset_reason)),
      plannedRestartReasonToString(
          plannedRestartReasonFromValue(record.planned_restart_reason)),
      diagDeviceStateToString(record.prev_device_state),
      static_cast<unsigned long>(record.prev_uptime_ms),
      static_cast<unsigned long>(record.prev_throttle_age_ms),
      static_cast<unsigned long>(record.prev_ui_age_ms),
      static_cast<unsigned long>(record.prev_bms_age_ms),
      static_cast<unsigned int>(record.esc_connected),
      static_cast<unsigned int>(record.bms_connected),
      static_cast<unsigned long>(record.prev_free_heap),
      static_cast<unsigned long>(record.prev_min_free_heap),
      static_cast<unsigned int>(record.coredump.present));

  if (record.coredump.valid == 1) {
    platform::console.format("[DIAG] coredump task=%s pc=0x%08lX cause=%lu vaddr=0x%08lX\n",
                     record.coredump.task,
                     static_cast<unsigned long>(record.coredump.pc),
                     static_cast<unsigned long>(record.coredump.exc_cause),
                     static_cast<unsigned long>(record.coredump.exc_vaddr));
  }
}

CoreDumpSummary diagReadCoreDumpSummary(DiagResetReason reason) {
  CoreDumpSummary summary = {};
  const bool should_read =
      reason == DiagResetReason::PANIC || reason == DiagResetReason::TASK_WDT ||
      reason == DiagResetReason::INT_WDT || reason == DiagResetReason::WDT;
  if (!should_read) {
    return summary;
  }

#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH && CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF
  if (esp_core_dump_image_check() != ESP_OK) {
    return summary;
  }

  summary.present = 1;
  esp_core_dump_summary_t core_dump = {};
  if (esp_core_dump_get_summary(&core_dump) == ESP_OK) {
    summary.valid = 1;
    memcpy(summary.task, core_dump.exc_task, sizeof(summary.task) - 1);
    summary.task[sizeof(summary.task) - 1] = '\0';
    summary.pc = core_dump.exc_pc;
    summary.exc_cause = core_dump.ex_info.exc_cause;
    summary.exc_vaddr = core_dump.ex_info.exc_vaddr;
    summary.backtrace_depth =
        core_dump.exc_bt_info.depth <= DIAG_BACKTRACE_DEPTH
            ? static_cast<uint8_t>(core_dump.exc_bt_info.depth)
            : DIAG_BACKTRACE_DEPTH;
    for (uint8_t i = 0; i < summary.backtrace_depth; ++i) {
      summary.backtrace[i] = core_dump.exc_bt_info.bt[i];
    }
  }
#endif
  return summary;
}

void diagShutdownHandler() { diagCaptureHeartbeatSnapshot(PlannedRestartReason::NONE, true); }

#endif  // PIO_UNIT_TESTING

}  // namespace

const char* diagResetReasonToString(DiagResetReason reason) {
  switch (reason) {
    case DiagResetReason::POWERON:
      return "POWERON";
    case DiagResetReason::EXT_PIN:
      return "EXTERNAL";
    case DiagResetReason::SOFTWARE:
      return "SOFTWARE";
    case DiagResetReason::PANIC:
      return "PANIC";
    case DiagResetReason::INT_WDT:
      return "INT_WDT";
    case DiagResetReason::TASK_WDT:
      return "TASK_WDT";
    case DiagResetReason::WDT:
      return "WDT";
    case DiagResetReason::DEEPSLEEP:
      return "DEEPSLEEP";
    case DiagResetReason::BROWNOUT:
      return "BROWNOUT";
    case DiagResetReason::SDIO:
      return "SDIO";
    case DiagResetReason::USB_UART:
      return "USB_UART";
    case DiagResetReason::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

const char* plannedRestartReasonToString(PlannedRestartReason reason) {
  switch (reason) {
    case PlannedRestartReason::USB_COMMAND_REBOOT:
      return "usb_command_reboot";
    case PlannedRestartReason::BLE_UNBOND_REBOOT:
      return "ble_unbond_reboot";
    case PlannedRestartReason::NONE:
    default:
      return "none";
  }
}

const char* diagDeviceStateToString(uint8_t state) {
  switch (diagClampState(state)) {
    case ARMED:
      return "ARMED";
    case ARMED_CRUISING:
      return "ARMED_CRUISING";
    case DISARMED:
    default:
      return "DISARMED";
  }
}

DiagResetReason diagResolveResetReason(DiagResetReason esp_reason,
                                       uint8_t raw_cpu0_reset_reason,
                                       uint8_t raw_cpu1_reset_reason) {
  if (esp_reason != DiagResetReason::UNKNOWN) {
    return esp_reason;
  }

  if (raw_cpu0_reset_reason == kRawUsbUartChipResetReason ||
      raw_cpu1_reset_reason == kRawUsbUartChipResetReason) {
    return DiagResetReason::USB_UART;
  }

  return DiagResetReason::UNKNOWN;
}

PlannedRestartReason diagConsumePlannedRestartReason(
    PlannedRestartReason* marker) {
  if (marker == nullptr) {
    return PlannedRestartReason::NONE;
  }
  const PlannedRestartReason consumed = *marker;
  *marker = PlannedRestartReason::NONE;
  return consumed;
}

BootDiagRecord diagBuildRecord(const BootDiagBuildInput& input) {
  BootDiagRecord record = {};
  record.boot_counter = input.boot_counter;
  record.reset_reason = static_cast<uint8_t>(input.reset_reason);
  record.raw_cpu0_reset_reason = input.raw_cpu0_reset_reason;
  record.raw_cpu1_reset_reason = input.raw_cpu1_reset_reason;
  record.planned_restart_reason =
      static_cast<uint8_t>(input.planned_restart_reason);
  record.prev_device_state = static_cast<uint8_t>(DISARMED);

  if (input.snapshot.magic == DIAG_SNAPSHOT_MAGIC) {
    record.snapshot_valid = 1;
    record.prev_uptime_ms = input.snapshot.uptime_ms;
    record.prev_throttle_age_ms =
        diagAgeMs(input.snapshot.uptime_ms, input.snapshot.last_throttle_run_ms);
    record.prev_ui_age_ms =
        diagAgeMs(input.snapshot.uptime_ms, input.snapshot.last_ui_run_ms);
    record.prev_bms_age_ms =
        diagAgeMs(input.snapshot.uptime_ms, input.snapshot.last_bms_run_ms);
    record.prev_free_heap = input.snapshot.free_heap;
    record.prev_min_free_heap = input.snapshot.min_free_heap;
    record.prev_device_state = diagClampState(input.snapshot.device_state);
    record.esc_connected = input.snapshot.esc_connected ? 1 : 0;
    record.bms_connected = input.snapshot.bms_connected ? 1 : 0;
  }

  record.coredump = input.coredump;
  return record;
}

void diagClearRetainedHistory(BootDiagPacket* packet) {
  if (packet == nullptr) return;
  memset(packet->history, 0, sizeof(packet->history));
  packet->history_count = 0;
}

void diagPushHistory(BootDiagPacket* packet, const BootDiagRecord& record) {
  if (packet == nullptr) return;
  if (packet->history_count < DIAG_HISTORY_CAPACITY) {
    packet->history[packet->history_count++] = record;
    return;
  }

  memmove(&packet->history[0], &packet->history[1],
          sizeof(BootDiagRecord) * (DIAG_HISTORY_CAPACITY - 1));
  packet->history[DIAG_HISTORY_CAPACITY - 1] = record;
}

void diagPacketToJson(cJSON* doc, const BootDiagPacket& packet) {
  while (doc && doc->child) cJSON_DeleteItemFromArray(doc, 0);
  platform::json::set(doc, "diag_v", DIAG_SCHEMA_VERSION);
  platform::json::set(doc, "hist_n", packet.history_count);

  cJSON* current = cJSON_AddObjectToObject(doc, "cur");
  diagRecordToJson(current, packet.current);

  cJSON* history = cJSON_AddArrayToObject(doc, "hist");
  for (uint8_t i = 0; i < packet.history_count; ++i) {
    cJSON* entry = cJSON_CreateObject();
    cJSON_AddItemToArray(history, entry);
    diagRecordToJson(entry, packet.history[i]);
  }
}

bool diagPacketFromJson(cJSON* doc, BootDiagPacket* packet) {
  if (packet == nullptr) {
    return false;
  }

  const uint8_t schema = platform::json::get<uint32_t>(doc, "diag_v", 0);
  if (schema != DIAG_SCHEMA_VERSION) {
    return false;
  }

  BootDiagPacket parsed = {};
  if (!diagRecordFromJsonObject(platform::json::field(doc, "cur"),
                                &parsed.current)) {
    return false;
  }

  const cJSON* history = platform::json::field(doc, "hist");
  for (const cJSON* entry = history ? history->child : nullptr; entry; entry = entry->next) {
    if (parsed.history_count >= DIAG_HISTORY_CAPACITY) {
      break;
    }
    if (!diagRecordFromJsonObject(entry,
                                  &parsed.history[parsed.history_count])) {
      return false;
    }
    ++parsed.history_count;
  }

  *packet = parsed;
  return true;
}

void diagnosticsInit() {
  zeroPacket(&gDiagPacket);

#ifndef PIO_UNIT_TESTING
  BootDiagMeta meta = {};
  diagInitMeta(&meta);
  PlannedRestartReason planned_restart = PlannedRestartReason::NONE;

  if (diagOpenPreferences()) {
    diagLoadMetaFromPreferences(&meta);
    diagLoadHistoryFromPreferences(meta, &gDiagPacket);
    planned_restart = diagLoadPlannedRestartFromPreferences();
    gDiagPreferences.close();
  } else {
    platform::console.line("[DIAG] Failed opening diagnostics namespace");
  }

  RuntimeHeartbeatSnapshot snapshot = diagReadHeartbeatSnapshot();
  if (planned_restart == PlannedRestartReason::NONE) {
    planned_restart =
        plannedRestartReasonFromValue(snapshot.planned_restart_reason);
  }

  const PlannedRestartReason consumed_planned_restart =
      diagConsumePlannedRestartReason(&planned_restart);
  const uint8_t raw_cpu0_reset_reason = rtc_get_reset_reason(0);
  const uint8_t raw_cpu1_reset_reason = rtc_get_reset_reason(1);
  const DiagResetReason reset_reason = diagResolveResetReason(
      diagResetReasonFromEspReset(esp_reset_reason()), raw_cpu0_reset_reason,
      raw_cpu1_reset_reason);

  BootDiagBuildInput input = {};
  input.boot_counter = meta.boot_counter + 1;
  input.reset_reason = reset_reason;
  input.raw_cpu0_reset_reason = raw_cpu0_reset_reason;
  input.raw_cpu1_reset_reason = raw_cpu1_reset_reason;
  input.planned_restart_reason = consumed_planned_restart;
  input.snapshot = snapshot;
  input.coredump = diagReadCoreDumpSummary(reset_reason);

  gDiagPacket.current = diagBuildRecord(input);
  diagPushHistory(&gDiagPacket, gDiagPacket.current);

  if (diagOpenPreferences()) {
    diagStorePlannedRestartToPreferences(PlannedRestartReason::NONE);
    diagStoreRecordToPreferences(meta.next_slot, gDiagPacket.current);
    meta.boot_counter = input.boot_counter;
    meta.next_slot = (meta.next_slot + 1) % DIAG_HISTORY_CAPACITY;
    if (meta.count < DIAG_HISTORY_CAPACITY) {
      ++meta.count;
    }
    diagStoreMetaToPreferences(meta);
    gDiagPreferences.close();
  }

  esp_register_shutdown_handler(diagShutdownHandler);
  diagPrintBootSummary(gDiagPacket.current);
#endif
}

void diagnosticsRefreshHeartbeat() {
#ifndef PIO_UNIT_TESTING
  diagCaptureHeartbeatSnapshot(PlannedRestartReason::NONE, false);
#endif
}

void diagnosticsMarkPlannedRestart(PlannedRestartReason reason) {
#ifndef PIO_UNIT_TESTING
  diagCaptureHeartbeatSnapshot(reason, true);
  if (diagOpenPreferences()) {
    diagStorePlannedRestartToPreferences(reason);
    gDiagPreferences.close();
  }
#else
  (void)reason;
#endif
}

void diagnosticsPopulatePacket(BootDiagPacket* packet) {
  if (packet == nullptr) return;
  *packet = gDiagPacket;
}

void diagnosticsPopulateJson(cJSON* doc) {
  diagPacketToJson(doc, gDiagPacket);
}

void diagnosticsSendJson(platform::Console& stream) {
  [[maybe_unused]] auto transaction = stream.transaction();
  // A full 16-record cJSON tree exceeds physical-board free RAM with BLE active.
  // Serialize one record at a time, retaining the existing packet schema.
  char output[1024];
  auto encode = [&](const BootDiagRecord& record) {
    platform::json::Document doc(cJSON_CreateObject());
    if (!doc) return false;
    diagRecordToJson(doc.get(), record);
    return cJSON_PrintPreallocated(doc.get(), output, sizeof(output), false) != 0;
  };
  if (!encode(gDiagPacket.current)) {
    stream.text("{\"diag_v\":0,\"err\":\"oom\"}");
    return;
  }
  const uint8_t count = std::min<uint8_t>(gDiagPacket.history_count, DIAG_HISTORY_CAPACITY);
  char header[64];
  const int length = snprintf(header, sizeof(header), "{\"diag_v\":%u,\"hist_n\":%u,\"cur\":",
                              DIAG_SCHEMA_VERSION, count);
  diagWriteStreamChunked(stream, reinterpret_cast<const uint8_t*>(header), length);
  diagWriteStreamChunked(stream, reinterpret_cast<const uint8_t*>(output), strlen(output));
  diagWriteStreamChunked(stream, reinterpret_cast<const uint8_t*>(",\"hist\":["), 9);
  for (uint8_t i = 0; i < count; ++i) {
    if (i) diagWriteStreamChunked(stream, reinterpret_cast<const uint8_t*>(","), 1);
    if (!encode(gDiagPacket.history[i])) snprintf(output, sizeof(output), "null");
    diagWriteStreamChunked(stream, reinterpret_cast<const uint8_t*>(output), strlen(output));
  }
  diagWriteStreamChunked(stream, reinterpret_cast<const uint8_t*>("]}"), 2);
}

void diagnosticsClearPersistentData() {
#ifndef PIO_UNIT_TESTING
  if (diagOpenPreferences()) {
    gDiagPreferences.eraseAll();
    gDiagPreferences.close();
  }
  esp_core_dump_image_erase();
  platform::console.line("[DIAG] Cleared persistent diagnostics");
#endif
  diagClearRetainedHistory(&gDiagPacket);
}

#ifdef PIO_UNIT_TESTING
void diagnosticsSetPacketForTest(const BootDiagPacket& packet) {
  gDiagPacket = packet;
}
#endif
