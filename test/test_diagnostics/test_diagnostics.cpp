#include <gtest/gtest.h>

#include <string>

#include "../native_stubs/Arduino.h"

#include "../../inc/sp140/diagnostics.h"
#include "../../src/sp140/diagnostics.cpp"

namespace {

RuntimeHeartbeatSnapshot makeSnapshot(uint32_t uptime_ms,
                                      uint8_t device_state,
                                      PlannedRestartReason planned) {
  RuntimeHeartbeatSnapshot snapshot = {};
  snapshot.magic = DIAG_SNAPSHOT_MAGIC;
  snapshot.uptime_ms = uptime_ms;
  snapshot.snapshot_ms = uptime_ms;
  snapshot.last_throttle_run_ms = uptime_ms - 25;
  snapshot.last_ui_run_ms = uptime_ms - 50;
  snapshot.last_bms_run_ms = uptime_ms - 100;
  snapshot.free_heap = 123456;
  snapshot.min_free_heap = 120000;
  snapshot.device_state = device_state;
  snapshot.esc_connected = 1;
  snapshot.bms_connected = 0;
  snapshot.planned_restart_reason = static_cast<uint8_t>(planned);
  return snapshot;
}

CoreDumpSummary makeCoreDump() {
  CoreDumpSummary summary = {};
  summary.present = 1;
  summary.valid = 1;
  summary.backtrace_depth = 3;
  strncpy(summary.task, "watchdog", sizeof(summary.task) - 1);
  summary.pc = 0x42001234;
  summary.exc_cause = 28;
  summary.exc_vaddr = 0x3FCAFE00;
  summary.backtrace[0] = 0x42001234;
  summary.backtrace[1] = 0x42004567;
  summary.backtrace[2] = 0x4200789A;
  return summary;
}

bool recordsEqual(const BootDiagRecord& lhs, const BootDiagRecord& rhs) {
  return memcmp(&lhs, &rhs, sizeof(BootDiagRecord)) == 0;
}

}  // namespace

TEST(Diagnostics, HistoryRolloverKeepsLatestSixteenRecords) {
  BootDiagPacket packet = {};

  for (uint32_t boot = 1; boot <= 18; ++boot) {
    BootDiagRecord record = {};
    record.boot_counter = boot;
    diagPushHistory(&packet, record);
  }

  ASSERT_EQ(packet.history_count, DIAG_HISTORY_CAPACITY);
  EXPECT_EQ(packet.history[0].boot_counter, 3u);
  EXPECT_EQ(packet.history[DIAG_HISTORY_CAPACITY - 1].boot_counter, 18u);
}

TEST(Diagnostics, PlannedRestartMarkerIsConsumedAndCleared) {
  PlannedRestartReason marker = PlannedRestartReason::USB_COMMAND_REBOOT;

  const PlannedRestartReason consumed = diagConsumePlannedRestartReason(&marker);

  EXPECT_EQ(consumed, PlannedRestartReason::USB_COMMAND_REBOOT);
  EXPECT_EQ(marker, PlannedRestartReason::NONE);
}

TEST(Diagnostics, JsonRoundTripPreservesCurrentAndHistory) {
  BootDiagPacket packet = {};
  packet.current.boot_counter = 42;
  packet.current.reset_reason = static_cast<uint8_t>(DiagResetReason::USB_UART);
  packet.current.raw_cpu0_reset_reason = 11;
  packet.current.raw_cpu1_reset_reason = 11;
  packet.current.prev_device_state = static_cast<uint8_t>(ARMED);
  packet.current.planned_restart_reason =
      static_cast<uint8_t>(PlannedRestartReason::NONE);
  packet.current.snapshot_valid = 1;
  packet.current.prev_uptime_ms = 3600000;
  packet.current.prev_throttle_age_ms = 25;
  packet.current.prev_ui_age_ms = 50;
  packet.current.prev_bms_age_ms = 100;
  packet.current.prev_free_heap = 123456;
  packet.current.prev_min_free_heap = 120000;
  packet.current.esc_connected = 1;
  packet.current.bms_connected = 0;
  packet.current.coredump = makeCoreDump();

  diagPushHistory(&packet, packet.current);

  BootDiagRecord older = packet.current;
  older.boot_counter = 41;
  older.reset_reason = static_cast<uint8_t>(DiagResetReason::PANIC);
  older.planned_restart_reason =
      static_cast<uint8_t>(PlannedRestartReason::BLE_UNBOND_REBOOT);
  diagPushHistory(&packet, older);

  JsonDocument doc;
  diagPacketToJson(doc, packet);

  std::string json;
  serializeJson(doc, json);

  JsonDocument parsed_doc;
  ASSERT_FALSE(deserializeJson(parsed_doc, json));

  BootDiagPacket parsed = {};
  ASSERT_TRUE(diagPacketFromJson(parsed_doc, &parsed));
  ASSERT_EQ(parsed.history_count, packet.history_count);
  EXPECT_TRUE(recordsEqual(parsed.current, packet.current));
  EXPECT_TRUE(recordsEqual(parsed.history[0], packet.history[0]));
  EXPECT_TRUE(recordsEqual(parsed.history[1], packet.history[1]));
}

TEST(Diagnostics, ResolveResetReasonUsesRawUsbResetWhenEspReasonUnknown) {
  EXPECT_EQ(diagResolveResetReason(DiagResetReason::UNKNOWN, 21, 1),
            DiagResetReason::USB_UART);
  EXPECT_EQ(diagResolveResetReason(DiagResetReason::UNKNOWN, 1, 21),
            DiagResetReason::USB_UART);
  EXPECT_EQ(diagResolveResetReason(DiagResetReason::TASK_WDT, 21, 21),
            DiagResetReason::TASK_WDT);
}

TEST(Diagnostics, ClearRetainedHistoryKeepsCurrentBootRecord) {
  BootDiagPacket packet = {};
  packet.current.boot_counter = 77;
  diagPushHistory(&packet, packet.current);
  diagPushHistory(&packet, packet.current);

  diagClearRetainedHistory(&packet);

  EXPECT_EQ(packet.current.boot_counter, 77u);
  EXPECT_EQ(packet.history_count, 0u);
}

TEST(Diagnostics, BuildRecordUsesSnapshotAndResetClassification) {
  BootDiagBuildInput input = {};
  input.boot_counter = 9;
  input.reset_reason = DiagResetReason::SOFTWARE;
  input.raw_cpu0_reset_reason = 3;
  input.raw_cpu1_reset_reason = 3;
  input.planned_restart_reason = PlannedRestartReason::USB_COMMAND_REBOOT;
  input.snapshot = makeSnapshot(7200000, static_cast<uint8_t>(ARMED_CRUISING),
                                PlannedRestartReason::USB_COMMAND_REBOOT);

  const BootDiagRecord record = diagBuildRecord(input);

  EXPECT_EQ(record.boot_counter, 9u);
  EXPECT_EQ(record.reset_reason,
            static_cast<uint8_t>(DiagResetReason::SOFTWARE));
  EXPECT_EQ(record.planned_restart_reason,
            static_cast<uint8_t>(PlannedRestartReason::USB_COMMAND_REBOOT));
  EXPECT_EQ(record.snapshot_valid, 1u);
  EXPECT_EQ(record.prev_device_state, static_cast<uint8_t>(ARMED_CRUISING));
  EXPECT_EQ(record.prev_uptime_ms, 7200000u);
  EXPECT_EQ(record.prev_throttle_age_ms, 25u);
  EXPECT_EQ(record.prev_ui_age_ms, 50u);
  EXPECT_EQ(record.prev_bms_age_ms, 100u);
  EXPECT_EQ(record.prev_free_heap, 123456u);
  EXPECT_EQ(record.prev_min_free_heap, 120000u);
  EXPECT_EQ(record.esc_connected, 1u);
  EXPECT_EQ(record.bms_connected, 0u);
}

TEST(Diagnostics, BuildRecordCarriesCrashSummaryForWatchdogFaults) {
  BootDiagBuildInput input = {};
  input.boot_counter = 10;
  input.reset_reason = DiagResetReason::TASK_WDT;
  input.raw_cpu0_reset_reason = 11;
  input.raw_cpu1_reset_reason = 11;
  input.snapshot = makeSnapshot(5400000, static_cast<uint8_t>(ARMED),
                                PlannedRestartReason::NONE);
  input.coredump = makeCoreDump();

  const BootDiagRecord record = diagBuildRecord(input);

  EXPECT_EQ(record.reset_reason, static_cast<uint8_t>(DiagResetReason::TASK_WDT));
  EXPECT_EQ(record.coredump.present, 1u);
  EXPECT_EQ(record.coredump.valid, 1u);
  EXPECT_STREQ(record.coredump.task, "watchdog");
  EXPECT_EQ(record.coredump.backtrace_depth, 3u);
  EXPECT_EQ(record.coredump.backtrace[1], 0x42004567u);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
