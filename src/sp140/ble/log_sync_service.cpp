#include "sp140/ble/log_sync_service.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <algorithm>
#include <cstring>

#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"
#include "sp140/logging/telemetry_logger.h"
#include "sp140/telemetry_hub.h"

namespace {

enum class Command : uint8_t {
  GET_MANIFEST = 1,
  START_STREAM = 2,
  ACK = 3,
  PAUSE = 4,
  RESUME = 5,
  ABORT = 6
};

enum class Response : uint8_t {
  MANIFEST = 0x81,
  STREAM_STARTED = 0x82,
  STREAM_COMPLETE = 0x83,
  ERROR = 0x84
};

enum class ErrorCode : uint8_t {
  BAD_REQUEST = 1,
  NO_DATA = 2,
  INTERNAL = 3
};

NimBLEService* pLogSyncService = nullptr;
NimBLECharacteristic* pLogSyncControl = nullptr;
NimBLECharacteristic* pLogSyncData = nullptr;
NimBLECharacteristic* pLogSyncMeta = nullptr;
NimBLECharacteristic* pLogSyncStatus = nullptr;
TaskHandle_t logSyncTaskHandle = nullptr;
SemaphoreHandle_t streamMutex = nullptr;

struct StreamState {
  bool active;
  bool paused;
  bool cursor_open;
  uint8_t source;
  uint32_t start_seq;
  uint32_t end_seq;
  uint32_t ack_seq;
  uint32_t highest_sent_seq;
  uint32_t next_seq;
  unsigned long last_ack_ms;
  unsigned long last_meta_ms;
  telemetry_log::StreamCursor cursor;
};

StreamState gStream = {};

constexpr size_t kMaxFrameBytes = 220;
constexpr uint32_t kWindowSize = 12u;
constexpr uint32_t kAckTimeoutMs = 3000u;
constexpr uint32_t kStatusPublishIntervalMs = 250u;

void writeU32LE(uint8_t* dst, uint32_t value) {
  dst[0] = static_cast<uint8_t>(value & 0xFFu);
  dst[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
  dst[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
  dst[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

uint32_t readU32LE(const uint8_t* src) {
  return (static_cast<uint32_t>(src[0])) |
         (static_cast<uint32_t>(src[1]) << 8) |
         (static_cast<uint32_t>(src[2]) << 16) |
         (static_cast<uint32_t>(src[3]) << 24);
}

void sendControlResponse(const uint8_t* data, size_t len) {
  if (pLogSyncControl == nullptr) return;
  pLogSyncControl->setValue(data, len);
  if (deviceConnected) {
    pLogSyncControl->indicate();
  }
}

uint8_t buildStatusFlags(const telemetry_log::Manifest& manifest) {
  uint8_t flags = 0u;
  const LoggingMode mode = telemetryHubGetLoggingMode();
  const bool backfill_pending = manifest.gap_file_count > 0u;
  const bool backfill_in_progress = gStream.active && (gStream.source == 0u);
  const bool live_connected = deviceConnected;

  if (live_connected) flags |= (1u << 0);            // BLE connected
  if (backfill_pending) flags |= (1u << 1);          // gap backlog pending
  if (backfill_in_progress) flags |= (1u << 2);      // gap stream in progress
  if (mode == LoggingMode::GAP_RECORDING) flags |= (1u << 3);
  if (mode == LoggingMode::GAP_SYNCING) flags |= (1u << 4);
  // Live telemetry means connected and not in disconnect-recording mode.
  if (live_connected && mode != LoggingMode::GAP_RECORDING) flags |= (1u << 5);
  // Backfill complete means no pending backlog and no active gap stream.
  if (!backfill_pending && !backfill_in_progress) flags |= (1u << 6);
  return flags;
}

void publishStatus() {
  if (pLogSyncStatus == nullptr) return;

  telemetry_log::Manifest manifest = {};
  if (!telemetry_log::getManifest(&manifest)) return;

  // LOG_SYNC_STATUS_UUID payload (36 bytes):
  // [0]=version, [1]=logging_mode, [2]=stream_active, [3]=stream_source,
  // [4]=flags, [8..11]=session_id, [12..15]=gap_file_count,
  // [16..19]=gap_file_bytes, [20..23]=gap_earliest_seq,
  // [24..27]=gap_latest_seq, [28..31]=backfill_ack_seq,
  // [32..35]=backfill_end_seq.
  uint8_t payload[36] = {};
  payload[0] = telemetry_log::kProtocolVersion;
  payload[1] = static_cast<uint8_t>(telemetryHubGetLoggingMode());
  payload[2] = gStream.active ? 1u : 0u;
  payload[3] = gStream.active ? gStream.source : 0xFFu;
  payload[4] = buildStatusFlags(manifest);
  payload[5] = 0u;
  payload[6] = 0u;
  payload[7] = 0u;
  writeU32LE(&payload[8], manifest.current_session_id);
  writeU32LE(&payload[12], manifest.gap_file_count);
  writeU32LE(&payload[16], manifest.gap_file_bytes);
  writeU32LE(&payload[20], manifest.earliest_seq);
  writeU32LE(&payload[24], manifest.latest_seq);
  writeU32LE(&payload[28], gStream.ack_seq);
  writeU32LE(&payload[32], gStream.end_seq);

  pLogSyncStatus->setValue(payload, sizeof(payload));
  if (deviceConnected) {
    pLogSyncStatus->notify();
  }
}

void publishMeta() {
  if (pLogSyncMeta == nullptr) return;

  uint8_t payload[32] = {};
  payload[0] = telemetry_log::kProtocolVersion;
  payload[1] = gStream.active ? 1u : 0u;
  payload[2] = gStream.paused ? 1u : 0u;
  payload[3] = gStream.source;
  writeU32LE(&payload[4], telemetry_log::currentSessionId());
  writeU32LE(&payload[8], gStream.start_seq);
  writeU32LE(&payload[12], gStream.end_seq);
  writeU32LE(&payload[16], gStream.ack_seq);
  writeU32LE(&payload[20], gStream.highest_sent_seq);
  writeU32LE(&payload[24], gStream.next_seq);
  writeU32LE(&payload[28], deviceConnected ? 1u : 0u);

  pLogSyncMeta->setValue(payload, sizeof(payload));
  if (deviceConnected) {
    pLogSyncMeta->notify();
  }
}

void sendManifest() {
  telemetry_log::Manifest manifest = {};
  if (!telemetry_log::getManifest(&manifest)) {
    uint8_t err[6] = {static_cast<uint8_t>(Response::ERROR), static_cast<uint8_t>(ErrorCode::INTERNAL), 0, 0, 0, 0};
    sendControlResponse(err, sizeof(err));
    return;
  }

  uint8_t payload[46] = {};
  payload[0] = static_cast<uint8_t>(Response::MANIFEST);
  payload[1] = telemetry_log::kProtocolVersion;
  writeU32LE(&payload[2], manifest.current_session_id);
  writeU32LE(&payload[6], manifest.earliest_seq);
  writeU32LE(&payload[10], manifest.latest_seq);
  writeU32LE(&payload[14], manifest.record_count);
  writeU32LE(&payload[18], manifest.used_bytes);
  writeU32LE(&payload[22], manifest.blackbox_earliest_seq);
  writeU32LE(&payload[26], manifest.blackbox_latest_seq);
  writeU32LE(&payload[30], manifest.blackbox_record_count);
  writeU32LE(&payload[34], manifest.gap_file_count);
  writeU32LE(&payload[38], manifest.gap_file_bytes);
  writeU32LE(&payload[42], gStream.active ? 1u : 0u);
  sendControlResponse(payload, sizeof(payload));
  publishMeta();
  publishStatus();
}

void stopStream(bool send_complete) {
  const uint8_t source = gStream.source;
  uint32_t final_seq = gStream.ack_seq;
  gStream = {};
  if (send_complete && source == 0u) {
    telemetry_log::deleteOldestGapFile();
  }
  if (send_complete) {
    uint8_t payload[5] = {static_cast<uint8_t>(Response::STREAM_COMPLETE), 0, 0, 0, 0};
    writeU32LE(&payload[1], final_seq);
    sendControlResponse(payload, sizeof(payload));
  }
  publishMeta();
  publishStatus();
}

void startStream(uint32_t start_seq, uint32_t end_seq, uint8_t source) {
  telemetry_log::Manifest manifest = {};
  if (!telemetry_log::getManifest(&manifest)) {
    uint8_t err[6] = {static_cast<uint8_t>(Response::ERROR), static_cast<uint8_t>(ErrorCode::NO_DATA), 0, 0, 0, 0};
    sendControlResponse(err, sizeof(err));
    return;
  }

  const bool stream_blackbox = (source == 1u);
  if (!stream_blackbox &&
      (manifest.record_count == 0u || manifest.latest_seq == 0u)) {
    uint8_t err[6] = {static_cast<uint8_t>(Response::ERROR), static_cast<uint8_t>(ErrorCode::NO_DATA), 0, 0, 0, 0};
    sendControlResponse(err, sizeof(err));
    return;
  }
  if (stream_blackbox && manifest.blackbox_latest_seq == 0u) {
    uint8_t err[6] = {static_cast<uint8_t>(Response::ERROR), static_cast<uint8_t>(ErrorCode::NO_DATA), 0, 0, 0, 0};
    sendControlResponse(err, sizeof(err));
    return;
  }

  if (stream_blackbox) {
    if (start_seq == 0u) start_seq = 1u;
    if (end_seq == 0u || end_seq > manifest.blackbox_latest_seq) {
      end_seq = manifest.blackbox_latest_seq;
    }
  } else {
    if (start_seq == 0u || start_seq < manifest.earliest_seq) {
      start_seq = manifest.earliest_seq;
    }
    if (end_seq == 0u || end_seq > manifest.latest_seq) {
      end_seq = manifest.latest_seq;
    }
  }

  if (start_seq > end_seq) {
    uint8_t err[6] = {static_cast<uint8_t>(Response::ERROR), static_cast<uint8_t>(ErrorCode::BAD_REQUEST), 0, 0, 0, 0};
    sendControlResponse(err, sizeof(err));
    return;
  }

  telemetry_log::StreamCursor cursor = {};
  if (!telemetry_log::openCursor(start_seq, end_seq, &cursor, source)) {
    uint8_t err[6] = {static_cast<uint8_t>(Response::ERROR), static_cast<uint8_t>(ErrorCode::NO_DATA), 0, 0, 0, 0};
    sendControlResponse(err, sizeof(err));
    return;
  }
  end_seq = cursor.end_seq;

  gStream = {};
  gStream.active = true;
  gStream.paused = false;
  gStream.cursor_open = true;
  gStream.source = source;
  gStream.start_seq = start_seq;
  gStream.end_seq = end_seq;
  gStream.ack_seq = (start_seq > 0u) ? (start_seq - 1u) : 0u;
  gStream.highest_sent_seq = gStream.ack_seq;
  gStream.next_seq = start_seq;
  gStream.last_ack_ms = millis();
  gStream.last_meta_ms = millis();
  gStream.cursor = cursor;

  uint8_t payload[10] = {
      static_cast<uint8_t>(Response::STREAM_STARTED), 0, 0, 0, 0,
      0,                                           0, 0, 0, source};
  writeU32LE(&payload[1], start_seq);
  writeU32LE(&payload[5], end_seq);
  sendControlResponse(payload, sizeof(payload));
  publishMeta();
  publishStatus();
}

bool sendOneFrame() {
  if (!gStream.cursor_open || !gStream.active || gStream.paused) return false;
  telemetry_log::ReplayFrameV1 frame = {};
  if (!telemetry_log::readCursorNext(&gStream.cursor, &frame)) {
    gStream.cursor_open = false;
    return false;
  }

  const size_t total_len = 1u + 1u + 2u + 4u + 4u + 4u + frame.payload_len;
  if (total_len > kMaxFrameBytes || pLogSyncData == nullptr) return false;

  uint8_t bytes[kMaxFrameBytes] = {};
  bytes[0] = frame.version;
  bytes[1] = frame.source_type;
  bytes[2] = static_cast<uint8_t>(frame.payload_len & 0xFFu);
  bytes[3] = static_cast<uint8_t>((frame.payload_len >> 8) & 0xFFu);
  writeU32LE(&bytes[4], frame.session_id);
  writeU32LE(&bytes[8], frame.seq);
  writeU32LE(&bytes[12], frame.source_ms);
  if (frame.payload_len > 0u) {
    memcpy(&bytes[16], frame.payload, frame.payload_len);
  }

  pLogSyncData->setValue(bytes, total_len);
  if (deviceConnected) {
    pLogSyncData->notify();
  }

  gStream.highest_sent_seq = std::max(gStream.highest_sent_seq, frame.seq);
  gStream.next_seq = frame.seq + 1u;
  return true;
}

void ensureCursorOpenFromNextSeq() {
  if (!gStream.active) return;
  if (gStream.cursor_open) return;
  telemetry_log::StreamCursor cursor = {};
  if (telemetry_log::openCursor(gStream.next_seq, gStream.end_seq, &cursor,
                                gStream.source)) {
    gStream.cursor = cursor;
    gStream.cursor_open = true;
  }
}

void logSyncTask(void* pv) {
  (void)pv;
  unsigned long last_status_ms = 0u;
  for (;;) {
    if (streamMutex != nullptr && xSemaphoreTake(streamMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      if (gStream.active && !gStream.paused) {
        if (gStream.ack_seq >= gStream.end_seq) {
          stopStream(true);
        } else {
          const unsigned long now = millis();
          if ((gStream.highest_sent_seq > gStream.ack_seq) &&
              (now - gStream.last_ack_ms > kAckTimeoutMs)) {
            gStream.next_seq = gStream.ack_seq + 1u;
            gStream.highest_sent_seq = gStream.ack_seq;
            gStream.cursor_open = false;
            gStream.last_ack_ms = now;
          }

          ensureCursorOpenFromNextSeq();
          uint32_t sent_this_tick = 0u;
          while (gStream.active && !gStream.paused &&
                 (gStream.highest_sent_seq - gStream.ack_seq) < kWindowSize &&
                 sent_this_tick < 4u) {
            if (!sendOneFrame()) break;
            sent_this_tick += 1u;
          }
        }
      }

      const unsigned long now = millis();
      if (now - gStream.last_meta_ms > 500u) {
        publishMeta();
        gStream.last_meta_ms = now;
      }
      if (now - last_status_ms > kStatusPublishIntervalMs) {
        publishStatus();
        last_status_ms = now;
      }

      xSemaphoreGive(streamMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

class LogSyncControlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    const std::string value = characteristic->getValue();
    if (value.empty()) return;

    if (streamMutex == nullptr || xSemaphoreTake(streamMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
      return;
    }

    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(value.data());
    const size_t len = value.size();
    const Command cmd = static_cast<Command>(bytes[0]);

    switch (cmd) {
      case Command::GET_MANIFEST:
        sendManifest();
        break;
      case Command::START_STREAM: {
        if (len < 9u) {
          uint8_t err[6] = {static_cast<uint8_t>(Response::ERROR), static_cast<uint8_t>(ErrorCode::BAD_REQUEST), 0, 0, 0, 0};
          sendControlResponse(err, sizeof(err));
          break;
        }
        const uint32_t start_seq = readU32LE(&bytes[1]);
        const uint32_t end_seq = readU32LE(&bytes[5]);
        const uint8_t source = (len >= 10u) ? bytes[9] : 1u;
        startStream(start_seq, end_seq, source);
      } break;
      case Command::ACK:
        if (len >= 5u && gStream.active) {
          const uint32_t ack_seq = readU32LE(&bytes[1]);
          if (ack_seq >= gStream.ack_seq) {
            gStream.ack_seq = std::min(ack_seq, gStream.end_seq);
            gStream.last_ack_ms = millis();
          }
        }
        break;
      case Command::PAUSE:
        gStream.paused = true;
        publishMeta();
        publishStatus();
        break;
      case Command::RESUME:
        gStream.paused = false;
        gStream.last_ack_ms = millis();
        publishMeta();
        publishStatus();
        break;
      case Command::ABORT:
        stopStream(false);
        break;
      default: {
        uint8_t err[6] = {static_cast<uint8_t>(Response::ERROR), static_cast<uint8_t>(ErrorCode::BAD_REQUEST), 0, 0, 0, 0};
        sendControlResponse(err, sizeof(err));
      } break;
    }

    xSemaphoreGive(streamMutex);
  }
};

}  // namespace

void initLogSyncBleService(NimBLEServer* server) {
  if (pLogSyncService != nullptr) return;

  streamMutex = xSemaphoreCreateMutex();
  if (streamMutex == nullptr) return;

  pLogSyncService = server->createService(NimBLEUUID(LOG_SYNC_SERVICE_UUID));
  pLogSyncControl = pLogSyncService->createCharacteristic(
      NimBLEUUID(LOG_SYNC_CONTROL_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
  pLogSyncData = pLogSyncService->createCharacteristic(
      NimBLEUUID(LOG_SYNC_DATA_UUID),
      NIMBLE_PROPERTY::NOTIFY);
  pLogSyncMeta = pLogSyncService->createCharacteristic(
      NimBLEUUID(LOG_SYNC_META_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pLogSyncStatus = pLogSyncService->createCharacteristic(
      NimBLEUUID(LOG_SYNC_STATUS_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  static LogSyncControlCallbacks callbacks;
  pLogSyncControl->setCallbacks(&callbacks);

  uint8_t initial[2] = {telemetry_log::kProtocolVersion, 0u};
  if (pLogSyncControl != nullptr) pLogSyncControl->setValue(initial, sizeof(initial));
  if (pLogSyncMeta != nullptr) pLogSyncMeta->setValue(initial, sizeof(initial));
  if (pLogSyncStatus != nullptr) pLogSyncStatus->setValue(initial, sizeof(initial));
  pLogSyncService->start();

  xTaskCreate(logSyncTask, "LogSyncTask", 6144, nullptr, 1, &logSyncTaskHandle);
  publishStatus();
}
