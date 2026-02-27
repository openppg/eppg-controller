#include "sp140/logging/telemetry_logger.h"

#include <Arduino.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <esp_partition.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "sp140/telemetry_hub.h"

namespace telemetry_log {
namespace {

constexpr uint32_t kHeaderMagic = 0x52474F4Cu; // "LOGR"
constexpr uint32_t kMetaMagic = 0x4D455441u;   // "ATEM"
constexpr uint16_t kRecordVersion = 1;
constexpr uint16_t kMetaVersion = 1;
constexpr uint32_t kMetaReservedBytes = 8192u; // Two 4KB pages
constexpr uint32_t kMetaPageSize = 4096u;
constexpr uint32_t kMetaSlotSize = 64u;
constexpr uint32_t kMetaSlotsPerPage = kMetaPageSize / kMetaSlotSize;
constexpr uint32_t kMetaSlotCount = kMetaSlotsPerPage * 2u;
constexpr size_t kRamBufferBytes = 72u * 1024u;
constexpr const char *kGapFilePrefix = "/gap_";
constexpr const char *kGapFileSuffix = ".bin";

constexpr uint32_t kEscFastBaseIntervalMs = 20u;  // 50Hz steady state
constexpr uint32_t kEscFastBurstIntervalMs = 20u; // 50Hz transient bursts
constexpr uint32_t kEscThermalIntervalMs = 500u;  // 2Hz
constexpr uint32_t kBmsMainIntervalMs = 500u;     // 2Hz
constexpr uint32_t kBmsCellsIntervalMs = 500u;    // 2Hz
constexpr uint32_t kCtrlMainIntervalMs = 1000u;   // 1Hz
constexpr uint32_t kBlackboxIntervalMs = 10000u;  // 0.1Hz
constexpr uint32_t kSessionDisarmDebounceMs = 5000u;
constexpr uint32_t kMetaFlushIntervalMs = 5000u;

struct __attribute__((packed)) TelemetryRecordHeader {
  uint32_t magic;
  uint16_t version;
  uint8_t source_type;
  uint8_t reserved0;
  uint32_t session_id;
  uint32_t seq;
  uint32_t source_ms;
  uint16_t payload_len;
  uint16_t reserved1;
  uint32_t payload_crc32;
};

struct __attribute__((packed)) RingMetaPage {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  uint32_t generation;
  uint32_t head;
  uint32_t tail;
  uint32_t used_bytes;
  uint32_t record_count;
  uint32_t last_seq;
  uint32_t last_session_id;
  uint32_t crc32;
};

struct __attribute__((packed)) MetaSlot {
  RingMetaPage meta;
  uint32_t slot_crc32;
  uint8_t padding[20];
};
static_assert(sizeof(MetaSlot) == kMetaSlotSize,
              "Meta slot size must be 64 bytes");

struct RingContext {
  const char *label;
  const esp_partition_t *part;
  RingMetaPage meta;
  uint32_t data_capacity;
  uint32_t meta_slot_index;
  bool dirty;
  unsigned long last_flush_ms;
};

struct __attribute__((packed)) EscFastPayload {
  uint16_t voltage_dV;
  int16_t current_dA;
  int32_t erpm;
  uint16_t in_pwm_us;
  uint16_t running_error;
  uint16_t selfcheck_error;
  int16_t mos_temp_dC;
};

struct __attribute__((packed)) EscThermalPayload {
  int16_t mos_temp_dC;
  int16_t cap_temp_dC;
  int16_t mcu_temp_dC;
  int16_t motor_temp_dC;
};

struct __attribute__((packed)) BmsMainPayload {
  uint16_t voltage_dV;
  int16_t current_dA;
  uint8_t soc;
  uint8_t flags;
  uint16_t highest_cell_mV;
  uint16_t lowest_cell_mV;
  uint16_t differential_mV;
  int16_t highest_temp_dC;
  int16_t lowest_temp_dC;
  uint32_t battery_cycle;
  uint8_t failure_level;
  uint8_t reserved[3];
};

struct __attribute__((packed)) BmsCellsPayload {
  uint16_t cell_mV[24];
};

struct __attribute__((packed)) CtrlMainPayload {
  int32_t altitude_cm;
  int16_t baro_temp_dC;
  int16_t vario_cmps;
  int16_t mcu_temp_dC;
  uint16_t pot_raw;
  uint32_t uptime_ms;
};

struct __attribute__((packed)) BlackboxSnapshotV2 {
  uint8_t version;
  uint8_t device_state;
  uint8_t ble_connected;
  uint8_t bms_connection_state;
  uint8_t esc_connection_state;
  uint8_t bms_soc;
  uint8_t bms_failure_level;
  uint8_t flags;

  uint16_t bms_voltage_dV;
  int16_t bms_current_dA;
  uint32_t bms_battery_cycle;

  uint16_t cell_mV[24];
  int16_t temp_dC[8];

  uint16_t esc_voltage_dV;
  int16_t esc_current_dA;
  int16_t esc_mos_temp_dC;
  int16_t esc_cap_temp_dC;
  int16_t esc_mcu_temp_dC;
  int16_t esc_motor_temp_dC;
  int32_t esc_erpm;
  uint16_t esc_in_pwm;
  uint16_t esc_running_error;

  int32_t altitude_cm;
  int16_t vario_cmps;
  int16_t baro_temp_dC;
  int16_t mcu_temp_dC;
  uint16_t throttle_raw;
  uint16_t uptime_min;
};

RingContext gBlackbox{.label = "blackbox",
                      .part = nullptr,
                      .meta = {},
                      .data_capacity = 0,
                      .meta_slot_index = 0,
                      .dirty = false,
                      .last_flush_ms = 0};

SemaphoreHandle_t gMutex = nullptr;
bool gInitialized = false;
bool gBleConnected = false;

Preferences gTelemetryPrefs;
uint32_t gSessionCounter = 0;
uint32_t gCurrentSessionId = 0;
bool gSessionActive = false;
bool gDisarmPending = false;
unsigned long gDisarmDeadlineMs = 0;

STR_ESC_TELEMETRY_140 gLastEsc = {};
STR_BMS_TELEMETRY_140 gLastBms = {};
bool gHaveEsc = false;
bool gHaveBms = false;
bool gHaveCtrl = false;
CtrlMainPayload gLastCtrl = {};

unsigned long gLastEscFastLogMs = 0;
unsigned long gLastEscThermalLogMs = 0;
unsigned long gLastBmsMainLogMs = 0;
unsigned long gLastBmsCellsLogMs = 0;
unsigned long gLastCtrlLogMs = 0;
unsigned long gLastBlackboxLogMs = 0;

unsigned long gLastEscTokenUpdateMs = 0;
float gEscBurstTokens = 200.0f;
constexpr float kEscTokenCapacity = 250.0f;
constexpr float kEscTokenRefillPerSec = 6.0f;

uint16_t gPrevEscPwm = 0;
float gPrevEscCurrent = 0.0f;
float gPrevEscRpm = 0.0f;
uint16_t gPrevEscRunErr = 0;
uint16_t gPrevEscSelfErr = 0;
bool gPrevEscValid = false;

uint8_t *gRamBuffer = nullptr;
size_t gRamHead = 0u;
size_t gRamUsed = 0u;
uint32_t gGapSeqCounter = 0u;
bool gGapFsReady = false;
File gActiveGapFile;
String gActiveGapPath;
LoggingMode gLoggingMode = LoggingMode::STREAMING;
uint8_t gCurrentDeviceState = static_cast<uint8_t>(DISARMED);

struct GapCursorRuntime {
  bool active;
  std::vector<String> files;
  size_t file_index;
  File file;
  uint32_t start_seq;
  uint32_t end_seq;
};

GapCursorRuntime gGapCursor = {};

uint32_t clampU32(int64_t value, uint32_t min_v, uint32_t max_v) {
  if (value < static_cast<int64_t>(min_v))
    return min_v;
  if (value > static_cast<int64_t>(max_v))
    return max_v;
  return static_cast<uint32_t>(value);
}

int16_t clampI16(int32_t value, int16_t min_v, int16_t max_v) {
  if (value < static_cast<int32_t>(min_v))
    return min_v;
  if (value > static_cast<int32_t>(max_v))
    return max_v;
  return static_cast<int16_t>(value);
}

uint32_t crc32Update(uint32_t crc, const uint8_t *data, size_t len) {
  crc = ~crc;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ (0xEDB88320u & static_cast<uint32_t>(
                                            -(static_cast<int32_t>(crc & 1u))));
    }
  }
  return ~crc;
}

uint32_t crc32Buffer(const void *data, size_t len) {
  return crc32Update(0u, reinterpret_cast<const uint8_t *>(data), len);
}

size_t ramTail() {
  return (gRamHead + kRamBufferBytes - gRamUsed) % kRamBufferBytes;
}

void ramWriteBytes(size_t offset, const uint8_t *data, size_t len) {
  if (len == 0u)
    return;
  const size_t first = std::min(len, kRamBufferBytes - offset);
  memcpy(&gRamBuffer[offset], data, first);
  if (len > first) {
    memcpy(&gRamBuffer[0], data + first, len - first);
  }
}

void ramReadBytes(size_t offset, uint8_t *out, size_t len) {
  if (len == 0u)
    return;
  const size_t first = std::min(len, kRamBufferBytes - offset);
  memcpy(out, &gRamBuffer[offset], first);
  if (len > first) {
    memcpy(out + first, &gRamBuffer[0], len - first);
  }
}

uint16_t ramReadU16(size_t offset) {
  uint8_t b[2] = {0u, 0u};
  ramReadBytes(offset, b, sizeof(b));
  return static_cast<uint16_t>(b[0] | (static_cast<uint16_t>(b[1]) << 8));
}

void ramWriteU16(size_t offset, uint16_t value) {
  uint8_t b[2] = {static_cast<uint8_t>(value & 0xFFu),
                  static_cast<uint8_t>((value >> 8) & 0xFFu)};
  ramWriteBytes(offset, b, sizeof(b));
}

bool pushRecordToRam(const TelemetryRecordHeader &header, const void *payload,
                     uint16_t payload_len) {
  if (gRamBuffer == nullptr)
    return false;

  const uint16_t record_len =
      static_cast<uint16_t>(sizeof(TelemetryRecordHeader) + payload_len);
  const size_t entry_len = sizeof(uint16_t) + record_len;
  if (entry_len > kRamBufferBytes)
    return false;

  while ((kRamBufferBytes - gRamUsed) < entry_len) {
    const size_t tail = ramTail();
    const uint16_t old_len = ramReadU16(tail);
    const size_t old_entry_len = sizeof(uint16_t) + old_len;
    if (old_entry_len > gRamUsed || old_entry_len == 0u) {
      gRamUsed = 0u;
      break;
    }
    gRamUsed -= old_entry_len;
  }

  ramWriteU16(gRamHead, record_len);
  gRamHead = (gRamHead + sizeof(uint16_t)) % kRamBufferBytes;
  ramWriteBytes(gRamHead, reinterpret_cast<const uint8_t *>(&header),
                sizeof(TelemetryRecordHeader));
  gRamHead = (gRamHead + sizeof(TelemetryRecordHeader)) % kRamBufferBytes;
  if (payload_len > 0u) {
    ramWriteBytes(gRamHead, reinterpret_cast<const uint8_t *>(payload),
                  payload_len);
    gRamHead = (gRamHead + payload_len) % kRamBufferBytes;
  }
  gRamUsed += entry_len;
  return true;
}

bool initGapFilesystem() {
  if (gGapFsReady)
    return true;
  gGapFsReady = LittleFS.begin(false, "/littlefs", 8, "gapfs");
  return gGapFsReady;
}

String makeGapFilePath(uint32_t session_id, uint32_t start_seq) {
  char path[48] = {};
  snprintf(path, sizeof(path), "/gap_%08lu_%010lu.bin",
           static_cast<unsigned long>(session_id),
           static_cast<unsigned long>(start_seq));
  return String(path);
}

bool isGapFilePath(const String &path) {
  if (!path.startsWith(kGapFilePrefix))
    return false;
  return path.endsWith(kGapFileSuffix);
}

std::vector<String> listGapFilesSorted() {
  std::vector<String> files;
  if (!gGapFsReady)
    return files;
  File root = LittleFS.open("/");
  if (!root)
    return files;
  File f = root.openNextFile();
  while (f) {
    if (!f.isDirectory()) {
      String name = f.name();
      if (isGapFilePath(name)) {
        files.push_back(name);
      }
    }
    f = root.openNextFile();
  }
  std::sort(files.begin(), files.end(),
            [](const String &a, const String &b) { return a < b; });
  return files;
}

bool readOneGapRecord(File &file, TelemetryRecordHeader *header,
                      uint8_t *payload_buf);

bool readGapFileSeqRange(const String &path, uint32_t *first_seq,
                         uint32_t *last_seq) {
  if (first_seq)
    *first_seq = 0u;
  if (last_seq)
    *last_seq = 0u;
  if (!gGapFsReady)
    return false;

  File f = LittleFS.open(path, "r");
  if (!f)
    return false;

  bool have_any = false;
  uint32_t first = 0u;
  uint32_t last = 0u;
  while (true) {
    TelemetryRecordHeader hdr = {};
    uint8_t payload[sizeof(ReplayFrameV1::payload)] = {};
    if (!readOneGapRecord(f, &hdr, payload)) {
      break;
    }
    if (!have_any) {
      first = hdr.seq;
      have_any = true;
    }
    last = hdr.seq;
  }

  if (!have_any)
    return false;
  if (first_seq)
    *first_seq = first;
  if (last_seq)
    *last_seq = last;
  return true;
}

void resetGapCursor() {
  if (gGapCursor.file) {
    gGapCursor.file.close();
  }
  gGapCursor = {};
}

bool openGapCursorFile(size_t index) {
  if (index >= gGapCursor.files.size())
    return false;
  if (gGapCursor.file) {
    gGapCursor.file.close();
  }
  gGapCursor.file = LittleFS.open(gGapCursor.files[index], "r");
  return static_cast<bool>(gGapCursor.file);
}

bool readOneGapRecord(File &file, TelemetryRecordHeader *header,
                      uint8_t *payload_buf) {
  if (header == nullptr || payload_buf == nullptr)
    return false;
  if (!file)
    return false;
  if (file.available() < static_cast<int>(sizeof(TelemetryRecordHeader))) {
    return false;
  }
  if (file.read(reinterpret_cast<uint8_t *>(header),
                sizeof(TelemetryRecordHeader)) !=
      static_cast<int>(sizeof(TelemetryRecordHeader))) {
    return false;
  }
  if (header->magic != kHeaderMagic || header->version != kRecordVersion ||
      header->payload_len > sizeof(ReplayFrameV1::payload)) {
    return false;
  }
  if (header->payload_len > 0u) {
    if (file.available() < header->payload_len)
      return false;
    if (file.read(payload_buf, header->payload_len) !=
        static_cast<int>(header->payload_len)) {
      return false;
    }
  }
  return true;
}

bool writeRecordToActiveGapFile(const TelemetryRecordHeader &header,
                                const void *payload) {
  if (!gGapFsReady || !gActiveGapFile)
    return false;
  const size_t wrote_header = gActiveGapFile.write(
      reinterpret_cast<const uint8_t *>(&header), sizeof(header));
  if (wrote_header != sizeof(header))
    return false;
  if (header.payload_len > 0u) {
    const size_t wrote_payload = gActiveGapFile.write(
        reinterpret_cast<const uint8_t *>(payload), header.payload_len);
    if (wrote_payload != header.payload_len)
      return false;
  }
  gActiveGapFile.flush();
  return true;
}

void snapshotRamToActiveGapFile() {
  if (!gGapFsReady || !gActiveGapFile || gRamBuffer == nullptr || gRamUsed == 0u)
    return;
  size_t offset = ramTail();
  size_t consumed = 0u;
  while (consumed < gRamUsed) {
    const uint16_t record_len = ramReadU16(offset);
    offset = (offset + sizeof(uint16_t)) % kRamBufferBytes;
    consumed += sizeof(uint16_t);
    if (record_len < sizeof(TelemetryRecordHeader) ||
        record_len > (sizeof(TelemetryRecordHeader) + sizeof(ReplayFrameV1::payload))) {
      break;
    }
    uint8_t record_buf[sizeof(TelemetryRecordHeader) + sizeof(ReplayFrameV1::payload)] = {};
    ramReadBytes(offset, record_buf, record_len);
    offset = (offset + record_len) % kRamBufferBytes;
    consumed += record_len;
    const TelemetryRecordHeader *header =
        reinterpret_cast<const TelemetryRecordHeader *>(record_buf);
    if (header->magic != kHeaderMagic || header->version != kRecordVersion) {
      continue;
    }
    const void *payload = record_buf + sizeof(TelemetryRecordHeader);
    writeRecordToActiveGapFile(*header, payload);
  }
}

void beginGapRecording() {
  gLoggingMode = LoggingMode::GAP_RECORDING;
  telemetryHubSetLoggingMode(gLoggingMode);
  if (!gGapFsReady)
    return;
  if (gActiveGapFile) {
    gActiveGapFile.close();
  }
  const uint32_t start_seq = gGapSeqCounter + 1u;
  gActiveGapPath = makeGapFilePath(gCurrentSessionId, start_seq);
  gActiveGapFile = LittleFS.open(gActiveGapPath, "w");
  if (!gActiveGapFile) {
    return;
  }
  snapshotRamToActiveGapFile();
}

void endGapRecording() {
  if (!gGapFsReady) {
    gLoggingMode = LoggingMode::STREAMING;
    telemetryHubSetLoggingMode(gLoggingMode);
    return;
  }
  if (gActiveGapFile) {
    gActiveGapFile.close();
  }
  const auto files = listGapFilesSorted();
  if (files.empty()) {
    gLoggingMode = LoggingMode::STREAMING;
  } else {
    gLoggingMode = LoggingMode::GAP_SYNCING;
  }
  telemetryHubSetLoggingMode(gLoggingMode);
}

void updateGapModeAfterDelete() {
  const auto files = listGapFilesSorted();
  if (files.empty()) {
    gLoggingMode = gBleConnected ? LoggingMode::STREAMING
                                 : LoggingMode::GAP_RECORDING;
  } else if (gBleConnected) {
    gLoggingMode = LoggingMode::GAP_SYNCING;
  } else {
    gLoggingMode = LoggingMode::GAP_RECORDING;
  }
  telemetryHubSetLoggingMode(gLoggingMode);
}

bool appendGapRecord(uint8_t source_type, uint32_t session_id, uint32_t source_ms,
                     const void *payload, uint16_t payload_len) {
  ++gGapSeqCounter;

  TelemetryRecordHeader header = {};
  header.magic = kHeaderMagic;
  header.version = kRecordVersion;
  header.source_type = source_type;
  header.session_id = session_id;
  header.seq = gGapSeqCounter;
  header.source_ms = source_ms;
  header.payload_len = payload_len;
  header.payload_crc32 = crc32Buffer(payload, payload_len);

  pushRecordToRam(header, payload, payload_len);
  if (gLoggingMode == LoggingMode::GAP_RECORDING) {
    writeRecordToActiveGapFile(header, payload);
  }
  return true;
}

void scanGapManifest(uint32_t *earliest_seq, uint32_t *latest_seq,
                     uint32_t *record_count, uint32_t *used_bytes,
                     uint32_t *file_count, uint32_t *file_bytes) {
  if (earliest_seq)
    *earliest_seq = 0u;
  if (latest_seq)
    *latest_seq = 0u;
  if (record_count)
    *record_count = 0u;
  if (used_bytes)
    *used_bytes = 0u;
  if (file_count)
    *file_count = 0u;
  if (file_bytes)
    *file_bytes = 0u;

  const auto files = listGapFilesSorted();
  if (file_count)
    *file_count = static_cast<uint32_t>(files.size());

  bool have_any = false;
  uint32_t earliest = 0u;
  uint32_t latest = 0u;
  uint32_t count = 0u;
  uint32_t bytes = 0u;

  for (const auto &path : files) {
    File f = LittleFS.open(path, "r");
    if (!f)
      continue;
    bytes += static_cast<uint32_t>(f.size());
    while (true) {
      TelemetryRecordHeader hdr = {};
      uint8_t payload[sizeof(ReplayFrameV1::payload)] = {};
      if (!readOneGapRecord(f, &hdr, payload)) {
        break;
      }
      if (!have_any) {
        earliest = hdr.seq;
        latest = hdr.seq;
        have_any = true;
      } else {
        if (hdr.seq < earliest)
          earliest = hdr.seq;
        if (hdr.seq > latest)
          latest = hdr.seq;
      }
      count += 1u;
    }
  }

  if (earliest_seq)
    *earliest_seq = earliest;
  if (latest_seq)
    *latest_seq = latest;
  if (record_count)
    *record_count = count;
  if (used_bytes)
    *used_bytes = bytes;
  if (file_bytes)
    *file_bytes = bytes;
}

uint32_t computeMetaCrc(const RingMetaPage &meta) {
  RingMetaPage copy = meta;
  copy.crc32 = 0u;
  return crc32Buffer(&copy, sizeof(copy));
}

uint32_t computeSlotCrc(const MetaSlot &slot) {
  MetaSlot copy = slot;
  copy.slot_crc32 = 0u;
  return crc32Buffer(&copy, sizeof(copy));
}

bool validateMeta(const RingContext &ctx, const RingMetaPage &meta) {
  if (meta.magic != kMetaMagic || meta.version != kMetaVersion)
    return false;
  if (meta.crc32 != computeMetaCrc(meta))
    return false;
  if (meta.head > ctx.data_capacity || meta.tail > ctx.data_capacity ||
      meta.used_bytes > ctx.data_capacity) {
    return false;
  }
  if (meta.tail != meta.used_bytes)
    return false; // linear append model
  return true;
}

bool validateSlot(const RingContext &ctx, const MetaSlot &slot) {
  if (slot.slot_crc32 != computeSlotCrc(slot))
    return false;
  return validateMeta(ctx, slot.meta);
}

void setMetaDefaults(RingMetaPage *meta) {
  memset(meta, 0, sizeof(*meta));
  meta->magic = kMetaMagic;
  meta->version = kMetaVersion;
  meta->generation = 1;
  meta->crc32 = computeMetaCrc(*meta);
}

bool writeMetaSlot(RingContext *ctx, bool erase_page_if_needed) {
  if (ctx == nullptr || ctx->part == nullptr)
    return false;

  ctx->meta.generation =
      (ctx->meta.generation == 0u) ? 1u : (ctx->meta.generation + 1u);
  ctx->meta.crc32 = computeMetaCrc(ctx->meta);

  const uint32_t next_slot = (ctx->meta_slot_index + 1u) % kMetaSlotCount;
  const uint32_t page_idx = next_slot / kMetaSlotsPerPage;
  const uint32_t slot_idx = next_slot % kMetaSlotsPerPage;
  const uint32_t page_offset = page_idx * kMetaPageSize;
  const uint32_t slot_offset = page_offset + (slot_idx * kMetaSlotSize);

  if (erase_page_if_needed && slot_idx == 0u) {
    esp_partition_erase_range(ctx->part, page_offset, kMetaPageSize);
  }

  MetaSlot slot = {};
  slot.meta = ctx->meta;
  slot.slot_crc32 = computeSlotCrc(slot);

  esp_err_t write_res =
      esp_partition_write(ctx->part, slot_offset, &slot, sizeof(slot));
  if (write_res != ESP_OK)
    return false;

  ctx->meta_slot_index = next_slot;
  ctx->dirty = false;
  ctx->last_flush_ms = millis();
  return true;
}

bool loadMeta(RingContext *ctx) {
  if (ctx == nullptr || ctx->part == nullptr)
    return false;

  bool found = false;
  RingMetaPage best_meta = {};
  uint32_t best_slot = 0u;
  uint32_t best_generation = 0u;

  for (uint32_t slot_index = 0u; slot_index < kMetaSlotCount; ++slot_index) {
    const uint32_t page_idx = slot_index / kMetaSlotsPerPage;
    const uint32_t slot_idx = slot_index % kMetaSlotsPerPage;
    const uint32_t slot_offset =
        (page_idx * kMetaPageSize) + (slot_idx * kMetaSlotSize);

    MetaSlot slot = {};
    if (esp_partition_read(ctx->part, slot_offset, &slot, sizeof(slot)) !=
        ESP_OK) {
      continue;
    }
    if (slot.meta.magic == 0xFFFFFFFFu && slot.meta.version == 0xFFFFu) {
      continue;
    }
    if (!validateSlot(*ctx, slot)) {
      continue;
    }
    if (!found || slot.meta.generation > best_generation) {
      found = true;
      best_generation = slot.meta.generation;
      best_meta = slot.meta;
      best_slot = slot_index;
    }
  }

  if (!found) {
    setMetaDefaults(&ctx->meta);
    ctx->meta_slot_index = 0u;
    esp_partition_erase_range(ctx->part, 0u, kMetaPageSize);
    MetaSlot slot = {};
    slot.meta = ctx->meta;
    slot.slot_crc32 = computeSlotCrc(slot);
    if (esp_partition_write(ctx->part, 0u, &slot, sizeof(slot)) != ESP_OK) {
      return false;
    }
    return true;
  }

  ctx->meta = best_meta;
  ctx->meta_slot_index = best_slot;
  return true;
}

bool initPartition(RingContext *ctx) {
  if (ctx == nullptr)
    return false;
  ctx->part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                       ESP_PARTITION_SUBTYPE_ANY, ctx->label);
  if (ctx->part == nullptr)
    return false;
  if (ctx->part->size <= kMetaReservedBytes)
    return false;
  ctx->data_capacity = ctx->part->size - kMetaReservedBytes;
  ctx->dirty = false;
  ctx->last_flush_ms = millis();
  return loadMeta(ctx);
}

bool readRecordHeader(const RingContext &ctx, uint32_t offset,
                      TelemetryRecordHeader *out_header) {
  if (out_header == nullptr)
    return false;
  if (offset + sizeof(TelemetryRecordHeader) > ctx.meta.used_bytes)
    return false;
  esp_err_t res = esp_partition_read(ctx.part, kMetaReservedBytes + offset,
                                     out_header, sizeof(TelemetryRecordHeader));
  if (res != ESP_OK)
    return false;
  if (out_header->magic != kHeaderMagic ||
      out_header->version != kRecordVersion)
    return false;
  if (out_header->payload_len > sizeof(ReplayFrameV1::payload))
    return false;
  const uint32_t total_len =
      sizeof(TelemetryRecordHeader) + out_header->payload_len;
  if (offset + total_len > ctx.meta.used_bytes)
    return false;
  return true;
}

bool appendRecord(RingContext *ctx, uint8_t source_type, uint32_t session_id,
                  uint32_t source_ms, const void *payload, uint16_t payload_len,
                  bool reset_on_full) {
  if (ctx == nullptr || ctx->part == nullptr)
    return false;
  const uint32_t total_len = sizeof(TelemetryRecordHeader) + payload_len;
  if (payload_len > sizeof(ReplayFrameV1::payload) ||
      total_len > ctx->data_capacity)
    return false;

  if (ctx->meta.tail + total_len > ctx->data_capacity) {
    if (!reset_on_full)
      return false;
    esp_partition_erase_range(ctx->part, kMetaReservedBytes,
                              ctx->data_capacity);
    ctx->meta.head = 0u;
    ctx->meta.tail = 0u;
    ctx->meta.used_bytes = 0u;
    ctx->meta.record_count = 0u;
    ctx->meta.last_seq = 0u;
    ctx->dirty = true;
  }

  TelemetryRecordHeader header = {};
  header.magic = kHeaderMagic;
  header.version = kRecordVersion;
  header.source_type = source_type;
  header.session_id = session_id;
  header.seq = ctx->meta.last_seq + 1u;
  header.source_ms = source_ms;
  header.payload_len = payload_len;
  header.payload_crc32 = crc32Buffer(payload, payload_len);

  const uint32_t write_offset = kMetaReservedBytes + ctx->meta.tail;
  if (esp_partition_write(ctx->part, write_offset, &header, sizeof(header)) !=
      ESP_OK) {
    return false;
  }
  if (payload_len > 0u &&
      esp_partition_write(ctx->part, write_offset + sizeof(header), payload,
                          payload_len) != ESP_OK) {
    return false;
  }

  ctx->meta.tail += total_len;
  ctx->meta.used_bytes = ctx->meta.tail;
  ctx->meta.record_count += 1u;
  ctx->meta.last_seq = header.seq;
  ctx->meta.last_session_id = session_id;
  ctx->dirty = true;
  return true;
}

void maybeFlushMeta(RingContext *ctx, bool force) {
  if (ctx == nullptr || ctx->part == nullptr || !ctx->dirty)
    return;
  const unsigned long now = millis();
  if (!force && (now - ctx->last_flush_ms) < kMetaFlushIntervalMs)
    return;
  writeMetaSlot(ctx, true);
}

void startSession() {
  gSessionCounter += 1u;
  gCurrentSessionId = gSessionCounter;
  gSessionActive = true;
  gDisarmPending = false;
  gDisarmDeadlineMs = 0u;
  gLastEscFastLogMs = 0u;
  gLastEscThermalLogMs = 0u;
  gLastBmsMainLogMs = 0u;
  gLastBmsCellsLogMs = 0u;
  gLastCtrlLogMs = 0u;
  gLastBlackboxLogMs = 0u;
  gPrevEscValid = false;
  gEscBurstTokens = kEscTokenCapacity;
  gLastEscTokenUpdateMs = millis();

  gTelemetryPrefs.putUInt("session_counter", gSessionCounter);
}

void endSession() {
  if (!gSessionActive)
    return;
  gSessionActive = false;
  gCurrentSessionId = 0u;
}

void updateEscTokens() {
  const unsigned long now = millis();
  if (gLastEscTokenUpdateMs == 0u) {
    gLastEscTokenUpdateMs = now;
    return;
  }
  const float elapsed_s =
      static_cast<float>(now - gLastEscTokenUpdateMs) / 1000.0f;
  gEscBurstTokens += elapsed_s * kEscTokenRefillPerSec;
  if (gEscBurstTokens > kEscTokenCapacity)
    gEscBurstTokens = kEscTokenCapacity;
  gLastEscTokenUpdateMs = now;
}

bool isEscTransient(const STR_ESC_TELEMETRY_140 &t) {
  if (!gPrevEscValid)
    return true;
  const uint16_t pwm = static_cast<uint16_t>(t.inPWM);
  const float rpm = t.eRPM;
  const float current = t.amps;
  if (abs(static_cast<int>(pwm) - static_cast<int>(gPrevEscPwm)) > 12)
    return true;
  if (fabsf(rpm - gPrevEscRpm) > 220.0f)
    return true;
  if (fabsf(current - gPrevEscCurrent) > 2.0f)
    return true;
  if (t.running_error != gPrevEscRunErr || t.selfcheck_error != gPrevEscSelfErr)
    return true;
  return false;
}

void updateEscPrevious(const STR_ESC_TELEMETRY_140 &t) {
  gPrevEscPwm = static_cast<uint16_t>(t.inPWM);
  gPrevEscRpm = t.eRPM;
  gPrevEscCurrent = t.amps;
  gPrevEscRunErr = t.running_error;
  gPrevEscSelfErr = t.selfcheck_error;
  gPrevEscValid = true;
}

void emitBlackboxSnapshot() {
  const unsigned long now = millis();
  if (gLastBlackboxLogMs != 0u &&
      (now - gLastBlackboxLogMs) < kBlackboxIntervalMs)
    return;

  BlackboxSnapshotV2 payload = {};
  payload.version = 2u;
  payload.device_state = gCurrentDeviceState;
  payload.ble_connected = gBleConnected ? 1u : 0u;
  payload.bms_connection_state =
      static_cast<uint8_t>(gHaveBms ? gLastBms.bmsState : TelemetryState::NOT_CONNECTED);
  payload.esc_connection_state =
      static_cast<uint8_t>(gHaveEsc ? gLastEsc.escState : TelemetryState::NOT_CONNECTED);
  if (gHaveBms) {
    payload.bms_voltage_dV =
        static_cast<uint16_t>(gLastBms.battery_voltage * 10.0f);
    payload.bms_current_dA = clampI16(
        static_cast<int32_t>(lroundf(gLastBms.battery_current * 10.0f)),
        -32768, 32767);
    payload.bms_soc = static_cast<uint8_t>(
        clampU32(static_cast<int64_t>(lroundf(gLastBms.soc)), 0u, 100u));
    payload.bms_failure_level = gLastBms.battery_fail_level;
    payload.bms_battery_cycle = gLastBms.battery_cycle;
    if (gLastBms.is_charging)
      payload.flags |= 0x01u;
    if (gLastBms.is_charge_mos)
      payload.flags |= 0x02u;
    if (gLastBms.is_discharge_mos)
      payload.flags |= 0x04u;
  }
  if (gHaveEsc) {
    payload.esc_voltage_dV = static_cast<uint16_t>(gLastEsc.volts * 10.0f);
    payload.esc_current_dA = clampI16(
        static_cast<int32_t>(lroundf(gLastEsc.amps * 10.0f)), -32768, 32767);
    payload.esc_mos_temp_dC =
        clampI16(static_cast<int32_t>(lroundf(gLastEsc.mos_temp * 10.0f)),
                 -32768, 32767);
    payload.esc_mcu_temp_dC =
        clampI16(static_cast<int32_t>(lroundf(gLastEsc.mcu_temp * 10.0f)),
                 -32768, 32767);
    payload.esc_cap_temp_dC =
        clampI16(static_cast<int32_t>(lroundf(gLastEsc.cap_temp * 10.0f)),
                 -32768, 32767);
    if (std::isnan(gLastEsc.motor_temp)) {
      payload.esc_motor_temp_dC = BLE_BMS_EXTENDED_INVALID_TEMP;
    } else {
      payload.esc_motor_temp_dC =
          clampI16(static_cast<int32_t>(lroundf(gLastEsc.motor_temp * 10.0f)),
                   -32768, 32767);
    }
    payload.esc_erpm = static_cast<int32_t>(gLastEsc.eRPM);
    payload.esc_in_pwm = static_cast<uint16_t>(
        clampU32(static_cast<int64_t>(lroundf(gLastEsc.inPWM)), 0u, 65535u));
    payload.esc_running_error = gLastEsc.running_error;
  }

  for (size_t i = 0; i < 24u; ++i) {
    payload.cell_mV[i] = 0u;
    if (gHaveBms) {
      float v = gLastBms.cell_voltages[i];
      if (v < 0.0f)
        v = 0.0f;
      if (v > 65.0f)
        v = 65.0f;
      payload.cell_mV[i] = static_cast<uint16_t>(v * 1000.0f);
    }
  }

  for (size_t i = 0; i < 8u; ++i)
    payload.temp_dC[i] = BLE_BMS_EXTENDED_INVALID_TEMP;
  if (gHaveBms) {
    payload.temp_dC[0] = clampI16(
        static_cast<int32_t>(lroundf(gLastBms.mos_temperature * 10.0f)),
        -32768, 32767);
    payload.temp_dC[1] = clampI16(
        static_cast<int32_t>(lroundf(gLastBms.balance_temperature * 10.0f)),
        -32768, 32767);
    payload.temp_dC[2] = clampI16(
        static_cast<int32_t>(lroundf(gLastBms.t1_temperature * 10.0f)), -32768,
        32767);
    payload.temp_dC[3] = clampI16(
        static_cast<int32_t>(lroundf(gLastBms.t2_temperature * 10.0f)), -32768,
        32767);
    payload.temp_dC[4] = clampI16(
        static_cast<int32_t>(lroundf(gLastBms.t3_temperature * 10.0f)), -32768,
        32767);
    payload.temp_dC[5] = clampI16(
        static_cast<int32_t>(lroundf(gLastBms.t4_temperature * 10.0f)), -32768,
        32767);
  }

  if (gHaveCtrl) {
    payload.altitude_cm = gLastCtrl.altitude_cm;
    payload.vario_cmps = gLastCtrl.vario_cmps;
    payload.baro_temp_dC = gLastCtrl.baro_temp_dC;
    payload.mcu_temp_dC = gLastCtrl.mcu_temp_dC;
    payload.throttle_raw = gLastCtrl.pot_raw;
    payload.uptime_min = static_cast<uint16_t>(
        clampU32(static_cast<int64_t>(gLastCtrl.uptime_ms / 60000u), 0u, 65535u));
  }

  if (appendRecord(&gBlackbox,
                   static_cast<uint8_t>(SourceType::BLACKBOX_SNAPSHOT),
                   gSessionActive ? gCurrentSessionId : 0u, now, &payload,
                   sizeof(payload), true)) {
    gLastBlackboxLogMs = now;
  }
}

} // namespace

void init() {
  if (gInitialized)
    return;

  gMutex = xSemaphoreCreateMutex();
  if (gMutex == nullptr)
    return;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(200)) != pdTRUE)
    return;

  bool ok = initPartition(&gBlackbox);
  if (ok) {
    if (gTelemetryPrefs.begin("telemetry", false)) {
      gSessionCounter = gTelemetryPrefs.getUInt("session_counter", 0u);
    }
    gBleConnected = false;
    gRamBuffer = static_cast<uint8_t *>(malloc(kRamBufferBytes));
    if (gRamBuffer != nullptr) {
      memset(gRamBuffer, 0, kRamBufferBytes);
    }
    gGapFsReady = initGapFilesystem();
    if (gGapFsReady) {
      uint32_t earliest = 0u;
      uint32_t latest = 0u;
      uint32_t count = 0u;
      uint32_t used = 0u;
      uint32_t files = 0u;
      uint32_t file_bytes = 0u;
      scanGapManifest(&earliest, &latest, &count, &used, &files, &file_bytes);
      gGapSeqCounter = latest;
    }
    gLoggingMode = LoggingMode::STREAMING;
    telemetryHubSetLoggingMode(gLoggingMode);
    gInitialized = true;
  }

  xSemaphoreGive(gMutex);
}

void tick() {
  if (!gInitialized)
    return;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(5)) != pdTRUE)
    return;

  const unsigned long now = millis();
  if (gDisarmPending && gSessionActive && now >= gDisarmDeadlineMs) {
    endSession();
    gDisarmPending = false;
    gDisarmDeadlineMs = 0u;
  }

  emitBlackboxSnapshot();
  maybeFlushMeta(&gBlackbox, false);

  xSemaphoreGive(gMutex);
}

void onDeviceStateChange(DeviceState oldState, DeviceState newState) {
  if (!gInitialized)
    return;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) != pdTRUE)
    return;

  const bool was_disarmed = (oldState == DISARMED);
  const bool is_armed = (newState == ARMED || newState == ARMED_CRUISING);
  gCurrentDeviceState = static_cast<uint8_t>(newState);

  if (was_disarmed && is_armed) {
    startSession();
  } else if (newState == DISARMED && oldState != DISARMED) {
    gDisarmPending = true;
    gDisarmDeadlineMs = millis() + kSessionDisarmDebounceMs;
  } else if (is_armed) {
    gDisarmPending = false;
    gDisarmDeadlineMs = 0u;
  }

  xSemaphoreGive(gMutex);
}

void onBleDisconnect() {
  if (!gInitialized)
    return;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) != pdTRUE)
    return;
  gBleConnected = false;
  beginGapRecording();
  xSemaphoreGive(gMutex);
}

void onBleReconnect() {
  if (!gInitialized)
    return;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) != pdTRUE)
    return;
  gBleConnected = true;
  endGapRecording();
  xSemaphoreGive(gMutex);
}

void logEscFast(const STR_ESC_TELEMETRY_140 &telemetry) {
  if (!gInitialized)
    return;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(3)) != pdTRUE)
    return;

  gLastEsc = telemetry;
  gHaveEsc = true;

  // Always log ESC at 50Hz, no transient token filtering required for new
  // requirement
  const unsigned long now = millis();
  uint32_t interval = kEscFastBaseIntervalMs;

  if ((gLastEscFastLogMs == 0u) || (now - gLastEscFastLogMs >= interval)) {
    EscFastPayload payload = {};
    payload.voltage_dV = static_cast<uint16_t>(telemetry.volts * 10.0f);
    payload.current_dA = clampI16(
        static_cast<int32_t>(lroundf(telemetry.amps * 10.0f)), -32768, 32767);
    payload.erpm = static_cast<int32_t>(telemetry.eRPM);
    payload.in_pwm_us = static_cast<uint16_t>(
        clampU32(static_cast<int64_t>(lroundf(telemetry.inPWM)), 0u, 65535u));
    payload.running_error = telemetry.running_error;
    payload.selfcheck_error = telemetry.selfcheck_error;
    payload.mos_temp_dC =
        clampI16(static_cast<int32_t>(lroundf(telemetry.mos_temp * 10.0f)),
                 -32768, 32767);

    if (appendGapRecord(static_cast<uint8_t>(SourceType::ESC_FAST),
                        gSessionActive ? gCurrentSessionId : 0u, now, &payload,
                        sizeof(payload))) {
      gLastEscFastLogMs = now;
    }
  }

  if ((gLastEscThermalLogMs == 0u) ||
      (now - gLastEscThermalLogMs >= kEscThermalIntervalMs)) {
    EscThermalPayload payload = {};
    payload.mos_temp_dC =
        clampI16(static_cast<int32_t>(lroundf(telemetry.mos_temp * 10.0f)),
                 -32768, 32767);
    payload.cap_temp_dC =
        clampI16(static_cast<int32_t>(lroundf(telemetry.cap_temp * 10.0f)),
                 -32768, 32767);
    payload.mcu_temp_dC =
        clampI16(static_cast<int32_t>(lroundf(telemetry.mcu_temp * 10.0f)),
                 -32768, 32767);
    if (std::isnan(telemetry.motor_temp)) {
      payload.motor_temp_dC = BLE_BMS_EXTENDED_INVALID_TEMP;
    } else {
      payload.motor_temp_dC =
          clampI16(static_cast<int32_t>(lroundf(telemetry.motor_temp * 10.0f)),
                   -32768, 32767);
    }
    if (appendGapRecord(static_cast<uint8_t>(SourceType::ESC_THERMAL),
                        gSessionActive ? gCurrentSessionId : 0u, now, &payload,
                        sizeof(payload))) {
      gLastEscThermalLogMs = now;
    }
  }

  updateEscPrevious(telemetry);
  xSemaphoreGive(gMutex);
}

void logBmsMain(const STR_BMS_TELEMETRY_140 &telemetry) {
  if (!gInitialized)
    return;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(3)) != pdTRUE)
    return;

  gLastBms = telemetry;
  gHaveBms = true;

  const unsigned long now = millis();
  // Removed kBmsMainIntervalMs interval check to log every BMS packet

  BmsMainPayload payload = {};
  payload.voltage_dV = static_cast<uint16_t>(telemetry.battery_voltage * 10.0f);
  payload.current_dA =
      clampI16(static_cast<int32_t>(lroundf(telemetry.battery_current * 10.0f)),
               -32768, 32767);
  payload.soc = static_cast<uint8_t>(
      clampU32(static_cast<int64_t>(lroundf(telemetry.soc)), 0u, 100u));
  payload.flags = 0u;
  if (telemetry.is_charging)
    payload.flags |= 0x01u;
  if (telemetry.is_charge_mos)
    payload.flags |= 0x02u;
  if (telemetry.is_discharge_mos)
    payload.flags |= 0x04u;
  payload.highest_cell_mV =
      static_cast<uint16_t>(telemetry.highest_cell_voltage * 1000.0f);
  payload.lowest_cell_mV =
      static_cast<uint16_t>(telemetry.lowest_cell_voltage * 1000.0f);
  payload.differential_mV =
      static_cast<uint16_t>(telemetry.voltage_differential * 1000.0f);
  payload.highest_temp_dC = clampI16(
      static_cast<int32_t>(lroundf(telemetry.highest_temperature * 10.0f)),
      -32768, 32767);
  payload.lowest_temp_dC = clampI16(
      static_cast<int32_t>(lroundf(telemetry.lowest_temperature * 10.0f)),
      -32768, 32767);
  payload.battery_cycle = telemetry.battery_cycle;
  payload.failure_level = telemetry.battery_fail_level;

  if (appendGapRecord(static_cast<uint8_t>(SourceType::BMS_MAIN),
                      gSessionActive ? gCurrentSessionId : 0u, now, &payload,
                      sizeof(payload))) {
    gLastBmsMainLogMs = now;
  }

  xSemaphoreGive(gMutex);
}

void logBmsCells(const STR_BMS_TELEMETRY_140 &telemetry) {
  if (!gInitialized)
    return;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(3)) != pdTRUE)
    return;

  gLastBms = telemetry;
  gHaveBms = true;

  const unsigned long now = millis();
  // Removed kBmsCellsIntervalMs interval check to log every BMS packets

  BmsCellsPayload payload = {};
  for (size_t i = 0; i < 24u; ++i) {
    float v = telemetry.cell_voltages[i];
    if (v < 0.0f)
      v = 0.0f;
    if (v > 65.0f)
      v = 65.0f;
    payload.cell_mV[i] = static_cast<uint16_t>(v * 1000.0f);
  }

  if (appendGapRecord(static_cast<uint8_t>(SourceType::BMS_CELLS24),
                      gSessionActive ? gCurrentSessionId : 0u, now, &payload,
                      sizeof(payload))) {
    gLastBmsCellsLogMs = now;
  }

  xSemaphoreGive(gMutex);
}

void logController(float altitude, float baro_temp, float vario, float mcu_temp,
                   uint16_t pot_raw, uint32_t uptime_ms) {
  if (!gInitialized)
    return;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(3)) != pdTRUE)
    return;

  CtrlMainPayload payload = {};
  payload.altitude_cm = static_cast<int32_t>(lroundf(altitude * 100.0f));
  payload.baro_temp_dC =
      clampI16(static_cast<int32_t>(lroundf(baro_temp * 10.0f)), -32768, 32767);
  payload.vario_cmps =
      clampI16(static_cast<int32_t>(lroundf(vario * 100.0f)), -32768, 32767);
  payload.mcu_temp_dC =
      clampI16(static_cast<int32_t>(lroundf(mcu_temp * 10.0f)), -32768, 32767);
  payload.pot_raw = pot_raw;
  payload.uptime_ms = uptime_ms;

  gLastCtrl = payload;
  gHaveCtrl = true;

  const unsigned long now = millis();
  if (gLastCtrlLogMs != 0u && (now - gLastCtrlLogMs) < kCtrlMainIntervalMs) {
    xSemaphoreGive(gMutex);
    return;
  }

  if (appendGapRecord(static_cast<uint8_t>(SourceType::CTRL_MAIN),
                      gSessionActive ? gCurrentSessionId : 0u, now, &payload,
                      sizeof(payload))) {
    gLastCtrlLogMs = now;
  }

  xSemaphoreGive(gMutex);
}

uint32_t currentSessionId() {
  if (!gInitialized)
    return 0u;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(3)) != pdTRUE)
    return 0u;
  const uint32_t sid = gCurrentSessionId;
  xSemaphoreGive(gMutex);
  return sid;
}

bool getManifest(Manifest *out_manifest) {
  if (!gInitialized || out_manifest == nullptr)
    return false;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(15)) != pdTRUE)
    return false;

  uint32_t gap_earliest = 0u;
  uint32_t gap_latest = 0u;
  uint32_t gap_records = 0u;
  uint32_t gap_bytes = 0u;
  uint32_t gap_file_count = 0u;
  uint32_t gap_file_bytes = 0u;
  scanGapManifest(&gap_earliest, &gap_latest, &gap_records, &gap_bytes,
                  &gap_file_count, &gap_file_bytes);

  Manifest manifest = {};
  manifest.current_session_id = gCurrentSessionId;
  manifest.earliest_seq = gap_earliest;
  manifest.latest_seq = gap_latest;
  manifest.record_count = gap_records;
  manifest.used_bytes = gap_bytes;
  manifest.gap_file_count = gap_file_count;
  manifest.gap_file_bytes = gap_file_bytes;
  if (gBlackbox.meta.record_count > 0u) {
    TelemetryRecordHeader first = {};
    if (readRecordHeader(gBlackbox, 0u, &first)) {
      manifest.blackbox_earliest_seq = first.seq;
    }
  }
  manifest.blackbox_latest_seq = gBlackbox.meta.last_seq;
  manifest.blackbox_record_count = gBlackbox.meta.record_count;

  *out_manifest = manifest;
  xSemaphoreGive(gMutex);
  return true;
}

bool openCursor(uint32_t start_seq, uint32_t end_seq,
                StreamCursor *out_cursor, uint8_t source) {
  if (!gInitialized || out_cursor == nullptr)
    return false;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) != pdTRUE)
    return false;

  StreamCursor cursor = {};
  cursor.valid = false;
  cursor.source = source;
  cursor.end_seq = end_seq;

  if (source == 0u) {
    resetGapCursor();
    const auto files = listGapFilesSorted();
    if (files.empty()) {
      xSemaphoreGive(gMutex);
      return false;
    }

    String selected_file;
    uint32_t selected_first = 0u;
    uint32_t selected_last = 0u;
    const uint32_t requested_end =
        (end_seq == 0u) ? 0xFFFFFFFFu : end_seq;

    for (const auto &path : files) {
      uint32_t file_first = 0u;
      uint32_t file_last = 0u;
      if (!readGapFileSeqRange(path, &file_first, &file_last)) {
        continue;
      }
      if (start_seq > file_last || requested_end < file_first) {
        continue;
      }
      selected_file = path;
      selected_first = file_first;
      selected_last = file_last;
      break;
    }

    if (selected_file.length() == 0) {
      xSemaphoreGive(gMutex);
      return false;
    }

    if (start_seq == 0u || start_seq < selected_first) {
      start_seq = selected_first;
    }
    if (end_seq == 0u || end_seq > selected_last) {
      end_seq = selected_last;
    }
    if (start_seq > end_seq) {
      xSemaphoreGive(gMutex);
      return false;
    }

    gGapCursor.active = true;
    gGapCursor.file_index = 0u;
    gGapCursor.files.push_back(selected_file);
    gGapCursor.start_seq = start_seq;
    gGapCursor.end_seq = end_seq;
    cursor.valid = true;
    cursor.end_seq = end_seq;
    *out_cursor = cursor;
    xSemaphoreGive(gMutex);
    return true;
  }

  const RingContext &ctx = gBlackbox;
  if (ctx.meta.record_count == 0u || ctx.meta.used_bytes == 0u) {
    xSemaphoreGive(gMutex);
    return false;
  }

  if (end_seq == 0u || end_seq > ctx.meta.last_seq)
    end_seq = ctx.meta.last_seq;
  cursor.end_seq = end_seq;

  uint32_t offset = 0u;
  while (offset < ctx.meta.used_bytes) {
    TelemetryRecordHeader header = {};
    if (!readRecordHeader(ctx, offset, &header))
      break;
    const uint32_t total_len =
        sizeof(TelemetryRecordHeader) + header.payload_len;
    if (header.seq >= start_seq) {
      cursor.valid = (header.seq <= end_seq);
      cursor.next_offset = offset;
      *out_cursor = cursor;
      xSemaphoreGive(gMutex);
      return cursor.valid;
    }
    offset += total_len;
  }

  xSemaphoreGive(gMutex);
  return false;
}

bool readCursorNext(StreamCursor *cursor, ReplayFrameV1 *out_frame) {
  if (!gInitialized || cursor == nullptr || out_frame == nullptr ||
      !cursor->valid)
    return false;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(20)) != pdTRUE)
    return false;

  if (cursor->source == 0u) {
    if (!gGapCursor.active) {
      cursor->valid = false;
      xSemaphoreGive(gMutex);
      return false;
    }
    while (true) {
      if (!gGapCursor.file) {
        if (gGapCursor.file_index >= gGapCursor.files.size() ||
            !openGapCursorFile(gGapCursor.file_index)) {
          cursor->valid = false;
          resetGapCursor();
          xSemaphoreGive(gMutex);
          return false;
        }
      }

      TelemetryRecordHeader header = {};
      uint8_t payload[sizeof(ReplayFrameV1::payload)] = {};
      if (!readOneGapRecord(gGapCursor.file, &header, payload)) {
        gGapCursor.file.close();
        gGapCursor.file_index += 1u;
        continue;
      }

      if (header.seq < gGapCursor.start_seq) {
        continue;
      }
      if (gGapCursor.end_seq != 0u && header.seq > gGapCursor.end_seq) {
        cursor->valid = false;
        resetGapCursor();
        xSemaphoreGive(gMutex);
        return false;
      }

      const uint32_t payload_crc = crc32Buffer(payload, header.payload_len);
      if (payload_crc != header.payload_crc32) {
        continue;
      }

      out_frame->version = kProtocolVersion;
      out_frame->source_type = header.source_type;
      out_frame->payload_len = header.payload_len;
      out_frame->session_id = header.session_id;
      out_frame->seq = header.seq;
      out_frame->source_ms = header.source_ms;
      if (header.payload_len > 0u) {
        memcpy(out_frame->payload, payload, header.payload_len);
      }
      xSemaphoreGive(gMutex);
      return true;
    }
  }

  const RingContext &ctx = gBlackbox;
  if (cursor->next_offset >= ctx.meta.used_bytes) {
    cursor->valid = false;
    xSemaphoreGive(gMutex);
    return false;
  }

  TelemetryRecordHeader header = {};
  if (!readRecordHeader(ctx, cursor->next_offset, &header)) {
    cursor->valid = false;
    xSemaphoreGive(gMutex);
    return false;
  }

  if (header.seq > cursor->end_seq) {
    cursor->valid = false;
    xSemaphoreGive(gMutex);
    return false;
  }

  if (header.payload_len > sizeof(out_frame->payload)) {
    cursor->valid = false;
    xSemaphoreGive(gMutex);
    return false;
  }

  const uint32_t payload_offset =
      kMetaReservedBytes + cursor->next_offset + sizeof(TelemetryRecordHeader);
  if (header.payload_len > 0u &&
      esp_partition_read(ctx.part, payload_offset, out_frame->payload,
                         header.payload_len) != ESP_OK) {
    cursor->valid = false;
    xSemaphoreGive(gMutex);
    return false;
  }

  const uint32_t payload_crc =
      crc32Buffer(out_frame->payload, header.payload_len);
  if (payload_crc != header.payload_crc32) {
    cursor->valid = false;
    xSemaphoreGive(gMutex);
    return false;
  }

  out_frame->version = kProtocolVersion;
  out_frame->source_type = header.source_type;
  out_frame->payload_len = header.payload_len;
  out_frame->session_id = header.session_id;
  out_frame->seq = header.seq;
  out_frame->source_ms = header.source_ms;

  cursor->next_offset += sizeof(TelemetryRecordHeader) + header.payload_len;
  if (cursor->next_offset >= ctx.meta.used_bytes)
    cursor->valid = false;

  xSemaphoreGive(gMutex);
  return true;
}

uint32_t getGapFileCount() {
  if (!gInitialized)
    return 0u;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(5)) != pdTRUE)
    return 0u;
  const uint32_t count = static_cast<uint32_t>(listGapFilesSorted().size());
  xSemaphoreGive(gMutex);
  return count;
}

bool deleteOldestGapFile() {
  if (!gInitialized || !gGapFsReady)
    return false;
  if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(30)) != pdTRUE)
    return false;

  const auto files = listGapFilesSorted();
  if (!files.empty()) {
    LittleFS.remove(files.front());
  }
  updateGapModeAfterDelete();
  xSemaphoreGive(gMutex);
  return true;
}

} // namespace telemetry_log
