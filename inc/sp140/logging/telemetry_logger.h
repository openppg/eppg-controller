#ifndef INC_SP140_LOGGING_TELEMETRY_LOGGER_H_
#define INC_SP140_LOGGING_TELEMETRY_LOGGER_H_

#include <cstddef>
#include <cstdint>

#include <Arduino.h>
#include "sp140/device_state.h"
#include "sp140/structs.h"

namespace telemetry_log {

enum : uint8_t {
  kProtocolVersion = 1
};

enum class SourceType : uint8_t {
  ESC_FAST = 1,
  ESC_THERMAL = 2,
  BMS_MAIN = 3,
  BMS_CELLS24 = 4,
  CTRL_MAIN = 5,
  SESSION_EVENT = 6,
  BLACKBOX_SNAPSHOT = 7
};

struct ReplayFrameV1 {
  uint8_t version;
  uint8_t source_type;
  uint16_t payload_len;
  uint32_t session_id;
  uint32_t seq;
  uint32_t source_ms;
  uint8_t payload[192];
};

struct Manifest {
  uint32_t current_session_id;
  uint32_t earliest_seq;
  uint32_t latest_seq;
  uint32_t record_count;
  uint32_t used_bytes;
  uint32_t blackbox_latest_seq;
  uint32_t blackbox_record_count;
};

struct StreamCursor {
  bool valid;
  uint32_t next_offset;
  uint32_t end_seq;
};

void init();
void tick();
void onDeviceStateChange(DeviceState oldState, DeviceState newState);

void logEscFast(const STR_ESC_TELEMETRY_140& telemetry);
void logBmsMain(const STR_BMS_TELEMETRY_140& telemetry);
void logBmsCells(const STR_BMS_TELEMETRY_140& telemetry);
void logController(float altitude, float baro_temp, float vario, float mcu_temp,
                   uint16_t pot_raw, uint32_t uptime_ms);

uint32_t currentSessionId();
bool getManifest(Manifest* out_manifest);

bool openCursor(uint32_t start_seq, uint32_t end_seq, StreamCursor* out_cursor);
bool readCursorNext(StreamCursor* cursor, ReplayFrameV1* out_frame);

}  // namespace telemetry_log

#endif  // INC_SP140_LOGGING_TELEMETRY_LOGGER_H_
