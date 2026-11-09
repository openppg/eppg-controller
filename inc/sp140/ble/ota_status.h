#ifndef INC_SP140_BLE_OTA_STATUS_H_
#define INC_SP140_BLE_OTA_STATUS_H_

#include <stdint.h>

enum class OtaPhase : uint8_t {
  Idle, Preparing, Receiving, Verifying, Restarting, Complete, Failed
};

enum class OtaError : uint8_t {
  None, Disconnected, Timeout, NoPartition, InvalidSize, Begin,
  Write, SizeMismatch, Verification, BootPartition, RolledBack
};

// Plain value snapshot: no BLE, flash handles or LVGL objects cross tasks.
struct OtaStatus {
  OtaPhase phase = OtaPhase::Idle;
  OtaError error = OtaError::None;
  uint32_t written = 0;
  uint32_t total = 0;
  uint32_t revision = 0;
  uint32_t changedAtMs = 0;
};

inline unsigned otaProgressPercent(const OtaStatus& status) {
  if (status.total == 0) return 0;
  const uint64_t percent = uint64_t(status.written) * 100 / status.total;
  return percent > 100 ? 100 : static_cast<unsigned>(percent);
}

inline bool otaStatusVisible(const OtaStatus& status, uint32_t now) {
  if (status.phase == OtaPhase::Idle) return false;
  const uint32_t elapsed = now - status.changedAtMs;
  if (status.phase == OtaPhase::Failed) return elapsed < 10000;
  if (status.phase == OtaPhase::Complete) return elapsed < 3500;
  return true;
}

#endif  // INC_SP140_BLE_OTA_STATUS_H_
