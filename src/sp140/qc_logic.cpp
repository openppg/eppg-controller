// Copyright 2026 <Zach Whitehead>
// OpenPPG
//
// FIRST_BOOT_QC pure logic. Deliberately free of Arduino/FreeRTOS/NVS so the
// native GoogleTest suite (test/test_qc) exercises the real implementation.

#include "sp140/qc_logic.h"

#include <stdio.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Boot gate decision
// ---------------------------------------------------------------------------

QcGateAction qcGateDecision(bool factoryQcPassed,
                            bool factoryRerunRequested,
                            bool userSettingsPresent) {
  // A deliberate serial-command rerun overrides everything (bench/service).
  if (factoryRerunRequested) {
    return QcGateAction::RUN_QC_RERUN;
  }
  // Already QC'd — normal boot.
  if (factoryQcPassed) {
    return QcGateAction::SKIP_NORMAL_BOOT;
  }
  // Existing unit (settings written by v8.0-or-prior firmware): back-fill the
  // pass flag and never auto-calibrate. The installed fleet must never see QC.
  if (userSettingsPresent) {
    return QcGateAction::MARK_LEGACY_AND_SKIP;
  }
  // Truly fresh NVS: brand-new factory controller.
  return QcGateAction::RUN_QC;
}

// ---------------------------------------------------------------------------
// Throttle calibration sanity gates
// ---------------------------------------------------------------------------

QcCalResult qcValidateCalibration(uint16_t rawMin, uint16_t rawMax,
                                  uint16_t releaseRecheck,
                                  const QcCalGates& gates) {
  if (rawMax <= rawMin || (uint16_t)(rawMax - rawMin) < gates.minSpan) {
    return QcCalResult::SPAN_TOO_SMALL;
  }
  if (rawMin > gates.maxIdle) {
    return QcCalResult::IDLE_TOO_HIGH;
  }
  if (rawMax < gates.minFull) {
    return QcCalResult::FULL_TOO_LOW;
  }
  const uint16_t diff = (releaseRecheck > rawMin)
                            ? (releaseRecheck - rawMin)
                            : (rawMin - releaseRecheck);
  if (diff > gates.releaseTolerance) {
    return QcCalResult::RELEASE_MISMATCH;
  }
  return QcCalResult::OK;
}

// ---------------------------------------------------------------------------
// Stability window detector
// ---------------------------------------------------------------------------

QcStabilityWindow::QcStabilityWindow(uint16_t epsilon, size_t windowSize)
    : size_(windowSize == 0 ? 1 : (windowSize > kMaxWindow ? kMaxWindow : windowSize)),
      count_(0),
      head_(0),
      epsilon_(epsilon) {
  memset(buf_, 0, sizeof(buf_));
}

void QcStabilityWindow::push(uint16_t raw) {
  buf_[head_] = raw;
  head_ = (head_ + 1) % size_;
  if (count_ < size_) {
    count_++;
  }
}

void QcStabilityWindow::reset() {
  count_ = 0;
  head_ = 0;
}

bool QcStabilityWindow::isStable() const {
  if (count_ < size_) {
    return false;
  }
  uint16_t lo = buf_[0];
  uint16_t hi = buf_[0];
  for (size_t i = 1; i < size_; i++) {
    if (buf_[i] < lo) lo = buf_[i];
    if (buf_[i] > hi) hi = buf_[i];
  }
  return (uint16_t)(hi - lo) <= epsilon_;
}

uint16_t QcStabilityWindow::median() const {
  if (count_ == 0) {
    return 0;
  }
  const size_t n = (count_ < size_) ? count_ : size_;
  uint16_t sorted[kMaxWindow];
  memcpy(sorted, buf_, n * sizeof(uint16_t));
  // Insertion sort — n <= 64.
  for (size_t i = 1; i < n; i++) {
    const uint16_t key = sorted[i];
    size_t j = i;
    while (j > 0 && sorted[j - 1] > key) {
      sorted[j] = sorted[j - 1];
      j--;
    }
    sorted[j] = key;
  }
  return sorted[n / 2];
}

// ---------------------------------------------------------------------------
// v8.2 scaffold: calibrated raw->PWM mapping (unwired in v8.1)
// ---------------------------------------------------------------------------

int qcPotRawToPwmCalibrated(uint16_t raw,
                            uint16_t potMin, uint16_t potMax,
                            int escMinPwm, int escMaxPwm,
                            float bottomPct, float topPct,
                            uint16_t floorDb) {
  if (potMax <= potMin) {
    return escMinPwm;  // degenerate calibration — always safe idle
  }
  const float span = static_cast<float>(potMax - potMin);

  float bottomDb = bottomPct * span;
  if (bottomDb < static_cast<float>(floorDb)) {
    bottomDb = static_cast<float>(floorDb);
  }
  const float topMargin = topPct * span;

  const float effMin = static_cast<float>(potMin) + bottomDb;
  const float effMax = static_cast<float>(potMax) - topMargin;
  if (effMax <= effMin) {
    return escMinPwm;  // margins collapsed the range — safe idle
  }

  float r = static_cast<float>(raw);
  if (r < effMin) r = effMin;
  if (r > effMax) r = effMax;

  const float frac = (r - effMin) / (effMax - effMin);
  const int pwm = escMinPwm +
                  static_cast<int>(frac * static_cast<float>(escMaxPwm - escMinPwm) + 0.5f);
  if (pwm < escMinPwm) return escMinPwm;
  if (pwm > escMaxPwm) return escMaxPwm;
  return pwm;
}

// ---------------------------------------------------------------------------
// QC record + JSON
// ---------------------------------------------------------------------------

const char* qcCheckStatusStr(QcCheckStatus s) {
  switch (s) {
    case QcCheckStatus::PASS: return "pass";
    case QcCheckStatus::FAIL: return "fail";
    case QcCheckStatus::SKIP: return "skip";
    case QcCheckStatus::NOT_RUN:
    default:
      return "not_run";
  }
}

static bool checkOk(QcCheckStatus s) {
  return s == QcCheckStatus::PASS || s == QcCheckStatus::SKIP;
}

bool qcRecordAllPassed(const QcRecord& r) {
  return checkOk(r.display) && checkOk(r.i2cBaro) && checkOk(r.spiBms) &&
         checkOk(r.canEsc) && checkOk(r.canBms) && checkOk(r.cpu) &&
         checkOk(r.nvs) && checkOk(r.throttle) && checkOk(r.cal) &&
         checkOk(r.buzzer) && checkOk(r.vibe) && checkOk(r.button);
}

// Append helper: writes either "null" or a quoted string.
static int appendIdField(char* out, size_t remaining, const char* key,
                         const char* value, bool trailingComma) {
  if (value[0] == '\0') {
    return snprintf(out, remaining, "\"%s\":null%s", key, trailingComma ? "," : "");
  }
  return snprintf(out, remaining, "\"%s\":\"%s\"%s", key, value, trailingComma ? "," : "");
}

size_t qcRecordToJson(const QcRecord& r, char* out, size_t outLen) {
  if (out == nullptr || outLen == 0) {
    return 0;
  }
  const uint16_t span = (r.potMax > r.potMin) ? (r.potMax - r.potMin) : 0;
  size_t pos = 0;

  int n = snprintf(out + pos, outLen - pos,
                   "{\"qc\":1,\"fw\":\"%s\",\"build\":\"%s\",\"result\":\"%s\","
                   "\"pot_min\":%u,\"pot_max\":%u,\"span\":%u,\"cal_saved\":%s,"
                   "\"baro_hpa\":%.1f,\"cpu_c\":%.1f,\"pack_v\":%.1f,",
                   r.fw, r.build,
                   qcRecordAllPassed(r) ? "PASSED" : "FAILED",
                   r.potMin, r.potMax, span, r.calSaved ? "true" : "false",
                   static_cast<double>(r.baroHpa),
                   static_cast<double>(r.cpuC),
                   static_cast<double>(r.packV));
  if (n < 0 || (size_t)n >= outLen - pos) return 0;
  pos += (size_t)n;

  n = appendIdField(out + pos, outLen - pos, "esc_hw_id", r.escHwId, true);
  if (n < 0 || (size_t)n >= outLen - pos) return 0;
  pos += (size_t)n;
  n = appendIdField(out + pos, outLen - pos, "esc_sn", r.escSn, true);
  if (n < 0 || (size_t)n >= outLen - pos) return 0;
  pos += (size_t)n;
  n = appendIdField(out + pos, outLen - pos, "bms_id", r.bmsId, true);
  if (n < 0 || (size_t)n >= outLen - pos) return 0;
  pos += (size_t)n;

  n = snprintf(out + pos, outLen - pos,
               "\"checks\":{\"display\":\"%s\",\"i2c_baro\":\"%s\",\"spi_bms\":\"%s\","
               "\"can_esc\":\"%s\",\"can_bms\":\"%s\",\"cpu\":\"%s\",\"nvs\":\"%s\","
               "\"throttle\":\"%s\",\"cal\":\"%s\",\"buzzer\":\"%s\",\"vibe\":\"%s\","
               "\"button\":\"%s\"}}",
               qcCheckStatusStr(r.display), qcCheckStatusStr(r.i2cBaro),
               qcCheckStatusStr(r.spiBms), qcCheckStatusStr(r.canEsc),
               qcCheckStatusStr(r.canBms), qcCheckStatusStr(r.cpu),
               qcCheckStatusStr(r.nvs), qcCheckStatusStr(r.throttle),
               qcCheckStatusStr(r.cal), qcCheckStatusStr(r.buzzer),
               qcCheckStatusStr(r.vibe), qcCheckStatusStr(r.button));
  if (n < 0 || (size_t)n >= outLen - pos) return 0;
  pos += (size_t)n;

  return pos;
}
