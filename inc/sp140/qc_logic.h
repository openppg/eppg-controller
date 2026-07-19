// Copyright 2026 <Zach Whitehead>
// OpenPPG
//
// FIRST_BOOT_QC pure logic — no Arduino/FreeRTOS/NVS dependencies so every
// decision that matters (gate, calibration gates, stability detection, the
// v8.2 mapping scaffold, record serialization) is natively unit-testable.
// See FIRST_BOOT_QC.md for the full design.

#ifndef INC_SP140_QC_LOGIC_H_
#define INC_SP140_QC_LOGIC_H_

#include <stdint.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
// Boot gate decision
// ---------------------------------------------------------------------------

// SKIP = already passed. MARK_LEGACY = fleet unit, never attempted.
// RUN = fresh / retry after fail / serial run_qc (caller clears rerun flag).
enum class QcGateAction : uint8_t {
  SKIP = 0,
  MARK_LEGACY,
  RUN,
};

// `attempted` separates fleet (user NVS, never QC'd) from a factory board that
// already got user defaults written then failed/aborted mid-QC.
QcGateAction qcGateDecision(bool passed, bool rerun, bool userSettings,
                            bool attempted);

// Pot-confirm thresholds from cal endpoints (or 0..4095 if cal unsaved).
struct QcPotConfirmLevels {
  uint16_t confirm;  // squeeze past = observed cue (~50% span)
  uint16_t release;  // must be at/below between checks (~10% span)
};

QcPotConfirmLevels qcPotConfirmLevels(uint16_t potMin, uint16_t potMax);

// ---------------------------------------------------------------------------
// Throttle calibration sanity gates
// ---------------------------------------------------------------------------

struct QcCalGates {
  uint16_t minSpan;           // reject if raw_max - raw_min below this
  uint16_t maxIdle;           // reject if raw_min above this (miswired/stuck)
  uint16_t minFull;           // reject if raw_max below this (never reaches full)
  uint16_t releaseTolerance;  // re-release must land within this of raw_min
};

enum class QcCalResult : uint8_t {
  OK = 0,
  SPAN_TOO_SMALL,
  IDLE_TOO_HIGH,
  FULL_TOO_LOW,
  RELEASE_MISMATCH,
};

QcCalResult qcValidateCalibration(uint16_t rawMin, uint16_t rawMax,
                                  uint16_t releaseRecheck,
                                  const QcCalGates& gates);

// ---------------------------------------------------------------------------
// Stability window detector (release/squeeze capture)
// ---------------------------------------------------------------------------

// Fixed-capacity ring buffer over raw ADC samples. "Stable" when the buffer is
// full and (max - min) <= epsilon across the whole window. Capture value is
// the window median (robust to single-sample glitches).
class QcStabilityWindow {
 public:
  static const size_t kMaxWindow = 64;

  QcStabilityWindow(uint16_t epsilon, size_t windowSize);

  void push(uint16_t raw);
  void reset();
  bool isFull() const { return count_ >= size_; }
  bool isStable() const;
  uint16_t median() const;  // only meaningful when isFull()

 private:
  uint16_t buf_[kMaxWindow];
  size_t size_;
  size_t count_;
  size_t head_;
  uint16_t epsilon_;
};

// ---------------------------------------------------------------------------
// v8.2 scaffold: calibrated raw->PWM mapping (pure function, wired to NOTHING
// in v8.1 — the live throttle path still uses the fixed 0..4095 mapping).
// ---------------------------------------------------------------------------

int qcPotRawToPwmCalibrated(uint16_t raw,
                            uint16_t potMin, uint16_t potMax,
                            int escMinPwm, int escMaxPwm,
                            float bottomPct, float topPct,
                            uint16_t floorDb);

// ---------------------------------------------------------------------------
// QC record + one-line JSON serialization
// ---------------------------------------------------------------------------

enum class QcCheckStatus : uint8_t {
  NOT_RUN = 0,
  PASS,
  FAIL,
  SKIP,  // operator button-confirmed deliberately-absent device (e.g. no ESC)
};

const char* qcCheckStatusStr(QcCheckStatus s);

struct QcRecord {
  char fw[8];        // "8.1"
  char build[24];    // build date string
  uint16_t potMin;
  uint16_t potMax;
  bool calSaved;
  float baroHpa;
  float cpuC;
  float packV;
  char escHwId[12];  // empty => null in JSON
  char escSn[36];    // empty => null in JSON
  char bmsId[36];    // empty => null in JSON
  QcCheckStatus display;
  QcCheckStatus i2cBaro;
  QcCheckStatus spiBms;
  QcCheckStatus canEsc;
  QcCheckStatus canBms;
  QcCheckStatus cpu;
  QcCheckStatus nvs;
  QcCheckStatus throttle;
  QcCheckStatus cal;
  QcCheckStatus buzzer;
  QcCheckStatus vibe;
  QcCheckStatus button;
};

// PASSED iff every check is PASS or SKIP (a NOT_RUN check means the flow was
// interrupted and must not count as a pass).
bool qcRecordAllPassed(const QcRecord& r);

// Serialize as a single JSON line (no trailing newline). Returns bytes
// written (excluding NUL), or 0 if the buffer was too small.
size_t qcRecordToJson(const QcRecord& r, char* out, size_t outLen);

#endif  // INC_SP140_QC_LOGIC_H_
