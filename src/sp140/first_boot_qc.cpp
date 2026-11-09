// Copyright 2026 <Zach Whitehead>
// OpenPPG
//
// FIRST_BOOT_QC flow. Design: FIRST_BOOT_QC.md. Pure decision logic lives in
// qc_logic.cpp (natively tested); this file owns hardware sequencing and runs
// single-threaded inside setup() — no app tasks exist yet, so all polling is
// direct (readESCTelemetry/updateBMSData/readThrottleRaw) and the UI is
// pumped inline.

#include "sp140/first_boot_qc.h"

#include "Arduino.h"
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "sp140/qc_logic.h"
#include "sp140/factory_settings.h"
#include "sp140/globals.h"
#include "sp140/altimeter.h"
#include "sp140/system_monitors.h"
#include "sp140/throttle.h"
#include "sp140/esc.h"
#include "sp140/bms.h"
#include "sp140/shared-config.h"
#include "sp140/buzzer.h"
#include "sp140/vibration_pwm.h"
#include "sp140/lvgl/lvgl_qc_screen.h"
#include "sp140/lvgl/lvgl_main_screen.h"
#include "../../inc/version.h"
#include "../../inc/sp140/esp32s3-config.h"

extern const char* buildDate;
extern HardwareConfig board_config;
extern SemaphoreHandle_t lvglMutex;

// ---------------------------------------------------------------------------
// Boot context (captured before refreshDeviceData() writes defaults)
// ---------------------------------------------------------------------------

static bool s_userSettingsPresentAtBoot = false;
static bool s_factoryQcPassedAtBoot = false;
static bool s_factoryQcAttemptedAtBoot = false;
static bool s_rerunRequestedAtBoot = false;
static bool s_contextCaptured = false;

void qcCaptureBootContext() {
  factorySettingsInit();

  // Probe the user namespace read-only with a LOCAL Preferences instance —
  // the global one is owned by device_settings and not yet initialized.
  Preferences userProbe;
  if (userProbe.begin("openppg", true)) {
    s_userSettingsPresentAtBoot = userProbe.isKey("ver_major");
    userProbe.end();
  } else {
    // Namespace doesn't exist yet => genuinely fresh unit.
    s_userSettingsPresentAtBoot = false;
  }

  s_factoryQcPassedAtBoot = factoryQcPassed();
  s_factoryQcAttemptedAtBoot = factoryQcAttempted();
  s_rerunRequestedAtBoot = factoryRerunRequested();
  s_contextCaptured = true;
}

bool qcShouldRun() {
  if (!s_contextCaptured) {
    return false;  // fail-safe for fleet; factory can always use run_qc
  }

  const QcGateAction action = qcGateDecision(
      s_factoryQcPassedAtBoot, s_rerunRequestedAtBoot,
      s_userSettingsPresentAtBoot, s_factoryQcAttemptedAtBoot);

  if (action == QcGateAction::MARK_LEGACY) {
    USBSerial.println(F("QC: existing unit detected - back-filling qc_passed"));
    factoryMarkLegacyUnit(factoryEncodeFw(VERSION_MAJOR, VERSION_MINOR));
    return false;
  }
  if (action != QcGateAction::RUN) {
    return false;  // SKIP
  }

  // Sticky before UI/HW work so a mid-flow power-cut retries next boot.
  factoryMarkQcAttempted();
  if (s_rerunRequestedAtBoot) {
    USBSerial.println(F("QC: rerun requested via serial command"));
    factoryClearRerunFlag();
  } else if (s_factoryQcAttemptedAtBoot) {
    USBSerial.println(F("QC: prior attempt incomplete - re-entering QC"));
  } else {
    USBSerial.println(F("QC: fresh factory unit - entering QC flow"));
  }
  return true;
}

// ---------------------------------------------------------------------------
// View seam — commit 2 is serial-only; the LVGL QC screen hooks in via these
// (lvgl_qc_screen.cpp provides rich rendering; serial output stays for the
// bench log either way).
// ---------------------------------------------------------------------------

static uint8_t s_checkRow = 0;

static void viewPump() {
  // Render + keep the system responsive / yield to IDLE so the task WDT
  // stays fed. Single-threaded: no other task contends for LVGL or SPI.
  lv_timer_handler();
  vTaskDelay(pdMS_TO_TICKS(5));
}

static void viewCheckResult(const char* name, QcCheckStatus status) {
  USBSerial.printf("QC: %-10s %s\n", name, qcCheckStatusStr(status));
  qcScreenShowChecklist();
  if (s_checkRow < QC_SCREEN_MAX_ROWS) {
    qcScreenSetCheck(s_checkRow, name, status, nullptr);
    s_checkRow++;
  }
  viewPump();
}

static void viewPrompt(const char* line1, const char* line2) {
  USBSerial.print(F("QC: "));
  USBSerial.print(line1);
  if (line2 != nullptr && line2[0] != '\0') {
    USBSerial.print(F(" - "));
    USBSerial.print(line2);
  }
  USBSerial.println();
  qcScreenPrompt(line1, line2);
  viewPump();
}

// ---------------------------------------------------------------------------
// Direct-poll helpers (no app tasks are running)
// ---------------------------------------------------------------------------

// Poll the top button (INPUT_PULLUP — LOW = pressed) with debounce. Returns
// true if a debounced press is seen within timeoutMs. Shows a live countdown
// on the prompt view.
static bool qcWaitButtonPress(uint32_t timeoutMs) {
  const uint32_t start = millis();
  uint32_t lowSince = 0;
  while (millis() - start < timeoutMs) {
    const uint32_t elapsed = millis() - start;
    char countdown[16];
    snprintf(countdown, sizeof(countdown), "%us",
             (unsigned int)((timeoutMs - elapsed) / 1000 + 1));
    qcScreenPromptValue(countdown);
    qcScreenPromptProgress(
        static_cast<uint8_t>(100 - (elapsed * 100) / timeoutMs));

    if (digitalRead(board_config.button_top) == LOW) {
      if (lowSince == 0) {
        lowSince = millis();
      } else if (millis() - lowSince >= 50) {  // 50 ms debounce, matches main
        // Wait for release so one press can't confirm two prompts.
        while (digitalRead(board_config.button_top) == LOW &&
               millis() - start < timeoutMs) {
          viewPump();
        }
        return true;
      }
    } else {
      lowSince = 0;
    }
    viewPump();
  }
  return false;
}

// Absent-device skip-confirm pattern: the operator explicitly acknowledges a
// deliberately missing device (bare-controller bench QC). Button press within
// the window => SKIP; no press => FAIL (a dead attached device must not be
// silently skippable).
static QcCheckStatus qcSkipConfirm(const char* deviceName) {
  char line[64];
  snprintf(line, sizeof(line), "%s NOT DETECTED", deviceName);
  viewPrompt(line, "press button to confirm testing without it");
  return qcWaitButtonPress(QC_SKIP_CONFIRM_TIMEOUT_MS) ? QcCheckStatus::SKIP
                                                       : QcCheckStatus::FAIL;
}

// Show a brief step result on the prompt view (cal + interactive checks live
// outside the 8-row POST checklist).
static void viewStepResult(const char* name, QcCheckStatus status) {
  USBSerial.printf("QC: %-10s %s\n", name, qcCheckStatusStr(status));
  char line[48];
  snprintf(line, sizeof(line), "%s: %s", name, qcCheckStatusStr(status));
  qcScreenPrompt(line, "");
  const uint32_t start = millis();
  while (millis() - start < 900) {
    viewPump();
  }
}

// ---------------------------------------------------------------------------
// Guided throttle calibration (CAPTURE ONLY — the live mapping stays on the
// fixed 0..4095 curve in v8.1; the switch to calibrated endpoints is v8.2,
// gated on the data these captures produce.)
// ---------------------------------------------------------------------------

// Screen-guided, auto-advancing capture: sample the pot at ~50 Hz into a
// stability window; capture the median once the full window sits within
// epsilon. No button involved. Returns false on step timeout.
static bool qcCaptureStableRaw(const char* instruction, const char* subtext,
                               uint16_t* outValue) {
  viewPrompt(instruction, subtext);
  QcStabilityWindow window(QC_STABLE_EPSILON, QC_STABLE_WINDOW_SAMPLES);
  const uint32_t start = millis();
  uint32_t lastSample = 0;
  uint32_t sampleCount = 0;

  while (millis() - start < QC_CAL_STEP_TIMEOUT_MS) {
    const uint32_t now = millis();
    if (now - lastSample >= 20) {  // ~50 Hz sampling
      lastSample = now;
      const uint16_t raw = readThrottleRaw();
      window.push(raw);
      sampleCount++;

      char valText[16];
      snprintf(valText, sizeof(valText), "%u", raw);
      qcScreenPromptValue(valText);
      // Progress = window fill; holds at 100 while waiting for stability.
      const uint8_t pct = window.isFull()
          ? 100
          : static_cast<uint8_t>(
                (sampleCount * 100) / QC_STABLE_WINDOW_SAMPLES);
      qcScreenPromptProgress(pct);

      if (window.isFull() && window.isStable()) {
        *outValue = window.median();
        return true;
      }
    }
    viewPump();
  }
  return false;
}

// Full calibration sequence: release -> squeeze -> release recheck, then the
// sanity gates. Saves to the factory namespace only when everything passes.
// On fail/timeout, leave rec->potMin/Max at the safe 0..4095 defaults so
// later pot-confirm checks do not use a rejected capture for thresholds.
static QcCheckStatus qcRunThrottleCalibration(QcRecord* rec) {
  uint16_t rawMin = 0;
  uint16_t rawMax = 0;
  uint16_t rawRecheck = 0;

  if (!qcCaptureStableRaw("RELEASE THROTTLE", "let go fully and hold still",
                          &rawMin) ||
      !qcCaptureStableRaw("SQUEEZE FULL", "hold full throttle steady",
                          &rawMax) ||
      !qcCaptureStableRaw("RELEASE AGAIN", "let go fully and hold still",
                          &rawRecheck)) {
    USBSerial.println(F("QC: calibration step timed out"));
    return QcCheckStatus::FAIL;
  }

  const QcCalGates gates = {QC_MIN_SPAN, QC_MAX_IDLE, QC_MIN_FULL,
                            QC_RELEASE_TOLERANCE};
  const QcCalResult result =
      qcValidateCalibration(rawMin, rawMax, rawRecheck, gates);

  USBSerial.printf("QC: cal raw_min=%u raw_max=%u recheck=%u result=%d\n",
                   rawMin, rawMax, rawRecheck, static_cast<int>(result));

  if (result != QcCalResult::OK) {
    // Do NOT save — keep safe 0..4095 defaults on the record for pot-confirm.
    return QcCheckStatus::FAIL;
  }

  rec->potMin = rawMin;
  rec->potMax = rawMax;
  factoryWriteCal(rawMin, rawMax);
  rec->calSaved = true;
  return QcCheckStatus::PASS;
}

// ---------------------------------------------------------------------------
// POST checks (bus-communication focus: I2C / SPI / CAN)
// ---------------------------------------------------------------------------

static QcCheckStatus postCheckI2cBaro(QcRecord* rec) {
  if (!bmpPresent) {
    return QcCheckStatus::FAIL;
  }
  const float hpa = getBaroPressure();
  rec->baroHpa = hpa;
  return (hpa >= 800.0f && hpa <= 1100.0f) ? QcCheckStatus::PASS
                                           : QcCheckStatus::FAIL;
}

static QcCheckStatus postCheckSpiBms() {
  // MCP2515 responding over SPI at init proves the SPI leg; the CAN traffic
  // itself is judged separately in the BMS CAN check.
  return bmsCanInitialized ? QcCheckStatus::PASS : QcCheckStatus::FAIL;
}

// Active DroneCAN request/response — this is the end-to-end CAN TX/RX proof.
// Never a throttle/setpoint command.
static QcCheckStatus postCheckCanEsc(QcRecord* rec) {
  if (!escTwaiInitialized) {
    return qcSkipConfirm("ESC");
  }
  requestEscHardwareInfo();
  for (int i = 0; i < QC_CAN_POLL_ITERATIONS; i++) {
    readESCTelemetry();
    if (escTelemetryData.escState == TelemetryState::CONNECTED) {
      // Capture identifiers for the QC record (may need a few more polls for
      // the hardware-info response; non-fatal if absent).
      for (int j = 0; j < QC_CAN_POLL_ITERATIONS &&
                      escTelemetryData.hardware_id == 0; j++) {
        readESCTelemetry();
        vTaskDelay(pdMS_TO_TICKS(QC_CAN_POLL_INTERVAL_MS));
      }
      if (escTelemetryData.hardware_id != 0) {
        snprintf(rec->escHwId, sizeof(rec->escHwId), "0x%04X",
                 escTelemetryData.hardware_id);
      }
      bool snNonZero = false;
      for (size_t b = 0; b < sizeof(escTelemetryData.sn_code); b++) {
        if (escTelemetryData.sn_code[b] != 0) {
          snNonZero = true;
          break;
        }
      }
      if (snNonZero) {
        size_t pos = 0;
        for (size_t b = 0; b < sizeof(escTelemetryData.sn_code) &&
                        pos + 2 < sizeof(rec->escSn); b++) {
          pos += snprintf(rec->escSn + pos, sizeof(rec->escSn) - pos, "%02X",
                          escTelemetryData.sn_code[b]);
        }
      }
      return QcCheckStatus::PASS;
    }
    vTaskDelay(pdMS_TO_TICKS(QC_CAN_POLL_INTERVAL_MS));
    viewPump();
  }
  return qcSkipConfirm("ESC");
}

// Passive listen for BMS broadcast telemetry.
static QcCheckStatus postCheckCanBms(QcRecord* rec) {
  if (!bmsCanInitialized) {
    return qcSkipConfirm("BMS");
  }
  for (int i = 0; i < QC_CAN_POLL_ITERATIONS; i++) {
    updateBMSData();
    if (bmsTelemetryData.bmsState == TelemetryState::CONNECTED) {
      const float packV = bmsTelemetryData.battery_voltage;
      rec->packV = packV;
      snprintf(rec->bmsId, sizeof(rec->bmsId), "%s",
               bmsTelemetryData.battery_id);
      return (packV >= 20.0f && packV <= 102.0f) ? QcCheckStatus::PASS
                                                 : QcCheckStatus::FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(QC_CAN_POLL_INTERVAL_MS));
    viewPump();
  }
  return qcSkipConfirm("BMS");
}

static QcCheckStatus postCheckCpu(QcRecord* rec) {
  const float cpuC = getCachedCpuTemperature();
  rec->cpuC = cpuC;
  return (cpuC >= -20.0f && cpuC <= 90.0f) ? QcCheckStatus::PASS
                                           : QcCheckStatus::FAIL;
}

static QcCheckStatus postCheckThrottleAdc() {
  const uint16_t raw = readThrottleRaw();
  return (raw <= QC_MAX_IDLE) ? QcCheckStatus::PASS : QcCheckStatus::FAIL;
}

// Interactive pot-confirm: squeeze ~50% span = observed cue; release gated.
typedef void (*QcCueFn)();
static void qcCueBuzzerOn() { startTone(2093); }
static void qcCueBuzzerOff() { stopTone(); }
static void qcCueVibeOn() { vibeDirectSet(220); }
static void qcCueVibeOff() { vibeDirectSet(0); }

static void showPotProgress(uint16_t raw, uint32_t elapsed, uint32_t timeoutMs) {
  char valText[16];
  snprintf(valText, sizeof(valText), "%u", raw);
  qcScreenPromptValue(valText);
  qcScreenPromptProgress(
      static_cast<uint8_t>(100 - (elapsed * 100) / timeoutMs));
}

// Wait until pot <= releaseLevel. False on timeout.
static bool qcWaitPotRelease(uint16_t releaseLevel) {
  viewPrompt("RELEASE THROTTLE", "let go to continue");
  const uint32_t start = millis();
  while (millis() - start < QC_CONFIRM_TIMEOUT_MS) {
    const uint16_t raw = readThrottleRaw();
    showPotProgress(raw, millis() - start, QC_CONFIRM_TIMEOUT_MS);
    if (raw <= releaseLevel) return true;
    viewPump();
  }
  return false;
}

static QcCheckStatus qcPotConfirm(const char* instruction, QcCueFn cueOn,
                                  QcCueFn cueOff, uint16_t potMin,
                                  uint16_t potMax) {
  const QcPotConfirmLevels lvl = qcPotConfirmLevels(potMin, potMax);

  // Release before + after: one held squeeze cannot blanket-pass checks.
  if (!qcWaitPotRelease(lvl.release)) return QcCheckStatus::FAIL;

  viewPrompt(instruction, "squeeze throttle to confirm");
  const uint32_t start = millis();
  bool cueOnState = false;
  uint32_t lastToggle = start;  // 700 ms quiet, then 300 ms on, repeat
  bool squeezed = false;

  while (millis() - start < QC_CONFIRM_TIMEOUT_MS) {
    const uint32_t now = millis();
    if (now - lastToggle >= (cueOnState ? 300u : 700u)) {
      cueOnState = !cueOnState;
      lastToggle = now;
      if (cueOnState) cueOn();
      else cueOff();
    }

    const uint16_t raw = readThrottleRaw();
    showPotProgress(raw, now - start, QC_CONFIRM_TIMEOUT_MS);
    if (raw >= lvl.confirm) {
      squeezed = true;
      break;
    }
    viewPump();
  }
  cueOff();

  if (!squeezed) return QcCheckStatus::FAIL;
  return qcWaitPotRelease(lvl.release) ? QcCheckStatus::PASS
                                       : QcCheckStatus::FAIL;
}

// ---------------------------------------------------------------------------
// The flow
// ---------------------------------------------------------------------------

// Append a failed/skipped check name to the banner detail line.
static void appendCheckNote(char* buf, size_t bufLen, const char* name,
                            QcCheckStatus status) {
  if (status != QcCheckStatus::FAIL && status != QcCheckStatus::SKIP) {
    return;
  }
  const size_t used = strlen(buf);
  snprintf(buf + used, bufLen - used, "%s%s%s",
           used > 0 ? " " : "", name,
           status == QcCheckStatus::SKIP ? "(skip)" : "");
}

void runFirstBootQc() {
  USBSerial.println(F("QC: ===== FACTORY QC START ====="));

  // Single-threaded here, but take the LVGL mutex to keep the same invariant
  // the splash/main-screen setup path uses.
  if (lvglMutex != NULL) {
    xSemaphoreTake(lvglMutex, portMAX_DELAY);
  }
  s_checkRow = 0;
  setupQcScreen(deviceData.theme == 1);
  viewPump();

  QcRecord rec = {};
  snprintf(rec.fw, sizeof(rec.fw), "%d.%d", VERSION_MAJOR, VERSION_MINOR);
  snprintf(rec.build, sizeof(rec.build), "%s", buildDate);
  rec.potMin = 0;
  rec.potMax = 4095;

  // --- Automatic POST (bus-communication focus) ---
  rec.display = QcCheckStatus::PASS;  // implicit: the UI is rendering
  viewCheckResult("display", rec.display);

  rec.i2cBaro = postCheckI2cBaro(&rec);
  viewCheckResult("i2c_baro", rec.i2cBaro);

  rec.spiBms = postCheckSpiBms();
  viewCheckResult("spi_bms", rec.spiBms);

  rec.canEsc = postCheckCanEsc(&rec);
  viewCheckResult("can_esc", rec.canEsc);

  rec.canBms = postCheckCanBms(&rec);
  viewCheckResult("can_bms", rec.canBms);

  rec.cpu = postCheckCpu(&rec);
  viewCheckResult("cpu", rec.cpu);

  rec.nvs = factoryNvsRoundTrip() ? QcCheckStatus::PASS : QcCheckStatus::FAIL;
  viewCheckResult("nvs", rec.nvs);

  rec.throttle = postCheckThrottleAdc();
  viewCheckResult("throttle", rec.throttle);

  // --- Guided throttle calibration (capture only; mapping unchanged) ---
  rec.cal = qcRunThrottleCalibration(&rec);
  viewStepResult("cal", rec.cal);

  // --- Interactive checks (pot-confirm; button used only for its own test) ---
  rec.buzzer = qcPotConfirm("TONE PLAYING - hear it?", qcCueBuzzerOn,
                            qcCueBuzzerOff, rec.potMin, rec.potMax);
  viewStepResult("buzzer", rec.buzzer);

  rec.vibe = qcPotConfirm("VIBRATING - feel it?", qcCueVibeOn, qcCueVibeOff,
                          rec.potMin, rec.potMax);
  viewStepResult("vibe", rec.vibe);

  viewPrompt("PRESS BUTTON", "press the top button");
  rec.button = qcWaitButtonPress(QC_CONFIRM_TIMEOUT_MS) ? QcCheckStatus::PASS
                                                        : QcCheckStatus::FAIL;
  viewStepResult("button", rec.button);

  // --- Persist + report ---
  // Always write the result: pass stamps qc_passed=1; fail stamps 0 so the
  // next boot retries (qc_attempted stays set) instead of legacy-backfilling.
  const bool passed = qcRecordAllPassed(rec);
  factoryWriteQcResult(passed, factoryEncodeFw(VERSION_MAJOR, VERSION_MINOR));

  char json[QC_RECORD_JSON_MAX];
  if (qcRecordToJson(rec, json, sizeof(json)) > 0) {
    factoryWriteQcRecordBlob(json, strlen(json) + 1);  // include NUL
    USBSerial.println(json);
  }

  // --- Final banner (hold ~5 s), then hand the display back ---
  char detail[128] = "";
  appendCheckNote(detail, sizeof(detail), "baro", rec.i2cBaro);
  appendCheckNote(detail, sizeof(detail), "spi", rec.spiBms);
  appendCheckNote(detail, sizeof(detail), "esc", rec.canEsc);
  appendCheckNote(detail, sizeof(detail), "bms", rec.canBms);
  appendCheckNote(detail, sizeof(detail), "cpu", rec.cpu);
  appendCheckNote(detail, sizeof(detail), "nvs", rec.nvs);
  appendCheckNote(detail, sizeof(detail), "throttle", rec.throttle);
  appendCheckNote(detail, sizeof(detail), "cal", rec.cal);
  appendCheckNote(detail, sizeof(detail), "buzzer", rec.buzzer);
  appendCheckNote(detail, sizeof(detail), "vibe", rec.vibe);
  appendCheckNote(detail, sizeof(detail), "button", rec.button);
  qcScreenBanner(passed, detail);
  const uint32_t bannerStart = millis();
  while (millis() - bannerStart < 5000) {
    viewPump();
  }
  teardownQcScreen(main_screen);
  viewPump();
  if (lvglMutex != NULL &&
      xSemaphoreGetMutexHolder(lvglMutex) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(lvglMutex);
  }

  USBSerial.printf("QC: ===== FACTORY QC %s =====\n",
                   passed ? "PASSED" : "FAILED");
}
