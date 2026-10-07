// Copyright 2026 <Zach Whitehead>
// OpenPPG
//
// Native tests for FIRST_BOOT_QC pure logic (gate decision, calibration
// sanity gates, stability window, v8.2 mapping scaffold, JSON record).

#include <gtest/gtest.h>
#include <string>

// Include the real implementation under test (pure — no Arduino deps).
#include "../../inc/sp140/qc_logic.h"
#include "../../src/sp140/qc_logic.cpp"

// ---------------------------------------------------------------------------
// Gate decision table — the fleet-safety contract.
// ---------------------------------------------------------------------------

TEST(QcGate, FreshFactoryUnitRunsQc) {
  EXPECT_EQ(qcGateDecision(false, false, false, false), QcGateAction::RUN);
}

TEST(QcGate, ExistingFleetUnitNeverSeesQc) {
  // User NVS from ≤v8.0, never attempted → back-fill, never QC.
  EXPECT_EQ(qcGateDecision(false, false, true, false), QcGateAction::MARK_LEGACY);
}

TEST(QcGate, FailedOrAbortedFactoryAttemptRetries) {
  // User defaults may exist, but attempted ⇒ retry, not legacy.
  EXPECT_EQ(qcGateDecision(false, false, true, true), QcGateAction::RUN);
  EXPECT_EQ(qcGateDecision(false, false, false, true), QcGateAction::RUN);
}

TEST(QcGate, PassedUnitBootsNormally) {
  EXPECT_EQ(qcGateDecision(true, false, false, false), QcGateAction::SKIP);
  EXPECT_EQ(qcGateDecision(true, false, true, true), QcGateAction::SKIP);
}

TEST(QcGate, SerialRerunOverridesEverything) {
  EXPECT_EQ(qcGateDecision(true, true, true, true), QcGateAction::RUN);
  EXPECT_EQ(qcGateDecision(false, true, true, false), QcGateAction::RUN);
  EXPECT_EQ(qcGateDecision(true, true, false, false), QcGateAction::RUN);
}

TEST(QcGate, ExhaustiveInputSpace) {
  for (int passed = 0; passed <= 1; passed++) {
    for (int rerun = 0; rerun <= 1; rerun++) {
      for (int user = 0; user <= 1; user++) {
        for (int attempted = 0; attempted <= 1; attempted++) {
          const QcGateAction a = qcGateDecision(passed, rerun, user, attempted);
          if (rerun || (!passed && (attempted || !user))) {
            EXPECT_EQ(a, QcGateAction::RUN);
          } else if (passed) {
            EXPECT_EQ(a, QcGateAction::SKIP);
          } else {
            EXPECT_EQ(a, QcGateAction::MARK_LEGACY);
          }
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Calibration sanity gates
// ---------------------------------------------------------------------------

static const QcCalGates kGates = {2000, 800, 3200, 100};  // span/idle/full/tol

TEST(QcCalGatesTest, TypicalUnitPasses) {
  // Design-doc typical unit: idle ~142, full ~3987, recheck near idle.
  EXPECT_EQ(qcValidateCalibration(142, 3987, 150, kGates), QcCalResult::OK);
}

TEST(QcCalGatesTest, SpanTooSmall) {
  // Bad pot / wiring: barely moves.
  EXPECT_EQ(qcValidateCalibration(1000, 2500, 1000, kGates),
            QcCalResult::SPAN_TOO_SMALL);
  // Inverted / equal values are also span failures, never underflow.
  EXPECT_EQ(qcValidateCalibration(3000, 3000, 3000, kGates),
            QcCalResult::SPAN_TOO_SMALL);
  EXPECT_EQ(qcValidateCalibration(3000, 2000, 3000, kGates),
            QcCalResult::SPAN_TOO_SMALL);
}

TEST(QcCalGatesTest, IdleTooHigh) {
  // Miswired / stuck-high: never returns near zero.
  EXPECT_EQ(qcValidateCalibration(900, 3987, 910, kGates),
            QcCalResult::IDLE_TOO_HIGH);
}

TEST(QcCalGatesTest, FullTooLow) {
  // Never reaches full press band.
  EXPECT_EQ(qcValidateCalibration(100, 3100, 110, kGates),
            QcCalResult::FULL_TOO_LOW);
}

TEST(QcCalGatesTest, ReleaseMismatch) {
  // Sticky lever: re-release lands far from captured idle.
  EXPECT_EQ(qcValidateCalibration(142, 3987, 400, kGates),
            QcCalResult::RELEASE_MISMATCH);
  // Tolerance is symmetric.
  EXPECT_EQ(qcValidateCalibration(400, 3987, 150, kGates),
            QcCalResult::RELEASE_MISMATCH);
}

TEST(QcCalGatesTest, BoundaryValues) {
  // Exactly at gates: span == minSpan passes, idle == maxIdle passes,
  // full == minFull passes, recheck at exact tolerance passes.
  EXPECT_EQ(qcValidateCalibration(800, 3200, 800 + kGates.releaseTolerance,
                                  QcCalGates{2400, 800, 3200, 100}),
            QcCalResult::OK);
}

// ---------------------------------------------------------------------------
// Pot-confirm thresholds (interactive checks use calibrated endpoints)
// ---------------------------------------------------------------------------

TEST(QcPotConfirmLevelsTest, UsesCalibratedSpan) {
  const QcPotConfirmLevels levels = qcPotConfirmLevels(142, 3987);
  EXPECT_EQ(levels.confirm, static_cast<uint16_t>(142 + (3987 - 142) / 2));
  EXPECT_EQ(levels.release, static_cast<uint16_t>(142 + (3987 - 142) / 10));
  EXPECT_LT(levels.release, levels.confirm);
}

TEST(QcPotConfirmLevelsTest, DegenerateFallsBackToFullAdcRange) {
  const QcPotConfirmLevels levels = qcPotConfirmLevels(3000, 1000);
  EXPECT_EQ(levels.confirm, 2047);
  EXPECT_EQ(levels.release, 409);
}

// ---------------------------------------------------------------------------
// Stability window
// ---------------------------------------------------------------------------

TEST(QcStabilityTest, NotStableUntilWindowFull) {
  QcStabilityWindow w(30, 25);
  for (int i = 0; i < 24; i++) {
    w.push(100);
    EXPECT_FALSE(w.isStable());
  }
  w.push(100);
  EXPECT_TRUE(w.isStable());
}

TEST(QcStabilityTest, NoisySignalNotStable) {
  QcStabilityWindow w(30, 25);
  for (int i = 0; i < 25; i++) {
    // Alternate +/- 50 counts around 100 — beyond epsilon 30.
    w.push(static_cast<uint16_t>((i % 2 == 0) ? 150 : 50));
  }
  EXPECT_FALSE(w.isStable());
}

TEST(QcStabilityTest, SmallJitterWithinEpsilonIsStable) {
  QcStabilityWindow w(30, 25);
  for (int i = 0; i < 25; i++) {
    w.push(static_cast<uint16_t>(100 + (i % 3)));  // 100..102 jitter
  }
  EXPECT_TRUE(w.isStable());
  EXPECT_NEAR(w.median(), 101, 1);
}

TEST(QcStabilityTest, BecomesStableAfterSettling) {
  QcStabilityWindow w(30, 10);
  // Operator moving the lever...
  for (int i = 0; i < 10; i++) {
    w.push(static_cast<uint16_t>(500 + i * 100));
  }
  EXPECT_FALSE(w.isStable());
  // ...then holds still: the moving window flushes out the ramp.
  for (int i = 0; i < 10; i++) {
    w.push(3990);
  }
  EXPECT_TRUE(w.isStable());
  EXPECT_EQ(w.median(), 3990);
}

TEST(QcStabilityTest, MedianRobustToSingleGlitch) {
  QcStabilityWindow w(4095, 25);  // wide epsilon; testing median only
  for (int i = 0; i < 24; i++) {
    w.push(140);
  }
  w.push(4000);  // one glitch sample
  EXPECT_EQ(w.median(), 140);
}

TEST(QcStabilityTest, ResetClears) {
  QcStabilityWindow w(30, 5);
  for (int i = 0; i < 5; i++) w.push(100);
  EXPECT_TRUE(w.isStable());
  w.reset();
  EXPECT_FALSE(w.isStable());
  EXPECT_FALSE(w.isFull());
}

// ---------------------------------------------------------------------------
// v8.2 mapping scaffold (pure math — NOT wired into the live throttle path)
// ---------------------------------------------------------------------------

static const int kEscMin = 1035;
static const int kEscMax = 1950;

TEST(QcCalibratedMapping, IdleAlwaysMapsToEscMin) {
  // Anything at/below the effective minimum is idle.
  EXPECT_EQ(qcPotRawToPwmCalibrated(0, 142, 3987, kEscMin, kEscMax,
                                    0.03f, 0.02f, 50),
            kEscMin);
  EXPECT_EQ(qcPotRawToPwmCalibrated(142, 142, 3987, kEscMin, kEscMax,
                                    0.03f, 0.02f, 50),
            kEscMin);
}

TEST(QcCalibratedMapping, FullPressReachesEscMax) {
  // The design-doc fix: units whose pot never reaches 4095 still get full
  // power once calibrated.
  EXPECT_EQ(qcPotRawToPwmCalibrated(3987, 142, 3987, kEscMin, kEscMax,
                                    0.03f, 0.02f, 50),
            kEscMax);
  EXPECT_EQ(qcPotRawToPwmCalibrated(4095, 142, 3987, kEscMin, kEscMax,
                                    0.03f, 0.02f, 50),
            kEscMax);
}

TEST(QcCalibratedMapping, MonotonicThroughRange) {
  int last = kEscMin;
  for (uint16_t raw = 0; raw <= 4095; raw = static_cast<uint16_t>(raw + 64)) {
    const int pwm = qcPotRawToPwmCalibrated(raw, 142, 3987, kEscMin, kEscMax,
                                            0.03f, 0.02f, 50);
    EXPECT_GE(pwm, last);
    EXPECT_GE(pwm, kEscMin);
    EXPECT_LE(pwm, kEscMax);
    last = pwm;
  }
}

TEST(QcCalibratedMapping, DegenerateCalibrationIsSafeIdle) {
  // Corrupted/backwards calibration can never produce a non-idle command.
  EXPECT_EQ(qcPotRawToPwmCalibrated(2000, 3000, 3000, kEscMin, kEscMax,
                                    0.03f, 0.02f, 50),
            kEscMin);
  EXPECT_EQ(qcPotRawToPwmCalibrated(2000, 3000, 1000, kEscMin, kEscMax,
                                    0.03f, 0.02f, 50),
            kEscMin);
}

TEST(QcCalibratedMapping, FloorDeadbandApplies) {
  // With a tiny span, the fixed floor deadband dominates the percentage.
  const int justAboveMin = qcPotRawToPwmCalibrated(
      1049, 1000, 3200, kEscMin, kEscMax, 0.0f, 0.0f, 50);
  EXPECT_EQ(justAboveMin, kEscMin);  // inside the 50-count floor deadband
}

// ---------------------------------------------------------------------------
// QC record + JSON
// ---------------------------------------------------------------------------

static QcRecord makePassingRecord() {
  QcRecord r = {};
  snprintf(r.fw, sizeof(r.fw), "8.1");
  snprintf(r.build, sizeof(r.build), "Jul  4 2026");
  r.potMin = 142;
  r.potMax = 3987;
  r.calSaved = true;
  r.baroHpa = 1002.1f;
  r.cpuC = 41.2f;
  r.packV = 98.7f;
  snprintf(r.escHwId, sizeof(r.escHwId), "0x1A2B");
  snprintf(r.escSn, sizeof(r.escSn), "A1B2C3");
  snprintf(r.bmsId, sizeof(r.bmsId), "BAT-001");
  r.display = r.i2cBaro = r.spiBms = r.canEsc = r.canBms = QcCheckStatus::PASS;
  r.cpu = r.nvs = r.throttle = r.cal = QcCheckStatus::PASS;
  r.buzzer = r.vibe = r.button = QcCheckStatus::PASS;
  return r;
}

TEST(QcRecordTest, AllPassIsPassed) {
  EXPECT_TRUE(qcRecordAllPassed(makePassingRecord()));
}

TEST(QcRecordTest, SkipDoesNotFailTheUnit) {
  QcRecord r = makePassingRecord();
  r.canEsc = QcCheckStatus::SKIP;  // bare-controller bench QC, no ESC
  r.canBms = QcCheckStatus::SKIP;  // no pack attached
  EXPECT_TRUE(qcRecordAllPassed(r));
}

TEST(QcRecordTest, AnyFailFailsTheUnit) {
  QcRecord r = makePassingRecord();
  r.i2cBaro = QcCheckStatus::FAIL;
  EXPECT_FALSE(qcRecordAllPassed(r));
}

TEST(QcRecordTest, NotRunCountsAsFailure) {
  // An interrupted flow must never report PASSED.
  QcRecord r = makePassingRecord();
  r.button = QcCheckStatus::NOT_RUN;
  EXPECT_FALSE(qcRecordAllPassed(r));
}

TEST(QcRecordTest, JsonGolden) {
  QcRecord r = makePassingRecord();
  r.canEsc = QcCheckStatus::SKIP;
  char buf[768];
  const size_t n = qcRecordToJson(r, buf, sizeof(buf));
  ASSERT_GT(n, 0u);

  const std::string json(buf);
  EXPECT_NE(json.find("\"qc\":1"), std::string::npos);
  EXPECT_NE(json.find("\"fw\":\"8.1\""), std::string::npos);
  EXPECT_NE(json.find("\"result\":\"PASSED\""), std::string::npos);
  EXPECT_NE(json.find("\"pot_min\":142"), std::string::npos);
  EXPECT_NE(json.find("\"pot_max\":3987"), std::string::npos);
  EXPECT_NE(json.find("\"span\":3845"), std::string::npos);
  EXPECT_NE(json.find("\"can_esc\":\"skip\""), std::string::npos);
  EXPECT_NE(json.find("\"buzzer\":\"pass\""), std::string::npos);
  // Must be exactly one JSON object on one line.
  EXPECT_EQ(json.front(), '{');
  EXPECT_EQ(json.back(), '}');
  EXPECT_EQ(json.find('\n'), std::string::npos);
}

TEST(QcRecordTest, JsonNullIds) {
  QcRecord r = makePassingRecord();
  r.escHwId[0] = '\0';
  r.escSn[0] = '\0';
  r.bmsId[0] = '\0';
  char buf[768];
  ASSERT_GT(qcRecordToJson(r, buf, sizeof(buf)), 0u);
  const std::string json(buf);
  EXPECT_NE(json.find("\"esc_hw_id\":null"), std::string::npos);
  EXPECT_NE(json.find("\"esc_sn\":null"), std::string::npos);
  EXPECT_NE(json.find("\"bms_id\":null"), std::string::npos);
}

TEST(QcRecordTest, JsonBufferTooSmallReturnsZero) {
  char tiny[32];
  EXPECT_EQ(qcRecordToJson(makePassingRecord(), tiny, sizeof(tiny)), 0u);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
