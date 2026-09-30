#include <gtest/gtest.h>
#include <cmath>

#include "../../inc/sp140/climb_efficiency.h"
#include "../../src/sp140/climb_efficiency.cpp"

namespace {

// Feeds the estimator the way the UI task does (~30 Hz) while flying at a
// constant power and climb rate. `wobble` adds a deterministic up/down
// altitude disturbance (hand movement / small gusts) with a 6 s period.
struct Flight {
  ClimbEfficiencyEstimator est;
  uint32_t t = 1000;
  float alt = 100.0f;
  bool armed = true;

  explicit Flight(const ClimbEfficiencyConfig& cfg = ClimbEfficiencyConfig())
      : est(cfg) {}

  void fly(uint32_t durationMs, float powerKw, float climb, float wobble = 0.0f) {
    for (uint32_t elapsed = 0; elapsed < durationMs; elapsed += 33) {
      t += 33;
      alt += climb * 0.033f;
      const float shown = alt + wobble * sinf(t * 2.0f * 3.14159f / 6000.0f);
      est.update(t, shown, powerKw, armed);
    }
  }

  const ClimbEfficiencyResult& r() const { return est.result(); }
};

// A typical SP140 in the fleet data: holds level at 3.2 kW, gains 0.37 m/s
// per extra kW near level, curvature q = 0.016 /kW (the firmware default).
constexpr float kLevel = 3.2f;
constexpr float kSlope = 0.37f;
constexpr float kQ = 0.016f;
float modelClimb(float p) {
  const float x = p - kLevel;
  return kSlope * x * (1.0f - kQ * x);
}
const float kBest = sqrtf(kLevel * kLevel + kLevel / kQ);  // 14.5 kW

// Level flight long enough for the pink tick, then a takeoff-style climb.
void cruiseThenClimb(Flight* f) {
  f->fly(60000, kLevel, 0.0f, 0.3f);
  f->fly(40000, 8.0f, modelClimb(8.0f), 0.3f);
}

}  // namespace

TEST(ClimbEfficiency, SteadyClimbBecomesValidAfterWindowAndSettle) {
  Flight f;
  f.fly(12000, 10.0f, 2.0f);
  EXPECT_EQ(f.r().phase, ClimbEffPhase::SETTLING);  // < 10 s window + 3 s settle

  f.fly(2000, 10.0f, 2.0f);
  ASSERT_EQ(f.r().phase, ClimbEffPhase::VALID);
  EXPECT_NEAR(f.r().climbRate, 2.0f, 0.02f);
  EXPECT_NEAR(f.r().powerKw, 10.0f, 0.01f);
  EXPECT_NEAR(f.r().yield, 0.2f, 0.003f);
  EXPECT_NEAR(climbYieldToMetersPerWh(f.r().yield), 0.72f, 0.01f);
}

TEST(ClimbEfficiency, WobbleAveragesOutOverTheWindow) {
  Flight f;
  f.fly(20000, 10.0f, 2.0f, 0.5f);  // +/-0.5 m arm movement
  ASSERT_EQ(f.r().phase, ClimbEffPhase::VALID);
  EXPECT_NEAR(f.r().climbRate, 2.0f, 0.15f);
}

TEST(ClimbEfficiency, PowerChangeIsUnsteadyUntilItSettles) {
  Flight f;
  f.fly(15000, 8.0f, 1.5f);
  ASSERT_EQ(f.r().phase, ClimbEffPhase::VALID);

  f.fly(2000, 14.0f, 3.0f);
  EXPECT_EQ(f.r().phase, ClimbEffPhase::UNSTEADY);
  EXPECT_TRUE(std::isnan(f.r().yield));

  f.fly(14000, 14.0f, 3.0f);
  ASSERT_EQ(f.r().phase, ClimbEffPhase::VALID);
  EXPECT_NEAR(f.r().climbRate, 3.0f, 0.02f);
}

TEST(ClimbEfficiency, GlideAndCruiseAreNotClimbs) {
  Flight f;
  f.fly(15000, 0.2f, -1.3f);
  EXPECT_EQ(f.r().phase, ClimbEffPhase::LOW_POWER);

  f.fly(15000, 5.0f, 0.0f);  // level cruise
  EXPECT_EQ(f.r().phase, ClimbEffPhase::NOT_CLIMBING);
  EXPECT_TRUE(std::isnan(f.r().yield));
}

TEST(ClimbEfficiency, IgnoresTakeoffBelowMinAltitude) {
  Flight f;
  f.alt = 0.0f;
  f.fly(15000, 12.0f, 0.5f);  // ends at ~7.5 m
  EXPECT_EQ(f.r().phase, ClimbEffPhase::NO_DATA);
}

TEST(ClimbEfficiency, ReadingGapRestartsWindow) {
  Flight f;
  f.fly(15000, 10.0f, 2.0f);
  ASSERT_EQ(f.r().phase, ClimbEffPhase::VALID);
  f.t += 2000;  // 2 s without readings
  f.fly(1000, 10.0f, 2.0f);
  EXPECT_NE(f.r().phase, ClimbEffPhase::VALID);
}

TEST(ClimbEfficiency, LearnsLevelFlightPowerFromCruise) {
  Flight f;
  f.fly(20000, 3.4f, 0.0f, 0.3f);
  EXPECT_FALSE(f.r().levelValid);  // not enough level flight yet
  f.fly(40000, 3.4f, 0.0f, 0.3f);
  ASSERT_TRUE(f.r().levelValid);
  EXPECT_NEAR(f.r().levelPowerKw, 3.4f, 0.1f);
}

TEST(ClimbEfficiency, LevelPowerCorrectsASlightClimb) {
  ClimbEfficiencyConfig cfg;
  cfg.priorSlope = 0.3f;
  Flight f(cfg);
  // Holding 3.4 kW while drifting up 0.3 m/s means level needs ~1 kW less.
  f.fly(60000, 3.4f, 0.3f);
  ASSERT_TRUE(f.r().levelValid);
  EXPECT_NEAR(f.r().levelPowerKw, 2.4f, 0.1f);
}

TEST(ClimbEfficiency, LevelPowerFollowsAChange) {
  Flight f;
  f.fly(180000, 3.2f, 0.0f);
  ASSERT_NEAR(f.r().levelPowerKw, 3.2f, 0.05f);
  // e.g. trimmers let out: level now takes 4 kW. With the ~5 min half-life
  // the tick has mostly moved over after 10 min.
  f.fly(600000, 4.0f, 0.0f);
  EXPECT_NEAR(f.r().levelPowerKw, 4.0f, 0.15f);
}

TEST(ClimbEfficiency, ClimbsDoNotCountAsLevelFlight) {
  Flight f;
  f.fly(60000, 10.0f, 2.0f);
  EXPECT_FALSE(f.r().levelValid);
  EXPECT_TRUE(std::isnan(f.r().levelPowerKw));
}

TEST(ClimbEfficiency, BandWaitsForLevelFlightAndAClimb) {
  Flight f;
  f.fly(60000, kLevel, 0.0f);
  ASSERT_TRUE(f.r().levelValid);
  EXPECT_FALSE(f.r().curveValid);  // has not climbed yet

  f.fly(40000, 8.0f, modelClimb(8.0f));
  EXPECT_TRUE(f.r().curveValid);
}

TEST(ClimbEfficiency, BestClimbPowerFollowsTheModel) {
  Flight f;
  cruiseThenClimb(&f);
  const ClimbEfficiencyResult& r = f.r();
  ASSERT_TRUE(r.curveValid);
  EXPECT_NEAR(r.levelPowerKw, kLevel, 0.15f);
  EXPECT_NEAR(r.bestPowerKw, kBest, 0.4f);
  EXPECT_LT(r.bandLowKw, r.bestPowerKw);
  EXPECT_GT(r.bandHighKw, r.bestPowerKw);
  // The 95 % band of this shape is broad: ~10 kW up to full power.
  EXPECT_NEAR(r.bandLowKw, 10.0f, 0.6f);
  EXPECT_FLOAT_EQ(r.bandHighKw, f.est.config().maxPowerKw);
  // The climb measured the slope.
  EXPECT_NEAR(r.slope, kSlope, 0.05f);
  EXPECT_NEAR(f.est.predictedClimb(8.0f), modelClimb(8.0f), 0.1f);
}

TEST(ClimbEfficiency, RelativeYieldAtAndAwayFromBest) {
  Flight f;
  cruiseThenClimb(&f);
  f.fly(20000, 11.0f, modelClimb(11.0f));
  ASSERT_EQ(f.r().phase, ClimbEffPhase::VALID);
  EXPECT_NEAR(f.r().relativeYield, 0.97f, 0.05f);

  f.fly(20000, 5.0f, modelClimb(5.0f));
  ASSERT_EQ(f.r().phase, ClimbEffPhase::VALID);
  EXPECT_LT(f.r().relativeYield, 0.75f);  // a gentle climb wastes energy
}

TEST(ClimbEfficiency, HeavierSetupMovesTheBandUp) {
  Flight light;
  cruiseThenClimb(&light);
  Flight heavy;
  heavy.fly(60000, 5.5f, 0.0f);
  heavy.fly(40000, 10.0f, 1.2f);
  ASSERT_TRUE(heavy.r().curveValid);
  EXPECT_GT(heavy.r().bandLowKw, light.r().bandLowKw + 2.0f);
  EXPECT_LE(heavy.r().bestPowerKw, heavy.est.config().maxPowerKw);
}

TEST(ClimbEfficiency, DisarmClearsWindowButKeepsWhatWasLearned) {
  Flight f;
  cruiseThenClimb(&f);
  ASSERT_TRUE(f.r().curveValid);
  const float best = f.r().bestPowerKw;

  f.armed = false;
  f.fly(1000, 0.0f, 0.0f);
  EXPECT_EQ(f.r().phase, ClimbEffPhase::NO_DATA);
  EXPECT_TRUE(f.r().levelValid);
  EXPECT_TRUE(f.r().curveValid);
  EXPECT_FLOAT_EQ(f.r().bestPowerKw, best);
}

TEST(ClimbEfficiency, LongCruiseKeepsTheBand) {
  Flight f;
  cruiseThenClimb(&f);
  ASSERT_TRUE(f.r().curveValid);
  const float best = f.r().bestPowerKw;
  f.fly(3600000, kLevel, 0.0f, 0.3f);  // an hour of cruise
  ASSERT_TRUE(f.r().curveValid);
  EXPECT_NEAR(f.r().bestPowerKw, best, 0.5f);
}

TEST(ClimbEfficiency, LongWindowCoarsensSlotsInsteadOfOverflowing) {
  ClimbEfficiencyConfig cfg;
  cfg.windowMs = 30000;
  cfg.settleMs = 5000;
  Flight f(cfg);
  EXPECT_GT(f.est.config().sampleMs, 100u);
  f.fly(40000, 10.0f, 2.0f);
  ASSERT_EQ(f.r().phase, ClimbEffPhase::VALID);
  EXPECT_NEAR(f.r().climbRate, 2.0f, 0.02f);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  RUN_ALL_TESTS();
  // Always return zero-code and allow PlatformIO to parse results
  return 0;
}
