#include <gtest/gtest.h>
#include <cmath>

#include "../../inc/sp140/climb_efficiency.h"
#include "../../inc/sp140/thermal_headroom.h"
#include "../../src/sp140/climb_efficiency.cpp"
#include "../../src/sp140/thermal_headroom.cpp"

namespace {

// Flies a power schedule at the UI rate while a "true" first-order motor
// follows the same model the firmware assumes, so the observer can be
// checked against known parameters.
struct HotFlight {
  ThermalHeadroom heat;
  uint32_t t = 1000;
  float motor = 45.0f;       // start at the true baseline
  float battery = NAN;
  float soc = NAN;
  float motorBase = 45.0f;   // true T_base
  float motorK = 1.0f;       // true k (the firmware default)
  float motorTau = 207.0f;

  void fly(uint32_t ms, float powerKw, float socDropPerS = 0.0f) {
    const float dt = 0.033f;
    const float a = expf(-dt / motorTau);
    for (uint32_t e = 0; e < ms; e += 33) {
      t += 33;
      motor = a * motor + (1 - a) * (motorBase + motorK * powerKw * powerKw);
      if (!std::isnan(soc)) soc -= socDropPerS * dt;
      const float temps[THERMAL_PART_COUNT] = {motor, NAN, NAN, NAN, battery, NAN, NAN};
      heat.update(t, powerKw, temps, soc);
    }
  }
  const ThermalHeadroomResult& r() const { return heat.result(); }
};

// Analytic warn limit for the motor from steady state (no SOC: 300 s).
float motorWarnLimit(float base, float t0) {
  const float th = 105.0f - 3.0f;
  const float e = expf(-300.0f / 207.0f);
  const float tss = (th - t0 * e) / (1 - e);
  return sqrtf((tss - base) / 1.0f);
}

}  // namespace

TEST(ThermalHeadroom, NoSensorsMeansNoLimit) {
  ThermalHeadroom h;
  const float temps[THERMAL_PART_COUNT] = {NAN, NAN, NAN, NAN, NAN, NAN, NAN};
  for (uint32_t t = 0; t < 30000; t += 33) h.update(t, 8.0f, temps, NAN);
  EXPECT_FALSE(h.result().valid);
  EXPECT_FLOAT_EQ(h.result().warnPowerKw, h.config().maxPowerKw);
}

TEST(ThermalHeadroom, ShowsNothingUntilTheFlightHasTaughtIt) {
  HotFlight f;
  f.motor = 25.0f;
  f.fly(60 * 1000, 0.5f);   // armed on the ground
  EXPECT_FALSE(f.r().valid);
  f.fly(10 * 60 * 1000, 3.5f);  // cruise only: steepness not measurable
  EXPECT_FALSE(f.r().valid);
  f.fly(90 * 1000, 12.0f);  // a climb gives the curve a second point
  f.fly(60 * 1000, 3.5f);
  EXPECT_TRUE(f.r().valid);
  EXPECT_TRUE(f.r().learnedMask & (1u << THERMAL_MOTOR));
}

TEST(ThermalHeadroom, LearnsTheBaselineAndProjectsTheLimit) {
  // A real flight: motor starts at air temperature (25 C; its cooling
  // baseline is air + 21), takeoff climb, then cruise.
  HotFlight f;
  f.motor = 25.0f;
  f.motorBase = 46.0f;
  f.fly(90 * 1000, 12.0f);
  f.fly(20 * 60 * 1000, 3.5f);
  ASSERT_TRUE(f.r().valid);
  EXPECT_NEAR(f.r().baseC[THERMAL_MOTOR], 46.0f, 2.0f);
  EXPECT_NEAR(f.r().gainC[THERMAL_MOTOR], 1.0f, 0.15f);
  EXPECT_EQ(f.r().warnLimiter, THERMAL_MOTOR);
  EXPECT_NEAR(f.r().warnPowerKw, motorWarnLimit(46.0f, f.motor), 0.4f);
  EXPECT_GT(f.r().critPowerKw, f.r().warnPowerKw);
}

TEST(ThermalHeadroom, HotterDayLowersTheLimit) {
  HotFlight cool;
  cool.motor = 25.0f;
  cool.motorBase = 46.0f;
  cool.fly(90 * 1000, 12.0f);
  cool.fly(20 * 60 * 1000, 3.5f);
  HotFlight hot;
  hot.motor = 40.0f;  // 15 C warmer air
  hot.motorBase = 61.0f;
  hot.fly(90 * 1000, 12.0f);
  hot.fly(20 * 60 * 1000, 3.5f);
  ASSERT_TRUE(cool.r().valid);
  ASSERT_TRUE(hot.r().valid);
  EXPECT_LT(hot.r().warnPowerKw, cool.r().warnPowerKw - 0.5f);
}

TEST(ThermalHeadroom, ClimbingHotLowersTheLimitLive) {
  HotFlight f;
  f.fly(90 * 1000, 12.0f);
  f.fly(10 * 60 * 1000, 3.5f);
  ASSERT_TRUE(f.r().valid);
  const float before = f.r().warnPowerKw;
  f.fly(3 * 60 * 1000, 12.0f);  // motor heats toward its warning
  EXPECT_LT(f.r().warnPowerKw, before - 1.0f);
}

TEST(ThermalHeadroom, AlreadyOverTheWarningMeansZero) {
  HotFlight f;
  f.motorBase = 90.0f;
  f.motor = 100.0f;
  f.fly(60 * 1000, 10.0f);   // heats past the warning...
  f.fly(3 * 60 * 1000, 3.0f);  // ...and learns from it
  ASSERT_TRUE(f.r().valid);
  EXPECT_NEAR(f.r().warnPowerKw, 0.0f, 0.5f);
}

TEST(ThermalHeadroom, BatteryHeatsWithoutCooling) {
  HotFlight f;  // battery only, on a 2.6 kWh pack draining at 3 kW
  f.motor = NAN;
  f.battery = 45.0f;
  f.soc = 80.0f;
  f.fly(8 * 60 * 1000, 3.0f, 100.0f * 3.0f / 3600.0f / 2.6f);
  const ThermalHeadroom& h = f.heat;
  ASSERT_TRUE(h.result().valid);
  EXPECT_NEAR(h.result().packKwh, 2.6f, 0.2f);
  // T(H) = T + k P^2 (1 - e^-H/tau) must stay under 50 - 3, with k the
  // pack-type gain (small pack assumed until measured).
  const float gain = h.partGain(THERMAL_BATTERY) * (1.0f - expf(-300.0f / 5000.0f));
  EXPECT_EQ(h.result().warnLimiter, THERMAL_BATTERY);
  EXPECT_NEAR(h.result().warnPowerKw, sqrtf(2.0f / gain), 0.3f);
}

TEST(ThermalHeadroom, BmsMosfetNearItsWarningLimitsTheClimb) {
  ThermalHeadroom h;
  // Cells cool, BMS MOSFET 2 C under its (margin-adjusted) warning, pack
  // measured from a 2.6 kWh SOC drop at 3 kW.
  float soc = 80.0f;
  for (uint32_t t = 0; t < 8 * 60 * 1000; t += 33) {
    soc -= 100.0f * 3.0f / 3600.0f / 2.6f * 0.033f;
    const float temps[THERMAL_PART_COUNT] = {NAN, NAN, NAN, NAN, 35.0f, 45.0f, 36.0f};
    h.update(t, 3.0f, temps, soc);
  }
  ASSERT_TRUE(h.result().valid);
  EXPECT_EQ(h.result().warnLimiter, THERMAL_BMS_MOS);
  EXPECT_LT(h.result().warnPowerKw, 12.0f);
}

TEST(ThermalHeadroom, SmallPackHeatsFasterThanLarge) {
  ThermalHeadroomConfig cfg;
  cfg.packKwhDefault = 2.3f;
  ThermalHeadroom small(cfg);
  cfg.packKwhDefault = 4.6f;
  ThermalHeadroom large(cfg);
  const float ratio = small.partGain(THERMAL_BATTERY) / large.partGain(THERMAL_BATTERY);
  EXPECT_NEAR(ratio, 2.8f, 0.2f);  // fleet: ~2.9x
}

TEST(ThermalHeadroom, LearnsASteeperMotorLive) {
  HotFlight f;
  f.motorK = 1.6f;  // runs hotter per kW^2 than the fleet prior (1.0)
  for (int i = 0; i < 6; ++i) {  // cruise / climb cycles excite both params
    f.fly(4 * 60 * 1000, 3.5f);
    f.fly(60 * 1000, 10.0f);
  }
  EXPECT_NEAR(f.r().gainC[THERMAL_MOTOR], 1.6f, 0.25f);
  EXPECT_NEAR(f.r().baseC[THERMAL_MOTOR], 45.0f, 4.0f);
}

TEST(ThermalHeadroom, LowBatteryShortensTheClimbAndRaisesTheLimit) {
  HotFlight full;
  full.soc = 90.0f;
  full.fly(90 * 1000, 12.0f);
  full.fly(10 * 60 * 1000, 3.5f);
  HotFlight low;
  low.soc = 18.0f;  // ~3 % above reserve: any climb is short
  low.fly(90 * 1000, 12.0f);
  low.fly(10 * 60 * 1000, 3.5f);
  ASSERT_TRUE(full.r().valid);
  // The pack runs out long before the motor could overheat.
  EXPECT_GT(low.r().warnPowerKw, full.r().warnPowerKw + 2.0f);
}

TEST(ThermalHeadroom, LearnsPackSizeFromSocDrop) {
  HotFlight f;
  f.soc = 80.0f;
  // A 4.8 kWh pack at 3 kW loses 100 * 3 / 4800 % per second.
  f.fly(8 * 60 * 1000, 3.0f, 100.0f * 3.0f / 3600.0f / 4.8f);
  EXPECT_NEAR(f.r().packKwh, 4.8f, 0.3f);
}

TEST(ClimbBarZones, NoThermalLimitKeepsTheEfficiencyBand) {
  ClimbEfficiencyConfig cfg;
  ClimbEfficiencyResult r = {};
  r.levelValid = r.curveValid = true;
  r.levelPowerKw = 3.2f;
  const ClimbBarZones z = climbBarZones(r, NAN, NAN, cfg);
  EXPECT_TRUE(z.green);
  EXPECT_FALSE(z.yellow);
  EXPECT_FALSE(z.red);
  EXPECT_NEAR(z.greenLowKw, 10.0f, 0.6f);
  EXPECT_FLOAT_EQ(z.greenHighKw, cfg.maxPowerKw);
}

TEST(ClimbBarZones, HeatPullsTheGreenBandBack) {
  ClimbEfficiencyConfig cfg;
  ClimbEfficiencyResult r = {};
  r.levelValid = r.curveValid = true;
  r.levelPowerKw = 3.2f;
  const ClimbBarZones z = climbBarZones(r, 8.6f, 9.6f, cfg);
  ASSERT_TRUE(z.green);
  EXPECT_FLOAT_EQ(z.greenHighKw, 8.6f);
  EXPECT_LT(z.greenLowKw, 8.6f);
  EXPECT_GT(z.greenLowKw, 6.0f);  // still a band worth being in
  ASSERT_TRUE(z.yellow);
  EXPECT_FLOAT_EQ(z.yellowLowKw, 8.6f);
  EXPECT_FLOAT_EQ(z.yellowHighKw, 9.6f);
  ASSERT_TRUE(z.red);
  EXPECT_FLOAT_EQ(z.redLowKw, 9.6f);
  EXPECT_FLOAT_EQ(z.redHighKw, cfg.maxPowerKw);
}

TEST(ClimbBarZones, GreenNeverOverlapsYellow) {
  ClimbEfficiencyConfig cfg;
  ClimbEfficiencyResult r = {};
  r.levelValid = r.curveValid = true;
  r.levelPowerKw = 3.2f;  // efficiency peak ~14.5 kW
  const ClimbBarZones z = climbBarZones(r, 15.0f, 15.8f, cfg);
  ASSERT_TRUE(z.green);
  EXPECT_LE(z.greenHighKw, 15.0f);
  EXPECT_TRUE(z.yellow);
}

TEST(ClimbBarZones, ThermalZonesShowBeforeTheBandIsLearned) {
  ClimbEfficiencyConfig cfg;
  ClimbEfficiencyResult r = {};  // nothing learned yet
  const ClimbBarZones z = climbBarZones(r, 8.0f, 9.0f, cfg);
  EXPECT_FALSE(z.green);
  EXPECT_TRUE(z.yellow);
  EXPECT_TRUE(z.red);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  RUN_ALL_TESTS();
  // Always return zero-code and allow PlatformIO to parse results
  return 0;
}
