#include <gtest/gtest.h>
#include "sp140/simple_monitor.h"
#include "sp140/monitor_config.h"

// Fake logger to capture log calls
class FakeLogger : public ILogger {
 public:
  struct Entry {
    SensorID id;
    AlertLevel lvl;
    float fval;
    bool bval;
    bool isBool;
  };
  std::vector<Entry> entries;

  void log(SensorID id, AlertLevel lvl, float v) override {
    entries.push_back({id, lvl, v, false, false});
  }
  void log(SensorID id, AlertLevel lvl, bool v) override {
    entries.push_back({id, lvl, 0.0f, v, true});
  }
};

TEST(HysteresisMonitor, PreventsBouncingAroundWarningHigh) {
  FakeLogger logger;

  // Create monitor that warns >50, crit > 80, with 2.0 hysteresis
  Thresholds thr{.warnLow = -100.0f, .warnHigh = 50.0f, .critLow = -200.0f, .critHigh = 80.0f, .hysteresis = 2.0f};

  float sensorVal = 0.0f;
  HysteresisSensorMonitor mon(SensorID::CPU_Temp, SensorCategory::INTERNAL, thr, [&]() { return sensorVal; }, &logger);

  // Start in OK
  mon.check();
  EXPECT_TRUE(logger.entries.empty());

  // Oscillate around warning threshold - should NOT bounce
  sensorVal = 49.5f;  // Still OK
  mon.check();
  EXPECT_TRUE(logger.entries.empty());

  sensorVal = 50.2f;  // Crosses into warning
  mon.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

  // Oscillation around threshold - should NOT bounce back to OK
  sensorVal = 49.8f;  // Within hysteresis deadband
  mon.check();
  EXPECT_EQ(logger.entries.size(), 1u);  // No new entry

  sensorVal = 50.1f;  // Still within hysteresis
  mon.check();
  EXPECT_EQ(logger.entries.size(), 1u);  // No new entry

  sensorVal = 49.9f;  // Still within hysteresis
  mon.check();
  EXPECT_EQ(logger.entries.size(), 1u);  // No new entry

  // Only when we go significantly below threshold should it clear
  sensorVal = 47.9f;  // Below warnHigh - hysteresis (50 - 2 = 48)
  mon.check();
  ASSERT_EQ(logger.entries.size(), 2u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::OK);
}

TEST(HysteresisMonitor, PreventsBouncingAroundWarningLow) {
  FakeLogger logger;

  // Create monitor that warns <10, crit < 0, with 2.0 hysteresis
  Thresholds thr{.warnLow = 10.0f, .warnHigh = 200.0f, .critLow = 0.0f, .critHigh = 300.0f, .hysteresis = 2.0f};

  float sensorVal = 50.0f;
  HysteresisSensorMonitor mon(SensorID::CPU_Temp, SensorCategory::INTERNAL, thr, [&]() { return sensorVal; }, &logger);

  // Start in OK
  mon.check();
  EXPECT_TRUE(logger.entries.empty());

  // Drop to warning
  sensorVal = 8.0f;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_LOW);

  // Oscillation around threshold - should NOT bounce
  sensorVal = 10.5f;  // Within hysteresis deadband
  mon.check();
  EXPECT_EQ(logger.entries.size(), 1u);  // No new entry

  sensorVal = 9.8f;   // Still within hysteresis
  mon.check();
  EXPECT_EQ(logger.entries.size(), 1u);  // No new entry

  // Only when we go significantly above threshold should it clear
  sensorVal = 12.1f;  // Above warnLow + hysteresis (10 + 2 = 12)
  mon.check();
  ASSERT_EQ(logger.entries.size(), 2u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::OK);
}

TEST(HysteresisMonitor, EscalatesToCriticalWhenAppropriate) {
  FakeLogger logger;

  // Create monitor that warns >50, crit > 80, with 2.0 hysteresis
  Thresholds thr{.warnLow = -100.0f, .warnHigh = 50.0f, .critLow = -200.0f, .critHigh = 80.0f, .hysteresis = 2.0f};

  float sensorVal = 0.0f;
  HysteresisSensorMonitor mon(SensorID::CPU_Temp, SensorCategory::INTERNAL, thr, [&]() { return sensorVal; }, &logger);

  // Start in OK
  mon.check();
  EXPECT_TRUE(logger.entries.empty());

  // Go to warning
  sensorVal = 55.0f;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

  // Escalate to critical
  sensorVal = 85.0f;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 2u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_HIGH);

  // Try to de-escalate but stay within hysteresis
  sensorVal = 79.5f;  // Within hysteresis (80 - 2 = 78)
  mon.check();
  EXPECT_EQ(logger.entries.size(), 2u);  // Should NOT de-escalate

  // Only de-escalate when significantly below critical threshold
  sensorVal = 77.9f;  // Below critHigh - hysteresis (80 - 2 = 78)
  mon.check();
  ASSERT_EQ(logger.entries.size(), 3u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);
}

TEST(HysteresisMonitor, ZeroHysteresisBehavesLikeRegularMonitor) {
  FakeLogger logger;

  // Create monitor with zero hysteresis - should behave like SensorMonitor
  Thresholds thr{.warnLow = -100.0f, .warnHigh = 50.0f, .critLow = -200.0f, .critHigh = 80.0f, .hysteresis = 0.0f};

  float sensorVal = 0.0f;
  HysteresisSensorMonitor mon(SensorID::CPU_Temp, SensorCategory::INTERNAL, thr, [&]() { return sensorVal; }, &logger);

  // Start in OK
  mon.check();
  EXPECT_TRUE(logger.entries.empty());

  // Immediately go to warning
  sensorVal = 55.0f;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

  // Immediately go back to OK
  sensorVal = 45.0f;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 2u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::OK);
}

TEST(HysteresisMonitor, HandlesAllThresholdTransitions) {
  FakeLogger logger;

  // Symmetric thresholds with hysteresis
  Thresholds thr{.warnLow = 20.0f, .warnHigh = 80.0f, .critLow = 10.0f, .critHigh = 90.0f, .hysteresis = 2.0f};

  float sensorVal = 50.0f;  // Start in middle
  HysteresisSensorMonitor mon(SensorID::CPU_Temp, SensorCategory::INTERNAL, thr, [&]() { return sensorVal; }, &logger);

  // Start in OK
  mon.check();
  EXPECT_TRUE(logger.entries.empty());

  // Test high side warning
  sensorVal = 85.0f;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

  // Test high side critical
  sensorVal = 95.0f;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 2u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_HIGH);

  // Test de-escalation from critical to warning
  sensorVal = 87.9f;  // Below critHigh - hysteresis (90 - 2 = 88)
  mon.check();
  ASSERT_EQ(logger.entries.size(), 3u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

  // Test clearing warning high
  sensorVal = 77.9f;  // Below warnHigh - hysteresis (80 - 2 = 78)
  mon.check();
  ASSERT_EQ(logger.entries.size(), 4u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::OK);

  // Test low side warning
  sensorVal = 15.0f;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 5u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_LOW);

  // Test low side critical
  sensorVal = 5.0f;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 6u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_LOW);

  // Test de-escalation from critical to warning low
  sensorVal = 12.1f;  // Above critLow + hysteresis (10 + 2 = 12)
  mon.check();
  ASSERT_EQ(logger.entries.size(), 7u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_LOW);

  // Test clearing warning low
  sensorVal = 22.1f;  // Above warnLow + hysteresis (20 + 2 = 22)
  mon.check();
  ASSERT_EQ(logger.entries.size(), 8u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::OK);
}

TEST(HysteresisMonitor, RespectsSensorIDAndCategory) {
  FakeLogger logger;

  Thresholds thr{.warnLow = -100.0f, .warnHigh = 50.0f, .critLow = -200.0f, .critHigh = 80.0f, .hysteresis = 2.0f};

  float sensorVal = 55.0f;
  HysteresisSensorMonitor mon(SensorID::ESC_MOS_Temp, SensorCategory::ESC, thr, [&]() { return sensorVal; }, &logger);

  mon.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().id, SensorID::ESC_MOS_Temp);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

  // Verify sensor properties
  EXPECT_EQ(mon.getSensorID(), SensorID::ESC_MOS_Temp);
  EXPECT_EQ(mon.getCategory(), SensorCategory::ESC);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
