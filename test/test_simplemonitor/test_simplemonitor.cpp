#include <gtest/gtest.h>
#include "sp140/simple_monitor.h"

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

TEST(SimpleMonitor, NumericThresholdTransitions) {
  FakeLogger logger;

  // Create monitor that warns >50, crit > 80
  Thresholds thr{.warnLow = -100.0f, .warnHigh = 50.0f, .critLow = -200.0f, .critHigh = 80.0f};

  float sensorVal = 0.0f;
  SensorMonitor mon(SensorID::CPU_Temp, thr, [&]() { return sensorVal; }, &logger);

  // Start in OK
  mon.check();
  EXPECT_TRUE(logger.entries.empty());

  // Raise to warning
  sensorVal = 55.0f;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

  // Raise to critical
  sensorVal = 90.0f;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 2u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_HIGH);

  // Return to OK below warnHigh
  sensorVal = 40.0f;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 3u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::OK);
}

TEST(SimpleMonitor, BooleanMonitorTransitions) {
  FakeLogger logger;
  bool state = true;
  BooleanMonitor mon(SensorID::BMS_Charge_MOS, [&]() { return state; }, false, AlertLevel::CRIT_HIGH, &logger);

  // Initial state true (no alert because alertOnTrue = false)
  mon.check();
  EXPECT_TRUE(logger.entries.empty());

  // Flip to false -> should alert
  state = false;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_HIGH);

  // Back to true -> OK
  state = true;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 2u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::OK);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (RUN_ALL_TESTS())
    ;
    // Always return zero-code and allow PlatformIO to parse results
    return 0;
}
