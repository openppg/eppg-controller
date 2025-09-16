#include <gtest/gtest.h>
#include "sp140/simple_monitor.h"

// Re-use FakeLogger pattern from previous tests
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

TEST(MultiLogger, FanOutToMultipleSinks) {
  FakeLogger sink1;
  FakeLogger sink2;
  MultiLogger multi;
  multi.addSink(&sink1);
  multi.addSink(&sink2);

  // Send numeric log
  multi.log(SensorID::CPU_Temp, AlertLevel::WARN_HIGH, 55.0f);
  ASSERT_EQ(sink1.entries.size(), 1u);
  ASSERT_EQ(sink2.entries.size(), 1u);
  EXPECT_EQ(sink1.entries[0].lvl, AlertLevel::WARN_HIGH);
  EXPECT_EQ(sink2.entries[0].fval, 55.0f);

  // Send boolean log
  multi.log(SensorID::BMS_Charge_MOS, AlertLevel::CRIT_HIGH, false);
  ASSERT_EQ(sink1.entries.size(), 2u);
  ASSERT_EQ(sink2.entries.size(), 2u);
  EXPECT_TRUE(sink1.entries[1].isBool);
  EXPECT_FALSE(sink2.entries[1].bval);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
