#include <gtest/gtest.h>
#include <atomic>
#include <thread>
#include "platform/peripheral.h"
#include "platform/barometer.h"
#include "sp140/altimeter.h"

namespace {
std::atomic<int> conversions{0};
std::atomic<bool> fail{false};
float pressure = 1000.0f;
STR_DEVICE_DATA_140_V1 settings{};
}
namespace platform {
bool Barometer::initialize() { return true; }
bool Barometer::readSample(float* t, float* p) {
  ++conversions;
  if (fail) return false;
  *p = pressure;
  *t = pressure - 1000.0f;
  return true;
}
float Barometer::readPressure() { return pressure * 100; }
float Barometer::readTemperature() { return pressure - 1000.0f; }
}
extern "C" void eppg_gpio_write(int, int) {}
class AltimeterTest : public testing::Test {
 protected:
  static void SetUpTestSuite() { settings.sea_pressure = 1013.25f; ASSERT_TRUE(setupAltimeter()); }
  void SetUp() override { fail = false; pressure = 1000.0f; setGroundAltitude(settings); }
};
TEST_F(AltimeterTest, ConsumersDoNotPerformConversions) {
  const auto before = conversions.load();
  for (int i = 0; i < 100; ++i) {
    getAltitude(settings); getVerticalSpeed(); getBaroTemperature(); getBaroPressure();
  }
  EXPECT_EQ(conversions, before);
}
TEST_F(AltimeterTest, OneConversionPublishesAllFields) {
  pressure = 1005.0f;
  const auto before = conversions.load();
  ASSERT_TRUE(sampleBarometer(settings));
  const auto s = getBarometerSnapshot();
  EXPECT_EQ(conversions, before + 1);
  EXPECT_TRUE(s.valid); EXPECT_FLOAT_EQ(s.pressure, 1005.0f);
  EXPECT_FLOAT_EQ(s.temperature, 5.0f); EXPECT_LT(s.altitude, 0.0f);
}
TEST_F(AltimeterTest, FailedConversionPreservesLastCompleteSample) {
  const auto before = getBarometerSnapshot(); fail = true;
  EXPECT_FALSE(sampleBarometer(settings));
  const auto after = getBarometerSnapshot();
  EXPECT_FLOAT_EQ(after.altitude, before.altitude);
  EXPECT_FLOAT_EQ(after.pressure, before.pressure);
  EXPECT_EQ(after.sampledAt, before.sampledAt);
}
TEST_F(AltimeterTest, DatumResetPublishesZeroAltitudeAndVario) {
  pressure = 990.0f; ASSERT_TRUE(sampleBarometer(settings));
  setGroundAltitude(settings);
  const auto s = getBarometerSnapshot();
  EXPECT_FLOAT_EQ(s.altitude, 0.0f); EXPECT_FLOAT_EQ(s.verticalSpeed, 0.0f);
}
TEST_F(AltimeterTest, ReadersNeverMixFieldsFromDifferentConversions) {
  std::atomic<bool> done{false};
  std::thread writer([&] { for (int i = 0; i < 10000; ++i) {
    pressure = 1000.0f + i % 20; sampleBarometer(settings);
  } done = true; });
  while (!done) {
    const auto s = getBarometerSnapshot();
    EXPECT_FLOAT_EQ(s.temperature, s.pressure - 1000.0f);
  }
  writer.join();
}
int main(int argc, char** argv) { testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
