#include <gtest/gtest.h>
#include <cmath>
#include "sp140/bms.h"

static const float kSentinel = BMS_CAN::TEMP_PROBE_DISCONNECTED;

TEST(BmsTempPolicy, BeforeFirstFramePassesNaNThroughSilently) {
  bool latch = false;
  const float raw[BMS_CELL_PROBE_COUNT] = {NAN, NAN, NAN, NAN};
  float out[BMS_CELL_PROBE_COUNT];
  EXPECT_FALSE(applyCellProbeTempPolicy(latch, NAN, NAN, raw, out));
  EXPECT_FALSE(latch);
  for (float v : out) EXPECT_TRUE(std::isnan(v));  // no -40 sentinel at boot
}

TEST(BmsTempPolicy, FirstFrameSetsLatchAndAppliesPolicy) {
  bool latch = false;
  const float raw[BMS_CELL_PROBE_COUNT] = {25.0f, NAN, NAN, NAN};
  float out[BMS_CELL_PROBE_COUNT];
  EXPECT_TRUE(applyCellProbeTempPolicy(latch, 30.0f, 28.0f, raw, out));
  EXPECT_TRUE(latch);
  EXPECT_FLOAT_EQ(out[0], 25.0f);
  EXPECT_TRUE(std::isnan(out[1]));   // up to 2 disconnected tolerated
  EXPECT_TRUE(std::isnan(out[2]));
  EXPECT_FLOAT_EQ(out[3], kSentinel);  // third one alerts
}

TEST(BmsTempPolicy, LatchedPolicyStillAlertsWhenTemperaturePathDies) {
  // After real data, a later all-NaN frame must not go silent.
  bool latch = false;
  const float good[BMS_CELL_PROBE_COUNT] = {25.0f, 26.0f, 27.0f, 28.0f};
  float out[BMS_CELL_PROBE_COUNT];
  ASSERT_TRUE(applyCellProbeTempPolicy(latch, 30.0f, 28.0f, good, out));

  const float dead[BMS_CELL_PROBE_COUNT] = {NAN, NAN, NAN, NAN};
  EXPECT_TRUE(applyCellProbeTempPolicy(latch, NAN, NAN, dead, out));
  int sentinels = 0;
  for (float v : out) {
    if (v == kSentinel) sentinels++;
  }
  EXPECT_EQ(sentinels, BMS_CELL_PROBE_COUNT - BMS_MAX_IGNORED_DISCONNECTED_PROBES);
}

TEST(BmsTempPolicy, MosOrBalanceReadingAloneCountsAsFrameSeen) {
  bool latch = false;
  const float raw[BMS_CELL_PROBE_COUNT] = {NAN, NAN, NAN, NAN};
  float out[BMS_CELL_PROBE_COUNT];
  EXPECT_TRUE(applyCellProbeTempPolicy(latch, NAN, 31.0f, raw, out));
  EXPECT_FLOAT_EQ(out[2], kSentinel);
  EXPECT_FLOAT_EQ(out[3], kSentinel);
}

TEST(BmsTempPolicy, AllConnectedPassesValuesThrough) {
  bool latch = false;
  const float raw[BMS_CELL_PROBE_COUNT] = {20.0f, 21.0f, 22.0f, 23.0f};
  float out[BMS_CELL_PROBE_COUNT];
  applyCellProbeTempPolicy(latch, 30.0f, 28.0f, raw, out);
  for (int i = 0; i < BMS_CELL_PROBE_COUNT; ++i) EXPECT_FLOAT_EQ(out[i], raw[i]);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
