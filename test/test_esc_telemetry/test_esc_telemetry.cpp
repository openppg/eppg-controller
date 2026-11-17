#include <gtest/gtest.h>
#include <cmath>
#include "sp140/esc_telemetry_logic.h"

// ---- Stale-data guard + runtime accumulation ----

TEST(EscRuntime, FirstNewFrameIsFreshButAddsNoRuntime) {
  EscRuntimeTracker t;
  EXPECT_TRUE(escRuntimeOnFrame(t, 500));
  EXPECT_EQ(t.accumulatedMs, 0u);
  EXPECT_FALSE(t.firstUpdate);
  EXPECT_EQ(t.lastTime10ms, 500u);
}

TEST(EscRuntime, CounterZeroAtBootIsTreatedAsStale) {
  // Matches the original behaviour: the tracker starts at 0, so a first
  // response whose counter is exactly 0 is ignored until the counter moves.
  EscRuntimeTracker t;
  EXPECT_FALSE(escRuntimeOnFrame(t, 0));
  EXPECT_TRUE(t.firstUpdate);
  EXPECT_TRUE(escRuntimeOnFrame(t, 1));
}

TEST(EscRuntime, UnchangedCounterIsStale) {
  EscRuntimeTracker t;
  ASSERT_TRUE(escRuntimeOnFrame(t, 100));
  ASSERT_TRUE(escRuntimeOnFrame(t, 105));
  const uint32_t before = t.accumulatedMs;
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(escRuntimeOnFrame(t, 105));
  }
  EXPECT_EQ(t.accumulatedMs, before);
}

TEST(EscRuntime, AccumulatesTenMsPerTick) {
  EscRuntimeTracker t;
  escRuntimeOnFrame(t, 100);
  escRuntimeOnFrame(t, 105);
  escRuntimeOnFrame(t, 125);
  EXPECT_EQ(t.accumulatedMs, 250u);  // 25 ticks * 10 ms
}

TEST(EscRuntime, HandlesUint16Rollover) {
  EscRuntimeTracker t;
  escRuntimeOnFrame(t, 65000);
  EXPECT_TRUE(escRuntimeOnFrame(t, 100));
  EXPECT_EQ(t.accumulatedMs, 6360u);  // 636 ticks across the wrap
}

TEST(EscRuntime, RuntimeKeepsGrowingOverManyWraps) {
  EscRuntimeTracker t;
  uint16_t counter = 1;
  escRuntimeOnFrame(t, counter);
  // 2 hours of 50 ms frames (5 ticks each) wraps the counter ~11 times.
  const int frames = 2 * 60 * 60 * 20;
  for (int i = 0; i < frames; ++i) {
    counter = static_cast<uint16_t>(counter + 5);
    ASSERT_TRUE(escRuntimeOnFrame(t, counter));
  }
  EXPECT_EQ(t.accumulatedMs, static_cast<uint32_t>(frames) * 50u);
}

TEST(EscRuntime, DisconnectRestartsRuntimeAndKeepsStaleGuard) {
  EscRuntimeTracker t;
  escRuntimeOnFrame(t, 100);
  escRuntimeOnFrame(t, 200);
  ASSERT_EQ(t.accumulatedMs, 1000u);

  escRuntimeOnDisconnect(t);
  EXPECT_EQ(t.accumulatedMs, 0u);
  // The library keeps returning its last response: still stale.
  EXPECT_FALSE(escRuntimeOnFrame(t, 200));
  // ESC comes back: first new frame restarts runtime from zero.
  EXPECT_TRUE(escRuntimeOnFrame(t, 7));
  EXPECT_EQ(t.accumulatedMs, 0u);
  EXPECT_TRUE(escRuntimeOnFrame(t, 9));
  EXPECT_EQ(t.accumulatedMs, 20u);
}

// ---- Link timeout ----

TEST(EscTimeout, NeverHeardFromIsTimedOut) {
  EXPECT_TRUE(escTelemetryTimedOut(0, 0, 50));
  EXPECT_TRUE(escTelemetryTimedOut(123456, 0, 50));
}

TEST(EscTimeout, BoundaryIsStrictlyGreaterThan) {
  EXPECT_FALSE(escTelemetryTimedOut(1050, 1000, 50));
  EXPECT_TRUE(escTelemetryTimedOut(1051, 1000, 50));
}

TEST(EscTimeout, SurvivesMillisWrap) {
  const uint32_t last = 0xFFFFFFF0u;  // 16 ms before wrap
  EXPECT_FALSE(escTelemetryTimedOut(last + 40u, last, 50));  // wraps to 0x18
  EXPECT_TRUE(escTelemetryTimedOut(last + 60u, last, 50));
}

// ---- Motor temperature sanitizing ----

TEST(MotorTemp, ValidRangePassesThrough) {
  EXPECT_FLOAT_EQ(sanitizeMotorTempC(25.0f), 25.0f);
  EXPECT_FLOAT_EQ(sanitizeMotorTempC(140.0f), 140.0f);   // inclusive max
  EXPECT_FLOAT_EQ(sanitizeMotorTempC(-19.9f), -19.9f);
}

TEST(MotorTemp, DisconnectedOrOutOfRangeBecomesNaN) {
  EXPECT_TRUE(std::isnan(sanitizeMotorTempC(149.0f)));   // open thermistor
  EXPECT_TRUE(std::isnan(sanitizeMotorTempC(140.1f)));
  EXPECT_TRUE(std::isnan(sanitizeMotorTempC(-20.0f)));   // exclusive min
  EXPECT_TRUE(std::isnan(sanitizeMotorTempC(-40.0f)));
  EXPECT_TRUE(std::isnan(sanitizeMotorTempC(NAN)));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
