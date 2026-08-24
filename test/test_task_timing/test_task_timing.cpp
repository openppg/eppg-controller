#include <gtest/gtest.h>

#include <stdint.h>

#include "sp140/task_timing.h"

TEST(TaskTiming, KeepsBaseWhenWorkFinishesWithinPeriod) {
  EXPECT_EQ(phaseAlignedWakeBase(100, 132, 33), 100u);
}

TEST(TaskTiming, DropsOneSlotWithoutShiftingPhase) {
  // A 37 ms frame on a 33 ms cadence advances the base to 33 ms. The caller's
  // vTaskDelayUntil then targets 66 ms, rather than now + 33 = 70 ms.
  EXPECT_EQ(phaseAlignedWakeBase(0, 37, 33), 33u);
}

TEST(TaskTiming, DropsMultipleElapsedSlots) {
  EXPECT_EQ(phaseAlignedWakeBase(10, 145, 40), 130u);
}

TEST(TaskTiming, HandlesTickCounterWraparound) {
  const uint32_t lastWake = UINT32_MAX - 10;
  EXPECT_EQ(phaseAlignedWakeBase(lastWake, 20, 10), 19u);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
