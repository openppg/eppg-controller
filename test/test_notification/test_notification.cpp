#include <gtest/gtest.h>
#include "sp140/notification.h"

// Native tests compile the implementation directly to validate pure helpers.
#include "../../src/sp140/notification.cpp"

TEST(NotificationRules, LowBatteryTriggersBelow20) {
  EXPECT_TRUE(shouldNotifyLowBattery(19.9f));
  EXPECT_TRUE(shouldNotifyLowBattery(1.0f));
}

TEST(NotificationRules, LowBatteryDoesNotTriggerAtOrAbove20) {
  EXPECT_FALSE(shouldNotifyLowBattery(20.0f));
  EXPECT_FALSE(shouldNotifyLowBattery(80.0f));
}

TEST(NotificationRules, LowBatteryDoesNotTriggerAtZeroOrNegative) {
  EXPECT_FALSE(shouldNotifyLowBattery(0.0f));
  EXPECT_FALSE(shouldNotifyLowBattery(-1.0f));
}

TEST(NotificationRules, BingoTriggersWhenCrossingBelow50) {
  EXPECT_TRUE(shouldNotifyBingo(49.9f, false, true));
}

TEST(NotificationRules, BingoDoesNotTriggerIfAlreadyFired) {
  EXPECT_FALSE(shouldNotifyBingo(49.9f, true, true));
}

TEST(NotificationRules, BingoDoesNotTriggerWithoutCrossingContext) {
  EXPECT_FALSE(shouldNotifyBingo(49.9f, false, false));
}

TEST(NotificationRules, BingoDoesNotTriggerAtOrAbove50) {
  EXPECT_FALSE(shouldNotifyBingo(50.0f, false, true));
  EXPECT_FALSE(shouldNotifyBingo(75.0f, false, true));
}

TEST(NotificationRules, BingoDoesNotTriggerAtZero) {
  EXPECT_FALSE(shouldNotifyBingo(0.0f, false, true));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
