#include <gtest/gtest.h>

// Create minimal stubs for ESP32 dependencies to avoid linking issues
#define ARDUINO_H  // Prevent Arduino.h from being included

// Include the actual source file
#include "../../src/sp140/throttle.cpp"

TEST(ThrottleTest, LimitedThrottleNoLimiting) {
    // Test cases where no limiting should occur

    // No change - should return current value
    EXPECT_EQ(limitedThrottle(1500, 1500, 50), 1500);

    // Small acceleration within threshold
    EXPECT_EQ(limitedThrottle(1540, 1500, 50), 1540);  // +40, threshold 50
    EXPECT_EQ(limitedThrottle(1549, 1500, 50), 1549);  // +49, threshold 50

    // Small deceleration within threshold (considering DECEL_MULTIPLIER = 2.0)
    EXPECT_EQ(limitedThrottle(1400, 1500, 50), 1400);  // -100, threshold*2 = 100
    EXPECT_EQ(limitedThrottle(1401, 1500, 50), 1401);  // -99, threshold*2 = 100
}

TEST(ThrottleTest, LimitedThrottleAccelerationLimiting) {
    // Test acceleration limiting (current > last + threshold)

    // Accelerating too fast - should limit to last + threshold
    EXPECT_EQ(limitedThrottle(1600, 1500, 50), 1550);  // +100 -> +50
    EXPECT_EQ(limitedThrottle(1580, 1500, 50), 1550);  // +80 -> +50
    EXPECT_EQ(limitedThrottle(1700, 1500, 50), 1550);  // +200 -> +50

    // Edge case: exactly at threshold (should still limit)
    EXPECT_EQ(limitedThrottle(1550, 1500, 50), 1550);  // +50 -> +50

    // Different threshold values
    EXPECT_EQ(limitedThrottle(1650, 1500, 100), 1600);  // +150 -> +100
    EXPECT_EQ(limitedThrottle(1520, 1500, 10), 1510);  // +20 -> +10
}

TEST(ThrottleTest, LimitedThrottleDecelerationLimiting) {
    // Test deceleration limiting (last - current >= threshold * DECEL_MULTIPLIER)
    // DECEL_MULTIPLIER = 2.0

    // Decelerating too fast - should limit to last - (threshold * 2)
    EXPECT_EQ(limitedThrottle(1300, 1500, 50), 1400);  // -200 -> -100 (50*2)
    EXPECT_EQ(limitedThrottle(1350, 1500, 50), 1400);  // -150 -> -100 (50*2)
    EXPECT_EQ(limitedThrottle(1200, 1500, 50), 1400);  // -300 -> -100 (50*2)

    // Edge case: exactly at decel threshold (should still limit)
    EXPECT_EQ(limitedThrottle(1400, 1500, 50), 1400);  // -100 -> -100 (50*2)

    // Different threshold values
    EXPECT_EQ(limitedThrottle(1300, 1500, 75), 1350);  // -200 -> -150 (75*2)
    EXPECT_EQ(limitedThrottle(1480, 1500, 10), 1480);  // -20 -> -20 (10*2=20, so -20 is OK)
    EXPECT_EQ(limitedThrottle(1470, 1500, 10), 1480);  // -30 -> -20 (10*2)
}

TEST(ThrottleTest, LimitedThrottleRealWorldScenarios) {
    // Test realistic PWM values (ESC_MIN_PWM = 1035, ESC_MAX_PWM = 1950)

    // Gradual acceleration from idle
    EXPECT_EQ(limitedThrottle(1100, 1035, 30), 1065);  // +65 -> +30
    EXPECT_EQ(limitedThrottle(1065, 1035, 30), 1065);  // +30 -> +30 (no limit)

    // Emergency deceleration from high throttle
    EXPECT_EQ(limitedThrottle(1035, 1500, 100), 1300); // -465 -> -200 (100*2)
    EXPECT_EQ(limitedThrottle(1035, 1800, 50), 1700);  // -765 -> -100 (50*2)

    // Performance mode differences (different thresholds)
    // Chill mode (lower threshold)
    EXPECT_EQ(limitedThrottle(1600, 1500, 50), 1550);  // +100 -> +50
    // Sport mode (higher threshold)
    EXPECT_EQ(limitedThrottle(1650, 1500, 120), 1620); // +150 -> +120
}

TEST(ThrottleTest, LimitedThrottleEdgeCases) {
    // Test edge cases and boundary conditions

    // Zero values
    EXPECT_EQ(limitedThrottle(0, 0, 10), 0);
    EXPECT_EQ(limitedThrottle(20, 0, 10), 10);      // +20 -> +10
    EXPECT_EQ(limitedThrottle(0, 50, 10), 30);      // -50 -> -20 (10*2)

    // Large values
    EXPECT_EQ(limitedThrottle(2000, 1000, 100), 1100); // +1000 -> +100
    EXPECT_EQ(limitedThrottle(500, 1000, 100), 800);   // -500 -> -200 (100*2)

    // Negative values (shouldn't occur in practice but test robustness)
    EXPECT_EQ(limitedThrottle(-10, 0, 5), -10);     // -10 -> -10 (exactly at limit 5*2=10)
    EXPECT_EQ(limitedThrottle(-20, 0, 5), -10);     // -20 -> -10 (5*2)

    // Very small threshold
    EXPECT_EQ(limitedThrottle(1002, 1000, 1), 1001); // +2 -> +1
    EXPECT_EQ(limitedThrottle(997, 1000, 1), 998);   // -3 -> -2 (1*2)
}

TEST(ThrottleTest, LimitedThrottleSequentialCalls) {
    // Test multiple sequential calls to simulate real usage

    int current = 1035;  // Start at ESC_MIN_PWM
    int threshold = 50;

    // Gradual acceleration
    current = limitedThrottle(1200, current, threshold);
    EXPECT_EQ(current, 1085);  // 1035 + 50

    current = limitedThrottle(1200, current, threshold);
    EXPECT_EQ(current, 1135);  // 1085 + 50

    current = limitedThrottle(1200, current, threshold);
    EXPECT_EQ(current, 1185);  // 1135 + 50

    current = limitedThrottle(1200, current, threshold);
    EXPECT_EQ(current, 1200);  // 1185 + 15 (finally reaches target)

    // Emergency stop
    current = limitedThrottle(1035, current, threshold);
    EXPECT_EQ(current, 1100);  // 1200 - 100 (50*2)

    current = limitedThrottle(1035, current, threshold);
    EXPECT_EQ(current, 1035);  // 1100 - 65 (reaches target)
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    RUN_ALL_TESTS();
    // Always return zero-code and allow PlatformIO to parse results
    return 0;
}
