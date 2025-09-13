// Use the native Arduino stubs provided by the repo and the real CircularBuffer
#include <gtest/gtest.h>
#include "../native_stubs/Arduino.h"

// Include the real implementations under test
#include "../../inc/sp140/shared-config.h"
#include "../../inc/sp140/throttle.h"
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

// Test potRawToPwm mapping function
TEST(ThrottleTest, PotRawToPwmMapping) {
    // Test basic mapping from ADC range (0-4095) to PWM range (1035-1950)
    EXPECT_EQ(potRawToPwm(0), 1035);        // Min ADC -> Min PWM
    EXPECT_EQ(potRawToPwm(4095), 1950);     // Max ADC -> Max PWM
    EXPECT_EQ(potRawToPwm(2047), 1492);     // Mid ADC -> Mid PWM (approx)

    // Test quarter points
    EXPECT_EQ(potRawToPwm(1024), 1263);     // 25% -> 25% of range
    EXPECT_EQ(potRawToPwm(3072), 1721);     // 75% -> 75% of range
}

// Test applyModeRampClamp function
TEST(ThrottleTest, ApplyModeRampClamp) {
    int prevPwm;

    // Test CHILL mode (mode 0) - slower ramp, lower max
    prevPwm = 1035;
    int result = applyModeRampClamp(1500, prevPwm, 0);
    EXPECT_EQ(prevPwm, result);  // prevPwm should be updated
    EXPECT_EQ(result, 1045);     // Should ramp by CHILL_MODE_RAMP_RATE (10)

    // Test SPORT mode (mode 1) - faster ramp, higher max
    prevPwm = 1035;
    result = applyModeRampClamp(1500, prevPwm, 1);
    EXPECT_EQ(result, 1062);     // Should ramp by SPORT_MODE_RAMP_RATE (27)

    // Test CHILL mode max PWM clamping
    prevPwm = 1840;
    result = applyModeRampClamp(1900, prevPwm, 0);
    EXPECT_EQ(result, 1850);     // Should clamp to CHILL_MODE_MAX_PWM

    // Test SPORT mode allows higher PWM
    prevPwm = 1840;
    result = applyModeRampClamp(1900, prevPwm, 1);
    EXPECT_EQ(result, 1867);     // Should ramp by 27, not clamp yet
}

// Test throttle filter functions
TEST(ThrottleTest, ThrottleFiltering) {
    // Clear buffer first
    throttleFilterClear();

    // Empty buffer should return 0
    EXPECT_EQ(throttleFilterAverage(), 0);

    // Add some values and test averaging
    throttleFilterPush(1000);
    EXPECT_EQ(throttleFilterAverage(), 1000);

    throttleFilterPush(1200);
    EXPECT_EQ(throttleFilterAverage(), 1100);  // (1000+1200)/2

    throttleFilterPush(1400);
    EXPECT_EQ(throttleFilterAverage(), 1200);  // (1000+1200+1400)/3

    // Test reset functionality
    throttleFilterReset(1500);
    EXPECT_EQ(throttleFilterAverage(), 1500);  // Should be filled with 1500
}

// Test resetThrottleState function
TEST(ThrottleTest, ResetThrottleState) {
    int prevPwm = 1500;

    // Add some values to buffer first
    throttleFilterPush(1200);
    throttleFilterPush(1300);
    throttleFilterPush(1400);

    // Reset should clear buffer and reset prevPwm
    resetThrottleState(prevPwm);

    EXPECT_EQ(prevPwm, 1035);              // Should reset to ESC_MIN_PWM
    EXPECT_EQ(throttleFilterAverage(), 0); // Buffer should be empty
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    RUN_ALL_TESTS();
    // Always return zero-code and allow PlatformIO to parse results
    return 0;
}
