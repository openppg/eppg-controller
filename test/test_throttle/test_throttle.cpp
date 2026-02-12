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
    EXPECT_EQ(result, 1043);     // Should ramp by CHILL_MODE_RAMP_RATE (8)

    // Test SPORT mode (mode 1) - faster ramp, higher max
    prevPwm = 1035;
    result = applyModeRampClamp(1500, prevPwm, 1);
    EXPECT_EQ(result, 1062);     // Should ramp by SPORT_MODE_RAMP_RATE (27)

    // Test CHILL mode max PWM clamping
    prevPwm = 1840;
    result = applyModeRampClamp(1900, prevPwm, 0);
    EXPECT_EQ(result, 1600);     // Should clamp to CHILL_MODE_MAX_PWM

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

// Test potRawToModePwm - mode-aware mapping
TEST(ThrottleTest, PotRawToModePwmMapping) {
    // SPORT mode (mode 1) should be identical to potRawToPwm (full range)
    EXPECT_EQ(potRawToModePwm(0, 1), 1035);
    EXPECT_EQ(potRawToModePwm(4095, 1), 1950);
    EXPECT_EQ(potRawToModePwm(2047, 1), 1492);
    EXPECT_EQ(potRawToModePwm(1024, 1), 1263);
    EXPECT_EQ(potRawToModePwm(3072, 1), 1721);

    // CHILL mode (mode 0) maps full physical range to 1035-1600
    // Range: 1600 - 1035 = 565
    EXPECT_EQ(potRawToModePwm(0, 0), 1035);        // Min ADC -> Min PWM
    EXPECT_EQ(potRawToModePwm(4095, 0), 1600);     // Max ADC -> CHILL max
    EXPECT_EQ(potRawToModePwm(2048, 0), 1317);     // 50% -> mid chill range
    EXPECT_EQ(potRawToModePwm(1024, 0), 1176);     // 25% -> 25% of chill range
    EXPECT_EQ(potRawToModePwm(3072, 0), 1458);     // 75% -> 75% of chill range
}

// Test calculateCruisePwm function - ensures cruise uses same mode-aware mapping
TEST(ThrottleTest, CalculateCruisePwmBasic) {
    // 50% pot in SPORT mode (mode 1) - should match potRawToModePwm exactly
    // potRawToModePwm(2048, 1) = 1492, cruise cap at 60% = 1584
    uint16_t result = calculateCruisePwm(2048, 1, 0.60);
    EXPECT_EQ(result, 1492);  // Below cruise cap

    // 50% pot in CHILL mode (mode 0) - uses chill range mapping
    // potRawToModePwm(2048, 0) = 1317, cruise cap at 60% = 1584
    result = calculateCruisePwm(2048, 0, 0.60);
    EXPECT_EQ(result, 1317);  // Chill mode maps full physical range to 1035-1600
}

// Test that cruise in chill mode uses the same mapping as normal throttle
TEST(ThrottleTest, CalculateCruisePwmChillModeConsistency) {
    // Cruise should use the same mode-aware mapping as normal throttle.
    // In chill mode, full physical range maps to 1035-1600.
    int normalThrottle = potRawToModePwm(2048, 0);  // 1317
    EXPECT_EQ(normalThrottle, 1317);

    // Cruise in chill mode should match the normal throttle mapping
    uint16_t cruiseThrottle = calculateCruisePwm(2048, 0, 0.60);
    EXPECT_EQ(cruiseThrottle, (uint16_t)normalThrottle);  // Both use same mapping
}

// Test chill mode at full physical range
TEST(ThrottleTest, CalculateCruisePwmChillModeFullRange) {
    // 100% pot in CHILL mode - maps to exactly CHILL_MODE_MAX_PWM (1600)
    // potRawToModePwm(4095, 0) = 1600, cruise cap at 70% = 1675
    uint16_t result = calculateCruisePwm(4095, 0, 0.70);
    EXPECT_EQ(result, 1600);  // Full physical range -> CHILL_MODE_MAX_PWM

    // 80% pot in CHILL mode - potRawToModePwm(3276, 0) = 1486
    // Cruise cap at 70% = 1675, so 1486 is below cap
    result = calculateCruisePwm(3276, 0, 0.70);
    EXPECT_EQ(result, 1486);  // Full granular control, no clamping
}

// Test cruise max percentage capping
TEST(ThrottleTest, CalculateCruisePwmCruiseMaxCap) {
    // Cruise max cap at 60%: 1035 + (1950-1035)*0.6 = 1035 + 549 = 1584

    // In SPORT mode at full throttle, cruise cap should apply
    // potRawToModePwm(4095, 1) = 1950, cruise cap at 60% = 1584
    uint16_t result = calculateCruisePwm(4095, 1, 0.60);
    EXPECT_EQ(result, 1584);  // Capped by cruise max

    // At 70% cruise cap: 1035 + 915*0.7 = 1675
    result = calculateCruisePwm(4095, 1, 0.70);
    EXPECT_EQ(result, 1675);  // Higher cruise cap

    // In chill mode at full throttle, chill max (1600) < cruise cap (1675)
    // potRawToModePwm(4095, 0) = 1600
    result = calculateCruisePwm(4095, 0, 0.70);
    EXPECT_EQ(result, 1600);  // Chill mode max is the limiter
}

// Test edge cases
TEST(ThrottleTest, CalculateCruisePwmEdgeCases) {
    // Minimum pot value - same in both modes
    uint16_t result = calculateCruisePwm(0, 0, 0.60);
    EXPECT_EQ(result, 1035);  // ESC_MIN_PWM

    result = calculateCruisePwm(0, 1, 0.60);
    EXPECT_EQ(result, 1035);  // ESC_MIN_PWM

    // Low throttle (25%) in chill mode - uses chill mapping
    // potRawToModePwm(1024, 0) = 1176
    result = calculateCruisePwm(1024, 0, 0.60);
    EXPECT_EQ(result, 1176);  // Chill mode maps to reduced range

    // Low throttle (25%) in sport mode - uses full mapping
    // potRawToModePwm(1024, 1) = 1263
    result = calculateCruisePwm(1024, 1, 0.60);
    EXPECT_EQ(result, 1263);
}

// Test cruise activation range check
TEST(ThrottleTest, IsPotInCruiseActivationRange) {
    // Using real values: engagement = 5% of 4095 = 204, max = 70% of 4095 = 2866
    const uint16_t engagementLevel = 204;
    const float maxActivationPct = 0.70;

    // Below engagement level - should NOT be in range
    EXPECT_FALSE(isPotInCruiseActivationRange(0, engagementLevel, maxActivationPct));
    EXPECT_FALSE(isPotInCruiseActivationRange(100, engagementLevel, maxActivationPct));
    EXPECT_FALSE(isPotInCruiseActivationRange(203, engagementLevel, maxActivationPct));

    // At engagement level - should be in range
    EXPECT_TRUE(isPotInCruiseActivationRange(204, engagementLevel, maxActivationPct));

    // In valid range - should be in range
    EXPECT_TRUE(isPotInCruiseActivationRange(500, engagementLevel, maxActivationPct));
    EXPECT_TRUE(isPotInCruiseActivationRange(1000, engagementLevel, maxActivationPct));
    EXPECT_TRUE(isPotInCruiseActivationRange(2000, engagementLevel, maxActivationPct));
    EXPECT_TRUE(isPotInCruiseActivationRange(2866, engagementLevel, maxActivationPct));  // At 70%

    // Above max activation - should NOT be in range
    EXPECT_FALSE(isPotInCruiseActivationRange(2867, engagementLevel, maxActivationPct));
    EXPECT_FALSE(isPotInCruiseActivationRange(3000, engagementLevel, maxActivationPct));
    EXPECT_FALSE(isPotInCruiseActivationRange(4095, engagementLevel, maxActivationPct));  // Full throttle
}

// Test cruise disengagement threshold
TEST(ThrottleTest, ShouldPotDisengageCruise) {
    // Using 80% threshold
    const float thresholdPct = 0.80;

    // Cruise activated at 50% pot (2048)
    // Disengage threshold = 2048 * 0.80 = 1638

    // Below threshold - should NOT disengage
    EXPECT_FALSE(shouldPotDisengageCruise(0, 2048, thresholdPct));
    EXPECT_FALSE(shouldPotDisengageCruise(1000, 2048, thresholdPct));
    EXPECT_FALSE(shouldPotDisengageCruise(1637, 2048, thresholdPct));

    // At or above threshold - should disengage
    EXPECT_TRUE(shouldPotDisengageCruise(1638, 2048, thresholdPct));
    EXPECT_TRUE(shouldPotDisengageCruise(2000, 2048, thresholdPct));
    EXPECT_TRUE(shouldPotDisengageCruise(2048, 2048, thresholdPct));  // At activation point
    EXPECT_TRUE(shouldPotDisengageCruise(3000, 2048, thresholdPct));  // Above activation
    EXPECT_TRUE(shouldPotDisengageCruise(4095, 2048, thresholdPct));  // Full throttle
}

// Test disengagement at different activation points
TEST(ThrottleTest, ShouldPotDisengageCruiseVariousActivations) {
    const float thresholdPct = 0.80;

    // Low cruise (25% pot = 1024), threshold = 819
    EXPECT_FALSE(shouldPotDisengageCruise(818, 1024, thresholdPct));
    EXPECT_TRUE(shouldPotDisengageCruise(819, 1024, thresholdPct));
    EXPECT_TRUE(shouldPotDisengageCruise(1024, 1024, thresholdPct));

    // High cruise (60% pot = 2457), threshold = 1965
    EXPECT_FALSE(shouldPotDisengageCruise(1964, 2457, thresholdPct));
    EXPECT_TRUE(shouldPotDisengageCruise(1965, 2457, thresholdPct));
    EXPECT_TRUE(shouldPotDisengageCruise(2457, 2457, thresholdPct));

    // Very low cruise (10% pot = 409), threshold = 327
    EXPECT_FALSE(shouldPotDisengageCruise(326, 409, thresholdPct));
    EXPECT_TRUE(shouldPotDisengageCruise(327, 409, thresholdPct));
}

// Test edge case: cruise at minimum engagement
TEST(ThrottleTest, CruiseActivationEdgeCases) {
    const uint16_t engagementLevel = 204;  // 5% of 4095
    const float maxActivationPct = 0.70;

    // Exactly at engagement level should be valid
    EXPECT_TRUE(isPotInCruiseActivationRange(204, engagementLevel, maxActivationPct));

    // Just below engagement should be invalid
    EXPECT_FALSE(isPotInCruiseActivationRange(203, engagementLevel, maxActivationPct));

    // At max threshold (70% = 2866) should be valid
    EXPECT_TRUE(isPotInCruiseActivationRange(2866, engagementLevel, maxActivationPct));

    // Just above max threshold should be invalid
    EXPECT_FALSE(isPotInCruiseActivationRange(2867, engagementLevel, maxActivationPct));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    RUN_ALL_TESTS();
    // Always return zero-code and allow PlatformIO to parse results
    return 0;
}
