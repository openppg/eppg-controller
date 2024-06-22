#include <Arduino.h>
#include <unity.h>

struct BatteryVoltagePoint {
  float voltage;
  float percent;
};

static BatteryVoltagePoint batteryLevels[] = {
    {101, 100}, {94.8, 90}, {93.36, 80}, {91.68, 70}, {89.76, 60},
    {87.6, 50}, {85.2, 40}, {82.32, 30}, {80.16, 20}, {78, 10}, {60, 0}
};

double mapd(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float getBatteryPercent(float voltage) {
    // Calculate the number of voltage-percentage mappings
    int numLevels = sizeof(batteryLevels) / sizeof(BatteryVoltagePoint);

    // Handle edge cases where the voltage is outside the defined range
    if (voltage >= batteryLevels[0].voltage) {
        return batteryLevels[0].percent;
    } else if (voltage <= batteryLevels[numLevels - 1].voltage) {
        return batteryLevels[numLevels - 1].percent;
    }

    // Iterate through the voltage-percentage mappings
    for (int i = 0; i < numLevels - 1; i++) {
        // Check if the input voltage is between the current and next mapping
        if (voltage <= batteryLevels[i].voltage && voltage > batteryLevels[i + 1].voltage) {
            // Interpolate the percentage between the current and next mapping
            return mapd(voltage, batteryLevels[i + 1].voltage, batteryLevels[i].voltage,
                        batteryLevels[i + 1].percent, batteryLevels[i].percent);
        }
    }

    return 0; // Fallback, should never reach here
}

void setUp(void)
{
  // set stuff up here
}

void tearDown(void)
{
  // clean stuff up here
}

// Test cases for getBatteryPercent function
void test_battery_percent_above_max_voltage() {
    float voltage = 102.0;
    float expected = 100.0;  // 101V corresponds to 100%
    TEST_ASSERT_EQUAL_FLOAT(expected, getBatteryPercent(voltage));
}

void test_battery_percent_below_min_voltage() {
    float voltage = 59.0;
    float expected = 0.0;
    TEST_ASSERT_EQUAL_FLOAT(expected, getBatteryPercent(voltage));
}

void test_battery_percent_between_voltages() {
    float voltage = 88.0;
    float expected = 55.0;  // Interpolated value between 60% (89.76V) and 50% (87.6V)
    TEST_ASSERT_EQUAL_FLOAT(expected, getBatteryPercent(voltage));
}

void test_battery_percent_exact_voltage_point() {
    float voltage = 87.6;
    float expected = 50.0;
    TEST_ASSERT_EQUAL_FLOAT(expected, getBatteryPercent(voltage));
}

void test_battery_percent_lowest_voltage() {
    float voltage = 60.0;
    float expected = 0.0;
    TEST_ASSERT_EQUAL_FLOAT(expected, getBatteryPercent(voltage));
}

void test_battery_percent_highest_voltage() {
    float voltage = 101.0;
    float expected = 100.0;
    TEST_ASSERT_EQUAL_FLOAT(expected, getBatteryPercent(voltage));
}

void setup()
{
  // NOTE!!! Wait for >2 secs
  // if board doesn't support software reset via Serial.DTR/RTS
  delay(2000);

  UNITY_BEGIN(); // IMPORTANT LINE!
  RUN_TEST(test_battery_percent_above_max_voltage);
  RUN_TEST(test_battery_percent_below_min_voltage);
  RUN_TEST(test_battery_percent_between_voltages);
  RUN_TEST(test_battery_percent_exact_voltage_point);
  RUN_TEST(test_battery_percent_lowest_voltage);
  RUN_TEST(test_battery_percent_highest_voltage);
  UNITY_END(); // stop unit testing
}

void loop()
{
  // Do nothing here
}
