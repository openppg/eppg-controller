#include <gtest/gtest.h>
#include "sp140/simple_monitor.h"
#include "sp140/monitor_config.h"

// Fake logger to capture log calls
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

TEST(SimpleMonitor, NumericThresholdTransitions) {
  FakeLogger logger;

  // Create monitor that warns >50, crit > 80
  Thresholds thr{.warnLow = -100.0f, .warnHigh = 50.0f, .critLow = -200.0f, .critHigh = 80.0f};

  float sensorVal = 0.0f;
  SensorMonitor mon(SensorID::CPU_Temp, thr, [&]() { return sensorVal; }, &logger);

  // Start in OK
  mon.check();
  EXPECT_TRUE(logger.entries.empty());

  // Raise to warning
  sensorVal = 55.0f;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

  // Raise to critical
  sensorVal = 90.0f;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 2u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_HIGH);

  // Return to OK below warnHigh
  sensorVal = 40.0f;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 3u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::OK);
}

TEST(SimpleMonitor, BooleanMonitorTransitions) {
  FakeLogger logger;
  bool state = true;
  BooleanMonitor mon(SensorID::BMS_Charge_MOS, [&]() { return state; }, false, AlertLevel::CRIT_HIGH, &logger);

  // Initial state true (no alert because alertOnTrue = false)
  mon.check();
  EXPECT_TRUE(logger.entries.empty());

  // Flip to false -> should alert
  state = false;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_HIGH);

  // Back to true -> OK
  state = true;
  mon.check();
  ASSERT_EQ(logger.entries.size(), 2u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::OK);
}

TEST(SimpleMonitor, BMSSOCAlerts) {
  FakeLogger logger;

  // Use the single source of truth for thresholds
  float fakeSOC = 50.0f; // Start with a healthy SOC
  SensorMonitor socMonitor(SensorID::BMS_SOC, bmsSOCThresholds, [&]() { return fakeSOC; }, &logger);

  // 1. Initial check, should be OK, no log
  socMonitor.check();
  EXPECT_TRUE(logger.entries.empty());

  // 2. Drop to warning level
  fakeSOC = 14.0f;
  socMonitor.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_LOW);
  EXPECT_EQ(logger.entries.back().fval, 14.0f);

  // 3. Drop to critical level
  fakeSOC = 4.0f;
  socMonitor.check();
  ASSERT_EQ(logger.entries.size(), 2u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_LOW);
  EXPECT_EQ(logger.entries.back().fval, 4.0f);

  // 4. Check again at critical, should not log again as state hasn't changed
  socMonitor.check();
  ASSERT_EQ(logger.entries.size(), 2u);

  // 5. Rise back to warning level
  fakeSOC = 12.0f;
  socMonitor.check();
  ASSERT_EQ(logger.entries.size(), 3u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_LOW);
  EXPECT_EQ(logger.entries.back().fval, 12.0f);

  // 6. Rise back to OK
  fakeSOC = 50.0f;
  socMonitor.check();
  ASSERT_EQ(logger.entries.size(), 4u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::OK);
  EXPECT_EQ(logger.entries.back().fval, 50.0f);
}

TEST(SimpleMonitor, BMSVoltageAlerts) {
  FakeLogger logger;
  float fakeVoltage;

  // -- High Cell Voltage --
  fakeVoltage = 4.0f;
  SensorMonitor highCellMon(SensorID::BMS_High_Cell_Voltage, bmsHighCellVoltageThresholds, [&]() { return fakeVoltage; }, &logger);
  highCellMon.check(); // OK
  EXPECT_TRUE(logger.entries.empty());

  fakeVoltage = 4.15f; // Warn
  highCellMon.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

  fakeVoltage = 4.25f; // Crit
  highCellMon.check();
  ASSERT_EQ(logger.entries.size(), 2u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_HIGH);

  // -- Low Cell Voltage --
  logger.entries.clear();
  fakeVoltage = 3.5f;
  SensorMonitor lowCellMon(SensorID::BMS_Low_Cell_Voltage, bmsLowCellVoltageThresholds, [&]() { return fakeVoltage; }, &logger);
  lowCellMon.check(); // OK
  EXPECT_TRUE(logger.entries.empty());

  fakeVoltage = 3.1f; // Warn
  lowCellMon.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_LOW);

  fakeVoltage = 2.9f; // Crit
  lowCellMon.check();
  ASSERT_EQ(logger.entries.size(), 2u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_LOW);

  // -- Total Voltage --
  logger.entries.clear();
  fakeVoltage = 98.0f;
  SensorMonitor totalVoltsMon(SensorID::BMS_Total_Voltage, bmsTotalVoltageHighThresholds, [&]() { return fakeVoltage; }, &logger);
  totalVoltsMon.check(); // OK
  EXPECT_TRUE(logger.entries.empty());

  fakeVoltage = 100.5f; // Warn
  totalVoltsMon.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

  fakeVoltage = 101.0f; // Crit
  totalVoltsMon.check();
  ASSERT_EQ(logger.entries.size(), 2u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_HIGH);

  // -- Voltage Differential --
  logger.entries.clear();
  fakeVoltage = 0.1f;
  SensorMonitor diffMon(SensorID::BMS_Voltage_Differential, bmsVoltageDifferentialThresholds, [&]() { return fakeVoltage; }, &logger);
  diffMon.check(); // OK
  EXPECT_TRUE(logger.entries.empty());

  fakeVoltage = 0.3f; // Warn
  diffMon.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

  fakeVoltage = 0.5f; // Crit
  diffMon.check();
  ASSERT_EQ(logger.entries.size(), 2u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_HIGH);
}

TEST(SimpleMonitor, BMSTemperatureAlerts) {
    FakeLogger logger;
    float fakeTemp = 30.0f;

    // We can use BMS_MOS_Temp as a representative sensor for bmsTempThresholds
    SensorMonitor tempMon(SensorID::BMS_MOS_Temp, bmsTempThresholds, [&]() { return fakeTemp; }, &logger);

    tempMon.check(); // OK
    EXPECT_TRUE(logger.entries.empty());

    fakeTemp = 55.0f; // Warn High
    tempMon.check();
    ASSERT_EQ(logger.entries.size(), 1u);
    EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

    fakeTemp = 65.0f; // Crit High
    tempMon.check();
    ASSERT_EQ(logger.entries.size(), 2u);
    EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_HIGH);
}

TEST(SimpleMonitor, ESCTemperatureAlerts) {
    FakeLogger logger;
    float fakeTemp;

    // -- MOS Temp High --
    fakeTemp = 80.0f;
    SensorMonitor mosTempMon(SensorID::ESC_MOS_Temp, escMosTempThresholds, [&]() { return fakeTemp; }, &logger);
    mosTempMon.check();
    EXPECT_TRUE(logger.entries.empty());

    fakeTemp = 95.0f; // Warn High
    mosTempMon.check();
    ASSERT_EQ(logger.entries.size(), 1u);
    EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

    fakeTemp = 115.0f; // Crit High
    mosTempMon.check();
    ASSERT_EQ(logger.entries.size(), 2u);
    EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_HIGH);

    // -- MOS Temp Low --
    logger.entries.clear();
    fakeTemp = 0.0f;
    SensorMonitor mosTempLowMon(SensorID::ESC_MOS_Temp, escMosTempThresholds, [&]() { return fakeTemp; }, &logger);
    mosTempLowMon.check();
    EXPECT_TRUE(logger.entries.empty());

    fakeTemp = -15.0f; // Warn Low
    mosTempLowMon.check();
    ASSERT_EQ(logger.entries.size(), 1u);
    EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_LOW);

    fakeTemp = -25.0f; // Crit Low
    mosTempLowMon.check();
    ASSERT_EQ(logger.entries.size(), 2u);
    EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_LOW);

    // -- MCU Temp High --
    logger.entries.clear();
    fakeTemp = 75.0f;
    SensorMonitor mcuTempMon(SensorID::ESC_MCU_Temp, escMcuTempThresholds, [&]() { return fakeTemp; }, &logger);
    mcuTempMon.check();
    EXPECT_TRUE(logger.entries.empty());

    fakeTemp = 85.0f; // Warn High
    mcuTempMon.check();
    ASSERT_EQ(logger.entries.size(), 1u);
    EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

    fakeTemp = 100.0f; // Crit High
    mcuTempMon.check();
    ASSERT_EQ(logger.entries.size(), 2u);
    EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_HIGH);

    // -- MCU Temp Low --
    logger.entries.clear();
    fakeTemp = 0.0f;
    SensorMonitor mcuTempLowMon(SensorID::ESC_MCU_Temp, escMcuTempThresholds, [&]() { return fakeTemp; }, &logger);
    mcuTempLowMon.check();
    EXPECT_TRUE(logger.entries.empty());

    fakeTemp = -15.0f; // Warn Low
    mcuTempLowMon.check();
    ASSERT_EQ(logger.entries.size(), 1u);
    EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_LOW);

    fakeTemp = -25.0f; // Crit Low
    mcuTempLowMon.check();
    ASSERT_EQ(logger.entries.size(), 2u);
    EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_LOW);

    // -- Cap Temp High --
    logger.entries.clear();
    fakeTemp = 80.0f;
    SensorMonitor capTempMon(SensorID::ESC_CAP_Temp, escCapTempThresholds, [&]() { return fakeTemp; }, &logger);
    capTempMon.check();
    EXPECT_TRUE(logger.entries.empty());

    fakeTemp = 90.0f; // Warn High
    capTempMon.check();
    ASSERT_EQ(logger.entries.size(), 1u);
    EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

    fakeTemp = 105.0f; // Crit High
    capTempMon.check();
    ASSERT_EQ(logger.entries.size(), 2u);
    EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_HIGH);

    // -- Cap Temp Low --
    logger.entries.clear();
    fakeTemp = 0.0f;
    SensorMonitor capTempLowMon(SensorID::ESC_CAP_Temp, escCapTempThresholds, [&]() { return fakeTemp; }, &logger);
    capTempLowMon.check();
    EXPECT_TRUE(logger.entries.empty());

    fakeTemp = -15.0f; // Warn Low
    capTempLowMon.check();
    ASSERT_EQ(logger.entries.size(), 1u);
    EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_LOW);

    fakeTemp = -25.0f; // Crit Low
    capTempLowMon.check();
    ASSERT_EQ(logger.entries.size(), 2u);
    EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_LOW);

    // -- Motor Temp High --
    logger.entries.clear();
    fakeTemp = 85.0f;
    SensorMonitor motorTempMon(SensorID::Motor_Temp, motorTempThresholds, [&]() { return fakeTemp; }, &logger);
    motorTempMon.check();
    EXPECT_TRUE(logger.entries.empty());

    fakeTemp = 95.0f; // Warn High
    motorTempMon.check();
    ASSERT_EQ(logger.entries.size(), 1u);
    EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

    fakeTemp = 115.0f; // Crit High
    motorTempMon.check();
    ASSERT_EQ(logger.entries.size(), 2u);
    EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_HIGH);

    // -- Motor Temp Low --
    logger.entries.clear();
    fakeTemp = -10.0f;
    SensorMonitor motorTempLowMon(SensorID::Motor_Temp, motorTempThresholds, [&]() { return fakeTemp; }, &logger);
    motorTempLowMon.check();
    EXPECT_TRUE(logger.entries.empty());

    fakeTemp = -22.0f; // Warn Low
    motorTempLowMon.check();
    ASSERT_EQ(logger.entries.size(), 1u);
    EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_LOW);

    fakeTemp = -30.0f; // Crit Low
    motorTempLowMon.check();
    ASSERT_EQ(logger.entries.size(), 2u);
    EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_LOW);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    RUN_ALL_TESTS();
    // Always return zero-code and allow PlatformIO to parse results
    return 0;
}
