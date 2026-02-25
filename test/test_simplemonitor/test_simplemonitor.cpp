#include <gtest/gtest.h>
#include "sp140/bms.h"
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
  SensorMonitor mon(SensorID::CPU_Temp, SensorCategory::INTERNAL, thr, [&]() { return sensorVal; }, &logger);

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
  BooleanMonitor mon(SensorID::BMS_Charge_MOS, SensorCategory::BMS, [&]() { return state; }, false, AlertLevel::CRIT_HIGH, &logger);

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
  SensorMonitor socMonitor(SensorID::BMS_SOC, SensorCategory::BMS, bmsSOCThresholds, [&]() { return fakeSOC; }, &logger);

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
  SensorMonitor highCellMon(SensorID::BMS_High_Cell_Voltage, SensorCategory::BMS, bmsHighCellVoltageThresholds, [&]() { return fakeVoltage; }, &logger);
  highCellMon.check(); // OK
  EXPECT_TRUE(logger.entries.empty());

  fakeVoltage = 4.19f; // Warn
  highCellMon.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

  fakeVoltage = 4.21f; // Crit
  highCellMon.check();
  ASSERT_EQ(logger.entries.size(), 2u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_HIGH);

  // -- Low Cell Voltage --
  logger.entries.clear();
  fakeVoltage = 3.5f;
  SensorMonitor lowCellMon(SensorID::BMS_Low_Cell_Voltage, SensorCategory::BMS, bmsLowCellVoltageThresholds, [&]() { return fakeVoltage; }, &logger);
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
  fakeVoltage = 85.0f;
  SensorMonitor totalVoltsMon(SensorID::BMS_Total_Voltage, SensorCategory::BMS, bmsTotalVoltageThresholds, [&]() { return fakeVoltage; }, &logger);
  totalVoltsMon.check(); // OK
  EXPECT_TRUE(logger.entries.empty());

  // Test Low Voltage Alerts
  fakeVoltage = 78.0f; // Warn Low
  totalVoltsMon.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_LOW);

  fakeVoltage = 68.0f; // Crit Low
  totalVoltsMon.check();
  ASSERT_EQ(logger.entries.size(), 2u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_LOW);

  // Return to OK
  fakeVoltage = 85.0f;
  totalVoltsMon.check();
  ASSERT_EQ(logger.entries.size(), 3u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::OK);

  // Test High Voltage Alerts
  fakeVoltage = 100.5f; // Warn High
  totalVoltsMon.check();
  ASSERT_EQ(logger.entries.size(), 4u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

  fakeVoltage = 101.0f; // Crit High
  totalVoltsMon.check();
  ASSERT_EQ(logger.entries.size(), 5u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::CRIT_HIGH);

  // Return to OK
  fakeVoltage = 85.0f;
  totalVoltsMon.check();
  ASSERT_EQ(logger.entries.size(), 6u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::OK);

  // -- Voltage Differential --
  logger.entries.clear();
  fakeVoltage = 0.1f;
  SensorMonitor diffMon(SensorID::BMS_Voltage_Differential, SensorCategory::BMS, bmsVoltageDifferentialThresholds, [&]() { return fakeVoltage; }, &logger);
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
    SensorMonitor tempMon(SensorID::BMS_MOS_Temp, SensorCategory::BMS, bmsTempThresholds, [&]() { return fakeTemp; }, &logger);

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

TEST(SimpleMonitor, BMSCellTemperatureDisconnectedIsIgnored) {
  FakeLogger logger;
  float fakeTemp = 30.0f;

  SensorMonitor cellTempMon(
    SensorID::BMS_T1_Temp,
    SensorCategory::BMS,
    bmsCellTempThresholds,
    [&]() { return fakeTemp; },
    &logger);

  cellTempMon.check();  // Baseline OK
  EXPECT_TRUE(logger.entries.empty());

  // Disconnected probe reading should be treated as invalid (sanitized to NaN upstream).
  fakeTemp = NAN;
  cellTempMon.check();
  EXPECT_TRUE(logger.entries.empty());

  // Remaining connected probes still report alerts normally.
  fakeTemp = 54.0f;
  cellTempMon.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);
}

TEST(SimpleMonitor, BMSCellTemperatureDisconnectClearsActiveAlert) {
  FakeLogger logger;
  float fakeTemp = 55.0f;

  SensorMonitor cellTempMon(
    SensorID::BMS_T2_Temp,
    SensorCategory::BMS,
    bmsCellTempThresholds,
    [&]() { return fakeTemp; },
    &logger);

  // Enter warning state first.
  cellTempMon.check();
  ASSERT_EQ(logger.entries.size(), 1u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::WARN_HIGH);

  // Disconnect should clear the active alert.
  fakeTemp = NAN;
  cellTempMon.check();
  ASSERT_EQ(logger.entries.size(), 2u);
  EXPECT_EQ(logger.entries.back().lvl, AlertLevel::OK);
}

TEST(SimpleMonitor, BMSCellTempSanitizerDisconnectThreshold) {
  EXPECT_TRUE(isnan(sanitizeBmsCellTempC(-40.0f)));
  EXPECT_TRUE(isnan(sanitizeBmsCellTempC(-41.0f)));
  EXPECT_FLOAT_EQ(sanitizeBmsCellTempC(-39.5f), -39.5f);
}

TEST(SimpleMonitor, BMSCellTempSanitizerCanKeepDisconnectedValueWhenRequired) {
  EXPECT_FLOAT_EQ(sanitizeBmsCellTempC(-40.0f, false), -40.0f);
  EXPECT_TRUE(isnan(sanitizeBmsCellTempC(-41.0f, false)));
}

TEST(SimpleMonitor, BMSCellTempSanitizerPreservesValidValues) {
  EXPECT_FLOAT_EQ(sanitizeBmsCellTempC(0.0f), 0.0f);
  EXPECT_FLOAT_EQ(sanitizeBmsCellTempC(25.0f), 25.0f);
  EXPECT_TRUE(isnan(sanitizeBmsCellTempC(NAN)));
}

TEST(SimpleMonitor, MonitorIDAccess) {
    FakeLogger logger;

    // Test that monitors can provide their sensor IDs
    float normalTemp = 50.0f;
    SensorMonitor escMonitor(SensorID::ESC_MOS_Temp, SensorCategory::ESC, escMosTempThresholds,
                            [&]() { return normalTemp; }, &logger);
    SensorMonitor bmsMonitor(SensorID::BMS_SOC, SensorCategory::BMS, bmsSOCThresholds,
                            [&]() { return normalTemp; }, &logger);
    SensorMonitor cpuMonitor(SensorID::CPU_Temp, SensorCategory::INTERNAL, cpuTempThresholds,
                            [&]() { return normalTemp; }, &logger);

    // Test that getSensorID works correctly
    EXPECT_EQ(escMonitor.getSensorID(), SensorID::ESC_MOS_Temp);
    EXPECT_EQ(bmsMonitor.getSensorID(), SensorID::BMS_SOC);
    EXPECT_EQ(cpuMonitor.getSensorID(), SensorID::CPU_Temp);

    // Test that monitors work as expected
    escMonitor.check();
    bmsMonitor.check();
    cpuMonitor.check();

    // All should be OK (no alerts) with normal values
    EXPECT_TRUE(logger.entries.empty());
}

TEST(SimpleMonitor, ESCTemperatureAlerts) {
    FakeLogger logger;
    float fakeTemp;

    // -- MOS Temp High --
    fakeTemp = 80.0f;
    SensorMonitor mosTempMon(SensorID::ESC_MOS_Temp, SensorCategory::ESC, escMosTempThresholds, [&]() { return fakeTemp; }, &logger);
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
    SensorMonitor mosTempLowMon(SensorID::ESC_MOS_Temp, SensorCategory::ESC, escMosTempThresholds, [&]() { return fakeTemp; }, &logger);
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
    SensorMonitor mcuTempMon(SensorID::ESC_MCU_Temp, SensorCategory::ESC, escMcuTempThresholds, [&]() { return fakeTemp; }, &logger);
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
    SensorMonitor mcuTempLowMon(SensorID::ESC_MCU_Temp, SensorCategory::ESC, escMcuTempThresholds, [&]() { return fakeTemp; }, &logger);
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
    SensorMonitor capTempMon(SensorID::ESC_CAP_Temp, SensorCategory::ESC, escCapTempThresholds, [&]() { return fakeTemp; }, &logger);
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
    SensorMonitor capTempLowMon(SensorID::ESC_CAP_Temp, SensorCategory::ESC, escCapTempThresholds, [&]() { return fakeTemp; }, &logger);
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
    SensorMonitor motorTempMon(SensorID::Motor_Temp, SensorCategory::ESC, motorTempThresholds, [&]() { return fakeTemp; }, &logger);
    motorTempMon.check();
    EXPECT_TRUE(logger.entries.empty());

    fakeTemp = 110.0f; // Warn High
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
    SensorMonitor motorTempLowMon(SensorID::Motor_Temp, SensorCategory::ESC, motorTempThresholds, [&]() { return fakeTemp; }, &logger);
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

TEST(SimpleMonitor, SensorMonitorCategoryProperty) {
  FakeLogger logger;

  // Test ESC category
  SensorMonitor escMonitor(SensorID::ESC_MOS_Temp, SensorCategory::ESC,
                          Thresholds{-10.0f, 10.0f, 90.0f, 100.0f},
                          []() { return 25.0f; }, &logger);
  EXPECT_EQ(escMonitor.getCategory(), SensorCategory::ESC);
  EXPECT_EQ(escMonitor.getSensorID(), SensorID::ESC_MOS_Temp);

  // Test BMS category
  SensorMonitor bmsMonitor(SensorID::BMS_SOC, SensorCategory::BMS,
                          Thresholds{5.0f, 15.0f, 85.0f, 95.0f},
                          []() { return 50.0f; }, &logger);
  EXPECT_EQ(bmsMonitor.getCategory(), SensorCategory::BMS);
  EXPECT_EQ(bmsMonitor.getSensorID(), SensorID::BMS_SOC);

  // Test INTERNAL category
  SensorMonitor internalMonitor(SensorID::CPU_Temp, SensorCategory::INTERNAL,
                               Thresholds{-10.0f, 10.0f, 70.0f, 85.0f},
                               []() { return 45.0f; }, &logger);
  EXPECT_EQ(internalMonitor.getCategory(), SensorCategory::INTERNAL);
  EXPECT_EQ(internalMonitor.getSensorID(), SensorID::CPU_Temp);

  // Test ALTIMETER category
  SensorMonitor altimeterMonitor(SensorID::Baro_Temp, SensorCategory::ALTIMETER,
                                Thresholds{-20.0f, 0.0f, 60.0f, 80.0f},
                                []() { return 20.0f; }, &logger);
  EXPECT_EQ(altimeterMonitor.getCategory(), SensorCategory::ALTIMETER);
  EXPECT_EQ(altimeterMonitor.getSensorID(), SensorID::Baro_Temp);
}

TEST(SimpleMonitor, BooleanMonitorCategoryProperty) {
  FakeLogger logger;

  // Test ESC category Boolean monitor
  BooleanMonitor escErrorMonitor(SensorID::ESC_OverCurrent_Error, SensorCategory::ESC,
                                []() { return false; }, true, AlertLevel::CRIT_HIGH, &logger);
  EXPECT_EQ(escErrorMonitor.getCategory(), SensorCategory::ESC);
  EXPECT_EQ(escErrorMonitor.getSensorID(), SensorID::ESC_OverCurrent_Error);

  // Test BMS category Boolean monitor
  BooleanMonitor bmsMosMonitor(SensorID::BMS_Charge_MOS, SensorCategory::BMS,
                              []() { return true; }, false, AlertLevel::CRIT_HIGH, &logger);
  EXPECT_EQ(bmsMosMonitor.getCategory(), SensorCategory::BMS);
  EXPECT_EQ(bmsMosMonitor.getSensorID(), SensorID::BMS_Charge_MOS);
}

TEST(SimpleMonitor, CategoryBasedFiltering) {
  FakeLogger logger;

  // Create monitors from different categories
  SensorMonitor escMonitor(SensorID::ESC_MOS_Temp, SensorCategory::ESC,
                          Thresholds{-10.0f, 10.0f, 90.0f, 100.0f},
                          []() { return 25.0f; }, &logger);

  SensorMonitor bmsMonitor(SensorID::BMS_SOC, SensorCategory::BMS,
                          Thresholds{5.0f, 15.0f, 85.0f, 95.0f},
                          []() { return 50.0f; }, &logger);

  SensorMonitor internalMonitor(SensorID::CPU_Temp, SensorCategory::INTERNAL,
                               Thresholds{-10.0f, 10.0f, 70.0f, 85.0f},
                               []() { return 45.0f; }, &logger);

  // Test that we can filter monitors by category
  std::vector<IMonitor*> testMonitors = {&escMonitor, &bmsMonitor, &internalMonitor};

  int escCount = 0, bmsCount = 0, internalCount = 0;

  for (auto* monitor : testMonitors) {
    switch (monitor->getCategory()) {
      case SensorCategory::ESC:
        escCount++;
        break;
      case SensorCategory::BMS:
        bmsCount++;
        break;
      case SensorCategory::INTERNAL:
        internalCount++;
        break;
      case SensorCategory::ALTIMETER:
        break;
    }
  }

  EXPECT_EQ(escCount, 1);
  EXPECT_EQ(bmsCount, 1);
  EXPECT_EQ(internalCount, 1);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    RUN_ALL_TESTS();
    // Always return zero-code and allow PlatformIO to parse results
    return 0;
}
