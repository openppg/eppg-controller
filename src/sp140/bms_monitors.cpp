#include "sp140/bms_monitors.h"
#include "sp140/monitor_config.h"
#include <Arduino.h>

// External references to core monitoring infrastructure
extern std::vector<IMonitor*> monitors;
extern MultiLogger multiLogger;

// External references to thread-safe data copies
extern STR_BMS_TELEMETRY_140 monitoringBmsData;

void addBMSMonitors() {
  // BMS MOSFET Temperature (Warning: 50°C, Critical: 60°C) - with hysteresis
  static HysteresisSensorMonitor* bmsMosTemp = new HysteresisSensorMonitor(
    SensorID::BMS_MOS_Temp,
    SensorCategory::BMS,
    bmsTempThresholds,
    []() { return monitoringBmsData.mos_temperature; },
    &multiLogger);
  monitors.push_back(bmsMosTemp);

  // BMS Balance Resistor Temperature (Warning: 50°C, Critical: 60°C) - with hysteresis
  static HysteresisSensorMonitor* bmsBalanceTemp = new HysteresisSensorMonitor(
    SensorID::BMS_Balance_Temp,
    SensorCategory::BMS,
    bmsTempThresholds,
    []() { return monitoringBmsData.balance_temperature; },
    &multiLogger);
  monitors.push_back(bmsBalanceTemp);

  // T1-T4 Cell Temperature Sensors (Warning: 50°C, Critical: 56°C) - with hysteresis
  static HysteresisSensorMonitor* bmsT1Temp = new HysteresisSensorMonitor(
    SensorID::BMS_T1_Temp,
    SensorCategory::BMS,
    bmsCellTempThresholds,
    []() { return monitoringBmsData.t1_temperature; },
    &multiLogger);
  monitors.push_back(bmsT1Temp);

  static HysteresisSensorMonitor* bmsT2Temp = new HysteresisSensorMonitor(
    SensorID::BMS_T2_Temp,
    SensorCategory::BMS,
    bmsCellTempThresholds,
    []() { return monitoringBmsData.t2_temperature; },
    &multiLogger);
  monitors.push_back(bmsT2Temp);

  static HysteresisSensorMonitor* bmsT3Temp = new HysteresisSensorMonitor(
    SensorID::BMS_T3_Temp,
    SensorCategory::BMS,
    bmsCellTempThresholds,
    []() { return monitoringBmsData.t3_temperature; },
    &multiLogger);
  monitors.push_back(bmsT3Temp);

  static HysteresisSensorMonitor* bmsT4Temp = new HysteresisSensorMonitor(
    SensorID::BMS_T4_Temp,
    SensorCategory::BMS,
    bmsCellTempThresholds,
    []() { return monitoringBmsData.t4_temperature; },
    &multiLogger);
  monitors.push_back(bmsT4Temp);

  // High Cell Voltage (Warn: 4.1V, Crit: 4.2V)
  static SensorMonitor* bmsHighCellVoltage = new SensorMonitor(
    SensorID::BMS_High_Cell_Voltage,
    SensorCategory::BMS,
    bmsHighCellVoltageThresholds,
    []() { return monitoringBmsData.highest_cell_voltage; },
    &multiLogger);
  monitors.push_back(bmsHighCellVoltage);

  // Low Cell Voltage (Warn: 3.2V, Crit: 3.0V)
  static SensorMonitor* bmsLowCellVoltage = new SensorMonitor(
    SensorID::BMS_Low_Cell_Voltage,
    SensorCategory::BMS,
    bmsLowCellVoltageThresholds,
    []() { return monitoringBmsData.lowest_cell_voltage; },
    &multiLogger);
  monitors.push_back(bmsLowCellVoltage);

  // State of Charge (Warn: 15%, Crit: 5%)
  static SensorMonitor* bmsSoc = new SensorMonitor(
    SensorID::BMS_SOC,
    SensorCategory::BMS,
    bmsSOCThresholds,
    []() { return monitoringBmsData.soc; },
    &multiLogger);
  monitors.push_back(bmsSoc);

  // Total Voltage (Low - Warn: 79.2V, Crit: 69.6V | High - Warn: 100.4V, Crit: 100.8V)
  static SensorMonitor* bmsTotalVoltage = new SensorMonitor(
    SensorID::BMS_Total_Voltage,
    SensorCategory::BMS,
    bmsTotalVoltageThresholds,
    []() { return monitoringBmsData.battery_voltage; },
    &multiLogger);
  monitors.push_back(bmsTotalVoltage);

  // Voltage Differential (Warn: 0.20V, Crit: 0.40V)
  static SensorMonitor* bmsVoltageDifferential = new SensorMonitor(
    SensorID::BMS_Voltage_Differential,
    SensorCategory::BMS,
    bmsVoltageDifferentialThresholds,
    []() { return monitoringBmsData.voltage_differential; },
    &multiLogger);
  monitors.push_back(bmsVoltageDifferential);

  // Charge MOS (Alert when OFF) - DISABLED
  // static BooleanMonitor* bmsChargeMos = new BooleanMonitor(
  //   SensorID::BMS_Charge_MOS,
  //   SensorCategory::BMS,
  //   []() { return monitoringBmsData.is_charge_mos; },
  //   false,  // Alert when false
  //   AlertLevel::CRIT_HIGH,
  //   &multiLogger);
  // monitors.push_back(bmsChargeMos);

  // Discharge MOS (Alert when OFF)
  static BooleanMonitor* bmsDischargeMos = new BooleanMonitor(
    SensorID::BMS_Discharge_MOS,
    SensorCategory::BMS,
    []() { return monitoringBmsData.is_discharge_mos; },
    false,  // Alert when false
    AlertLevel::CRIT_HIGH,
    &multiLogger);
  monitors.push_back(bmsDischargeMos);
}
