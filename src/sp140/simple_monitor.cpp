#include "sp140/simple_monitor.h"
#include "sp140/globals.h"
#include "sp140/altimeter.h"
#include "sp140/utilities.h"

// Global instances
std::vector<IMonitor*> monitors;
SerialLogger serialLogger;

void SerialLogger::log(SensorID id, AlertLevel lvl, float v) {
  const char* levelNames[] = {"OK", "WARN_LOW", "WARN_HIGH", "CRIT_LOW", "CRIT_HIGH", "INFO"};
  USBSerial.printf("[%lu] [%s] %s = %.2f\n", millis(), levelNames[(int)lvl], sensorIDToString(id), v);
}

void SerialLogger::log(SensorID id, AlertLevel lvl, bool v) {
  const char* levelNames[] = {"OK", "WARN_LOW", "WARN_HIGH", "CRIT_LOW", "CRIT_HIGH", "INFO"};
  USBSerial.printf("[%lu] [%s] %s = %s\n", millis(), levelNames[(int)lvl], sensorIDToString(id), v ? "ON" : "OFF");
}

const char* sensorIDToString(SensorID id) {
  switch (id) {
    // ESC
    case SensorID::ESC_MOS_Temp: return "ESC_MOS_Temp";
    case SensorID::ESC_MCU_Temp: return "ESC_MCU_Temp";
    case SensorID::ESC_CAP_Temp: return "ESC_CAP_Temp";
    case SensorID::Motor_Temp: return "Motor_Temp";

    // BMS
    case SensorID::BMS_MOS_Temp: return "BMS_MOS_Temp";
    case SensorID::BMS_Balance_Temp: return "BMS_Balance_Temp";
    case SensorID::BMS_T1_Temp: return "BMS_T1_Temp";
    case SensorID::BMS_T2_Temp: return "BMS_T2_Temp";
    case SensorID::BMS_T3_Temp: return "BMS_T3_Temp";
    case SensorID::BMS_T4_Temp: return "BMS_T4_Temp";
    case SensorID::BMS_High_Cell_Voltage: return "BMS_High_Cell_Voltage";
    case SensorID::BMS_Low_Cell_Voltage: return "BMS_Low_Cell_Voltage";
    case SensorID::BMS_SOC: return "BMS_SOC";
    case SensorID::BMS_Total_Voltage: return "BMS_Total_Voltage";
    case SensorID::BMS_Voltage_Differential: return "BMS_Voltage_Differential";
    case SensorID::BMS_Charge_MOS: return "BMS_Charge_MOS";
    case SensorID::BMS_Discharge_MOS: return "BMS_Discharge_MOS";

    // Altimeter
    case SensorID::Baro_Temp: return "Baro_Temp";

    // Internal
    case SensorID::CPU_Temp: return "CPU_Temp";

    default: return "Unknown_Sensor";
  }
}

void initSimpleMonitor() {
  USBSerial.println("Initializing Simple Monitor System");
  monitors.clear();
  addESCMonitors();
  addBMSMonitors();
  addAltimeterMonitors();
  addInternalMonitors();
  USBSerial.printf("Monitoring %d sensors\n", monitors.size());
}

void checkAllSensors() {
  for (auto* monitor : monitors) {
    if (monitor) {
      monitor->check();
    }
  }
}

void addESCMonitors() {
  // ESC MOS Temperature Monitor
  // Min: -10°C, Warning: 90°C, Critical: 110°C
  static SensorMonitor* escMosTemp = new SensorMonitor(
    SensorID::ESC_MOS_Temp,
    {.warnLow = -10, .warnHigh = 90, .critLow = -20, .critHigh = 110},
    []() { return escTelemetryData.mos_temp; },
    &serialLogger);
  monitors.push_back(escMosTemp);

  // ESC MCU Temperature Monitor
  // Min: -10°C, Warning: 80°C, Critical: 95°C
  static SensorMonitor* escMcuTemp = new SensorMonitor(
    SensorID::ESC_MCU_Temp,
    {.warnLow = -10, .warnHigh = 80, .critLow = -20, .critHigh = 95},
    []() { return escTelemetryData.mcu_temp; },
    &serialLogger);
  monitors.push_back(escMcuTemp);

  // ESC Capacitor Temperature Monitor
  // Min: -10°C, Warning: 85°C, Critical: 100°C
  static SensorMonitor* escCapTemp = new SensorMonitor(
    SensorID::ESC_CAP_Temp,
    {.warnLow = -10, .warnHigh = 85, .critLow = -20, .critHigh = 100},
    []() { return escTelemetryData.cap_temp; },
    &serialLogger);
  monitors.push_back(escCapTemp);

  // Motor Temperature Monitor
  // Min: -20°C, Warning: 90°C, Critical: 110°C
  static SensorMonitor* motorTemp = new SensorMonitor(
    SensorID::Motor_Temp,
    {.warnLow = -20, .warnHigh = 90, .critLow = -25, .critHigh = 110},
    []() { return escTelemetryData.motor_temp; },
    &serialLogger);
  monitors.push_back(motorTemp);
}

void addBMSMonitors() {
  // BMS MOSFET Temperature (Warning: 50°C, Critical: 60°C)
  static SensorMonitor* bmsMosTemp = new SensorMonitor(
    SensorID::BMS_MOS_Temp,
    {.warnLow = -10, .warnHigh = 50, .critLow = -15, .critHigh = 60},
    []() { return bmsTelemetryData.mos_temperature; },
    &serialLogger);
  monitors.push_back(bmsMosTemp);

  // BMS Balance Resistor Temperature (Warning: 50°C, Critical: 60°C)
  static SensorMonitor* bmsBalanceTemp = new SensorMonitor(
    SensorID::BMS_Balance_Temp,
    {.warnLow = -10, .warnHigh = 50, .critLow = -15, .critHigh = 60},
    []() { return bmsTelemetryData.balance_temperature; },
    &serialLogger);
  monitors.push_back(bmsBalanceTemp);

  // T1-T4 Cell Temperature Sensors (Warning: 50°C, Critical: 56°C)
  static const Thresholds bmsCellTempThresholds = {
    .warnLow = -10, .warnHigh = 50, .critLow = -15, .critHigh = 56
  };

  static SensorMonitor* bmsT1Temp = new SensorMonitor(
    SensorID::BMS_T1_Temp,
    bmsCellTempThresholds,
    []() { return bmsTelemetryData.t1_temperature; },
    &serialLogger);
  monitors.push_back(bmsT1Temp);

  static SensorMonitor* bmsT2Temp = new SensorMonitor(
    SensorID::BMS_T2_Temp,
    bmsCellTempThresholds,
    []() { return bmsTelemetryData.t2_temperature; },
    &serialLogger);
  monitors.push_back(bmsT2Temp);

  static SensorMonitor* bmsT3Temp = new SensorMonitor(
    SensorID::BMS_T3_Temp,
    bmsCellTempThresholds,
    []() { return bmsTelemetryData.t3_temperature; },
    &serialLogger);
  monitors.push_back(bmsT3Temp);

  static SensorMonitor* bmsT4Temp = new SensorMonitor(
    SensorID::BMS_T4_Temp,
    bmsCellTempThresholds,
    []() { return bmsTelemetryData.t4_temperature; },
    &serialLogger);
  monitors.push_back(bmsT4Temp);

  // High Cell Voltage (Warn: 4.1V, Crit: 4.2V)
  static SensorMonitor* bmsHighCellVoltage = new SensorMonitor(
    SensorID::BMS_High_Cell_Voltage,
    {.warnLow = 0.0, .warnHigh = 4.1, .critLow = 0.0, .critHigh = 4.2},
    []() { return bmsTelemetryData.highest_cell_voltage; },
    &serialLogger);
  monitors.push_back(bmsHighCellVoltage);

  // Low Cell Voltage (Warn: 3.2V, Crit: 3.0V)
  static SensorMonitor* bmsLowCellVoltage = new SensorMonitor(
    SensorID::BMS_Low_Cell_Voltage,
    {.warnLow = 3.2, .warnHigh = 5.0, .critLow = 3.0, .critHigh = 5.0},
    []() { return bmsTelemetryData.lowest_cell_voltage; },
    &serialLogger);
  monitors.push_back(bmsLowCellVoltage);

  // State of Charge (Warn: 15%, Crit: 5%)
  static SensorMonitor* bmsSoc = new SensorMonitor(
    SensorID::BMS_SOC,
    {.warnLow = 15.0, .warnHigh = 101.0, .critLow = 5.0, .critHigh = 110.0},
    []() { return bmsTelemetryData.soc; },
    &serialLogger);
  monitors.push_back(bmsSoc);

  // Total Voltage High (Warn: 100.4V, Crit: 100.8V)
  static SensorMonitor* bmsTotalVoltage = new SensorMonitor(
    SensorID::BMS_Total_Voltage,
    {.warnLow = 0.0, .warnHigh = 100.4, .critLow = 0.0, .critHigh = 100.8},
    []() { return bmsTelemetryData.battery_voltage; },
    &serialLogger);
  monitors.push_back(bmsTotalVoltage);

  // Voltage Differential (Warn: 0.20V, Crit: 0.40V)
  // 0.0V is ideal (perfectly balanced), only alert on HIGH values
  static SensorMonitor* bmsVoltageDifferential = new SensorMonitor(
    SensorID::BMS_Voltage_Differential,
    {.warnLow = -1.0, .warnHigh = 0.2, .critLow = -2.0, .critHigh = 0.4},
    []() { return bmsTelemetryData.voltage_differential; },
    &serialLogger);
  monitors.push_back(bmsVoltageDifferential);

  // Charge MOS (Alert when OFF)
  static BooleanMonitor* bmsChargeMos = new BooleanMonitor(
    SensorID::BMS_Charge_MOS,
    []() { return bmsTelemetryData.is_charge_mos; },
    false,  // Alert when false
    AlertLevel::CRIT_HIGH,
    &serialLogger);
  monitors.push_back(bmsChargeMos);

  // Discharge MOS (Alert when OFF)
  static BooleanMonitor* bmsDischargeMos = new BooleanMonitor(
    SensorID::BMS_Discharge_MOS,
    []() { return bmsTelemetryData.is_discharge_mos; },
    false,  // Alert when false
    AlertLevel::CRIT_HIGH,
    &serialLogger);
  monitors.push_back(bmsDischargeMos);
}

void addAltimeterMonitors() {
  // Barometer Temperature (Warning: 50°C, Critical: 80°C)
  static SensorMonitor* baroTemp = new SensorMonitor(
    SensorID::Baro_Temp,
    {.warnLow = 0, .warnHigh = 50, .critLow = -10, .critHigh = 80},
    []() { return getBaroTemperature(); },
    &serialLogger);
  monitors.push_back(baroTemp);
}

void addInternalMonitors() {
  // ESP32-S3 CPU Temperature (Warning: 50°C, Critical: 80°C)
  static SensorMonitor* cpuTemp = new SensorMonitor(
    SensorID::CPU_Temp,
    {.warnLow = 0, .warnHigh = 50, .critLow = -10, .critHigh = 80},
    []() { return temperatureRead(); },
    &serialLogger);
  monitors.push_back(cpuTemp);
}
