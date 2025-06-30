#include "sp140/simple_monitor.h"
#include "sp140/globals.h"
#include "sp140/altimeter.h"
#include "sp140/utilities.h"

// Global instances
std::vector<SensorMonitor*> sensors;
SerialLogger serialLogger;

void SerialLogger::log(SensorID id, AlertLevel lvl, float v) {
  const char* levelNames[] = {"OK", "WARN_LOW", "WARN_HIGH", "CRIT_LOW", "CRIT_HIGH"};
  USBSerial.printf("[%lu] [%s] %s = %.2f\n", millis(), levelNames[(int)lvl], sensorIDToString(id), v);
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

    // Altimeter
    case SensorID::Baro_Temp: return "Baro_Temp";

    // Internal
    case SensorID::CPU_Temp: return "CPU_Temp";

    default: return "Unknown_Sensor";
  }
}

void initSimpleMonitor() {
  USBSerial.println("Initializing Simple Monitor System");
  sensors.clear();
  addESCMonitors();
  addBMSMonitors();
  addAltimeterMonitors();
  addInternalMonitors();
  USBSerial.printf("Monitoring %d sensors\n", sensors.size());
}

void checkAllSensors() {
  for (auto* sensor : sensors) {
    if (sensor) {
      sensor->check();
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
  sensors.push_back(escMosTemp);

  // ESC MCU Temperature Monitor
  // Min: -10°C, Warning: 80°C, Critical: 95°C
  static SensorMonitor* escMcuTemp = new SensorMonitor(
    SensorID::ESC_MCU_Temp,
    {.warnLow = -10, .warnHigh = 80, .critLow = -20, .critHigh = 95},
    []() { return escTelemetryData.mcu_temp; },
    &serialLogger);
  sensors.push_back(escMcuTemp);

  // ESC Capacitor Temperature Monitor
  // Min: -10°C, Warning: 85°C, Critical: 100°C
  static SensorMonitor* escCapTemp = new SensorMonitor(
    SensorID::ESC_CAP_Temp,
    {.warnLow = -10, .warnHigh = 85, .critLow = -20, .critHigh = 100},
    []() { return escTelemetryData.cap_temp; },
    &serialLogger);
  sensors.push_back(escCapTemp);

  // Motor Temperature Monitor
  // Min: -20°C, Warning: 90°C, Critical: 110°C
  static SensorMonitor* motorTemp = new SensorMonitor(
    SensorID::Motor_Temp,
    {.warnLow = -20, .warnHigh = 90, .critLow = -25, .critHigh = 110},
    []() { return escTelemetryData.motor_temp; },
    &serialLogger);
  sensors.push_back(motorTemp);
}

void addBMSMonitors() {
  // BMS MOSFET Temperature (Warning: 50°C, Critical: 60°C)
  static SensorMonitor* bmsMosTemp = new SensorMonitor(
    SensorID::BMS_MOS_Temp,
    {.warnLow = -10, .warnHigh = 50, .critLow = -15, .critHigh = 60},
    []() { return bmsTelemetryData.mos_temperature; },
    &serialLogger);
  sensors.push_back(bmsMosTemp);

  // BMS Balance Resistor Temperature (Warning: 50°C, Critical: 60°C)
  static SensorMonitor* bmsBalanceTemp = new SensorMonitor(
    SensorID::BMS_Balance_Temp,
    {.warnLow = -10, .warnHigh = 50, .critLow = -15, .critHigh = 60},
    []() { return bmsTelemetryData.balance_temperature; },
    &serialLogger);
  sensors.push_back(bmsBalanceTemp);

  // T1-T4 Cell Temperature Sensors (Warning: 50°C, Critical: 56°C)
  static const Thresholds bmsCellTempThresholds = {
    .warnLow = -10, .warnHigh = 50, .critLow = -15, .critHigh = 56
  };

  static SensorMonitor* bmsT1Temp = new SensorMonitor(
    SensorID::BMS_T1_Temp,
    bmsCellTempThresholds,
    []() { return bmsTelemetryData.t1_temperature; },
    &serialLogger);
  sensors.push_back(bmsT1Temp);

  static SensorMonitor* bmsT2Temp = new SensorMonitor(
    SensorID::BMS_T2_Temp,
    bmsCellTempThresholds,
    []() { return bmsTelemetryData.t2_temperature; },
    &serialLogger);
  sensors.push_back(bmsT2Temp);

  static SensorMonitor* bmsT3Temp = new SensorMonitor(
    SensorID::BMS_T3_Temp,
    bmsCellTempThresholds,
    []() { return bmsTelemetryData.t3_temperature; },
    &serialLogger);
  sensors.push_back(bmsT3Temp);

  static SensorMonitor* bmsT4Temp = new SensorMonitor(
    SensorID::BMS_T4_Temp,
    bmsCellTempThresholds,
    []() { return bmsTelemetryData.t4_temperature; },
    &serialLogger);
  sensors.push_back(bmsT4Temp);
}

void addAltimeterMonitors() {
  // Barometer Temperature (Warning: 50°C, Critical: 80°C)
  static SensorMonitor* baroTemp = new SensorMonitor(
    SensorID::Baro_Temp,
    {.warnLow = 0, .warnHigh = 50, .critLow = -10, .critHigh = 80},
    []() { return getBaroTemperature(); },
    &serialLogger);
  sensors.push_back(baroTemp);
}

void addInternalMonitors() {
  // ESP32-S3 CPU Temperature (Warning: 50°C, Critical: 80°C)
  static SensorMonitor* cpuTemp = new SensorMonitor(
    SensorID::CPU_Temp,
    {.warnLow = 0, .warnHigh = 50, .critLow = -10, .critHigh = 80},
    []() { return temperatureRead(); },
    &serialLogger);
  sensors.push_back(cpuTemp);
}
