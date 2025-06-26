#include "sp140/simple_monitor.h"
#include "sp140/globals.h"

// Global instances
std::vector<SensorMonitor*> sensors;
SerialLogger serialLogger;

void initSimpleMonitor() {
  USBSerial.println("Initializing Simple Monitor System");
  sensors.clear();
  addESCMonitors();
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
    "ESC_MOS_Temp",
    {.warnLow = -10, .warnHigh = 90, .critLow = -20, .critHigh = 110},
    []() { return escTelemetryData.mos_temp; },
    &serialLogger
  );
  sensors.push_back(escMosTemp);

  // ESC MCU Temperature Monitor
  // Min: -10°C, Warning: 80°C, Critical: 95°C
  static SensorMonitor* escMcuTemp = new SensorMonitor(
    "ESC_MCU_Temp",
    {.warnLow = -10, .warnHigh = 80, .critLow = -20, .critHigh = 95},
    []() { return escTelemetryData.mcu_temp; },
    &serialLogger
  );
  sensors.push_back(escMcuTemp);

  // ESC Capacitor Temperature Monitor
  // Min: -10°C, Warning: 85°C, Critical: 100°C
  static SensorMonitor* escCapTemp = new SensorMonitor(
    "ESC_CAP_Temp",
    {.warnLow = -10, .warnHigh = 85, .critLow = -20, .critHigh = 100},
    []() { return escTelemetryData.cap_temp; },
    &serialLogger
  );
  sensors.push_back(escCapTemp);

  // Motor Temperature Monitor
  // Using same thresholds as MOS for now
  static SensorMonitor* motorTemp = new SensorMonitor(
    "Motor_Temp",
    {.warnLow = -10, .warnHigh = 90, .critLow = -20, .critHigh = 110},
    []() { return escTelemetryData.motor_temp; },
    &serialLogger
  );
  sensors.push_back(motorTemp);
}
