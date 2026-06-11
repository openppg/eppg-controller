#include "sp140/system_monitors.h"
#include "sp140/monitor_config.h"
#include "sp140/altimeter.h"
#include <Arduino.h>

// External references to core monitoring infrastructure
extern std::vector<IMonitor*> monitors;
extern MultiLogger multiLogger;

// External reference to BMP sensor status
extern bool bmpPresent;

// CPU temperature cache. SINGLE-WRITER: only refreshCpuTemperature() (and the
// one-time prime below) may call temperatureRead() — concurrent tsens access
// from two tasks triggers "tsens: Do not configure the temp sensor when it's
// running!". Refresh is owned by ctrlSensorTask (10 Hz); every other consumer
// reads the cache via getCachedCpuTemperature(). The cache is primed once in
// setup (single-threaded, before tasks exist) so readers never observe the
// 0.0f initial value — warnLow in cpuTempThresholds is 0.
static volatile float cachedCpuTemp = 0.0f;
static unsigned long lastCpuTempRead = 0;
static const unsigned long CPU_TEMP_READ_INTERVAL = 1000;  // Read every 1 second

void primeCpuTemperatureCache() {
  cachedCpuTemp = temperatureRead();
  lastCpuTempRead = millis();
}

float refreshCpuTemperature() {
  unsigned long now = millis();
  if (now - lastCpuTempRead >= CPU_TEMP_READ_INTERVAL) {
    cachedCpuTemp = temperatureRead();
    lastCpuTempRead = now;
  }
  return cachedCpuTemp;
}

float getCachedCpuTemperature() {
  return cachedCpuTemp;  // pure read; aligned 32-bit float load is atomic on Xtensa
}

void addInternalMonitors() {
  // ESP32-S3 CPU Temperature (Warning: 50°C, Critical: 80°C) - with hysteresis
  static HysteresisSensorMonitor* cpuTemp = new HysteresisSensorMonitor(
    SensorID::CPU_Temp,
    SensorCategory::INTERNAL,
    cpuTempThresholds,
    []() { return getCachedCpuTemperature(); },
    &multiLogger);
  monitors.push_back(cpuTemp);
}

void addAltimeterMonitors() {
  // BMP3xx Initialization (Alert when failed)
  static BooleanMonitor* bmpInitFailure = new BooleanMonitor(
    SensorID::Baro_Init_Failure,
    SensorCategory::ALTIMETER,
    []() { return bmpPresent; },
    false,  // Alert when false (i.e., when initialization failed)
    AlertLevel::WARN_HIGH,
    &multiLogger);
  monitors.push_back(bmpInitFailure);

  // Barometer Temperature — reads cached float (no I2C), safe from any task.
  // Cache is populated by uiTask's getAltitude/getBaroTemperature calls.
  static SensorMonitor* baroTemp = new SensorMonitor(
    SensorID::Baro_Temp,
    SensorCategory::ALTIMETER,
    baroTempThresholds,
    []() { return getCachedBaroTemperature(); },
    &multiLogger);
  monitors.push_back(baroTemp);
}
