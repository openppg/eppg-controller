#include "sp140/system_monitors.h"
#include "sp140/monitor_config.h"
#include "sp140/altimeter.h"
#include "sp140/time_utils.h"
#include <Arduino.h>
#include <atomic>

// External references to core monitoring infrastructure
extern std::vector<IMonitor*> monitors;
extern MultiLogger multiLogger;

// External reference to BMP sensor status
extern bool bmpPresent;

// ESP32 tsens configuration is not safe from concurrent tasks. One task owns
// temperatureRead(); readers consume this atomic cross-core publication.
static std::atomic<float> cachedCpuTemp{0.0f};
static unsigned long lastCpuTempRead = 0;
static const unsigned long CPU_TEMP_READ_INTERVAL = 1000;  // Read every 1 second

void primeCpuTemperatureCache() {
  cachedCpuTemp.store(temperatureRead(), std::memory_order_relaxed);
  lastCpuTempRead = timeMillis();
}

float refreshCpuTemperature() {
  unsigned long now = timeMillis();
  if (now - lastCpuTempRead >= CPU_TEMP_READ_INTERVAL) {
    cachedCpuTemp.store(temperatureRead(), std::memory_order_relaxed);
    lastCpuTempRead = now;
  }
  return cachedCpuTemp.load(std::memory_order_relaxed);
}

float getCachedCpuTemperature() {
  return cachedCpuTemp.load(std::memory_order_relaxed);
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

  // Barometer temperature comes from the dedicated producer's coherent sample;
  // this callback never performs I2C from the monitoring task.
  static SensorMonitor* baroTemp = new SensorMonitor(
    SensorID::Baro_Temp,
    SensorCategory::ALTIMETER,
    baroTempThresholds,
    []() { return getCachedBaroTemperature(); },
    &multiLogger);
  monitors.push_back(baroTemp);
}
