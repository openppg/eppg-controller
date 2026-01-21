#include "sp140/system_monitors.h"
#include "sp140/monitor_config.h"
#include "sp140/altimeter.h"
#include <Arduino.h>

// External references to core monitoring infrastructure
extern std::vector<IMonitor*> monitors;
extern MultiLogger multiLogger;

// External reference to BMP sensor status
extern bool bmpPresent;

// Cached CPU temperature to avoid "tsens: Do not configure the temp sensor when it's running!" error
static float cachedCpuTemp = 0.0f;
static unsigned long lastCpuTempRead = 0;
static const unsigned long CPU_TEMP_READ_INTERVAL = 1000;  // Read every 1 second

float getCachedCpuTemperature() {
  unsigned long now = millis();
  if (now - lastCpuTempRead >= CPU_TEMP_READ_INTERVAL) {
    cachedCpuTemp = temperatureRead();
    lastCpuTempRead = now;
  }
  return cachedCpuTemp;
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

  // Barometer Temperature monitor disabled for now.
  // TODO: The BMP barometer shares the I2C bus with other peripherals; without a
  // global I2C mutex the `Wire` driver can log "Unfinished Repeated Start"
  // errors when transactions from multiple tasks interleave.  Until we
  // introduce a proper mutex (or move barometer access into a dedicated
  // task), skip registering this monitor to avoid the noise and potential
  // bus resets.
  //
  // static SensorMonitor* baroTemp = new SensorMonitor(
  //   SensorID::Baro_Temp,
  //   SensorCategory::ALTIMETER,
  //   baroTempThresholds,
  //   []() { return getBaroTemperature(); },
  //   &multiLogger);
  // monitors.push_back(baroTemp);
}
