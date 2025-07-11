#include "sp140/system_monitors.h"
#include "sp140/monitor_config.h"
#include "sp140/altimeter.h"
#include <Arduino.h>

// External references to core monitoring infrastructure
extern std::vector<IMonitor*> monitors;
extern MultiLogger multiLogger;

void addInternalMonitors() {
  // ESP32-S3 CPU Temperature (Warning: 50°C, Critical: 80°C)
  static SensorMonitor* cpuTemp = new SensorMonitor(
    SensorID::CPU_Temp,
    cpuTempThresholds,
    []() { return temperatureRead(); },
    &multiLogger);
  monitors.push_back(cpuTemp);
}

void addAltimeterMonitors() {
  // Barometer Temperature monitor disabled for now.
  // TODO:
  // The BMP barometer shares the I2C bus with other peripherals; without a
  // global I2C mutex the `Wire` driver can log "Unfinished Repeated Start"
  // errors when transactions from multiple tasks interleave.  Until we
  // introduce a proper mutex (or move barometer access into a dedicated
  // task), skip registering this monitor to avoid the noise and potential
  // bus resets.
  //
  // static SensorMonitor* baroTemp = new SensorMonitor(
  //   SensorID::Baro_Temp,
  //   baroTempThresholds,
  //   []() { return getBaroTemperature(); },
  //   &multiLogger);
  // monitors.push_back(baroTemp);
}
