#ifndef INC_SP140_SYSTEM_MONITORS_H_
#define INC_SP140_SYSTEM_MONITORS_H_

#include "simple_monitor.h"

// System monitoring functions
void addInternalMonitors();
void addAltimeterMonitors();

// CPU temperature cache (single-writer; see system_monitors.cpp).
// primeCpuTemperatureCache: one-time read during single-threaded setup.
void primeCpuTemperatureCache();
// refreshCpuTemperature: re-reads tsens at most once per second. Call ONLY
// from ctrlSensorTask — it is the sole runtime owner of the sensor.
float refreshCpuTemperature();
// getCachedCpuTemperature: lock-free read of the cached value, safe anywhere.
float getCachedCpuTemperature();

#endif  // INC_SP140_SYSTEM_MONITORS_H_
