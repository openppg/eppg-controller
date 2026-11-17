#ifndef INC_SP140_SYSTEM_MONITORS_H_
#define INC_SP140_SYSTEM_MONITORS_H_

#include "simple_monitor.h"

// System monitoring functions
void addInternalMonitors();
void addAltimeterMonitors();

// Prime the CPU temperature cache during single-threaded setup.
void primeCpuTemperatureCache();

// Refresh the sensor at most once per second. ctrlSensorTask is the sole
// runtime owner; all other tasks must use getCachedCpuTemperature().
float refreshCpuTemperature();

// Cross-core read of the last published CPU temperature; never touches tsens.
float getCachedCpuTemperature();

#endif  // INC_SP140_SYSTEM_MONITORS_H_
