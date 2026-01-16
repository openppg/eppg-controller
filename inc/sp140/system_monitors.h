#ifndef INC_SP140_SYSTEM_MONITORS_H_
#define INC_SP140_SYSTEM_MONITORS_H_

#include "simple_monitor.h"

// System monitoring functions
void addInternalMonitors();
void addAltimeterMonitors();

// Thread-safe cached CPU temperature reading (updates max once per second)
float getCachedCpuTemperature();

#endif  // INC_SP140_SYSTEM_MONITORS_H_
