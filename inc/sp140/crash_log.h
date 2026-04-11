#ifndef INC_SP140_CRASH_LOG_H_
#define INC_SP140_CRASH_LOG_H_

#include <Arduino.h>
#include "sp140/device_state.h"

// Boot loop detection thresholds
#define BOOT_LOOP_THRESHOLD 5
#define BOOT_LOOP_UPTIME_MS 30000  // 30 seconds

// Read previous crash data from NVS and print to serial.
// Call early in setup(), after USBSerial.begin().
void crashLogReadAndReport();

// Write current state as a heartbeat to NVS.
// Call periodically from a low-priority task (e.g., monitoringTask).
void crashLogHeartbeat();

// Update the persisted armed state in NVS.
// Call from changeDeviceState().
void crashLogUpdateArmedState(DeviceState state);

// Send crash log data as JSON over serial.
// Called by the "crash" serial command.
void sendCrashLogData();

#endif  // INC_SP140_CRASH_LOG_H_
