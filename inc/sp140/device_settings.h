#ifndef INC_SP140_DEVICE_SETTINGS_H_
#define INC_SP140_DEVICE_SETTINGS_H_

#include "structs.h"
#include "esp32s3-config.h"

// Constants
extern const unsigned int DEFAULT_SCREEN_ROTATION;
void refreshDeviceData();
void writeDeviceData();
void resetDeviceData();
void parse_serial_commands();
void send_device_data();
bool sanitizeDeviceData();
void debugHardwareConfig(const HardwareConfig& config);

#endif // INC_SP140_DEVICE_SETTINGS_H_
