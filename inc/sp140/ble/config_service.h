#ifndef INC_SP140_BLE_CONFIG_SERVICE_H_
#define INC_SP140_BLE_CONFIG_SERVICE_H_

#include <string>

class NimBLEServer;

void initConfigBleService(NimBLEServer* server, const std::string& uniqueId);
void updateThrottleBLE(int value);

#endif  // INC_SP140_BLE_CONFIG_SERVICE_H_
