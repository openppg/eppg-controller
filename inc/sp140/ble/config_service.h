#ifndef INC_SP140_BLE_CONFIG_SERVICE_H_
#define INC_SP140_BLE_CONFIG_SERVICE_H_

#include <string>

class NimBLEServer;

void initConfigBleService(NimBLEServer* server, const std::string& uniqueId);
void updateThrottleBLE(int value);

// Push ESC relay status + stream the result blob over notify. Call from the
// 50 Hz BLE notify task.
void pumpEscRelayNotify();

#endif  // INC_SP140_BLE_CONFIG_SERVICE_H_
