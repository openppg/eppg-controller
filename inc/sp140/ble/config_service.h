#ifndef INC_SP140_BLE_CONFIG_SERVICE_H_
#define INC_SP140_BLE_CONFIG_SERVICE_H_

class BLEServer;

void initConfigBleService(BLEServer* server);
void updateThrottleBLE(int value);

#endif  // INC_SP140_BLE_CONFIG_SERVICE_H_
