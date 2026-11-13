#ifndef INC_SP140_BLE_H_
#define INC_SP140_BLE_H_

#include "platform/ble.h"

// BLE globals
extern platform::BleCharacteristic* pThrottleCharacteristic;
extern platform::BleCharacteristic* pDeviceStateCharacteristic;
extern platform::BleServer* pServer;
extern volatile uint16_t connectedHandle;
extern volatile bool deviceConnected;

#endif  // INC_SP140_BLE_H_
