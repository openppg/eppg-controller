#ifndef INC_SP140_BLE_H_
#define INC_SP140_BLE_H_

#include <NimBLEDevice.h>
#include <NimBLEUtils.h>
#include <NimBLEServer.h>

// BLE globals
extern NimBLECharacteristic* pThrottleCharacteristic;
extern NimBLECharacteristic* pDeviceStateCharacteristic;
extern NimBLEServer* pServer;
extern volatile uint16_t connectedHandle;
extern volatile bool deviceConnected;
extern volatile bool oldDeviceConnected;

#endif  // INC_SP140_BLE_H_
