#ifndef INC_SP140_BLE_H_
#define INC_SP140_BLE_H_

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// BLE globals
extern BLECharacteristic* pThrottleCharacteristic;
extern BLECharacteristic* pDeviceStateCharacteristic;
extern BLEServer* pServer;
extern bool deviceConnected;
extern bool oldDeviceConnected;

#endif  // INC_SP140_BLE_H_
