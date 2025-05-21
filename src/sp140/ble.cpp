#include "sp140/ble.h"

// Define the globals
BLECharacteristic* pThrottleCharacteristic = nullptr;
BLECharacteristic* pDeviceStateCharacteristic = nullptr;
BLEServer* pServer = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;
