#include "sp140/ble.h"

// Define the globals
BLECharacteristic* pThrottleCharacteristic = nullptr;
BLECharacteristic* pDeviceStateCharacteristic = nullptr;
BLEServer* pServer = nullptr;
volatile bool deviceConnected = false;
volatile bool oldDeviceConnected = false;
