#include "sp140/ble.h"

// Define the globals
NimBLECharacteristic* pThrottleCharacteristic = nullptr;
NimBLECharacteristic* pDeviceStateCharacteristic = nullptr;
NimBLEServer* pServer = nullptr;
volatile bool deviceConnected = false;
