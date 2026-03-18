#include "sp140/ble.h"

// Define the globals
NimBLECharacteristic* pThrottleCharacteristic = nullptr;
NimBLECharacteristic* pDeviceStateCharacteristic = nullptr;
NimBLEServer* pServer = nullptr;
volatile uint16_t connectedHandle = BLE_HS_CONN_HANDLE_NONE;
volatile bool deviceConnected = false;
volatile bool oldDeviceConnected = false;
