#include "platform/runtime.h"
#include "sp140/ble.h"

// Define the globals
platform::BleCharacteristic* pThrottleCharacteristic = nullptr;
platform::BleCharacteristic* pDeviceStateCharacteristic = nullptr;
platform::BleServer* pServer = nullptr;
volatile uint16_t connectedHandle = BLE_HS_CONN_HANDLE_NONE;
volatile bool deviceConnected = false;
