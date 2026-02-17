#ifndef INC_SP140_BLE_BLE_CORE_H_
#define INC_SP140_BLE_BLE_CORE_H_

// Core BLE helpers for initializing and maintaining the server.

void setupBLE();

// Allow modules to trigger advertising after disconnects.
void restartBLEAdvertising();

// Request fast connection parameters (15ms interval) for OTA transfers.
void requestFastConnParams();

// Restore normal connection parameters (~36ms interval) after OTA.
void requestNormalConnParams();

#endif  // INC_SP140_BLE_BLE_CORE_H_
