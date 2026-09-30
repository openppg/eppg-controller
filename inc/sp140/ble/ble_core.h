#ifndef INC_SP140_BLE_BLE_CORE_H_
#define INC_SP140_BLE_BLE_CORE_H_

#include <cstdint>

// Core BLE helpers for initializing and maintaining the server.

void setupBLE();

// Allow modules to trigger advertising after disconnects.
void restartBLEAdvertising();

// Request fast connection parameters (15ms interval) for OTA transfers.
void requestFastConnParams();

// Restore normal connection parameters (~36ms interval) after OTA.
void requestNormalConnParams();

// Temporarily disable whitelist filtering so a new device can bond.
// Advertising reopens for ~60 seconds then whitelisting is restored.
void enterBLEPairingMode();
bool isBLEPairingModeActive();

// Negotiated ATT MTU of the active connection: 23 (BLE default) from connect
// until the MTU exchange completes, then the negotiated value; 0 while
// disconnected. Full Fast-Link telemetry notifies need MTU >= payload + 3.
uint16_t getNegotiatedBLEMtu();

#endif  // INC_SP140_BLE_BLE_CORE_H_
