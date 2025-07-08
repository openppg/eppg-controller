#ifndef BLE_OTA_TEST_H
#define BLE_OTA_TEST_H

#ifdef ENABLE_BLE_OTA

// BLE OTA management functions
void setupBLEOTA();
void loopBLEOTA();
void cleanupBLEOTA();

// Status functions
bool isBLEOTAActive();
bool isBLEOTAConnected();

// Information and testing functions
void printOTAInfo();
bool testOTAPartitions();
void printFirmwareInfo();

#else

// Stub functions when BLE OTA is disabled
inline void setupBLEOTA() {}
inline void loopBLEOTA() {}
inline void cleanupBLEOTA() {}
inline bool isBLEOTAActive() { return false; }
inline bool isBLEOTAConnected() { return false; }
inline void printOTAInfo() {}
inline bool testOTAPartitions() { return true; }
inline void printFirmwareInfo() {}

#endif // ENABLE_BLE_OTA

#endif // BLE_OTA_TEST_H