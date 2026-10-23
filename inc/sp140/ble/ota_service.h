#ifndef INC_SP140_BLE_OTA_SERVICE_H_
#define INC_SP140_BLE_OTA_SERVICE_H_

#include <NimBLEDevice.h>
#include "sp140/ble/ota_status.h"

/**
 * Initialize the OTA BLE service.
 * @param pServer Pointer to the NimBLEServer instance.
 */
void initOtaBleService(NimBLEServer* pServer);

/**
 * Check if an OTA update is currently in progress.
 * @return true if OTA is active (should pause other tasks)
 */
bool isOtaInProgress();

/**
 * Abort any in-progress OTA update and reset state.
 * Safe to call even if no OTA is in progress.
 */
void abortOta(OtaError error = OtaError::Disconnected);

// Nonblocking snapshot, independent of the mutex held during flash operations.
bool readOtaStatus(OtaStatus& status);
void acknowledgeOtaRendered(uint32_t revision);
bool otaBlocksInput();

// Called after startup validation, before advertising. Handles the persistent
// boot receipt even on devices whose older bootloader lacks rollback support.
void finishOtaBoot(bool firmwareValid);

/**
 * Check if OTA has been idle too long and abort if so.
 * Call periodically (e.g. from a monitoring task).
 */
void checkOtaTimeout();

#endif  // INC_SP140_BLE_OTA_SERVICE_H_
