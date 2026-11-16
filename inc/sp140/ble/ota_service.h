#ifndef INC_SP140_BLE_OTA_SERVICE_H_
#define INC_SP140_BLE_OTA_SERVICE_H_

#include <NimBLEDevice.h>
#include "freertos/FreeRTOS.h"
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
 * Holds the OTA state lock while toggleArm() decides to arm. Arming checks
 * isOtaInProgress() and then needs up to ~50 ms (barometer datum) before it
 * sets ARMED; a CMD_START landing in that gap would see DISARMED, start the
 * update and later reboot an armed controller. With the guard held across the
 * check and the ARMED transition, START waits and then sees ARMED.
 * held() is false when the lock couldn't be taken in time, which means a
 * START is erasing flash right now: don't arm.
 */
class OtaArmGuard {
 public:
  explicit OtaArmGuard(TickType_t timeout);
  ~OtaArmGuard();
  OtaArmGuard(const OtaArmGuard&) = delete;
  OtaArmGuard& operator=(const OtaArmGuard&) = delete;
  bool held() const { return held_; }

 private:
  bool held_ = false;
  bool taken_ = false;
};

/**
 * Abort any in-progress OTA update and reset state.
 * Safe to call even if no OTA is in progress. `error` is only shown on the
 * update screen when a session was actually running.
 */
void abortOta(OtaError error = OtaError::Disconnected);

// Status for the update screen. Nonblocking: a queue peek that never takes the
// OTA state lock, so the UI task can't wait on a flash operation.
bool readOtaStatus(OtaStatus& status);

// True while an update runs or its result is still on screen. Nonblocking.
bool otaBlocksInput();

// Called once at the end of setup(), after the first-boot validation, with
// whether esp_ota_mark_app_valid_cancel_rollback() succeeded (true when the
// image was not pending verification). Consumes the one-shot "ota-ui" receipt
// written just before the post-update reboot and publishes the result screen.
void finishOtaBoot(bool firmwareValid);

/**
 * Check if OTA has been idle too long and abort if so.
 * Call periodically (e.g. from a monitoring task).
 */
void checkOtaTimeout();

#endif  // INC_SP140_BLE_OTA_SERVICE_H_
