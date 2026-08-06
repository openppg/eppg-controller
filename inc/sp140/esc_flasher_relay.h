#ifndef INC_SP140_ESC_FLASHER_RELAY_H_
#define INC_SP140_ESC_FLASHER_RELAY_H_

#include <stdint.h>

// =============================================================================
// ESC Firmware Relay
//
// Relays an ESC firmware image from the phone app (over BLE) to the ESC (over
// CAN), using the SINE/Mad Motors bootloader protocol (GetBootStatus 200 /
// StartFwUpgrade 201 / SendFwData 202 / EndFwUpgrade 203) ported from
// powerpack-flash-qc/src/esc_flasher.cpp.
//
// Transport model: BUFFER-THEN-FLASH.
//   1. The app sends FW_START(hwId, totalSize), then streams the complete image
//      (32-byte header + firmware) in offset-addressed chunks into a controller
//      heap/PSRAM buffer (BLE task), then FW_END.
//   2. Only after the full image is received does the throttle task (the sole
//      CAN owner) stream it to the ESC bootloader in 256-byte SendFwData chunks,
//      paced by the ESC's next_index flow control.
// This decouples the BLE transfer from the CAN streaming entirely, so a slow
// ESC flash cannot trip a BLE idle timeout, and the buffer is written by only
// one task at a time (BLE during RECEIVING, throttle during flashing).
//
// Safety: only starts while DISARMED; aborts if the device arms; the controller
// never reboots (only the ESC does); the hardware_id is validated against the
// connected ESC before entering the bootloader. Fully non-blocking on the
// throttle task (no delay()/vTaskDelay()), so the 50 Hz control loop is never
// stalled. See: powerpack-flash-qc/configs/ESC-Config-Relay-Design.md (§4).
// =============================================================================

enum class EscFwPhase : uint8_t {
  IDLE = 0,
  RECEIVING,      // BLE filling the image buffer
  RESTARTING,     // RestartNode sent, waiting (non-blocking) for ESC reboot
  CHECKING_BOOT,  // polling GetBootStatus for bootloader mode
  STARTING,       // StartFwUpgrade sent
  SENDING,        // streaming SendFwData chunks
  ENDING,         // EndFwUpgrade sent
  DONE_OK,
  DONE_FAIL,
};

enum class EscFwCode : uint8_t {
  IDLE           = 0x00,
  RECEIVING      = 0x01,
  ENTER_BOOTLDR  = 0x02,
  FLASHING       = 0x03,
  SUCCESS        = 0x04,
  FAILED         = 0x80,
  REJECTED_ARMED = 0x81,
  TIMEOUT        = 0x82,
  REJECTED_HWID  = 0x83,   // image hardware_id != connected ESC
  REJECTED_BUSY  = 0x84,   // another ESC session active / alloc failed / bad size
  ESC_REJECTED   = 0x85,   // ESC refused the image (bootloader/Start/End state != 0)
};

struct EscFwStatus {
  EscFwCode code;
  EscFwPhase phase;
  uint16_t progressPermille;  // 0..1000
};

// Call once from initESC() (after the CanardAdapter has been begun).
void escFlasherRelayInit();

// Drive the flasher state machine. MUST be called from the throttle task — it
// is invoked from readESCTelemetry(). Non-blocking.
void escFlasherRelayServiceTick();

// ---- BLE-side API (safe to call from the BLE task) -------------------------
// Begin a transfer: allocate a buffer for totalSize bytes (the complete image
// incl. 32-byte header) and validate hwId against the connected ESC. Returns
// false if armed, busy, hwId mismatch, size out of range, or allocation fails.
bool escFlasherRelayBegin(uint16_t hardwareId, uint32_t totalSize);

// Copy a received chunk into the image buffer at the given offset. Returns the
// total number of contiguous-from-zero bytes received so far, or -1 on error.
int32_t escFlasherRelayWriteChunk(uint32_t offset, const uint8_t* data, uint16_t len);

// All bytes sent: validate and hand off to the throttle task to flash. Returns
// false if the image is incomplete/invalid.
bool escFlasherRelayEnd();

// Abort and free the buffer.
void escFlasherRelayAbort();

// Latched status snapshot for the BLE status read/notify.
EscFwStatus escFlasherRelayGetStatus();

// True while receiving OR flashing (for the arm interlock).
bool escFlasherRelayIsActive();

#endif  // INC_SP140_ESC_FLASHER_RELAY_H_
