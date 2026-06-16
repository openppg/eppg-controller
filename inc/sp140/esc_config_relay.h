#ifndef INC_SP140_ESC_CONFIG_RELAY_H_
#define INC_SP140_ESC_CONFIG_RELAY_H_

#include <stdint.h>

// =============================================================================
// ESC Configuration Relay
//
// Relays SINE / Mad Motors ESC parameter-config commands from the phone app
// (over BLE) to the ESC (over CAN), using the proven SetConfig / SaveConfig /
// RestartNode / GetConfig sequence ported from powerpack-flash-qc.
//
// Concurrency model: the controller's single CAN owner is the throttle task
// (readESCTelemetry() -> adapter.processTxRxOnce()). All CAN traffic for a
// relay session therefore runs on that task via escConfigRelayServiceTick().
// The BLE task only ENQUEUES a request (escConfigRelayRequest*), exactly like
// the existing requestEscHardwareInfo() flag handoff in esc.cpp. The CanardAdapter
// is never touched from two tasks, so no mutex is needed on it.
//
// Safety: a session only starts while the device is DISARMED. The session is
// fully non-blocking — it never calls delay()/vTaskDelay(), so it cannot stall
// the 50 Hz control loop. Only the ESC reboots; the controller does not.
//
// See: powerpack-flash-qc/configs/ESC-Config-Relay-Design.md
// =============================================================================

// Canonical "reverse motor direction" parameter (SINE config_id 0x0080, UINT16,
// 0 = positive/normal, 1 = inversion/reversed). It is a "basic" parameter,
// documented as accessible WITHOUT the password — see
// powerpack-flash-qc/configs/ESC-CAN-Config-Protocol.md.
static const uint16_t ESC_PARAM_DIRECTION = 0x0080;

enum class EscRelayPhase : uint8_t {
  IDLE = 0,
  UNLOCK,
  WRITE,
  SAVE,
  RESTART,
  WAIT_REBOOT,
  VERIFY,
  DONE_OK,
  DONE_FAIL,
};

// Status codes surfaced to the app (ESC_RELAY_STATUS notify byte 0 in the
// design doc).
enum class EscRelayStatusCode : uint8_t {
  IDLE            = 0x00,
  ACCEPTED        = 0x01,
  RUNNING         = 0x02,
  REBOOTING       = 0x03,
  VERIFIED_OK     = 0x04,
  FAILED          = 0x80,
  REJECTED_ARMED  = 0x81,
  TIMEOUT         = 0x82,
  VERIFY_MISMATCH = 0x83,
  ESC_FLAG_ERROR  = 0x85,
};

struct EscRelayStatus {
  EscRelayStatusCode code;
  EscRelayPhase phase;
  uint16_t config_id;
  uint8_t detail;          // raw ESC flag byte on failure, else 0
  uint8_t readback[8];     // verified read-back bytes (valid on VERIFIED_OK)
  uint8_t readback_len;
};

// Call once from initESC() (after the CanardAdapter has been begun).
void escConfigRelayInit();

// Drive the session state machine. MUST be called from the throttle task — it
// is invoked from readESCTelemetry(). Non-blocking.
void escConfigRelayServiceTick();

// ---- Request API (safe to call from the BLE task) --------------------------
// Returns false if a session is already in progress, the payload is too large,
// or the relay is not initialized. The DISARMED check is (re)enforced on the
// throttle task before the session actually starts.
//
// Writes a single parameter, persists it (SaveConfig), restarts the ESC, then
// verifies the read-back across the reboot (GetConfig byte-compare).
bool escConfigRelayRequestSetParam(uint16_t config_id,
                                   const uint8_t* data, uint8_t len);

// Convenience for the canonical reverse-direction toggle.
bool escConfigRelayRequestReverseDirection(bool reversed);

// Latched status snapshot for the BLE status characteristic / UI.
EscRelayStatus escConfigRelayGetStatus();

// True while a session is in progress OR a request is pending (for the arm
// interlock — block arming while this is true).
bool escConfigRelayIsActive();

// Require PasswordUnlock (SINE service 225) before writing, and re-unlock after
// the ESC reboots. Default false. The reverse-direction param does NOT need it;
// restricted params do. The decisive bench test runs with this false to confirm
// that 0x0080 is writable from the controller's node 0x01 with no unlock.
void escConfigRelaySetRequireUnlock(bool require);

#endif  // INC_SP140_ESC_CONFIG_RELAY_H_
