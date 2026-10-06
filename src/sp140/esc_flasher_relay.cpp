#include "sp140/esc_flasher_relay.h"

#include <Arduino.h>
#include "sp140/time_utils.h"
#include <string.h>
#include <stdlib.h>
#include <canard.h>
#include <CanardAdapter.h>
#include <dronecan_msgs.h>
#include "esp_heap_caps.h"

#include "sp140/esc.h"               // escAdapter(), escTelemetryData
#include "sp140/device_state.h"      // DeviceState
#include "sp140/esc_config_relay.h"  // escConfigRelayIsActive() cross-gate

extern volatile DeviceState currentState;

static const uint8_t  ESC_CAN_NODE_ID = 0x20;
static const uint8_t  CONFIG_HOST_NODE_ID = 0x40;  // ESC restricts bootloader/config to host 0x40
static const uint8_t  CTRL_NORMAL_NODE_ID = 0x01;  // controller's normal CAN node id
static const uint16_t FW_CHUNK_SIZE   = 256;     // SendFwData payload (matches flash-qc)
static const unsigned long FW_TIMEOUT_MS = 150;  // per-request response wait
static const uint8_t  FW_MAX_RETRIES  = 10;
static const unsigned long REBOOT_WAIT_MS = 700;   // ESC reboot before bootloader answers
static const unsigned long RX_TIMEOUT_MS  = 15000;  // abort if the app stalls mid-transfer
static const uint32_t MAX_IMAGE_SIZE  = 256u * 1024u;  // sanity cap
static const uint32_t HEADER_SIZE     = 32;            // firmware data starts at byte 32

// =============================================================================
// Response capture (decoded on the throttle task inside processTxRxOnce).
// =============================================================================
static volatile bool     s_gotResponse = false;
static volatile uint8_t  s_responseState = 0xFF;
static volatile uint16_t s_responseNextIndex = 0;
static volatile uint8_t  s_bootMode = 0;

static uint8_t s_tidRestart = 0, s_tidBoot = 0, s_tidStart = 0, s_tidData = 0, s_tidEnd = 0;

// =============================================================================
// CanardAdapterNode capturing the bootloader-protocol responses.
// Mirrors EscFlasherNode in powerpack-flash-qc/src/esc_flasher.cpp.
// =============================================================================
class EscFlasherRelayNode : public CanardAdapterNode {
 public:
  explicit EscFlasherRelayNode(CanardAdapter& adapter) : CanardAdapterNode(adapter) {}
  void begin() { _beginNode(); }
  CanardInstance* canard() { return _canard; }

  bool shouldAcceptTransfer(uint64_t* out_signature,
                            uint16_t data_type_id,
                            CanardTransferType transfer_type,
                            uint8_t source_node_id) override {
    if (transfer_type != CanardTransferTypeResponse || source_node_id != ESC_CAN_NODE_ID)
      return false;
    switch (data_type_id) {
      case SINE_ESC_GETBOOTSTATUS_ID:       *out_signature = SINE_ESC_GETBOOTSTATUS_SIGNATURE;  return true;
      case SINE_ESC_STARTFWUPGRADE_ID:      *out_signature = SINE_ESC_STARTFWUPGRADE_SIGNATURE; return true;
      case SINE_ESC_SENDFWDATA_RESPONSE_ID: *out_signature = SINE_ESC_SENDFWDATA_RESPONSE_SIGNATURE; return true;
      case SINE_ESC_ENDFWUPGRADE_ID:        *out_signature = SINE_ESC_ENDFWUPGRADE_SIGNATURE;   return true;
      case UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_ID:
        *out_signature = UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_SIGNATURE; return true;
      default: return false;
    }
  }

  void onTransferReceived(CanardRxTransfer* t) override {
    if (t->transfer_type != CanardTransferTypeResponse) return;
    switch (t->data_type_id) {
      case SINE_ESC_GETBOOTSTATUS_ID: {
        sine_esc_GetBootStatusResponse res;
        if (!sine_esc_GetBootStatusResponse_decode(t, &res)) { s_bootMode = res.mode; s_gotResponse = true; }
        break;
      }
      case SINE_ESC_STARTFWUPGRADE_ID: {
        sine_esc_StartFwUpgradeResponse res;
        if (!sine_esc_StartFwUpgradeResponse_decode(t, &res)) { s_responseState = res.state; s_gotResponse = true; }
        break;
      }
      case SINE_ESC_SENDFWDATA_RESPONSE_ID: {
        sine_esc_SendFwDataResponse res;
        if (!sine_esc_SendFwDataResponse_decode(t, &res)) {
          s_responseState = res.state; s_responseNextIndex = res.next_index; s_gotResponse = true;
        }
        break;
      }
      case SINE_ESC_ENDFWUPGRADE_ID: {
        sine_esc_EndFwUpgradeResponse res;
        if (!sine_esc_EndFwUpgradeResponse_decode(t, &res)) { s_responseState = res.state; s_gotResponse = true; }
        break;
      }
      case UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_ID: {
        uavcan_protocol_RestartNodeResponse res;
        if (!uavcan_protocol_RestartNodeResponse_decode(t, &res)) { s_responseState = res.ok ? 0 : 1; s_gotResponse = true; }
        break;
      }
      default: break;
    }
  }
};

static EscFlasherRelayNode* s_node = nullptr;

// =============================================================================
// Session state (guarded by s_mux; buffer contents are written only during
// RECEIVING on the BLE task and read only during flashing on the throttle task)
// =============================================================================
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static uint8_t* s_buf = nullptr;
static uint32_t s_bufSize = 0;       // total image size (header + firmware)
static uint32_t s_received = 0;      // contiguous bytes received from offset 0
static volatile bool s_abortRequested = false;

static EscFwPhase s_phase = EscFwPhase::IDLE;
static EscFwStatus s_status = { EscFwCode::IDLE, EscFwPhase::IDLE, 0 };

// Parsed header / flash progress (throttle task).
static uint16_t s_hwId = 0, s_sizeKb = 0, s_crc = 0;
static uint16_t s_totalChunks = 0, s_curChunk = 0;
static unsigned long s_lastSendMs = 0, s_phaseStartMs = 0, s_lastRxMs = 0;
static uint8_t s_retries = 0;

static void setStatus(EscFwCode code, EscFwPhase phase, uint16_t permille) {
  portENTER_CRITICAL(&s_mux);
  s_status.code = code; s_status.phase = phase; s_status.progressPermille = permille;
  portEXIT_CRITICAL(&s_mux);
}

// Free the buffer. Only ever called on the throttle task (in the tick) or by
// Begin's own failure path before any throttle access.
static void freeBuffer() {
  portENTER_CRITICAL(&s_mux);
  uint8_t* p = s_buf;
  s_buf = nullptr;
  s_bufSize = 0;
  s_received = 0;
  portEXIT_CRITICAL(&s_mux);
  if (p) free(p);
}

static void finishFail(EscFwCode code) {
  freeBuffer();
  setStatus(code, EscFwPhase::DONE_FAIL, s_status.progressPermille);
  escAdapter().setLocalNodeId(CTRL_NORMAL_NODE_ID);  // restore normal CAN identity
  s_phase = EscFwPhase::DONE_FAIL;
}

// =============================================================================
// CAN send helpers (throttle task) — use the library encoders directly.
// =============================================================================
static void sendRestartNode() {
  uavcan_protocol_RestartNodeRequest req;
  req.magic_number = UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAGIC_NUMBER;  // 0xACCE551B1E
  uint8_t buffer[UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAX_SIZE];
  uint32_t len = uavcan_protocol_RestartNodeRequest_encode(&req, buffer);
  CanardTxTransfer transfer = {
    .transfer_type = CanardTransferTypeRequest,
    .data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_SIGNATURE,
    .data_type_id = UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_ID,
    .inout_transfer_id = &s_tidRestart,
    .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
    .payload = buffer,
    .payload_len = (uint16_t)len,
  };
  canardRequestOrRespondObj(s_node->canard(), ESC_CAN_NODE_ID, &transfer);
}

static void sendGetBootStatus() {
  sine_esc_GetBootStatusRequest req;
  uint8_t buffer[SINE_ESC_GETBOOTSTATUS_REQUEST_MAX_SIZE];
  uint32_t len = sine_esc_GetBootStatusRequest_encode(&req, buffer);
  CanardTxTransfer transfer = {
    .transfer_type = CanardTransferTypeRequest,
    .data_type_signature = SINE_ESC_GETBOOTSTATUS_REQUEST_SIGNATURE,
    .data_type_id = SINE_ESC_GETBOOTSTATUS_REQUEST_ID,
    .inout_transfer_id = &s_tidBoot,
    .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
    .payload = buffer,
    .payload_len = (uint16_t)len,
  };
  canardRequestOrRespondObj(s_node->canard(), ESC_CAN_NODE_ID, &transfer);
}

static void sendStartUpgrade() {
  sine_esc_StartFwUpgradeRequest req = {
    .hardware_id = s_hwId,
    .filse_size_kb = s_sizeKb,
    .file_crc = s_crc,
  };
  uint8_t buffer[SINE_ESC_STARTFWUPGRADE_REQUEST_MAX_SIZE];
  uint32_t len = sine_esc_StartFwUpgradeRequest_encode(&req, buffer);
  CanardTxTransfer transfer = {
    .transfer_type = CanardTransferTypeRequest,
    .data_type_signature = SINE_ESC_STARTFWUPGRADE_REQUEST_SIGNATURE,
    .data_type_id = SINE_ESC_STARTFWUPGRADE_REQUEST_ID,
    .inout_transfer_id = &s_tidStart,
    .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
    .payload = buffer,
    .payload_len = (uint16_t)len,
  };
  canardRequestOrRespondObj(s_node->canard(), ESC_CAN_NODE_ID, &transfer);
}

static void sendFwDataChunk(uint16_t index) {
  sine_esc_SendFwDataRequest req;
  req.index = index;
  memset(req.data, 0xFF, FW_CHUNK_SIZE);
  uint32_t off = HEADER_SIZE + (uint32_t)index * FW_CHUNK_SIZE;
  if (s_buf && off < s_bufSize) {
    uint32_t remaining = s_bufSize - off;
    uint32_t copyLen = (remaining < FW_CHUNK_SIZE) ? remaining : FW_CHUNK_SIZE;
    memcpy(req.data, s_buf + off, copyLen);
  }
  uint8_t buffer[SINE_ESC_SENDFWDATA_REQUEST_MAX_SIZE];
  uint32_t len = sine_esc_SendFwDataRequest_encode(&req, buffer);
  CanardTxTransfer transfer = {
    .transfer_type = CanardTransferTypeRequest,
    .data_type_signature = SINE_ESC_SENDFWDATA_REQUEST_SIGNATURE,
    .data_type_id = SINE_ESC_SENDFWDATA_REQUEST_ID,
    .inout_transfer_id = &s_tidData,
    .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
    .payload = buffer,
    .payload_len = (uint16_t)len,
  };
  canardRequestOrRespondObj(s_node->canard(), ESC_CAN_NODE_ID, &transfer);
}

static void sendEndUpgrade() {
  sine_esc_EndFwUpgradeRequest req;
  uint8_t buffer[SINE_ESC_ENDFWUPGRADE_REQUEST_MAX_SIZE];
  uint32_t len = sine_esc_EndFwUpgradeRequest_encode(&req, buffer);
  CanardTxTransfer transfer = {
    .transfer_type = CanardTransferTypeRequest,
    .data_type_signature = SINE_ESC_ENDFWUPGRADE_REQUEST_SIGNATURE,
    .data_type_id = SINE_ESC_ENDFWUPGRADE_REQUEST_ID,
    .inout_transfer_id = &s_tidEnd,
    .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
    .payload = buffer,
    .payload_len = (uint16_t)len,
  };
  canardRequestOrRespondObj(s_node->canard(), ESC_CAN_NODE_ID, &transfer);
}

// =============================================================================
// Public API
// =============================================================================
void escFlasherRelayInit() {
  if (!s_node) {
    s_node = new EscFlasherRelayNode(escAdapter());
    s_node->begin();
  }
  s_phase = EscFwPhase::IDLE;
  setStatus(EscFwCode::IDLE, EscFwPhase::IDLE, 0);
}

bool escFlasherRelayIsActive() {
  return s_phase != EscFwPhase::IDLE &&
         s_phase != EscFwPhase::DONE_OK &&
         s_phase != EscFwPhase::DONE_FAIL;
}

bool escFlasherRelayBegin(uint16_t hardwareId, uint32_t totalSize) {
  if (!s_node) return false;
  if (escFlasherRelayIsActive() || escConfigRelayIsActive()) {
    setStatus(EscFwCode::REJECTED_BUSY, EscFwPhase::DONE_FAIL, 0);
    return false;
  }
  if (currentState != DISARMED) {
    setStatus(EscFwCode::REJECTED_ARMED, EscFwPhase::DONE_FAIL, 0);
    return false;
  }
  if (totalSize < HEADER_SIZE || totalSize > MAX_IMAGE_SIZE) {
    setStatus(EscFwCode::REJECTED_BUSY, EscFwPhase::DONE_FAIL, 0);
    return false;
  }
  // Validate against the connected ESC: refuse a mismatched image before we
  // ever put the ESC into the bootloader (classic brick cause).
  uint16_t connectedHw = escTelemetryData.hardware_id;
  if (connectedHw == 0 || connectedHw != hardwareId) {
    setStatus(EscFwCode::REJECTED_HWID, EscFwPhase::DONE_FAIL, 0);
    return false;
  }

  uint8_t* buf = static_cast<uint8_t*>(heap_caps_malloc(totalSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!buf) buf = static_cast<uint8_t*>(malloc(totalSize));
  if (!buf) {
    setStatus(EscFwCode::REJECTED_BUSY, EscFwPhase::DONE_FAIL, 0);
    return false;
  }

  portENTER_CRITICAL(&s_mux);
  s_buf = buf;
  s_bufSize = totalSize;
  s_received = 0;
  s_abortRequested = false;
  s_phase = EscFwPhase::RECEIVING;
  s_status.code = EscFwCode::RECEIVING;
  s_status.phase = EscFwPhase::RECEIVING;
  s_status.progressPermille = 0;
  portEXIT_CRITICAL(&s_mux);
  s_lastRxMs = timeMillis();
  s_hwId = hardwareId;
  USBSerial.printf("ESC FW relay: begin hwId=0x%04X size=%u\n", hardwareId, totalSize);
  return true;
}

int32_t escFlasherRelayWriteChunk(uint32_t offset, const uint8_t* data, uint16_t len) {
  if (!data) return -1;
  int32_t received = -1;
  portENTER_CRITICAL(&s_mux);
  if (s_phase == EscFwPhase::RECEIVING && s_buf && !s_abortRequested &&
      offset <= s_bufSize && (uint64_t)offset + len <= s_bufSize) {
    memcpy(s_buf + offset, data, len);
    // Track contiguous-from-zero progress (the app writes sequentially).
    if (offset <= s_received && offset + len > s_received) {
      s_received = offset + len;
    }
    received = (int32_t)s_received;
    uint16_t permille = s_bufSize ? (uint16_t)((uint64_t)s_received * 500 / s_bufSize) : 0;  // receive = first 50%
    s_status.progressPermille = permille;
  }
  portEXIT_CRITICAL(&s_mux);
  if (received >= 0) s_lastRxMs = timeMillis();
  return received;
}

bool escFlasherRelayEnd() {
  bool ok = false;
  portENTER_CRITICAL(&s_mux);
  if (s_phase == EscFwPhase::RECEIVING && s_buf && s_received >= s_bufSize && s_bufSize >= HEADER_SIZE) {
    // Parse header (little-endian): hwId[0..1], sizeKb[4..5], crc[6..7].
    s_hwId = s_buf[0] | ((uint16_t)s_buf[1] << 8);
    s_sizeKb = s_buf[4] | ((uint16_t)s_buf[5] << 8);
    s_crc = s_buf[6] | ((uint16_t)s_buf[7] << 8);
    uint32_t fwBytes = s_bufSize - HEADER_SIZE;
    s_totalChunks = (uint16_t)((fwBytes + FW_CHUNK_SIZE - 1) / FW_CHUNK_SIZE);
    s_curChunk = 0;
    s_retries = 0;
    s_gotResponse = false;
    s_phase = EscFwPhase::RESTARTING;
    s_status.code = EscFwCode::ENTER_BOOTLDR;
    s_status.phase = EscFwPhase::RESTARTING;
    ok = true;
  }
  portEXIT_CRITICAL(&s_mux);
  if (ok) {
    s_phaseStartMs = timeMillis();
    s_lastSendMs = 0;  // force first restart send on next tick
    USBSerial.printf("ESC FW relay: end, flashing hwId=0x%04X sizeKb=%u chunks=%u\n",
                     s_hwId, s_sizeKb, s_totalChunks);
  } else {
    USBSerial.println("ESC FW relay: end rejected (incomplete image)");
  }
  return ok;
}

void escFlasherRelayAbort() {
  s_abortRequested = true;  // the throttle tick frees and finishes
}

EscFwStatus escFlasherRelayGetStatus() {
  EscFwStatus copy;
  portENTER_CRITICAL(&s_mux);
  copy = s_status;
  portEXIT_CRITICAL(&s_mux);
  return copy;
}

// =============================================================================
// State machine — driven each throttle tick (~20 ms). NEVER blocks.
// =============================================================================
void escFlasherRelayServiceTick() {
  if (!s_node) return;
  if (!escFlasherRelayIsActive()) return;

  if (s_abortRequested) {
    finishFail(EscFwCode::FAILED);
    s_abortRequested = false;
    return;
  }

  const unsigned long now = timeMillis();

  // RECEIVING: no CAN traffic; just guard against an app that stalls.
  if (s_phase == EscFwPhase::RECEIVING) {
    if (now - s_lastRxMs > RX_TIMEOUT_MS) {
      USBSerial.println("ESC FW relay: receive timeout");
      finishFail(EscFwCode::TIMEOUT);
    }
    return;
  }

  // Abort if the device arms mid-flash (defense in depth).
  if (currentState != DISARMED) {
    finishFail(EscFwCode::REJECTED_ARMED);
    return;
  }

  for (int i = 0; i < 6; i++) escAdapter().processTxRxOnce();

  const bool timedOut = (s_lastSendMs != 0) && (now - s_lastSendMs) > FW_TIMEOUT_MS;

  switch (s_phase) {
    // ---------------------------------------------------------------------
    case EscFwPhase::RESTARTING:
      if (s_lastSendMs == 0) {  // first entry: kick the restart
        // The ESC bootloader only accepts the upgrade protocol from host 0x40.
        escAdapter().setLocalNodeId(CONFIG_HOST_NODE_ID);
        sendRestartNode();
        s_lastSendMs = now;
        s_phaseStartMs = now;
        setStatus(EscFwCode::ENTER_BOOTLDR, EscFwPhase::RESTARTING, 0);
        break;
      }
      // Non-blocking reboot wait, then move to bootloader check (the ESC may
      // not answer RestartNode before rebooting — proceed regardless).
      if ((now - s_phaseStartMs) >= REBOOT_WAIT_MS) {
        s_phase = EscFwPhase::CHECKING_BOOT;
        s_retries = 0;
        s_gotResponse = false;
        sendGetBootStatus();
        s_lastSendMs = now;
        setStatus(EscFwCode::ENTER_BOOTLDR, EscFwPhase::CHECKING_BOOT, 0);
      }
      break;

    // ---------------------------------------------------------------------
    case EscFwPhase::CHECKING_BOOT:
      if (s_gotResponse) {
        if (s_bootMode == 2) {
          s_phase = EscFwPhase::STARTING;
          s_retries = 0;
          s_gotResponse = false;
          sendStartUpgrade();
          s_lastSendMs = now;
        } else if (s_retries++ < FW_MAX_RETRIES) {
          // Not in bootloader yet: re-kick a restart and re-probe.
          s_gotResponse = false;
          sendRestartNode();
          s_phase = EscFwPhase::RESTARTING;
          s_phaseStartMs = now;
          s_lastSendMs = now;
        } else {
          finishFail(EscFwCode::ESC_REJECTED);  // won't enter bootloader
        }
      } else if (timedOut) {
        if (s_retries++ < FW_MAX_RETRIES) { s_gotResponse = false; sendGetBootStatus(); s_lastSendMs = now; }
        else finishFail(EscFwCode::TIMEOUT);
      }
      break;

    // ---------------------------------------------------------------------
    case EscFwPhase::STARTING:
      if (s_gotResponse) {
        if (s_responseState == 0) {
          s_phase = EscFwPhase::SENDING;
          s_curChunk = 0;
          s_retries = 0;
          s_gotResponse = false;
          setStatus(EscFwCode::FLASHING, EscFwPhase::SENDING, 500);
          sendFwDataChunk(0);
          s_lastSendMs = now;
        } else {
          finishFail(EscFwCode::ESC_REJECTED);  // image rejected by ESC
        }
      } else if (timedOut) {
        if (s_retries++ < FW_MAX_RETRIES) { s_gotResponse = false; sendStartUpgrade(); s_lastSendMs = now; }
        else finishFail(EscFwCode::TIMEOUT);
      }
      break;

    // ---------------------------------------------------------------------
    case EscFwPhase::SENDING:
      if (s_gotResponse) {
        if (s_responseState == 0) {
          s_curChunk = s_responseNextIndex;  // ESC flow control
          s_retries = 0;
          // Progress: receive was 0..500, flashing is 500..1000.
          uint16_t permille = s_totalChunks
              ? (uint16_t)(500 + (uint32_t)s_curChunk * 500 / s_totalChunks) : 1000;
          setStatus(EscFwCode::FLASHING, EscFwPhase::SENDING, permille);
          if (s_curChunk >= s_totalChunks) {
            s_phase = EscFwPhase::ENDING;
            s_retries = 0;
            s_gotResponse = false;
            sendEndUpgrade();
            s_lastSendMs = now;
          } else {
            s_gotResponse = false;
            sendFwDataChunk(s_curChunk);
            s_lastSendMs = now;
          }
        } else if (s_retries++ < FW_MAX_RETRIES) {
          s_gotResponse = false; sendFwDataChunk(s_curChunk); s_lastSendMs = now;  // resend same chunk
        } else {
          finishFail(EscFwCode::ESC_REJECTED);
        }
      } else if (timedOut) {
        if (s_retries++ < FW_MAX_RETRIES) { s_gotResponse = false; sendFwDataChunk(s_curChunk); s_lastSendMs = now; }
        else finishFail(EscFwCode::TIMEOUT);
      }
      break;

    // ---------------------------------------------------------------------
    case EscFwPhase::ENDING:
      if (s_gotResponse) {
        if (s_responseState == 0) {
          freeBuffer();
          escAdapter().setLocalNodeId(CTRL_NORMAL_NODE_ID);  // restore normal CAN identity
          setStatus(EscFwCode::SUCCESS, EscFwPhase::DONE_OK, 1000);
          s_phase = EscFwPhase::DONE_OK;
          USBSerial.println("ESC FW relay: SUCCESS");
        } else {
          finishFail(EscFwCode::ESC_REJECTED);  // ESC CRC/verify failed
        }
      } else if (timedOut) {
        if (s_retries++ < FW_MAX_RETRIES) { s_gotResponse = false; sendEndUpgrade(); s_lastSendMs = now; }
        else finishFail(EscFwCode::TIMEOUT);
      }
      break;

    default:
      break;
  }
}
