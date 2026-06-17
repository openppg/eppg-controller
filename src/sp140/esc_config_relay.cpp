#include "sp140/esc_config_relay.h"

#include <Arduino.h>
#include <string.h>
#include <canard.h>
#include <CanardAdapter.h>
#include <dronecan_msgs.h>

#include "sp140/esc.h"            // escAdapter()
#include "sp140/device_state.h"  // DeviceState
#include "sp140/esc_flasher_relay.h"  // escFlasherRelayIsActive() cross-gate
#include "sp140/esc_param_ids.h"      // ESC_PARAM_IDS for read-all

// Device arming state, owned by the device-state machine (same symbol the OTA
// service gates on — see ble/ota_service.cpp). Relay only runs when DISARMED.
extern volatile DeviceState currentState;

// =============================================================================
// SINE / Mad Motors vendor config service definitions
// (ported from powerpack-flash-qc/inc/esc_config_protocol.h — kept local so the
//  controller does not depend on the flash-qc tree)
// =============================================================================
#define SINE_ESC_SETCONFIG_ID         220
#define SINE_ESC_SETCONFIG_SIGNATURE  0x8764693A96F2952EULL
#define SINE_ESC_GETCONFIG_ID         221
#define SINE_ESC_GETCONFIG_SIGNATURE  0x2EEE28C9EB22CF0CULL
#define SINE_ESC_SAVECONFIG_ID        222
#define SINE_ESC_SAVECONFIG_SIGNATURE 0xA3F15F80DDB60005ULL
#define SINE_ESC_PASSWORD_UNLOCK_ID        225
#define SINE_ESC_PASSWORD_UNLOCK_SIGNATURE 0xB970F68E635BAD53ULL

static const uint8_t ESC_CAN_NODE_ID = 0x20;        // ESC's CAN node id (esc.cpp esc.begin(0x20))
static const uint8_t CONFIG_HOST_NODE_ID = 0x40;    // ESC restricts config access to host node 0x40
static const uint8_t CTRL_NORMAL_NODE_ID = 0x01;    // controller's normal CAN node id (esc.cpp LOCAL_NODE_ID)
static const char    ESC_PASSWORD[]  = "123456";    // confirmed via Mad Motors PC-tool capture

// Per-request timing. Each tick (~20 ms) issues at most one request and polls
// for its response across subsequent ticks — never blocking.
static const unsigned long RELAY_TIMEOUT_MS   = 200;     // per-request response wait
// Array/curve params (int16[21]) come back as a multi-frame CAN transfer that
// takes much longer to reassemble than a single-frame scalar response; 200 ms
// is too short and they were getting marked timed-out and skipped during a
// read-all. Give array reads a longer window so all 115 params come back.
static const unsigned long RELAY_ARRAY_TIMEOUT_MS = 1200;
static const uint8_t       RELAY_MAX_RETRIES  = 8;
static const unsigned long REBOOT_WAIT_MS      = 5000;   // ESC reboot before it answers again
static const unsigned long WARMUP_WINDOW_MS    = 9000;   // probe window after reboot wait
static const unsigned long REUNLOCK_WINDOW_MS  = 5000;   // post-reboot unlock retries window
static const unsigned long PROBE_INTERVAL_MS   = 400;
// While reading all params the device is DISARMED, so we can pump the CAN bus
// hard for a slice of each tick (process many params per tick) instead of one
// param per 20 ms control tick — that's the difference between a multi-second
// read and a sub-second one. Bounded so we still yield to the rest of the system.
static const unsigned long READ_TICK_BUDGET_MS = 12;

// True for the SINE int16[21] curve params, which answer over a multi-frame
// transfer and therefore need RELAY_ARRAY_TIMEOUT_MS rather than the scalar wait.
static inline bool relayIsArrayParamId(uint16_t id) {
  return id == 0x0140 || id == 0x0141 ||           // normal acc/dec curves
         id == 0x060E || id == 0x060F || id == 0x0610;  // speed curves
}

// =============================================================================
// Decoded responses (written on the throttle task from the Canard callback,
// read in the same task by the state machine — single-task, so plain statics).
// =============================================================================
struct SetConfigResp { uint16_t recv_config_id; int8_t flag; };
struct GetConfigResp { uint16_t recv_config_id; int8_t flag; uint8_t data[48]; uint8_t data_len; };
struct SaveConfigResp { uint8_t save_success; uint8_t save_error; uint8_t state; };
struct UnlockResp { uint16_t cmd_id; uint8_t flag; };

static volatile bool s_gotSet = false, s_gotGet = false, s_gotSave = false, s_gotUnlock = false;
static SetConfigResp  s_setResp;
static GetConfigResp  s_getResp;
static SaveConfigResp s_saveResp;
static UnlockResp     s_unlockResp;

static uint8_t s_tidSet = 0, s_tidGet = 0, s_tidSave = 0, s_tidUnlock = 0, s_tidRestart = 0;

// =============================================================================
// CanardAdapterNode that captures the ESC's config-service responses.
// Mirrors EscConfigNode in powerpack-flash-qc/src/esc_config.cpp.
// =============================================================================
class EscConfigRelayNode : public CanardAdapterNode {
 public:
  explicit EscConfigRelayNode(CanardAdapter& adapter) : CanardAdapterNode(adapter) {}
  void begin() { _beginNode(); }
  CanardInstance* canard() { return _canard; }

  bool shouldAcceptTransfer(uint64_t* out_signature,
                            uint16_t data_type_id,
                            CanardTransferType transfer_type,
                            uint8_t source_node_id) override {
    if (transfer_type != CanardTransferTypeResponse || source_node_id != ESC_CAN_NODE_ID)
      return false;
    switch (data_type_id) {
      case SINE_ESC_SETCONFIG_ID:      *out_signature = SINE_ESC_SETCONFIG_SIGNATURE;      return true;
      case SINE_ESC_GETCONFIG_ID:      *out_signature = SINE_ESC_GETCONFIG_SIGNATURE;      return true;
      case SINE_ESC_SAVECONFIG_ID:     *out_signature = SINE_ESC_SAVECONFIG_SIGNATURE;     return true;
      case SINE_ESC_PASSWORD_UNLOCK_ID:*out_signature = SINE_ESC_PASSWORD_UNLOCK_SIGNATURE; return true;
      case UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_ID:
        *out_signature = UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_SIGNATURE; return true;
      default: return false;
    }
  }

  void onTransferReceived(CanardRxTransfer* t) override {
    if (t->transfer_type != CanardTransferTypeResponse) return;
    uint32_t ofs = 0;
    switch (t->data_type_id) {
      case SINE_ESC_SETCONFIG_ID: {
        canardDecodeScalar(t, 0, 16, false, &s_setResp.recv_config_id);
        int8_t flag = 0; canardDecodeScalar(t, 16, 8, true, &flag);
        s_setResp.flag = flag; s_gotSet = true;
        break;
      }
      case SINE_ESC_GETCONFIG_ID: {
        canardDecodeScalar(t, 0, 16, false, &s_getResp.recv_config_id);
        int8_t flag = 0; canardDecodeScalar(t, 16, 8, true, &flag);
        s_getResp.flag = flag; ofs = 24;
        uint16_t total_bits = (uint16_t)t->payload_len * 8;
        s_getResp.data_len = 0;
        while (ofs + 8 <= total_bits && s_getResp.data_len < 48) {
          canardDecodeScalar(t, ofs, 8, false, &s_getResp.data[s_getResp.data_len]);
          ofs += 8; s_getResp.data_len++;
        }
        s_gotGet = true;
        break;
      }
      case SINE_ESC_SAVECONFIG_ID: {
        canardDecodeScalar(t, 0, 8, false, &s_saveResp.save_success);
        s_gotSave = true;
        break;
      }
      case SINE_ESC_PASSWORD_UNLOCK_ID: {
        canardDecodeScalar(t, 0, 16, false, &s_unlockResp.cmd_id);
        canardDecodeScalar(t, 16, 8, false, &s_unlockResp.flag);
        s_gotUnlock = true;
        break;
      }
      default: break;
    }
  }
};

static EscConfigRelayNode* s_node = nullptr;

// =============================================================================
// Send helpers (all run on the throttle task)
// =============================================================================
static void sendSetConfig(uint16_t config_id, const uint8_t* data, uint8_t len) {
  uint8_t buffer[64];
  memset(buffer, 0, sizeof(buffer));
  uint32_t bit = 0;
  canardEncodeScalar(buffer, bit, 16, &config_id); bit += 16;
  for (uint8_t i = 0; i < len && i < 48; i++) { canardEncodeScalar(buffer, bit, 8, &data[i]); bit += 8; }
  uint32_t payload_len = (bit + 7) / 8;

  CanardTxTransfer transfer = {
    .transfer_type = CanardTransferTypeRequest,
    .data_type_signature = SINE_ESC_SETCONFIG_SIGNATURE,
    .data_type_id = SINE_ESC_SETCONFIG_ID,
    .inout_transfer_id = &s_tidSet,
    .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
    .payload = buffer,
    .payload_len = (uint16_t)payload_len
  };
  canardRequestOrRespondObj(s_node->canard(), ESC_CAN_NODE_ID, &transfer);
}

static void sendGetConfig(uint16_t config_id) {
  uint8_t buffer[4];
  memset(buffer, 0, sizeof(buffer));
  canardEncodeScalar(buffer, 0, 16, &config_id);

  CanardTxTransfer transfer = {
    .transfer_type = CanardTransferTypeRequest,
    .data_type_signature = SINE_ESC_GETCONFIG_SIGNATURE,
    .data_type_id = SINE_ESC_GETCONFIG_ID,
    .inout_transfer_id = &s_tidGet,
    .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
    .payload = buffer,
    .payload_len = 2
  };
  canardRequestOrRespondObj(s_node->canard(), ESC_CAN_NODE_ID, &transfer);
}

static void sendSaveConfig() {
  uint8_t buffer[4];                 // empty payload, but pass a valid pointer
  memset(buffer, 0, sizeof(buffer));
  CanardTxTransfer transfer = {
    .transfer_type = CanardTransferTypeRequest,
    .data_type_signature = SINE_ESC_SAVECONFIG_SIGNATURE,
    .data_type_id = SINE_ESC_SAVECONFIG_ID,
    .inout_transfer_id = &s_tidSave,
    .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
    .payload = buffer,
    .payload_len = 0
  };
  canardRequestOrRespondObj(s_node->canard(), ESC_CAN_NODE_ID, &transfer);
}

static void sendPasswordUnlock() {
  uint8_t buffer[64];
  memset(buffer, 0, sizeof(buffer));
  uint16_t cmd_id = 0x0001;
  uint32_t bit = 0;
  canardEncodeScalar(buffer, bit, 16, &cmd_id); bit += 16;
  uint8_t pwlen = (uint8_t)strlen(ESC_PASSWORD);
  uint8_t pad = (pwlen < 10) ? 10 : pwlen;     // zero-pad to 10 bytes (matches PC tool)
  for (uint8_t i = 0; i < pad; i++) {
    uint8_t c = (i < pwlen) ? (uint8_t)ESC_PASSWORD[i] : 0;
    canardEncodeScalar(buffer, bit, 8, &c); bit += 8;
  }
  uint32_t payload_len = (bit + 7) / 8;

  CanardTxTransfer transfer = {
    .transfer_type = CanardTransferTypeRequest,
    .data_type_signature = SINE_ESC_PASSWORD_UNLOCK_SIGNATURE,
    .data_type_id = SINE_ESC_PASSWORD_UNLOCK_ID,
    .inout_transfer_id = &s_tidUnlock,
    .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
    .payload = buffer,
    .payload_len = (uint16_t)payload_len
  };
  canardRequestOrRespondObj(s_node->canard(), ESC_CAN_NODE_ID, &transfer);
}

static void sendRestart() {
  uavcan_protocol_RestartNodeRequest req;
  req.magic_number = UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAGIC_NUMBER;  // 0xACCE551B1E
  uint8_t buffer[UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAX_SIZE];
  memset(buffer, 0, sizeof(buffer));
  uint32_t len = uavcan_protocol_RestartNodeRequest_encode(&req, buffer);

  CanardTxTransfer transfer = {
    .transfer_type = CanardTransferTypeRequest,
    .data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_SIGNATURE,
    .data_type_id = UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_ID,
    .inout_transfer_id = &s_tidRestart,
    .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
    .payload = buffer,
    .payload_len = (uint16_t)len
  };
  canardRequestOrRespondObj(s_node->canard(), ESC_CAN_NODE_ID, &transfer);
}

// =============================================================================
// Session state
// =============================================================================
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// Pending request handoff (written by BLE task, consumed by throttle task).
static volatile bool s_reqPending = false;
static uint16_t s_reqId = 0;
static uint8_t  s_reqData[48];
static uint8_t  s_reqLen = 0;

// Default ON: the ESC requires host node 0x40 + password unlock for config
// access (confirmed on hardware — node 0x01 without unlock is ignored).
static volatile bool s_requireUnlock = true;

static EscRelayPhase s_phase = EscRelayPhase::IDLE;
static EscRelayStatus s_status = {};

// Active session working set.
static uint16_t s_targetId = 0;
static uint8_t  s_expData[48];
static uint8_t  s_expLen = 0;
static unsigned long s_lastSendMs = 0;
static unsigned long s_phaseStartMs = 0;
static unsigned long s_lastProbeMs = 0;
static uint8_t  s_retries = 0;
static bool     s_reUnlocked = false;

// Read-all session state.
static volatile bool s_readReqPending = false;
static bool     s_readMode = false;
static uint16_t s_readIndex = 0;
static uint8_t  s_resultBlob[3072];   // tuples: [config_id u16][flag u8][len u8][data]
static uint16_t s_resultLen = 0;

// Telemetry-throttle settle window: after a session ends, keep telemetry notify
// suppressed for this long so the app can poll the final status and fetch the
// result blob over a quiet link (telemetry at ~66 Hz otherwise starves reads).
static volatile unsigned long s_terminalAtMs = 0;
static volatile bool s_hadSession = false;
static const unsigned long RESULT_SETTLE_MS = 9000;

static void setStatus(EscRelayStatusCode code, EscRelayPhase phase, uint8_t detail) {
  portENTER_CRITICAL(&s_mux);
  s_status.code = code;
  s_status.phase = phase;
  s_status.config_id = s_targetId;
  s_status.detail = detail;
  portEXIT_CRITICAL(&s_mux);
}

static void finishOk(const uint8_t* readback, uint8_t len) {
  USBSerial.printf("[ESCWR] VERIFIED_OK target=0x%04X readbackLen=%d\n",
                   s_targetId, (int)len);  // TEMP diag
  portENTER_CRITICAL(&s_mux);
  s_status.code = EscRelayStatusCode::VERIFIED_OK;
  s_status.phase = EscRelayPhase::DONE_OK;
  s_status.config_id = s_targetId;
  s_status.detail = 0;
  s_status.readback_len = (len > 8) ? 8 : len;
  for (uint8_t i = 0; i < s_status.readback_len; i++) s_status.readback[i] = readback[i];
  portEXIT_CRITICAL(&s_mux);
  escAdapter().setLocalNodeId(CTRL_NORMAL_NODE_ID);  // restore normal CAN identity
  s_terminalAtMs = millis();
  s_hadSession = true;  // open the telemetry-throttle settle window for read-back
  s_phase = EscRelayPhase::DONE_OK;
}

static void finishFail(EscRelayStatusCode code, uint8_t detail) {
  USBSerial.printf("[ESCWR] FAIL code=0x%02X atPhase=%d target=0x%04X reUnlocked=%d\n",
                   (int)code, (int)s_phase, s_targetId, (int)s_reUnlocked);  // TEMP diag
  setStatus(code, EscRelayPhase::DONE_FAIL, detail);
  escAdapter().setLocalNodeId(CTRL_NORMAL_NODE_ID);  // restore normal CAN identity
  s_terminalAtMs = millis();
  s_hadSession = true;  // open the telemetry-throttle settle window for read-back
  s_phase = EscRelayPhase::DONE_FAIL;
}

static bool dataMatches(const uint8_t* got, uint8_t gotLen, const uint8_t* exp, uint8_t expLen) {
  if (gotLen < expLen) return false;
  for (uint8_t i = 0; i < expLen; i++) if (got[i] != exp[i]) return false;
  return true;
}

// ---- read-all helpers -------------------------------------------------------
static void readStoreTuple(uint16_t id, uint8_t flag, const uint8_t* data, uint8_t dlen) {
  if ((uint32_t)s_resultLen + 4 + dlen > sizeof(s_resultBlob)) return;  // drop if full
  s_resultBlob[s_resultLen++] = id & 0xFF;
  s_resultBlob[s_resultLen++] = (id >> 8) & 0xFF;
  s_resultBlob[s_resultLen++] = flag;
  s_resultBlob[s_resultLen++] = dlen;
  if (dlen && data) { memcpy(&s_resultBlob[s_resultLen], data, dlen); s_resultLen += dlen; }
}

static void finishReadDone() {
  portENTER_CRITICAL(&s_mux);
  s_status.code = EscRelayStatusCode::READ_DONE;
  s_status.phase = EscRelayPhase::DONE_OK;
  s_status.config_id = s_resultLen;  // blob length for the app to fetch
  s_status.detail = 100;
  portEXIT_CRITICAL(&s_mux);
  escAdapter().setLocalNodeId(CTRL_NORMAL_NODE_ID);  // restore normal CAN identity
  s_terminalAtMs = millis();
  s_hadSession = true;  // open the telemetry-throttle settle window for read-back
  s_phase = EscRelayPhase::DONE_OK;
}

static void readAdvance(unsigned long now) {
  s_readIndex++;
  s_retries = 0;
  s_gotGet = false;
  if (s_readIndex >= ESC_PARAM_IDS_COUNT) { finishReadDone(); return; }
  uint8_t pct = (uint8_t)((uint32_t)s_readIndex * 100 / ESC_PARAM_IDS_COUNT);
  setStatus(EscRelayStatusCode::READING, EscRelayPhase::READING, pct);
  sendGetConfig(ESC_PARAM_IDS[s_readIndex]);
  s_lastSendMs = now;
}

static void beginReadSession() {
  escAdapter().setLocalNodeId(CONFIG_HOST_NODE_ID);
  s_readMode = true;
  s_readIndex = 0;
  s_resultLen = 0;
  s_retries = 0;
  s_reUnlocked = false;
  s_gotSet = s_gotGet = s_gotSave = s_gotUnlock = false;
  if (s_requireUnlock) {
    s_phase = EscRelayPhase::UNLOCK;
    setStatus(EscRelayStatusCode::READING, EscRelayPhase::UNLOCK, 0);
    sendPasswordUnlock();
  } else {
    s_phase = EscRelayPhase::READING;
    setStatus(EscRelayStatusCode::READING, EscRelayPhase::READING, 0);
    sendGetConfig(ESC_PARAM_IDS[0]);
  }
  s_lastSendMs = millis();
}

static void beginSession(uint16_t id, const uint8_t* data, uint8_t len) {
  // The ESC restricts config access to host node 0x40 — present as 0x40 for the
  // session so the ESC accepts our requests and addresses its responses to us.
  // Restored to 0x01 on every exit path (finishOk / finishFail).
  escAdapter().setLocalNodeId(CONFIG_HOST_NODE_ID);

  s_readMode = false;
  s_targetId = id;
  s_expLen = (len > 48) ? 48 : len;
  memcpy(s_expData, data, s_expLen);
  s_retries = 0;
  s_reUnlocked = false;
  s_gotSet = s_gotGet = s_gotSave = s_gotUnlock = false;

  if (s_requireUnlock) {
    s_phase = EscRelayPhase::UNLOCK;
    setStatus(EscRelayStatusCode::RUNNING, EscRelayPhase::UNLOCK, 0);
    sendPasswordUnlock();
  } else {
    s_phase = EscRelayPhase::WRITE;
    setStatus(EscRelayStatusCode::RUNNING, EscRelayPhase::WRITE, 0);
    sendSetConfig(s_targetId, s_expData, s_expLen);
  }
  s_lastSendMs = millis();
}

// =============================================================================
// Public API
// =============================================================================
void escConfigRelayInit() {
  if (!s_node) {
    s_node = new EscConfigRelayNode(escAdapter());
    s_node->begin();
  }
  s_phase = EscRelayPhase::IDLE;
  s_status = {};
  s_status.code = EscRelayStatusCode::IDLE;
  s_status.phase = EscRelayPhase::IDLE;
}

void escConfigRelaySetRequireUnlock(bool require) { s_requireUnlock = require; }

bool escConfigRelayIsActive() {
  if (s_reqPending || s_readReqPending) return true;
  return s_phase != EscRelayPhase::IDLE &&
         s_phase != EscRelayPhase::DONE_OK &&
         s_phase != EscRelayPhase::DONE_FAIL;
}

bool escConfigRelayResultPending() {
  if (escConfigRelayIsActive()) return true;
  if (!s_hadSession) return false;
  return (millis() - s_terminalAtMs) < RESULT_SETTLE_MS;
}

bool escConfigRelayRequestSetParam(uint16_t config_id, const uint8_t* data, uint8_t len) {
  if (!s_node || len > 48 || data == nullptr) return false;
  if (escConfigRelayIsActive() || escFlasherRelayIsActive()) return false;

  portENTER_CRITICAL(&s_mux);
  s_reqId = config_id;
  s_reqLen = len;
  memcpy((void*)s_reqData, data, len);
  s_reqPending = true;
  s_status.code = EscRelayStatusCode::ACCEPTED;
  s_status.config_id = config_id;
  portEXIT_CRITICAL(&s_mux);
  return true;
}

bool escConfigRelayRequestReverseDirection(bool reversed) {
  uint16_t v = reversed ? 1 : 0;
  uint8_t d[2] = { (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF) };
  return escConfigRelayRequestSetParam(ESC_PARAM_DIRECTION, d, 2);
}

bool escConfigRelayRequestReadAll() {
  if (!s_node) return false;
  if (escConfigRelayIsActive() || escFlasherRelayIsActive()) return false;
  portENTER_CRITICAL(&s_mux);
  s_readReqPending = true;
  s_status.code = EscRelayStatusCode::ACCEPTED;
  portEXIT_CRITICAL(&s_mux);
  return true;
}

uint16_t escConfigRelayResultLen() { return s_resultLen; }

uint16_t escConfigRelayReadResult(uint32_t offset, uint8_t* out, uint16_t maxLen) {
  if (!out || offset >= s_resultLen) return 0;
  uint16_t avail = s_resultLen - (uint16_t)offset;
  uint16_t n = (avail < maxLen) ? avail : maxLen;
  memcpy(out, &s_resultBlob[offset], n);
  return n;
}

EscRelayStatus escConfigRelayGetStatus() {
  EscRelayStatus copy;
  portENTER_CRITICAL(&s_mux);
  copy = s_status;
  portEXIT_CRITICAL(&s_mux);
  return copy;
}

// =============================================================================
// State machine — driven each throttle tick (~20 ms). NEVER blocks.
// =============================================================================
void escConfigRelayServiceTick() {
  if (!s_node) return;

  const bool terminal = (s_phase == EscRelayPhase::IDLE ||
                         s_phase == EscRelayPhase::DONE_OK ||
                         s_phase == EscRelayPhase::DONE_FAIL);
  // Nothing to do: don't touch the CAN bus when fully idle (keeps the existing
  // telemetry cadence undisturbed).
  if (terminal && !s_reqPending && !s_readReqPending) return;

  // Pump CAN so responses to our requests are captured between ticks.
  for (int i = 0; i < 6; i++) escAdapter().processTxRxOnce();

  // --- pick up a new request only when idle/terminal -----------------------
  if (terminal) {
    bool pending = false, readPending = false;
    uint16_t id = 0; uint8_t data[48]; uint8_t len = 0;
    portENTER_CRITICAL(&s_mux);
    if (s_reqPending) {
      pending = true; id = s_reqId; len = s_reqLen;
      memcpy(data, (const void*)s_reqData, len);
      s_reqPending = false;
    } else if (s_readReqPending) {
      readPending = true;
      s_readReqPending = false;
    }
    portEXIT_CRITICAL(&s_mux);
    if (!pending && !readPending) return;

    // Safety gate (defense in depth — the BLE handler also checks).
    if (currentState != DISARMED) {
      if (pending) s_targetId = id;
      finishFail(EscRelayStatusCode::REJECTED_ARMED, 0);
      return;
    }
    if (pending) beginSession(id, data, len);
    else beginReadSession();
    return;
  }

  // Defense in depth: if the device arms while a session is in flight, abort
  // immediately and stop driving the CAN bus. (The param may already be saved,
  // but we will not continue restart/verify traffic once armed.)
  if (currentState != DISARMED) {
    finishFail(EscRelayStatusCode::REJECTED_ARMED, 0);
    return;
  }

  const unsigned long now = millis();
  // Array/curve params answer over a slower multi-frame transfer — give them a
  // longer per-request window while reading; everything else uses the scalar wait.
  unsigned long timeoutWindow = RELAY_TIMEOUT_MS;
  if (s_phase == EscRelayPhase::READING && s_readIndex < ESC_PARAM_IDS_COUNT &&
      relayIsArrayParamId(ESC_PARAM_IDS[s_readIndex])) {
    timeoutWindow = RELAY_ARRAY_TIMEOUT_MS;
  }
  const bool timedOut = (now - s_lastSendMs) > timeoutWindow;

  switch (s_phase) {
    // ---------------------------------------------------------------------
    case EscRelayPhase::UNLOCK:
      if (s_gotUnlock) {
        s_retries = 0;
        if (s_readMode) {
          s_phase = EscRelayPhase::READING;
          setStatus(EscRelayStatusCode::READING, EscRelayPhase::READING, 0);
          s_gotGet = false;
          sendGetConfig(ESC_PARAM_IDS[0]);
        } else {
          s_phase = EscRelayPhase::WRITE;
          setStatus(EscRelayStatusCode::RUNNING, EscRelayPhase::WRITE, 0);
          s_gotSet = false;
          sendSetConfig(s_targetId, s_expData, s_expLen);
        }
        s_lastSendMs = now;
      } else if (timedOut) {
        if (s_retries++ < RELAY_MAX_RETRIES) { s_gotUnlock = false; sendPasswordUnlock(); s_lastSendMs = now; }
        else { finishFail(EscRelayStatusCode::TIMEOUT, 0); }
      }
      break;

    // ---------------------------------------------------------------------
    // READING: read-all — GetConfig each param into the result blob.
    case EscRelayPhase::READING: {
      // Process as many params as fit in this tick's budget (disarmed → safe to
      // busy-pump the bus). We still send one GetConfig at a time and wait for
      // its response, but with short sub-tick waits instead of a full 20 ms tick.
      const unsigned long deadline = now + READ_TICK_BUDGET_MS;
      for (;;) {
        if (s_gotGet) {
          readStoreTuple(ESC_PARAM_IDS[s_readIndex], (uint8_t)s_getResp.flag,
                         s_getResp.data, s_getResp.data_len);
          readAdvance(millis());                          // store + send next GetConfig
          if (s_phase != EscRelayPhase::READING) break;   // finished (node restored)
          escAdapter().processTxRxOnce();                 // push the next request out
        } else {
          const unsigned long win = relayIsArrayParamId(ESC_PARAM_IDS[s_readIndex])
                                        ? RELAY_ARRAY_TIMEOUT_MS : RELAY_TIMEOUT_MS;
          if ((millis() - s_lastSendMs) > win) {
            if (s_retries++ < RELAY_MAX_RETRIES) {
              s_gotGet = false; sendGetConfig(ESC_PARAM_IDS[s_readIndex]); s_lastSendMs = millis();
            } else {
              readStoreTuple(ESC_PARAM_IDS[s_readIndex], 0xFF, nullptr, 0);  // timed out — skip
              readAdvance(millis());
              if (s_phase != EscRelayPhase::READING) break;
            }
          }
          escAdapter().processTxRxOnce();                 // drain RX — response may arrive
          delayMicroseconds(150);
        }
        if (millis() >= deadline) break;                  // yield; resume next tick
      }
      break;
    }

    // ---------------------------------------------------------------------
    case EscRelayPhase::WRITE:
      if (s_gotSet) {
        int8_t flag = s_setResp.flag;
        bool ok = (s_setResp.recv_config_id == s_targetId) && (flag == 0 || flag == 6 || flag == 7);
        USBSerial.printf("[ESCWR] SET recv=0x%04X flag=%d ok=%d\n",
                         s_setResp.recv_config_id, (int)flag, (int)ok);  // TEMP diag
        if (ok) {
          s_phase = EscRelayPhase::SAVE;
          setStatus(EscRelayStatusCode::RUNNING, EscRelayPhase::SAVE, 0);
          s_retries = 0;
          s_gotSave = false;
          sendSaveConfig();
          s_lastSendMs = now;
        } else if (s_retries++ < RELAY_MAX_RETRIES) {
          s_gotSet = false; sendSetConfig(s_targetId, s_expData, s_expLen); s_lastSendMs = now;
        } else {
          finishFail(EscRelayStatusCode::ESC_FLAG_ERROR, (uint8_t)flag);
        }
      } else if (timedOut) {
        if (s_retries++ < RELAY_MAX_RETRIES) { s_gotSet = false; sendSetConfig(s_targetId, s_expData, s_expLen); s_lastSendMs = now; }
        else finishFail(EscRelayStatusCode::TIMEOUT, 0);
      }
      break;

    // ---------------------------------------------------------------------
    case EscRelayPhase::SAVE:
      if (s_gotSave) {
        USBSerial.printf("[ESCWR] SAVE success=%d -> restart\n", s_saveResp.save_success);  // TEMP diag
        if (s_saveResp.save_success == 0) {
          s_phase = EscRelayPhase::RESTART;
          // fall through to RESTART on the next tick after sending below
          setStatus(EscRelayStatusCode::REBOOTING, EscRelayPhase::RESTART, 0);
          sendRestart();
          for (int i = 0; i < 10; i++) escAdapter().processTxRxOnce();  // flush, non-blocking
          s_phase = EscRelayPhase::WAIT_REBOOT;
          s_phaseStartMs = now;
          s_lastProbeMs = 0;
        } else {
          finishFail(EscRelayStatusCode::FAILED, s_saveResp.save_success);
        }
      } else if (timedOut) {
        if (s_retries++ < RELAY_MAX_RETRIES) { s_gotSave = false; sendSaveConfig(); s_lastSendMs = now; }
        else finishFail(EscRelayStatusCode::TIMEOUT, 0);
      }
      break;

    // ---------------------------------------------------------------------
    // WAIT_REBOOT: non-blocking. Wait for the ESC to reboot, (optionally)
    // re-unlock, then probe with GetConfig until it answers; that answer IS
    // the verification.
    // ---------------------------------------------------------------------
    case EscRelayPhase::WAIT_REBOOT: {
      const unsigned long elapsed = now - s_phaseStartMs;

      // Phase A: give the ESC time to reboot before it answers.
      if (elapsed < REBOOT_WAIT_MS) {
        setStatus(EscRelayStatusCode::REBOOTING, EscRelayPhase::WAIT_REBOOT, 0);
        break;
      }

      // Phase B: re-unlock if required (password state is lost on reboot).
      if (s_requireUnlock && !s_reUnlocked) {
        if (now - s_lastSendMs > 500) {
          s_gotUnlock = false; sendPasswordUnlock(); s_lastSendMs = now;
          USBSerial.printf("[ESCWR] reboot+%lums: re-unlock sent\n", elapsed);  // TEMP diag
        }
        if (s_gotUnlock) { s_reUnlocked = true; USBSerial.println("[ESCWR] re-unlock OK"); }  // TEMP diag
        if (elapsed > REBOOT_WAIT_MS + REUNLOCK_WINDOW_MS) {
          finishFail(EscRelayStatusCode::TIMEOUT, 0);  // post-reboot unlock failed
        }
        break;
      }

      // Phase C: probe / verify.
      setStatus(EscRelayStatusCode::RUNNING, EscRelayPhase::VERIFY, 0);
      if (now - s_lastProbeMs > PROBE_INTERVAL_MS) {
        s_gotGet = false;
        sendGetConfig(s_targetId);
        s_lastProbeMs = now;
        USBSerial.printf("[ESCWR] reboot+%lums: verify probe sent\n", elapsed);  // TEMP diag
      }
      if (s_gotGet) {  // TEMP diag
        USBSerial.printf("[ESCWR] verify resp recv=0x%04X flag=%d len=%d\n",
                         s_getResp.recv_config_id, (int)s_getResp.flag, s_getResp.data_len);
      }
      if (s_gotGet && s_getResp.recv_config_id == s_targetId && s_getResp.flag == 0) {
        if (dataMatches(s_getResp.data, s_getResp.data_len, s_expData, s_expLen)) {
          finishOk(s_getResp.data, s_getResp.data_len);
        } else {
          finishFail(EscRelayStatusCode::VERIFY_MISMATCH, 0);
        }
        break;
      }
      if (elapsed > REBOOT_WAIT_MS + WARMUP_WINDOW_MS) {
        finishFail(EscRelayStatusCode::TIMEOUT, 0);  // ESC never answered after reboot
      }
      break;
    }

    default:
      break;
  }
}
