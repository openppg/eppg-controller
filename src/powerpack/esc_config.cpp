#include "powerpack/esc_config.h"
#include "powerpack/config.h"
#include "powerpack/dronecan_param.h"
#include "powerpack/param_table.h"
#include <Arduino.h>
#include <CanardAdapter.h>
#include <dronecan_msgs.h>

extern CanardAdapter canAdapter;

static ConfigProgress s_cfgProgress = {
    .state = ConfigState::IDLE,
    .currentParam = 0,
    .totalParams = PRODUCTION_CONFIG_COUNT,
    .currentParamName = "",
    .statusMsg = "Idle",
    .percentComplete = 0
};

static uint8_t s_getSetTransferId = 0;
static uint8_t s_execOpcodeTransferId = 0;
static uint8_t s_restartTransferId = 0;

static unsigned long s_lastSendMs = 0;
static uint8_t s_retries = 0;

// Response flags
static volatile bool s_gotParamResponse = false;
static volatile bool s_gotOpcodeResponse = false;
static volatile bool s_opcodeOk = false;
static ParamGetSetResponse s_paramResponse;

// =============================================================================
// CanardAdapterNode for config responses
// =============================================================================

class EscConfigNode : public CanardAdapterNode {
public:
    EscConfigNode(CanardAdapter& adapter) : CanardAdapterNode(adapter) {}
    void begin() { _beginNode(); }
    CanardInstance* canard() { return _canard; }

    bool shouldAcceptTransfer(uint64_t* out_data_type_signature,
                              uint16_t data_type_id,
                              CanardTransferType transfer_type,
                              uint8_t source_node_id) override {
        if (transfer_type != CanardTransferTypeResponse || source_node_id != ESC_NODE_ID)
            return false;
        if (data_type_id == UAVCAN_PARAM_GETSET_ID) {
            *out_data_type_signature = UAVCAN_PARAM_GETSET_SIGNATURE;
            return true;
        }
        if (data_type_id == UAVCAN_PARAM_EXECUTEOPCODE_ID) {
            *out_data_type_signature = UAVCAN_PARAM_EXECUTEOPCODE_SIGNATURE;
            return true;
        }
        if (data_type_id == UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_ID) {
            *out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_SIGNATURE;
            return true;
        }
        return false;
    }

    void onTransferReceived(CanardRxTransfer* transfer) override {
        if (transfer->transfer_type != CanardTransferTypeResponse) return;
        if (transfer->data_type_id == UAVCAN_PARAM_GETSET_ID) {
            paramGetSetResponse_decode(transfer, &s_paramResponse);
            s_gotParamResponse = true;
        } else if (transfer->data_type_id == UAVCAN_PARAM_EXECUTEOPCODE_ID) {
            ParamExecuteOpcodeResponse res;
            paramExecuteOpcodeResponse_decode(transfer, &res);
            s_opcodeOk = res.ok;
            s_gotOpcodeResponse = true;
        }
    }
};

static EscConfigNode* s_configNode = nullptr;

// =============================================================================
// Send helpers
// =============================================================================

static void sendParamSet(const char* name, int64_t value) {
    ParamGetSetRequest req = {};
    req.index = 0;
    req.value.tag = PARAM_VALUE_TAG_INTEGER;
    req.value.integer_value = value;
    req.name_len = strlen(name);
    if (req.name_len > 92) req.name_len = 92;
    memcpy(req.name, name, req.name_len);

    uint8_t buffer[128];
    uint32_t len = paramGetSetRequest_encode(&req, buffer);

    CanardTxTransfer transfer = {
        .transfer_type = CanardTransferTypeRequest,
        .data_type_signature = UAVCAN_PARAM_GETSET_SIGNATURE,
        .data_type_id = UAVCAN_PARAM_GETSET_ID,
        .inout_transfer_id = &s_getSetTransferId,
        .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
        .payload = buffer,
        .payload_len = (uint16_t)len
    };
    canardRequestOrRespondObj(s_configNode->canard(), ESC_NODE_ID, &transfer);
}

static void sendSaveToFlash() {
    ParamExecuteOpcodeRequest req = {
        .opcode = PARAM_OPCODE_SAVE,
        .argument = 0
    };

    uint8_t buffer[8];
    uint32_t len = paramExecuteOpcodeRequest_encode(&req, buffer);

    CanardTxTransfer transfer = {
        .transfer_type = CanardTransferTypeRequest,
        .data_type_signature = UAVCAN_PARAM_EXECUTEOPCODE_SIGNATURE,
        .data_type_id = UAVCAN_PARAM_EXECUTEOPCODE_ID,
        .inout_transfer_id = &s_execOpcodeTransferId,
        .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
        .payload = buffer,
        .payload_len = (uint16_t)len
    };
    canardRequestOrRespondObj(s_configNode->canard(), ESC_NODE_ID, &transfer);
}

static void sendRestart() {
    uavcan_protocol_RestartNodeRequest req;
    req.magic_number = UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAGIC_NUMBER;

    uint8_t buffer[UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAX_SIZE];
    uint32_t len = uavcan_protocol_RestartNodeRequest_encode(&req, buffer);

    CanardTxTransfer transfer = {
        .transfer_type = CanardTransferTypeRequest,
        .data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_SIGNATURE,
        .data_type_id = UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_ID,
        .inout_transfer_id = &s_restartTransferId,
        .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
        .payload = buffer,
        .payload_len = (uint16_t)len
    };
    canardRequestOrRespondObj(s_configNode->canard(), ESC_NODE_ID, &transfer);
}

// =============================================================================
// Public API
// =============================================================================

void escConfigInit() {
    if (!s_configNode) {
        s_configNode = new EscConfigNode(canAdapter);
        s_configNode->begin();
    }
}

void escConfigStart() {
    s_cfgProgress.state = ConfigState::WRITING_PARAMS;
    s_cfgProgress.currentParam = 0;
    s_cfgProgress.totalParams = PRODUCTION_CONFIG_COUNT;
    s_cfgProgress.percentComplete = 0;
    s_cfgProgress.statusMsg = "Writing config...";
    s_retries = 0;

    // Send first param
    const ParamEntry& entry = PRODUCTION_CONFIG[0];
    s_cfgProgress.currentParamName = entry.name;
    USBSerial.printf("Config: Writing %s = %lld\n", entry.name, entry.value);
    s_gotParamResponse = false;
    sendParamSet(entry.name, entry.value);
    s_lastSendMs = millis();
}

bool escConfigTick() {
    canAdapter.processTxRxOnce();

    if (s_cfgProgress.state == ConfigState::IDLE ||
        s_cfgProgress.state == ConfigState::SUCCESS ||
        s_cfgProgress.state == ConfigState::FAILED) {
        return false;
    }

    unsigned long now = millis();
    bool timedOut = (now - s_lastSendMs) > FW_UPDATE_TIMEOUT_MS;

    switch (s_cfgProgress.state) {

    case ConfigState::WRITING_PARAMS:
        if (s_gotParamResponse) {
            // Verify response
            if (s_paramResponse.value.tag == PARAM_VALUE_TAG_INTEGER) {
                int64_t expected = PRODUCTION_CONFIG[s_cfgProgress.currentParam].value;
                if (s_paramResponse.value.integer_value != expected) {
                    USBSerial.printf("Config: WARN %s readback=%lld expected=%lld\n",
                        PRODUCTION_CONFIG[s_cfgProgress.currentParam].name,
                        s_paramResponse.value.integer_value, expected);
                }
            }

            s_cfgProgress.currentParam++;
            s_cfgProgress.percentComplete = (uint8_t)((uint32_t)s_cfgProgress.currentParam * 100 / s_cfgProgress.totalParams);
            s_retries = 0;

            if (s_cfgProgress.currentParam >= s_cfgProgress.totalParams) {
                // All params written, save to flash
                USBSerial.println("Config: All params written, saving...");
                s_cfgProgress.state = ConfigState::SAVING;
                s_cfgProgress.statusMsg = "Saving to flash...";
                s_gotOpcodeResponse = false;
                sendSaveToFlash();
                s_lastSendMs = millis();
            } else {
                // Send next param
                const ParamEntry& entry = PRODUCTION_CONFIG[s_cfgProgress.currentParam];
                s_cfgProgress.currentParamName = entry.name;
                USBSerial.printf("Config: Writing %s = %lld\n", entry.name, entry.value);
                s_gotParamResponse = false;
                sendParamSet(entry.name, entry.value);
                s_lastSendMs = millis();
            }
        } else if (timedOut) {
            if (s_retries < FW_UPDATE_MAX_RETRIES) {
                s_retries++;
                s_gotParamResponse = false;
                const ParamEntry& entry = PRODUCTION_CONFIG[s_cfgProgress.currentParam];
                sendParamSet(entry.name, entry.value);
                s_lastSendMs = millis();
            } else {
                USBSerial.printf("Config: Timeout on %s\n",
                    PRODUCTION_CONFIG[s_cfgProgress.currentParam].name);
                s_cfgProgress.state = ConfigState::FAILED;
                s_cfgProgress.statusMsg = "Param write timeout";
            }
        }
        break;

    case ConfigState::SAVING:
        if (s_gotOpcodeResponse) {
            if (s_opcodeOk) {
                USBSerial.println("Config: Saved to flash, restarting...");
                s_cfgProgress.state = ConfigState::RESTARTING;
                s_cfgProgress.statusMsg = "Restarting ESC...";
                sendRestart();
                s_lastSendMs = millis();
                // Give the ESC time to restart
                vTaskDelay(pdMS_TO_TICKS(1000));
                s_cfgProgress.state = ConfigState::SUCCESS;
                s_cfgProgress.statusMsg = "Config applied!";
                s_cfgProgress.percentComplete = 100;
            } else {
                s_cfgProgress.state = ConfigState::FAILED;
                s_cfgProgress.statusMsg = "Save to flash failed";
            }
        } else if (timedOut) {
            if (s_retries < FW_UPDATE_MAX_RETRIES) {
                s_retries++;
                s_gotOpcodeResponse = false;
                sendSaveToFlash();
                s_lastSendMs = millis();
            } else {
                s_cfgProgress.state = ConfigState::FAILED;
                s_cfgProgress.statusMsg = "Save timeout";
            }
        }
        break;

    default:
        break;
    }

    return true;
}

const ConfigProgress& escConfigGetProgress() { return s_cfgProgress; }
bool escConfigSucceeded() { return s_cfgProgress.state == ConfigState::SUCCESS; }
