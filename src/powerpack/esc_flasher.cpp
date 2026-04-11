#include "powerpack/esc_flasher.h"
#include "powerpack/config.h"
#include <Arduino.h>
#include <CanardAdapter.h>
#include <dronecan_msgs.h>

// Access the shared CAN adapter (defined in main.cpp)
extern CanardAdapter canAdapter;

static FlashProgress s_progress = {
    .state = FlashState::IDLE,
    .currentChunk = 0,
    .totalChunks = 0,
    .retryCount = 0,
    .statusMsg = "Idle",
    .percentComplete = 0
};

// Firmware file data
static const uint8_t* s_fwData = nullptr;
static uint32_t s_fwLen = 0;

// Parsed header
static uint16_t s_hwId = 0;
static uint16_t s_fileSizeKb = 0;
static uint16_t s_fileCrc = 0;

// Timing
static unsigned long s_lastSendMs = 0;
static uint8_t s_retries = 0;

// Transfer IDs for each service
static uint8_t s_restartTransferId = 0;
static uint8_t s_bootStatusTransferId = 0;
static uint8_t s_startUpgradeTransferId = 0;
static uint8_t s_sendFwDataTransferId = 0;
static uint8_t s_endUpgradeTransferId = 0;

// State machine flags - set when response received
static volatile bool s_gotResponse = false;
static volatile uint8_t s_responseState = 0xFF;
static volatile uint16_t s_responseNextIndex = 0;
static volatile uint8_t s_bootMode = 0;

// Forward declarations for response handlers
static bool handleBootStatusResponse(CanardRxTransfer* transfer);
static bool handleStartUpgradeResponse(CanardRxTransfer* transfer);
static bool handleSendFwDataResponse(CanardRxTransfer* transfer);
static bool handleEndUpgradeResponse(CanardRxTransfer* transfer);
static bool handleRestartResponse(CanardRxTransfer* transfer);

// =============================================================================
// CanardAdapterNode subclass to handle firmware update responses
// =============================================================================

class EscFlasherNode : public CanardAdapterNode {
public:
    EscFlasherNode(CanardAdapter& adapter) : CanardAdapterNode(adapter) {}

    void begin() { _beginNode(); }

    // Expose _canard for sending from free functions
    CanardInstance* canard() { return _canard; }

    bool shouldAcceptTransfer(uint64_t* out_data_type_signature,
                              uint16_t data_type_id,
                              CanardTransferType transfer_type,
                              uint8_t source_node_id) override {
        if (transfer_type != CanardTransferTypeResponse || source_node_id != ESC_NODE_ID) {
            return false;
        }
        switch (data_type_id) {
            case SINE_ESC_GETBOOTSTATUS_ID:
                *out_data_type_signature = SINE_ESC_GETBOOTSTATUS_SIGNATURE;
                return true;
            case SINE_ESC_STARTFWUPGRADE_ID:
                *out_data_type_signature = SINE_ESC_STARTFWUPGRADE_SIGNATURE;
                return true;
            case SINE_ESC_SENDFWDATA_RESPONSE_ID:
                *out_data_type_signature = SINE_ESC_SENDFWDATA_RESPONSE_SIGNATURE;
                return true;
            case SINE_ESC_ENDFWUPGRADE_ID:
                *out_data_type_signature = SINE_ESC_ENDFWUPGRADE_SIGNATURE;
                return true;
            case UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_ID:
                *out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_SIGNATURE;
                return true;
            default:
                return false;
        }
    }

    void onTransferReceived(CanardRxTransfer* transfer) override {
        if (transfer->transfer_type != CanardTransferTypeResponse) return;
        switch (transfer->data_type_id) {
            case SINE_ESC_GETBOOTSTATUS_ID:
                handleBootStatusResponse(transfer);
                break;
            case SINE_ESC_STARTFWUPGRADE_ID:
                handleStartUpgradeResponse(transfer);
                break;
            case SINE_ESC_SENDFWDATA_RESPONSE_ID:
                handleSendFwDataResponse(transfer);
                break;
            case SINE_ESC_ENDFWUPGRADE_ID:
                handleEndUpgradeResponse(transfer);
                break;
            case UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_ID:
                handleRestartResponse(transfer);
                break;
        }
    }
};

static EscFlasherNode* s_flasherNode = nullptr;

// =============================================================================
// Response handlers
// =============================================================================

static bool handleBootStatusResponse(CanardRxTransfer* transfer) {
    sine_esc_GetBootStatusResponse res;
    if (!sine_esc_GetBootStatusResponse_decode(transfer, &res)) {
        s_bootMode = res.mode;
        s_gotResponse = true;
        return true;
    }
    return false;
}

static bool handleStartUpgradeResponse(CanardRxTransfer* transfer) {
    sine_esc_StartFwUpgradeResponse res;
    if (!sine_esc_StartFwUpgradeResponse_decode(transfer, &res)) {
        s_responseState = res.state;
        s_gotResponse = true;
        return true;
    }
    return false;
}

static bool handleSendFwDataResponse(CanardRxTransfer* transfer) {
    sine_esc_SendFwDataResponse res;
    if (!sine_esc_SendFwDataResponse_decode(transfer, &res)) {
        s_responseState = res.state;
        s_responseNextIndex = res.next_index;
        s_gotResponse = true;
        return true;
    }
    return false;
}

static bool handleEndUpgradeResponse(CanardRxTransfer* transfer) {
    sine_esc_EndFwUpgradeResponse res;
    if (!sine_esc_EndFwUpgradeResponse_decode(transfer, &res)) {
        s_responseState = res.state;
        s_gotResponse = true;
        return true;
    }
    return false;
}

static bool handleRestartResponse(CanardRxTransfer* transfer) {
    uavcan_protocol_RestartNodeResponse res;
    if (!uavcan_protocol_RestartNodeResponse_decode(transfer, &res)) {
        s_responseState = res.ok ? 0 : 1;
        s_gotResponse = true;
        return true;
    }
    return false;
}

// =============================================================================
// Send helpers
// =============================================================================

static void sendRestartNode() {
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
    canardRequestOrRespondObj(s_flasherNode->canard(), ESC_NODE_ID, &transfer);
}

static void sendGetBootStatus() {
    sine_esc_GetBootStatusRequest req;
    uint8_t buffer[SINE_ESC_GETBOOTSTATUS_REQUEST_MAX_SIZE];
    uint32_t len = sine_esc_GetBootStatusRequest_encode(&req, buffer);

    CanardTxTransfer transfer = {
        .transfer_type = CanardTransferTypeRequest,
        .data_type_signature = SINE_ESC_GETBOOTSTATUS_REQUEST_SIGNATURE,
        .data_type_id = SINE_ESC_GETBOOTSTATUS_REQUEST_ID,
        .inout_transfer_id = &s_bootStatusTransferId,
        .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
        .payload = buffer,
        .payload_len = (uint16_t)len
    };
    canardRequestOrRespondObj(s_flasherNode->canard(), ESC_NODE_ID, &transfer);
}

static void sendStartUpgrade() {
    sine_esc_StartFwUpgradeRequest req = {
        .hardware_id = s_hwId,
        .filse_size_kb = s_fileSizeKb,
        .file_crc = s_fileCrc
    };

    uint8_t buffer[SINE_ESC_STARTFWUPGRADE_REQUEST_MAX_SIZE];
    uint32_t len = sine_esc_StartFwUpgradeRequest_encode(&req, buffer);

    CanardTxTransfer transfer = {
        .transfer_type = CanardTransferTypeRequest,
        .data_type_signature = SINE_ESC_STARTFWUPGRADE_REQUEST_SIGNATURE,
        .data_type_id = SINE_ESC_STARTFWUPGRADE_REQUEST_ID,
        .inout_transfer_id = &s_startUpgradeTransferId,
        .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
        .payload = buffer,
        .payload_len = (uint16_t)len
    };
    canardRequestOrRespondObj(s_flasherNode->canard(), ESC_NODE_ID, &transfer);
}

static void sendFwDataChunk(uint16_t index) {
    sine_esc_SendFwDataRequest req;
    req.index = index;

    // Firmware data starts at byte 32 of the file
    uint32_t fwDataOffset = 32 + (uint32_t)index * FW_CHUNK_SIZE;
    uint32_t fwDataLen = s_fwLen - 32;  // Total firmware data bytes (excluding header)

    // Fill the chunk
    memset(req.data, 0xFF, FW_CHUNK_SIZE);
    if (fwDataOffset < s_fwLen) {
        uint32_t remaining = s_fwLen - fwDataOffset;
        uint32_t copyLen = (remaining < FW_CHUNK_SIZE) ? remaining : FW_CHUNK_SIZE;
        memcpy(req.data, s_fwData + fwDataOffset, copyLen);
    }

    uint8_t buffer[SINE_ESC_SENDFWDATA_REQUEST_MAX_SIZE];
    uint32_t len = sine_esc_SendFwDataRequest_encode(&req, buffer);

    CanardTxTransfer transfer = {
        .transfer_type = CanardTransferTypeRequest,
        .data_type_signature = SINE_ESC_SENDFWDATA_REQUEST_SIGNATURE,
        .data_type_id = SINE_ESC_SENDFWDATA_REQUEST_ID,
        .inout_transfer_id = &s_sendFwDataTransferId,
        .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
        .payload = buffer,
        .payload_len = (uint16_t)len
    };
    canardRequestOrRespondObj(s_flasherNode->canard(), ESC_NODE_ID, &transfer);
}

static void sendEndUpgrade() {
    sine_esc_EndFwUpgradeRequest req;
    uint8_t buffer[SINE_ESC_ENDFWUPGRADE_REQUEST_MAX_SIZE];
    uint32_t len = sine_esc_EndFwUpgradeRequest_encode(&req, buffer);

    CanardTxTransfer transfer = {
        .transfer_type = CanardTransferTypeRequest,
        .data_type_signature = SINE_ESC_ENDFWUPGRADE_REQUEST_SIGNATURE,
        .data_type_id = SINE_ESC_ENDFWUPGRADE_REQUEST_ID,
        .inout_transfer_id = &s_endUpgradeTransferId,
        .priority = CANARD_TRANSFER_PRIORITY_HIGHEST,
        .payload = buffer,
        .payload_len = (uint16_t)len
    };
    canardRequestOrRespondObj(s_flasherNode->canard(), ESC_NODE_ID, &transfer);
}

// =============================================================================
// Public API
// =============================================================================

void escFlasherInit() {
    if (!s_flasherNode) {
        s_flasherNode = new EscFlasherNode(canAdapter);
        s_flasherNode->begin();
    }
}

void escFlasherStart(const uint8_t* fwData, uint32_t fwLen) {
    if (fwLen < 32) {
        s_progress.state = FlashState::FAILED;
        s_progress.statusMsg = "FW file too small";
        return;
    }

    s_fwData = fwData;
    s_fwLen = fwLen;

    // Parse header (little-endian)
    s_hwId = fwData[0] | ((uint16_t)fwData[1] << 8);
    // bytes 2-3 not used
    s_fileSizeKb = fwData[4] | ((uint16_t)fwData[5] << 8);
    s_fileCrc = fwData[6] | ((uint16_t)fwData[7] << 8);

    uint32_t fwDataBytes = fwLen - 32;
    s_progress.totalChunks = (fwDataBytes + FW_CHUNK_SIZE - 1) / FW_CHUNK_SIZE;
    s_progress.currentChunk = 0;
    s_progress.retryCount = 0;
    s_progress.percentComplete = 0;

    USBSerial.printf("FW Flash: hwId=0x%04X sizeKb=%u crc=0x%04X chunks=%u\n",
                     s_hwId, s_fileSizeKb, s_fileCrc, s_progress.totalChunks);

    // Start by restarting the ESC
    s_progress.state = FlashState::RESTARTING;
    s_progress.statusMsg = "Restarting ESC...";
    s_gotResponse = false;
    s_retries = 0;
    sendRestartNode();
    s_lastSendMs = millis();
}

bool escFlasherTick() {
    // Process CAN TX/RX
    canAdapter.processTxRxOnce();

    if (s_progress.state == FlashState::IDLE ||
        s_progress.state == FlashState::SUCCESS ||
        s_progress.state == FlashState::FAILED) {
        return false;  // Not running
    }

    unsigned long now = millis();
    bool timedOut = (now - s_lastSendMs) > FW_UPDATE_TIMEOUT_MS;

    switch (s_progress.state) {

    case FlashState::RESTARTING:
        if (s_gotResponse || timedOut) {
            // Whether or not we got a restart response, move to checking boot.
            // The ESC may not respond to restart before rebooting.
            if (timedOut && s_retries < 2) {
                // Give the ESC time to reboot
                s_retries++;
                s_lastSendMs = now;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(500));  // Wait for ESC to reboot
            s_progress.state = FlashState::CHECKING_BOOT;
            s_progress.statusMsg = "Checking bootloader...";
            s_gotResponse = false;
            s_retries = 0;
            sendGetBootStatus();
            s_lastSendMs = millis();
        }
        break;

    case FlashState::CHECKING_BOOT:
        if (s_gotResponse) {
            if (s_bootMode == 2) {
                // In bootloader mode
                USBSerial.println("FW Flash: ESC in bootloader mode");
                s_progress.state = FlashState::STARTING;
                s_progress.statusMsg = "Starting upgrade...";
                s_gotResponse = false;
                s_retries = 0;
                sendStartUpgrade();
                s_lastSendMs = millis();
            } else {
                USBSerial.printf("FW Flash: ESC mode=%u, not bootloader\n", s_bootMode);
                // Retry restart
                if (s_retries < FW_UPDATE_MAX_RETRIES) {
                    s_retries++;
                    s_gotResponse = false;
                    sendRestartNode();
                    s_lastSendMs = millis();
                    vTaskDelay(pdMS_TO_TICKS(500));
                    sendGetBootStatus();
                    s_lastSendMs = millis();
                } else {
                    s_progress.state = FlashState::FAILED;
                    s_progress.statusMsg = "ESC won't enter bootloader";
                }
            }
        } else if (timedOut) {
            if (s_retries < FW_UPDATE_MAX_RETRIES) {
                s_retries++;
                s_gotResponse = false;
                sendGetBootStatus();
                s_lastSendMs = millis();
            } else {
                s_progress.state = FlashState::FAILED;
                s_progress.statusMsg = "Boot status timeout";
            }
        }
        break;

    case FlashState::STARTING:
        if (s_gotResponse) {
            if (s_responseState == 0) {
                USBSerial.println("FW Flash: Upgrade started, sending data...");
                s_progress.state = FlashState::SENDING_DATA;
                s_progress.statusMsg = "Sending firmware...";
                s_progress.currentChunk = 0;
                s_gotResponse = false;
                s_retries = 0;
                sendFwDataChunk(0);
                s_lastSendMs = millis();
            } else {
                USBSerial.printf("FW Flash: StartUpgrade failed, state=%u\n", s_responseState);
                s_progress.state = FlashState::FAILED;
                s_progress.statusMsg = "Firmware rejected by ESC";
            }
        } else if (timedOut) {
            if (s_retries < FW_UPDATE_MAX_RETRIES) {
                s_retries++;
                s_gotResponse = false;
                sendStartUpgrade();
                s_lastSendMs = millis();
            } else {
                s_progress.state = FlashState::FAILED;
                s_progress.statusMsg = "Start upgrade timeout";
            }
        }
        break;

    case FlashState::SENDING_DATA:
        if (s_gotResponse) {
            if (s_responseState == 0) {
                // Chunk accepted, use next_index from response
                s_progress.currentChunk = s_responseNextIndex;
                s_progress.percentComplete = (uint8_t)((uint32_t)s_progress.currentChunk * 100 / s_progress.totalChunks);
                s_progress.retryCount = 0;

                if (s_progress.currentChunk >= s_progress.totalChunks) {
                    // All chunks sent
                    USBSerial.println("FW Flash: All data sent, ending upgrade...");
                    s_progress.state = FlashState::ENDING;
                    s_progress.statusMsg = "Finalizing...";
                    s_progress.percentComplete = 100;
                    s_gotResponse = false;
                    s_retries = 0;
                    sendEndUpgrade();
                    s_lastSendMs = millis();
                } else {
                    // Send next chunk
                    s_gotResponse = false;
                    s_retries = 0;
                    sendFwDataChunk(s_progress.currentChunk);
                    s_lastSendMs = millis();
                }
            } else {
                USBSerial.printf("FW Flash: Data error at chunk %u\n", s_progress.currentChunk);
                // Retry same chunk
                if (s_retries < FW_UPDATE_MAX_RETRIES) {
                    s_retries++;
                    s_gotResponse = false;
                    sendFwDataChunk(s_progress.currentChunk);
                    s_lastSendMs = millis();
                } else {
                    s_progress.state = FlashState::FAILED;
                    s_progress.statusMsg = "Data transfer failed";
                }
            }
        } else if (timedOut) {
            if (s_retries < FW_UPDATE_MAX_RETRIES) {
                s_retries++;
                s_progress.retryCount = s_retries;
                s_gotResponse = false;
                sendFwDataChunk(s_progress.currentChunk);
                s_lastSendMs = millis();
            } else {
                s_progress.state = FlashState::FAILED;
                s_progress.statusMsg = "Transfer timeout";
            }
        }
        break;

    case FlashState::ENDING:
        if (s_gotResponse) {
            if (s_responseState == 0) {
                USBSerial.println("FW Flash: SUCCESS");
                s_progress.state = FlashState::SUCCESS;
                s_progress.statusMsg = "Firmware updated!";
            } else {
                USBSerial.printf("FW Flash: End upgrade failed, state=%u\n", s_responseState);
                s_progress.state = FlashState::FAILED;
                s_progress.statusMsg = "Upgrade verification failed";
            }
        } else if (timedOut) {
            if (s_retries < FW_UPDATE_MAX_RETRIES) {
                s_retries++;
                s_gotResponse = false;
                sendEndUpgrade();
                s_lastSendMs = millis();
            } else {
                s_progress.state = FlashState::FAILED;
                s_progress.statusMsg = "End upgrade timeout";
            }
        }
        break;

    default:
        break;
    }

    return true;  // Still running
}

const FlashProgress& escFlasherGetProgress() { return s_progress; }
bool escFlasherIsIdle() { return s_progress.state == FlashState::IDLE; }
bool escFlasherSucceeded() { return s_progress.state == FlashState::SUCCESS; }
