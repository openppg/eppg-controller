#include "sp140/ble/ota_service.h"

#include <Arduino.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "sp140/ble/ble_ids.h"
#include "sp140/device_state.h"
#include <cstring>

// Access global state to block OTA when ARMED
extern volatile DeviceState currentState;

namespace {

// Protocol Constants
const uint16_t CMD_START = 0x0001;
const uint16_t CMD_END = 0x0002;
const uint16_t CMD_ACK = 0x0003;

const uint16_t ACK_SUCCESS = 0x0000;
const uint16_t ACK_ERR_CRC = 0x0001;
const uint16_t ACK_ERR_SECTOR = 0x0002;
const uint16_t ACK_ERR_LEN = 0x0003;

// State
volatile bool otaInProgress = false;
volatile size_t imageTotalLen = 0;
esp_ota_handle_t updateHandle = 0;
const esp_partition_t* updatePartition = nullptr;

// Buffering
uint8_t sectorBuffer[4096];
uint16_t sectorBufferLen = 0;
uint16_t currentSectorIndex = 0;
size_t receivedBytes = 0;

// OTA characteristics
NimBLECharacteristic* pRecvFwChar = nullptr;
NimBLECharacteristic* pProgressChar = nullptr;
NimBLECharacteristic* pCommandChar = nullptr;
NimBLECharacteristic* pCustomerChar = nullptr;

// CRC16 Implementation (Polynomial 0x8005, reversed for LE)
uint16_t crc16_le(uint16_t crc, const uint8_t *buffer, size_t len) {
    while (len--) {
        crc ^= *buffer++;
        for (int i = 0; i < 8; i++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else crc = (crc >> 1);
        }
    }
    return crc;
}

void sendAck(uint16_t sector, uint16_t status, uint16_t expectedSector = 0) {
    if (pRecvFwChar) {
        uint8_t packet[6];
        packet[0] = sector & 0xFF;
        packet[1] = (sector >> 8) & 0xFF;
        packet[2] = status & 0xFF;
        packet[3] = (status >> 8) & 0xFF;

        // Android app expects expected sector index in bytes 4-5 on sector error.
        packet[4] = expectedSector & 0xFF;
        packet[5] = (expectedSector >> 8) & 0xFF;

        pRecvFwChar->setValue(packet, 6);
        pRecvFwChar->indicate();
        USBSerial.printf("OTA: Sent ACK Sector=%d Status=%d\n", sector, status);
    }
}

// Send Command Response (ID=3, Payload, CRC)
// Matches esp_ble_ota_cmd_t structure: 20 bytes total
void sendCommandResponse(uint16_t ackId, uint16_t status) {
    if (pCommandChar) {
        uint8_t packet[20];  // 2 ID + 16 Payload + 2 CRC
        memset(packet, 0, 20);

        packet[0] = CMD_ACK & 0xFF;  // 0x03
        packet[1] = (CMD_ACK >> 8) & 0xFF;  // 0x00

        // Payload (Bytes 2-17): Byte 2-3 = Ack ID, Byte 4-5 = Status
        packet[2] = ackId & 0xFF;
        packet[3] = (ackId >> 8) & 0xFF;
        packet[4] = status & 0xFF;
        packet[5] = (status >> 8) & 0xFF;

        uint16_t crc = crc16_le(0, packet, 18);
        packet[18] = crc & 0xFF;
        packet[19] = (crc >> 8) & 0xFF;

        pCommandChar->setValue(packet, 20);
        pCommandChar->indicate();
        USBSerial.printf("OTA: Sent CMD Response AckId=%d Status=%d\n", ackId, status);
    }
}

void abortOta() {
    if (otaInProgress) {
        if (updateHandle) esp_ota_abort(updateHandle);
        otaInProgress = false;
        USBSerial.println("OTA: Aborted.");
    }
    sectorBufferLen = 0;
    currentSectorIndex = 0;
    receivedBytes = 0;
}

class OtaCommandCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        std::string value = pChar->getValue();
        // Command Packet: ID(2) + Payload(16) + CRC(2) = 20 bytes
        if (value.length() < 20) {
            USBSerial.println("OTA Error: Invalid CMD length");
            // Can't reliably reply if packet structure is broken
            return;
        }

        const uint8_t* data = (const uint8_t*)value.data();

        uint16_t cmdId = data[0] | (data[1] << 8);

        // Verify CRC
        uint16_t rxCrc = data[18] | (data[19] << 8);
        uint16_t calcCrc = crc16_le(0, data, 18);
        if (rxCrc != calcCrc) {
            USBSerial.printf("OTA Error: CMD CRC Fail Exp %04X Got %04X\n", rxCrc, calcCrc);
            sendCommandResponse(cmdId, 0x0001);  // Reject (Status 1)
            return;
        }

        if (cmdId == CMD_START) {
            USBSerial.println("OTA: CMD_START");

            if (currentState == ARMED || currentState == ARMED_CRUISING) {
                 USBSerial.println("OTA Blocked: Device ARMED");
                 sendCommandResponse(CMD_START, 0x0001);  // Reject
                 return;
            }

            updatePartition = esp_ota_get_next_update_partition(nullptr);
            if (!updatePartition) {
                USBSerial.println("OTA Error: No partition");
                sendCommandResponse(CMD_START, 0x0001);
                return;
            }

            // Validate image length (Bytes 2-5 of payload -> data[2]..data[5])
            uint32_t imageLen = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);
            if (imageLen > updatePartition->size) {
                USBSerial.printf("OTA Error: Image size %u > partition %u\n", imageLen, updatePartition->size);
                sendCommandResponse(CMD_START, 0x0001);
                return;
            }

            esp_err_t err = esp_ota_begin(updatePartition, OTA_SIZE_UNKNOWN, &updateHandle);
            if (err != ESP_OK) {
                USBSerial.printf("OTA Error: Begin failed 0x%x\n", err);
                sendCommandResponse(CMD_START, 0x0001);
                return;
            }

            otaInProgress = true;
            imageTotalLen = imageLen;
            receivedBytes = 0;
            sectorBufferLen = 0;
            currentSectorIndex = 0;

            sendCommandResponse(CMD_START, 0x0000);  // Accept

        } else if (cmdId == CMD_END) {
            USBSerial.println("OTA: CMD_END");
            if (otaInProgress) {
                // Defensive: Verify total bytes received match manifest
                if (receivedBytes != imageTotalLen) {
                    USBSerial.printf("OTA Error: Size Mismatch Rx:%u Exp:%u\n", receivedBytes, imageTotalLen);
                    sendCommandResponse(CMD_END, 0x0001);  // Fail
                    abortOta();
                    return;
                }

                if (esp_ota_end(updateHandle) == ESP_OK) {
                    if (esp_ota_set_boot_partition(updatePartition) == ESP_OK) {
                        USBSerial.println("OTA Success. Restarting...");
                        sendCommandResponse(CMD_END, 0x0000);  // Success
                        delay(1000);  // Allow BLE flush
                        ESP.restart();
                        return;
                    } else {
                        USBSerial.println("OTA Error: Set Boot Partition Failed");
                    }
                } else {
                    USBSerial.println("OTA Error: OTA End Failed");
                }
                sendCommandResponse(CMD_END, 0x0001);  // Fail
                abortOta();
            } else {
                sendCommandResponse(CMD_END, 0x0001);  // Not in progress
            }
        }
    }
};

class OtaDataCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        if (!otaInProgress) return;

        std::string valStr = pChar->getValue();
        size_t len = valStr.length();
        const uint8_t* data = (const uint8_t*)valStr.data();

        if (len < 3) return;  // Header: Sector(2) + Seq(1)

        uint16_t sector = data[0] | (data[1] << 8);
        uint8_t seq = data[2];
        const uint8_t* payload = data + 3;
        size_t payloadLen = len - 3;

        if (sector != currentSectorIndex) {
            USBSerial.printf("OTA Warn: Sector mismatch exp %d got %d\n", currentSectorIndex, sector);
            sectorBufferLen = 0;
            sendAck(sector, ACK_ERR_SECTOR, currentSectorIndex);
            return;
        }

        if (seq == 0xFF) {
            // Last packet: Payload is Data + CRC16(2)
            if (payloadLen < 2) return;

            size_t actualDataLen = payloadLen - 2;

            if (sectorBufferLen + actualDataLen > sizeof(sectorBuffer)) {
                sendAck(sector, ACK_ERR_LEN);
                sectorBufferLen = 0;
                return;
            }

            memcpy(sectorBuffer + sectorBufferLen, payload, actualDataLen);
            sectorBufferLen += actualDataLen;

            // Verify CRC
            uint16_t rxCrc = payload[actualDataLen] | (payload[actualDataLen + 1] << 8);
            uint16_t calcCrc = crc16_le(0, sectorBuffer, sectorBufferLen);

            if (rxCrc == calcCrc) {
                esp_err_t err = esp_ota_write(updateHandle, sectorBuffer, sectorBufferLen);
                if (err == ESP_OK) {
                    receivedBytes += sectorBufferLen;
                    sendAck(sector, ACK_SUCCESS);
                    currentSectorIndex++;
                    sectorBufferLen = 0;
                } else {
                    USBSerial.printf("OTA Error: Write failed 0x%x\n", err);
                    sendAck(sector, ACK_ERR_SECTOR, currentSectorIndex);  // Force retry
                    sectorBufferLen = 0;
                    // Don't hard abort yet, allow retry?
                    // Espressif tools usually retry logic is client side.
                    // If we return ERR_SECTOR, client re-sends sector.
                }
            } else {
                USBSerial.printf("OTA Error: CRC fail exp 0x%04X got 0x%04X\n", rxCrc, calcCrc);
                sendAck(sector, ACK_ERR_CRC);
                sectorBufferLen = 0;
            }

        } else {
            // Normal packet
            if (sectorBufferLen + payloadLen > sizeof(sectorBuffer)) {
                USBSerial.println("OTA Error: Buffer overflow");
                sectorBufferLen = 0;
                sendAck(sector, ACK_ERR_LEN);
                return;
            }
            memcpy(sectorBuffer + sectorBufferLen, payload, payloadLen);
            sectorBufferLen += payloadLen;
        }
    }
};

static OtaCommandCallback cmdCallback;
static OtaDataCallback dataCallback;

}  // namespace

void initOtaBleService(NimBLEServer* pServer) {
    NimBLEService* pService = pServer->createService(OTA_SERVICE_UUID);

    // Firmware data (Write No Response + Indicate for ACKs)
    pRecvFwChar = pService->createCharacteristic(
        OTA_RECV_FW_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::INDICATE);
    pRecvFwChar->setCallbacks(&dataCallback);

    // Progress (Indicate only; app subscribes)
    pProgressChar = pService->createCharacteristic(
        OTA_PROGRESS_UUID,
        NIMBLE_PROPERTY::INDICATE);

    // Command (Write + Indicate)
    pCommandChar = pService->createCharacteristic(
        OTA_COMMAND_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
    pCommandChar->setCallbacks(&cmdCallback);

    // Customer (Indicate only; app subscribes)
    pCustomerChar = pService->createCharacteristic(
        OTA_CUSTOMER_UUID,
        NIMBLE_PROPERTY::INDICATE);

    pService->start();
}

bool isOtaInProgress() {
    return otaInProgress;
}
