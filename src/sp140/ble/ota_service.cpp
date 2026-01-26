#include "sp140/ble/ota_service.h"

#include <Arduino.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "sp140/ble/ble_ids.h"
#include <cstring>

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
esp_ota_handle_t updateHandle = 0;
const esp_partition_t* updatePartition = nullptr;
volatile unsigned long lastOtaActivity = 0;

// Buffering
uint8_t sectorBuffer[4096];
uint16_t sectorBufferLen = 0;
uint16_t currentSectorIndex = 0;

// Notify Characteristic
NimBLECharacteristic* pCommandChar = nullptr; 

// CRC16 Implementation (Polynomial 0x8005, reversed for LE)
// Matches Espressif's expected CRC16-ARC/Modbus
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

void abortOta() {
    if (otaInProgress) {
        if (updateHandle) esp_ota_end(updateHandle);
        otaInProgress = false;
        USBSerial.println("OTA: Aborted.");
    }
    sectorBufferLen = 0;
    currentSectorIndex = 0;
}

void sendAck(uint16_t sector, uint16_t status) {
    if (pCommandChar) {
        uint8_t packet[6];
        // Sector (2B LE)
        packet[0] = sector & 0xFF;
        packet[1] = (sector >> 8) & 0xFF;
        // Status (2B LE)
        packet[2] = status & 0xFF;
        packet[3] = (status >> 8) & 0xFF;
        // CRC (2B LE) - CRC of the first 4 bytes
        uint16_t crc = crc16_le(0, packet, 4);
        packet[4] = crc & 0xFF;
        packet[5] = (crc >> 8) & 0xFF;
        
        pCommandChar->setValue(packet, 6);
        pCommandChar->notify();
        USBSerial.printf("OTA: Sent ACK Sector=%d Status=%d\n", sector, status);
    }
}

class OtaCommandCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        std::string value = pChar->getValue();
        if (value.length() < 4) return; // Min length check

        lastOtaActivity = millis();
        const uint8_t* data = (const uint8_t*)value.data();
        
        uint16_t cmdId = data[0] | (data[1] << 8);

        if (cmdId == CMD_START) {
            USBSerial.println("OTA: CMD_START");
            updatePartition = esp_ota_get_next_update_partition(nullptr);
            if (!updatePartition) {
                USBSerial.println("OTA Error: No partition");
                return;
            }
            
            esp_err_t err = esp_ota_begin(updatePartition, OTA_SIZE_UNKNOWN, &updateHandle);
            if (err != ESP_OK) {
                USBSerial.printf("OTA Error: Begin failed 0x%x\n", err);
                return; 
            }

            otaInProgress = true;
            sectorBufferLen = 0;
            currentSectorIndex = 0;
            
            // Send Command Response (ID=3, Payload=Success)
            uint8_t response[6] = {
                0x03, 0x00, // ID = 3 (Response)
                0x00, 0x00, // Payload = 0 (Success)
                0x00, 0x00  // CRC placeholder (TODO: calculate if strictly needed)
            };
            pChar->setValue(response, 6);
            pChar->notify();

        } else if (cmdId == CMD_END) {
            USBSerial.println("OTA: CMD_END");
            if (otaInProgress) {
                if (esp_ota_end(updateHandle) == ESP_OK) {
                    if (esp_ota_set_boot_partition(updatePartition) == ESP_OK) {
                        USBSerial.println("OTA Success. Restarting...");
                        
                        uint8_t response[6] = { 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };
                        pChar->setValue(response, 6);
                        pChar->notify();
                        
                        delay(500);
                        ESP.restart();
                    }
                }
                abortOta();
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

        if (len < 3) return; // Header: Sector(2) + Seq(1)

        lastOtaActivity = millis();

        uint16_t sector = data[0] | (data[1] << 8);
        uint8_t seq = data[2];
        const uint8_t* payload = data + 3;
        size_t payloadLen = len - 3;

        if (sector != currentSectorIndex) {
            // Silently ignore or warn? Official app might retry if we don't ACK.
            // But if we send ACK_ERR_SECTOR, it forces resync.
            USBSerial.printf("OTA Warn: Sector mismatch exp %d got %d\n", currentSectorIndex, sector);
            sendAck(sector, ACK_ERR_SECTOR);
            return;
        }

        if (seq == 0xFF) { // Last packet of sector: Payload contains Data + CRC16(2)
            if (payloadLen < 2) return;
            
            size_t actualDataLen = payloadLen - 2;
            
            if (sectorBufferLen + actualDataLen > sizeof(sectorBuffer)) {
                sendAck(sector, ACK_ERR_LEN);
                sectorBufferLen = 0; // Reset buffer
                return;
            }

            memcpy(sectorBuffer + sectorBufferLen, payload, actualDataLen);
            sectorBufferLen += actualDataLen;

            // Verify CRC
            // CRC is at the end of the packet payload
            uint16_t rxCrc = payload[actualDataLen] | (payload[actualDataLen + 1] << 8);
            uint16_t calcCrc = crc16_le(0, sectorBuffer, sectorBufferLen);

            if (rxCrc == calcCrc) {
                // Write to flash
                esp_err_t err = esp_ota_write(updateHandle, sectorBuffer, sectorBufferLen);
                if (err == ESP_OK) {
                    sendAck(sector, ACK_SUCCESS);
                    currentSectorIndex++;
                    sectorBufferLen = 0;
                } else {
                    USBSerial.printf("OTA Error: Write failed 0x%x\n", err);
                    abortOta();
                }
            } else {
                USBSerial.printf("OTA Error: CRC fail exp 0x%04X got 0x%04X\n", rxCrc, calcCrc);
                sendAck(sector, ACK_ERR_CRC);
                sectorBufferLen = 0; // Reset for retry
            }

        } else { // Normal packet
            if (sectorBufferLen + payloadLen > sizeof(sectorBuffer)) {
                USBSerial.println("OTA Error: Buffer overflow");
                // Don't ACK, let client retry or timeout
                return; 
            }
            memcpy(sectorBuffer + sectorBufferLen, payload, payloadLen);
            sectorBufferLen += payloadLen;
        }
    }
};

static OtaCommandCallback cmdCallback;
static OtaDataCallback dataCallback;

} // namespace

void initOtaBleService(NimBLEServer* pServer) {
    NimBLEService* pService = pServer->createService(OTA_SERVICE_UUID);

    // Command (Write + Notify for ACKs)
    pCommandChar = pService->createCharacteristic(
        OTA_COMMAND_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
    );
    pCommandChar->setCallbacks(&cmdCallback);

    // Data (Write No Response)
    NimBLECharacteristic* pDataChar = pService->createCharacteristic(
        OTA_DATA_UUID,
        NIMBLE_PROPERTY::WRITE_NR
    );
    pDataChar->setCallbacks(&dataCallback);

    pService->start();
}

bool isOtaInProgress() {
    return otaInProgress;
}