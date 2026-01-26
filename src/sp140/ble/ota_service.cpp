#include "sp140/ble/ota_service.h"

#include <Arduino.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "sp140/ble/ble_ids.h"

namespace {

// OTA Commands (matching Espressif example constants where applicable)
// Example: 0x01 = Start, 0x02 = End, 0x03 = Abort
const uint8_t CMD_START = 0x01;
const uint8_t CMD_END = 0x02;
const uint8_t CMD_ABORT = 0x03;

// OTA Responses
const uint8_t RESP_SUCCESS = 0x00;
const uint8_t RESP_ERROR_BEGIN = 0xE1;
const uint8_t RESP_ERROR_WRITE = 0xE2;
const uint8_t RESP_ERROR_END = 0xE3;
const uint8_t RESP_ERROR_UNKNOWN = 0xFF;

volatile bool otaInProgress = false;
esp_ota_handle_t updateHandle = 0;
const esp_partition_t* updatePartition = nullptr;
size_t receivedBytes = 0;

// Status characteristic for notifications
NimBLECharacteristic* pStatusChar = nullptr;

void sendResponse(uint8_t responseCode) {
    if (pStatusChar) {
        pStatusChar->setValue(&responseCode, 1);
        pStatusChar->notify();
        USBSerial.printf("OTA: Sent response 0x%02X\n", responseCode);
    }
}

class OtaCommandCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        (void)connInfo;
        std::string value = pChar->getValue();
        if (value.length() == 0) return;

        uint8_t command = value[0];

        if (command == CMD_START) {
            USBSerial.println("OTA: Start command received");
            
            updatePartition = esp_ota_get_next_update_partition(nullptr);
            if (!updatePartition) {
                USBSerial.println("OTA Error: No update partition found");
                sendResponse(RESP_ERROR_BEGIN);
                return;
            }

            USBSerial.printf("OTA: Writing to partition subtype %d at offset 0x%x\n", 
                             updatePartition->subtype, updatePartition->address);

            esp_err_t err = esp_ota_begin(updatePartition, OTA_SIZE_UNKNOWN, &updateHandle);
            if (err != ESP_OK) {
                USBSerial.printf("OTA Error: esp_ota_begin failed (0x%x)\n", err);
                sendResponse(RESP_ERROR_BEGIN);
                return;
            }

            otaInProgress = true;
            receivedBytes = 0;
            sendResponse(RESP_SUCCESS);

        } else if (command == CMD_END) {
            USBSerial.println("OTA: End command received");

            if (!otaInProgress) {
                 sendResponse(RESP_ERROR_UNKNOWN);
                 return;
            }

            esp_err_t err = esp_ota_end(updateHandle);
            if (err != ESP_OK) {
                USBSerial.printf("OTA Error: esp_ota_end failed (0x%x)\n", err);
                sendResponse(RESP_ERROR_END);
                otaInProgress = false;
                return;
            }

            err = esp_ota_set_boot_partition(updatePartition);
            if (err != ESP_OK) {
                USBSerial.printf("OTA Error: set_boot_partition failed (0x%x)\n", err);
                sendResponse(RESP_ERROR_END);
                otaInProgress = false;
                return;
            }

            USBSerial.println("OTA: Success! Restarting...");
            sendResponse(RESP_SUCCESS);
            delay(1000);
            ESP.restart();

        } else if (command == CMD_ABORT) {
             USBSerial.println("OTA: Abort command received");
             if (otaInProgress) {
                 esp_ota_end(updateHandle);
                 otaInProgress = false;
             }
             sendResponse(RESP_SUCCESS);
        }
    }
};

class OtaDataCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        (void)connInfo;
        if (!otaInProgress) return;

        std::string value = pChar->getValue();
        if (value.length() > 0) {
            esp_err_t err = esp_ota_write(updateHandle, value.data(), value.length());
            if (err != ESP_OK) {
                USBSerial.printf("OTA Error: Write failed (0x%x)\n", err);
                sendResponse(RESP_ERROR_WRITE);
                // Abort?
            } else {
                receivedBytes += value.length();
                if (receivedBytes % 10240 < value.length()) {
                     USBSerial.printf("OTA: Received %d bytes\n", receivedBytes);
                }
            }
        }
    }
};

} // namespace

void initOtaBleService(NimBLEServer* pServer) {
    NimBLEService* pService = pServer->createService(OTA_SERVICE_UUID);

    // Command Characteristic (Write)
    NimBLECharacteristic* pCommandChar = pService->createCharacteristic(
        OTA_COMMAND_UUID,
        NIMBLE_PROPERTY::WRITE
    );
    pCommandChar->setCallbacks(new OtaCommandCallback());

    // Status Characteristic (Notify)
    pStatusChar = pService->createCharacteristic(
        OTA_STATUS_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    // Data Characteristic (Write No Response)
    NimBLECharacteristic* pDataChar = pService->createCharacteristic(
        OTA_DATA_UUID,
        NIMBLE_PROPERTY::WRITE_NR // Write No Response for speed
    );
    pDataChar->setCallbacks(new OtaDataCallback());

    pService->start();
}

bool isOtaInProgress() {
    return otaInProgress;
}