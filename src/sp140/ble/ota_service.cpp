#include "sp140/ble/ota_service.h"

#include <Arduino.h>
#include "sp140/time_utils.h"
#include <Preferences.h>
#include <atomic>
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "sp140/ble/ble_core.h"
#include "sp140/ble/ble_ids.h"
#include "sp140/device_state.h"
#include <cstring>

// Access global state to block OTA when ARMED
extern volatile DeviceState currentState;
extern SemaphoreHandle_t stateMutex;

namespace {

// Serializes every OTA state mutation. The command/data write callbacks run on
// the NimBLE host task while checkOtaTimeout() runs on bleNotifyTask. Without
// this, a timeout-driven abort on bleNotifyTask could call esp_ota_abort() and
// resetOtaState() while a data callback is mid esp_ota_write() on the host task
// — a use-after-abort on updateHandle/sectorBuffer that corrupts flash/heap.
// abortOtaImpl() assumes the lock is already held; the public entry points
// (callbacks, checkOtaTimeout, abortOta) acquire it.
SemaphoreHandle_t otaStateMutex = nullptr;

class OtaStateLock {
 public:
  explicit OtaStateLock(TickType_t timeout) {
    if (otaStateMutex != nullptr) {
      held_ = (xSemaphoreTake(otaStateMutex, timeout) == pdTRUE);
    }
  }
  ~OtaStateLock() {
    if (held_ && otaStateMutex != nullptr) {
      xSemaphoreGive(otaStateMutex);
    }
  }
  bool held() const { return held_; }

 private:
  bool held_ = false;
};

// Protocol Constants
const uint16_t CMD_START = 0x0001;
const uint16_t CMD_END = 0x0002;
const uint16_t CMD_ACK = 0x0003;

const uint16_t ACK_SUCCESS = 0x0000;
const uint16_t ACK_ERR_CRC = 0x0001;
const uint16_t ACK_ERR_SECTOR = 0x0002;
const uint16_t ACK_ERR_LEN = 0x0003;

// State
std::atomic<bool> otaInProgress{false};
QueueHandle_t otaStatusQueue = nullptr;
OtaStatus otaStatus;
std::atomic<uint32_t> renderedRevision{0};

// Called under otaStateMutex. Queue copies are independent of flash writes, so
// the UI never waits on this mutex while holding lvglMutex / the SPI bus.
void publishStatus(OtaPhase phase, OtaError error = OtaError::None) {
    otaStatus.phase = phase;
    otaStatus.error = error;
    otaStatus.changedAtMs = timeMillis();
    ++otaStatus.revision;
    if (otaStatusQueue) xQueueOverwrite(otaStatusQueue, &otaStatus);
}

void waitForStatusFrame() {
    const uint32_t started = timeMillis();
    while (renderedRevision.load() != otaStatus.revision &&
           timeMillis() - started < 250) {
        timeDelay(10);
    }
}

// One receipt per completed installation, never per progress packet. Store the
// target slot so rollback cannot be mistaken for a successful new-firmware boot.
void saveBootReceipt(uint32_t address) {
    Preferences prefs;
    if (prefs.begin("ota-ui", false)) {
        if (prefs.putUInt("target", address) != sizeof(address)) {
            USBSerial.println("OTA: Could not save update-screen boot receipt");
        }
        prefs.end();
    }
}
volatile size_t imageTotalLen = 0;
esp_ota_handle_t updateHandle = 0;
const esp_partition_t* updatePartition = nullptr;

// Buffering
uint8_t sectorBuffer[4096];
uint16_t sectorBufferLen = 0;
uint16_t currentSectorIndex = 0;
size_t receivedBytes = 0;
uint32_t packetCount = 0;

// Idle timeout: abort OTA if no data received within this period
const unsigned long OTA_TIMEOUT_MS = 30000;
volatile unsigned long lastOtaActivityMs = 0;

// OTA characteristics
NimBLECharacteristic* pRecvFwChar = nullptr;
NimBLECharacteristic* pProgressChar = nullptr;
NimBLECharacteristic* pCommandChar = nullptr;
NimBLECharacteristic* pCustomerChar = nullptr;

// CRC16-CCITT (poly 0x1021, init 0) to match Espressif Android app
uint16_t crc16_ccitt(const uint8_t* buffer, size_t len) {
    uint16_t crc16 = 0;
    for (size_t i = 0; i < len; ++i) {
        crc16 ^= static_cast<uint16_t>(buffer[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if (crc16 & 0x8000) {
                crc16 = static_cast<uint16_t>((crc16 << 1) ^ 0x1021);
            } else {
                crc16 = static_cast<uint16_t>(crc16 << 1);
            }
        }
    }
    return crc16;
}

void resetOtaState() {
    updateHandle = 0;
    updatePartition = nullptr;
    imageTotalLen = 0;
    sectorBufferLen = 0;
    currentSectorIndex = 0;
    receivedBytes = 0;
    packetCount = 0;
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
        pRecvFwChar->notify();
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

        uint16_t crc = crc16_ccitt(packet, 18);
        packet[18] = crc & 0xFF;
        packet[19] = (crc >> 8) & 0xFF;

        pCommandChar->setValue(packet, 20);
        pCommandChar->notify();
        USBSerial.printf("OTA: Sent CMD Response AckId=%d Status=%d\n", ackId, status);
    }
}

// Forward declaration
void abortOtaImpl(OtaError error = OtaError::None);

void handleCmdStart(const uint8_t* data) {
    USBSerial.println("OTA: CMD_START");

    // A duplicate CMD_START (app restarted the transfer) would orphan the live
    // esp_ota handle, because resetOtaState() below only zeroes updateHandle
    // without esp_ota_abort(). Abort the previous session cleanly first. Runs
    // under the OtaStateLock already held by OtaCommandCallback::onWrite.
    if (otaInProgress) {
        USBSerial.println("OTA: CMD_START during active update - aborting previous session");
        abortOtaImpl();
    }

    if (!otaStatusQueue || !otaStateMutex || !stateMutex) {
        sendCommandResponse(CMD_START, 0x0001);
        return;
    }
    // Serialize acceptance with changeDeviceState so preparation cannot race
    // the final arm transition on the button task.
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) != pdTRUE) return;
    if (currentState == ARMED || currentState == ARMED_CRUISING) {
        xSemaphoreGive(stateMutex);
        USBSerial.println("OTA Blocked: Device ARMED");
        sendCommandResponse(CMD_START, 0x0001);
        return;
    }

    // Cover partition preparation as well as the transfer with the interlock.
    otaInProgress = true;
    xSemaphoreGive(stateMutex);
    resetOtaState();
    otaStatus.written = 0;
    otaStatus.total = 0;
    publishStatus(OtaPhase::Preparing);
    waitForStatusFrame();

    updatePartition = esp_ota_get_next_update_partition(nullptr);
    if (!updatePartition) {
        USBSerial.println("OTA Error: No partition");
        abortOtaImpl(OtaError::NoPartition);
        sendCommandResponse(CMD_START, 0x0001);
        return;
    }

    uint32_t imageLen = static_cast<uint32_t>(data[2])
                      | (static_cast<uint32_t>(data[3]) << 8)
                      | (static_cast<uint32_t>(data[4]) << 16)
                      | (static_cast<uint32_t>(data[5]) << 24);
    if (imageLen == 0 || imageLen > updatePartition->size) {
        USBSerial.printf("OTA Error: Invalid image size %u (partition %u)\n", imageLen, updatePartition->size);
        abortOtaImpl(OtaError::InvalidSize);
        sendCommandResponse(CMD_START, 0x0001);
        return;
    }

    esp_err_t err = esp_ota_begin(updatePartition, imageLen, &updateHandle);
    if (err != ESP_OK) {
        USBSerial.printf("OTA Error: Begin failed 0x%x\n", err);
        updateHandle = 0;
        abortOtaImpl(OtaError::Begin);
        sendCommandResponse(CMD_START, 0x0001);
        return;
    }

    otaInProgress = true;
    imageTotalLen = imageLen;
    otaStatus.total = imageLen;
    publishStatus(OtaPhase::Receiving);
    lastOtaActivityMs = timeMillis();
    requestFastConnParams();
    sendCommandResponse(CMD_START, 0x0000);  // Accept
}

class OtaCommandCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        OtaStateLock lock(portMAX_DELAY);  // serialize vs checkOtaTimeout (other task)
        if (!lock.held()) return;
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
        uint16_t calcCrc = crc16_ccitt(data, 18);
        if (rxCrc != calcCrc) {
            USBSerial.printf("OTA Error: CMD CRC Fail Exp %04X Got %04X\n", calcCrc, rxCrc);
            sendCommandResponse(cmdId, 0x0001);  // Reject (Status 1)
            return;
        }

        if (cmdId == CMD_START) {
            handleCmdStart(data);
        } else if (cmdId == CMD_END) {
            USBSerial.println("OTA: CMD_END");
            if (otaInProgress) {
                // Defensive: Verify total bytes received match manifest
                if (receivedBytes != imageTotalLen) {
                    USBSerial.printf("OTA Error: Size Mismatch Rx:%u Exp:%u\n", receivedBytes, imageTotalLen);
                    sendCommandResponse(CMD_END, 0x0001);  // Fail
                    abortOtaImpl(OtaError::SizeMismatch);
                    return;
                }

                publishStatus(OtaPhase::Verifying);
                waitForStatusFrame();
                const esp_err_t endResult = esp_ota_end(updateHandle);
                // esp_ota_end releases the handle even when validation fails.
                updateHandle = 0;
                OtaError failure = OtaError::Verification;
                if (endResult == ESP_OK) {
                    if (esp_ota_set_boot_partition(updatePartition) == ESP_OK) {
                        USBSerial.println("OTA Success. Restarting...");
                        saveBootReceipt(updatePartition->address);
                        publishStatus(OtaPhase::Restarting);
                        sendCommandResponse(CMD_END, 0x0000);  // Success
                        waitForStatusFrame();
                        timeDelay(1000);  // Allow BLE flush
                        ESP.restart();
                        return;
                    } else {
                        USBSerial.println("OTA Error: Set Boot Partition Failed");
                        failure = OtaError::BootPartition;
                    }
                } else {
                    USBSerial.println("OTA Error: OTA End Failed");
                }
                sendCommandResponse(CMD_END, 0x0001);  // Fail
                abortOtaImpl(failure);
            } else {
                sendCommandResponse(CMD_END, 0x0001);  // Not in progress
            }
        }
    }
};

class OtaDataCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        OtaStateLock lock(portMAX_DELAY);  // serialize vs checkOtaTimeout (other task)
        if (!lock.held()) return;
        if (!otaInProgress) return;

        std::string valStr = pChar->getValue();
        size_t len = valStr.length();
        const uint8_t* data = (const uint8_t*)valStr.data();

        if (len < 3) return;  // Header: Sector(2) + Seq(1)

        lastOtaActivityMs = timeMillis();
        packetCount++;
        if (packetCount % 100 == 0) {
            USBSerial.printf("OTA: Received %lu packets (sector %u)\n",
                             static_cast<unsigned long>(packetCount),
                             static_cast<unsigned int>(currentSectorIndex));
        }

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
            uint16_t calcCrc = crc16_ccitt(sectorBuffer, sectorBufferLen);

            if (rxCrc == calcCrc) {
                esp_err_t err = esp_ota_write(updateHandle, sectorBuffer, sectorBufferLen);
                if (err == ESP_OK) {
                    receivedBytes += sectorBufferLen;
                    otaStatus.written = receivedBytes;
                    publishStatus(OtaPhase::Receiving);
                    sendAck(sector, ACK_SUCCESS);
                    USBSerial.printf("OTA: Sector %u written (%u bytes total)\n",
                                     static_cast<unsigned int>(currentSectorIndex),
                                     static_cast<unsigned int>(receivedBytes));
                    currentSectorIndex++;
                    sectorBufferLen = 0;
                } else {
                    USBSerial.printf("OTA Error: Write failed 0x%x\n", err);
                    sendAck(sector, ACK_ERR_SECTOR, currentSectorIndex);
                    // Must abort: the OTA handle's internal write offset may
                    // have advanced partially, so retrying this sector would
                    // write data at the wrong position and corrupt the image.
                    abortOtaImpl(OtaError::Write);
                }
            } else {
                USBSerial.printf("OTA Error: CRC fail exp 0x%04X got 0x%04X\n",
                                 calcCrc, rxCrc);
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

void abortOtaImpl(OtaError error) {
    // Keep the last progress/error snapshot after resetting the transport.
    if (error != OtaError::None) publishStatus(OtaPhase::Failed, error);
    if (otaInProgress) {
        if (updateHandle) esp_ota_abort(updateHandle);
        otaInProgress = false;
        requestNormalConnParams();
        USBSerial.println("OTA: Aborted.");
    }
    resetOtaState();
}

static OtaCommandCallback cmdCallback;
static OtaDataCallback dataCallback;

}  // namespace

void abortOta(OtaError error) {
    // A disconnect during the acknowledged restart is expected, not a failure.
    OtaStateLock lock(portMAX_DELAY);
    if (!lock.held() || !otaInProgress || otaStatus.phase == OtaPhase::Restarting) return;
    abortOtaImpl(error);
}

bool readOtaStatus(OtaStatus& status) {
    return otaStatusQueue && xQueuePeek(otaStatusQueue, &status, 0) == pdTRUE;
}

void acknowledgeOtaRendered(uint32_t revision) {
    renderedRevision.store(revision);
}

bool otaBlocksInput() {
    OtaStatus status;
    return isOtaInProgress() || (readOtaStatus(status) && otaStatusVisible(status, timeMillis()));
}

void finishOtaBoot(bool firmwareValid) {
    OtaStateLock lock(portMAX_DELAY);
    if (!lock.held()) return;
    Preferences prefs;
    if (!prefs.begin("ota-ui", false)) return;
    const uint32_t target = prefs.getUInt("target", 0);
    if (target != 0) {
        prefs.remove("target");
        const esp_partition_t* running = esp_ota_get_running_partition();
        const bool installed = firmwareValid && running && running->address == target;
        publishStatus(installed ? OtaPhase::Complete : OtaPhase::Failed,
                      installed ? OtaError::None : firmwareValid ? OtaError::RolledBack : OtaError::Verification);
    }
    prefs.end();
}

void checkOtaTimeout() {
    // Take the lock without waiting: if a host-task callback is mid-write the
    // device is clearly not idle, so skip this tick rather than block or race.
    OtaStateLock lock(0);
    if (!lock.held()) return;
    if (otaInProgress && (timeMillis() - lastOtaActivityMs > OTA_TIMEOUT_MS)) {
        USBSerial.println("OTA: Idle timeout, aborting.");
        abortOtaImpl(OtaError::Timeout);
    }
}

bool isOtaInProgress() {
    return otaInProgress;
}

void initOtaBleService(NimBLEServer* pServer) {
    if (otaStateMutex == nullptr) {
        otaStateMutex = xSemaphoreCreateMutex();
    }

    if (!otaStatusQueue) otaStatusQueue = xQueueCreate(1, sizeof(OtaStatus));

    NimBLEService* pService = pServer->createService(OTA_SERVICE_UUID);

    // Firmware data (Write No Response + Indicate for ACKs)
    pRecvFwChar = pService->createCharacteristic(
        OTA_RECV_FW_UUID,
        NIMBLE_PROPERTY::WRITE |
        NIMBLE_PROPERTY::WRITE_NR |
        NIMBLE_PROPERTY::NOTIFY |
        NIMBLE_PROPERTY::INDICATE);
    pRecvFwChar->setCallbacks(&dataCallback);

    // Progress (Indicate only; app subscribes for transfer progress)
    pProgressChar = pService->createCharacteristic(
        OTA_PROGRESS_UUID,
        NIMBLE_PROPERTY::NOTIFY |
        NIMBLE_PROPERTY::INDICATE);

    // Command (Write + Indicate)
    pCommandChar = pService->createCharacteristic(
        OTA_COMMAND_UUID,
        NIMBLE_PROPERTY::WRITE |
        NIMBLE_PROPERTY::NOTIFY |
        NIMBLE_PROPERTY::INDICATE);
    pCommandChar->setCallbacks(&cmdCallback);

    // Customer (Indicate only; app subscribes)
    pCustomerChar = pService->createCharacteristic(
        OTA_CUSTOMER_UUID,
        NIMBLE_PROPERTY::NOTIFY |
        NIMBLE_PROPERTY::INDICATE);

    pService->start();
}
