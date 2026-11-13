#include "BMS_CAN.h"

#include <cmath>

BMS_CAN::BMS_CAN(uint8_t csPin, long baudrate) : baudrate(baudrate) {
  mcp = new platform::Mcp2515(csPin);
  initializeMembers();
}

void BMS_CAN::initializeMembers() {
  soc = 0.0;
  batteryVoltage = 0.0;
  batteryCurrent = 0.0;
  highestCellVoltage = 0.0;
  lowestCellVoltage = 0.0;
  batteryCharging = false;
  chargeWireConnected = false;
  batteryLowSOC = false;
  batteryReady = false;
  dischargeMOSStatus = false;
  chargeMOSStatus = false;
  highestTemperature = NAN;
  lowestTemperature = NAN;
  batteryCycle = 0;
  energyCycle = 0.0;
  batteryFailureLevel = 0;
  lastMessageTime = 0; // Initialize lastMessageTime

  // Initialize BMS type detection state
  bmsType = BMS_TYPE_UNKNOWN;
  cellEndianDetected = false;
  cellVoltagesLittleEndian = false;

  // Initialize Battery ID
  memset(batteryID, 0, sizeof(batteryID));
  // Initialize temperature readings to NaN (no data yet)
  for (int i = 0; i < kMaxTemperatures; i++) {
    temperatures[i] = NAN;
  }
  // Initialize cell voltages
  memset(cellVoltages, 0, sizeof(cellVoltages));

#ifdef FREERTOS_H
  dataMutex = xSemaphoreCreateMutex();
#endif
}

bool BMS_CAN::begin() {
  return baudrate == 250000 && mcp->initialize(); // begin() returns int, we want bool
}

void BMS_CAN::update() {
  platform::CanFrame frame;
  while (mcp->receive(&frame)) {
    if (frame.extended) parsePacket(frame.id, frame.data, frame.length);
  }
}

void BMS_CAN::parsePacket(uint32_t id, uint8_t* data, uint8_t len) {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  lastMessageTime = platform::millis32(); // Update timestamp whenever any valid packet is received
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif

  if (id == 0x18FF28F4) { // BMS Basic Information 1
    bool newChargeWireConnected = data[0] & 0x01;
    bool newBatteryCharging = data[0] & 0x02;
    bool newBatteryLowSOC = data[0] & 0x04;
    bool newBatteryReady = data[0] & 0x08;
    bool newDischargeMOSStatus = data[0] & 0x10;
    bool newChargeMOSStatus = data[0] & 0x20;
    float newSOC = data[1];
    float newBatteryVoltage = (data[4] | (data[5] << 8)) * 0.1;
    float newBatteryCurrent = ((data[2] | (data[3] << 8)) - 5000) * 0.1;
    uint8_t newBatteryFailureLevel = data[6];

#ifdef FREERTOS_H
    xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
    // Update member variables
    chargeWireConnected = newChargeWireConnected;
    batteryCharging = newBatteryCharging;
    batteryLowSOC = newBatteryLowSOC;
    batteryReady = newBatteryReady;
    dischargeMOSStatus = newDischargeMOSStatus;
    chargeMOSStatus = newChargeMOSStatus;
    soc = newSOC;
    batteryVoltage = newBatteryVoltage;
    batteryCurrent = newBatteryCurrent;
    batteryFailureLevel = newBatteryFailureLevel;
#ifdef FREERTOS_H
    xSemaphoreGive(dataMutex);
#endif
  } else if (id == 0x18FE28F4) { // BMS Basic Information 2
    float newHighestCellVoltage = (data[0] | (data[1] << 8)) * 0.001;
    float newLowestCellVoltage = (data[2] | (data[3] << 8)) * 0.001;
    float newHighestTemperature = data[4] - 40;
    float newLowestTemperature = data[5] - 40;
    if (newHighestTemperature <= TEMP_PROBE_DISCONNECTED) newHighestTemperature = NAN;
    if (newLowestTemperature <= TEMP_PROBE_DISCONNECTED) newLowestTemperature = NAN;

#ifdef FREERTOS_H
    xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
    highestCellVoltage = newHighestCellVoltage;
    lowestCellVoltage = newLowestCellVoltage;
    highestTemperature = newHighestTemperature;
    lowestTemperature = newLowestTemperature;
#ifdef FREERTOS_H
    xSemaphoreGive(dataMutex);
#endif
  } else if (id == 0x18A328F4) { // Battery Cycle Information (Type B/newer only)
    // This frame is exclusive to Type B/newer BMS. Both cycle count and energy are
    // big-endian. Unit for energy is mAh (raw * 0.001 = Ah).
    uint32_t newBatteryCycle = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                               ((uint32_t)data[2] << 8) | data[3];
    uint32_t newEnergyCycleRaw = ((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) |
                                 ((uint32_t)data[6] << 8) | data[7];
    float newEnergyCycleAh = newEnergyCycleRaw * 0.001f;

#ifdef FREERTOS_H
    xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
    batteryCycle = newBatteryCycle;
    energyCycle = newEnergyCycleAh;
#ifdef FREERTOS_H
    xSemaphoreGive(dataMutex);
#endif
  } else if (id >= 0x18A028F4 && id <= 0x18A328F4 && len == 8) {
    // Battery ID parsing (Type B/newer only; receiving this confirms BMS type)
    int offset = (id - 0x18A028F4) * 8;
    if (offset >= 0 && offset < 24) { // Ensure we don't overflow our buffer
#ifdef FREERTOS_H
      xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
      if (offset == 0) {
        bmsType = BMS_TYPE_B; // Battery ID only exists on Type B (newer) BMS
      }
      for (int i = 0; i < 8; i++) {
        batteryID[offset + i] = (char)data[i];
      }
#ifdef FREERTOS_H
      xSemaphoreGive(dataMutex);
#endif
    }
  } else if (id == 0x18B428F4 && len == 8) {
    // Battery Temperature Parsing (Only first frame, PF=0xB4)
    // Sanitize: a raw byte of 0 decodes to -40C which means probe disconnected.
    float newTemps[kMaxTemperatures];
    for (int i = 0; i < kMaxTemperatures; i++) {
      float raw = (float)data[i] - 40.0f;
      newTemps[i] = (raw <= TEMP_PROBE_DISCONNECTED) ? NAN : raw;
    }

    // Recompute highest/lowest from connected probes only
    float newHighest = NAN;
    float newLowest = NAN;
    for (int i = 0; i < kMaxTemperatures; i++) {
      if (std::isnan(newTemps[i])) continue;
      if (std::isnan(newHighest) || newTemps[i] > newHighest) newHighest = newTemps[i];
      if (std::isnan(newLowest) || newTemps[i] < newLowest) newLowest = newTemps[i];
    }

#ifdef FREERTOS_H
    xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
    for (int i = 0; i < kMaxTemperatures; i++) {
      temperatures[i] = newTemps[i];
    }
    highestTemperature = newHighest;
    lowestTemperature = newLowest;
#ifdef FREERTOS_H
    xSemaphoreGive(dataMutex);
#endif
  } else {
    // Per-cell voltage frames: ID 0x18PP28F4, PP = PF (0xC8…0xC8 + kMaxCells/4 -1)
    uint8_t frameType = (id >> 24) & 0xFF;
    uint8_t pf = (id >> 16) & 0xFF;
    uint8_t sourceAddr = (id >> 8) & 0xFF;
    if (frameType == 0x18 && sourceAddr == 0x28 && pf >= 0xC8 && pf < 0xC8 + kMaxCells / 4 && len == 8) {
#ifdef FREERTOS_H
      xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
      int baseCell = (pf - 0xC8) * 4;

      // Auto-detect cell voltage endianness on first frame, using highestCellVoltage
      // (populated from basic info frame 0x18FE28F4) as the reference.
      if (!cellEndianDetected && highestCellVoltage > 0.0f) {
        float leScore = 0.0f, beScore = 0.0f;
        for (int i = 0; i < 4; i++) {
          uint16_t le_mv = (uint16_t)data[i * 2] | ((uint16_t)data[i * 2 + 1] << 8);
          uint16_t be_mv = ((uint16_t)data[i * 2] << 8) | (uint16_t)data[i * 2 + 1];
          float le_v = le_mv / 1000.0f;
          float be_v = be_mv / 1000.0f;
          if (le_v >= 2.0f && le_v <= 4.5f) leScore += 1.0f - fabsf(le_v - highestCellVoltage);
          if (be_v >= 2.0f && be_v <= 4.5f) beScore += 1.0f - fabsf(be_v - highestCellVoltage);
        }
        cellVoltagesLittleEndian = (leScore > beScore);
        cellEndianDetected = true;
        bmsType = cellVoltagesLittleEndian ? BMS_TYPE_B : BMS_TYPE_A;
      }

      for (int i = 0; i < 4; i++) {
        int cellIdx = baseCell + i;
        if (cellIdx < kMaxCells) {
          uint16_t voltage_mV;
          if (cellVoltagesLittleEndian) {
            voltage_mV = (uint16_t)data[i * 2] | ((uint16_t)data[i * 2 + 1] << 8);
          } else {
            voltage_mV = ((uint16_t)data[i * 2] << 8) | (uint16_t)data[i * 2 + 1];
          }
          cellVoltages[cellIdx] = voltage_mV / 1000.0f;
        }
      }
#ifdef FREERTOS_H
      xSemaphoreGive(dataMutex);
#endif
    }
  }
}

float BMS_CAN::getSOC() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  float value = soc;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

float BMS_CAN::getBatteryVoltage() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  float value = batteryVoltage;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

float BMS_CAN::getBatteryCurrent() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  float value = batteryCurrent;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

float BMS_CAN::getPower() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  float value = (batteryVoltage * batteryCurrent) / 1000.0;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

float BMS_CAN::getHighestCellVoltage() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  float value = highestCellVoltage;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

float BMS_CAN::getLowestCellVoltage() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  float value = lowestCellVoltage;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

bool BMS_CAN::isBatteryCharging() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  bool value = batteryCharging;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

bool BMS_CAN::isChargeWireConnected() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  bool value = chargeWireConnected;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

bool BMS_CAN::isBatteryLowSOC() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  bool value = batteryLowSOC;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

bool BMS_CAN::isBatteryReady() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  bool value = batteryReady;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

bool BMS_CAN::isDischargeMOSStatus() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  bool value = dischargeMOSStatus;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

bool BMS_CAN::isChargeMOSStatus() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  bool value = chargeMOSStatus;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

float BMS_CAN::getHighestTemperature() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  float value = highestTemperature;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

float BMS_CAN::getLowestTemperature() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  float value = lowestTemperature;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

uint32_t BMS_CAN::getBatteryCycle() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  uint32_t value = batteryCycle;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

float BMS_CAN::getEnergyCycle() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  float value = energyCycle;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

uint8_t BMS_CAN::getBatteryFailureLevel() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  uint8_t value = batteryFailureLevel;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

std::string BMS_CAN::getBatteryID() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  std::string value = std::string(batteryID);
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

bool BMS_CAN::isConnected(unsigned long timeout_ms) {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  unsigned long currentTime = platform::millis32();
  bool connected = (currentTime - lastMessageTime) < timeout_ms;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return connected;
}

unsigned long BMS_CAN::getLastMessageTime() {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  unsigned long time = lastMessageTime;
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return time;
}

float BMS_CAN::getTemperature(uint8_t index) {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  float value = 0.0;
  if (index < kMaxTemperatures) {
    value = temperatures[index];
  }
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

bool BMS_CAN::isTemperatureConnected(uint8_t index) {
  return !std::isnan(getTemperature(index));
}

uint8_t BMS_CAN::getConnectedTemperatureCount(uint8_t startIndex, uint8_t count) {
  uint8_t connected = 0;
  for (uint8_t i = 0; i < count; i++) {
    if (isTemperatureConnected(startIndex + i)) {
      connected++;
    }
  }
  return connected;
}

float BMS_CAN::getCellVoltage(uint8_t index) {
#ifdef FREERTOS_H
  xSemaphoreTake(dataMutex, portMAX_DELAY);
#endif
  float value = 0.0;
  if (index < kMaxCells) {
    value = cellVoltages[index];
  }
#ifdef FREERTOS_H
  xSemaphoreGive(dataMutex);
#endif
  return value;
}

BmsType BMS_CAN::getBmsType() const {
  return bmsType;
}
