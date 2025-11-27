#include <Arduino.h>

#include "sp140/ble/esc_service.h"

#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"

namespace {

BLEService* pEscService = nullptr;
BLECharacteristic* pESCVoltage = nullptr;
BLECharacteristic* pESCCurrent = nullptr;
BLECharacteristic* pESCRPM = nullptr;
BLECharacteristic* pESCTemps = nullptr;

struct EscTempsPacket {
  float mos_temp;
  float cap_temp;
  float mcu_temp;
  float motor_temp;
};

}  // namespace

void initEscBleService(BLEServer* server) {
  if (pEscService != nullptr) {
    return;
  }

  pEscService = server->createService(BLEUUID(ESC_TELEMETRY_SERVICE_UUID), 20);

  pESCVoltage = pEscService->createCharacteristic(
      BLEUUID(ESC_VOLTAGE_UUID), BLECharacteristic::PROPERTY_READ);

  pESCCurrent = pEscService->createCharacteristic(
      BLEUUID(ESC_CURRENT_UUID), BLECharacteristic::PROPERTY_READ);

  pESCRPM = pEscService->createCharacteristic(
      BLEUUID(ESC_RPM_UUID), BLECharacteristic::PROPERTY_READ);

  pESCTemps = pEscService->createCharacteristic(
      BLEUUID(ESC_TEMPS_UUID), BLECharacteristic::PROPERTY_READ);

  // Ensure deterministic startup values.
  float initialFloat = 0.0f;
  int32_t initialRpm = 0;

  if (pESCVoltage) pESCVoltage->setValue(initialFloat);
  if (pESCCurrent) pESCCurrent->setValue(initialFloat);
  if (pESCRPM) pESCRPM->setValue(initialRpm);
  if (pESCTemps) {
    EscTempsPacket temps = {0.0f, 0.0f, 0.0f, 0.0f};
    pESCTemps->setValue(reinterpret_cast<uint8_t*>(&temps), sizeof(temps));
  }

  pEscService->start();
}

void updateESCTelemetryBLE(const STR_ESC_TELEMETRY_140& telemetry) {
  if (pEscService == nullptr) {
    return;
  }

  float voltage = telemetry.volts;
  float current = telemetry.amps;
  int32_t rpm = static_cast<int32_t>(telemetry.eRPM);
  EscTempsPacket temps = {
    telemetry.mos_temp,
    telemetry.cap_temp,
    telemetry.mcu_temp,
    telemetry.motor_temp
  };

  if (pESCVoltage) pESCVoltage->setValue(voltage);
  if (pESCCurrent) pESCCurrent->setValue(current);
  if (pESCRPM) pESCRPM->setValue(rpm);
  if (pESCTemps) pESCTemps->setValue(reinterpret_cast<uint8_t*>(&temps), sizeof(temps));
}
