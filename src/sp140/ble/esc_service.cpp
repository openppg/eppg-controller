#include <Arduino.h>

#include "sp140/ble/esc_service.h"

#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"

namespace {

NimBLEService* pEscService = nullptr;
NimBLECharacteristic* pESCVoltage = nullptr;
NimBLECharacteristic* pESCCurrent = nullptr;
NimBLECharacteristic* pESCRPM = nullptr;
NimBLECharacteristic* pESCTemps = nullptr;

struct EscTempsPacket {
  float mos_temp;
  float cap_temp;
  float mcu_temp;
  float motor_temp;
};

}  // namespace

void initEscBleService(NimBLEServer* server) {
  if (pEscService != nullptr) {
    return;
  }

  pEscService = server->createService(NimBLEUUID(ESC_TELEMETRY_SERVICE_UUID));

  pESCVoltage = pEscService->createCharacteristic(
      NimBLEUUID(ESC_VOLTAGE_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pESCCurrent = pEscService->createCharacteristic(
      NimBLEUUID(ESC_CURRENT_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pESCRPM = pEscService->createCharacteristic(
      NimBLEUUID(ESC_RPM_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pESCTemps = pEscService->createCharacteristic(
      NimBLEUUID(ESC_TEMPS_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

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

  if (!deviceConnected) {
    return;  // No notifications needed without a subscriber.
  }

  if (pESCVoltage) pESCVoltage->notify();
  if (pESCCurrent) pESCCurrent->notify();
  if (pESCRPM) pESCRPM->notify();
  if (pESCTemps) pESCTemps->notify();
}
