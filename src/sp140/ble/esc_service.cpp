#include <Arduino.h>

#include "sp140/ble/esc_service.h"

#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"

namespace {

NimBLEService* pEscService = nullptr;

// Binary packed telemetry characteristic (V1)
NimBLECharacteristic* pESCPackedTelemetry = nullptr;

#ifndef DISABLE_LEGACY_BLE_TELEMETRY
// Legacy individual characteristics
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
#endif  // DISABLE_LEGACY_BLE_TELEMETRY

}  // namespace

void initEscBleService(NimBLEServer* server) {
  if (pEscService != nullptr) {
    return;
  }

  pEscService = server->createService(NimBLEUUID(ESC_TELEMETRY_SERVICE_UUID));

  // Binary packed telemetry characteristic (always enabled)
  pESCPackedTelemetry = pEscService->createCharacteristic(
      NimBLEUUID(ESC_PACKED_TELEMETRY_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  // Initialize packed telemetry with zeros
  BLE_ESC_Telemetry_V1 initialPacket = {};
  initialPacket.version = 1;
  pESCPackedTelemetry->setValue(
      reinterpret_cast<uint8_t*>(&initialPacket),
      sizeof(BLE_ESC_Telemetry_V1));

#ifndef DISABLE_LEGACY_BLE_TELEMETRY
  // Legacy individual characteristics
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
#endif  // DISABLE_LEGACY_BLE_TELEMETRY

  pEscService->start();
}

// Binary packed telemetry update (V1 protocol)
void updateESCPackedTelemetry(const STR_ESC_TELEMETRY_140& telemetry) {
  if (pEscService == nullptr || pESCPackedTelemetry == nullptr) {
    return;
  }

  BLE_ESC_Telemetry_V1 packet;
  packet.version = 1;
  packet.connection_state = static_cast<uint8_t>(telemetry.escState);
  packet.volts = telemetry.volts;
  packet.amps = telemetry.amps;
  packet.mos_temp = telemetry.mos_temp;
  packet.cap_temp = telemetry.cap_temp;
  packet.mcu_temp = telemetry.mcu_temp;
  // motor_temp is NaN when sensor is disconnected/invalid.
  packet.motor_temp = telemetry.motor_temp;
  packet.eRPM = static_cast<int32_t>(telemetry.eRPM);
  packet.inPWM = static_cast<uint16_t>(telemetry.inPWM);
  packet.running_error = telemetry.running_error;
  packet.selfcheck_error = telemetry.selfcheck_error;
  packet.lastUpdateMs = static_cast<uint32_t>(telemetry.lastUpdateMs);

  pESCPackedTelemetry->setValue(
      reinterpret_cast<uint8_t*>(&packet),
      sizeof(BLE_ESC_Telemetry_V1));

  if (deviceConnected) {
    pESCPackedTelemetry->notify();
  }
}

#ifndef DISABLE_LEGACY_BLE_TELEMETRY
// Legacy individual characteristic updates
void updateESCTelemetryBLE(const STR_ESC_TELEMETRY_140& telemetry) {
  if (pEscService == nullptr) {
    return;
  }

  float voltage = telemetry.volts;
  float current = telemetry.amps;
  int32_t rpm = static_cast<int32_t>(telemetry.eRPM);
  // motor_temp is NaN when sensor is disconnected/invalid.
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
#else
// Stub when legacy telemetry is disabled
void updateESCTelemetryBLE(const STR_ESC_TELEMETRY_140& telemetry) {
  (void)telemetry;  // Suppress unused parameter warning
}
#endif  // DISABLE_LEGACY_BLE_TELEMETRY
