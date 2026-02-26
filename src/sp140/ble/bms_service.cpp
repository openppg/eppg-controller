#include <Arduino.h>

#include "sp140/ble/bms_service.h"

#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"

namespace {

NimBLEService* pBmsService = nullptr;

// Binary packed telemetry characteristic (V1)
NimBLECharacteristic* pBMSPackedTelemetry = nullptr;
NimBLECharacteristic* pBMSExtendedTelemetry = nullptr;

}  // namespace

void initBmsBleService(NimBLEServer* server) {
  // Lazily guard against repeated init calls.
  if (pBmsService != nullptr) {
    return;
  }

  pBmsService = server->createService(NimBLEUUID(BMS_TELEMETRY_SERVICE_UUID));

  // Binary packed telemetry characteristic (always enabled)
  pBMSPackedTelemetry = pBmsService->createCharacteristic(
      NimBLEUUID(BMS_PACKED_TELEMETRY_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  // Initialize packed telemetry with zeros
  BLE_BMS_Telemetry_V1 initialPacket = {};
  initialPacket.version = 1;
  pBMSPackedTelemetry->setValue(
      reinterpret_cast<uint8_t*>(&initialPacket),
      sizeof(BLE_BMS_Telemetry_V1));

  // Extended BMS payload with all cell voltages (V2 compact format)
  pBMSExtendedTelemetry = pBmsService->createCharacteristic(
      NimBLEUUID(BMS_EXTENDED_TELEMETRY_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  BLE_BMS_Extended_V2 initialExtended = {};
  initialExtended.version = 2;
  pBMSExtendedTelemetry->setValue(
      reinterpret_cast<uint8_t*>(&initialExtended),
      sizeof(BLE_BMS_Extended_V2));

  pBmsService->start();
}

// Binary packed telemetry update (V1 protocol)
void updateBMSPackedTelemetry(const STR_BMS_TELEMETRY_140& telemetry, uint8_t bms_id) {
  if (pBmsService == nullptr || pBMSPackedTelemetry == nullptr) {
    return;
  }

  BLE_BMS_Telemetry_V1 packet;
  packet.version = 1;
  packet.bms_id = bms_id;
  packet.connection_state = static_cast<uint8_t>(telemetry.bmsState);
  packet.soc = telemetry.soc;
  packet.battery_voltage = telemetry.battery_voltage;
  packet.battery_current = telemetry.battery_current;
  packet.power = telemetry.power;
  packet.highest_cell_voltage = telemetry.highest_cell_voltage;
  packet.lowest_cell_voltage = telemetry.lowest_cell_voltage;
  packet.highest_temperature = telemetry.highest_temperature;
  packet.lowest_temperature = telemetry.lowest_temperature;
  packet.voltage_differential = telemetry.voltage_differential;
  packet.battery_fail_level = telemetry.battery_fail_level;
  packet.is_charge_mos = telemetry.is_charge_mos ? 1 : 0;
  packet.is_discharge_mos = telemetry.is_discharge_mos ? 1 : 0;
  packet.is_charging = telemetry.is_charging ? 1 : 0;
  packet.battery_cycle = telemetry.battery_cycle;
  packet.energy_cycle = telemetry.energy_cycle;
  packet.lastUpdateMs = static_cast<uint32_t>(telemetry.lastUpdateMs);

  pBMSPackedTelemetry->setValue(
      reinterpret_cast<uint8_t*>(&packet),
      sizeof(BLE_BMS_Telemetry_V1));

  if (deviceConnected) {
    pBMSPackedTelemetry->notify();
  }
}

void updateBMSExtendedTelemetry(const STR_BMS_TELEMETRY_140& telemetry, uint8_t bms_id) {
  if (pBmsService == nullptr || pBMSExtendedTelemetry == nullptr) {
    return;
  }

  // Keep BLE bandwidth predictable: extended packet at 1Hz.
  static unsigned long lastUpdateMs = 0;
  const unsigned long now = millis();
  if ((now - lastUpdateMs) < 1000ul) {
    return;
  }
  lastUpdateMs = now;

  BLE_BMS_Extended_V2 packet = {};
  packet.version = 2;
  packet.bms_id = bms_id;
  packet.lastUpdateMs = static_cast<uint32_t>(telemetry.lastUpdateMs);

  for (uint8_t i = 0; i < BLE_BMS_EXTENDED_CELL_COUNT; ++i) {
    float cellV = telemetry.cell_voltages[i];
    if (cellV < 0.0f) cellV = 0.0f;
    if (cellV > 65.0f) cellV = 65.0f;
    packet.cell_mV[i] = static_cast<uint16_t>(cellV * 1000.0f);
  }

  // Temps 0..5 are present in STR_BMS_TELEMETRY_140. Remaining channels are invalid.
  for (uint8_t i = 0; i < BLE_BMS_EXTENDED_TEMP_COUNT; ++i) {
    packet.temp_dC[i] = BLE_BMS_EXTENDED_INVALID_TEMP;
  }
  packet.temp_dC[0] = static_cast<int16_t>(telemetry.mos_temperature * 10.0f);
  packet.temp_dC[1] = static_cast<int16_t>(telemetry.balance_temperature * 10.0f);
  packet.temp_dC[2] = static_cast<int16_t>(telemetry.t1_temperature * 10.0f);
  packet.temp_dC[3] = static_cast<int16_t>(telemetry.t2_temperature * 10.0f);
  packet.temp_dC[4] = static_cast<int16_t>(telemetry.t3_temperature * 10.0f);
  packet.temp_dC[5] = static_cast<int16_t>(telemetry.t4_temperature * 10.0f);

  pBMSExtendedTelemetry->setValue(
      reinterpret_cast<uint8_t*>(&packet),
      sizeof(BLE_BMS_Extended_V2));

  if (deviceConnected) {
    pBMSExtendedTelemetry->notify();
  }
}
