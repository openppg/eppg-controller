#include <Arduino.h>

#include "sp140/ble/bms_service.h"

#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"

namespace {

NimBLEService* pBmsService = nullptr;

// Binary packed telemetry characteristic (V1)
NimBLECharacteristic* pBMSPackedTelemetry = nullptr;

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
