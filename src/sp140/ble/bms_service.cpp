#include <Arduino.h>

#include "sp140/ble/bms_service.h"

#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"
#include "sp140/ble/ble_utils.h"

namespace {

NimBLEService* pBmsService = nullptr;

// Binary packed telemetry characteristic (V1)
NimBLECharacteristic* pBMSPackedTelemetry = nullptr;

#ifndef DISABLE_LEGACY_BLE_TELEMETRY
// Legacy individual characteristics
NimBLECharacteristic* pBMSSOC = nullptr;
NimBLECharacteristic* pBMSVoltage = nullptr;
NimBLECharacteristic* pBMSCurrent = nullptr;
NimBLECharacteristic* pBMSPower = nullptr;
NimBLECharacteristic* pBMSHighCell = nullptr;
NimBLECharacteristic* pBMSLowCell = nullptr;
NimBLECharacteristic* pBMSHighTemp = nullptr;
NimBLECharacteristic* pBMSLowTemp = nullptr;
NimBLECharacteristic* pBMSFailureLevel = nullptr;
NimBLECharacteristic* pBMSVoltageDiff = nullptr;
NimBLECharacteristic* pBMSCellVoltages = nullptr;
NimBLECharacteristic* pBMSChargeMos = nullptr;
NimBLECharacteristic* pBMSDischargeMos = nullptr;
NimBLECharacteristic* pBMSTemperatures = nullptr;

// Track previous values to only notify on change
uint8_t lastFailureLevel = 0;
uint8_t lastChargeMos = 0;
uint8_t lastDischargeMos = 0;
#endif  // DISABLE_LEGACY_BLE_TELEMETRY

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

#ifndef DISABLE_LEGACY_BLE_TELEMETRY
  // Legacy individual characteristics
  pBMSSOC = pBmsService->createCharacteristic(
      NimBLEUUID(BMS_SOC_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pBMSVoltage = pBmsService->createCharacteristic(
      NimBLEUUID(BMS_VOLTAGE_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pBMSCurrent = pBmsService->createCharacteristic(
      NimBLEUUID(BMS_CURRENT_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pBMSPower = pBmsService->createCharacteristic(
      NimBLEUUID(BMS_POWER_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pBMSHighCell = pBmsService->createCharacteristic(
      NimBLEUUID(BMS_HIGH_CELL_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pBMSLowCell = pBmsService->createCharacteristic(
      NimBLEUUID(BMS_LOW_CELL_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pBMSHighTemp = pBmsService->createCharacteristic(
      NimBLEUUID(BMS_HIGH_TEMP_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pBMSLowTemp = pBmsService->createCharacteristic(
      NimBLEUUID(BMS_LOW_TEMP_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pBMSFailureLevel = pBmsService->createCharacteristic(
      NimBLEUUID(BMS_FAILURE_LEVEL_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pBMSVoltageDiff = pBmsService->createCharacteristic(
      NimBLEUUID(BMS_VOLTAGE_DIFF_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pBMSCellVoltages = pBmsService->createCharacteristic(
      NimBLEUUID(BMS_CELL_VOLTAGES_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pBMSChargeMos = pBmsService->createCharacteristic(
      NimBLEUUID(BMS_CHARGE_MOS_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pBMSDischargeMos = pBmsService->createCharacteristic(
      NimBLEUUID(BMS_DISCHARGE_MOS_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pBMSTemperatures = pBmsService->createCharacteristic(
      NimBLEUUID(BMS_TEMPERATURES_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  // Ensure characteristics have deterministic startup values.
  uint16_t initial_cell_values[BMS_CELLS_NUM] = {0};
  pBMSCellVoltages->setValue(
      reinterpret_cast<uint8_t*>(initial_cell_values),
      BMS_CELLS_NUM * sizeof(uint16_t));

  uint8_t initial_flag = 0;
  pBMSChargeMos->setValue(&initial_flag, sizeof(initial_flag));
  pBMSDischargeMos->setValue(&initial_flag, sizeof(initial_flag));

  uint8_t initial_temps[17] = {0};
  initial_temps[0] = 0x00;  // No valid sensors initially (bitmap = 0x00)
  pBMSTemperatures->setValue(initial_temps, 17);
#endif  // DISABLE_LEGACY_BLE_TELEMETRY

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

#ifndef DISABLE_LEGACY_BLE_TELEMETRY
// Legacy individual characteristic updates
void updateBMSTelemetry(const STR_BMS_TELEMETRY_140& telemetry) {
  if (pBmsService == nullptr) {
    return;  // Not initialized yet.
  }

  float soc = telemetry.soc;
  float voltage = telemetry.battery_voltage;
  float current = telemetry.battery_current;
  float power = telemetry.power;
  float highCell = telemetry.highest_cell_voltage;
  float lowCell = telemetry.lowest_cell_voltage;
  float highTemp = telemetry.highest_temperature;
  float lowTemp = telemetry.lowest_temperature;
  float voltageDiff = telemetry.voltage_differential;

  if (pBMSSOC) pBMSSOC->setValue(soc);
  if (pBMSVoltage) pBMSVoltage->setValue(voltage);
  if (pBMSCurrent) pBMSCurrent->setValue(current);
  if (pBMSPower) pBMSPower->setValue(power);
  if (pBMSHighCell) pBMSHighCell->setValue(highCell);
  if (pBMSLowCell) pBMSLowCell->setValue(lowCell);
  if (pBMSHighTemp) pBMSHighTemp->setValue(highTemp);
  if (pBMSLowTemp) pBMSLowTemp->setValue(lowTemp);
  if (pBMSVoltageDiff) pBMSVoltageDiff->setValue(voltageDiff);

  if (pBMSCellVoltages) {
    uint16_t cell_millivolts[BMS_CELLS_NUM];
    for (uint8_t i = 0; i < BMS_CELLS_NUM; i++) {
      cell_millivolts[i] = static_cast<uint16_t>(telemetry.cell_voltages[i] * 1000.0f);
    }
    pBMSCellVoltages->setValue(
        reinterpret_cast<uint8_t*>(cell_millivolts),
        BMS_CELLS_NUM * sizeof(uint16_t));
  }

  // Update temperatures characteristic
  // Format (17 bytes): bitmap(1) + 8×int16_t temperatures (deci-degrees C)
  // Sensor mapping: [0]=MOS, [1]=Balance, [2]=T1, [3]=T2, [4]=T3, [5]=T4, [6-7]=Reserved
  if (pBMSTemperatures) {
    uint8_t temp_buffer[17];
    uint8_t validBitmap = 0;

    // Bytes 1-16: 8×int16_t temperatures in deci-degrees C (0.1°C resolution)
    int16_t* temps = reinterpret_cast<int16_t*>(&temp_buffer[1]);
    const float sensorTemps[6] = {
      telemetry.mos_temperature,
      telemetry.balance_temperature,
      telemetry.t1_temperature,
      telemetry.t2_temperature,
      telemetry.t3_temperature,
      telemetry.t4_temperature
    };

    for (uint8_t i = 0; i < 6; i++) {
      bool isValid = !isnan(sensorTemps[i]);

      if (isValid) {
        validBitmap |= static_cast<uint8_t>(1U << i);
      }
      temps[i] = isValid ? static_cast<int16_t>(sensorTemps[i] * 10.0f) : 0;
    }

    temp_buffer[0] = validBitmap;
    temps[6] = 0;  // [6] Reserved
    temps[7] = 0;  // [7] Reserved

    pBMSTemperatures->setValue(temp_buffer, 17);
  }

  // Handle state-change characteristics - only notify on change
  setAndNotifyOnChange(pBMSFailureLevel, telemetry.battery_fail_level, lastFailureLevel);
  setAndNotifyOnChange(pBMSChargeMos, static_cast<uint8_t>(telemetry.is_charge_mos ? 1 : 0), lastChargeMos);
  setAndNotifyOnChange(pBMSDischargeMos, static_cast<uint8_t>(telemetry.is_discharge_mos ? 1 : 0), lastDischargeMos);

  if (!deviceConnected) {
    return;  // No notifications needed without a subscriber.
  }

  if (pBMSSOC) pBMSSOC->notify();
  if (pBMSVoltage) pBMSVoltage->notify();
  if (pBMSCurrent) pBMSCurrent->notify();
  if (pBMSPower) pBMSPower->notify();
  if (pBMSHighCell) pBMSHighCell->notify();
  if (pBMSLowCell) pBMSLowCell->notify();
  if (pBMSHighTemp) pBMSHighTemp->notify();
  if (pBMSLowTemp) pBMSLowTemp->notify();
  if (pBMSVoltageDiff) pBMSVoltageDiff->notify();
  if (pBMSTemperatures) pBMSTemperatures->notify();
  // Note: pBMSFailureLevel, pBMSChargeMos, pBMSDischargeMos notify separately above, only on change
}
#else
// Stub when legacy telemetry is disabled
void updateBMSTelemetry(const STR_BMS_TELEMETRY_140& telemetry) {
  (void)telemetry;  // Suppress unused parameter warning
}
#endif  // DISABLE_LEGACY_BLE_TELEMETRY
