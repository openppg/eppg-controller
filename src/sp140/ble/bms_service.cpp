#include <Arduino.h>

#include "sp140/ble/bms_service.h"

#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"

namespace {

BLEService* pBmsService = nullptr;
BLECharacteristic* pBMSSOC = nullptr;
BLECharacteristic* pBMSVoltage = nullptr;
BLECharacteristic* pBMSCurrent = nullptr;
BLECharacteristic* pBMSPower = nullptr;
BLECharacteristic* pBMSHighCell = nullptr;
BLECharacteristic* pBMSLowCell = nullptr;
BLECharacteristic* pBMSHighTemp = nullptr;
BLECharacteristic* pBMSLowTemp = nullptr;
BLECharacteristic* pBMSFailureLevel = nullptr;
BLECharacteristic* pBMSVoltageDiff = nullptr;
BLECharacteristic* pBMSCellVoltages = nullptr;
BLECharacteristic* pBMSChargeMos = nullptr;
BLECharacteristic* pBMSDischargeMos = nullptr;
BLECharacteristic* pBMSTemperatures = nullptr;

}  // namespace

void initBmsBleService(BLEServer* server) {
  // Lazily guard against repeated init calls.
  if (pBmsService != nullptr) {
    return;
  }

  pBmsService = server->createService(BLEUUID(BMS_TELEMETRY_SERVICE_UUID), 30);

  pBMSSOC = pBmsService->createCharacteristic(
      BLEUUID(BMS_SOC_UUID), BLECharacteristic::PROPERTY_READ);

  pBMSVoltage = pBmsService->createCharacteristic(
      BLEUUID(BMS_VOLTAGE_UUID), BLECharacteristic::PROPERTY_READ);

  pBMSCurrent = pBmsService->createCharacteristic(
      BLEUUID(BMS_CURRENT_UUID), BLECharacteristic::PROPERTY_READ);

  pBMSPower = pBmsService->createCharacteristic(
      BLEUUID(BMS_POWER_UUID), BLECharacteristic::PROPERTY_READ);

  pBMSHighCell = pBmsService->createCharacteristic(
      BLEUUID(BMS_HIGH_CELL_UUID), BLECharacteristic::PROPERTY_READ);

  pBMSLowCell = pBmsService->createCharacteristic(
      BLEUUID(BMS_LOW_CELL_UUID), BLECharacteristic::PROPERTY_READ);

  pBMSHighTemp = pBmsService->createCharacteristic(
      BLEUUID(BMS_HIGH_TEMP_UUID), BLECharacteristic::PROPERTY_READ);

  pBMSLowTemp = pBmsService->createCharacteristic(
      BLEUUID(BMS_LOW_TEMP_UUID), BLECharacteristic::PROPERTY_READ);

  pBMSFailureLevel = pBmsService->createCharacteristic(
      BLEUUID(BMS_FAILURE_LEVEL_UUID), BLECharacteristic::PROPERTY_READ);

  pBMSVoltageDiff = pBmsService->createCharacteristic(
      BLEUUID(BMS_VOLTAGE_DIFF_UUID), BLECharacteristic::PROPERTY_READ);

  pBMSCellVoltages = pBmsService->createCharacteristic(
      BLEUUID(BMS_CELL_VOLTAGES_UUID), BLECharacteristic::PROPERTY_READ);

  pBMSChargeMos = pBmsService->createCharacteristic(
      BLEUUID(BMS_CHARGE_MOS_UUID), BLECharacteristic::PROPERTY_READ);

  pBMSDischargeMos = pBmsService->createCharacteristic(
      BLEUUID(BMS_DISCHARGE_MOS_UUID), BLECharacteristic::PROPERTY_READ);

  pBMSTemperatures = pBmsService->createCharacteristic(
      BLEUUID(BMS_TEMPERATURES_UUID),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

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

  pBmsService->start();
}

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
  if (pBMSFailureLevel) {
    uint8_t failureLevel = telemetry.battery_failure_level;
    pBMSFailureLevel->setValue(&failureLevel, sizeof(failureLevel));
  }
  if (pBMSVoltageDiff) pBMSVoltageDiff->setValue(voltageDiff);

  if (pBMSChargeMos) {
    uint8_t chargeMos = telemetry.is_charge_mos ? 1 : 0;
    pBMSChargeMos->setValue(&chargeMos, sizeof(chargeMos));
  }

  if (pBMSDischargeMos) {
    uint8_t dischargeMos = telemetry.is_discharge_mos ? 1 : 0;
    pBMSDischargeMos->setValue(&dischargeMos, sizeof(dischargeMos));
  }

  if (pBMSCellVoltages) {
    uint16_t cell_millivolts[BMS_CELLS_NUM];
    for (uint8_t i = 0; i < BMS_CELLS_NUM; i++) {
      cell_millivolts[i] = static_cast<uint16_t>(telemetry.cell_voltages[i] * 1000.0f);
    }
    pBMSCellVoltages->setValue(
        reinterpret_cast<uint8_t*>(cell_millivolts),
        BMS_CELLS_NUM * sizeof(uint16_t));
  }

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
  if (pBMSFailureLevel) pBMSFailureLevel->notify();
  if (pBMSVoltageDiff) pBMSVoltageDiff->notify();
  if (pBMSChargeMos) pBMSChargeMos->notify();
  if (pBMSDischargeMos) pBMSDischargeMos->notify();
}
