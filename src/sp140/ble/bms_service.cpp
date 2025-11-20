#include <Arduino.h>

#include "sp140/ble/bms_service.h"

#include "sp140/ble.h"
#include "sp140/ble/ble_ids.h"
#include "sp140/ble/ble_utils.h"

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

// Track previous values to only notify on change
uint8_t lastFailureLevel = 0;
uint8_t lastChargeMos = 0;
uint8_t lastDischargeMos = 0;

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
      BLEUUID(BMS_FAILURE_LEVEL_UUID), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pBMSFailureLevel->addDescriptor(new BLE2902());

  pBMSVoltageDiff = pBmsService->createCharacteristic(
      BLEUUID(BMS_VOLTAGE_DIFF_UUID), BLECharacteristic::PROPERTY_READ);

  pBMSCellVoltages = pBmsService->createCharacteristic(
      BLEUUID(BMS_CELL_VOLTAGES_UUID), BLECharacteristic::PROPERTY_READ);

  pBMSChargeMos = pBmsService->createCharacteristic(
      BLEUUID(BMS_CHARGE_MOS_UUID), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pBMSChargeMos->addDescriptor(new BLE2902());

  pBMSDischargeMos = pBmsService->createCharacteristic(
      BLEUUID(BMS_DISCHARGE_MOS_UUID), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pBMSDischargeMos->addDescriptor(new BLE2902());

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

    // Byte 0: Valid sensor bitmap (bit N = sensor N is valid)
    // 0b00111111 = sensors 0-5 valid (6 temperature sensors)
    temp_buffer[0] = 0b00111111;

    // Bytes 1-16: 8×int16_t temperatures in deci-degrees C (0.1°C resolution)
    int16_t* temps = reinterpret_cast<int16_t*>(&temp_buffer[1]);
    temps[0] = static_cast<int16_t>(telemetry.mos_temperature * 10.0f);      // [0] MOS
    temps[1] = static_cast<int16_t>(telemetry.balance_temperature * 10.0f);  // [1] Balance
    temps[2] = static_cast<int16_t>(telemetry.t1_temperature * 10.0f);       // [2] T1
    temps[3] = static_cast<int16_t>(telemetry.t2_temperature * 10.0f);       // [3] T2
    temps[4] = static_cast<int16_t>(telemetry.t3_temperature * 10.0f);       // [4] T3
    temps[5] = static_cast<int16_t>(telemetry.t4_temperature * 10.0f);       // [5] T4
    temps[6] = 0;  // [6] Reserved
    temps[7] = 0;  // [7] Reserved

    pBMSTemperatures->setValue(temp_buffer, 17);
  }

  // Handle state-change characteristics - only notify on change
  setAndNotifyOnChange(pBMSFailureLevel, telemetry.battery_failure_level, lastFailureLevel);
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
