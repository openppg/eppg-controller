#include "sp140/ble/bms_packet_codec.h"

namespace {

constexpr uint8_t kBmsTelemetryVersion = 1;

#if defined(ARDUINO_ARCH_ESP32)
static_assert(sizeof(unsigned long) == 4,
              "ESP32 build expects 32-bit unsigned long for telemetry timestamps");
#endif

void fillTemperatureArray(
    const STR_BMS_TELEMETRY_140& telemetry,
    float temperatures[BMS_TEMPERATURE_SENSORS_NUM]) {
  // Order is fixed for client parsing: [mos, balance, t1, t2, t3, t4]
  temperatures[0] = telemetry.mos_temperature;
  temperatures[1] = telemetry.balance_temperature;
  temperatures[2] = telemetry.t1_temperature;
  temperatures[3] = telemetry.t2_temperature;
  temperatures[4] = telemetry.t3_temperature;
  temperatures[5] = telemetry.t4_temperature;
}

}  // namespace

BLE_BMS_Telemetry_V1 buildBMSPackedTelemetryV1(
    const STR_BMS_TELEMETRY_140& telemetry, uint8_t bms_id) {
  BLE_BMS_Telemetry_V1 packet = {};
  packet.version = kBmsTelemetryVersion;
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

  return packet;
}

BLE_BMS_Extended_Telemetry_V1 buildBMSExtendedTelemetryV1(
    const STR_BMS_TELEMETRY_140& telemetry, uint8_t bms_id) {
  BLE_BMS_Extended_Telemetry_V1 packet = {};
  packet.version = kBmsTelemetryVersion;
  packet.bms_id = bms_id;
  packet.connection_state = static_cast<uint8_t>(telemetry.bmsState);
  packet.soc = telemetry.soc;
  packet.battery_voltage = telemetry.battery_voltage;
  packet.battery_current = telemetry.battery_current;
  packet.power = telemetry.power;
  packet.battery_fail_level = telemetry.battery_fail_level;
  packet.is_charge_mos = telemetry.is_charge_mos ? 1 : 0;
  packet.is_discharge_mos = telemetry.is_discharge_mos ? 1 : 0;
  packet.is_charging = telemetry.is_charging ? 1 : 0;
  packet.battery_cycle = telemetry.battery_cycle;
  packet.energy_cycle = telemetry.energy_cycle;
  packet.lastUpdateMs = static_cast<uint32_t>(telemetry.lastUpdateMs);

  for (uint8_t i = 0; i < BMS_CELLS_NUM; ++i) {
    packet.cell_voltages[i] = telemetry.cell_voltages[i];
  }
  fillTemperatureArray(telemetry, packet.temperatures);

  return packet;
}
