#include <cstddef>

#include <gtest/gtest.h>

#include "../native_stubs/Arduino.h"
#include "../../inc/sp140/ble/bms_packet_codec.h"
#include "../../src/sp140/ble/bms_packet_codec.cpp"

namespace {

STR_BMS_TELEMETRY_140 makeConnectedTelemetry() {
  STR_BMS_TELEMETRY_140 telemetry = {};
  telemetry.bmsState = TelemetryState::CONNECTED;
  telemetry.soc = 79.5f;
  telemetry.battery_voltage = 95.2f;
  telemetry.battery_current = 31.4f;
  telemetry.power = 2.99f;
  telemetry.highest_cell_voltage = 4.11f;
  telemetry.lowest_cell_voltage = 3.94f;
  telemetry.highest_temperature = 52.0f;
  telemetry.lowest_temperature = 24.0f;
  telemetry.energy_cycle = 4.8f;
  telemetry.battery_cycle = 112;
  telemetry.battery_fail_level = 2;
  telemetry.voltage_differential = 0.17f;
  telemetry.lastUpdateMs = 123456u;
  telemetry.is_charging = true;
  telemetry.is_charge_mos = true;
  telemetry.is_discharge_mos = false;
  for (uint8_t i = 0; i < BMS_CELLS_NUM; ++i) {
    telemetry.cell_voltages[i] = 3.90f + (static_cast<float>(i) * 0.01f);
  }
  telemetry.mos_temperature = 47.0f;
  telemetry.balance_temperature = 39.0f;
  telemetry.t1_temperature = 33.0f;
  telemetry.t2_temperature = 34.0f;
  telemetry.t3_temperature = 35.0f;
  telemetry.t4_temperature = 36.0f;
  return telemetry;
}

}  // namespace

TEST(BMSPacketCodec, ConnectedTelemetryMapsToPackedAndExtended) {
  STR_BMS_TELEMETRY_140 telemetry = makeConnectedTelemetry();

  BLE_BMS_Telemetry_V1 packed = buildBMSPackedTelemetryV1(telemetry, 3);
  BLE_BMS_Extended_Telemetry_V1 extended = buildBMSExtendedTelemetryV1(telemetry, 3);

  EXPECT_EQ(packed.version, 1);
  EXPECT_EQ(packed.bms_id, 3);
  EXPECT_EQ(packed.connection_state, static_cast<uint8_t>(TelemetryState::CONNECTED));
  EXPECT_FLOAT_EQ(packed.soc, telemetry.soc);
  EXPECT_FLOAT_EQ(packed.battery_voltage, telemetry.battery_voltage);
  EXPECT_FLOAT_EQ(packed.battery_current, telemetry.battery_current);
  EXPECT_FLOAT_EQ(packed.power, telemetry.power);
  EXPECT_FLOAT_EQ(packed.energy_cycle, telemetry.energy_cycle);

  EXPECT_EQ(extended.version, 1);
  EXPECT_EQ(extended.bms_id, 3);
  EXPECT_EQ(extended.connection_state, static_cast<uint8_t>(TelemetryState::CONNECTED));
  EXPECT_FLOAT_EQ(extended.soc, telemetry.soc);
  EXPECT_FLOAT_EQ(extended.battery_voltage, telemetry.battery_voltage);
  EXPECT_FLOAT_EQ(extended.battery_current, telemetry.battery_current);
  EXPECT_FLOAT_EQ(extended.power, telemetry.power);
  EXPECT_FLOAT_EQ(extended.cell_voltages[0], telemetry.cell_voltages[0]);
  EXPECT_FLOAT_EQ(extended.cell_voltages[BMS_CELLS_NUM - 1],
                  telemetry.cell_voltages[BMS_CELLS_NUM - 1]);
  EXPECT_FLOAT_EQ(extended.temperatures[0], telemetry.mos_temperature);
  EXPECT_FLOAT_EQ(extended.temperatures[1], telemetry.balance_temperature);
  EXPECT_FLOAT_EQ(extended.temperatures[2], telemetry.t1_temperature);
  EXPECT_FLOAT_EQ(extended.temperatures[3], telemetry.t2_temperature);
  EXPECT_FLOAT_EQ(extended.temperatures[4], telemetry.t3_temperature);
  EXPECT_FLOAT_EQ(extended.temperatures[5], telemetry.t4_temperature);
}

TEST(BMSPacketCodec, DisconnectedTelemetryKeepsLastKnownValues) {
  STR_BMS_TELEMETRY_140 telemetry = makeConnectedTelemetry();
  telemetry.bmsState = TelemetryState::NOT_CONNECTED;

  BLE_BMS_Telemetry_V1 packed = buildBMSPackedTelemetryV1(telemetry, 0);
  BLE_BMS_Extended_Telemetry_V1 extended = buildBMSExtendedTelemetryV1(telemetry, 0);

  EXPECT_EQ(packed.connection_state, static_cast<uint8_t>(TelemetryState::NOT_CONNECTED));
  EXPECT_FLOAT_EQ(packed.soc, telemetry.soc);
  EXPECT_FLOAT_EQ(packed.battery_voltage, telemetry.battery_voltage);
  EXPECT_FLOAT_EQ(packed.battery_current, telemetry.battery_current);
  EXPECT_FLOAT_EQ(packed.power, telemetry.power);
  EXPECT_FLOAT_EQ(packed.highest_cell_voltage, telemetry.highest_cell_voltage);
  EXPECT_FLOAT_EQ(packed.lowest_cell_voltage, telemetry.lowest_cell_voltage);
  EXPECT_FLOAT_EQ(packed.highest_temperature, telemetry.highest_temperature);
  EXPECT_FLOAT_EQ(packed.lowest_temperature, telemetry.lowest_temperature);
  EXPECT_FLOAT_EQ(packed.voltage_differential, telemetry.voltage_differential);
  EXPECT_FLOAT_EQ(packed.energy_cycle, telemetry.energy_cycle);

  EXPECT_EQ(extended.connection_state, static_cast<uint8_t>(TelemetryState::NOT_CONNECTED));
  EXPECT_FLOAT_EQ(extended.soc, telemetry.soc);
  EXPECT_FLOAT_EQ(extended.battery_voltage, telemetry.battery_voltage);
  EXPECT_FLOAT_EQ(extended.battery_current, telemetry.battery_current);
  EXPECT_FLOAT_EQ(extended.power, telemetry.power);
  EXPECT_FLOAT_EQ(extended.energy_cycle, telemetry.energy_cycle);
  for (uint8_t i = 0; i < BMS_CELLS_NUM; ++i) {
    EXPECT_FLOAT_EQ(extended.cell_voltages[i], telemetry.cell_voltages[i]);
  }
  const float expectedTemps[BMS_TEMPERATURE_SENSORS_NUM] = {
      telemetry.mos_temperature,
      telemetry.balance_temperature,
      telemetry.t1_temperature,
      telemetry.t2_temperature,
      telemetry.t3_temperature,
      telemetry.t4_temperature,
  };
  for (uint8_t i = 0; i < BMS_TEMPERATURE_SENSORS_NUM; ++i) {
    EXPECT_FLOAT_EQ(extended.temperatures[i], expectedTemps[i]);
  }
}

TEST(BMSPacketCodec, StructSizeExpectations) {
  EXPECT_EQ(sizeof(BLE_BMS_Telemetry_V1), static_cast<std::size_t>(55));
  EXPECT_EQ(sizeof(BLE_BMS_Extended_Telemetry_V1), static_cast<std::size_t>(155));
  EXPECT_LE(sizeof(BLE_BMS_Extended_Telemetry_V1), static_cast<std::size_t>(182));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    RUN_ALL_TESTS();
    return 0;
}
