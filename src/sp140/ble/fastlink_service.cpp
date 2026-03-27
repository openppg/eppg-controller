#include "sp140/ble/fastlink_service.h"

#include <cmath>
#include <limits>

#include "sp140/ble.h"  // For deviceConnected
#include "sp140/ble/ble_ids.h"
#include "sp140/ble/ota_service.h"
#include "sp140/esc.h"  // For requestEscHardwareInfo()
#include <Arduino.h>
#include <NimBLECharacteristic.h>
#include <NimBLEDevice.h>

namespace {
NimBLECharacteristic *pFastLinkCharacteristic = nullptr;
uint32_t gFastLinkNotifyOkCount = 0;
uint32_t gFastLinkNotifyFailCount = 0;
uint32_t gFastLinkSkippedNoConnCount = 0;
uint32_t gFastLinkSkippedOtaCount = 0;
uint32_t gFastLinkLastStatsMs = 0;
uint32_t gFastLinkPacketId = 0;

constexpr uint32_t kFastLinkTelemetryProperties =
NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY |
NIMBLE_PROPERTY::READ_ENC;
constexpr uint32_t kFastLinkCommandProperties =
NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::WRITE_ENC;
constexpr int32_t kI8Min = std::numeric_limits<int8_t>::min();
constexpr int32_t kI8Max = std::numeric_limits<int8_t>::max();
constexpr int32_t kU8Max = std::numeric_limits<uint8_t>::max();
constexpr int32_t kI16Min = std::numeric_limits<int16_t>::min();
constexpr int32_t kI16Max = std::numeric_limits<int16_t>::max();
constexpr int32_t kU16Max = std::numeric_limits<uint16_t>::max();
constexpr int64_t kU32Max = std::numeric_limits<uint32_t>::max();

inline int8_t toI8(int32_t value) {
  return static_cast<int8_t>(constrain(value, kI8Min, kI8Max));
}

inline uint8_t toU8(int32_t value) {
  return static_cast<uint8_t>(constrain(value, 0, kU8Max));
}

inline int16_t toI16(int32_t value) {
  return static_cast<int16_t>(constrain(value, kI16Min, kI16Max));
}

inline uint16_t toU16(int32_t value) {
  return static_cast<uint16_t>(constrain(value, 0, kU16Max));
}

inline uint32_t toU32(int64_t value) {
  return static_cast<uint32_t>(constrain(value, 0LL, kU32Max));
}

// Float-to-fixed-point: multiply by scale, round, return as int32.
inline int32_t scaled(float value, float scale) {
  return static_cast<int32_t>(lroundf(value * scale));
}

BLE_FastLink_Telemetry buildFastLinkTelemetry(const TelemetryHub &hub,
                                              DeviceState deviceState) {
  BLE_FastLink_Telemetry fastLink = {};
  fastLink.version = FASTLINK_PROTOCOL_VERSION;
  fastLink.packet_id = gFastLinkPacketId++;
  fastLink.uptime_ms = millis();

  // Controller mapping (float -> fixed-point)
  fastLink.altitude_cm = scaled(hub.altitude, 100.0f);
  fastLink.baro_temp_dC = toI16(scaled(hub.baro_temp, 10.0f));
  fastLink.baro_pressure_dHPa = toU16(scaled(hub.baro_pressure, 10.0f));
  fastLink.vario_cmps = toI16(scaled(hub.vario, 100.0f));
  fastLink.mcu_temp_dC = toI16(scaled(hub.mcu_temp, 10.0f));
  fastLink.pot_raw = hub.pot_raw;
  fastLink.device_state = static_cast<uint8_t>(deviceState);

  // ESC mapping (float -> fixed-point, matches native CAN 0.1-unit resolution)
  fastLink.esc_status = static_cast<uint8_t>(hub.esc.escState);
  fastLink.esc_volts_dV = toU16(scaled(hub.esc.volts, 10.0f));
  fastLink.esc_amps_dA = toI16(scaled(hub.esc.amps, 10.0f));
  fastLink.esc_phase_current_dA = toI16(scaled(hub.esc.phase_current, 10.0f));
  fastLink.esc_rpm = hub.esc.eRPM;
  fastLink.esc_temp_mos_dC = toI16(scaled(hub.esc.mos_temp, 10.0f));
  fastLink.esc_temp_cap_dC = toI16(scaled(hub.esc.cap_temp, 10.0f));
  fastLink.esc_temp_mcu_dC = toI16(scaled(hub.esc.mcu_temp, 10.0f));
  fastLink.esc_temp_motor_dC = std::isnan(hub.esc.motor_temp)
      ? kNoTempSensorDC
      : toI16(scaled(hub.esc.motor_temp, 10.0f));
  fastLink.esc_inPWM = toU16(scaled(hub.esc.inPWM, 10.0f));
  fastLink.esc_outPWM = hub.esc.comm_pwm;
  fastLink.esc_v_modulation = hub.esc.v_modulation;
  fastLink.esc_error = hub.esc.running_error;
  fastLink.esc_selfcheck = hub.esc.selfcheck_error;
  fastLink.esc_hardware_id = hub.esc.hardware_id;
  fastLink.esc_fw_version = hub.esc.fw_version;
  fastLink.esc_bootloader_version = hub.esc.bootloader_version;
  memcpy(fastLink.esc_sn_code, hub.esc.sn_code, sizeof(fastLink.esc_sn_code));
  fastLink.esc_runtime_ms = hub.esc.esc_runtime_ms;

  // BMS mapping (float -> fixed-point, matches native CAN resolution)
  fastLink.bms_status = static_cast<uint8_t>(hub.bms.bmsState);
  fastLink.bms_soc = toU8(scaled(hub.bms.soc, 1.0f));
  fastLink.bms_volts_dV = toU16(scaled(hub.bms.battery_voltage, 10.0f));
  fastLink.bms_amps_dA = toI16(scaled(hub.bms.battery_current, 10.0f));
  fastLink.bms_energy_cycle_mAh = toU32(scaled(hub.bms.energy_cycle_ah, 1000.0f));
  fastLink.bms_battery_cycle = hub.bms.battery_cycle;
  fastLink.bms_fail_level = hub.bms.battery_fail_level;
  fastLink.bms_is_charging = hub.bms.is_charging;
  fastLink.bms_is_charge_mos = hub.bms.is_charge_mos;
  fastLink.bms_is_discharge_mos = hub.bms.is_discharge_mos;
  fastLink.bms_charge_wire = hub.bms.charge_wire_connected;
  fastLink.bms_low_soc_warning = hub.bms.low_soc_warning;
  fastLink.bms_battery_ready = hub.bms.battery_ready;
  fastLink.bms_highest_temp_C = toI8(scaled(hub.bms.highest_temperature, 1.0f));
  fastLink.bms_lowest_temp_C = toI8(scaled(hub.bms.lowest_temperature, 1.0f));
  fastLink.bms_cell_max_mV = toU16(scaled(hub.bms.highest_cell_voltage, 1000.0f));
  fastLink.bms_cell_min_mV = toU16(scaled(hub.bms.lowest_cell_voltage, 1000.0f));
  fastLink.bms_voltage_diff_mV = toU16(scaled(hub.bms.voltage_differential, 1000.0f));
  memcpy(fastLink.bms_battery_id, hub.bms.battery_id, sizeof(fastLink.bms_battery_id));
  fastLink.bms_type = hub.bms.bms_type;

  // Cell voltages: float V -> uint16_t mV (native 1 mV resolution)
  for (int i = 0; i < BMS_CELLS_NUM; i++) {
    fastLink.bms_cell_voltages_mV[i] = toU16(scaled(hub.bms.cell_voltages[i], 1000.0f));
  }

  // Temperature sensors: float C -> int8_t C (native 1C resolution)
  fastLink.bms_temp_sensors_C[0] = toI8(scaled(hub.bms.mos_temperature, 1.0f));
  fastLink.bms_temp_sensors_C[1] = toI8(scaled(hub.bms.balance_temperature, 1.0f));
  fastLink.bms_temp_sensors_C[2] = toI8(scaled(hub.bms.t1_temperature, 1.0f));
  fastLink.bms_temp_sensors_C[3] = toI8(scaled(hub.bms.t2_temperature, 1.0f));
  fastLink.bms_temp_sensors_C[4] = toI8(scaled(hub.bms.t3_temperature, 1.0f));
  fastLink.bms_temp_sensors_C[5] = toI8(scaled(hub.bms.t4_temperature, 1.0f));

  return fastLink;
}

void updateFastLinkTelemetry(const BLE_FastLink_Telemetry &data) {
  if (pFastLinkCharacteristic != nullptr) {
    pFastLinkCharacteristic->setValue((uint8_t *)&data,
                                      sizeof(BLE_FastLink_Telemetry));
    if (isOtaInProgress()) {
      ++gFastLinkSkippedOtaCount;
    } else if (deviceConnected) {
      const bool sent = pFastLinkCharacteristic->notify();
      if (sent) {
        ++gFastLinkNotifyOkCount;
      } else {
        ++gFastLinkNotifyFailCount;
      }
    } else {
      ++gFastLinkSkippedNoConnCount;
    }

    const uint32_t nowMs = millis();
    if (nowMs - gFastLinkLastStatsMs >= 2000) {
      gFastLinkLastStatsMs = nowMs;
      USBSerial.printf(
          "[FASTLINK] stats ok=%lu fail=%lu skippedNoConn=%lu skippedOta=%lu v=%u packet=%lu uptime=%lu connected=%d ota=%d\n",
          static_cast<unsigned long>(gFastLinkNotifyOkCount),
          static_cast<unsigned long>(gFastLinkNotifyFailCount),
          static_cast<unsigned long>(gFastLinkSkippedNoConnCount),
          static_cast<unsigned long>(gFastLinkSkippedOtaCount),
          static_cast<unsigned>(data.version),
          static_cast<unsigned long>(data.packet_id),
          static_cast<unsigned long>(data.uptime_ms), deviceConnected ? 1 : 0,
          isOtaInProgress() ? 1 : 0);
    }
  }
}

// Handles write commands from the app on FAST_LINK_COMMAND_UUID.
// Command 0x01: request ESC hardware info (HW ID, FW version, bootloader, serial).
class FastLinkCommandCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override {
    std::string val = pChar->getValue();
    if (val.empty()) return;
    switch (static_cast<uint8_t>(val[0])) {
      case 0x01:
        requestEscHardwareInfo();
        break;
      default:
        break;
    }
  }
};

class FastLinkTelemetryCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo,
                   uint16_t subValue) override {
    USBSerial.printf(
        "[FASTLINK] subscribe conn=%u addr=%s value=0x%02x notify=%u indicate=%u\n",
        connInfo.getConnHandle(), connInfo.getAddress().toString().c_str(),
        subValue, subValue & 0x01, (subValue >> 1) & 0x01);
  }
};

FastLinkCommandCallbacks commandCallbacks;
FastLinkTelemetryCallbacks telemetryCallbacks;
}  // namespace

void initFastLinkBleService(NimBLEServer *pServer) {
  if (pServer == nullptr)
    return;

  auto *pService = pServer->createService(FAST_LINK_TELEMETRY_SERVICE_UUID);

  pFastLinkCharacteristic = pService->createCharacteristic(
      FAST_LINK_TELEMETRY_UUID,
      kFastLinkTelemetryProperties);
  pFastLinkCharacteristic->setCallbacks(&telemetryCallbacks);

  // Set initial value so GATT reads before the first notify return a valid
  // version byte instead of all-zeros (which the app rejects as version 0).
  BLE_FastLink_Telemetry initialData = {};
  initialData.version = FASTLINK_PROTOCOL_VERSION;
  pFastLinkCharacteristic->setValue(
      reinterpret_cast<uint8_t *>(&initialData),
      sizeof(BLE_FastLink_Telemetry));

  // Write-without-response command channel: app sends 0x01 to request ESC hw info
  auto *pCommandCharacteristic = pService->createCharacteristic(
      FAST_LINK_COMMAND_UUID,
      kFastLinkCommandProperties);
  pCommandCharacteristic->setCallbacks(&commandCallbacks);
}

void publishFastLinkTelemetry(const TelemetryHub &hub, DeviceState deviceState) {
  updateFastLinkTelemetry(
      buildFastLinkTelemetry(hub, deviceState));
}
