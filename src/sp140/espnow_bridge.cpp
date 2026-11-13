#include "sp140/espnow_bridge.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <cstring>

#include "sp140/globals.h"
#include "sp140/telemetry_hub.h"
#include "sp140/wireless_settings.h"

static_assert(sizeof(BLE_FastLink_Telemetry) <= ESP_NOW_MAX_DATA_LEN,
              "Fast-Link telemetry no longer fits in one ESP-NOW packet");

namespace {

portMUX_TYPE s_bridgeMux = portMUX_INITIALIZER_UNLOCKED;
bool s_active = false;
volatile bool s_sendInFlight = false;
uint32_t s_sessionId = 0;
uint32_t s_txSequence = 0;

wireless::ControlPacket s_latestControl = {};
bool s_hasControl = false;
uint32_t s_controlReceivedMs = 0;
uint32_t s_receiverSessionId = 0;
uint32_t s_receiverLastSequence = 0;
bool s_receiverHasSequence = false;
bool s_failsafeLatched = true;

wireless::AckPacket s_latestAck = {};
bool s_ackPending = false;
uint32_t s_ackReceivedMs = 0;

BLE_FastLink_Telemetry s_latestTelemetry = {};
bool s_telemetryPending = false;
uint32_t s_telemetryReceivedMs = 0;

uint32_t s_lastAckSendMs = 0;
uint32_t s_lastTelemetrySendMs = 0;

bool sourceIsPeer(const uint8_t* mac) {
  return mac != nullptr &&
         memcmp(mac, wirelessSettings.peer_mac,
                sizeof(wirelessSettings.peer_mac)) == 0;
}

bool sendPacket(const void* data, size_t length) {
  if (!s_active || data == nullptr || length == 0 || s_sendInFlight) {
    return false;
  }
  s_sendInFlight = true;
  const esp_err_t result = esp_now_send(
      wirelessSettings.peer_mac, static_cast<const uint8_t*>(data), length);
  if (result != ESP_OK) {
    s_sendInFlight = false;
    return false;
  }
  return true;
}

void onEspNowSend(const uint8_t* mac, esp_now_send_status_t status) {
  (void)mac;
  (void)status;
  s_sendInFlight = false;
}

void receiveControl(const uint8_t* data, int length) {
  if (!wirelessIsReceiver() ||
      length != static_cast<int>(sizeof(wireless::ControlPacket))) {
    return;
  }

  wireless::ControlPacket packet = {};
  memcpy(&packet, data, sizeof(packet));
  if (!wireless::validate(packet)) return;

  portENTER_CRITICAL(&s_bridgeMux);
  const bool new_session = !s_receiverHasSequence ||
                           packet.session_id != s_receiverSessionId;
  // A new/rebooted remote is only admitted while explicitly disarmed. This
  // prevents a retained or replayed ARMED level from causing an automatic arm.
  if (new_session && packet.requested_state != DISARMED) {
    portEXIT_CRITICAL(&s_bridgeMux);
    return;
  }
  if (!new_session &&
      !wireless::sequenceNewer(packet.sequence, s_receiverLastSequence)) {
    portEXIT_CRITICAL(&s_bridgeMux);
    return;
  }

  if (new_session) {
    s_receiverSessionId = packet.session_id;
    s_receiverHasSequence = true;
    s_failsafeLatched = true;
  }
  s_receiverLastSequence = packet.sequence;
  s_latestControl = packet;
  s_controlReceivedMs = millis();
  s_hasControl = true;
  portEXIT_CRITICAL(&s_bridgeMux);
}

void receiveAck(const uint8_t* data, int length) {
  if (!wirelessIsRemote() ||
      length != static_cast<int>(sizeof(wireless::AckPacket))) {
    return;
  }
  wireless::AckPacket packet = {};
  memcpy(&packet, data, sizeof(packet));
  if (!wireless::validate(packet) || packet.session_id != s_sessionId) return;

  portENTER_CRITICAL(&s_bridgeMux);
  s_latestAck = packet;
  s_ackReceivedMs = millis();
  s_ackPending = true;
  portEXIT_CRITICAL(&s_bridgeMux);
}

void receiveTelemetry(const uint8_t* data, int length) {
  if (!wirelessIsRemote() ||
      length != static_cast<int>(sizeof(BLE_FastLink_Telemetry))) {
    return;
  }
  BLE_FastLink_Telemetry packet = {};
  memcpy(&packet, data, sizeof(packet));
  if (packet.version != FASTLINK_PROTOCOL_VERSION) return;

  portENTER_CRITICAL(&s_bridgeMux);
  s_latestTelemetry = packet;
  s_telemetryReceivedMs = millis();
  s_telemetryPending = true;
  portEXIT_CRITICAL(&s_bridgeMux);
}

void onEspNowReceive(const uint8_t* mac, const uint8_t* data, int length) {
  if (!sourceIsPeer(mac) || data == nullptr || length <= 0) return;
  if (wirelessIsReceiver()) {
    receiveControl(data, length);
  } else if (wirelessIsRemote()) {
    receiveAck(data, length);
    receiveTelemetry(data, length);
  }
}

}  // namespace

bool beginEspNowBridge() {
  if (wirelessSettings.role == WirelessRole::STANDALONE) return false;
  if (!wirelessSettings.peer_configured ||
      !wirelessSettings.secret_configured) {
    USBSerial.println("[ESPNOW] Role configured without peer/key; disabled");
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  if (esp_wifi_set_channel(wirelessSettings.channel,
                           WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    USBSerial.println("[ESPNOW] Failed to set Wi-Fi channel");
    return false;
  }
  if (esp_now_init() != ESP_OK) {
    USBSerial.println("[ESPNOW] esp_now_init failed");
    return false;
  }

  uint8_t pmk[ESP_NOW_KEY_LEN] = {};
  uint8_t lmk[ESP_NOW_KEY_LEN] = {};
  for (size_t i = 0; i < ESP_NOW_KEY_LEN; ++i) {
    pmk[i] = wirelessSettings.pairing_secret[i] ^ 0xA5U;
    lmk[i] = wirelessSettings.pairing_secret[i] ^ 0x5AU;
  }
  if (esp_now_set_pmk(pmk) != ESP_OK) {
    USBSerial.println("[ESPNOW] Failed to configure PMK");
    esp_now_deinit();
    return false;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, wirelessSettings.peer_mac, sizeof(peer.peer_addr));
  memcpy(peer.lmk, lmk, sizeof(peer.lmk));
  peer.channel = wirelessSettings.channel;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = true;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    USBSerial.println("[ESPNOW] Failed to add encrypted peer");
    esp_now_deinit();
    return false;
  }

  esp_now_register_send_cb(onEspNowSend);
  esp_now_register_recv_cb(onEspNowReceive);
  s_sessionId = esp_random();
  if (s_sessionId == 0) s_sessionId = 1;
  s_active = true;

  uint8_t local_mac[6] = {};
  esp_wifi_get_mac(WIFI_IF_STA, local_mac);
  USBSerial.printf(
      "[ESPNOW] role=%s channel=%u local=%02X:%02X:%02X:%02X:%02X:%02X\n",
      wirelessRoleName(wirelessSettings.role), wirelessSettings.channel,
      local_mac[0], local_mac[1], local_mac[2], local_mac[3], local_mac[4],
      local_mac[5]);
  return true;
}

bool espNowBridgeActive() { return s_active; }

bool espNowSendControl(uint16_t throttle_raw, DeviceState requested_state,
                       uint8_t performance_mode) {
  if (!wirelessIsRemote() || throttle_raw > wireless::kThrottleRawMax) {
    return false;
  }
  wireless::ControlPacket packet = {};
  packet.session_id = s_sessionId;
  packet.sequence = ++s_txSequence;
  packet.tx_uptime_ms = millis();
  packet.throttle_raw = throttle_raw;
  packet.requested_state = static_cast<uint8_t>(requested_state);
  packet.performance_mode = performance_mode;
  wireless::finalize(&packet);
  return sendPacket(&packet, sizeof(packet));
}

bool espNowTakeAck(wireless::AckPacket* ack) {
  if (ack == nullptr) return false;
  portENTER_CRITICAL(&s_bridgeMux);
  const bool available = s_ackPending;
  if (available) {
    *ack = s_latestAck;
    s_ackPending = false;
  }
  portEXIT_CRITICAL(&s_bridgeMux);
  return available;
}

bool espNowTakeTelemetry(BLE_FastLink_Telemetry* telemetry) {
  if (telemetry == nullptr) return false;
  portENTER_CRITICAL(&s_bridgeMux);
  const bool available = s_telemetryPending;
  if (available) {
    *telemetry = s_latestTelemetry;
    s_telemetryPending = false;
  }
  portEXIT_CRITICAL(&s_bridgeMux);
  return available;
}

void espNowApplyMirroredTelemetry(
    const BLE_FastLink_Telemetry& telemetry) {
  if (!wirelessIsRemote() ||
      telemetry.version != FASTLINK_PROTOCOL_VERSION) {
    return;
  }

  STR_ESC_TELEMETRY_140 esc = {};
  esc.escState = static_cast<TelemetryState>(telemetry.esc_status);
  esc.volts = telemetry.esc_volts_dV / 10.0f;
  esc.amps = telemetry.esc_amps_dA / 10.0f;
  esc.phase_current = telemetry.esc_phase_current_dA / 10.0f;
  esc.eRPM = telemetry.esc_rpm;
  esc.mos_temp = telemetry.esc_temp_mos_dC / 10.0f;
  esc.cap_temp = telemetry.esc_temp_cap_dC / 10.0f;
  esc.mcu_temp = telemetry.esc_temp_mcu_dC / 10.0f;
  esc.motor_temp = telemetry.esc_temp_motor_dC == kNoTempSensorDC
      ? NAN
      : telemetry.esc_temp_motor_dC / 10.0f;
  esc.inPWM = telemetry.esc_inPWM / 10.0f;
  esc.outPWM = telemetry.esc_outPWM;
  esc.comm_pwm = telemetry.esc_outPWM;
  esc.v_modulation = telemetry.esc_v_modulation;
  esc.running_error = telemetry.esc_error;
  esc.selfcheck_error = telemetry.esc_selfcheck;
  esc.hardware_id = telemetry.esc_hardware_id;
  esc.fw_version = telemetry.esc_fw_version;
  esc.bootloader_version = telemetry.esc_bootloader_version;
  esc.esc_runtime_ms = telemetry.esc_runtime_ms;
  esc.lastUpdateMs = millis();
  memcpy(esc.sn_code, telemetry.esc_sn_code, sizeof(esc.sn_code));

  STR_BMS_TELEMETRY_140 bms = {};
  bms.bmsState = static_cast<TelemetryState>(telemetry.bms_status);
  bms.soc = telemetry.bms_soc;
  bms.battery_voltage = telemetry.bms_volts_dV / 10.0f;
  bms.battery_current = telemetry.bms_amps_dA / 10.0f;
  bms.power = bms.battery_voltage * bms.battery_current / 1000.0f;
  bms.energy_cycle_ah = telemetry.bms_energy_cycle_mAh / 1000.0f;
  bms.battery_cycle = telemetry.bms_battery_cycle;
  bms.battery_fail_level = telemetry.bms_fail_level;
  bms.is_charging = telemetry.bms_is_charging != 0;
  bms.is_charge_mos = telemetry.bms_is_charge_mos != 0;
  bms.is_discharge_mos = telemetry.bms_is_discharge_mos != 0;
  bms.charge_wire_connected = telemetry.bms_charge_wire != 0;
  bms.low_soc_warning = telemetry.bms_low_soc_warning != 0;
  bms.battery_ready = telemetry.bms_battery_ready != 0;
  bms.highest_temperature = telemetry.bms_highest_temp_C;
  bms.lowest_temperature = telemetry.bms_lowest_temp_C;
  bms.highest_cell_voltage = telemetry.bms_cell_max_mV / 1000.0f;
  bms.lowest_cell_voltage = telemetry.bms_cell_min_mV / 1000.0f;
  bms.voltage_differential = telemetry.bms_voltage_diff_mV / 1000.0f;
  memcpy(bms.battery_id, telemetry.bms_battery_id,
         sizeof(bms.battery_id));
  bms.battery_id[sizeof(bms.battery_id) - 1] = '\0';
  bms.bms_type = telemetry.bms_type;
  for (size_t i = 0; i < BMS_CELLS_NUM; ++i) {
    bms.cell_voltages[i] = telemetry.bms_cell_voltages_mV[i] / 1000.0f;
  }
  bms.mos_temperature = telemetry.bms_temp_sensors_C[0];
  bms.balance_temperature = telemetry.bms_temp_sensors_C[1];
  bms.t1_temperature = telemetry.bms_temp_sensors_C[2];
  bms.t2_temperature = telemetry.bms_temp_sensors_C[3];
  bms.t3_temperature = telemetry.bms_temp_sensors_C[4];
  bms.t4_temperature = telemetry.bms_temp_sensors_C[5];
  bms.lastUpdateMs = millis();

  escTelemetryData = esc;
  bmsTelemetryData = bms;
  telemetryHubWriteEsc(esc);
  telemetryHubWriteBms(bms);
  telemetryHubWriteController(
      telemetry.altitude_cm / 100.0f, telemetry.baro_temp_dC / 10.0f,
      telemetry.baro_pressure_dHPa / 10.0f, telemetry.vario_cmps / 100.0f,
      telemetry.mcu_temp_dC / 10.0f, telemetry.pot_raw,
      telemetry.uptime_ms);

  if (bms.bmsState == TelemetryState::CONNECTED) {
    unifiedBatteryData = {bms.battery_voltage, bms.battery_current, bms.power,
                          bms.soc};
  } else if (esc.escState == TelemetryState::CONNECTED) {
    unifiedBatteryData = {
        esc.volts, esc.amps, esc.volts * esc.amps / 1000.0f, 0.0f};
  } else {
    unifiedBatteryData = {0.0f, 0.0f, 0.0f, 0.0f};
  }
}

void espNowMarkMirroredTelemetryDisconnected() {
  if (!wirelessIsRemote()) return;
  escTelemetryData.escState = TelemetryState::NOT_CONNECTED;
  bmsTelemetryData.bmsState = TelemetryState::NOT_CONNECTED;
  telemetryHubWriteEsc(escTelemetryData);
  telemetryHubWriteBms(bmsTelemetryData);
  unifiedBatteryData = {0.0f, 0.0f, 0.0f, 0.0f};
}

uint32_t espNowRemoteStatusAgeMs() {
  portENTER_CRITICAL(&s_bridgeMux);
  const uint32_t received_ms = s_ackReceivedMs;
  portEXIT_CRITICAL(&s_bridgeMux);
  return received_ms == 0 ? UINT32_MAX : millis() - received_ms;
}

uint32_t espNowRemoteTelemetryAgeMs() {
  portENTER_CRITICAL(&s_bridgeMux);
  const uint32_t received_ms = s_telemetryReceivedMs;
  portEXIT_CRITICAL(&s_bridgeMux);
  return received_ms == 0 ? UINT32_MAX : millis() - received_ms;
}

bool espNowGetLatestControl(wireless::ControlPacket* control,
                            uint32_t* age_ms) {
  if (control == nullptr || age_ms == nullptr) return false;
  portENTER_CRITICAL(&s_bridgeMux);
  const bool available = s_hasControl;
  if (available) {
    *control = s_latestControl;
    *age_ms = millis() - s_controlReceivedMs;
  }
  portEXIT_CRITICAL(&s_bridgeMux);
  return available;
}

void espNowSetFailsafeLatched(bool latched) {
  portENTER_CRITICAL(&s_bridgeMux);
  s_failsafeLatched = latched;
  portEXIT_CRITICAL(&s_bridgeMux);
}

bool espNowSendAck(DeviceState actual_state) {
  if (!wirelessIsReceiver()) return false;
  const uint32_t now = millis();
  if (now - s_lastAckSendMs < 40) return false;

  wireless::ControlPacket control = {};
  uint32_t age_ms = UINT32_MAX;
  if (!espNowGetLatestControl(&control, &age_ms)) return false;

  wireless::AckPacket ack = {};
  ack.session_id = control.session_id;
  ack.accepted_sequence = control.sequence;
  ack.rx_uptime_ms = now;
  ack.actual_state = static_cast<uint8_t>(actual_state);
  ack.flags = wireless::ACK_COMMAND_ACCEPTED;
  if (age_ms <= ESPNOW_CONTROL_TIMEOUT_MS) ack.flags |= wireless::ACK_LINK_READY;
  portENTER_CRITICAL(&s_bridgeMux);
  if (s_failsafeLatched) ack.flags |= wireless::ACK_FAILSAFE_LATCHED;
  portEXIT_CRITICAL(&s_bridgeMux);
  ack.command_age_ms = static_cast<uint16_t>(
      age_ms > UINT16_MAX ? UINT16_MAX : age_ms);
  wireless::finalize(&ack);
  if (!sendPacket(&ack, sizeof(ack))) return false;
  s_lastAckSendMs = now;
  return true;
}

bool espNowSendTelemetry(const BLE_FastLink_Telemetry& telemetry) {
  if (!wirelessIsReceiver()) return false;
  const uint32_t now = millis();
  if (now - s_lastTelemetrySendMs < 100) return false;
  if (!sendPacket(&telemetry, sizeof(telemetry))) return false;
  s_lastTelemetrySendMs = now;
  return true;
}
