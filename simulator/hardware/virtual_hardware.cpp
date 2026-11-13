#include "virtual_hardware.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <optional>
#include <vector>

#include "Adafruit_MCP2515.h"
#include "BMS_CAN.h"
#include "CanardAdapter.h"
#include "SineEsc.h"
#include "driver/twai.h"
#include "host_can.h"
#include "sine_esc.BroadcastStatusInfo.h"
#include "sine_esc.SetThrottleSettings2_req.h"
#include "sine_esc.SetThrottleSettings2_res.h"

namespace {

using eppg::sim::CanBusStats;
using eppg::sim::host_can::Frame;

std::deque<Frame> g_mcp_rx;
std::deque<Frame> g_twai_rx;
std::vector<Frame> g_twai_tx;

enum class Destination : std::uint8_t { Esc, Controller, BmsController };

struct ScheduledFrame {
  Frame frame;
  Destination destination = Destination::Controller;
  std::uint64_t ready_us = 0;
};

class VirtualCanBus {
 public:
  explicit VirtualCanBus(std::uint32_t bitrate) : bitrate_(bitrate) {}

  void submit(Frame frame, Destination destination, std::uint64_t now_us,
              bool physical_link) {
    ++sequence_;
    ++stats_.tx_frames;
    stats_.tx_bytes += frame.len;
    if (!physical_link || bus_off_ || now_us < recovery_until_us_ ||
        (drop_every_ != 0 && sequence_ % drop_every_ == 0)) {
      ++stats_.dropped_frames;
      return;
    }
    if (queue_.size() >= kQueueDepth) {
      ++stats_.queue_overflows;
      ++stats_.dropped_frames;
      return;
    }
    if (corrupt_every_ != 0 && sequence_ % corrupt_every_ == 0 &&
        frame.len != 0) {
      frame.data[frame.len / 2] ^= 0x01;
      ++stats_.corrupted_frames;
    }
    // Extended classic-CAN frame: SOF/arbitration/control/CRC/ACK/EOF and
    // intermission plus a conservative worst-normal bit-stuffing allowance.
    const std::uint64_t unstuffed_bits = 67 + 8ULL * frame.len;
    const std::uint64_t stuffed_bits = unstuffed_bits + (34 + 8ULL * frame.len) / 4;
    const std::uint64_t duration_us =
        std::max<std::uint64_t>(1, (stuffed_bits * 1000000ULL + bitrate_ - 1) /
                                      bitrate_);
    const std::uint64_t start_us = std::max(now_us, bus_free_us_);
    bus_free_us_ = start_us + duration_us;
    queue_.push_back({frame, destination, bus_free_us_});
  }

  template <typename Deliver>
  void advance(std::uint64_t now_us, Deliver deliver) {
    if (bus_off_ && now_us >= recovery_until_us_ && recovery_requested_) {
      bus_off_ = false;
      recovery_requested_ = false;
      bus_free_us_ = now_us;
    }
    while (!queue_.empty() && queue_.front().ready_us <= now_us) {
      ScheduledFrame item = queue_.front();
      queue_.pop_front();
      ++stats_.rx_frames;
      stats_.rx_bytes += item.frame.len;
      deliver(item);
    }
  }

  void set_drop_every(std::uint32_t n) { drop_every_ = n; }
  void set_corrupt_every(std::uint32_t n) { corrupt_every_ = n; }
  void set_bus_off(bool active, std::uint64_t now_us) {
    if (active) {
      if (!bus_off_) ++stats_.bus_off_count;
      bus_off_ = true;
      recovery_requested_ = false;
      stats_.dropped_frames += queue_.size();
      queue_.clear();
    } else if (bus_off_) {
      // ESP-IDF recovery is serviced by the firmware's 250 ms TWAI health poll.
      recovery_requested_ = true;
      recovery_until_us_ = now_us + 250000ULL;
    }
  }
  bool available(std::uint64_t now_us) const {
    return !bus_off_ && now_us >= recovery_until_us_;
  }
  const CanBusStats& stats() const { return stats_; }

 private:
  static constexpr std::size_t kQueueDepth = 64;
  std::uint32_t bitrate_;
  std::deque<ScheduledFrame> queue_;
  std::uint64_t bus_free_us_ = 0;
  std::uint64_t sequence_ = 0;
  std::uint64_t recovery_until_us_ = 0;
  std::uint32_t drop_every_ = 0;
  std::uint32_t corrupt_every_ = 0;
  bool bus_off_ = false;
  bool recovery_requested_ = false;
  CanBusStats stats_;
};

std::uint16_t clamp_u16(float value) {
  return static_cast<std::uint16_t>(
      std::clamp(std::lround(value), 0L, 65535L));
}

std::int16_t clamp_i16(float value) {
  return static_cast<std::int16_t>(
      std::clamp(std::lround(value),
                 static_cast<long>(std::numeric_limits<std::int16_t>::min()),
                 static_cast<long>(std::numeric_limits<std::int16_t>::max())));
}

}  // namespace

namespace eppg::sim::host_can {

void reset() {
  g_mcp_rx.clear();
  g_twai_rx.clear();
  g_twai_tx.clear();
}
void enqueue_mcp_rx(const Frame& frame) { g_mcp_rx.push_back(frame); }
void enqueue_twai_rx(const Frame& frame) { g_twai_rx.push_back(frame); }
std::vector<Frame> take_twai_tx() {
  std::vector<Frame> result;
  result.swap(g_twai_tx);
  return result;
}
std::size_t twai_rx_size() { return g_twai_rx.size(); }

}  // namespace eppg::sim::host_can

Adafruit_MCP2515::Adafruit_MCP2515(std::int8_t, SPIClass*) {}
Adafruit_MCP2515::Adafruit_MCP2515(std::int8_t, std::int8_t, std::int8_t,
                                   std::int8_t) {}
int Adafruit_MCP2515::begin(long) { return 1; }
int Adafruit_MCP2515::parsePacket() {
  if (g_mcp_rx.empty()) return 0;
  active_ = g_mcp_rx.front();
  g_mcp_rx.pop_front();
  read_index_ = 0;
  return active_.len;
}
std::uint32_t Adafruit_MCP2515::packetId() const { return active_.id; }
int Adafruit_MCP2515::read() {
  if (read_index_ >= active_.len) return -1;
  return active_.data[read_index_++];
}

extern "C" esp_err_t twai_read_alerts(uint32_t* alerts, TickType_t) {
  *alerts = g_twai_rx.empty() ? 0 : TWAI_ALERT_RX_DATA;
  return ESP_OK;
}
extern "C" esp_err_t twai_get_status_info(twai_status_info_t* status) {
  *status = {};
  status->msgs_to_rx = static_cast<std::uint32_t>(g_twai_rx.size());
  return ESP_OK;
}
extern "C" esp_err_t twai_receive(twai_message_t* message, TickType_t) {
  if (g_twai_rx.empty()) return ESP_FAIL;
  const Frame frame = g_twai_rx.front();
  g_twai_rx.pop_front();
  *message = {};
  message->identifier = frame.id;
  message->data_length_code = frame.len;
  message->extd = frame.extended;
  message->rtr = frame.rtr;
  std::copy_n(frame.data.begin(), frame.len, message->data);
  return ESP_OK;
}
extern "C" esp_err_t twai_transmit(const twai_message_t* message, TickType_t) {
  Frame frame;
  frame.id = message->identifier;
  frame.len = std::min<std::uint8_t>(message->data_length_code, 8);
  frame.extended = message->extd;
  frame.rtr = message->rtr;
  std::copy_n(message->data, frame.len, frame.data.begin());
  g_twai_tx.push_back(frame);
  return ESP_OK;
}

namespace eppg::sim {

struct VirtualHardware::Impl {
  Impl()
      : esc_bus(1000000),
        bms_bus(250000),
        controller_esc(controller_adapter),
        bms(5, 250000) {
    host_can::reset();
    controller_adapter.begin(controller_pool.data(), controller_pool.size());
    controller_adapter.setLocalNodeId(0x01);
    controller_esc.begin(0x20);
    controller_esc.clearModelReceiveFlags();
    canardInit(&esc_canard, esc_pool.data(), esc_pool.size(), esc_rx,
               esc_accept, this);
    canardSetLocalNodeID(&esc_canard, 0x20);
    bms.begin();
  }

  static bool esc_accept(const CanardInstance*, std::uint64_t* signature,
                         std::uint16_t data_type_id,
                         CanardTransferType transfer_type, std::uint8_t) {
    if (transfer_type == CanardTransferTypeRequest &&
        data_type_id == SINE_ESC_SETTHROTTLESETTINGS2_REQUEST_ID) {
      *signature = SINE_ESC_SETTHROTTLESETTINGS2_REQUEST_SIGNATURE;
      return true;
    }
    return false;
  }

  static void esc_rx(CanardInstance* instance, CanardRxTransfer* transfer) {
    auto* self = static_cast<Impl*>(instance->user_reference);
    if (transfer->transfer_type == CanardTransferTypeRequest &&
        transfer->data_type_id == SINE_ESC_SETTHROTTLESETTINGS2_REQUEST_ID) {
      sine_esc_SetThrottleSettings2Request request{};
      if (!sine_esc_SetThrottleSettings2Request_decode(transfer, &request)) {
        self->wire_pwm_01us = request.pwm_us;
        self->last_command_ms = self->now_ms;
        self->queue_esc_response(transfer->source_node_id,
                                 transfer->transfer_id);
      }
    }
  }

  void fill_response(sine_esc_SetThrottleSettings2Response& response) const {
    response.recv_pwm = wire_pwm_01us;
    response.comm_pwm = wire_pwm_01us;
    response.speed = static_cast<std::int32_t>(std::lround(plant_rpm));
    response.current = clamp_i16(plant_amps * 10.0f * 1.7f);
    response.bus_current = clamp_i16(plant_amps * 10.0f);
    response.voltage = clamp_u16(plant_voltage * 10.0f);
    response.v_modulation = clamp_u16(
        std::clamp((wire_pwm_01us - 10000) / 10000.0f, 0.0f, 1.0f) * 1000.0f);
    response.mos_temp = clamp_i16(plant_mos_temp * 10.0f);
    response.cap_temp = clamp_i16((plant_mos_temp - 3.0f) * 10.0f);
    response.mcu_temp = clamp_i16((plant_mos_temp - 7.0f) * 10.0f);
    response.motor_temp = clamp_i16((plant_mos_temp + 5.0f) * 10.0f);
    response.time_10ms = static_cast<std::uint16_t>(now_ms / 10ULL);
  }

  void queue_esc_response(std::uint8_t destination_node,
                          std::uint8_t transfer_id) {
    sine_esc_SetThrottleSettings2Response response{};
    fill_response(response);
    std::uint8_t payload[SINE_ESC_SETTHROTTLESETTINGS2_RESPONSE_MAX_SIZE]{};
    const std::uint32_t payload_len =
        sine_esc_SetThrottleSettings2Response_encode(&response, payload);
    response_transfer_id = transfer_id;
    CanardTxTransfer tx{};
    tx.transfer_type = CanardTransferTypeResponse;
    tx.data_type_signature = SINE_ESC_SETTHROTTLESETTINGS2_RESPONSE_SIGNATURE;
    tx.data_type_id = SINE_ESC_SETTHROTTLESETTINGS2_RESPONSE_ID;
    tx.inout_transfer_id = &response_transfer_id;
    tx.priority = CANARD_TRANSFER_PRIORITY_HIGHEST;
    tx.payload = payload;
    tx.payload_len = static_cast<std::uint16_t>(payload_len);
    canardRequestOrRespondObj(&esc_canard, destination_node, &tx);
  }

  void flush_esc_node_tx(std::uint64_t time_ms, bool link) {
    while (const CanardCANFrame* tx = canardPeekTxQueue(&esc_canard)) {
      Frame frame;
      frame.id = tx->id & CANARD_CAN_EXT_ID_MASK;
      frame.len = tx->data_len;
      frame.extended = true;
      std::copy_n(tx->data, tx->data_len, frame.data.begin());
      esc_bus.submit(frame, Destination::Controller, time_ms * 1000ULL, link);
      canardPopTxQueue(&esc_canard);
    }
  }

  void deliver_esc(const ScheduledFrame& item) {
    if (item.destination == Destination::Controller) {
      host_can::enqueue_twai_rx(item.frame);
      return;
    }
    CanardCANFrame frame{};
    frame.id = item.frame.id | CANARD_CAN_FRAME_EFF;
    frame.data_len = item.frame.len;
    std::copy_n(item.frame.data.begin(), item.frame.len, frame.data);
    canardHandleRxFrame(&esc_canard, &frame, item.ready_us);
  }

  void advance(std::uint64_t time_ms) {
    now_ms = time_ms;
    const std::uint64_t now_us = time_ms * 1000ULL;
    esc_bus.advance(now_us, [this](const ScheduledFrame& item) {
      deliver_esc(item);
    });
    // A request delivered above may have synchronously queued a service
    // response in libcanard. Put those frames onto the return half of the bus.
    flush_esc_node_tx(time_ms, esc_physical_link);
    bms_bus.advance(now_us, [](const ScheduledFrame& item) {
      if (item.destination == Destination::BmsController) {
        host_can::enqueue_mcp_rx(item.frame);
      }
    });
  }

  void send_esc_status(std::uint64_t time_ms, bool link) {
    sine_esc_BroadcastStatusInfo status{};
    status.recv_pwm = wire_pwm_01us;
    status.comm_pwm = wire_pwm_01us;
    status.speed = static_cast<std::int32_t>(std::lround(plant_rpm));
    status.current = clamp_i16(plant_amps * 10.0f * 1.7f);
    status.bus_current = clamp_i16(plant_amps * 10.0f);
    status.voltage = clamp_u16(plant_voltage * 10.0f);
    status.v_modulation = clamp_u16(
        std::clamp((wire_pwm_01us - 10000) / 10000.0f, 0.0f, 1.0f) * 1000.0f);
    status.mos_temp = clamp_i16(plant_mos_temp * 10.0f);
    status.cap_temp = clamp_i16((plant_mos_temp - 3.0f) * 10.0f);
    status.mcu_temp = clamp_i16((plant_mos_temp - 7.0f) * 10.0f);
    status.motor_temp = clamp_i16((plant_mos_temp + 5.0f) * 10.0f);
    status.time_10ms = static_cast<std::uint16_t>(time_ms / 10ULL);

    std::uint8_t payload[SINE_ESC_BROADCASTSTATUSINFO_MAX_SIZE]{};
    const std::uint32_t payload_len =
        sine_esc_BroadcastStatusInfo_encode(&status, payload);
    CanardTxTransfer transfer{};
    transfer.transfer_type = CanardTransferTypeBroadcast;
    transfer.data_type_signature = SINE_ESC_BROADCASTSTATUSINFO_SIGNATURE;
    transfer.data_type_id = SINE_ESC_BROADCASTSTATUSINFO_ID;
    transfer.inout_transfer_id = &status_transfer_id;
    transfer.priority = CANARD_TRANSFER_PRIORITY_MEDIUM;
    transfer.payload = payload;
    transfer.payload_len = static_cast<std::uint16_t>(payload_len);
    canardBroadcastObj(&esc_canard, &transfer);
    flush_esc_node_tx(time_ms, link);
  }

  void command(std::uint64_t time_ms, std::uint16_t pwm_us,
               float requested_mos_temp, bool link) {
    esc_physical_link = link;
    advance(time_ms);
    controller_adapter.processTxRxOnce();
    const auto& model = controller_esc.getModel();
    if (model.hasSetThrottleSettings2Response &&
        (!have_esc_rx ||
         model.setThrottleSettings2Response.time_10ms != last_esc_tick)) {
      const auto& status = model.setThrottleSettings2Response;
      have_esc_rx = true;
      last_esc_tick = status.time_10ms;
      esc_telemetry.last_rx_ms = time_ms;
      esc_telemetry.rpm = static_cast<float>(status.speed);
      esc_telemetry.amps = status.bus_current / 10.0f;
      esc_telemetry.voltage_v = status.voltage / 10.0f;
      esc_telemetry.mos_temp_c = status.mos_temp / 10.0f;
      esc_telemetry.commanded_pwm_01us = status.recv_pwm;
    }
    esc_telemetry.connected =
        have_esc_rx && time_ms - esc_telemetry.last_rx_ms <= 50;

    controller_esc.setThrottleSettings2(static_cast<std::uint16_t>(pwm_us * 10));
    controller_adapter.processTxRxOnce();
    for (const auto& frame : host_can::take_twai_tx()) {
      esc_bus.submit(frame, Destination::Esc, time_ms * 1000ULL, link);
    }

    plant_mos_temp = esc_temp_override.value_or(requested_mos_temp);
    const bool command_fresh = time_ms - last_command_ms <= 300;
    const float normalized = command_fresh
        ? std::clamp((wire_pwm_01us - 10000) / 9500.0f, 0.0f, 1.0f)
        : 0.0f;
    plant_rpm = esc_rpm_override.value_or(
        plant_rpm + (normalized * 12000.0f - plant_rpm) * 0.12f);
    plant_amps = esc_amps_override.value_or(
        plant_amps + (normalized * normalized * 120.0f - plant_amps) * 0.15f);
    plant_voltage = esc_voltage_override.value_or(100.8f - plant_amps * 0.012f);
    if (!esc_temp_override) {
      plant_mos_temp +=
          ((25.0f + normalized * 75.0f) - plant_mos_temp) * 0.0025f;
    }
    send_esc_status(time_ms, link);
  }

  static Frame bms_frame(std::uint32_t id,
                         std::initializer_list<std::uint8_t> bytes) {
    Frame frame;
    frame.id = id;
    frame.len = static_cast<std::uint8_t>(std::min<std::size_t>(bytes.size(), 8));
    std::copy_n(bytes.begin(), frame.len, frame.data.begin());
    return frame;
  }

  void submit_bms(Frame frame, std::uint64_t time_ms, bool link) {
    bms_bus.submit(frame, Destination::BmsController, time_ms * 1000ULL, link);
  }

  void update_battery(std::uint64_t time_ms, float load_amps,
                      float requested_soc, float requested_temp, bool link) {
    advance(time_ms);
    bms.update();
    if (bms.getLastMessageTime() != 0) {
      have_bms_rx = true;
      bms_telemetry.last_rx_ms = bms.getLastMessageTime();
      bms_telemetry.voltage_v = bms.getBatteryVoltage();
      bms_telemetry.current_a = bms.getBatteryCurrent();
      bms_telemetry.soc_pct = bms.getSOC();
      bms_telemetry.temperature_c = bms.getHighestTemperature();
      bms_telemetry.highest_cell_v = bms.getHighestCellVoltage();
      bms_telemetry.lowest_cell_v = bms.getLowestCellVoltage();
    }
    bms_telemetry.connected =
        have_bms_rx && time_ms - bms_telemetry.last_rx_ms <= 500;

    const float effective_load = bms_current_override.value_or(load_amps);
    const float delta_hours = 0.1f / 3600.0f;
    plant_soc = bms_soc_override.value_or(std::clamp(
        requested_soc - effective_load * delta_hours / 100.0f * 100.0f,
        0.0f, 100.0f));
    plant_bms_temp = bms_temp_override.value_or(requested_temp);
    const float cell_v = 3.0f + 1.2f * plant_soc / 100.0f;
    const float voltage = bms_voltage_override.value_or(
        24.0f * cell_v - effective_load * 0.012f);
    // ANT reports current around a 5000-count zero point. Positive values are
    // discharge/load current, matching the controller-facing parser result.
    const std::uint16_t current_raw =
        clamp_u16(5000.0f + effective_load * 10.0f);
    const std::uint16_t voltage_raw = clamp_u16(voltage * 10.0f);
    const std::uint16_t max_cell_mv = clamp_u16((cell_v + 0.012f) * 1000.0f);
    const std::uint16_t min_cell_mv = clamp_u16((cell_v - 0.010f) * 1000.0f);
    const std::uint8_t temp_raw = static_cast<std::uint8_t>(
        std::clamp(std::lround(plant_bms_temp + 40.0f), 0L, 255L));

    submit_bms(bms_frame(0x18FF28F4,
                         {0x18, static_cast<std::uint8_t>(std::lround(plant_soc)),
                          static_cast<std::uint8_t>(current_raw),
                          static_cast<std::uint8_t>(current_raw >> 8),
                          static_cast<std::uint8_t>(voltage_raw),
                          static_cast<std::uint8_t>(voltage_raw >> 8), 0, 0}),
               time_ms, link);
    submit_bms(bms_frame(0x18FE28F4,
                         {static_cast<std::uint8_t>(max_cell_mv),
                          static_cast<std::uint8_t>(max_cell_mv >> 8),
                          static_cast<std::uint8_t>(min_cell_mv),
                          static_cast<std::uint8_t>(min_cell_mv >> 8), temp_raw,
                          temp_raw, 0, 0}),
               time_ms, link);
    submit_bms(bms_frame(0x18B428F4,
                         {temp_raw, temp_raw, temp_raw, temp_raw, temp_raw,
                          temp_raw, temp_raw, temp_raw}),
               time_ms, link);
    for (std::uint32_t group = 0; group < 6; ++group) {
      std::array<std::uint8_t, 8> bytes{};
      for (std::size_t cell = 0; cell < 4; ++cell) {
        const std::uint16_t mv = clamp_u16(
            (cell_v + (static_cast<int>(cell + group) % 3 - 1) * 0.004f) *
            1000.0f);
        bytes[cell * 2] = static_cast<std::uint8_t>(mv);
        bytes[cell * 2 + 1] = static_cast<std::uint8_t>(mv >> 8);
      }
      Frame frame;
      frame.id = 0x18C828F4 + (group << 16);
      frame.len = 8;
      frame.data = bytes;
      submit_bms(frame, time_ms, link);
    }
  }

  VirtualCanBus esc_bus;
  VirtualCanBus bms_bus;
  std::array<std::uint8_t, 4096> controller_pool{};
  std::array<std::uint8_t, 4096> esc_pool{};
  CanardAdapter controller_adapter;
  SineEsc controller_esc;
  CanardInstance esc_canard{};
  BMS_CAN bms;
  EscWireTelemetry esc_telemetry;
  BmsWireTelemetry bms_telemetry;
  std::uint64_t now_ms = 0;
  std::uint64_t last_command_ms = 0;
  std::uint16_t wire_pwm_01us = 10000;
  std::uint16_t last_esc_tick = 0;
  std::uint8_t status_transfer_id = 0;
  std::uint8_t response_transfer_id = 0;
  float plant_rpm = 0.0f;
  float plant_amps = 0.0f;
  float plant_voltage = 100.8f;
  float plant_mos_temp = 25.0f;
  float plant_soc = 90.0f;
  float plant_bms_temp = 25.0f;
  std::optional<float> esc_rpm_override;
  std::optional<float> esc_amps_override;
  std::optional<float> esc_voltage_override;
  std::optional<float> esc_temp_override;
  std::optional<float> bms_voltage_override;
  std::optional<float> bms_current_override;
  std::optional<float> bms_soc_override;
  std::optional<float> bms_temp_override;
  bool have_esc_rx = false;
  bool have_bms_rx = false;
  bool esc_physical_link = true;
};

VirtualHardware::VirtualHardware() : impl_(std::make_unique<Impl>()) {}
VirtualHardware::~VirtualHardware() = default;
void VirtualHardware::advance(std::uint64_t now_ms) { impl_->advance(now_ms); }
void VirtualHardware::command_esc(std::uint64_t now_ms, std::uint16_t pwm_us,
                                  float requested_mos_temp_c,
                                  bool physical_link) {
  impl_->command(now_ms, pwm_us, requested_mos_temp_c, physical_link);
}
void VirtualHardware::update_bms(std::uint64_t now_ms, float load_amps,
                                 float requested_soc,
                                 float requested_temp_c, bool physical_link) {
  impl_->update_battery(now_ms, load_amps, requested_soc, requested_temp_c,
                        physical_link);
}
void VirtualHardware::set_esc_drop_every(std::uint32_t n) {
  impl_->esc_bus.set_drop_every(n);
}
void VirtualHardware::set_bms_drop_every(std::uint32_t n) {
  impl_->bms_bus.set_drop_every(n);
}
void VirtualHardware::set_esc_corrupt_every(std::uint32_t n) {
  impl_->esc_bus.set_corrupt_every(n);
}
void VirtualHardware::set_bms_corrupt_every(std::uint32_t n) {
  impl_->bms_bus.set_corrupt_every(n);
}
void VirtualHardware::set_esc_bus_off(bool active, std::uint64_t now_ms) {
  impl_->esc_bus.set_bus_off(active, now_ms * 1000ULL);
}
void VirtualHardware::set_bms_bus_off(bool active, std::uint64_t now_ms) {
  impl_->bms_bus.set_bus_off(active, now_ms * 1000ULL);
}
void VirtualHardware::set_esc_rpm(float rpm) { impl_->esc_rpm_override = rpm; }
void VirtualHardware::set_esc_amps(float amps) { impl_->esc_amps_override = amps; }
void VirtualHardware::set_esc_voltage(float voltage_v) {
  impl_->esc_voltage_override = voltage_v;
}
void VirtualHardware::set_esc_temperature(float temperature_c) {
  impl_->esc_temp_override = temperature_c;
}
void VirtualHardware::set_bms_voltage(float voltage_v) {
  impl_->bms_voltage_override = voltage_v;
}
void VirtualHardware::set_bms_current(float current_a) {
  impl_->bms_current_override = current_a;
}
void VirtualHardware::set_bms_soc(float soc_pct) {
  impl_->bms_soc_override = soc_pct;
}
void VirtualHardware::set_bms_temperature(float temperature_c) {
  impl_->bms_temp_override = temperature_c;
}
const EscWireTelemetry& VirtualHardware::esc() const {
  return impl_->esc_telemetry;
}
const BmsWireTelemetry& VirtualHardware::bms() const {
  return impl_->bms_telemetry;
}
const CanBusStats& VirtualHardware::esc_stats() const {
  return impl_->esc_bus.stats();
}
const CanBusStats& VirtualHardware::bms_stats() const {
  return impl_->bms_bus.stats();
}

}  // namespace eppg::sim
