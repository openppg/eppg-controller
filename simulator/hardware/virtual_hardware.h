#pragma once

#include <cstdint>
#include <memory>

namespace eppg::sim {

struct CanBusStats {
  std::uint64_t tx_frames = 0;
  std::uint64_t rx_frames = 0;
  std::uint64_t tx_bytes = 0;
  std::uint64_t rx_bytes = 0;
  std::uint64_t dropped_frames = 0;
  std::uint64_t corrupted_frames = 0;
  std::uint64_t queue_overflows = 0;
  std::uint64_t bus_off_count = 0;
};

struct EscWireTelemetry {
  bool connected = false;
  float voltage_v = 0.0f;
  float rpm = 0.0f;
  float amps = 0.0f;
  float mos_temp_c = 25.0f;
  std::uint16_t commanded_pwm_01us = 10000;
  std::uint64_t last_rx_ms = 0;
};

struct BmsWireTelemetry {
  bool connected = false;
  float voltage_v = 0.0f;
  float current_a = 0.0f;
  float soc_pct = 0.0f;
  float temperature_c = 25.0f;
  float highest_cell_v = 0.0f;
  float lowest_cell_v = 0.0f;
  std::uint64_t last_rx_ms = 0;
};

// Owns two independent classic-CAN networks: ESP32-S3 TWAI <-> SINE ESC at
// 1 Mbit/s and MCP2515 <-> ANT BMS at 250 kbit/s. Vendor protocol code is
// compiled into the implementation unchanged.
class VirtualHardware {
 public:
  VirtualHardware();
  ~VirtualHardware();
  VirtualHardware(const VirtualHardware&) = delete;
  VirtualHardware& operator=(const VirtualHardware&) = delete;

  void advance(std::uint64_t now_ms);
  void command_esc(std::uint64_t now_ms, std::uint16_t pwm_us,
                   float requested_mos_temp_c, bool physical_link);
  void update_bms(std::uint64_t now_ms, float load_amps, float requested_soc,
                  float requested_temp_c, bool physical_link);

  void set_esc_drop_every(std::uint32_t n);
  void set_bms_drop_every(std::uint32_t n);
  void set_esc_corrupt_every(std::uint32_t n);
  void set_bms_corrupt_every(std::uint32_t n);
  void set_esc_bus_off(bool active, std::uint64_t now_ms);
  void set_bms_bus_off(bool active, std::uint64_t now_ms);
  void set_esc_rpm(float rpm);
  void set_esc_amps(float amps);
  void set_esc_voltage(float voltage_v);
  void set_esc_temperature(float temperature_c);
  void set_bms_voltage(float voltage_v);
  void set_bms_current(float current_a);
  void set_bms_soc(float soc_pct);
  void set_bms_temperature(float temperature_c);

  const EscWireTelemetry& esc() const;
  const BmsWireTelemetry& bms() const;
  const CanBusStats& esc_stats() const;
  const CanBusStats& bms_stats() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace eppg::sim
