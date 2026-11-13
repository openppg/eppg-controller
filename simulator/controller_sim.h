#pragma once

#include <cstdint>
#include <deque>
#include <iosfwd>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace eppg::sim {

class VirtualHardware;

enum class DeviceState : uint8_t { Disarmed = 0, Armed = 1, Cruising = 2 };

struct Event {
  uint64_t at_ms = 0;
  std::string action;
  std::string value;
  std::size_t source_line = 0;
};

struct TaskStats {
  uint64_t period_ms = 0;
  uint64_t runs = 0;
  uint64_t last_run_ms = 0;
};

struct Snapshot {
  uint64_t time_ms = 0;
  DeviceState state = DeviceState::Disarmed;
  uint16_t throttle_raw = 0;
  int throttle_pwm = 1000;
  float esc_voltage_v = 100.8f;
  float esc_rpm = 0.0f;
  float esc_amps = 0.0f;
  float esc_mos_temp_c = 25.0f;
  float bms_voltage_v = 97.2f;
  float bms_lowest_cell_v = 0.0f;
  float bms_highest_cell_v = 0.0f;
  float bms_current_a = 0.0f;
  float bms_soc_pct = 90.0f;
  float bms_temp_c = 25.0f;
  float altitude_m = 0.0f;
  float vertical_speed_mps = 0.0f;
  float pressure_hpa = 1013.25f;
  bool esc_connected = true;
  bool bms_connected = true;
  bool barometer_connected = true;
  uint64_t can_tx_frames = 0;
  uint64_t can_rx_frames = 0;
  uint64_t can_tx_bytes = 0;
  uint64_t can_rx_bytes = 0;
  uint64_t can_dropped_frames = 0;
  uint64_t can_corrupted_frames = 0;
  uint64_t can_bus_off_count = 0;
  uint64_t ble_notifications = 0;
  uint64_t watchdog_resets = 0;
  std::vector<std::string> alerts;
};

class ControllerSim {
 public:
  ControllerSim();
  ~ControllerSim();
  ControllerSim(const ControllerSim&) = delete;
  ControllerSim& operator=(const ControllerSim&) = delete;

  void schedule(Event event);
  void run(uint64_t duration_ms, std::ostream* trace = nullptr);
  const Snapshot& snapshot() const { return snapshot_; }
  const std::map<std::string, TaskStats>& task_stats() const { return tasks_; }
  const std::vector<std::string>& event_log() const { return event_log_; }
  std::string snapshot_json() const;

 private:
  void apply_events();
  void apply_event(const Event& event);
  void run_due_tasks(std::ostream* trace);
  void task_throttle();
  void task_ui(std::ostream* trace);
  void task_bms();
  void task_sensors();
  void task_monitor();
  void task_ble();
  void task_watchdog();
  void update_can_stats();
  void add_or_clear_alert(const std::string& name, bool active);
  void log(const std::string& message);
  static bool parse_bool(const Event& event);
  static double parse_number(const Event& event);

  Snapshot snapshot_;
  std::vector<Event> events_;
  std::size_t next_event_ = 0;
  std::map<std::string, TaskStats> tasks_;
  std::vector<std::string> event_log_;
  std::deque<std::pair<uint64_t, float>> altitude_history_;

  uint8_t performance_mode_ = 1;
  uint16_t cruise_pwm_ = 1035;
  uint16_t cruised_raw_ = 0;
  uint64_t cruised_at_ms_ = 0;
  int previous_pwm_ = 1035;
  float requested_altitude_m_ = 0.0f;
  float sea_level_pressure_hpa_ = 1013.25f;
  float cpu_temp_c_ = 42.0f;
  bool esc_link_requested_ = true;
  bool bms_link_requested_ = true;
  bool throttle_task_stalled_ = false;
  bool esc_link_established_ = false;
  bool bms_link_established_ = false;
  uint64_t esc_link_up_since_ms_ = 0;
  uint64_t bms_link_up_since_ms_ = 0;
  uint64_t esc_link_down_since_ms_ = 0;
  uint64_t bms_link_down_since_ms_ = 0;
  uint64_t last_trace_ms_ = UINT64_MAX;
  bool has_run_ = false;
  bool previous_esc_connected_ = false;
  bool previous_bms_connected_ = false;
  std::unique_ptr<VirtualHardware> hardware_;
};

std::vector<Event> load_scenario(const std::string& path);
const char* state_name(DeviceState state);

}  // namespace eppg::sim
