#include "controller_sim.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "sp140/esp32s3-config.h"
#include "sp140/shared-config.h"
#include "sp140/throttle.h"
#include "hardware/virtual_hardware.h"

HardwareConfig board_config{};

namespace eppg::sim {

uint64_t g_virtual_time_ms = 0;
int g_analog_read_value = 0;

namespace {

std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::string json_escape(const std::string& value) {
  std::ostringstream out;
  for (const char ch : value) {
    switch (ch) {
      case '\\': out << "\\\\"; break;
      case '"': out << "\\\""; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default: out << ch; break;
    }
  }
  return out.str();
}

}  // namespace

const char* state_name(DeviceState state) {
  switch (state) {
    case DeviceState::Disarmed: return "DISARMED";
    case DeviceState::Armed: return "ARMED";
    case DeviceState::Cruising: return "ARMED_CRUISING";
  }
  return "UNKNOWN";
}

ControllerSim::ControllerSim() : hardware_(std::make_unique<VirtualHardware>()) {
  tasks_ = {
      {"throttle", {20, 0, 0}}, {"ui", {33, 0, 0}},
      {"bms", {100, 0, 0}},     {"sensors", {100, 0, 0}},
      {"monitor", {100, 0, 0}}, {"ble", {20, 0, 0}},
      {"watchdog", {100, 0, 0}},
  };
  board_config.throttle_pin = 7;
  resetThrottleState(previous_pwm_);
}

ControllerSim::~ControllerSim() = default;

void ControllerSim::schedule(Event event) {
  if (has_run_ && event.at_ms <= snapshot_.time_ms) {
    throw std::runtime_error("cannot schedule an event at or before current "
                             "simulator time");
  }
  events_.push_back(std::move(event));
  std::stable_sort(events_.begin(), events_.end(),
                   [](const Event& lhs, const Event& rhs) {
                     return lhs.at_ms < rhs.at_ms;
                   });
}

void ControllerSim::run(uint64_t duration_ms, std::ostream* trace) {
  if (duration_ms < snapshot_.time_ms) {
    throw std::runtime_error("duration precedes current simulator time");
  }
  if (has_run_) ++snapshot_.time_ms;
  while (snapshot_.time_ms <= duration_ms) {
    g_virtual_time_ms = snapshot_.time_ms;
    hardware_->advance(snapshot_.time_ms);
    apply_events();
    run_due_tasks(trace);
    ++snapshot_.time_ms;
  }
  snapshot_.time_ms = duration_ms;
  g_virtual_time_ms = duration_ms;
  has_run_ = true;
}

void ControllerSim::apply_events() {
  while (next_event_ < events_.size() &&
         events_[next_event_].at_ms == snapshot_.time_ms) {
    apply_event(events_[next_event_++]);
  }
}

double ControllerSim::parse_number(const Event& event) {
  try {
    std::size_t used = 0;
    const double result = std::stod(event.value, &used);
    if (used != event.value.size() || !std::isfinite(result)) {
      throw std::invalid_argument("invalid");
    }
    return result;
  } catch (const std::exception&) {
    throw std::runtime_error("scenario line " +
                             std::to_string(event.source_line) +
                             ": invalid number '" + event.value + "'");
  }
}

bool ControllerSim::parse_bool(const Event& event) {
  if (event.value == "1" || event.value == "true" || event.value == "on") {
    return true;
  }
  if (event.value == "0" || event.value == "false" || event.value == "off") {
    return false;
  }
  throw std::runtime_error("scenario line " + std::to_string(event.source_line) +
                           ": expected boolean, got '" + event.value + "'");
}

void ControllerSim::apply_event(const Event& event) {
  const auto parse_float = [&event]() {
    const double number = parse_number(event);
    if (number < -std::numeric_limits<float>::max() ||
        number > std::numeric_limits<float>::max()) {
      throw std::runtime_error("scenario line " +
                               std::to_string(event.source_line) +
                               ": number is outside the supported range");
    }
    return static_cast<float>(number);
  };

  if (event.action == "throttle") {
    snapshot_.throttle_raw = static_cast<uint16_t>(
        std::clamp(parse_number(event), 0.0, 4095.0));
  } else if (event.action == "arm") {
    if (parse_bool(event)) {
      if (snapshot_.throttle_raw >= static_cast<uint16_t>(0.05 * 4095)) {
        log("arm rejected: throttle is above engagement floor");
      } else {
        snapshot_.state = DeviceState::Armed;
        resetThrottleState(previous_pwm_);
        log("state -> ARMED");
      }
    } else {
      snapshot_.state = DeviceState::Disarmed;
      snapshot_.throttle_pwm = ESC_DISARMED_PWM;
      resetThrottleState(previous_pwm_);
      log("state -> DISARMED");
    }
  } else if (event.action == "cruise") {
    const bool enabled = parse_bool(event);
    if (enabled && snapshot_.state == DeviceState::Armed) {
      if (!isPotInCruiseActivationRange(
              snapshot_.throttle_raw,
              static_cast<uint16_t>(0.05 * 4095), 0.70f)) {
        log("cruise rejected: throttle is outside activation range");
      } else {
        cruised_raw_ = snapshot_.throttle_raw;
        cruised_at_ms_ = snapshot_.time_ms;
        cruise_pwm_ = calculateCruisePwm(cruised_raw_, performance_mode_, 0.60f);
        snapshot_.state = DeviceState::Cruising;
        log("state -> ARMED_CRUISING");
      }
    } else if (!enabled && snapshot_.state == DeviceState::Cruising) {
      snapshot_.state = DeviceState::Armed;
      throttleFilterReset(
          potRawToModePwm(snapshot_.throttle_raw, performance_mode_));
      log("state -> ARMED");
    }
  } else if (event.action == "mode") {
    if (event.value == "chill" || event.value == "0") performance_mode_ = 0;
    else if (event.value == "sport" || event.value == "1") performance_mode_ = 1;
    else throw std::runtime_error("scenario line " +
                                  std::to_string(event.source_line) +
                                  ": mode must be chill or sport");
  } else if (event.action == "altitude") {
    requested_altitude_m_ = parse_float();
    if (requested_altitude_m_ < -1000.0f ||
        requested_altitude_m_ >= 44330.0f) {
      throw std::runtime_error("scenario line " +
                               std::to_string(event.source_line) +
                               ": altitude must be in [-1000, 44330) m");
    }
  } else if (event.action == "esc_link") {
    esc_link_requested_ = parse_bool(event);
    if (!esc_link_requested_) esc_link_down_since_ms_ = snapshot_.time_ms;
    else esc_link_up_since_ms_ = snapshot_.time_ms;
  } else if (event.action == "bms_link") {
    bms_link_requested_ = parse_bool(event);
    if (!bms_link_requested_) bms_link_down_since_ms_ = snapshot_.time_ms;
    else bms_link_up_since_ms_ = snapshot_.time_ms;
  } else if (event.action == "barometer_link") {
    snapshot_.barometer_connected = parse_bool(event);
  } else if (event.action == "soc") {
    snapshot_.bms_soc_pct = static_cast<float>(
        std::clamp(parse_number(event), 0.0, 100.0));
    hardware_->set_bms_soc(snapshot_.bms_soc_pct);
  } else if (event.action == "bms_temp") {
    snapshot_.bms_temp_c = parse_float();
    hardware_->set_bms_temperature(snapshot_.bms_temp_c);
  } else if (event.action == "esc_temp") {
    snapshot_.esc_mos_temp_c = parse_float();
    hardware_->set_esc_temperature(snapshot_.esc_mos_temp_c);
  } else if (event.action == "esc_rpm") {
    const float value = static_cast<float>(std::clamp(parse_number(event), 0.0, 100000.0));
    hardware_->set_esc_rpm(value);
  } else if (event.action == "esc_amps") {
    const float value = static_cast<float>(std::clamp(parse_number(event), -500.0, 500.0));
    hardware_->set_esc_amps(value);
  } else if (event.action == "esc_voltage") {
    const float value = static_cast<float>(std::clamp(parse_number(event), 0.0, 200.0));
    hardware_->set_esc_voltage(value);
  } else if (event.action == "bms_voltage") {
    const float value = static_cast<float>(std::clamp(parse_number(event), 0.0, 200.0));
    hardware_->set_bms_voltage(value);
  } else if (event.action == "bms_current") {
    const float value = static_cast<float>(std::clamp(parse_number(event), -500.0, 500.0));
    hardware_->set_bms_current(value);
  } else if (event.action == "cpu_temp") {
    cpu_temp_c_ = parse_float();
  } else if (event.action == "stall_throttle") {
    throttle_task_stalled_ = parse_bool(event);
  } else if (event.action == "esc_drop_every") {
    hardware_->set_esc_drop_every(static_cast<uint32_t>(
        std::clamp(parse_number(event), 0.0,
                   static_cast<double>(std::numeric_limits<uint32_t>::max()))));
  } else if (event.action == "bms_drop_every") {
    hardware_->set_bms_drop_every(static_cast<uint32_t>(
        std::clamp(parse_number(event), 0.0,
                   static_cast<double>(std::numeric_limits<uint32_t>::max()))));
  } else if (event.action == "esc_corrupt_every") {
    hardware_->set_esc_corrupt_every(static_cast<uint32_t>(
        std::clamp(parse_number(event), 0.0,
                   static_cast<double>(std::numeric_limits<uint32_t>::max()))));
  } else if (event.action == "bms_corrupt_every") {
    hardware_->set_bms_corrupt_every(static_cast<uint32_t>(
        std::clamp(parse_number(event), 0.0,
                   static_cast<double>(std::numeric_limits<uint32_t>::max()))));
  } else if (event.action == "esc_bus_off") {
    hardware_->set_esc_bus_off(parse_bool(event), snapshot_.time_ms);
  } else if (event.action == "bms_bus_off") {
    hardware_->set_bms_bus_off(parse_bool(event), snapshot_.time_ms);
  } else {
    throw std::runtime_error("scenario line " + std::to_string(event.source_line) +
                             ": unknown action '" + event.action + "'");
  }
}

void ControllerSim::run_due_tasks(std::ostream* trace) {
  const auto due = [this](const char* name) {
    const auto& task = tasks_.at(name);
    return snapshot_.time_ms % task.period_ms == 0;
  };
  const auto run = [this](const char* name, auto callback) {
    auto& task = tasks_.at(name);
    callback();
    ++task.runs;
    task.last_run_ms = snapshot_.time_ms;
  };

  if (due("throttle") && !throttle_task_stalled_)
    run("throttle", [this] { task_throttle(); });
  if (due("bms")) run("bms", [this] { task_bms(); });
  if (due("sensors")) run("sensors", [this] { task_sensors(); });
  if (due("monitor")) run("monitor", [this] { task_monitor(); });
  if (due("ble")) run("ble", [this] { task_ble(); });
  if (due("ui")) run("ui", [this, trace] { task_ui(trace); });
  if (due("watchdog")) run("watchdog", [this] { task_watchdog(); });
}

void ControllerSim::task_throttle() {
  g_analog_read_value = snapshot_.throttle_raw;
  int target = ESC_DISARMED_PWM;
  if (snapshot_.state == DeviceState::Armed) {
    target = getSmoothedThrottlePwmFromRaw(snapshot_.throttle_raw,
                                           performance_mode_);
    target = applyModeRampClamp(target, previous_pwm_, performance_mode_);
  } else if (snapshot_.state == DeviceState::Cruising) {
    // Production sends the captured cruise PWM directly. The ramp/filter state
    // remains parked until cruise exits, when the filter is seeded from the
    // current potentiometer position.
    target = cruise_pwm_;
    if (snapshot_.time_ms - cruised_at_ms_ > 2000 &&
        shouldPotDisengageCruise(snapshot_.throttle_raw, cruised_raw_, 0.80f)) {
      snapshot_.state = DeviceState::Armed;
      throttleFilterReset(
          potRawToModePwm(snapshot_.throttle_raw, performance_mode_));
      log("state -> ARMED (cruise throttle override)");
    }
  } else {
    resetThrottleState(previous_pwm_);
  }
  snapshot_.throttle_pwm = target;

  hardware_->command_esc(snapshot_.time_ms, static_cast<uint16_t>(target),
                         snapshot_.esc_mos_temp_c, esc_link_requested_);
  const auto& esc = hardware_->esc();
  snapshot_.esc_connected = esc.connected;
  if (esc.connected) {
    snapshot_.esc_rpm = esc.rpm;
    snapshot_.esc_amps = esc.amps;
    snapshot_.esc_voltage_v = esc.voltage_v;
    snapshot_.esc_mos_temp_c = esc.mos_temp_c;
  }
  update_can_stats();
}

void ControllerSim::update_can_stats() {
  const auto& esc_stats = hardware_->esc_stats();
  const auto& bms_stats = hardware_->bms_stats();
  snapshot_.can_tx_frames = esc_stats.tx_frames + bms_stats.tx_frames;
  snapshot_.can_rx_frames = esc_stats.rx_frames + bms_stats.rx_frames;
  snapshot_.can_tx_bytes = esc_stats.tx_bytes + bms_stats.tx_bytes;
  snapshot_.can_rx_bytes = esc_stats.rx_bytes + bms_stats.rx_bytes;
  snapshot_.can_dropped_frames =
      esc_stats.dropped_frames + bms_stats.dropped_frames;
  snapshot_.can_corrupted_frames =
      esc_stats.corrupted_frames + bms_stats.corrupted_frames;
  snapshot_.can_bus_off_count =
      esc_stats.bus_off_count + bms_stats.bus_off_count;
}

void ControllerSim::task_bms() {
  hardware_->update_bms(snapshot_.time_ms, snapshot_.esc_amps,
                        snapshot_.bms_soc_pct, snapshot_.bms_temp_c,
                        bms_link_requested_);
  const auto& bms = hardware_->bms();
  snapshot_.bms_connected = bms.connected;
  if (bms.connected) {
    snapshot_.bms_voltage_v = bms.voltage_v;
    snapshot_.bms_current_a = bms.current_a;
    snapshot_.bms_soc_pct = bms.soc_pct;
    snapshot_.bms_temp_c = bms.temperature_c;
  }
  update_can_stats();
}

void ControllerSim::task_sensors() {
  if (!snapshot_.barometer_connected) return;
  snapshot_.altitude_m = requested_altitude_m_;
  snapshot_.pressure_hpa = sea_level_pressure_hpa_ *
      std::pow(1.0f - snapshot_.altitude_m / 44330.0f, 5.255f);
  altitude_history_.emplace_back(snapshot_.time_ms, snapshot_.altitude_m);
  while (altitude_history_.size() > 50) altitude_history_.pop_front();
  if (altitude_history_.size() >= 2) {
    const auto& first = altitude_history_.front();
    const auto& last = altitude_history_.back();
    const float seconds = (last.first - first.first) / 1000.0f;
    if (seconds > 0.0f) {
      snapshot_.vertical_speed_mps = std::clamp(
          (last.second - first.second) / seconds, -250.0f, 250.0f);
    }
  }
}

void ControllerSim::add_or_clear_alert(const std::string& name, bool active) {
  const auto found = std::find(snapshot_.alerts.begin(), snapshot_.alerts.end(), name);
  if (active && found == snapshot_.alerts.end()) {
    snapshot_.alerts.push_back(name);
    log("alert +" + name);
  } else if (!active && found != snapshot_.alerts.end()) {
    snapshot_.alerts.erase(found);
    log("alert -" + name);
  }
}

void ControllerSim::task_monitor() {
  // Track the edge the controller actually observes after protocol decoding
  // and stale-data timeout, rather than the instant an electrical fault was
  // injected. This matches what the firmware can know.
  if (snapshot_.esc_connected != previous_esc_connected_) {
    if (snapshot_.esc_connected) esc_link_up_since_ms_ = snapshot_.time_ms;
    else esc_link_down_since_ms_ = snapshot_.time_ms;
    previous_esc_connected_ = snapshot_.esc_connected;
  }
  if (snapshot_.bms_connected != previous_bms_connected_) {
    if (snapshot_.bms_connected) bms_link_up_since_ms_ = snapshot_.time_ms;
    else bms_link_down_since_ms_ = snapshot_.time_ms;
    previous_bms_connected_ = snapshot_.bms_connected;
  }
  if (snapshot_.esc_connected) {
    if (!esc_link_established_ && snapshot_.time_ms - esc_link_up_since_ms_ >= 1000)
      esc_link_established_ = true;
  }
  if (snapshot_.bms_connected) {
    if (!bms_link_established_ && snapshot_.time_ms - bms_link_up_since_ms_ >= 1000)
      bms_link_established_ = true;
  }
  const auto alert_active = [this](const char* name) {
    return std::find(snapshot_.alerts.begin(), snapshot_.alerts.end(), name) !=
           snapshot_.alerts.end();
  };
  const bool esc_alert = alert_active("ESC_CAN_LINK_LOST");
  const bool bms_alert = alert_active("BMS_CAN_LINK_LOST");
  add_or_clear_alert(
      "ESC_CAN_LINK_LOST",
      esc_link_established_ &&
          ((!snapshot_.esc_connected &&
            snapshot_.time_ms - esc_link_down_since_ms_ >= 500) ||
           (snapshot_.esc_connected && esc_alert &&
            snapshot_.time_ms - esc_link_up_since_ms_ < 1000)));
  add_or_clear_alert(
      "BMS_CAN_LINK_LOST",
      bms_link_established_ &&
          ((!snapshot_.bms_connected &&
            snapshot_.time_ms - bms_link_down_since_ms_ >= 500) ||
           (snapshot_.bms_connected && bms_alert &&
            snapshot_.time_ms - bms_link_up_since_ms_ < 1000)));
  add_or_clear_alert("BARO_INIT_FAILURE", !snapshot_.barometer_connected);
  add_or_clear_alert("ESC_MOS_TEMP_WARN",
                     snapshot_.esc_connected &&
                         snapshot_.esc_mos_temp_c >= 90.0f &&
                         snapshot_.esc_mos_temp_c < 110.0f);
  add_or_clear_alert("ESC_MOS_TEMP_CRIT",
                     snapshot_.esc_connected &&
                         snapshot_.esc_mos_temp_c >= 110.0f);
  add_or_clear_alert("BMS_TEMP_WARN",
                     snapshot_.bms_connected && snapshot_.bms_temp_c >= 50.0f &&
                         snapshot_.bms_temp_c < 60.0f);
  add_or_clear_alert("BMS_TEMP_CRIT",
                     snapshot_.bms_connected && snapshot_.bms_temp_c >= 60.0f);
  add_or_clear_alert("BMS_SOC_WARN",
                     snapshot_.bms_connected && snapshot_.bms_soc_pct <= 15.0f &&
                         snapshot_.bms_soc_pct > 5.0f);
  add_or_clear_alert("BMS_SOC_CRIT",
                     snapshot_.bms_connected && snapshot_.bms_soc_pct <= 5.0f);
  add_or_clear_alert("CPU_TEMP_WARN", cpu_temp_c_ >= 60.0f && cpu_temp_c_ < 80.0f);
  add_or_clear_alert("CPU_TEMP_CRIT", cpu_temp_c_ >= 80.0f);
}

void ControllerSim::task_ble() { ++snapshot_.ble_notifications; }

void ControllerSim::task_ui(std::ostream* trace) {
  if (trace != nullptr && last_trace_ms_ != snapshot_.time_ms) {
    *trace << snapshot_json() << '\n';
    last_trace_ms_ = snapshot_.time_ms;
  }
}

void ControllerSim::task_watchdog() {
  auto& throttle = tasks_.at("throttle");
  if (snapshot_.time_ms - throttle.last_run_ms >= 5000) {
    ++snapshot_.watchdog_resets;
    snapshot_.state = DeviceState::Disarmed;
    snapshot_.throttle_pwm = ESC_DISARMED_PWM;
    throttle_task_stalled_ = false;
    throttle.last_run_ms = snapshot_.time_ms;
    resetThrottleState(previous_pwm_);
    log("watchdog reset: throttle task missed 5000 ms deadline");
  }
}

void ControllerSim::log(const std::string& message) {
  event_log_.push_back(std::to_string(snapshot_.time_ms) + "ms " + message);
}

std::string ControllerSim::snapshot_json() const {
  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << "{\"time_ms\":" << snapshot_.time_ms
      << ",\"state\":\"" << state_name(snapshot_.state) << "\""
      << ",\"throttle_raw\":" << snapshot_.throttle_raw
      << ",\"throttle_pwm\":" << snapshot_.throttle_pwm
      << ",\"esc\":{\"connected\":" << (snapshot_.esc_connected ? "true" : "false")
      << ",\"voltage_v\":" << snapshot_.esc_voltage_v
      << ",\"rpm\":" << snapshot_.esc_rpm << ",\"amps\":" << snapshot_.esc_amps
      << ",\"mos_temp_c\":" << snapshot_.esc_mos_temp_c << "}"
      << ",\"bms\":{\"connected\":" << (snapshot_.bms_connected ? "true" : "false")
      << ",\"voltage_v\":" << snapshot_.bms_voltage_v
      << ",\"current_a\":" << snapshot_.bms_current_a
      << ",\"soc_pct\":" << snapshot_.bms_soc_pct
      << ",\"temp_c\":" << snapshot_.bms_temp_c << "}"
      << ",\"barometer\":{\"connected\":"
      << (snapshot_.barometer_connected ? "true" : "false")
      << ",\"altitude_m\":" << snapshot_.altitude_m
      << ",\"vertical_speed_mps\":" << snapshot_.vertical_speed_mps
      << ",\"pressure_hpa\":" << snapshot_.pressure_hpa << "}"
      << ",\"io\":{\"can_tx_frames\":" << snapshot_.can_tx_frames
      << ",\"can_rx_frames\":" << snapshot_.can_rx_frames
      << ",\"can_tx_bytes\":" << snapshot_.can_tx_bytes
      << ",\"can_rx_bytes\":" << snapshot_.can_rx_bytes
      << ",\"can_dropped_frames\":" << snapshot_.can_dropped_frames
      << ",\"can_corrupted_frames\":" << snapshot_.can_corrupted_frames
      << ",\"can_bus_off_count\":" << snapshot_.can_bus_off_count
      << ",\"ble_notifications\":" << snapshot_.ble_notifications << "}"
      << ",\"can\":{\"esc\":{\"bitrate\":1000000"
      << ",\"tx_frames\":" << hardware_->esc_stats().tx_frames
      << ",\"rx_frames\":" << hardware_->esc_stats().rx_frames
      << ",\"tx_bytes\":" << hardware_->esc_stats().tx_bytes
      << ",\"rx_bytes\":" << hardware_->esc_stats().rx_bytes
      << ",\"dropped\":" << hardware_->esc_stats().dropped_frames
      << ",\"corrupted\":" << hardware_->esc_stats().corrupted_frames
      << ",\"queue_overflows\":" << hardware_->esc_stats().queue_overflows
      << ",\"bus_off_count\":" << hardware_->esc_stats().bus_off_count << "}"
      << ",\"bms\":{\"bitrate\":250000"
      << ",\"tx_frames\":" << hardware_->bms_stats().tx_frames
      << ",\"rx_frames\":" << hardware_->bms_stats().rx_frames
      << ",\"tx_bytes\":" << hardware_->bms_stats().tx_bytes
      << ",\"rx_bytes\":" << hardware_->bms_stats().rx_bytes
      << ",\"dropped\":" << hardware_->bms_stats().dropped_frames
      << ",\"corrupted\":" << hardware_->bms_stats().corrupted_frames
      << ",\"queue_overflows\":" << hardware_->bms_stats().queue_overflows
      << ",\"bus_off_count\":" << hardware_->bms_stats().bus_off_count
      << "}}"
      << ",\"watchdog_resets\":" << snapshot_.watchdog_resets
      << ",\"alerts\":[";
  for (std::size_t i = 0; i < snapshot_.alerts.size(); ++i) {
    if (i != 0) out << ',';
    out << '"' << json_escape(snapshot_.alerts[i]) << '"';
  }
  out << "]}";
  return out.str();
}

std::vector<Event> load_scenario(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open scenario: " + path);
  std::vector<Event> result;
  std::string line;
  std::size_t line_number = 0;
  uint64_t previous_time = 0;
  while (std::getline(input, line)) {
    ++line_number;
    line = trim(line);
    if (line.empty() || line[0] == '#') continue;
    std::istringstream fields(line);
    std::string time_text;
    std::string action;
    std::string value;
    if (!std::getline(fields, time_text, ',') ||
        !std::getline(fields, action, ',') ||
        !std::getline(fields, value)) {
      throw std::runtime_error("scenario line " + std::to_string(line_number) +
                               ": expected time_ms,action,value");
    }
    Event event;
    event.source_line = line_number;
    event.action = trim(action);
    event.value = trim(value);
    try {
      std::size_t used = 0;
      const std::string normalized_time = trim(time_text);
      if (normalized_time.empty() || normalized_time.front() == '-') {
        throw std::invalid_argument("invalid");
      }
      event.at_ms = std::stoull(normalized_time, &used);
      if (used != normalized_time.size()) throw std::invalid_argument("trailing");
    } catch (const std::exception&) {
      throw std::runtime_error("scenario line " + std::to_string(line_number) +
                               ": invalid time");
    }
    if (!result.empty() && event.at_ms < previous_time) {
      throw std::runtime_error("scenario line " + std::to_string(line_number) +
                               ": events must be time-ordered");
    }
    previous_time = event.at_ms;
    result.push_back(std::move(event));
  }
  return result;
}

}  // namespace eppg::sim
