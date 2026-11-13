#include "controller_sim.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

bool has_alert(const eppg::sim::ControllerSim& sim, const std::string& alert) {
  const auto& alerts = sim.snapshot().alerts;
  return std::find(alerts.begin(), alerts.end(), alert) != alerts.end();
}

void schedule(eppg::sim::ControllerSim& sim, uint64_t at,
              const std::string& action, const std::string& value) {
  sim.schedule({at, action, value, 1});
}

void test_flight_and_rtos_cadence() {
  eppg::sim::ControllerSim sim;
  schedule(sim, 0, "throttle", "0");
  schedule(sim, 100, "arm", "1");
  schedule(sim, 200, "throttle", "4095");
  schedule(sim, 1200, "altitude", "25");
  sim.run(2000);
  const auto& state = sim.snapshot();
  require(state.state == eppg::sim::DeviceState::Armed, "controller did not arm");
  require(state.throttle_pwm > 1035 && state.throttle_pwm <= 1950,
          "production throttle pipeline did not ramp");
  require(state.esc_rpm > 1000.0f, "ESC model did not spin");
  require(state.altitude_m == 25.0f, "barometer altitude was not applied");
  require(sim.task_stats().at("throttle").runs == 101,
          "50 Hz throttle cadence changed");
  require(sim.task_stats().at("ui").runs == 61, "30 Hz UI cadence changed");
}

void test_can_loss_debounce() {
  eppg::sim::ControllerSim sim;
  schedule(sim, 0, "throttle", "0");
  schedule(sim, 100, "arm", "1");
  schedule(sim, 1500, "esc_link", "off");
  sim.run(1900);
  require(sim.snapshot().alerts.empty(), "CAN loss fired before 500 ms debounce");
  sim.run(2100);
  require(!sim.snapshot().alerts.empty() &&
              sim.snapshot().alerts.front() == "ESC_CAN_LINK_LOST",
          "established ESC CAN loss did not alert");
}

void test_can_recovery_debounce() {
  eppg::sim::ControllerSim sim;
  schedule(sim, 1500, "esc_link", "off");
  schedule(sim, 2200, "esc_link", "on");
  sim.run(2700);
  require(has_alert(sim, "ESC_CAN_LINK_LOST"),
          "CAN loss cleared before the recovery establish window");
  sim.run(3200);
  require(has_alert(sim, "ESC_CAN_LINK_LOST"),
          "CAN loss cleared before decoded telemetry was stable");
  sim.run(3300);
  require(!has_alert(sim, "ESC_CAN_LINK_LOST"),
          "CAN loss did not clear after stable recovery");
}

void test_real_wire_latency_and_parsers() {
  eppg::sim::ControllerSim sim;
  sim.run(0);
  require(!sim.snapshot().esc_connected,
          "ESC transfer arrived without CAN serialization latency");
  require(!sim.snapshot().bms_connected,
          "BMS transfer arrived without CAN serialization latency");
  sim.run(20);
  require(sim.snapshot().esc_connected,
          "real DroneCAN status did not reassemble on the next task tick");
  sim.run(100);
  require(sim.snapshot().bms_connected,
          "real ANT BMS parser did not consume MCP2515 frames");
  require(sim.snapshot().can_tx_bytes > sim.snapshot().can_tx_frames,
          "CAN accounting is not based on payload bytes");
  require(std::fabs(sim.snapshot().bms_voltage_v - 97.9f) < 0.2f,
          "BMS voltage did not survive raw 0.1 V quantization");
}

void test_wire_drop_disconnects_esc() {
  eppg::sim::ControllerSim sim;
  schedule(sim, 1500, "esc_drop_every", "1");
  sim.run(2200);
  require(sim.snapshot().can_dropped_frames > 0,
          "wire drop injector did not discard frames");
  require(!sim.snapshot().esc_connected,
          "ESC stayed connected without valid DroneCAN transfers");
  require(has_alert(sim, "ESC_CAN_LINK_LOST"),
          "wire-level ESC loss did not reach the monitor");
}

void test_dronecan_crc_rejects_corruption() {
  eppg::sim::ControllerSim sim;
  schedule(sim, 1500, "esc_corrupt_every", "2");
  sim.run(2200);
  require(sim.snapshot().can_corrupted_frames > 0,
          "ESC corruption injector did not alter frames");
  require(!sim.snapshot().esc_connected,
          "libcanard accepted repeatedly corrupted multi-frame telemetry");
  require(has_alert(sim, "ESC_CAN_LINK_LOST"),
          "DroneCAN CRC/reassembly rejection did not reach the monitor");
}

void test_bus_off_recovery_timing() {
  eppg::sim::ControllerSim sim;
  schedule(sim, 1500, "esc_bus_off", "on");
  schedule(sim, 1800, "esc_bus_off", "off");
  sim.run(2000);
  require(!sim.snapshot().esc_connected,
          "TWAI recovered before the 250 ms health/recovery interval");
  require(sim.snapshot().can_bus_off_count == 1,
          "bus-off transition was not counted exactly once");
  sim.run(2200);
  require(sim.snapshot().esc_connected,
          "TWAI did not resume valid DroneCAN traffic after recovery");
}

void test_bms_wire_corruption_is_observable() {
  eppg::sim::ControllerSim sim;
  schedule(sim, 1500, "bms_corrupt_every", "1");
  sim.run(1700);
  require(sim.snapshot().can_corrupted_frames > 0,
          "BMS corruption injector did not alter raw frames");
  // ANT's basic frames have no end-to-end CRC. The real parser therefore
  // accepts a damaged payload; surfacing that is an important fidelity fact.
  require(sim.snapshot().bms_connected,
          "raw ANT frame corruption should not imply transport disconnect");
}

void test_watchdog_disarms() {
  eppg::sim::ControllerSim sim;
  schedule(sim, 0, "throttle", "0");
  schedule(sim, 100, "arm", "1");
  schedule(sim, 200, "throttle", "3000");
  schedule(sim, 400, "stall_throttle", "on");
  sim.run(5600);
  require(sim.snapshot().watchdog_resets == 1, "watchdog did not reset once");
  require(sim.snapshot().state == eppg::sim::DeviceState::Disarmed,
          "watchdog reset did not fail safe");
  require(sim.snapshot().throttle_pwm == 1000,
          "watchdog reset did not command disarmed PWM");
}

void test_arm_interlock() {
  eppg::sim::ControllerSim sim;
  schedule(sim, 0, "throttle", "1000");
  schedule(sim, 10, "arm", "1");
  sim.run(100);
  require(sim.snapshot().state == eppg::sim::DeviceState::Disarmed,
          "unsafe arm was accepted");
}

void test_arm_interlock_boundary() {
  eppg::sim::ControllerSim sim;
  schedule(sim, 0, "throttle", "204");
  schedule(sim, 10, "arm", "1");
  sim.run(100);
  require(sim.snapshot().state == eppg::sim::DeviceState::Disarmed,
          "engagement-floor boundary incorrectly armed");
}

void test_cruise_rules_and_override() {
  eppg::sim::ControllerSim sim;
  schedule(sim, 0, "throttle", "0");
  schedule(sim, 100, "arm", "1");
  schedule(sim, 200, "cruise", "on");
  schedule(sim, 300, "throttle", "1000");
  schedule(sim, 400, "cruise", "on");
  sim.run(500);
  require(sim.snapshot().state == eppg::sim::DeviceState::Cruising,
          "valid cruise activation was rejected");
  sim.run(2400);
  require(sim.snapshot().state == eppg::sim::DeviceState::Cruising,
          "cruise override fired during the grace period");
  sim.run(2420);
  require(sim.snapshot().state == eppg::sim::DeviceState::Armed,
          "cruise throttle override did not disengage cruise");
}

void test_invalid_number_rejected() {
  eppg::sim::ControllerSim sim;
  schedule(sim, 0, "altitude", "nan");
  bool threw = false;
  try {
    sim.run(0);
  } catch (const std::runtime_error&) {
    threw = true;
  }
  require(threw, "non-finite scenario number was accepted");
}

void test_alert_levels_are_exclusive() {
  eppg::sim::ControllerSim sim;
  schedule(sim, 0, "esc_temp", "120");
  schedule(sim, 0, "soc", "3");
  sim.run(100);
  require(has_alert(sim, "ESC_MOS_TEMP_CRIT") &&
              !has_alert(sim, "ESC_MOS_TEMP_WARN"),
          "ESC warning and critical levels were not exclusive");
  require(has_alert(sim, "BMS_SOC_CRIT") &&
              !has_alert(sim, "BMS_SOC_WARN"),
          "BMS SOC warning and critical levels were not exclusive");
}

void test_wire_telemetry_injection() {
  eppg::sim::ControllerSim sim;
  schedule(sim, 0, "esc_rpm", "4321");
  schedule(sim, 0, "esc_amps", "37.4");
  schedule(sim, 0, "esc_voltage", "91.2");
  schedule(sim, 0, "bms_voltage", "89.6");
  schedule(sim, 0, "bms_current", "34.2");
  schedule(sim, 0, "soc", "64");
  schedule(sim, 0, "bms_temp", "38");
  schedule(sim, 0, "esc_temp", "54");
  sim.run(300);
  const auto& state = sim.snapshot();
  require(std::fabs(state.esc_rpm - 4321.0f) < 1.0f,
          "injected ESC RPM did not traverse DroneCAN");
  require(std::fabs(state.esc_amps - 37.4f) < 0.2f,
          "injected ESC current did not traverse DroneCAN");
  require(std::fabs(state.esc_voltage_v - 91.2f) < 0.2f,
          "injected ESC voltage did not traverse DroneCAN");
  require(std::fabs(state.bms_voltage_v - 89.6f) < 0.2f,
          "injected BMS voltage did not traverse ANT CAN");
  require(std::fabs(state.bms_current_a - 34.2f) < 0.2f,
          "injected BMS current did not traverse ANT CAN");
  require(std::fabs(state.bms_soc_pct - 64.0f) < 1.0f,
          "injected BMS SOC did not traverse ANT CAN");
  require(std::fabs(state.bms_temp_c - 38.0f) < 1.0f,
          "injected BMS temperature did not traverse ANT CAN");
  require(std::fabs(state.esc_mos_temp_c - 54.0f) < 0.2f,
          "injected ESC temperature did not traverse DroneCAN");
}

}  // namespace

int main() {
  try {
    test_flight_and_rtos_cadence();
    test_can_loss_debounce();
    test_can_recovery_debounce();
    test_real_wire_latency_and_parsers();
    test_wire_drop_disconnects_esc();
    test_dronecan_crc_rejects_corruption();
    test_bus_off_recovery_timing();
    test_bms_wire_corruption_is_observable();
    test_watchdog_disarms();
    test_arm_interlock();
    test_arm_interlock_boundary();
    test_cruise_rules_and_override();
    test_invalid_number_rejected();
    test_alert_levels_are_exclusive();
    test_wire_telemetry_injection();
    std::cout << "15 simulator integration tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "simulator test failed: " << error.what() << '\n';
    return 1;
  }
}
