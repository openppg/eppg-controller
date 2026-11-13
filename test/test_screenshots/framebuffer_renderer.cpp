#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <Arduino.h>
#include "emulator_display.h"
#include "sp140/lvgl/lvgl_main_screen.h"
#include "sp140/lvgl/lvgl_updates.h"
#include "sp140/structs.h"

namespace {

bool connected(const char* value) {
  return std::strcmp(value, "1") == 0 || std::strcmp(value, "true") == 0;
}

float number(const char* value) {
  return std::strtof(value, nullptr);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 17 && argc != 19) {
    std::fprintf(stderr,
                 "usage: framebuffer_renderer output.bmp state mode time_ms "
                 "esc_connected esc_volts esc_rpm esc_amps esc_temp "
                 "bms_connected bms_volts bms_current bms_soc bms_temp "
                 "altitude vertical_speed [bms_lowest_cell_v bms_highest_cell_v]\n");
    return 2;
  }

  const std::string state = argv[2];
  const bool armed = state != "DISARMED";
  const bool cruising = state == "ARMED_CRUISING";

  STR_DEVICE_DATA_140_V1 device = {};
  device.version_major = 8;
  device.version_minor = 0;
  device.screen_rotation = 1;
  device.sea_pressure = 1013.25f;
  device.metric_temp = true;
  device.metric_alt = true;
  device.performance_mode = std::strcmp(argv[3], "chill") == 0 ? 0 : 1;
  device.theme = 0;

  STR_ESC_TELEMETRY_140 esc = {};
  esc.escState = connected(argv[5]) ? TelemetryState::CONNECTED
                                    : TelemetryState::NOT_CONNECTED;
  esc.volts = number(argv[6]);
  esc.eRPM = number(argv[7]);
  esc.amps = number(argv[8]);
  esc.mos_temp = number(argv[9]);
  esc.cap_temp = esc.mos_temp;
  esc.mcu_temp = esc.mos_temp;
  esc.motor_temp = esc.mos_temp;

  STR_BMS_TELEMETRY_140 bms = {};
  bms.bmsState = connected(argv[10]) ? TelemetryState::CONNECTED
                                    : TelemetryState::NOT_CONNECTED;
  bms.battery_voltage = number(argv[11]);
  if (argc == 19) {
    bms.lowest_cell_voltage = number(argv[17]);
    bms.highest_cell_voltage = number(argv[18]);
  }
  bms.battery_current = number(argv[12]);
  bms.soc = number(argv[13]);
  bms.highest_temperature = number(argv[14]);
  bms.lowest_temperature = bms.highest_temperature;
  bms.mos_temperature = bms.highest_temperature;
  bms.t1_temperature = bms.highest_temperature;
  bms.t2_temperature = bms.highest_temperature;
  bms.t3_temperature = NAN;
  bms.t4_temperature = NAN;
  bms.battery_ready = bms.bmsState == TelemetryState::CONNECTED;
  bms.is_charge_mos = bms.battery_ready;
  bms.is_discharge_mos = bms.battery_ready;

  bms.power = bms.battery_voltage * bms.battery_current / 1000.0f;

  UnifiedBatteryData battery = {};
  battery.volts = bms.battery_voltage;
  battery.amps = bms.bmsState == TelemetryState::CONNECTED
      ? bms.battery_current : esc.amps;
  battery.power = bms.power;
  battery.soc = bms.soc;

  const unsigned int elapsed_ms = static_cast<unsigned int>(
      std::strtoul(argv[4], nullptr, 10));
  const unsigned int armed_start_ms = armed ? millis() - elapsed_ms : 0;
  emulator_init_display(false);
  setupMainScreen(false);
  updateLvglMainScreen(device, esc, bms, battery, number(argv[15]), armed,
                       cruising, armed_start_ms);
  updateClimbRateIndicator(number(argv[16]), false);
  emulator_render_frame();

  const bool saved = emulator_save_bmp(argv[1]);
  emulator_teardown();
  if (!saved) {
    std::fprintf(stderr, "failed to write framebuffer: %s\n", argv[1]);
    return 1;
  }
  return 0;
}
