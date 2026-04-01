// Video generator POC: renders a 30-second flight scenario to MP4 via ffmpeg pipe
// Build with the same CMake as screenshot_tests, or standalone via the video target.
//
// Scenario timeline (30s @ 30 FPS = 900 frames):
//   0-2s:   Splash screen
//   2-3s:   Transition to main screen (disarmed, idle)
//   3-5s:   Idle on ground
//   5-6s:   Arm (state change, timer starts)
//   6-12s:  Takeoff - throttle up, altitude climbs, power ramps
//   12-13s: Cruise engage
//   13-20s: Cruise, temps rising
//   20-23s: ESC temp warning triggers
//   23-26s: Critical alert - motor temp critical, red border flashing
//   26-28s: Descend / power down
//   28-29s: Landing, disarm
//   29-30s: Idle on ground, disarmed

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <map>
#include <vector>

#include "emulator_display.h"
#include "sp140/lvgl/lvgl_core.h"
#include "sp140/lvgl/lvgl_main_screen.h"
#include "sp140/lvgl/lvgl_updates.h"
#include "sp140/lvgl/lvgl_alerts.h"
#include "sp140/structs.h"
#include "sp140/simple_monitor.h"
#include "sp140/alert_display.h"
#include "version.h"

static const int FPS = 30;
static const int DURATION_SEC = 30;
static const int TOTAL_FRAMES = FPS * DURATION_SEC;
static const int FRAME_MS = 1000 / FPS;  // ~33ms per frame

// Smooth interpolation helper
static float lerp(float a, float b, float t) {
  t = std::max(0.0f, std::min(1.0f, t));
  return a + (b - a) * t;
}

// Ease-in-out for smoother transitions
static float ease(float t) {
  t = std::max(0.0f, std::min(1.0f, t));
  return t * t * (3.0f - 2.0f * t);
}

// Write raw RGB888 pixels (top-down) from the RGB565 framebuffer to a FILE*
static void write_frame_rgb888(FILE* pipe) {
  uint16_t* fb = emulator_get_framebuffer();
  for (int y = 0; y < SCREEN_HEIGHT; y++) {
    for (int x = 0; x < SCREEN_WIDTH; x++) {
      uint16_t rgb565 = fb[y * SCREEN_WIDTH + x];
      uint8_t r = ((rgb565 >> 11) & 0x1F) * 255 / 31;
      uint8_t g = ((rgb565 >> 5) & 0x3F) * 255 / 63;
      uint8_t b = (rgb565 & 0x1F) * 255 / 31;
      uint8_t pixel[3] = {r, g, b};
      fwrite(pixel, 1, 3, pipe);
    }
  }
}

// Render one LVGL frame with realistic tick advancement
static void render_tick(int tick_ms) {
  lv_tick_inc(tick_ms);
  lv_timer_handler();
  lv_refr_now(main_display);
}

// ============================================================
// Alert aggregator — mirrors alert_display.cpp logic without
// FreeRTOS queues. Same data structures, same rotation cadence.
// ============================================================
struct AlertAggregator {
  std::map<SensorID, AlertLevel> currentLevels;
  AlertCounts counts = {0, 0};
  std::vector<SensorID> critList;
  std::vector<SensorID> warnList;
  size_t critRotateIdx = 0;
  size_t warnRotateIdx = 0;
  unsigned long lastRotateMs = 0;
  bool lastCriticalState = false;

  // Process a sensor alert event (same as alertAggregationTask queue receive)
  void processEvent(SensorID id, AlertLevel level) {
    if (level == AlertLevel::OK) {
      currentLevels.erase(id);
    } else {
      currentLevels[id] = level;
    }
    recalcAndPublish();
  }

  // Rotate display every 2 seconds (same as alertAggregationTask timer)
  void tick(unsigned long nowMs) {
    bool hasAlerts = !critList.empty() || !warnList.empty();
    if (hasAlerts && (nowMs - lastRotateMs >= 2000)) {
      lastRotateMs = nowMs;
      if (!critList.empty()) {
        critRotateIdx = (critRotateIdx + 1) % critList.size();
      }
      if (!warnList.empty()) {
        warnRotateIdx = (warnRotateIdx + 1) % warnList.size();
      }
      applyToUI();
    }
  }

private:
  // Same logic as recalcCountsAndPublish() in alert_display.cpp
  void recalcAndPublish() {
    AlertCounts newCounts = {0, 0};
    std::vector<SensorID> newCritList;
    std::vector<SensorID> newWarnList;

    for (const auto& kv : currentLevels) {
      switch (kv.second) {
        case AlertLevel::WARN_LOW:
        case AlertLevel::WARN_HIGH:
          newWarnList.push_back(kv.first);
          newCounts.warningCount++;
          break;
        case AlertLevel::CRIT_LOW:
        case AlertLevel::CRIT_HIGH:
          newCritList.push_back(kv.first);
          newCounts.criticalCount++;
          break;
        default:
          break;
      }
    }

    bool critListChanged = (newCritList != critList);
    bool warnListChanged = (newWarnList != warnList);
    bool countsChanged = (newCounts.warningCount != counts.warningCount ||
                          newCounts.criticalCount != counts.criticalCount);

    counts = newCounts;
    if (critListChanged) {
      critList = newCritList;
      critRotateIdx = 0;
    }
    if (warnListChanged) {
      warnList = newWarnList;
      warnRotateIdx = 0;
    }

    if (countsChanged || critListChanged || warnListChanged) {
      lastRotateMs = millis();
      applyToUI();
    }
  }

  // Same logic as main.cpp:438-472 — the UI task's queue consumer
  void applyToUI() {
    updateAlertCounterDisplay(counts);

    // Critical display (top row, altitude area)
    if (!critList.empty()) {
      SensorID id = critList[critRotateIdx];
      AlertLevel level = currentLevels[id];
      lv_showAlertTextWithLevel(id, level, true);
    } else {
      lv_hideCriticalText();
    }

    // Warning display (below critical)
    if (!warnList.empty()) {
      SensorID id = warnList[warnRotateIdx];
      AlertLevel level = currentLevels[id];
      lv_showAlertTextWithLevel(id, level, false);
    } else {
      lv_hideWarningText();
    }

    // Control critical border flash (same as main.cpp:460-472)
    bool criticalActive = (counts.criticalCount > 0);
    if (criticalActive != lastCriticalState) {
      if (criticalActive) {
        startCriticalBorderFlashDirect();
      } else {
        stopCriticalBorderFlashDirect();
      }
      lastCriticalState = criticalActive;
    }
  }
};

// ============================================================
// Simple threshold checker — mirrors HysteresisSensorMonitor
// from monitor_config.h thresholds
// ============================================================
struct ThresholdCheck {
  SensorID id;
  float warnHigh;
  float critHigh;
  AlertLevel lastLevel = AlertLevel::OK;

  AlertLevel check(float value) {
    if (std::isnan(value)) return lastLevel;
    AlertLevel newLevel;
    if (value >= critHigh) newLevel = AlertLevel::CRIT_HIGH;
    else if (value >= warnHigh) newLevel = AlertLevel::WARN_HIGH;
    else newLevel = AlertLevel::OK;
    AlertLevel prev = lastLevel;
    lastLevel = newLevel;
    if (newLevel != prev) return newLevel;
    return AlertLevel::OK;  // no change sentinel — caller checks
  }

  bool changed(float value) {
    if (std::isnan(value)) return false;
    AlertLevel newLevel;
    if (value >= critHigh) newLevel = AlertLevel::CRIT_HIGH;
    else if (value >= warnHigh) newLevel = AlertLevel::WARN_HIGH;
    else newLevel = AlertLevel::OK;
    return newLevel != lastLevel;
  }

  AlertLevel evaluate(float value) {
    if (std::isnan(value)) return lastLevel;
    AlertLevel prev = lastLevel;
    if (value >= critHigh) lastLevel = AlertLevel::CRIT_HIGH;
    else if (value >= warnHigh) lastLevel = AlertLevel::WARN_HIGH;
    else lastLevel = AlertLevel::OK;
    return lastLevel;
  }
};

int main(int argc, char** argv) {
  const char* output_path = "test/test_screenshots/output/flight_scenario.mp4";
  if (argc > 1) {
    output_path = argv[1];
  }

  printf("=== EPPG Display Video Generator ===\n");
  printf("Output: %s\n", output_path);
  printf("Resolution: %dx%d @ %d FPS, %d seconds (%d frames)\n",
         SCREEN_WIDTH, SCREEN_HEIGHT, FPS, DURATION_SEC, TOTAL_FRAMES);

  // Open ffmpeg pipe - scale up 4x for visibility since 160x128 is tiny
  char cmd[512];
  snprintf(cmd, sizeof(cmd),
    "ffmpeg -y -f rawvideo -pixel_format rgb24 -video_size %dx%d "
    "-framerate %d -i pipe:0 "
    "-vf scale=%d:%d:flags=neighbor "
    "-c:v libx264 -pix_fmt yuv420p -crf 18 -preset fast "
    "-loglevel warning '%s'",
    SCREEN_WIDTH, SCREEN_HEIGHT, FPS,
    SCREEN_WIDTH * 4, SCREEN_HEIGHT * 4,
    output_path);

  FILE* pipe = popen(cmd, "w");
  if (!pipe) {
    fprintf(stderr, "Failed to open ffmpeg pipe. Is ffmpeg installed?\n");
    return 1;
  }

  // Initialize LVGL + display (light mode - the default)
  bool darkMode = false;
  emulator_init_display(darkMode);

  // --- Phase tracking ---
  enum Phase {
    SPLASH,
    TRANSITION_TO_MAIN,
    IDLE_GROUND,
    ARMING,
    TAKEOFF,
    CRUISE_ENGAGE,
    CRUISING_STEADY,
    TEMP_WARNING,
    TEMP_CRITICAL,
    DESCEND,
    LANDING_DISARM,
    IDLE_END
  };

  // Create device data
  STR_DEVICE_DATA_140_V1 dd = {};
  dd.version_major = VERSION_MAJOR;
  dd.version_minor = VERSION_MINOR;
  dd.armed_time = 125;  // 2h05m total Hobbs
  dd.screen_rotation = 1;
  dd.sea_pressure = 1013.25f;
  dd.metric_temp = true;
  dd.metric_alt = true;
  dd.performance_mode = 1;
  dd.theme = darkMode ? 1 : 0;
  dd.revision = 3;

  // Splash screen objects (created manually like in test)
  lv_obj_t* splash_screen = lv_obj_create(NULL);
  lv_screen_load(splash_screen);
  lv_obj_remove_flag(splash_screen, LV_OBJ_FLAG_SCROLLABLE);
  lv_color_t bg_color = darkMode ? lv_color_black() : lv_color_white();
  lv_color_t fg_color = darkMode ? lv_color_white() : lv_color_black();
  lv_obj_set_style_bg_color(splash_screen, bg_color, LV_PART_MAIN);

  lv_obj_t* title_label = lv_label_create(splash_screen);
  lv_label_set_text(title_label, "OpenPPG");
  lv_obj_set_style_text_font(title_label, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(title_label, fg_color, 0);
  lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 15);

  lv_obj_t* version_label = lv_label_create(splash_screen);
  char version_str[10];
  snprintf(version_str, sizeof(version_str), "v%d.%d", VERSION_MAJOR, VERSION_MINOR);
  lv_label_set_text(version_label, version_str);
  lv_obj_set_style_text_font(version_label, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(version_label, fg_color, 0);
  lv_obj_align(version_label, LV_ALIGN_CENTER, 0, 0);

  lv_obj_t* time_label = lv_label_create(splash_screen);
  lv_label_set_text(time_label, "02:05");
  lv_obj_set_style_text_font(time_label, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(time_label, fg_color, 0);
  lv_obj_align(time_label, LV_ALIGN_BOTTOM_MID, 0, -20);

  bool main_screen_created = false;
  unsigned int arm_start_millis = 0;

  // Alert system — uses real aggregation logic from alert_display.cpp
  AlertAggregator alerts;
  // Threshold monitors — values from monitor_config.h
  ThresholdCheck escMosCheck  = {SensorID::ESC_MOS_Temp,  90.0f, 110.0f};
  ThresholdCheck motorCheck   = {SensorID::Motor_Temp,   105.0f, 115.0f};
  ThresholdCheck bmsCellCheck = {SensorID::BMS_T1_Temp,   50.0f,  56.0f};

  printf("Rendering frames...\n");

  for (int frame = 0; frame < TOTAL_FRAMES; frame++) {
    float t = (float)frame / FPS;  // time in seconds

    // Determine phase
    Phase phase;
    if (t < 2.0f) phase = SPLASH;
    else if (t < 3.0f) phase = TRANSITION_TO_MAIN;
    else if (t < 5.0f) phase = IDLE_GROUND;
    else if (t < 6.0f) phase = ARMING;
    else if (t < 12.0f) phase = TAKEOFF;
    else if (t < 13.0f) phase = CRUISE_ENGAGE;
    else if (t < 20.0f) phase = CRUISING_STEADY;
    else if (t < 23.0f) phase = TEMP_WARNING;
    else if (t < 26.0f) phase = TEMP_CRITICAL;
    else if (t < 28.0f) phase = DESCEND;
    else if (t < 29.0f) phase = LANDING_DISARM;
    else phase = IDLE_END;

    // --- SPLASH PHASE ---
    if (phase == SPLASH) {
      render_tick(FRAME_MS);
      write_frame_rgb888(pipe);
      continue;
    }

    // --- TRANSITION: delete splash, create main screen ---
    if (!main_screen_created) {
      lv_obj_delete(splash_screen);
      splash_screen = NULL;

      setupMainScreen(darkMode);
      setupAlertCounterUI(darkMode);
      main_screen_created = true;
    }

    // --- Compute telemetry values based on phase ---
    float power_kw = 0.0f;
    float altitude = 0.0f;
    float soc = 95.0f;
    float volts = 96.0f;
    float esc_mos_temp = 42.0f;
    float motor_temp = 55.0f;
    float batt_highest_temp = 32.0f;
    bool armed = false;
    bool cruising = false;

    switch (phase) {
      case TRANSITION_TO_MAIN:
      case IDLE_GROUND: {
        power_kw = 0.0f;
        altitude = 0.0f;
        soc = 95.0f;
        volts = 96.0f;
        break;
      }

      case ARMING: {
        // Use real millis() since updateLvglMainScreen calls millis() internally
        if (arm_start_millis == 0) arm_start_millis = millis();
        armed = true;
        power_kw = 0.0f;
        altitude = 0.0f;
        soc = 95.0f;
        volts = 96.0f;
        break;
      }

      case TAKEOFF: {
        armed = true;
        float takeoff_t = (t - 6.0f) / 6.0f;
        power_kw = lerp(0.0f, 12.0f, ease(std::min(takeoff_t * 2.0f, 1.0f)));
        altitude = lerp(0.0f, 350.0f, ease(takeoff_t));
        soc = lerp(95.0f, 82.0f, takeoff_t);
        volts = lerp(96.0f, 88.0f, takeoff_t);
        esc_mos_temp = lerp(42.0f, 58.0f, takeoff_t);
        motor_temp = lerp(55.0f, 72.0f, takeoff_t);
        break;
      }

      case CRUISE_ENGAGE: {
        armed = true;
        cruising = true;
        float cruise_t = (t - 12.0f) / 1.0f;
        power_kw = lerp(12.0f, 5.5f, ease(cruise_t));
        altitude = lerp(350.0f, 380.0f, cruise_t);
        soc = lerp(82.0f, 80.0f, cruise_t);
        volts = lerp(88.0f, 89.5f, cruise_t);
        esc_mos_temp = lerp(58.0f, 55.0f, cruise_t);
        motor_temp = lerp(72.0f, 68.0f, cruise_t);

        if (cruise_t < 0.05f && cruise_icon_img != NULL) {
          startCruiseIconFlash();
        }
        break;
      }

      case CRUISING_STEADY: {
        armed = true;
        cruising = true;
        float steady_t = (t - 13.0f) / 7.0f;
        power_kw = 5.5f + sinf(steady_t * 6.28f) * 0.3f;
        altitude = lerp(380.0f, 420.0f, steady_t);
        soc = lerp(80.0f, 68.0f, steady_t);
        volts = lerp(89.5f, 86.0f, steady_t);
        esc_mos_temp = lerp(55.0f, 85.0f, steady_t);
        motor_temp = lerp(68.0f, 100.0f, steady_t);
        batt_highest_temp = lerp(32.0f, 42.0f, steady_t);
        break;
      }

      case TEMP_WARNING: {
        armed = true;
        cruising = true;
        float warn_t = (t - 20.0f) / 3.0f;
        power_kw = lerp(5.5f, 4.0f, warn_t);
        altitude = lerp(420.0f, 400.0f, warn_t);
        soc = lerp(68.0f, 62.0f, warn_t);
        volts = lerp(86.0f, 84.5f, warn_t);
        esc_mos_temp = lerp(85.0f, 95.0f, warn_t);
        motor_temp = lerp(100.0f, 108.0f, warn_t);
        batt_highest_temp = lerp(42.0f, 48.0f, warn_t);
        break;
      }

      case TEMP_CRITICAL: {
        armed = true;
        cruising = false;  // disengage cruise
        float crit_t = (t - 23.0f) / 3.0f;
        power_kw = lerp(4.0f, 1.0f, ease(crit_t));
        altitude = lerp(400.0f, 300.0f, ease(crit_t));
        soc = lerp(62.0f, 58.0f, crit_t);
        volts = lerp(84.5f, 85.5f, crit_t);
        esc_mos_temp = lerp(95.0f, 112.0f, crit_t);
        motor_temp = lerp(108.0f, 118.0f, crit_t);
        batt_highest_temp = lerp(48.0f, 52.0f, crit_t);
        break;
      }

      case DESCEND: {
        armed = true;
        float desc_t = (t - 26.0f) / 2.0f;
        power_kw = lerp(1.0f, 0.0f, ease(desc_t));
        altitude = lerp(300.0f, 20.0f, ease(desc_t));
        soc = lerp(58.0f, 56.0f, desc_t);
        volts = lerp(85.5f, 90.0f, desc_t);
        esc_mos_temp = lerp(112.0f, 95.0f, desc_t);
        motor_temp = lerp(118.0f, 105.0f, desc_t);
        batt_highest_temp = lerp(52.0f, 48.0f, desc_t);
        break;
      }

      case LANDING_DISARM: {
        float land_t = (t - 28.0f) / 1.0f;
        if (land_t < 0.5f) {
          armed = true;
        } else {
          armed = false;
          arm_start_millis = 0;
        }
        power_kw = 0.0f;
        altitude = 0.0f;
        soc = 56.0f;
        volts = 92.0f;
        esc_mos_temp = lerp(95.0f, 75.0f, land_t);
        motor_temp = lerp(105.0f, 85.0f, land_t);
        batt_highest_temp = 45.0f;
        break;
      }

      case IDLE_END: {
        armed = false;
        power_kw = 0.0f;
        altitude = 0.0f;
        soc = 56.0f;
        volts = 92.0f;
        esc_mos_temp = 70.0f;
        motor_temp = 80.0f;
        batt_highest_temp = 42.0f;
        break;
      }

      default:
        break;
    }

    // Build telemetry structs
    STR_ESC_TELEMETRY_140 esc = {};
    esc.escState = TelemetryState::CONNECTED;
    esc.volts = volts;
    esc.amps = (volts > 0) ? power_kw * 1000.0f / volts : 0;
    esc.mos_temp = esc_mos_temp;
    esc.cap_temp = esc_mos_temp - 5.0f;
    esc.mcu_temp = esc_mos_temp - 10.0f;
    esc.motor_temp = motor_temp;
    esc.eRPM = power_kw * 1500.0f;
    esc.phase_current = esc.amps * 1.3f;

    STR_BMS_TELEMETRY_140 bms = {};
    bms.bmsState = TelemetryState::CONNECTED;
    bms.soc = soc;
    bms.battery_voltage = volts;
    bms.battery_current = esc.amps;
    bms.power = power_kw;
    bms.highest_cell_voltage = volts / 24.0f + 0.02f;
    bms.lowest_cell_voltage = volts / 24.0f - 0.02f;
    bms.highest_temperature = batt_highest_temp;
    bms.lowest_temperature = batt_highest_temp - 5.0f;
    bms.voltage_differential = 0.04f;
    bms.is_charging = false;
    bms.is_charge_mos = true;
    bms.is_discharge_mos = true;
    bms.battery_ready = true;
    bms.mos_temperature = batt_highest_temp - 2.0f;
    bms.balance_temperature = batt_highest_temp - 4.0f;
    bms.t1_temperature = batt_highest_temp;
    bms.t2_temperature = batt_highest_temp - 2.0f;
    bms.t3_temperature = NAN;
    bms.t4_temperature = NAN;

    UnifiedBatteryData ubd = {};
    ubd.soc = soc;
    ubd.volts = volts;
    ubd.power = power_kw;
    ubd.amps = esc.amps;

    // Update the main screen with telemetry
    updateLvglMainScreen(dd, esc, bms, ubd, altitude, armed, cruising,
                         armed ? arm_start_millis : 0);

    // Run threshold checks and feed events to aggregator
    // (mirrors checkAllSensorsWithData → monitor.check → sendAlertEvent flow)
    AlertLevel lvl;
    lvl = escMosCheck.evaluate(esc_mos_temp);
    if (lvl != escMosCheck.lastLevel || escMosCheck.changed(esc_mos_temp)) {
      // Force re-evaluate to update lastLevel
    }
    {
      // Check each sensor — only fire event when level changes
      AlertLevel newLvl;

      newLvl = AlertLevel::OK;
      if (esc_mos_temp >= 110.0f) newLvl = AlertLevel::CRIT_HIGH;
      else if (esc_mos_temp >= 90.0f) newLvl = AlertLevel::WARN_HIGH;
      if (newLvl != escMosCheck.lastLevel) {
        escMosCheck.lastLevel = newLvl;
        alerts.processEvent(SensorID::ESC_MOS_Temp, newLvl);
      }

      newLvl = AlertLevel::OK;
      if (motor_temp >= 115.0f) newLvl = AlertLevel::CRIT_HIGH;
      else if (motor_temp >= 105.0f) newLvl = AlertLevel::WARN_HIGH;
      if (newLvl != motorCheck.lastLevel) {
        motorCheck.lastLevel = newLvl;
        alerts.processEvent(SensorID::Motor_Temp, newLvl);
      }

      newLvl = AlertLevel::OK;
      if (batt_highest_temp >= 56.0f) newLvl = AlertLevel::CRIT_HIGH;
      else if (batt_highest_temp >= 50.0f) newLvl = AlertLevel::WARN_HIGH;
      if (newLvl != bmsCellCheck.lastLevel) {
        bmsCellCheck.lastLevel = newLvl;
        alerts.processEvent(SensorID::BMS_T1_Temp, newLvl);
      }
    }

    // Run alert rotation (every 2 seconds, same as real aggregation task)
    alerts.tick(millis());

    // Render with realistic tick
    render_tick(FRAME_MS);

    // Write frame to ffmpeg
    write_frame_rgb888(pipe);

    // Progress indicator
    if (frame % FPS == 0) {
      printf("  %ds / %ds (frame %d/%d)\n", frame / FPS, DURATION_SEC, frame, TOTAL_FRAMES);
    }
  }

  printf("  %ds / %ds (frame %d/%d)\n", DURATION_SEC, DURATION_SEC, TOTAL_FRAMES, TOTAL_FRAMES);
  printf("Encoding...\n");
  int ret = pclose(pipe);
  if (ret != 0) {
    fprintf(stderr, "ffmpeg exited with code %d\n", ret);
    return 1;
  }

  printf("Done! Video saved to: %s\n", output_path);
  printf("  (scaled 4x to %dx%d for visibility)\n", SCREEN_WIDTH * 4, SCREEN_HEIGHT * 4);
  return 0;
}
