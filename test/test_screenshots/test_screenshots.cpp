#include <gtest/gtest.h>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <sys/stat.h>

#include "emulator_display.h"
#include "sp140/lvgl/lvgl_main_screen.h"
#include "sp140/lvgl/lvgl_updates.h"
#include "sp140/lvgl/lvgl_alerts.h"
#include "sp140/structs.h"
#include "version.h"

// Helper: path to reference screenshots (relative to project root)
static const char* REFERENCE_DIR = "test/test_screenshots/reference";
static const char* OUTPUT_DIR = "test/test_screenshots/output";

static bool file_exists(const char* path) {
  struct stat st;
  return stat(path, &st) == 0;
}

static void ensure_output_dir() {
  mkdir(OUTPUT_DIR, 0755);
}

// Create default device data for testing
static STR_DEVICE_DATA_140_V1 make_default_device_data(bool darkMode = false) {
  STR_DEVICE_DATA_140_V1 dd = {};
  dd.version_major = 8;
  dd.version_minor = 0;
  dd.armed_time = 125;  // 2h05m
  dd.screen_rotation = 1;
  dd.sea_pressure = 1013.25f;
  dd.metric_temp = true;
  dd.metric_alt = true;
  dd.performance_mode = 1;
  dd.theme = darkMode ? 1 : 0;
  dd.revision = 3;
  dd.timezone_offset = 0;
  return dd;
}

// Create typical in-flight ESC telemetry
static STR_ESC_TELEMETRY_140 make_esc_connected() {
  STR_ESC_TELEMETRY_140 esc = {};
  esc.escState = TelemetryState::CONNECTED;
  esc.volts = 88.5f;
  esc.amps = 45.2f;
  esc.mos_temp = 52.0f;
  esc.cap_temp = 48.0f;
  esc.mcu_temp = 42.0f;
  esc.motor_temp = 65.0f;
  esc.eRPM = 12000.0f;
  return esc;
}

// Create typical BMS telemetry
static STR_BMS_TELEMETRY_140 make_bms_connected() {
  STR_BMS_TELEMETRY_140 bms = {};
  bms.bmsState = TelemetryState::CONNECTED;
  bms.soc = 72.0f;
  bms.battery_voltage = 88.5f;
  bms.battery_current = 45.2f;
  bms.power = 4.0f;
  bms.highest_cell_voltage = 3.95f;
  bms.lowest_cell_voltage = 3.90f;
  bms.highest_temperature = 35.0f;
  bms.lowest_temperature = 28.0f;
  bms.voltage_differential = 0.05f;
  bms.is_charging = false;
  bms.is_charge_mos = true;
  bms.is_discharge_mos = true;
  bms.mos_temperature = 32.0f;
  bms.balance_temperature = 30.0f;
  bms.t1_temperature = 35.0f;
  bms.t2_temperature = 33.0f;
  bms.t3_temperature = NAN;
  bms.t4_temperature = NAN;
  return bms;
}

static UnifiedBatteryData make_unified_battery(float soc, float volts, float power) {
  UnifiedBatteryData ubd = {};
  ubd.soc = soc;
  ubd.volts = volts;
  ubd.power = power;
  ubd.amps = (volts > 0) ? power * 1000.0f / volts : 0;
  return ubd;
}

// ============================================================
// Test fixture
// ============================================================
class ScreenshotTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ensure_output_dir();
  }

  void TearDown() override {
    emulator_teardown();
  }

  // Set up display + main screen, render with given data, save screenshot
  void render_and_save(const char* name, bool darkMode,
                       const STR_DEVICE_DATA_140_V1& dd,
                       const STR_ESC_TELEMETRY_140& esc,
                       const STR_BMS_TELEMETRY_140& bms,
                       const UnifiedBatteryData& ubd,
                       float altitude, bool armed, bool cruising,
                       unsigned int armedStartMillis = 0) {
    emulator_init_display(darkMode);
    setupMainScreen(darkMode);

    // Apply data
    updateLvglMainScreen(dd, esc, bms, ubd, altitude, armed, cruising, armedStartMillis);

    // Render
    emulator_render_frame();

    // Save output
    char out_path[256];
    snprintf(out_path, sizeof(out_path), "%s/%s.bmp", OUTPUT_DIR, name);
    ASSERT_TRUE(emulator_save_bmp(out_path)) << "Failed to save " << out_path;

    // Compare with reference if it exists
    char ref_path[256];
    snprintf(ref_path, sizeof(ref_path), "%s/%s.bmp", REFERENCE_DIR, name);
    if (file_exists(ref_path)) {
      int diff = emulator_compare_bmp(ref_path, out_path);
      if (diff > 0) {
        char diff_path[256];
        snprintf(diff_path, sizeof(diff_path), "%s/%s_diff.bmp", OUTPUT_DIR, name);
        emulator_save_diff_bmp(ref_path, out_path, diff_path);
        EXPECT_EQ(0, diff)
          << "Screenshot regression: " << name << " has " << diff << " differing pixels\n"
          << "  Reference: " << ref_path << "\n"
          << "  Output:    " << out_path << "\n"
          << "  Diff:      " << diff_path << "  (magenta = changed pixels)\n"
          << "  View all:  open " << OUTPUT_DIR;
      }
    } else {
      // No reference yet - copy output as new reference
      printf("  [INFO] No reference for '%s' - generating initial reference\n", name);
      FILE* src = fopen(out_path, "rb");
      FILE* dst = fopen(ref_path, "wb");
      if (src && dst) {
        char buf[4096];
        size_t n;
        while ((n = fread(buf, 1, sizeof(buf), src)) > 0) {
          fwrite(buf, 1, n, dst);
        }
      }
      if (src) fclose(src);
      if (dst) fclose(dst);
    }
  }
};

// ============================================================
// Test cases - each generates a screenshot of a different UI state
// ============================================================

TEST_F(ScreenshotTest, MainScreen_Idle_Light) {
  auto dd = make_default_device_data(false);
  auto esc = make_esc_connected();
  auto bms = make_bms_connected();
  auto ubd = make_unified_battery(72.0f, 88.5f, 0.0f);

  render_and_save("main_idle_light", false, dd, esc, bms, ubd,
                  0.0f, false, false);
}

TEST_F(ScreenshotTest, MainScreen_Idle_Dark) {
  auto dd = make_default_device_data(true);
  auto esc = make_esc_connected();
  auto bms = make_bms_connected();
  auto ubd = make_unified_battery(72.0f, 88.5f, 0.0f);

  render_and_save("main_idle_dark", true, dd, esc, bms, ubd,
                  0.0f, false, false);
}

TEST_F(ScreenshotTest, MainScreen_Armed_Flying_Light) {
  auto dd = make_default_device_data(false);
  auto esc = make_esc_connected();
  auto bms = make_bms_connected();
  auto ubd = make_unified_battery(65.0f, 86.0f, 8.5f);

  render_and_save("main_armed_flying_light", false, dd, esc, bms, ubd,
                  450.3f, true, false, millis() - 180000);
}

TEST_F(ScreenshotTest, MainScreen_Armed_Flying_Dark) {
  auto dd = make_default_device_data(true);
  auto esc = make_esc_connected();
  auto bms = make_bms_connected();
  auto ubd = make_unified_battery(65.0f, 86.0f, 8.5f);

  render_and_save("main_armed_flying_dark", true, dd, esc, bms, ubd,
                  450.3f, true, false, millis() - 180000);
}

TEST_F(ScreenshotTest, MainScreen_Armed_Cruising_Light) {
  auto dd = make_default_device_data(false);
  auto esc = make_esc_connected();
  auto bms = make_bms_connected();
  auto ubd = make_unified_battery(58.0f, 84.0f, 6.2f);

  render_and_save("main_armed_cruising_light", false, dd, esc, bms, ubd,
                  820.7f, true, true, millis() - 600000);
}

TEST_F(ScreenshotTest, MainScreen_Armed_Cruising_Dark) {
  auto dd = make_default_device_data(true);
  auto esc = make_esc_connected();
  auto bms = make_bms_connected();
  auto ubd = make_unified_battery(58.0f, 84.0f, 6.2f);

  render_and_save("main_armed_cruising_dark", true, dd, esc, bms, ubd,
                  820.7f, true, true, millis() - 600000);
}

TEST_F(ScreenshotTest, MainScreen_LowBattery_Light) {
  auto dd = make_default_device_data(false);
  auto esc = make_esc_connected();
  auto bms = make_bms_connected();
  bms.soc = 8.0f;
  bms.lowest_cell_voltage = 3.15f;
  auto ubd = make_unified_battery(8.0f, 72.0f, 2.0f);

  render_and_save("main_low_battery_light", false, dd, esc, bms, ubd,
                  200.0f, true, false, millis() - 300000);
}

TEST_F(ScreenshotTest, MainScreen_LowBattery_Dark) {
  auto dd = make_default_device_data(true);
  auto esc = make_esc_connected();
  auto bms = make_bms_connected();
  bms.soc = 8.0f;
  bms.lowest_cell_voltage = 3.15f;
  auto ubd = make_unified_battery(8.0f, 72.0f, 2.0f);

  render_and_save("main_low_battery_dark", true, dd, esc, bms, ubd,
                  200.0f, true, false, millis() - 300000);
}

TEST_F(ScreenshotTest, MainScreen_HighAltitude_Light) {
  auto dd = make_default_device_data(false);
  dd.metric_alt = true;
  auto esc = make_esc_connected();
  auto bms = make_bms_connected();
  auto ubd = make_unified_battery(45.0f, 82.0f, 4.5f);

  render_and_save("main_high_altitude_light", false, dd, esc, bms, ubd,
                  2456.8f, true, false, millis() - 900000);
}

TEST_F(ScreenshotTest, MainScreen_HighAltitude_Dark) {
  auto dd = make_default_device_data(true);
  dd.metric_alt = true;
  auto esc = make_esc_connected();
  auto bms = make_bms_connected();
  auto ubd = make_unified_battery(45.0f, 82.0f, 4.5f);

  render_and_save("main_high_altitude_dark", true, dd, esc, bms, ubd,
                  2456.8f, true, false, millis() - 900000);
}

TEST_F(ScreenshotTest, MainScreen_HighPower_Light) {
  auto dd = make_default_device_data(false);
  dd.performance_mode = 2;
  auto esc = make_esc_connected();
  esc.mos_temp = 85.0f;
  esc.motor_temp = 95.0f;
  auto bms = make_bms_connected();
  bms.highest_temperature = 48.0f;
  auto ubd = make_unified_battery(50.0f, 85.0f, 18.5f);

  render_and_save("main_high_power_light", false, dd, esc, bms, ubd,
                  300.0f, true, false, millis() - 60000);
}

TEST_F(ScreenshotTest, MainScreen_HighPower_Dark) {
  auto dd = make_default_device_data(true);
  dd.performance_mode = 2;
  auto esc = make_esc_connected();
  esc.mos_temp = 85.0f;
  esc.motor_temp = 95.0f;
  auto bms = make_bms_connected();
  bms.highest_temperature = 48.0f;
  auto ubd = make_unified_battery(50.0f, 85.0f, 18.5f);

  render_and_save("main_high_power_dark", true, dd, esc, bms, ubd,
                  300.0f, true, false, millis() - 60000);
}

TEST_F(ScreenshotTest, MainScreen_Charging_Light) {
  auto dd = make_default_device_data(false);
  auto esc = STR_ESC_TELEMETRY_140{};
  esc.escState = TelemetryState::NOT_CONNECTED;
  auto bms = make_bms_connected();
  bms.is_charging = true;
  bms.soc = 85.0f;
  auto ubd = make_unified_battery(85.0f, 96.0f, -0.5f);

  render_and_save("main_charging_light", false, dd, esc, bms, ubd,
                  0.0f, false, false);
}

TEST_F(ScreenshotTest, MainScreen_Charging_Dark) {
  auto dd = make_default_device_data(true);
  auto esc = STR_ESC_TELEMETRY_140{};
  esc.escState = TelemetryState::NOT_CONNECTED;
  auto bms = make_bms_connected();
  bms.is_charging = true;
  bms.soc = 85.0f;
  auto ubd = make_unified_battery(85.0f, 96.0f, -0.5f);

  render_and_save("main_charging_dark", true, dd, esc, bms, ubd,
                  0.0f, false, false);
}

TEST_F(ScreenshotTest, MainScreen_ESCDisconnected_Light) {
  auto dd = make_default_device_data(false);
  auto esc = STR_ESC_TELEMETRY_140{};
  esc.escState = TelemetryState::NOT_CONNECTED;
  auto bms = make_bms_connected();
  auto ubd = make_unified_battery(72.0f, 88.5f, 0.0f);

  render_and_save("main_esc_disconnected_light", false, dd, esc, bms, ubd,
                  0.0f, false, false);
}

TEST_F(ScreenshotTest, MainScreen_ESCDisconnected_Dark) {
  auto dd = make_default_device_data(true);
  auto esc = STR_ESC_TELEMETRY_140{};
  esc.escState = TelemetryState::NOT_CONNECTED;
  auto bms = make_bms_connected();
  auto ubd = make_unified_battery(72.0f, 88.5f, 0.0f);

  render_and_save("main_esc_disconnected_dark", true, dd, esc, bms, ubd,
                  0.0f, false, false);
}

TEST_F(ScreenshotTest, MainScreen_FullBattery_Light) {
  auto dd = make_default_device_data(false);
  auto esc = make_esc_connected();
  auto bms = make_bms_connected();
  bms.soc = 100.0f;
  bms.highest_cell_voltage = 4.20f;
  bms.lowest_cell_voltage = 4.18f;
  auto ubd = make_unified_battery(100.0f, 100.8f, 0.0f);

  render_and_save("main_full_battery_light", false, dd, esc, bms, ubd,
                  0.0f, false, false);
}

TEST_F(ScreenshotTest, MainScreen_FullBattery_Dark) {
  auto dd = make_default_device_data(true);
  auto esc = make_esc_connected();
  auto bms = make_bms_connected();
  bms.soc = 100.0f;
  bms.highest_cell_voltage = 4.20f;
  bms.lowest_cell_voltage = 4.18f;
  auto ubd = make_unified_battery(100.0f, 100.8f, 0.0f);

  render_and_save("main_full_battery_dark", true, dd, esc, bms, ubd,
                  0.0f, false, false);
}

// ============================================================
// Warning & alert tests
// ============================================================

// ESC temp in warning range (>=90), motor temp in warning range (>=105)
TEST_F(ScreenshotTest, MainScreen_TempWarnings_Light) {
  auto dd = make_default_device_data(false);
  auto esc = make_esc_connected();
  esc.mos_temp = 95.0f;     // Above escMosTempThresholds.warnHigh (90)
  esc.motor_temp = 108.0f;  // Above motorTempThresholds.warnHigh (105)
  auto bms = make_bms_connected();
  bms.highest_temperature = 52.0f;  // Above bmsCellTempThresholds.warnHigh (50)
  bms.t1_temperature = 52.0f;
  auto ubd = make_unified_battery(60.0f, 85.0f, 6.0f);

  render_and_save("main_temp_warnings_light", false, dd, esc, bms, ubd,
                  300.0f, true, false, millis() - 120000);
}

TEST_F(ScreenshotTest, MainScreen_TempWarnings_Dark) {
  auto dd = make_default_device_data(true);
  auto esc = make_esc_connected();
  esc.mos_temp = 95.0f;     // Above escMosTempThresholds.warnHigh (90)
  esc.motor_temp = 108.0f;  // Above motorTempThresholds.warnHigh (105)
  auto bms = make_bms_connected();
  bms.highest_temperature = 52.0f;  // Above bmsCellTempThresholds.warnHigh (50)
  bms.t1_temperature = 52.0f;
  auto ubd = make_unified_battery(60.0f, 85.0f, 6.0f);

  render_and_save("main_temp_warnings_dark", true, dd, esc, bms, ubd,
                  300.0f, true, false, millis() - 120000);
}

// ESC temp in critical range (>=110), motor temp critical (>=115), battery critical (>=56)
TEST_F(ScreenshotTest, MainScreen_TempCritical_Light) {
  auto dd = make_default_device_data(false);
  auto esc = make_esc_connected();
  esc.mos_temp = 115.0f;    // Above escMosTempThresholds.critHigh (110)
  esc.motor_temp = 120.0f;  // Above motorTempThresholds.critHigh (115)
  auto bms = make_bms_connected();
  bms.highest_temperature = 58.0f;  // Above bmsCellTempThresholds.critHigh (56)
  bms.t1_temperature = 58.0f;
  auto ubd = make_unified_battery(40.0f, 80.0f, 12.0f);

  render_and_save("main_temp_critical_light", false, dd, esc, bms, ubd,
                  250.0f, true, false, millis() - 180000);
}

TEST_F(ScreenshotTest, MainScreen_TempCritical_Dark) {
  auto dd = make_default_device_data(true);
  auto esc = make_esc_connected();
  esc.mos_temp = 115.0f;    // Above escMosTempThresholds.critHigh (110)
  esc.motor_temp = 120.0f;  // Above motorTempThresholds.critHigh (115)
  auto bms = make_bms_connected();
  bms.highest_temperature = 58.0f;  // Above bmsCellTempThresholds.critHigh (56)
  bms.t1_temperature = 58.0f;
  auto ubd = make_unified_battery(40.0f, 80.0f, 12.0f);

  render_and_save("main_temp_critical_dark", true, dd, esc, bms, ubd,
                  250.0f, true, false, millis() - 180000);
}

// Helper: save and compare for manual (non-render_and_save) tests
static void save_and_compare(const char* name) {
  char out_path[256], ref_path[256];
  snprintf(out_path, sizeof(out_path), "%s/%s.bmp", OUTPUT_DIR, name);
  snprintf(ref_path, sizeof(ref_path), "%s/%s.bmp", REFERENCE_DIR, name);
  ASSERT_TRUE(emulator_save_bmp(out_path)) << "Failed to save " << out_path;

  if (file_exists(ref_path)) {
    int diff = emulator_compare_bmp(ref_path, out_path);
    if (diff > 0) {
      char diff_path[256];
      snprintf(diff_path, sizeof(diff_path), "%s/%s_diff.bmp", OUTPUT_DIR, name);
      emulator_save_diff_bmp(ref_path, out_path, diff_path);
      EXPECT_EQ(0, diff) << "Screenshot regression: " << name << " has " << diff << " differing pixels";
    }
  } else {
    printf("  [INFO] No reference for '%s' - generating initial reference\n", name);
    FILE* src = fopen(out_path, "rb");
    FILE* dst = fopen(ref_path, "wb");
    if (src && dst) { char buf[4096]; size_t n; while ((n = fread(buf, 1, sizeof(buf), src)) > 0) fwrite(buf, 1, n, dst); }
    if (src) fclose(src);
    if (dst) fclose(dst);
  }
}

// Warning alert counters + text visible (driven by alert display system)
TEST_F(ScreenshotTest, MainScreen_WarningAlerts_Light) {
  auto dd = make_default_device_data(false);
  auto esc = make_esc_connected();
  esc.mos_temp = 92.0f;  // Warning-range temp
  auto bms = make_bms_connected();
  auto ubd = make_unified_battery(55.0f, 84.0f, 5.0f);

  emulator_init_display(false);
  setupMainScreen(false);
  updateLvglMainScreen(dd, esc, bms, ubd, 400.0f, true, false, millis() - 60000);

  AlertCounts counts = {.warningCount = 2, .criticalCount = 0};
  updateAlertCounterDisplay(counts);
  lv_showAlertTextWithLevel(SensorID::ESC_MOS_Temp, AlertLevel::WARN_HIGH, false);

  emulator_render_frame();
  save_and_compare("main_warning_alerts_light");
}

TEST_F(ScreenshotTest, MainScreen_WarningAlerts_Dark) {
  auto dd = make_default_device_data(true);
  auto esc = make_esc_connected();
  esc.mos_temp = 92.0f;  // Warning-range temp
  auto bms = make_bms_connected();
  auto ubd = make_unified_battery(55.0f, 84.0f, 5.0f);

  emulator_init_display(true);
  setupMainScreen(true);
  updateLvglMainScreen(dd, esc, bms, ubd, 400.0f, true, false, millis() - 60000);

  AlertCounts counts = {.warningCount = 2, .criticalCount = 0};
  updateAlertCounterDisplay(counts);
  lv_showAlertTextWithLevel(SensorID::ESC_MOS_Temp, AlertLevel::WARN_HIGH, false);

  emulator_render_frame();
  save_and_compare("main_warning_alerts_dark");
}

// Critical alert counters + text visible (both warning and critical)
TEST_F(ScreenshotTest, MainScreen_CriticalAlerts_Light) {
  auto dd = make_default_device_data(false);
  auto esc = make_esc_connected();
  esc.mos_temp = 115.0f;
  esc.motor_temp = 120.0f;
  auto bms = make_bms_connected();
  bms.highest_temperature = 58.0f;
  bms.t1_temperature = 58.0f;
  auto ubd = make_unified_battery(35.0f, 78.0f, 14.0f);

  emulator_init_display(false);
  setupMainScreen(false);
  updateLvglMainScreen(dd, esc, bms, ubd, 200.0f, true, false, millis() - 300000);

  AlertCounts counts = {.warningCount = 1, .criticalCount = 3};
  updateAlertCounterDisplay(counts);
  lv_showAlertTextWithLevel(SensorID::Motor_Temp, AlertLevel::CRIT_HIGH, true);
  lv_showAlertTextWithLevel(SensorID::ESC_MOS_Temp, AlertLevel::WARN_HIGH, false);

  if (critical_border != NULL) {
    lv_obj_set_style_border_opa(critical_border, LV_OPA_100, LV_PART_MAIN);
  }

  emulator_render_frame();
  save_and_compare("main_critical_alerts_light");
}

TEST_F(ScreenshotTest, MainScreen_CriticalAlerts_Dark) {
  auto dd = make_default_device_data(true);
  auto esc = make_esc_connected();
  esc.mos_temp = 115.0f;
  esc.motor_temp = 120.0f;
  auto bms = make_bms_connected();
  bms.highest_temperature = 58.0f;
  bms.t1_temperature = 58.0f;
  auto ubd = make_unified_battery(35.0f, 78.0f, 14.0f);

  emulator_init_display(true);
  setupMainScreen(true);
  updateLvglMainScreen(dd, esc, bms, ubd, 200.0f, true, false, millis() - 300000);

  AlertCounts counts = {.warningCount = 1, .criticalCount = 3};
  updateAlertCounterDisplay(counts);
  lv_showAlertTextWithLevel(SensorID::Motor_Temp, AlertLevel::CRIT_HIGH, true);
  lv_showAlertTextWithLevel(SensorID::ESC_MOS_Temp, AlertLevel::WARN_HIGH, false);

  if (critical_border != NULL) {
    lv_obj_set_style_border_opa(critical_border, LV_OPA_100, LV_PART_MAIN);
  }

  emulator_render_frame();
  save_and_compare("main_critical_alerts_dark");
}

// ============================================================
// Splash screen tests (recreated from lvgl_core.cpp displayLvglSplash)
// Uses VERSION_MAJOR/VERSION_MINOR from version.h so references
// auto-update on rebuild after a version bump — just run
// build_and_run.sh --update-references after changing the version.
// ============================================================

// Helper to render a splash screen and save/compare
static void render_splash(const char* name, bool darkMode) {
  lv_obj_t* splash_screen = lv_obj_create(NULL);
  lv_scr_load(splash_screen);
  lv_obj_clear_flag(splash_screen, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(splash_screen,
                            darkMode ? lv_color_black() : lv_color_white(),
                            LV_PART_MAIN);

  lv_obj_t* title_label = lv_label_create(splash_screen);
  lv_label_set_text(title_label, "OpenPPG");
  lv_obj_set_style_text_font(title_label, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(title_label,
                              darkMode ? lv_color_white() : lv_color_black(), 0);
  lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 15);

  lv_obj_t* version_label = lv_label_create(splash_screen);
  char version_str[10];
  snprintf(version_str, sizeof(version_str), "v%d.%d", VERSION_MAJOR, VERSION_MINOR);
  lv_label_set_text(version_label, version_str);
  lv_obj_set_style_text_font(version_label, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(version_label,
                              darkMode ? lv_color_white() : lv_color_black(), 0);
  lv_obj_align(version_label, LV_ALIGN_CENTER, 0, 0);

  lv_obj_t* time_label = lv_label_create(splash_screen);
  lv_label_set_text(time_label, "02:05");
  lv_obj_set_style_text_font(time_label, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(time_label,
                              darkMode ? lv_color_white() : lv_color_black(), 0);
  lv_obj_align(time_label, LV_ALIGN_BOTTOM_MID, 0, -20);

  emulator_render_frame();

  char out_path[256], ref_path[256];
  snprintf(out_path, sizeof(out_path), "%s/%s.bmp", OUTPUT_DIR, name);
  snprintf(ref_path, sizeof(ref_path), "%s/%s.bmp", REFERENCE_DIR, name);
  emulator_save_bmp(out_path);

  if (file_exists(ref_path)) {
    int diff = emulator_compare_bmp(ref_path, out_path);
    if (diff > 0) {
      char diff_path[256];
      snprintf(diff_path, sizeof(diff_path), "%s/%s_diff.bmp", OUTPUT_DIR, name);
      emulator_save_diff_bmp(ref_path, out_path, diff_path);
      EXPECT_EQ(0, diff) << "Screenshot regression: " << name << " has " << diff << " differing pixels";
    }
  } else {
    printf("  [INFO] No reference for '%s' - generating initial reference\n", name);
    FILE* src = fopen(out_path, "rb");
    FILE* dst = fopen(ref_path, "wb");
    if (src && dst) { char buf[4096]; size_t n; while ((n = fread(buf, 1, sizeof(buf), src)) > 0) fwrite(buf, 1, n, dst); }
    if (src) fclose(src);
    if (dst) fclose(dst);
  }

  // Load a scratch screen before deleting so LVGL's active-screen pointer
  // is not left dangling between tests.
  lv_obj_t* scratch = lv_obj_create(NULL);
  lv_scr_load(scratch);
  lv_obj_del(splash_screen);
}

TEST_F(ScreenshotTest, SplashScreen_Light) {
  emulator_init_display(false);
  render_splash("splash_light", false);
}

TEST_F(ScreenshotTest, SplashScreen_Dark) {
  emulator_init_display(true);
  render_splash("splash_dark", true);
}
