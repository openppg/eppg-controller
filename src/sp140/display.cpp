#include "sp140/display.h"

#include "../../inc/version.h"
#include "sp140/structs.h"

#define FONT_HEIGHT_OFFSET 8

// DEBUG WATCHDOG
#ifdef RP_PIO
  #include "hardware/watchdog.h"
  bool watchdogCausedReboot = false;
  bool watchdogEnableCausedReboot = false;
#endif

Adafruit_ST7735* display;
GFXcanvas16 canvas(SCREEN_WIDTH, SCREEN_HEIGHT);

// Light Mode Colors
UIColors lightModeColors = {
  BLACK,   // Default Text Color
  RED,     // Error Text Color
  BLUE,    // Chill Text Color
  WHITE,   // Default BG Color
  CYAN,    // Armed BG Color
  YELLOW,  // Cruise BG Color
  BLACK    // UI Accent Color
};

// Dark Mode Colors
UIColors darkModeColors = {
  WHITE,   // Default Text Color
  RED,     // Error Text Color
  CYAN,    // Chill Text Color
  BLACK,   // Default BG Color
  BLUE,    // Armed BG Color
  ORANGE,  // Cruise BG Color
  GRAY     // UI Accent Color
};

// Pointer to the current color set
UIColors *currentTheme;

// Clears screen and resets properties
void resetRotation(unsigned int rotation) {
  display->setRotation(rotation);  // 1=right hand, 3=left hand
}

// Show splash screen with animated text
void displayMeta(const STR_DEVICE_DATA_140_V1& deviceData, int duration) {
  display->fillScreen(currentTheme->default_bg);
  display->setTextSize(1);
  display->setFont(Fonts::Title);
  display->setTextColor(currentTheme->default_text);

  // Animate "OpenPPG" text
  const char* text = "OpenPPG";
  int x = 25;
  int y = 30;
  for (int i = 0; text[i] != '\0'; i++) {
    display->setCursor(x, y);
    display->print(text[i]);
    x += display->getCursorX() - x;  // Move cursor to the right
    delay(100);  // Adjust delay for animation speed
  }

  display->setFont(Fonts::Large);
  display->setTextSize(1);
  display->setCursor(60, 60);
  display->printf("v%d.%d", VERSION_MAJOR, VERSION_MINOR);

  // Total armed time
  display->setCursor(60, 90);

  const int hours = deviceData.armed_time / 60;
  const int minutes = deviceData.armed_time % 60;
  display->printf("%02d:%02d", hours, minutes);
  #ifdef SCREEN_DEBUG
    delay(200);
  #else
    delay(duration);
  #endif
}

// inital screen setup and config with devicedata and boardconfig passed in
void setupDisplay(const STR_DEVICE_DATA_140_V1& deviceData, const HardwareConfig& board_config) {
  USBSerial.println("setupDisplay");
#ifdef RP_PIO
  watchdogCausedReboot = watchdog_caused_reboot();
  watchdogEnableCausedReboot = watchdog_enable_caused_reboot();
#endif

  display = new Adafruit_ST7735(
    board_config.tft_cs,
    board_config.tft_dc,
    board_config.tft_mosi,
    board_config.tft_sclk,
    board_config.tft_rst);

  display->initR(INITR_BLACKTAB);  // Init ST7735S chip, black tab
  resetRotation(deviceData.screen_rotation);
  setTheme(deviceData.theme);  // 0=light, 1=dark
  display->fillScreen(ST77XX_BLACK);
  displayMeta(deviceData);
}

void updateDisplay(
  const STR_DEVICE_DATA_140_V1& deviceData,
  const STR_ESC_TELEMETRY_140& escTelemetry,
  const STR_BMS_TELEMETRY_140& bmsTelemetry,
  const UnifiedBatteryData& unifiedBatteryData,
  float altitude, bool armed, bool cruising,
  unsigned int armedStartMillis
  ) {
  canvas.fillScreen(currentTheme->default_bg);
  canvas.setTextWrap(false);
  canvas.setFont(Fonts::Large);
  canvas.setTextSize(1);

  const unsigned int nowMillis = millis();

  // Display region lines
  canvas.drawFastHLine(0, 32, 160, currentTheme->ui_accent);
  canvas.drawFastVLine(100, 0, 32, currentTheme->ui_accent);
  canvas.drawFastHLine(0, 80, 160, currentTheme->ui_accent);
  canvas.drawFastHLine(0, 92, 160, currentTheme->ui_accent);

  // Display battery level and status
  const float batteryPercent = unifiedBatteryData.soc;

  //   Display battery bar
  if (batteryPercent > 0) {
    unsigned int batteryColor = RED;
    if (batteryPercent >= BATTERY_MEDIUM_THRESHOLD) batteryColor = GREEN;
    else if (batteryPercent >= BATTERY_LOW_THRESHOLD) batteryColor = YELLOW;
    int batteryPercentWidth = map(static_cast<int>(batteryPercent), 0, 100, 0, 100);
    canvas.fillRect(0, 0, batteryPercentWidth, 32, batteryColor);
  } else {
    canvas.setTextColor(currentTheme->error_text);
    if (unifiedBatteryData.volts > 10) {
      canvas.setCursor(6, 15);
      canvas.print("BATTERY");
      canvas.setCursor(6, 31);
      canvas.print(" DEAD");
    } else {
      canvas.setCursor(10, 15);
      canvas.print("NO");
      canvas.setCursor(10, 31);
      canvas.print("TELEM");
    }
  }
  // Draw ends of battery outline
  canvas.fillRect(97, 0, 3, 9, currentTheme->ui_accent);
  canvas.fillRect(97, 23, 3, 9, currentTheme->ui_accent);
  canvas.fillRect(0, 0, 1, 2, currentTheme->ui_accent);
  canvas.fillRect(0, 30, 1, 2, currentTheme->ui_accent);

  // Display battery percent
  canvas.setCursor(108, 16 + FONT_HEIGHT_OFFSET);
  canvas.setTextColor(currentTheme->default_text);
  if (batteryPercent > 0) {
    canvas.printf("%3d%%", static_cast<int>(batteryPercent));
  } else {
    canvas.print(" ?%");
  }

  float kWatts = bmsTelemetry.power;  // Already in kW
  float volts = unifiedBatteryData.volts;
  float kWh = wattHoursUsed / 1000.0;
  float amps = unifiedBatteryData.amps;

  // for testing
  // float kWatts = 15.1;
  // float volts = 98.7;
  // float kWh = 3.343;
  // float amps = 10.35;

  canvas.setFont(Fonts::Medium);

  canvas.setCursor(1, 40 + FONT_HEIGHT_OFFSET);
  canvas.printf(kWatts < 10 ? "  %4.1fkW" : "%4.1fkW", kWatts);

  canvas.setCursor(100, 40 + FONT_HEIGHT_OFFSET);
  canvas.printf(volts > 99.9 ? "%3.0fV" : "%4.1fV", volts);

  canvas.setCursor(1, 55 + FONT_HEIGHT_OFFSET);
  canvas.printf(kWh > 99.9 ? "%3.0fkWh" : "%4.1fkWh", kWh);

  canvas.setCursor(100, 55 + FONT_HEIGHT_OFFSET);
  canvas.printf(amps > 99.9 ? "%3.0fA" : "%4.1fA", amps);

  canvas.setCursor(5, 70 + FONT_HEIGHT_OFFSET);
  canvas.printf("%2.2fV", bmsTelemetry.lowest_cell_voltage);

  canvas.setCursor(50, 70 + FONT_HEIGHT_OFFSET);
  canvas.printf("%0.3fV", bmsTelemetry.voltage_differential);

  canvas.setCursor(100, 70 + FONT_HEIGHT_OFFSET);
  canvas.printf(bmsTelemetry.highest_temperature > 99.9 ? "%3.0fC" : "%4.1fC", bmsTelemetry.highest_temperature);

  // Display modes
  canvas.setCursor(8, 90);
  canvas.setFont(Fonts::Small);
  if (deviceData.performance_mode == 0) {
    canvas.setTextColor(currentTheme->chill_text);
    canvas.print("CHILL");
  } else {
    canvas.setTextColor(RED);
    canvas.print("SPORT");
  }

  // canvas.setCursor(46, 83);
  // if (armed) {
  //   canvas.setTextColor(BLACK, CYAN);
  //   canvas.print("ARMED");
  // } else {
  //   canvas.setTextColor(BLACK, GREEN);
  //   canvas.print("SAFED");
  // }

  // if (cruising) {
  //   canvas.setCursor(84, 83);
  //   canvas.setTextColor(BLACK, YELLOW);
  //   canvas.print("CRUISE");
  // }

  // canvas.setCursor(124, 83);
  // canvas.setTextColor(BLACK);
  // canvas.printf("FLAG%2d", escTelemetry.statusFlag);

  // Display statusbar
  unsigned int statusBarColor = currentTheme->default_bg;
  if (cruising) statusBarColor = currentTheme->cruise_bg;
  else if (armed) statusBarColor = currentTheme->armed_bg;
  canvas.fillRect(0, 93, 160, 40, statusBarColor);

  // Display armed time for the current session
  canvas.setTextColor(currentTheme->default_text);
  canvas.setFont(Fonts::Large);
  canvas.setCursor(8, 108 + FONT_HEIGHT_OFFSET);
  static unsigned int _lastArmedMillis = 0;
  if (armed) {
    _lastArmedMillis = nowMillis;
    const int sessionSeconds = (_lastArmedMillis - armedStartMillis) / 1000.0;
    canvas.printf("%02d:%02d", sessionSeconds / 60, sessionSeconds % 60);
  } else {
    // Show current time in 24hr format when disarmed
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    canvas.printf("%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
  }

  // Display altitude
  canvas.setCursor(80, 108 + FONT_HEIGHT_OFFSET);
  if (altitude == __FLT_MIN__) {
    canvas.setTextColor(currentTheme->error_text);
    canvas.print(F("ALTERR"));
  } else {
    canvas.setTextColor(currentTheme->default_text);
    if (deviceData.metric_alt) {
      canvas.printf("%6.1fm", altitude);
    } else {
      canvas.printf("%5dft", static_cast<int>(round(altitude * 3.28084)));
    }
  }

  // ESC temperatures
  // Display MOSFET temperature
  canvas.setCursor(120, 90);
  canvas.setTextColor(currentTheme->default_text);
  canvas.setFont(Fonts::Small);
  canvas.print("F:");
  float mos_temp;
  mos_temp = escTelemetry.mos_temp;

  if (mos_temp >= 100) {
    canvas.setTextColor(currentTheme->error_text);  // If temperature is 100C+, display in red.
  }
  if (mos_temp == __FLT_MIN__ || mos_temp == 0.0) {  // If temperature is not available, display a question mark.
    canvas.printf("?%c", 247);
  } else {  // Otherwise, display the temperature. (in degrees C)
    canvas.printf("%0.0f", mos_temp);
  }

  // Display capacitor temperature
  canvas.setCursor(80, 90);
  canvas.setTextColor(currentTheme->default_text);
  canvas.print("C:");
  float cap_temp;
  cap_temp = escTelemetry.cap_temp;

  if (cap_temp == __FLT_MIN__ || cap_temp == 0.0) {  // If temperature is not available, display a question mark.
    canvas.print("?c");
  } else {  // Otherwise, display the temperature. (in degrees C)
    canvas.printf("%0.0f", cap_temp);
  }

  // Display MCU temperature
  canvas.setCursor(45, 90);
  canvas.setTextColor(currentTheme->default_text);
  canvas.print("M:");
  float mcu_temp;
  mcu_temp = escTelemetry.mcu_temp;

  if (mcu_temp == __FLT_MIN__ || mcu_temp == 0.0) {  // If temperature is not available, display a question mark.
    canvas.print("?c");
  } else {  // Otherwise, display the temperature. (in degrees C)
    canvas.printf("%0.0f", mcu_temp);
  }

//  // DEBUG TIMING
//  canvas.setTextSize(1);
//  canvas.setCursor(4, 118);
//  static unsigned int lastDisplayMillis = 0;
//  canvas.printf("%5d  %5d", nowMillis - escTelemetry.lastUpdateMillis, nowMillis - lastDisplayMillis);
//  lastDisplayMillis = nowMillis;
//
//  canvas.printf("  %3d %2d %2d", escTelemetry.lastReadBytes, escTelemetry.errorStopBytes, escTelemetry.errorChecksum);

//  // DEBUG WATCHDOG
//  #ifdef RP_PIO
//    canvas.setTextSize(1);
//    canvas.setCursor(4, 118);
//    canvas.printf("watchdog %d %d", watchdogCausedReboot, watchdogEnableCausedReboot);
//  #endif
//
//  // DEBUG FREE MEMORY
//  #ifdef RP_PIO
//    canvas.printf("  mem %d", rp2040.getFreeHeap());
//  #endif


  // Draw the canvas to the display->
  display->drawRGBBitmap(0, 0, canvas.getBuffer(), canvas.width(), canvas.height());
}

// Set the theme
void setTheme(int theme) {
  if (theme == 1) {
    currentTheme = &darkModeColors;
    // Serial.println("Switched to Dark Mode");
  } else {
    currentTheme = &lightModeColors;
    // Serial.println("Switched to Light Mode");
  }
}
