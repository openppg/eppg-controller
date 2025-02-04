#include "sp140/display.h"
#include "../../inc/version.h"
#include "sp140/structs.h"

#define FONT_HEIGHT_OFFSET 8

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

// Add this helper function near the top with other functions
void drawBluetoothSymbol(GFXcanvas16& canvas, int16_t x, int16_t y, uint16_t color) {
  // Draw a smaller bluetooth symbol (6x8 pixels)
  canvas.drawLine(x + 2, y, x + 2, y + 7, color);     // Vertical line
  canvas.drawLine(x + 2, y, x + 4, y + 2, color);     // Upper right
  canvas.drawLine(x + 4, y + 2, x + 2, y + 4, color); // Middle right upper
  canvas.drawLine(x + 2, y + 4, x + 4, y + 6, color); // Middle right lower
  canvas.drawLine(x + 4, y + 6, x + 2, y + 8, color); // Lower right
  canvas.drawLine(x, y + 2, x + 2, y + 4, color);     // Middle left upper
  canvas.drawLine(x, y + 6, x + 2, y + 4, color);     // Middle left lower
}

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
    delay(150);
  #else
    delay(duration);
  #endif
}

// inital screen setup and config with devicedata and boardconfig passed in
void setupDisplay(const STR_DEVICE_DATA_140_V1& deviceData, const HardwareConfig& board_config) {
  USBSerial.println("setupDisplay");

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
  float batteryPercent = unifiedBatteryData.soc;
  float totalVolts = unifiedBatteryData.volts;
  float lowestCellV = bmsTelemetry.lowest_cell_voltage;
  float batteryTemp = bmsTelemetry.highest_temperature;
  float escTemp = escTelemetry.mos_temp;
  float motorTemp = escTelemetry.motor_temp;

  canvas.fillScreen(currentTheme->default_bg);
  canvas.setTextWrap(false);
  canvas.setFont(Fonts::Medium);
  canvas.setTextSize(1);

  // Draw all backgrounds first
  // Top section backgrounds
  // Battery background if connected and has percentage
  if (bmsTelemetry.state == TelemetryState::CONNECTED && batteryPercent > 0) {
    unsigned int batteryColor = RED;
    if (batteryPercent >= BATTERY_MEDIUM_THRESHOLD) batteryColor = GREEN;
    else if (batteryPercent >= BATTERY_LOW_THRESHOLD) batteryColor = YELLOW;
    int batteryWidth = map(static_cast<int>(batteryPercent), 0, 100, 0, SCREEN_WIDTH);
    canvas.fillRect(0, 0, batteryWidth, 32, batteryColor);
  }

  // Middle section armed background
  if (armed) {
    canvas.fillRect(90, 32, 70, 43, CYAN);
  }

  // Temperature section backgrounds
  const int tempBoxHeight = 17;
  const int tempStartY = 75;
  const int tempBoxWidth = 40;
  const int tempBoxX = 120;

  // Draw temperature backgrounds only if devices are connected
  if (bmsTelemetry.state == TelemetryState::CONNECTED) {
    if (batteryTemp >= TEMP_CRITICAL_THRESHOLD) {
      canvas.fillRect(tempBoxX, tempStartY, tempBoxWidth, tempBoxHeight, RED);
    } else if (batteryTemp >= TEMP_WARNING_THRESHOLD) {
      canvas.fillRect(tempBoxX, tempStartY, tempBoxWidth, tempBoxHeight, ORANGE);
    }
  }

  if (escTelemetry.state == TelemetryState::CONNECTED) {
    if (escTemp >= TEMP_CRITICAL_THRESHOLD) {
      canvas.fillRect(tempBoxX, tempStartY + tempBoxHeight, tempBoxWidth, tempBoxHeight, RED);
    } else if (escTemp >= TEMP_WARNING_THRESHOLD) {
      canvas.fillRect(tempBoxX, tempStartY + tempBoxHeight, tempBoxWidth, tempBoxHeight, ORANGE);
    }

    if (motorTemp >= TEMP_CRITICAL_THRESHOLD) {
      canvas.fillRect(tempBoxX, tempStartY + (tempBoxHeight * 2), tempBoxWidth, tempBoxHeight, RED);
    } else if (motorTemp >= TEMP_WARNING_THRESHOLD) {
      canvas.fillRect(tempBoxX, tempStartY + (tempBoxHeight * 2), tempBoxWidth, tempBoxHeight, ORANGE);
    }
  }

  // Now draw all borders and lines
  // Top section borders
  canvas.drawFastHLine(0, 32, 160, currentTheme->ui_accent);  // Bottom border

  // Middle section borders
  canvas.drawFastVLine(90, 32, 43, currentTheme->ui_accent);  // Vertical divider
  canvas.drawFastHLine(0, 75, 160, currentTheme->ui_accent);  // Bottom border moved up

  // Temperature section borders
  canvas.drawFastVLine(120, 75, 53, currentTheme->ui_accent);  // Vertical divider
  canvas.drawFastHLine(tempBoxX, tempStartY + tempBoxHeight, tempBoxWidth, currentTheme->ui_accent);  // Horizontal dividers
  canvas.drawFastHLine(tempBoxX, tempStartY + (tempBoxHeight * 2), tempBoxWidth, currentTheme->ui_accent);

  // Now draw all text and content
  // Top section text
  // Left voltage - only if BMS connected
  if (bmsTelemetry.state == TelemetryState::CONNECTED) {
    if (lowestCellV <= CELL_VOLTAGE_CRITICAL) {
      canvas.setTextColor(RED);
    } else if (lowestCellV <= CELL_VOLTAGE_WARNING) {
      canvas.setTextColor(ORANGE);
    } else {
      canvas.setTextColor(currentTheme->default_text);
    }
    canvas.setFont(Fonts::Regular12);
    canvas.setCursor(2, 12 + FONT_HEIGHT_OFFSET);
    canvas.printf("%2.2fv", lowestCellV);
  }

  // Battery percentage and status
  if (bmsTelemetry.state == TelemetryState::CONNECTED) {
    canvas.setTextColor(BLACK);
    canvas.setFont(Fonts::SemiBold22);
    int xPos = (batteryPercent == 100) ? 50 : 60;
    canvas.setCursor(xPos, 16 + FONT_HEIGHT_OFFSET);
    canvas.printf("%d%%", static_cast<int>(batteryPercent));
  } else {
    canvas.setTextColor(currentTheme->error_text);
    canvas.setCursor(50, 16 + FONT_HEIGHT_OFFSET);
    canvas.print("NO TELEM");
  }

  // Right voltage - only if BMS connected
  if (bmsTelemetry.state == TelemetryState::CONNECTED) {
    canvas.setTextColor(currentTheme->default_text);
    canvas.setFont(Fonts::Regular12);
    canvas.setCursor(128, 12 + FONT_HEIGHT_OFFSET);
    canvas.printf("%2.0fv", totalVolts);
  }

  // Middle section text
  // Power display - only if BMS connected
  if (bmsTelemetry.state == TelemetryState::CONNECTED) {
    canvas.setFont(Fonts::SemiBold24);
    canvas.setTextColor(currentTheme->default_text);
    canvas.setCursor(2, 60);
    float kWatts = bmsTelemetry.power;
    canvas.printf(kWatts < 10 ? "%.1f" : "%.1f", kWatts);

    canvas.setFont(Fonts::Regular14);
    canvas.setCursor(canvas.getCursorX() + 3, 60);
    canvas.print("kW");

    // Power bar
    const int powerBarY = 65;
    const int powerBarHeight = 8;
    const int powerBarWidth = 85;
    const float maxPower = 20.0;
    const int filledWidth = map(static_cast<int>(kWatts * 100), 0, static_cast<int>(maxPower * 100), 0, powerBarWidth);
    if (filledWidth > 0) {
      canvas.fillRect(2, powerBarY, filledWidth, powerBarHeight, GREEN);
    }
  }

  // Performance mode
  canvas.setFont(Fonts::Regular12);
  canvas.setTextColor(currentTheme->default_text);
  canvas.setCursor(95, 46);
  canvas.print(deviceData.performance_mode == 0 ? "CHILL" : "SPORT");

  // Draw Bluetooth symbol
  bool bluetoothConnected = true;  // TODO: make this dynamic
  if (bluetoothConnected) {
    drawBluetoothSymbol(canvas, 145, 38, currentTheme->default_text);
  }

  // Armed time
  canvas.setFont(Fonts::SemiBold20);
  canvas.setTextColor(currentTheme->default_text);
  canvas.setCursor(95, 68);
  const unsigned int nowMillis = millis();
  if (armed) {
    const int sessionSeconds = (nowMillis - armedStartMillis) / 1000;
    canvas.printf("%02d:%02d", sessionSeconds / 60, sessionSeconds % 60);
  } else {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    canvas.printf("%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
  }

  // Bottom section text
  // Altitude
  canvas.setFont(Fonts::SemiBold24);
  canvas.setTextColor(currentTheme->default_text);
  canvas.setCursor(2, 115);
  if (altitude == __FLT_MIN__) {
    canvas.setTextColor(currentTheme->error_text);
    canvas.print("ERR");
  } else {
    if (deviceData.metric_alt) {
      canvas.printf("%.1f", altitude);
      canvas.setFont(Fonts::Regular14);
      canvas.print("m");
    } else {
      canvas.printf("%d", static_cast<int>(round(altitude * 3.28084)));
      canvas.setFont(Fonts::Regular14);
      canvas.print("ft");
    }
  }

  // Temperature values
  canvas.setFont(Fonts::Regular14);
  auto drawTempText = [&](const char* label, float temp, int boxY, TelemetryState state) {
    canvas.setCursor(125, boxY + 13);
    canvas.print(label);
    canvas.setCursor(140, boxY + 13);

    if (state == TelemetryState::CONNECTED) {
      if (temp >= TEMP_CRITICAL_THRESHOLD) {
        canvas.setTextColor(WHITE);
      } else if (temp >= TEMP_WARNING_THRESHOLD) {
        canvas.setTextColor(BLACK);
      } else {
        canvas.setTextColor(currentTheme->default_text);
      }
      canvas.printf("%d", static_cast<int>(temp));
    } else {
      canvas.setTextColor(currentTheme->default_text);
      canvas.print("-");
    }
  };

  drawTempText("B", batteryTemp, tempStartY, bmsTelemetry.state);
  drawTempText("E", escTemp, tempStartY + tempBoxHeight, escTelemetry.state);
  drawTempText("M", motorTemp, tempStartY + (tempBoxHeight * 2), escTelemetry.state);

  // Draw the canvas to the display
  display->drawRGBBitmap(0, 0, canvas.getBuffer(), canvas.width(), canvas.height());
}

// Old display update function
void updateDisplayOld(
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
    canvas.setTextColor(currentTheme->error_text);
  }  // If temperature is 100C+, display in red.
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

  // Get temperatures for coloring
  float batteryTemp = bmsTelemetry.highest_temperature;
  float escTemp = escTelemetry.highest_temp;
  float motorTemp = escTelemetry.motor_temp;

  // Draw temperature backgrounds
  if (batteryTemp >= TEMP_CRITICAL_THRESHOLD) {
    canvas.fillRect(120, 85, 40, 14, RED);
  } else if (batteryTemp >= TEMP_WARNING_THRESHOLD) {
    canvas.fillRect(120, 85, 40, 14, ORANGE);
  }

  if (escTemp >= TEMP_CRITICAL_THRESHOLD) {
    canvas.fillRect(120, 99, 40, 14, RED);
  } else if (escTemp >= TEMP_WARNING_THRESHOLD) {
    canvas.fillRect(120, 99, 40, 14, ORANGE);
  }

  if (motorTemp >= TEMP_CRITICAL_THRESHOLD) {
    canvas.fillRect(120, 113, 40, 14, RED);
  } else if (motorTemp >= TEMP_WARNING_THRESHOLD) {
    canvas.fillRect(120, 113, 40, 14, ORANGE);
  }

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
