#include "sp140/display.h"
#include "../../inc/version.h"
#include "sp140/structs.h"
#include "sp140/bms.h"  // Include for MCP_CS

#define FONT_HEIGHT_OFFSET 8

// This is now defined in sp140.ino as a global
extern SPIClass* hardwareSPI;
extern int8_t displayCS;
extern int8_t bmsCS;

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
  canvas.drawLine(x + 2, y, x + 2, y + 7, color);      // Vertical line
  canvas.drawLine(x + 2, y, x + 4, y + 2, color);      // Upper right
  canvas.drawLine(x + 4, y + 2, x + 2, y + 4, color);  // Middle right upper
  canvas.drawLine(x + 2, y + 4, x + 4, y + 6, color);  // Middle right lower
  canvas.drawLine(x + 4, y + 6, x + 2, y + 8, color);  // Lower right
  canvas.drawLine(x, y + 2, x + 2, y + 4, color);      // Middle left upper
  canvas.drawLine(x, y + 6, x + 2, y + 4, color);      // Middle left lower
}

// Clears screen and resets properties
void resetRotation(unsigned int rotation) {
  display->setRotation(rotation);  // 1=right hand, 3=left hand
}

void setupDisplay(const STR_DEVICE_DATA_140_V1& deviceData, const HardwareConfig& board_config, SPIClass* spi) {
  USBSerial.println("setupDisplay");

  // Create the display with the passed SPI instance
  display = new Adafruit_ST7735(
    spi,
    displayCS,
    board_config.tft_dc,
    board_config.tft_rst);

  display->initR(INITR_BLACKTAB);  // Init ST7735S chip, black tab
  resetRotation(deviceData.screen_rotation);
  setTheme(deviceData.theme);  // 0=light, 1=dark
  display->fillScreen(ST77XX_BLACK);
  displayMeta(deviceData, 1500);
}

// Show splash screen with animated text
void displayMeta(const STR_DEVICE_DATA_140_V1& deviceData, int duration) {
  // Make sure display CS is selected
  digitalWrite(displayCS, LOW);

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

  // Deselect display CS when done
  digitalWrite(displayCS, HIGH);
}

void updateDisplay(
  const STR_DEVICE_DATA_140_V1& deviceData,
  const STR_ESC_TELEMETRY_140& escTelemetry,
  const STR_BMS_TELEMETRY_140& bmsTelemetry,
  const UnifiedBatteryData& unifiedBatteryData,
  float altitude, bool armed, bool cruising,
  unsigned int armedStartMillis
  ) {
  // Ensure BMS is deselected and display is selected
  digitalWrite(bmsCS, HIGH);
  digitalWrite(displayCS, LOW);

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
  if (bmsTelemetry.bmsState == TelemetryState::CONNECTED && batteryPercent > 0) {
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
  if (bmsTelemetry.bmsState == TelemetryState::CONNECTED) {
    if (batteryTemp >= TEMP_CRITICAL_THRESHOLD) {
      canvas.fillRect(tempBoxX, tempStartY, tempBoxWidth, tempBoxHeight, RED);
    } else if (batteryTemp >= TEMP_WARNING_THRESHOLD) {
      canvas.fillRect(tempBoxX, tempStartY, tempBoxWidth, tempBoxHeight, ORANGE);
    }
  }

  if (escTelemetry.escState == TelemetryState::CONNECTED) {
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
  if (bmsTelemetry.bmsState == TelemetryState::CONNECTED) {
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
  if (bmsTelemetry.bmsState == TelemetryState::CONNECTED) {
    canvas.setTextColor(BLACK);
    canvas.setFont(Fonts::SemiBold22);
    int xPos = (batteryPercent == 100) ? 50 : 60;
    canvas.setCursor(xPos, 16 + FONT_HEIGHT_OFFSET);
    canvas.printf("%d%%", static_cast<int>(batteryPercent));
  } else {
    canvas.setTextColor(currentTheme->error_text);
    canvas.setCursor(50, 14 + FONT_HEIGHT_OFFSET);
    canvas.print("NO DATA");
  }

  // Right voltage - only if BMS connected
  if (bmsTelemetry.bmsState == TelemetryState::CONNECTED) {
    canvas.setTextColor(currentTheme->default_text);
    canvas.setFont(Fonts::Regular12);
    canvas.setCursor(128, 12 + FONT_HEIGHT_OFFSET);
    canvas.printf("%2.0fv", totalVolts);
  }

  // Middle section text
  // Power display - only if BMS connected
  if (bmsTelemetry.bmsState == TelemetryState::CONNECTED) {
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
  static unsigned int _lastArmedMillis = 0;
  if (armed) _lastArmedMillis = nowMillis;
  const int sessionSeconds = (_lastArmedMillis - armedStartMillis) / 1000.0;
  canvas.printf("%02d:%02d", sessionSeconds / 60, sessionSeconds % 60);

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

  drawTempText("B", batteryTemp, tempStartY, bmsTelemetry.bmsState);
  drawTempText("E", escTemp, tempStartY + tempBoxHeight, escTelemetry.escState);
  drawTempText("M", motorTemp, tempStartY + (tempBoxHeight * 2), escTelemetry.escState);

  // Draw the canvas to the display
  display->drawRGBBitmap(0, 0, canvas.getBuffer(), canvas.width(), canvas.height());

  // Deselect display CS when done
  digitalWrite(displayCS, HIGH);
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
