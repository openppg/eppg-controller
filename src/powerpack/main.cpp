// PowerPack QC Flash/Test Tool
// Runs on the same ESP32-S3 hand controller hardware as the SP140 flight firmware.
// Purpose: Flash ESC firmware, write production config, validate sensors.

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include "driver/twai.h"
#include <CanardAdapter.h>

#include "powerpack/config.h"
#include "powerpack/esc_flasher.h"
#include "powerpack/esc_config.h"
#include "powerpack/qc_test.h"

// =============================================================================
// Globals
// =============================================================================

// CAN adapter shared by all modules
CanardAdapter canAdapter;
static uint8_t canMemoryPool[2048] __attribute__((aligned(8)));

// NeoPixel LED
static Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Display
static Adafruit_ST7735 tft(DISPLAY_CS_PIN, DISPLAY_DC_PIN, DISPLAY_RST_PIN);

// Tool state
enum class ToolState : uint8_t {
    WAITING,          // Waiting for button press
    FLASHING,         // Firmware update in progress
    CONFIGURING,      // Writing config params
    TESTING,          // Running QC sensor validation
    RESULT_PASS,      // All tests passed
    RESULT_FAIL       // One or more tests failed
};

static ToolState toolState = ToolState::WAITING;
static bool buttonPressed = false;

// =============================================================================
// Placeholder ESC firmware binary
// In production, embed the actual .bin file here using:
//   xxd -i firmware.bin > fw_data.h
// Or use PROGMEM/LittleFS. For now, this is a placeholder.
// =============================================================================

// ESC firmware binary - SE24250D V2.33.102
#include "fw_data.h"
static const uint8_t* ESC_FW_DATA = ESC_FW_EMBEDDED_DATA;
static const uint32_t ESC_FW_SIZE = ESC_FW_EMBEDDED_SIZE;

// =============================================================================
// LED helpers
// =============================================================================

static void setLED(uint8_t r, uint8_t g, uint8_t b) {
    pixel.setPixelColor(0, pixel.Color(r, g, b));
    pixel.show();
}

static void ledOff()    { setLED(0, 0, 0); }
static void ledBlue()   { setLED(0, 0, 80); }
static void ledGreen()  { setLED(0, 80, 0); }
static void ledRed()    { setLED(80, 0, 0); }
static void ledYellow() { setLED(80, 80, 0); }

// =============================================================================
// Display helpers
// =============================================================================

static void displayInit() {
    SPI.begin(SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
    tft.initR(INITR_BLACKTAB);            // 160x128 ST7735 (same as SP140)
    tft.setRotation(3);                   // Landscape, same as SP140
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextWrap(false);               // Prevent wrapping - we manage layout
}

static void displayClear() {
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0, 2);
}

// Title fits on one line: size 1 = 26 chars, size 2 = 13 chars max at 160px
// We use a custom size check to pick the largest that fits.
static void displayTitle(const char* title, uint16_t color = ST77XX_WHITE) {
    displayClear();
    uint8_t len = strlen(title);
    // Size 2 = 12px wide per char, fits 13 chars on 160px
    // Size 1 = 6px wide per char, fits 26 chars on 160px
    uint8_t sz = (len <= 13) ? 2 : 1;
    tft.setTextSize(sz);
    tft.setTextColor(color);
    tft.println(title);
    // Add spacing after title
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(0, tft.getCursorY() + 4);  // 4px gap below title
}

static void displayLine(const char* text, uint16_t color = ST77XX_WHITE) {
    tft.setTextColor(color);
    tft.println(text);
}

static void displayProgress(const char* label, uint8_t percent) {
    // Progress bar: 12px tall, 4px gap above for the label text
    // Layout from bottom: [bar 12px] [gap 4px] [label 8px] [gap 4px]
    int barH = 12;
    int barY = tft.height() - barH - 2;     // 2px bottom margin
    int barW = tft.width() - 4;
    int fillW = (int)barW * percent / 100;

    // Clear bar + label area
    int labelY = barY - 14;                  // 8px text + 6px gap
    tft.fillRect(0, labelY, tft.width(), tft.height() - labelY, ST77XX_BLACK);

    // Label line
    tft.setCursor(2, labelY);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);
    char buf[40];
    snprintf(buf, sizeof(buf), "%s  %d%%", label, percent);
    tft.print(buf);

    // Progress bar
    tft.drawRect(2, barY, barW, barH, ST77XX_WHITE);
    if (fillW > 0) {
        tft.fillRect(3, barY + 1, fillW - 1, barH - 2, ST77XX_CYAN);
    }
}

static void displayResult(const char* label, const char* value, QcResult result) {
    uint16_t color;
    switch (result) {
        case QcResult::PASS: color = ST77XX_GREEN; break;
        case QcResult::FAIL: color = ST77XX_RED; break;
        case QcResult::WARN: color = ST77XX_YELLOW; break;
        default: color = ST77XX_WHITE; break;
    }
    tft.setTextColor(color);
    char buf[40];
    snprintf(buf, sizeof(buf), "%-12s %s", label, value);
    tft.println(buf);
}

// =============================================================================
// CAN bus setup (same as sp140/esc.cpp:setupTWAI)
// =============================================================================

static bool setupTWAI() {
    twai_status_info_t status_info;
    esp_err_t status_result = twai_get_status_info(&status_info);

    if (status_result == ESP_OK) {
        USBSerial.printf("TWAI already installed, state=%d\n", status_info.state);
        return true;
    } else if (status_result != ESP_ERR_INVALID_STATE) {
        USBSerial.printf("TWAI status error: %s\n", esp_err_to_name(status_result));
        return false;
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)ESC_TX_PIN, (gpio_num_t)ESC_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        USBSerial.println("TWAI install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        USBSerial.println("TWAI start failed");
        twai_driver_uninstall();
        return false;
    }

    uint32_t alerts = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS |
                      TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
    twai_reconfigure_alerts(alerts, NULL);

    USBSerial.println("TWAI initialized OK");
    return true;
}

// =============================================================================
// Button handling
// =============================================================================

static void IRAM_ATTR buttonISR() {
    buttonPressed = true;
}

// Returns 0=no press, 1=short press (<1s), 2=long press (>=2s)
static uint8_t checkButtonPress() {
    if (!buttonPressed) return 0;
    buttonPressed = false;

    // Debounce
    vTaskDelay(pdMS_TO_TICKS(50));
    if (digitalRead(BUTTON_PIN) == HIGH) return 1;  // Already released = short tap

    // Measure hold duration
    unsigned long pressStart = millis();
    while (digitalRead(BUTTON_PIN) == LOW) {
        vTaskDelay(pdMS_TO_TICKS(20));
        if (millis() - pressStart > 2000) break;
    }

    unsigned long held = millis() - pressStart;
    return (held >= 2000) ? 2 : 1;
}

// =============================================================================
// Serial command interface
// =============================================================================

static void processSerialCommands() {
    if (!USBSerial.available()) return;

    String cmd = USBSerial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "start" || cmd == "flash") {
        if (toolState == ToolState::WAITING ||
            toolState == ToolState::RESULT_PASS ||
            toolState == ToolState::RESULT_FAIL) {
            toolState = ToolState::FLASHING;
        }
    } else if (cmd == "config") {
        if (toolState == ToolState::WAITING) {
            toolState = ToolState::CONFIGURING;
        }
    } else if (cmd == "test") {
        if (toolState == ToolState::WAITING ||
            toolState == ToolState::RESULT_PASS ||
            toolState == ToolState::RESULT_FAIL) {
            toolState = ToolState::TESTING;
        }
    } else if (cmd == "status") {
        USBSerial.printf("State: %d\n", (int)toolState);
    } else if (cmd == "help") {
        USBSerial.println("Commands: start, config, test, status, help");
    }
}

// =============================================================================
// Main QC flow task
// =============================================================================

static void qcFlowTask(void* pvParameters) {
    for (;;) {
        switch (toolState) {

        case ToolState::WAITING:
            ledBlue();
            displayTitle("PowerPack QC");
            displayLine("v1.0  FW:V2.33.102");
            displayLine("");
            displayLine("SHORT press: Config+Test", ST77XX_CYAN);
            displayLine("LONG  press: Flash+All", ST77XX_YELLOW);
            displayLine("");
            displayLine("Serial: start/config/test");
            // Wait for button in this state
            while (toolState == ToolState::WAITING) {
                uint8_t press = checkButtonPress();
                if (press == 1) {
                    // Short press = config + test (skip firmware flash)
                    USBSerial.println("\n=== Config + Test (no flash) ===");
                    toolState = ToolState::CONFIGURING;
                } else if (press == 2) {
                    // Long press = full sequence including firmware flash
                    USBSerial.println("\n=== Full Flash + Config + Test ===");
                    if (ESC_FW_SIZE > 32) {
                        toolState = ToolState::FLASHING;
                    } else {
                        USBSerial.println("No FW embedded, skipping flash");
                        toolState = ToolState::CONFIGURING;
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            break;

        case ToolState::FLASHING:
            ledYellow();
            displayTitle("Flashing ESC", ST77XX_YELLOW);
            displayLine("HW:0x0150 90KB");
            USBSerial.println("--- Phase 1: Firmware Update ---");
            escFlasherStart(ESC_FW_DATA, ESC_FW_SIZE);
            while (escFlasherTick()) {
                const FlashProgress& p = escFlasherGetProgress();
                displayProgress(p.statusMsg, p.percentComplete);
                if (p.state == FlashState::SENDING_DATA && (p.currentChunk % 20 == 0)) {
                    USBSerial.printf("Flashing: %u%% (%u/%u)\n",
                        p.percentComplete, p.currentChunk, p.totalChunks);
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            if (escFlasherSucceeded()) {
                USBSerial.println("Firmware update SUCCESS");
                displayProgress("FW Done!", 100);
                vTaskDelay(pdMS_TO_TICKS(3000));
                toolState = ToolState::CONFIGURING;
            } else {
                USBSerial.printf("Firmware update FAILED: %s\n",
                    escFlasherGetProgress().statusMsg);
                displayTitle("FLASH FAIL", ST77XX_RED);
                displayLine(escFlasherGetProgress().statusMsg);
                toolState = ToolState::RESULT_FAIL;
            }
            break;

        case ToolState::CONFIGURING:
            ledYellow();
            displayTitle("Writing Config", ST77XX_YELLOW);
            USBSerial.println("--- Phase 2: Configuration ---");
            escConfigStart();
            while (escConfigTick()) {
                const ConfigProgress& p = escConfigGetProgress();
                displayProgress(p.currentParamName, p.percentComplete);
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            if (escConfigSucceeded()) {
                USBSerial.println("Config write SUCCESS");
                displayProgress("Config Done!", 100);
                vTaskDelay(pdMS_TO_TICKS(3000));
                toolState = ToolState::TESTING;
            } else {
                USBSerial.printf("Config write FAILED: %s\n",
                    escConfigGetProgress().statusMsg);
                displayTitle("CONFIG FAIL", ST77XX_RED);
                displayLine(escConfigGetProgress().statusMsg);
                toolState = ToolState::RESULT_FAIL;
            }
            break;

        case ToolState::TESTING:
            ledYellow();
            displayTitle("QC Test", ST77XX_YELLOW);
            USBSerial.println("--- Phase 3: Full QC Test ---");
            qcTestStart();
            while (qcTestTick()) {
                displayProgress(qcTestStatusLine(), qcTestProgress());
                vTaskDelay(pdMS_TO_TICKS(30));
            }

            {
                const QcTestResults& r = qcTestGetResults();
                char buf[24];

                if (r.overall == QcResult::PASS) {
                    displayTitle("PASS", ST77XX_GREEN);
                    toolState = ToolState::RESULT_PASS;
                } else if (r.overall == QcResult::WARN) {
                    displayTitle("WARN", ST77XX_YELLOW);
                    toolState = ToolState::RESULT_PASS;  // WARN still counts as pass
                } else {
                    displayTitle("FAIL", ST77XX_RED);
                    toolState = ToolState::RESULT_FAIL;
                }

                // Page 1: Key results (fits ~10 lines on 128px at size 1)
                snprintf(buf, sizeof(buf), "%ldRPM", (long)r.peakRpm);
                displayResult("Peak RPM", buf, r.reachesTargetRpm.result);

                snprintf(buf, sizeof(buf), "%.1fA", r.peakPhaseA);
                displayResult("Peak I", buf, r.peakPhaseCurrent.result);

                snprintf(buf, sizeof(buf), "%.1fV", r.voltageSag.value);
                displayResult("V Sag", buf, r.voltageSag.result);

                snprintf(buf, sizeof(buf), "%.1fC", r.mosTempIdle.value);
                displayResult("MOS T", buf, r.mosTempIdle.result);

                snprintf(buf, sizeof(buf), "%.1fC", r.motorTempIdle.value);
                displayResult("Motor T", buf, r.motorTempIdle.result);

                snprintf(buf, sizeof(buf), "%.1fV", r.busVoltageIdle.value);
                displayResult("Bus V", buf, r.busVoltageIdle.result);

                snprintf(buf, sizeof(buf), "%s", r.snapIdle.selfChkErr == 0 ? "None" : "ERROR");
                displayResult("SelfChk", buf, r.selfCheck.result);

                // Decode spin errors into a human-readable string
                {
                    uint16_t raw = (uint16_t)r.noSpinErrors.value;
                    uint16_t benignMask = 0x0138;  // bits 3,4,5,8
                    uint16_t critical = raw & ~benignMask;
                    if (raw == 0) {
                        snprintf(buf, sizeof(buf), "None");
                    } else if (critical == 0) {
                        // Only benign errors - build short label
                        char *p = buf;
                        *p = '\0';
                        if (raw & 0x0008) p += snprintf(p, sizeof(buf)-(p-buf), "PWM ");
                        if (raw & 0x0010) p += snprintf(p, sizeof(buf)-(p-buf), "NoLd ");
                        if (raw & 0x0020) p += snprintf(p, sizeof(buf)-(p-buf), "Sat ");
                        if (raw & 0x0100) p += snprintf(p, sizeof(buf)-(p-buf), "Comm");
                        // Trim trailing space
                        size_t len = strlen(buf);
                        if (len > 0 && buf[len-1] == ' ') buf[len-1] = '\0';
                    } else {
                        snprintf(buf, sizeof(buf), "0x%04X!", critical);
                    }
                }
                displayResult("SpinErr", buf, r.noSpinErrors.result);
            }
            break;

        case ToolState::RESULT_PASS:
            ledGreen();
            buttonPressed = false;  // Clear any presses from during the test
            while (toolState == ToolState::RESULT_PASS) {
                if (checkButtonPress() > 0) {
                    toolState = ToolState::WAITING;
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            break;

        case ToolState::RESULT_FAIL:
            ledRed();
            buttonPressed = false;  // Clear any presses from during the test
            while (toolState == ToolState::RESULT_FAIL) {
                if (checkButtonPress() > 0) {
                    toolState = ToolState::WAITING;
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// =============================================================================
// Arduino entry points
// =============================================================================

void setup() {
    USBSerial.begin(115200);
    delay(1000);

    USBSerial.println();
    USBSerial.println("================================");
    USBSerial.println("  OpenPPG PowerPack QC Tool v1.0");
    USBSerial.println("================================");
    #ifdef GIT_REV
    USBSerial.printf("Git: %s\n", GIT_REV);
    #endif

    // Display
    displayInit();
    displayTitle("PowerPack QC");
    displayLine("Initializing...");

    // NeoPixel
    pixel.begin();
    pixel.setBrightness(50);
    ledBlue();

    // Button
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

    // CAN bus
    if (!setupTWAI()) {
        USBSerial.println("FATAL: CAN bus init failed!");
        ledRed();
        while (1) { delay(1000); }
    }

    // Initialize canard adapter
    canAdapter.begin(canMemoryPool, sizeof(canMemoryPool));
    canAdapter.setLocalNodeId(LOCAL_NODE_ID);

    // Initialize all modules
    escFlasherInit();
    escConfigInit();
    qcTestInit();

    USBSerial.println("Ready. Press button or type 'start' to begin.");
    USBSerial.println("Commands: start, config, test, status, help");

    // Create main QC flow task
    xTaskCreatePinnedToCore(qcFlowTask, "QCFlow", 8192, NULL, 2, NULL, 0);
}

void loop() {
    processSerialCommands();
    delay(50);
}
