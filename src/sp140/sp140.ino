// Copyright 2020 <Zach Whitehead>
// OpenPPG
#include "Arduino.h"

#include "../../inc/sp140/esp32s3-config.h"

#include "../../inc/sp140/structs.h"  // data structs
#include "../../inc/sp140/utilities.h"
#include <Adafruit_NeoPixel.h>        // RGB LED
#include <ArduinoJson.h>
#include <CircularBuffer.hpp>      // smooth out readings
#include <ResponsiveAnalogRead.h>  // smoothing for throttle
#include <SPI.h>
#include <TimeLib.h>  // convert time to hours mins etc
#include <Wire.h>

// ESP32S3 (CAN) specific libraries here
#include "esp_task_wdt.h"

#ifdef WIFI_DEBUG
  #include "Insights.h"
  #include "WiFi.h"
  #include "inttypes.h"
  #include "esp_err.h"
#endif

#include "../../inc/sp140/globals.h"  // device config
#include "../../inc/sp140/esc.h"
#include "../../inc/sp140/lvgl/lvgl_display.h"
#include "../../inc/sp140/bms.h"
#include "../../inc/sp140/altimeter.h"
#include "../../inc/sp140/debug.h"
#include "../../inc/sp140/simple_monitor.h"
#include "../../inc/sp140/alert_display.h"

#include "../../inc/sp140/buzzer.h"
#include "../../inc/sp140/device_state.h"
#include "../../inc/sp140/mode.h"
#include "../../inc/sp140/throttle.h"
#include "../../inc/sp140/vibration_pwm.h"
#include "../../inc/sp140/led.h"

// Global variable for shared SPI
SPIClass* hardwareSPI = nullptr;

// Store CS pins as globals to avoid accessing protected members
int8_t displayCS = -1;
int8_t bmsCS = MCP_CS;

#define BUTTON_DEBOUNCE_TIME_MS 50
#define FIRST_CLICK_MAX_HOLD_MS 500    // Maximum time for first click to be considered a click
#define SECOND_HOLD_TIME_MS 2000       // How long to hold on second press to arm
#define CRUISE_HOLD_TIME_MS 2000
#define BUTTON_SEQUENCE_TIMEOUT_MS 1500  // Time window for arm/disarm sequence
#define PERFORMANCE_MODE_HOLD_MS 3000   // Longer hold time for performance mode

// Throttle control constants
#define CHILL_MODE_MAX_PWM 1850   // 85% max power in chill mode
#define CHILL_MODE_RAMP_RATE 50   // How fast throttle can increase per cycle in chill mode
#define SPORT_MODE_RAMP_RATE 120  // How fast throttle can increase per cycle in sport mode

#define DECEL_MULTIPLIER 2.0     // How much faster deceleration is vs acceleration
#define CRUISE_MAX_PERCENTAGE 0.60  // Maximum cruise throttle as a percentage of the total ESC range (e.g., 0.60 = 60%)
#define CRUISE_DISENGAGE_POT_THRESHOLD_PERCENTAGE 0.80  // Current pot must be >= this % of activation value to disengage
#define CRUISE_DISENGAGE_GRACE_PERIOD_MS 2000  // Delay before checking pot disengagement after cruise activation
#define CRUISE_ACTIVATION_MAX_POT_PERCENTAGE 0.70  // Prevent cruise activation if pot is above this percentage

// Button state tracking
volatile bool buttonPressed = false;
volatile uint32_t buttonPressStartTime = 0;
volatile uint32_t buttonReleaseStartTime = 0;
volatile bool armSequenceStarted = false;
TaskHandle_t buttonTaskHandle = NULL;

UBaseType_t uxCoreAffinityMask0 = (1 << 0);  // Core 0
UBaseType_t uxCoreAffinityMask1 = (1 << 1);  // Core 1

HardwareConfig board_config;
bool bmsCanInitialized = false;
bool escTwaiInitialized = false;

ResponsiveAnalogRead* pot;

UnifiedBatteryData unifiedBatteryData = {0.0f, 0.0f, 0.0f};

CircularBuffer<int, 8> potBuffer;

Adafruit_NeoPixel pixels(1, 21, NEO_GRB + NEO_KHZ800);
uint32_t led_color = LED_RED;  // current LED color

// Global variable for device state
volatile DeviceState currentState = DISARMED;

// Add volatile bool flag to control UI drawing start
volatile bool uiReady = false;

SemaphoreHandle_t eepromSemaphore;
SemaphoreHandle_t stateMutex;

// Add near other queue declarations
QueueHandle_t bleStateQueue = NULL;
QueueHandle_t deviceStateQueue = NULL;

// Add struct for BLE state updates
struct BLEStateUpdate {
  uint8_t state;
  bool needsNotify;
};

// Add new task handle
TaskHandle_t bleStateUpdateTaskHandle = NULL;
TaskHandle_t deviceStateUpdateTaskHandle = NULL;

// Add near other task handles
TaskHandle_t webSerialTaskHandle = NULL;

// Add mutex for LVGL thread safety
SemaphoreHandle_t lvglMutex;

// Variable to track the current screen page
ScreenPage currentScreenPage = MAIN_SCREEN;

// Function to switch screen pages safely
void switchScreenPage(ScreenPage newPage) {
  if (xSemaphoreTake(lvglMutex, portMAX_DELAY) == pdTRUE) {
    currentScreenPage = newPage;
    // Potentially add code here to initialize the new screen if needed
    xSemaphoreGive(lvglMutex);
  } else {
    USBSerial.println("Failed to acquire LVGL mutex for screen switch");
  }
}

// Add this function to handle BLE updates safely
void bleStateUpdateTask(void* parameter) {
  BLEStateUpdate update;

  while (true) {
    if (xQueueReceive(bleStateQueue, &update, portMAX_DELAY) == pdTRUE) {
      if (pDeviceStateCharacteristic != nullptr) {
        // Add delay to give BLE stack breathing room
        vTaskDelay(pdMS_TO_TICKS(20));

        // Set value first
        pDeviceStateCharacteristic->setValue(&update.state, sizeof(update.state));

        // Only notify if requested and connected
        if (update.needsNotify && deviceConnected) {
          vTaskDelay(pdMS_TO_TICKS(10));  // Additional delay before notify
          pDeviceStateCharacteristic->notify();
        }
      }
    }
    // Small delay between updates
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// Add new task to handle device state updates (similar to updateBLETask)
void deviceStateUpdateTask(void* parameter) {
  uint8_t state;

  while (true) {
    if (xQueueReceive(deviceStateQueue, &state, portMAX_DELAY) == pdTRUE) {
      if (pDeviceStateCharacteristic != nullptr && deviceConnected) {
        vTaskDelay(pdMS_TO_TICKS(20));  // Give BLE stack breathing room
        pDeviceStateCharacteristic->setValue(&state, sizeof(state));
        // Temporarily remove the notify call
        // pDeviceStateCharacteristic->notify();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void changeDeviceState(DeviceState newState) {
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    DeviceState oldState = currentState;
    currentState = newState;

    // Send state update to queue
    uint8_t state = (uint8_t)newState;
    if (deviceStateQueue != NULL) {
      xQueueOverwrite(deviceStateQueue, &state);  // Always use latest state
    }

    USBSerial.print("Device State Changed to: ");
    switch(newState) {
      case DISARMED:
        USBSerial.println("DISARMED");
        break;
      case ARMED:
        USBSerial.println("ARMED");
        break;
      case ARMED_CRUISING:
        USBSerial.println("ARMED_CRUISING");
        break;
    }

    // Handle state transition actions
    switch (newState) {
      case DISARMED:
        disarmSystem();
        break;
      case ARMED:
        if (oldState == DISARMED) {
          armSystem();
        } else if (oldState == ARMED_CRUISING) {
          afterCruiseEnd();
        }
        break;
      case ARMED_CRUISING:
        if (oldState == ARMED) {
          afterCruiseStart();
        }
        break;
    }
    xSemaphoreGive(stateMutex);
  }
}

uint32_t armedAtMillis = 0;
uint32_t cruisedAtMillisMilis = 0;
unsigned long armedSecs = 0;

TaskHandle_t blinkLEDTaskHandle = NULL;
TaskHandle_t throttleTaskHandle = NULL;
TaskHandle_t watchdogTaskHandle = NULL;
TaskHandle_t spiCommTaskHandle = NULL;
TaskHandle_t vibeTaskHandle = NULL;  // Vibration motor task
TaskHandle_t monitoringTaskHandle = NULL;  // Sensor monitoring task

QueueHandle_t melodyQueue = NULL;
TaskHandle_t audioTaskHandle = NULL;

QueueHandle_t bmsTelemetryQueue = NULL;
QueueHandle_t throttleUpdateQueue = NULL;
QueueHandle_t escTelemetryQueue = NULL;
QueueHandle_t vibeQueue = NULL;  // Vibration motor queue

// Snapshot queue for sensor monitoring
QueueHandle_t telemetrySnapshotQueue = NULL;

unsigned long lastDisarmTime = 0;
const unsigned long DISARM_COOLDOWN = 500;  // 500ms cooldown

#pragma message "Warning: OpenPPG software is in beta"

// Initialize shared SPI and chip select pins
void setupSPI(const HardwareConfig& board_config) {
  USBSerial.println("Setting up SPI bus");

  // Store CS pins
  displayCS = board_config.tft_cs;
  bmsCS = MCP_CS;

  // Configure both CS pins as outputs and set HIGH (deselected)
  pinMode(displayCS, OUTPUT);
  digitalWrite(displayCS, HIGH);

  pinMode(bmsCS, OUTPUT);
  digitalWrite(bmsCS, HIGH);

  // Initialize hardware SPI for use by both display and BMS
  hardwareSPI = new SPIClass(HSPI);
  // Use the board_config pins instead of the defines
  hardwareSPI->begin(board_config.spi_sclk, board_config.spi_miso, board_config.spi_mosi, -1);
}

void watchdogTask(void* parameter) {
  for (;;) {
    #ifndef OPENPPG_DEBUG
      ESP_ERROR_CHECK(esp_task_wdt_reset());
    #endif
    vTaskDelay(pdMS_TO_TICKS(100));  // Delay for 100ms
  }
}

void blinkLEDTask(void *pvParameters) {
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {  // infinite loop
    blinkLED();  // call blinkLED function
    delay(500);  // wait for 500ms
  }
  vTaskDelete(NULL);  // should never reach this
}

void throttleTask(void *pvParameters) {
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {  // infinite loop
    handleThrottle();  //
    pushTelemetrySnapshot();
    delay(20);  // wait for 20ms
  }
  vTaskDelete(NULL);  // should never reach this
}

void updateBLETask(void *pvParameters) {
  STR_BMS_TELEMETRY_140 newBmsTelemetry;

  while (true) {
    // Add error checking for queue
    if (bmsTelemetryQueue == NULL) {
      USBSerial.println("BMS Queue not initialized!");
      vTaskDelay(pdMS_TO_TICKS(1000));  // Wait a second before retrying
      continue;
    }

    // Wait for new data with timeout
    if (xQueueReceive(bmsTelemetryQueue, &newBmsTelemetry, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Update BLE characteristics with the received data
      updateBMSTelemetry(newBmsTelemetry);
    }

    // Add a small delay to prevent task starvation
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void refreshDisplay() {
  // Prevent drawing until setup() signals UI is ready
  if (!uiReady || main_screen == NULL) {
    return;
  }

  if (xSemaphoreTake(lvglMutex, portMAX_DELAY) == pdTRUE) {
    // Get the current relative altitude (updates buffer for vario)
    const float currentRelativeAltitude = getAltitude(deviceData);

    // Determine the altitude to show on the display
    float altitudeToShow = 0.0f;  // Default to 0
    if (currentState != DISARMED) {
      altitudeToShow = currentRelativeAltitude;  // Show relative altitude when armed/cruising
    }
    bool isArmed = (currentState != DISARMED);
    bool isCruising = (currentState == ARMED_CRUISING);

    // Select the appropriate screen update function based on the current page
    switch (currentScreenPage) {
      case MAIN_SCREEN:
        updateLvglMainScreen(
          deviceData,
          escTelemetryData,
          bmsTelemetryData,
          unifiedBatteryData,
          altitudeToShow,
          isArmed,
          isCruising,
          armedAtMillis);
        break;
      // Add cases for other screens here later
      // case SETTINGS_SCREEN:
      //   updateLvglSettingsScreen(...);
      //   break;
      default:
        // Handle unknown screen state if necessary
        break;
    }

    // Handle alert counter updates, non-blocking
    AlertCounts newCounts;
    if (xQueueReceive(alertCountQueue, &newCounts, 0) == pdTRUE) {
      updateAlertCounterDisplay(newCounts);
    }

    // General LVGL update handler
    updateLvgl();

    xSemaphoreGive(lvglMutex);
  } else {
    USBSerial.println("Failed to acquire LVGL mutex in refreshDisplay");
  }
}

void monitoringTask(void *pvParameters) {
  TelemetrySnapshot snap;
  for (;;) {
    if (xQueueReceive(telemetrySnapshotQueue, &snap, pdMS_TO_TICKS(100)) == pdTRUE) {
      // Run monitors using the fresh snapshot
      if (monitoringEnabled) {
        checkAllSensorsWithData(snap.esc, snap.bms);
      }
    }
  }
}

void spiCommTask(void *pvParameters) {
  for (;;) {
      #ifdef SCREEN_DEBUG
        float altitude = 0;
        generateFakeTelemetry(escTelemetryData, bmsTelemetryData, unifiedBatteryData, altitude);
        xQueueOverwrite(bmsTelemetryQueue, &bmsTelemetryData);  // Always latest data
        xQueueOverwrite(escTelemetryQueue, &escTelemetryData);  // Always latest data
        refreshDisplay();
        pushTelemetrySnapshot();
      #else
        // 1. Check if CAN transceiver is initialized
        if (bmsCanInitialized) {
          updateBMSData();  // This function handles its own CS pins
          if (bms_can->isConnected()) {
            bmsTelemetryData.bmsState = TelemetryState::CONNECTED;
          } else {
            bmsTelemetryData.bmsState = TelemetryState::NOT_CONNECTED;
          }
        }

        // 2. Use BMS data if connected, otherwise use ESC data
        if (bmsTelemetryData.bmsState == TelemetryState::CONNECTED) {
          // BMS is initialized AND currently connected
          unifiedBatteryData.volts = bmsTelemetryData.battery_voltage;
          unifiedBatteryData.amps = bmsTelemetryData.battery_current;
          unifiedBatteryData.soc = bmsTelemetryData.soc;
          unifiedBatteryData.power = bmsTelemetryData.power;
                    xQueueOverwrite(bmsTelemetryQueue, &bmsTelemetryData);
        } else if (escTelemetryData.escState == TelemetryState::CONNECTED) {
          // BMS is either not initialized OR not currently connected

          // Fallback to ESC data for unified view
          unifiedBatteryData.volts = escTelemetryData.volts;
          unifiedBatteryData.amps = escTelemetryData.amps;
          unifiedBatteryData.power = escTelemetryData.amps * escTelemetryData.volts / 1000.0;

          unifiedBatteryData.soc = 0.0;  // We don't estimate SOC from voltage anymore
        } else {  // Not connected to either BMS or ESC, probably USB powered
          // No connection to either BMS or ESC
          unifiedBatteryData.volts = 0.0;
          unifiedBatteryData.amps = 0.0;
          unifiedBatteryData.soc = 0.0;
        }

                // Update display - CS pin management is handled inside refreshDisplay via LVGL mutex
        refreshDisplay();

        // Publish latest telemetry snapshot for monitoring
        pushTelemetrySnapshot();

      #endif

      vTaskDelay(pdMS_TO_TICKS(40));  // ~25fps
  }
}

void loadHardwareConfig() {
  board_config = s3_config;  // ESP32S3 is only supported board

  pot = new ResponsiveAnalogRead(board_config.throttle_pin, false);
}

void printBootMessage() {
  USBSerial.print(F("Booting up V"));
  USBSerial.print(VERSION_STRING);
  USBSerial.print(F(" git:"));
  USBSerial.println(GIT_REV);
}

void setupBarometer() {
  const int bmp_enabler = 9;
  pinMode(bmp_enabler, OUTPUT);
  digitalWrite(bmp_enabler, HIGH);  // barometer fix for V2 board
}

void setupLED() {
  pinMode(board_config.led_sw, OUTPUT);   // set up the internal LED2 pin
  if (board_config.enable_neopixel) {
    pixels.begin();
    setLEDColor(led_color);
  }
}

void setupAnalogRead() {
  //analogReadResolution(12);   // M0 family chips provides 12bit ADC resolution
  pot->setAnalogResolution(4096);
}

void setupWatchdog() {
#ifndef OPENPPG_DEBUG
  // Initialize Task Watchdog
  //ESP_ERROR_CHECK(esp_task_wdt_init(3000, true));  // 3 second timeout, panic on timeout
#endif // OPENPPG_DEBUG
}

#define TAG "OpenPPG"


/**
 * Initializes the necessary components and configurations for the device setup.
 * This function is called once at the beginning of the program execution.
 */

void setup() {
  USBSerial.begin(115200);  // This is for debug output and WebSerial

  // Pull CSB (pin 42) high to activate I2C mode
  // temporary fix TODO remove
  digitalWrite(42, HIGH);
  pinMode(42, OUTPUT);

  // Initialize LVGL mutex before anything else
  lvglMutex = xSemaphoreCreateMutex();
  if (lvglMutex == NULL) {
    USBSerial.println("Error creating LVGL mutex");
  }

  refreshDeviceData();
  printBootMessage();
  setupBarometer();

  loadHardwareConfig();
  setupLED();  // Defaults to RED
  setupAnalogRead();
  initButtons();
  setupWatchdog();
  setup140();

#ifdef WIFI_DEBUG
  setupWiFi();
#endif
  setLEDColor(LED_YELLOW);  // Booting up

  // First initialize the shared SPI bus
  setupSPI(board_config);

  // Then setup the display - use LVGL display instead of the old one
  // setupDisplay(deviceData, board_config, hardwareSPI);
  // Pass hardcoded pin values for DC and RST
  setupLvglDisplay(deviceData, board_config.tft_dc, board_config.tft_rst, hardwareSPI);

  // Initialise alert display aggregation & UI
  initAlertDisplay();

  #ifndef SCREEN_DEBUG
    // Pass the hardware SPI instance to the BMS_CAN initialization
    initBMSCAN(hardwareSPI);
  #endif

  initESC();
  eepromSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(eepromSemaphore);
  stateMutex = xSemaphoreCreateMutex();
  setESCThrottle(ESC_DISARMED_PWM);
  initVibeMotor();

  // Create all queues first
  bmsTelemetryQueue = xQueueCreate(1, sizeof(STR_BMS_TELEMETRY_140));
  if (bmsTelemetryQueue == NULL) {
    USBSerial.println("Error creating BMS telemetry queue");
  }

  throttleUpdateQueue = xQueueCreate(1, sizeof(uint16_t));
  if (throttleUpdateQueue == NULL) {
    USBSerial.println("Error creating throttle update queue");
  }

  escTelemetryQueue = xQueueCreate(1, sizeof(STR_ESC_TELEMETRY_140));
  if (escTelemetryQueue == NULL) {
    USBSerial.println("Error creating ESC telemetry queue");
  }

  bleStateQueue = xQueueCreate(5, sizeof(BLEStateUpdate));
  if (bleStateQueue == NULL) {
    USBSerial.println("Error creating BLE state queue");
  }

  deviceStateQueue = xQueueCreate(1, sizeof(uint8_t));
  if (deviceStateQueue == NULL) {
    USBSerial.println("Error creating device state queue");
  }

  // Snapshot queue for monitoring (size 1)
  telemetrySnapshotQueue = xQueueCreate(1, sizeof(TelemetrySnapshot));
  if (telemetrySnapshotQueue == NULL) {
    USBSerial.println("Error creating telemetry snapshot queue");
  }

    setupBLE();

  // Initialize the simple monitoring system (but keep it disabled initially)
  initSimpleMonitor();

  setupTasks();  // Create all tasks after queues and BLE are initialized

  pulseVibeMotor();
  if (digitalRead(board_config.button_top) == LOW) {  // LOW means pressed since it's INPUT_PULLUP
    perfModeSwitch();
  }
  setLEDColor(LED_GREEN);

  // Show LVGL splash screen
  if (xSemaphoreTake(lvglMutex, portMAX_DELAY) == pdTRUE) {
    displayLvglSplash(deviceData, 2000);
    // Don't release mutex here yet
  } else {
    USBSerial.println("Failed to acquire LVGL mutex for splash");
    // If we failed mutex, maybe skip main screen setup? Or attempt anyway?
    // Let's assume splash failed but we try to proceed.
  }

  // Setup the main screen UI elements AFTER the splash
  USBSerial.println("Setting up main screen after splash");
  if (main_screen == NULL) {  // Check if it wasn't somehow created already
    setupMainScreen(deviceData.theme == 1);
  }

  // Explicitly load the main screen to make it active
  if (main_screen != NULL) {
      lv_scr_load(main_screen);
      USBSerial.println("Main screen loaded");
  } else {
     USBSerial.println("Error: Main screen object is NULL after setup attempt.");
  }

  // Release the mutex if it was taken for the splash
  if (lvglMutex != NULL && xSemaphoreGetMutexHolder(lvglMutex) == xTaskGetCurrentTaskHandle()) {
      xSemaphoreGive(lvglMutex);
  }

  // Send initial device data after setup
  send_device_data();
  // Signal that the UI is ready for updates from tasks
  uiReady = true;

  // Enable sensor monitoring after splash screen and UI setup
  // This gives BMS/ESC time to initialize and start providing valid data
  enableMonitoring();
}

// set up all the main threads/tasks with core 0 affinity
void setupTasks() {
  xTaskCreate(blinkLEDTask, "blinkLed", 1536, NULL, 1, &blinkLEDTaskHandle);
  xTaskCreatePinnedToCore(throttleTask, "throttle", 4096, NULL, 3, &throttleTaskHandle, 0);
  xTaskCreatePinnedToCore(spiCommTask, "SPIComm", 10096, NULL, 5, &spiCommTaskHandle, 1);
  xTaskCreate(updateBLETask, "BLE Update Task", 8192, NULL, 1, NULL);
  xTaskCreate(deviceStateUpdateTask, "State Update Task", 4096, NULL, 1, &deviceStateUpdateTaskHandle);
  // Create BLE update task with high priority but on core 1
  xTaskCreatePinnedToCore(bleStateUpdateTask, "BLEStateUpdate", 8192, NULL, 4, &bleStateUpdateTaskHandle, 1);

  // Create melody queue
  melodyQueue = xQueueCreate(5, sizeof(MelodyRequest));

  // Create audio task - pin to core 1 to avoid interference with throttle
  xTaskCreatePinnedToCore(audioTask, "Audio", 2048, NULL, 2, &audioTaskHandle, 1);

  // TODO: add watchdog task (based on esc writing to CAN)
  //xTaskCreatePinnedToCore(watchdogTask, "watchdog", 1000, NULL, 5, &watchdogTaskHandle, 0);  // Run on core 0

  xTaskCreate(updateESCBLETask, "ESC BLE Update Task", 8192, NULL, 1, NULL);

  // Create WebSerial task
  xTaskCreate(
    webSerialTask,
    "WebSerial",
    4096,
    NULL,
    1,
    &webSerialTaskHandle);

  // Create monitoring task
  xTaskCreate(monitoringTask, "Monitoring", 4096, NULL, 1, &monitoringTaskHandle);
}

void setup140() {
  if (ENABLE_BUZZ) {
    initBuzz();
  }
  const int SDA_PIN = 44;
  const int SCL_PIN = 41;
  Wire.setPins(SDA_PIN, SCL_PIN);
  setupAltimeter(board_config.alt_wire);
  if (ENABLE_VIBE) {
    initVibeMotor();
  }
}

// main loop all work is done in tasks
void loop() {
  delay(25);
}

void initButtons() {
  pinMode(board_config.button_top, INPUT_PULLUP);

  // Create button handling task
  xTaskCreatePinnedToCore(buttonHandlerTask, "ButtonHandler", 2048, NULL, 2, &buttonTaskHandle, 0);
}

// Add new button handler task
void buttonHandlerTask(void* parameter) {
  uint32_t lastDebounceTime = 0;
  bool lastButtonState = HIGH;
  bool buttonState;

  while (true) {
    buttonState = digitalRead(board_config.button_top);
    uint32_t currentTime = millis();

    // Debounce
    if ((currentTime - lastDebounceTime) > BUTTON_DEBOUNCE_TIME_MS) {
      if (buttonState != lastButtonState) {
        lastDebounceTime = currentTime;

        if (buttonState == LOW) {  // Button pressed
          buttonPressed = true;
          buttonPressStartTime = currentTime;
          USBSerial.println("Button pressed");
          USBSerial.print("Arm sequence state: ");
          USBSerial.println(armSequenceStarted ? "ACTIVE" : "INACTIVE");
        } else {  // Button released
          buttonPressed = false;
          buttonReleaseStartTime = currentTime;

          uint32_t holdDuration = buttonReleaseStartTime - buttonPressStartTime;
          USBSerial.print("Button released after ");
          USBSerial.print(holdDuration);
          USBSerial.println("ms");

          // Only start arm sequence if it was a quick click
          if (!armSequenceStarted && holdDuration < FIRST_CLICK_MAX_HOLD_MS) {
            armSequenceStarted = true;
            USBSerial.println("Quick click detected - arm sequence started");
          }
        }

        lastButtonState = buttonState;
      }

      // Handle arm sequence first
      if (armSequenceStarted) {
        // Only check for timeout if we're waiting for the second press
        // (button is not currently pressed)
        if (!buttonPressed &&
            (currentTime - buttonReleaseStartTime) > BUTTON_SEQUENCE_TIMEOUT_MS) {
          armSequenceStarted = false;
          USBSerial.println("Arm sequence timed out - waiting too long between click and hold");
        }

        // If button is pressed, check for arm completion
        if (buttonPressed) {
          uint32_t currentHoldTime = currentTime - buttonPressStartTime;

          // Complete arm sequence if held long enough
          if (currentHoldTime >= SECOND_HOLD_TIME_MS) {
            USBSerial.println("Arm sequence complete - toggling arm state");
            toggleArm();
            armSequenceStarted = false;
            buttonPressed = false;
            buttonPressStartTime = currentTime;  // Reset to prevent immediate cruise activation
          }
        }
      }
      // Only handle other button actions if we're not in an arm sequence
      else if (buttonPressed) {
        uint32_t currentHoldTime = currentTime - buttonPressStartTime;

        // Handle performance mode (only when disarmed and held long enough)
        if (currentState == DISARMED && currentHoldTime >= PERFORMANCE_MODE_HOLD_MS) {
          // toggle performance mode
          perfModeSwitch();
          buttonPressed = false;
          buttonPressStartTime = currentTime;
        }
        // Handle cruise control (when armed or cruising and held long enough)
        else if ((currentState == ARMED || currentState == ARMED_CRUISING) && currentHoldTime >= CRUISE_HOLD_TIME_MS) {
          USBSerial.println("Cruise control button activated");
          toggleCruise();
          buttonPressed = false;
          buttonPressStartTime = currentTime;
        }
      }

      // Debug current state
      if (buttonPressed) {
        uint32_t currentHoldTime = currentTime - buttonPressStartTime;
        if (currentHoldTime % 500 == 0) {  // Print every 500ms
          USBSerial.print("Current hold time: ");
          USBSerial.print(currentHoldTime);
          USBSerial.println("ms");
          USBSerial.print("Arm sequence: ");
          USBSerial.println(armSequenceStarted ? "ACTIVE" : "INACTIVE");
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void printTime(const char* label) {
  USBSerial.print(label);
  USBSerial.println(millis());
}

void disarmESC() {
  setESCThrottle(ESC_DISARMED_PWM);
}

// reset smoothing
void resetSmoothing() {
  potBuffer.clear();
}

void resumeLEDTask() {
  vTaskResume(blinkLEDTaskHandle);  // blink LED while disarmed
}

void runDisarmAlert() {
  u_int16_t disarm_melody[] = { 2637, 2093 };
  playMelody(disarm_melody, 2);
  pulseVibeMotor();
}

void updateArmedTime() {
  uint16_t newArmedTime = round(armedSecs / 60);
  if (newArmedTime <= UINT16_MAX - deviceData.armed_time) {
    deviceData.armed_time += newArmedTime;
  } else {
    // If adding the new time would cause an overflow, set the value to the maximum
    deviceData.armed_time = UINT16_MAX;
  }
}

void disarmSystem() {
  disarmESC();
  resetSmoothing();
  resumeLEDTask();
  runDisarmAlert();
  updateArmedTime();
  writeDeviceData();

  delay(500);  // TODO: just disable button thread to not allow immediate rearming
  // Set the last disarm time
  lastDisarmTime = millis();
}

void handleArmFail() {
  startArmFailIconFlash();
  handleArmFailMelody();
  pulseVibeMotor();
}

void toggleArm() {
  if (currentState == DISARMED) {
    // Check if enough time has passed since last disarm
    if (millis() - lastDisarmTime >= DISARM_COOLDOWN) {
      if (!throttleEngaged()) {
        changeDeviceState(ARMED);
      } else {
        handleArmFail();
      }
    }
    // If we're still in the cooldown period, do nothing
  } else {
    changeDeviceState(DISARMED);
  }
}

void toggleCruise() {
  switch (currentState) {
    case ARMED:
      // Check if throttle is engaged (not at zero)
      if (throttleEngaged()) {
        // Check if throttle is too high to activate cruise
        pot->update();  // Ensure we have the latest value
        int currentPotVal = pot->getValue();
        const int activationThreshold = (int)(4095 * CRUISE_ACTIVATION_MAX_POT_PERCENTAGE);  // Calculate 70% threshold

        if (currentPotVal > activationThreshold) {
          // Throttle is engaged and too high, flash the icon
          startCruiseIconFlash();
        } else {
          // Throttle is engaged and not too high, activate cruise
          changeDeviceState(ARMED_CRUISING);
          pulseVibeMotor();
        }
      } else {
         // Throttle not engaged enough to set level, flash the icon
         startCruiseIconFlash();
         USBSerial.println("Cruise activation failed: Throttle not engaged.");
      }
      break;
    case ARMED_CRUISING:
      changeDeviceState(ARMED);  // Disengage cruise
      pulseVibeMotor();
      break;
    case DISARMED:
      // Do nothing
      break;
  }
}

bool throttleSafe(int threshold = POT_ENGAGEMENT_LEVEL) {
  pot->update();
  return pot->getRawValue() < threshold;
}

/**
 * Checks if cruise control should be disengaged based on potentiometer override
 * @param potVal Current raw potentiometer value
 * @return true if cruise should be disengaged, false otherwise
 */
bool shouldDisengageCruise(int potVal) {
  unsigned long timeSinceCruiseStart = millis() - cruisedAtMillis;

  // Only check for disengagement *after* the grace period has passed
  if (timeSinceCruiseStart > CRUISE_DISENGAGE_GRACE_PERIOD_MS) {
    // Calculate the disengagement threshold based on the *raw potentiometer value* when cruise was engaged
    int disengageThresholdPotVal = (int)(cruisedPotVal * CRUISE_DISENGAGE_POT_THRESHOLD_PERCENTAGE);

    // If the *current raw potentiometer value* is greater than or equal to the threshold
    if (potVal >= disengageThresholdPotVal) {
      USBSerial.print("Cruise override: potVal ");
      USBSerial.print(potVal);
      USBSerial.print(" >= threshold ");
      USBSerial.println(disengageThresholdPotVal);
      return true;
    }
  }
  return false;
}

/**
 * Handles throttle control when in ARMED_CRUISING state
 * @param currentCruiseThrottlePWM Reference to the cruise PWM value
 * @param potVal Current raw potentiometer value
 */
void handleCruisingThrottle(uint16_t& currentCruiseThrottlePWM, int potVal) {
  // Set the ESC throttle to the determined (and potentially capped) cruise PWM
  setESCThrottle(currentCruiseThrottlePWM);

  // Check for cruise disengagement via potentiometer override
  if (shouldDisengageCruise(potVal)) {
    changeDeviceState(ARMED);  // Transition back to normal ARMED state
    pulseVibeMotor();
  }
}

/**
 * Handles throttle control when in normal ARMED state (non-cruise)
 * @param potLvl Smoothed potentiometer value
 * @param prevPotLvl Reference to previous potentiometer level for ramping
 */
void handleArmedThrottle(int potLvl, int& prevPotLvl) {
  // Apply throttle limits based on performance mode
  int rampRate = deviceData.performance_mode == 0 ? CHILL_MODE_RAMP_RATE : SPORT_MODE_RAMP_RATE;
  int maxPWM = deviceData.performance_mode == 0 ? CHILL_MODE_MAX_PWM : ESC_MAX_PWM;

  // Apply throttle ramping limits using the smoothed value
  potLvl = limitedThrottle(potLvl, prevPotLvl, rampRate);

  // Map throttle value to PWM range using the smoothed value
  int localThrottlePWM = map(potLvl, 0, 4095, ESC_MIN_PWM, maxPWM);
  setESCThrottle(localThrottlePWM);
  prevPotLvl = potLvl;  // Store smoothed value locally for next iteration
}

/**
 * Main throttle handling function - processes potentiometer input and controls ESC based on device state
 * Called every 20ms from throttleTask
 *
 * States handled:
 * - DISARMED: ESC off, reset smoothing buffers
 * - ARMED_CRUISING: Maintain cruise speed, check for potentiometer override
 * - ARMED: Normal throttle control with ramping and performance mode limits
 */
void handleThrottle() {
  static uint16_t currentCruiseThrottlePWM = ESC_MIN_PWM;
  static int prevPotLvl = 0;  // Local static variable for throttle smoothing
  uint16_t newPWM;

  // Check for throttle updates from cruise activation
  if (xQueueReceive(throttleUpdateQueue, &newPWM, 0) == pdTRUE) {
    if (currentState == ARMED_CRUISING) {
      currentCruiseThrottlePWM = newPWM;
      USBSerial.print("Cruise PWM initialized/updated to: ");
      USBSerial.println(currentCruiseThrottlePWM);
    }
  }

  // Read and buffer potentiometer values
  pot->update();
  int potVal = pot->getValue();     // Raw potentiometer value (0-4095)
  potBuffer.push(potVal);           // Add raw value to buffer
  int potLvl = averagePotBuffer();  // Calculate smoothed value

  // Handle throttle based on current device state
  switch (currentState) {
    case DISARMED:
      setESCThrottle(ESC_DISARMED_PWM);
      prevPotLvl = 0;     // Reset throttle memory when disarmed
      potBuffer.clear();  // Clear the buffer when disarmed
      break;

    case ARMED_CRUISING:
      handleCruisingThrottle(currentCruiseThrottlePWM, potVal);
      break;

    case ARMED:
      handleArmedThrottle(potLvl, prevPotLvl);
      break;
  }

    // Read/Sync ESC Telemetry (runs in all armed states)
  readESCTelemetry();
  syncESCTelemetry();
}

// Declare the static variable from esc.cpp as extern here so sync can access it
// extern unsigned long lastEscTimeUpdateMillis;  // REMOVED - Using struct member now

void syncESCTelemetry() {
  unsigned long currentTime = millis();
  // Update ESC state based first on TWAI init, then on time since last update received
  if (!escTwaiInitialized) {
    escTelemetryData.escState = TelemetryState::NOT_CONNECTED;
  }

  // Send ESC telemetry data to queue for BLE updates
  if (escTelemetryQueue != NULL) {
    xQueueOverwrite(escTelemetryQueue, &escTelemetryData);  // Always use latest data
  } else {
    USBSerial.println("ESC Queue is NULL!");
  }
}

bool throttleEngaged() {
  return !throttleSafe();
}

// average the pot buffer
int averagePotBuffer() {
  if (potBuffer.isEmpty()) {
    return 0;
  }
  int sum = 0;
  for (decltype(potBuffer)::index_t i = 0; i < potBuffer.size(); i++) {
    sum += potBuffer[i];
  }
  return sum / potBuffer.size();
}

// get the PPG ready to fly
bool armSystem() {
  uint16_t arm_melody[] = { 2093, 2637 };
  // const unsigned int arm_vibes[] = { 1, 85, 1, 85, 1, 85, 1 };
  setESCThrottle(ESC_DISARMED_PWM);  // initialize the signal to low

  armedAtMillis = millis();
  setGroundAltitude(deviceData);

  vTaskSuspend(blinkLEDTaskHandle);
  setLEDs(HIGH);  // solid LED while armed
  playMelody(arm_melody, 2);
  // runVibePattern(arm_vibes, 7);
  pulseVibeMotor();  // Ensure this is the active call
  return true;
}

void afterCruiseStart() {
  cruisedPotVal = pot->getValue();  // Store the raw pot value (0-4095) at activation
  cruisedAtMillis = millis();

  // Determine the maximum PWM based on the current flight mode (Chill/Sport)
  int maxPwmForCurrentMode = (deviceData.performance_mode == 0) ? CHILL_MODE_MAX_PWM : ESC_MAX_PWM;

  // Calculate the PWM corresponding to the potentiometer value *at the time of activation*, respecting the current flight mode's maximum PWM.
  uint16_t calculatedActivationPWM = map(cruisedPotVal, 0, 4095, ESC_MIN_PWM, maxPwmForCurrentMode);

  // Calculate the absolute maximum PWM allowed for cruise control (e.g., 60% of full ESC range)
  uint16_t absoluteMaxCruisePWM = ESC_MIN_PWM + (uint16_t)((ESC_MAX_PWM - ESC_MIN_PWM) * CRUISE_MAX_PERCENTAGE);

  // Determine the actual PWM to use for cruise: the lower of the calculated activation PWM and the absolute cap.
  uint16_t initialCruisePWM = min(calculatedActivationPWM, absoluteMaxCruisePWM);

  // Send the capped initialCruisePWM value to the throttle task via queue
  if (xQueueSend(throttleUpdateQueue, &initialCruisePWM, pdMS_TO_TICKS(100)) != pdTRUE) {
    USBSerial.println("Failed to queue initial cruise throttle PWM");
  }

  //pulseVibeMotor();
}

void afterCruiseEnd() {
  // Instead of clearing the buffer which causes throttle to drop to 0,
  // pre-populate it with the current throttle position to ensure smooth transition
  pot->update();
  int currentPotVal = pot->getValue();

  // Pre-fill the buffer with current pot value for smooth transition
  potBuffer.clear();  // Clear first
  for (int i = 0; i < 8; i++) {  // Buffer size is 8
    potBuffer.push(currentPotVal);
  }

  cruisedPotVal = 0;
  //pulseVibeMotor();
}

void playCruiseSound() {
  if (ENABLE_BUZZ) {
    uint16_t notify_melody[] = { 1976 };
    playMelody(notify_melody, 1);
  }
}

void audioTask(void* parameter) {
  MelodyRequest request;

  for (;;) {
    if (xQueueReceive(melodyQueue, &request, portMAX_DELAY) == pdTRUE) {
      if (!ENABLE_BUZZ) continue;

      TickType_t nextWakeTime = xTaskGetTickCount();
      for (int i = 0; i < request.size; i++) {
        startTone(request.notes[i]);
        TickType_t delayTicks = pdMS_TO_TICKS(request.duration);
        if (delayTicks == 0) { delayTicks = 1; }  // Ensure non-zero delay
        vTaskDelayUntil(&nextWakeTime, delayTicks);
      }
      stopTone();
    }
  }
}

void updateESCBLETask(void *pvParameters) {
  STR_ESC_TELEMETRY_140 newEscTelemetry;

  while (true) {
    // Add error checking for queue
    if (escTelemetryQueue == NULL) {
      USBSerial.println("ESC Queue not initialized!");
      vTaskDelay(pdMS_TO_TICKS(1000));  // Wait a second before retrying
      continue;
    }

    // Wait for new data with timeout
    if (xQueueReceive(escTelemetryQueue, &newEscTelemetry, pdMS_TO_TICKS(100)) == pdTRUE) {
      // Update BLE characteristics with the received data
      updateESCTelemetryBLE(newEscTelemetry);
    }

    // Add a small delay to prevent task starvation
    vTaskDelay(pdMS_TO_TICKS(40));
  }
}

bool initBMSCAN(SPIClass* spi) {
  USBSerial.println("Initializing BMS CAN...");

  // Ensure Display CS is HIGH (deselected) before working with BMS
  digitalWrite(displayCS, HIGH);

  // Create the BMS_CAN instance with the provided SPI
  bms_can = new BMS_CAN(bmsCS, MCP_BAUDRATE, spi);

  if (!bms_can->begin()) {
    USBSerial.println("Error initializing BMS_CAN");
    bmsCanInitialized = false;  // BMS initialization failed
    return false;
  }
  USBSerial.println("BMS CAN initialized successfully");
  bmsCanInitialized = true;  // BMS successfully initialized
  return true;
}

// WebSerial command handling task
void webSerialTask(void *pvParameters) {
  while (true) {
    if (USBSerial.available()) {
      // Check if we're allowed to process commands
      if (currentState == DISARMED) {
        parse_serial_commands();

        // Clear any remaining data
        while (USBSerial.available()) {
          USBSerial.read();
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100ms
  }
}

// Helper to push latest telemetry snapshot to queue (size 1, overwrite)
void pushTelemetrySnapshot() {
  if (telemetrySnapshotQueue == NULL) return;

  TelemetrySnapshot snap;
  snap.esc = escTelemetryData;
  snap.bms = bmsTelemetryData;
  xQueueOverwrite(telemetrySnapshotQueue, &snap);
}
