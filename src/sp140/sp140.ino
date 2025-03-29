// Copyright 2020 <Zach Whitehead>
// OpenPPG
#include "Arduino.h"

#include "../../inc/sp140/esp32s3-config.h"

#include "../../inc/sp140/structs.h"         // data structs
#include <Adafruit_NeoPixel.h>   // LEDs
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
#include "../../inc/sp140/display.h"
#include "../../inc/sp140/bms.h"
#include "../../inc/sp140/altimeter.h"
#include "../../inc/sp140/debug.h"

// New modular includes
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

// Function prototypes for functions previously in display.cpp and bms.cpp
void setupDisplay(const STR_DEVICE_DATA_140_V1& deviceData, const HardwareConfig& board_config);
bool initBMSCAN(SPIClass* spi);

#define BUTTON_DEBOUNCE_TIME_MS 50
#define FIRST_CLICK_MAX_HOLD_MS 500    // Maximum time for first click to be considered a click
#define SECOND_HOLD_TIME_MS 2000       // How long to hold on second press to arm
#define CRUISE_HOLD_TIME_MS 2000
#define BUTTON_SEQUENCE_TIMEOUT_MS 1500 // Time window for arm/disarm sequence
#define PERFORMANCE_MODE_HOLD_MS 3000   // Longer hold time for performance mode

// Throttle control constants
#define CHILL_MODE_MAX_PWM 1850  // 85% max power in chill mode
#define CHILL_MODE_RAMP_RATE 50  // How fast throttle can increase per cycle in chill mode
#define SPORT_MODE_RAMP_RATE 120 // How fast throttle can increase per cycle in sport mode
#define DECEL_MULTIPLIER 2.0     // How much faster deceleration is vs acceleration

// Button state tracking
volatile bool buttonPressed = false;
volatile uint32_t buttonPressStartTime = 0;
volatile uint32_t buttonReleaseStartTime = 0;
volatile bool armSequenceStarted = false;
TaskHandle_t buttonTaskHandle = NULL;

UBaseType_t uxCoreAffinityMask0 = (1 << 0);  // Core 0
UBaseType_t uxCoreAffinityMask1 = (1 << 1);  // Core 1

HardwareConfig board_config;
bool isBMSPresent = false;

ResponsiveAnalogRead* pot;

UnifiedBatteryData unifiedBatteryData = {0.0f, 0.0f, 0.0f};

CircularBuffer<float, 50> voltageBuffer;
CircularBuffer<int, 8> potBuffer;

Adafruit_NeoPixel pixels(1, 21, NEO_GRB + NEO_KHZ800);
uint32_t led_color = LED_RED; // current LED color

// Global variable for device state
volatile DeviceState currentState = DISARMED;

SemaphoreHandle_t eepromSemaphore;
SemaphoreHandle_t tftSemaphore;
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

// Add this function to handle BLE updates safely
void bleStateUpdateTask(void* parameter) {
  BLEStateUpdate update;

  while(true) {
    if(xQueueReceive(bleStateQueue, &update, portMAX_DELAY) == pdTRUE) {
      if(pDeviceStateCharacteristic != nullptr) {
        // Add delay to give BLE stack breathing room
        vTaskDelay(pdMS_TO_TICKS(20));

        // Set value first
        pDeviceStateCharacteristic->setValue(&update.state, sizeof(update.state));

        // Only notify if requested and connected
        if(update.needsNotify && deviceConnected) {
          vTaskDelay(pdMS_TO_TICKS(10)); // Additional delay before notify
          pDeviceStateCharacteristic->notify();
        }
      }
    }
    // Small delay between updates
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// Add new task to handle device state updates (similar to updateBLETask)
void deviceStateUpdateTask(void* parameter) {
  uint8_t state;

  while(true) {
    if(xQueueReceive(deviceStateQueue, &state, portMAX_DELAY) == pdTRUE) {
      if(pDeviceStateCharacteristic != nullptr && deviceConnected) {
        vTaskDelay(pdMS_TO_TICKS(20)); // Give BLE stack breathing room
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
      xQueueOverwrite(deviceStateQueue, &state); // Always use latest state
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
TaskHandle_t telemetryEscTaskHandle = NULL;
TaskHandle_t trackPowerTaskHandle = NULL;
TaskHandle_t watchdogTaskHandle = NULL;
TaskHandle_t spiCommTaskHandle = NULL;

// Add these near the top with other global declarations
QueueHandle_t melodyQueue = NULL;
TaskHandle_t audioTaskHandle = NULL;

QueueHandle_t bmsTelemetryQueue = NULL;
QueueHandle_t throttleUpdateQueue = NULL;
QueueHandle_t escTelemetryQueue = NULL;

unsigned long lastDisarmTime = 0;
const unsigned long DISARM_COOLDOWN = 500;  // 500ms cooldown

#pragma message "Warning: OpenPPG software is in beta"

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
  vTaskDelete(NULL); // should never reach this
}

void throttleTask(void *pvParameters) {
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {  // infinite loop
    handleThrottle();  //
    delay(20);  // wait for 20ms
  }
  vTaskDelete(NULL); // should never reach this
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
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void trackPowerTask(void *pvParameters) {
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {  // infinite loop
    trackPower();
    delay(500);  // wait for 250ms
  }
  vTaskDelete(NULL);  // should never reach this
}

void refreshDisplay() {
  const float altitude = getAltitude(deviceData);
  bool isArmed = (currentState != DISARMED);
  bool isCruising = (currentState == ARMED_CRUISING);
  updateDisplay(deviceData,
    escTelemetryData,
    bmsTelemetryData,
    unifiedBatteryData,
    altitude,
    isArmed,
    isCruising,
    armedAtMillis);
}

#define TELEMETRY_TIMEOUT_MS 1000  // Add near other defines


void spiCommTask(void *pvParameters) {
  for (;;) {
      #ifdef SCREEN_DEBUG
        float altitude = 0;
        generateFakeTelemetry(escTelemetryData, bmsTelemetryData, unifiedBatteryData, altitude);
        xQueueOverwrite(bmsTelemetryQueue, &bmsTelemetryData);  // Always latest data
        xQueueOverwrite(escTelemetryQueue, &escTelemetryData);  // Always latest data
        refreshDisplay();
      #else
        unsigned long currentTime = millis();

        // Update BMS data and state
        if (isBMSPresent) {
          // CS pin management is now handled inside updateBMSData
          updateBMSData();
          unifiedBatteryData.volts = bmsTelemetryData.battery_voltage;
          unifiedBatteryData.amps = bmsTelemetryData.battery_current;
          unifiedBatteryData.soc = bmsTelemetryData.soc;

          // Update BMS state based on connection status
          // Properly manage CS pins
          digitalWrite(displayCS, HIGH);  // Deselect display
          digitalWrite(bmsCS, LOW);       // Select BMS
          bmsTelemetryData.state = bms_can->isConnected() ? TelemetryState::CONNECTED : TelemetryState::NOT_CONNECTED;
          digitalWrite(bmsCS, HIGH);      // Deselect BMS when done

          xQueueOverwrite(bmsTelemetryQueue, &bmsTelemetryData);  // Always latest data
        } else {
          bmsTelemetryData.state = TelemetryState::NOT_CONNECTED;
          unifiedBatteryData.volts = escTelemetryData.volts;
          unifiedBatteryData.amps = escTelemetryData.amps;
          unifiedBatteryData.soc = 0.0; // We don't estimate SOC from voltage anymore
        }

        // Update display - CS pin management is now handled inside refreshDisplay

        refreshDisplay();

      #endif
      delay(100);  // ~10fps
  }
}

void loadHardwareConfig() {
  board_config = s3_config; // ESP32S3 is only supported board

  pot = new ResponsiveAnalogRead(board_config.throttle_pin, false);
}

void printBootMessage() {
  USBSerial.print(F("Booting up V"));
  USBSerial.print(VERSION_STRING);
}

void setupBarometer() {
  const int bmp_enabler = 9;
  pinMode(bmp_enabler, OUTPUT);
  digitalWrite(bmp_enabler, HIGH); // barometer fix for V2 board
}

void setupLED() {
  pinMode(board_config.led_sw, OUTPUT);   // set up the internal LED2 pin
  if(board_config.enable_neopixel){
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


void upgradeDeviceRevision() {  // Renamed to reflect the change from EEPROM to Preferences
  deviceData.revision = ESPCAN;
  writeDeviceData();
}

#define TAG "OpenPPG"

// Add near other task declarations at the top
TaskHandle_t timeDebugTaskHandle = NULL;

// Add the new task function before setupTasks()
void timeDebugTask(void *pvParameters) {
  for (;;) {
    timePrint();
    vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1 second
  }
}

// Add near other helper functions
void timePrint() {
  struct tm now;
  getLocalTime(&now, 0);
  if (now.tm_year >= 117) USBSerial.println(&now, "%B %d %Y %H:%M:%S (%A)");
}

/**
 * Initializes the necessary components and configurations for the device setup.
 * This function is called once at the beginning of the program execution.
 */

void setup() {
  USBSerial.begin(115200); // This is for debug output and WebSerial

  // Pull CSB (pin 42) high to activate I2C mode
  // temporary fix TODO remove
  digitalWrite(42, HIGH);
  pinMode(42, OUTPUT);

  refreshDeviceData();
  printBootMessage();
  setupBarometer();

  upgradeDeviceRevision();
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
  setupDisplay(deviceData, board_config);

  #ifndef SCREEN_DEBUG
    // Pass the hardware SPI instance to the BMS_CAN initialization
    isBMSPresent = initBMSCAN(hardwareSPI);
  #endif

  initESC(0);
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

  setupBLE();
  setupTasks();  // Create all tasks after queues and BLE are initialized

  pulseVibeMotor();
  if (digitalRead(board_config.button_top) == LOW) {  // LOW means pressed since it's INPUT_PULLUP
    modeSwitch(false);
  }

  setLEDColor(LED_GREEN);

  // Send initial device data after setup
  send_device_data();
}

// set up all the main threads/tasks with core 0 affinity
void setupTasks() {
  //xTaskCreate(telemetryEscTask, "telemetryEscTask", 4096, NULL, 2, &telemetryEscTaskHandle);
  xTaskCreate(blinkLEDTask, "blinkLed", 1536, NULL, 1, &blinkLEDTaskHandle);
  xTaskCreatePinnedToCore(throttleTask, "throttle", 4096, NULL, 3, &throttleTaskHandle, 0);
  xTaskCreatePinnedToCore(spiCommTask, "SPIComm", 10096, NULL, 5, &spiCommTaskHandle, 1);
  xTaskCreate(updateBLETask, "BLE Update Task", 4096, NULL, 1, NULL);
  xTaskCreate(deviceStateUpdateTask, "State Update Task", 4096, NULL, 1, &deviceStateUpdateTaskHandle);
  // Create BLE update task with high priority but on core 1
  xTaskCreatePinnedToCore(bleStateUpdateTask, "BLEStateUpdate", 4096, NULL, 4, &bleStateUpdateTaskHandle, 1);
  //xTaskCreate(timeDebugTask, "Time Debug", 2048, NULL, 1, &timeDebugTaskHandle);

  // Create melody queue
  melodyQueue = xQueueCreate(5, sizeof(MelodyRequest));

  // Create audio task - pin to core 1 to avoid interference with throttle
  xTaskCreatePinnedToCore(audioTask, "Audio", 2048, NULL, 2, &audioTaskHandle, 1);

  // TODO: add watchdog task (based on esc writing to CAN)
  //xTaskCreatePinnedToCore(watchdogTask, "watchdog", 1000, NULL, 5, &watchdogTaskHandle, 0);  // Run on core 0

  xTaskCreate(updateESCBLETask, "ESC BLE Update Task", 4096, NULL, 1, NULL);

  // Create WebSerial task
  xTaskCreate(
    webSerialTask,
    "WebSerial",
    4096,
    NULL,
    1,
    &webSerialTaskHandle
  );
}


void setup140() {
  // TODO: write to CAN bus

  initBuzz();
  const int SDA_PIN = 44;
  const int SCL_PIN = 41;
  Wire.setPins(SDA_PIN, SCL_PIN);
  setupAltimeter(board_config.alt_wire);
  if (board_config.enable_vib) {
    initVibeMotor();
  }
}

// main loop - check for serial commands
void loop() {
  // Just delay, all work is done in tasks
  delay(25);
}

void initButtons() {
  pinMode(board_config.button_top, INPUT_PULLUP);

  // Create button handling task
  xTaskCreatePinnedToCore(
    buttonHandlerTask,
    "ButtonHandler",
    2048,
    NULL,
    2,
    &buttonTaskHandle,
    0
  );
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

        if (buttonState == LOW) { // Button pressed
          buttonPressed = true;
          buttonPressStartTime = currentTime;
          USBSerial.println("Button pressed");
          USBSerial.print("Arm sequence state: ");
          USBSerial.println(armSequenceStarted ? "ACTIVE" : "INACTIVE");
        } else { // Button released
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
            buttonPressStartTime = currentTime; // Reset to prevent immediate cruise activation
          }
        }
      }
      // Only handle other button actions if we're not in an arm sequence
      else if (buttonPressed) {
        uint32_t currentHoldTime = currentTime - buttonPressStartTime;

        // Handle performance mode (only when disarmed and held long enough)
        if (currentState == DISARMED && currentHoldTime >= PERFORMANCE_MODE_HOLD_MS) {
          USBSerial.println("Toggling performance mode");
          modeSwitch(false);
          buttonPressed = false;
          buttonPressStartTime = currentTime;
        }
        // Handle cruise control (only when armed and held long enough)
        else if (currentState == ARMED && currentHoldTime >= CRUISE_HOLD_TIME_MS) {
          USBSerial.println("Cruise control activated");
          toggleCruise();
          buttonPressed = false;
          buttonPressStartTime = currentTime;
        }
      }

      // Debug current state
      if (buttonPressed) {
        uint32_t currentHoldTime = currentTime - buttonPressStartTime;
        if (currentHoldTime % 500 == 0) { // Print every 500ms
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
  // Note: prevPotLvl is now handled locally in handleThrottle function
}

void resumeLEDTask() {
  vTaskResume(blinkLEDTaskHandle);  // blink LED while disarmed
}

void runDisarmAlert() {
  u_int16_t disarm_melody[] = { 2093, 1976, 880 };
  const unsigned int disarm_vibes[] = { 78, 49 };
  runVibePattern(disarm_vibes, 2);
  playMelody(disarm_melody, 3);
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
      if (throttleEngaged()) {
        changeDeviceState(ARMED_CRUISING);
      } else {
        // Call modeSwitch when armed but throttle not engaged
        modeSwitch(false);
      }
      break;
    case ARMED_CRUISING:
      changeDeviceState(ARMED);  // This will now call removeCruise
      break;
    case DISARMED:
      // show stats screen
      //TODO refactor updateDisplayTaskHandle since now its shared spiCommTask.
      // the screen updates should be aware of this and
      // suspend/resume in their own funcitons vs at a task level.

      // displayMeta(deviceData);
      break;
  }
}

// Variable to track if a button was clicked
bool wasClicked = false;

// Timestamp when the button was released
unsigned long releaseTime = 0;

// Time threshold for LongClick after release (in milliseconds)
const unsigned long longClickThreshold = 3500;  // adjust as necessary

//#define THROTTLE_DEBUG

#ifdef THROTTLE_DEBUG
// Debug mode globals
static int debugValue = 0;
static bool increasing = true;
static unsigned long lastStepTime = 0;
static const int STEPS_PER_SECOND = 4;  // 4 steps per second
static const unsigned long STEP_INTERVAL = 1000 / STEPS_PER_SECOND; // 250ms per step
static const int STEP_SIZE = ceil(4096.0 / (STEPS_PER_SECOND * 30)); // Complete up or down in ~30 seconds
#endif

 // Normal throttle behavior
#define ESC_MIN_SPIN_PWM 1105

bool throttleSafe(int threshold = POT_ENGAGEMENT_LEVEL) {
  pot->update();
  return pot->getRawValue() < threshold;
}

void handleThrottle() {
  static int maxPWM = ESC_MAX_PWM;
  static uint16_t currentCruiseThrottlePWM = ESC_MIN_SPIN_PWM;
  static int prevPotLvl = 0;  // Local static variable for throttle smoothing
  uint16_t newPWM;

  // Check for throttle updates from cruise control
  if (xQueueReceive(throttleUpdateQueue, &newPWM, 0) == pdTRUE) {
    if (currentState == ARMED_CRUISING) {
      currentCruiseThrottlePWM = newPWM;
    }
  }

  pot->update();
  int potVal = pot->getValue();
  int potLvl = potVal;

  // Handle throttle based on current state and update BLE accordingly
  if (currentState == DISARMED) {
    setESCThrottle(ESC_DISARMED_PWM);
    updateThrottleBLE(0); // Send 0 throttle when disarmed
    prevPotLvl = 0;  // Reset throttle memory when disarmed
  } else if (currentState == ARMED_CRUISING) {
    setESCThrottle(currentCruiseThrottlePWM);
    // Map PWM back to throttle value for BLE
    int mappedThrottle = map(currentCruiseThrottlePWM, ESC_MIN_SPIN_PWM, ESC_MAX_PWM, 0, 4095);
    USBSerial.print("Cruise Mode - Sending throttle value: ");
    USBSerial.print(mappedThrottle);
    USBSerial.print(" (PWM: ");
    USBSerial.print(currentCruiseThrottlePWM);
    USBSerial.println(")");
    updateThrottleBLE(mappedThrottle);
    // Don't update prevPotLvl in cruise mode
  } else {
    // Apply throttle limits based on performance mode
    int rampRate = deviceData.performance_mode == 0 ? CHILL_MODE_RAMP_RATE : SPORT_MODE_RAMP_RATE;
    maxPWM = deviceData.performance_mode == 0 ? CHILL_MODE_MAX_PWM : ESC_MAX_PWM;

    // Apply throttle ramping limits
    potLvl = limitedThrottle(potLvl, prevPotLvl, rampRate);

    // Map throttle value to PWM range
    int localThrottlePWM = map(potLvl, 0, 4095, ESC_MIN_SPIN_PWM, maxPWM);
    setESCThrottle(localThrottlePWM);
    prevPotLvl = potLvl;  // Store locally for next iteration
    updateThrottleBLE(potLvl); // Send actual throttle value
  }

  // Check if throttle has moved away from cruise position
  if (currentState == ARMED_CRUISING) {
    int safeThreshold = cruisedPotVal + 200; // Allow 5% movement
    if (!throttleSafe(safeThreshold)) {
      USBSerial.println("Cruise control override detected");
      changeDeviceState(ARMED);
      return;
    }
  }

  readESCTelemetry();
  syncESCTelemetry();
}

// push telemetry data to the queue for BLE updates etc
void syncESCTelemetry() {
  unsigned long currentTime = millis();

  if (!isBMSPresent) {
    unifiedBatteryData.volts = escTelemetryData.volts;
    unifiedBatteryData.amps = escTelemetryData.amps;
    unifiedBatteryData.soc = 0.0; // We don't estimate SOC from voltage anymore
  }

  // Update ESC state based on last update time
  if (escTelemetryData.lastUpdateMs == 0) {
    escTelemetryData.state = TelemetryState::NOT_CONNECTED;
  } else if (currentTime - escTelemetryData.lastUpdateMs > TELEMETRY_TIMEOUT_MS) {
    escTelemetryData.state = TelemetryState::STALE;
  } else {
    escTelemetryData.state = TelemetryState::CONNECTED;
  }

  // Send to queue for BLE updates
  if (escTelemetryQueue != NULL) {
    xQueueOverwrite(escTelemetryQueue, &escTelemetryData);  // Always use latest data
  } else {
    USBSerial.println("ESC Queue is NULL!");
  }
}

bool throttleEngaged() {
  return !throttleSafe();
}

int averagePotBuffer() {
  int sum = 0;
  for (decltype(potBuffer)::index_t i = 0; i < potBuffer.size(); i++) {
    sum += potBuffer[i];
  }
  return sum / potBuffer.size();
}

// get the PPG ready to fly
bool armSystem() {
  uint16_t arm_melody[] = { 1760, 1976, 2093 };
  const unsigned int arm_vibes[] = { 1, 85, 1, 85, 1, 85, 1 };
  setESCThrottle(ESC_DISARMED_PWM);  // initialize the signal to low

  armedAtMillis = millis();
  setGroundAltitude(deviceData);

  vTaskSuspend(blinkLEDTaskHandle);
  setLEDs(HIGH); // solid LED while armed
  runVibePattern(arm_vibes, 7);
  playMelody(arm_melody, 3);

  return true;
}

void afterCruiseStart() {
  cruisedPotVal = pot->getValue();
  cruisedAtMillis = millis();

  // Calculate initial cruise PWM
  uint16_t initialPWM = map(cruisedPotVal, 0, 4095, ESC_MIN_SPIN_PWM, ESC_MAX_PWM);

  // Send to queue
  if (xQueueSend(throttleUpdateQueue, &initialPWM, pdMS_TO_TICKS(100)) != pdTRUE) {
    USBSerial.println("Failed to queue initial cruise throttle");
  }

  // Update BLE characteristic with raw throttle value
  if (pThrottleCharacteristic && deviceConnected) {
    pThrottleCharacteristic->setValue((uint8_t*)&cruisedPotVal, sizeof(cruisedPotVal));
    pThrottleCharacteristic->notify();
  }

  playCruiseSound();
  pulseVibeMotor();
}

void afterCruiseEnd() {
  cruisedPotVal = 0;
  playCruiseSound();
  pulseVibeMotor();
}

void playCruiseSound() {
  if (ENABLE_BUZ) {
    uint16_t notify_melody[] = { 900, 900 };
    playMelody(notify_melody, 2);
  }
}

unsigned long prevPwrMillis = 0;

void trackPower() {
  unsigned long currentPwrMillis = millis();
  unsigned long msec_diff = (currentPwrMillis - prevPwrMillis);  // eg 0.30 sec
  prevPwrMillis = currentPwrMillis;

  if (currentState != DISARMED) {
    wattHoursUsed += round(watts/60/60*msec_diff)/1000.0;
  }
}

void audioTask(void* parameter) {
  MelodyRequest request;

  for(;;) {
    if(xQueueReceive(melodyQueue, &request, portMAX_DELAY) == pdTRUE) {
      if (!ENABLE_BUZ) continue;

      TickType_t nextWakeTime = xTaskGetTickCount();
      for(int i = 0; i < request.size; i++) {
        tone(board_config.buzzer_pin, request.notes[i]);
        TickType_t delayTicks = pdMS_TO_TICKS(request.duration);
        if(delayTicks == 0) { delayTicks = 1; } // Ensure non-zero delay
        vTaskDelayUntil(&nextWakeTime, delayTicks);
      }
      noTone(board_config.buzzer_pin);
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
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setupDisplay(const STR_DEVICE_DATA_140_V1& deviceData, const HardwareConfig& board_config) {
  USBSerial.println("setupDisplay");

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

  // Create the display with the hardware SPI
  display = new Adafruit_ST7735(
    hardwareSPI,
    displayCS,
    board_config.tft_dc,
    board_config.tft_rst);

  display->initR(INITR_BLACKTAB);  // Init ST7735S chip, black tab
  resetRotation(deviceData.screen_rotation);
  setTheme(deviceData.theme);  // 0=light, 1=dark
  display->fillScreen(ST77XX_BLACK);
  displayMeta(deviceData, 1500);
}

bool initBMSCAN(SPIClass* spi) {
  USBSerial.println("Initializing BMS CAN...");

  // Ensure Display CS is HIGH (deselected) before working with BMS
  digitalWrite(displayCS, HIGH);

  // CS pin should already be configured in setupDisplay

  // Create the BMS_CAN instance with the provided SPI
  bms_can = new BMS_CAN(bmsCS, MCP_BAUDRATE, spi);

  if (!bms_can->begin()) {
    USBSerial.println("Error initializing BMS_CAN");
    isBMSPresent = false;  // BMS initialization failed
    return false;
  }
  USBSerial.println("BMS CAN initialized successfully");
  isBMSPresent = true;  // BMS successfully initialized
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
    vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
  }
}
