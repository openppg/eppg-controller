// Copyright 2020 <Zach Whitehead>
// OpenPPG
#include "Arduino.h"

#include "../../lib/crc.c"       // packet error checking
#ifdef RP_PIO
  #include "../../inc/sp140/rp2040-config.h"         // device config
  #include <FreeRTOS.h>
  #include <task.h>
  #include <semphr.h>
  #include <map>
#elif CAN_PIO
  #include "../../inc/sp140/esp32s3-config.h"
#endif

#include "../../inc/sp140/structs.h"         // data structs
#include <AceButton.h>           // button clicks
#include <Adafruit_NeoPixel.h>   // LEDs
#include <ArduinoJson.h>
#include <CircularBuffer.hpp>      // smooth out readings
#include <ResponsiveAnalogRead.h>  // smoothing for throttle
#include <SPI.h>
#include <TimeLib.h>  // convert time to hours mins etc
#include <Wire.h>
#ifdef USE_TINYUSB
  #include "Adafruit_TinyUSB.h"
#endif

#ifdef RP_PIO
  // rp2040 specific libraries here
  #include <EEPROM.h>
  #include "hardware/watchdog.h"
  #include "pico/unique_id.h"
#elif CAN_PIO
  // ESP32S3 (CAN) specific libraries here
  #include "esp_task_wdt.h"
  #include "EEPROM.h"
#endif

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

#ifdef USE_DRV2605
  #include "../../inc/sp140/vibration_drv2605.h"
#else
  #include "../../inc/sp140/vibration_pwm.h"
#endif

using namespace ace_button;

UBaseType_t uxCoreAffinityMask0 = (1 << 0);  // Core 0
UBaseType_t uxCoreAffinityMask1 = (1 << 1);  // Core 1

HardwareConfig board_config;
bool isBMSPresent = false;

// USB WebUSB object
#ifdef USE_TINYUSB
Adafruit_USBD_WebUSB usb_web;
WEBUSB_URL_DEF(landingPage, 1 /*https*/, "config.openppg.com");
#endif

ResponsiveAnalogRead* pot;
AceButton* button_top;
ButtonConfig* buttonConfig;

UnifiedBatteryData unifiedBatteryData = {0.0f, 0.0f, 0.0f};

CircularBuffer<float, 50> voltageBuffer;
CircularBuffer<int, 8> potBuffer;

#ifndef CAN_PIO
  Adafruit_NeoPixel pixels(1, LED_BUILTIN, NEO_GRB + NEO_KHZ800);
#else
  Adafruit_NeoPixel pixels(1, 21, NEO_GRB + NEO_KHZ800);
#endif
uint32_t led_color = LED_RED; // current LED color

// New enum for device state
enum DeviceState {
  DISARMED,
  ARMED,
  ARMED_CRUISING
};

// Global variable for device state
volatile DeviceState currentState = DISARMED;

SemaphoreHandle_t eepromSemaphore;
SemaphoreHandle_t tftSemaphore;
SemaphoreHandle_t stateMutex;

void changeDeviceState(DeviceState newState) {
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    DeviceState oldState = currentState;
    currentState = newState;

    // Update BLE characteristic
    if (pDeviceStateCharacteristic && deviceConnected) {
      uint8_t state = (uint8_t)newState;
      pDeviceStateCharacteristic->setValue(&state, sizeof(state));
      pDeviceStateCharacteristic->notify();
    }

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

QueueHandle_t bmsTelemetryQueue = NULL;
QueueHandle_t throttleUpdateQueue = NULL;


unsigned long lastDisarmTime = 0;
const unsigned long DISARM_COOLDOWN = 500;  // 500ms cooldown

#pragma message "Warning: OpenPPG software is in beta"

void watchdogTask(void* parameter) {
  for (;;) {
    #ifndef OPENPPG_DEBUG
      #ifdef RP_PIO
        watchdog_update();
      #elif CAN_PIO
       ESP_ERROR_CHECK(esp_task_wdt_reset());
      #endif
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

// TODO: remove this task because its called from handleThrottle()
void telemetryEscTask(void *pvParameters) {
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {  // infinite loop
    readESCTelemetry();
    if (!isBMSPresent) {
      unifiedBatteryData.volts = escTelemetryData.volts;
      unifiedBatteryData.amps = escTelemetryData.amps;
      unifiedBatteryData.soc = getBatteryPercent(escTelemetryData.volts);
    }
    delay(50);  // wait for 100ms
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

// For tasks that use the SPI bus cant run in parallel
void spiCommTask(void *pvParameters) {
  for (;;) {
      // Update BMS data
      if (isBMSPresent) {
        updateBMSData();
        unifiedBatteryData.volts = bmsTelemetryData.battery_voltage;
        unifiedBatteryData.amps = bmsTelemetryData.battery_current;
        unifiedBatteryData.soc = bmsTelemetryData.soc;

        xQueueOverwrite(bmsTelemetryQueue, &bmsTelemetryData);  // Always latest data
      }

      // Update display
      refreshDisplay();
      delay(250);
  }
}

void loadHardwareConfig() {
  board_config = s3_config; // ESP32S3 is only supported board

  pot = new ResponsiveAnalogRead(board_config.throttle_pin, false);
  button_top = new AceButton(board_config.button_top);
  buttonConfig = button_top->getButtonConfig();
}

#ifdef USE_TINYUSB
void setupUSBWeb() {
  usb_web.begin();
  usb_web.setLandingPage(&landingPage);
  usb_web.setLineStateCallback(line_state_callback);
}
#endif

void printBootMessage() {
  #ifdef CAN_PIO
  USBSerial.print(F("Booting up V"));
  USBSerial.print(VERSION_STRING);
  #else
  Serial.print(F("Booting up (USB) V"));
  Serial.print(VERSION_STRING);
  #endif
}

void setupBarometer() {
  const int bmp_enabler = 9;
  pinMode(bmp_enabler, OUTPUT);
  digitalWrite(bmp_enabler, HIGH); // barometer fix for V2 board
}

void setupEEPROM() {  // TODO: move to extra-data.ino or own file
#ifdef RP_PIO
  EEPROM.begin(255);
#elif CAN_PIO
  // Initialize EEPROM
  if (!EEPROM.begin(sizeof(deviceData))) {
    USBSerial.println("Failed to initialise EEPROM");
    // You might want to add some error handling here
  } else {
    USBSerial.println("EEPROM initialized successfully");
  }
#endif
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
  #ifdef RP_PIO
    watchdog_enable(4000, 1);
  #elif CAN_PIO
    // Initialize Task Watchdog
    //ESP_ERROR_CHECK(esp_task_wdt_init(3000, true));  // 3 second timeout, panic on timeout
  #endif
#endif // OPENPPG_DEBUG
}


void upgradeDeviceRevisionInEEPROM() {
#ifdef RP_PIO
  if (deviceData.revision == M0) {  // onetime conversion because default is 1
    deviceData.revision = V1;
    writeDeviceData();
  }
#elif CAN_PIO
  deviceData.revision = ESPCAN;
  writeDeviceData();
#endif
}

#define TAG "OpenPPG"

void testTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(500); // 500ms
  xLastWakeTime = xTaskGetTickCount();
  int count = 0;

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    count++;

    TickType_t period = xTaskGetTickCount() - xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    USBSerial.println(".");
    //setESCThrottle(ESC_DISARMED_PWM);
    //dumpESCMessages();

    // Log the heartbeat event
    Insights.event(TAG, "[heartbeat][%d] [period_ms][%u]", count, period * portTICK_PERIOD_MS);

    // Every 20 iterations (roughly every 10 seconds), dump the heap info
    if (count % 20 == 0) {
      Insights.metrics.dumpHeap();
    }

    // Every 120 iterations (roughly every minute), log some system info
    if (count % 120 == 0) {
      Insights.event(TAG, "[uptime_min][%d] [free_heap][%u] [min_free_heap][%u]",
                     count / 120,
                     esp_get_free_heap_size(),
                     esp_get_minimum_free_heap_size());
    }

    // Log throttle level and device state every 10 iterations (roughly every 5 seconds)
    if (count % 10 == 0) {
      Insights.metrics.dumpHeap();
    }
  }
}

TaskHandle_t testTaskHandle = NULL;

#ifdef WIFI_DEBUG

void setupWiFi() {
//   WiFi.mode(WIFI_STA);
//   WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     USBSerial.println("Wifi connecting...");
//   }
//   USBSerial.println("");
//   USBSerial.println("WiFi connected");

//   if(!Insights.begin(insights_auth_key)){
//     USBSerial.println("Failed to initialize Insights");
//   }
}
#endif


/**
 * Initializes the necessary components and configurations for the device setup.
 * This function is called once at the beginning of the program execution.
 */

void setup() {
  USBSerial.begin(115200);
  #ifdef USE_TINYUSB
    setupUSBWeb();
  #endif
  // Pull CSB (pin 42) high to activate I2C mode
  // temporary fix TODO remove
  digitalWrite(42, HIGH);
  pinMode(42, OUTPUT);

  setupEEPROM();
  refreshDeviceData();
  printBootMessage();
  setupBarometer();

  upgradeDeviceRevisionInEEPROM();
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
  isBMSPresent = initBMSCAN();
  initESC(0);
  eepromSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(eepromSemaphore);
  stateMutex = xSemaphoreCreateMutex();
  setESCThrottle(ESC_DISARMED_PWM);
  initVibeMotor();

  // Create BMS telemetry queue before setting up tasks
  bmsTelemetryQueue = xQueueCreate(1, sizeof(STR_BMS_TELEMETRY_140));
  if (bmsTelemetryQueue == NULL) {
    USBSerial.println("Error creating BMS telemetry queue");
  }

  // Add throttle update queue
  throttleUpdateQueue = xQueueCreate(1, sizeof(uint16_t));
  if (throttleUpdateQueue == NULL) {
    USBSerial.println("Error creating throttle update queue");
  }

  setupTasks();  // Move this after queue creation

  pulseVibeMotor();
  if (button_top->isPressedRaw()) {
    modeSwitch(false);
  }
  setupBLE();
  // TODO: refactor to use queue etc
  setLEDColor(LED_GREEN);
}

// set up all the main threads/tasks with core 0 affinity
void setupTasks() {
 #ifdef CAN_PIO
  //xTaskCreate(telemetryEscTask, "telemetryEscTask", 4096, NULL, 2, &telemetryEscTaskHandle);
  xTaskCreate(blinkLEDTask, "blinkLed", 1536, NULL, 1, &blinkLEDTaskHandle);
  xTaskCreatePinnedToCore(throttleTask, "throttle", 4096, NULL, 3, &throttleTaskHandle, 0);
  xTaskCreatePinnedToCore(spiCommTask, "SPIComm", 10096, NULL, 5, &spiCommTaskHandle, 1);
  xTaskCreate(updateBLETask, "BLE Update Task", 4096, NULL, 1, NULL);

  // TODO: add watchdog task (based on esc writing to CAN)
  //xTaskCreatePinnedToCore(watchdogTask, "watchdog", 1000, NULL, 5, &watchdogTaskHandle, 0);  // Run on core 0
#endif
}


void setup140() {
  #ifdef CAN_PIO
  // TODO: write to CAN bus
  #else
    initESC(board_config.esc_pin);
  #endif

  initBuzz();
  #ifdef RP_PIO
  Wire1.setSDA(A0); // Have to use Wire1 because pins are assigned that in hardware
  Wire1.setSCL(A1);
  #elif CAN_PIO
  const int SDA_PIN = 44;
  const int SCL_PIN = 41;
  Wire.setPins(SDA_PIN, SCL_PIN);
  #endif
  setupAltimeter(board_config.alt_wire);
  if (board_config.enable_vib) {
    initVibeMotor();
  }
}

// main loop - everything runs in threads
void loop() {
//   // from WebUSB to both Serial & webUSB
// #ifdef USE_TINYUSB
//   if (currentState == DISARMED && usb_web.available()) parse_usb_serial();
// #endif

  // more stable in main loop
  checkButtons();
  delay(5);
  //USBSerial.print(".");
}

void checkButtons() {
  //USBSerial.println("checkButtons");
  button_top->check();
}

void printTime(const char* label) {
  Serial.print(label);
  Serial.println(millis());
}

void disarmESC() {
  setESCThrottle(ESC_DISARMED_PWM);
}

// reset smoothing
void resetSmoothing() {
  potBuffer.clear();
  prevPotLvl = 0;
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

void handleButtonEvent(AceButton* btn, uint8_t eventType, uint8_t /* st */) {
  switch (eventType) {
  case AceButton::kEventClicked:
    wasClicked = true;
    USBSerial.println("Button Clicked");
    break;
  case AceButton::kEventReleased:
    if (wasClicked) {
      releaseTime = millis();
      wasClicked = false;
      USBSerial.println("Button Released after Click");
    }
    break;
  case AceButton::kEventDoubleClicked:
    //toggleArm();
    USBSerial.println("Double Click arm");
    break;
  case AceButton::kEventLongPressed:
      toggleArm();
      //toggleCruise
      USBSerial.println("Long Press - cruise");
    break;
  }
}

// initial button setup and config
void initButtons() {
  pinMode(board_config.button_top, INPUT_PULLUP);

  buttonConfig->setEventHandler(handleButtonEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterLongPress);
  buttonConfig->setLongPressDelay(2000);
  buttonConfig->setClickDelay(300);
  //buttonConfig->setDoubleClickDelay(900);
  //buttonConfig->setDebounceDelay(100);
}

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

void handleThrottle() {
  static int maxPWM = ESC_MAX_PWM;
  static uint16_t currentCruiseThrottlePWM = ESC_MIN_SPIN_PWM;
  uint16_t newPWM;

  // Check for throttle updates
  if (xQueueReceive(throttleUpdateQueue, &newPWM, 0) == pdTRUE) {
    if (currentState == ARMED_CRUISING) {
      currentCruiseThrottlePWM = newPWM;
    }
  }

  pot->update();
  int potVal = pot->getValue();

  // Update BLE clients with new value
  updateThrottleBLE(potVal);

  if (currentState == DISARMED) {
    setESCThrottle(ESC_DISARMED_PWM);
  } else if (currentState == ARMED_CRUISING) {
    setESCThrottle(currentCruiseThrottlePWM);
  } else {
    int localThrottlePWM = map(potVal, 0, 4095, ESC_MIN_SPIN_PWM, maxPWM);
    setESCThrottle(localThrottlePWM);
  }
  readESCTelemetry();
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

// Returns true if the throttle/pot is above the engagement threshold
bool throttleEngaged() {
  pot->update();
  if (pot->getRawValue() >= POT_ENGAGEMENT_LEVEL) {
    return true;
  }
  return false;
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
  Insights.metrics.setInt("flight_duration", (millis() - armedAtMillis) / 1000);
  Insights.metrics.setFloat("watt_hours_used", wattHoursUsed);
}
