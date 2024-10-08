// Copyright 2020 <Zach Whitehead>
// OpenPPG
#include "Arduino.h"

#include "../../lib/crc.c"       // packet error checking
#ifdef M0_PIO
  #include "../../inc/sp140/m0-config.h"          // device config
  // TODO: find best SAMD21 FreeRTOS port
#elif RP_PIO
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

#ifdef M0_PIO
  // SAMD21 specific libraries here
  #include <Adafruit_SleepyDog.h>  // watchdog
  #include <extEEPROM.h>  // https://github.com/PaoloP74/extEEPROM
#elif RP_PIO
  // rp2040 specific libraries here
  #include <EEPROM.h>
  #include "hardware/watchdog.h"
  #include "pico/unique_id.h"
#elif CAN_PIO
  // ESP32S3 (CAN) specific libraries here
  #include "esp_task_wdt.h"
  #include "EEPROM.h"
#endif

#include "../../inc/sp140/globals.h"  // device config
#include "../../inc/sp140/esc.h"
#include "../../inc/sp140/display.h"
#include "../../inc/sp140/bms.h"
#include "../../inc/sp140/altimeter.h"
#include "../../inc/sp140/vibration.h"


using namespace ace_button;

UBaseType_t uxCoreAffinityMask0 = (1 << 0); // Core 0
UBaseType_t uxCoreAffinityMask1 = (1 << 1); // Core 1

HardwareConfig board_config;

// USB WebUSB object
#ifdef USE_TINYUSB
Adafruit_USBD_WebUSB usb_web;
WEBUSB_URL_DEF(landingPage, 1 /*https*/, "config.openppg.com");
#endif

ResponsiveAnalogRead* pot;
AceButton* button_top;
ButtonConfig* buttonConfig;

#ifdef M0_PIO
  extEEPROM eep(kbits_64, 1, 64);
#endif

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
    switch (newState) {
      case DISARMED:
        disarmSystem();
        break;
      case ARMED:
        if (oldState == DISARMED) {
          // Perform full arm sequence when transitioning from DISARMED to ARMED
          armSystem();
        } else if (oldState == ARMED_CRUISING) {
          // When transitioning from ARMED_CRUISING to ARMED, only remove cruise
          // This avoids re-arming the system unnecessarily
           afterCruiseEnd();
        }
        break;
      case ARMED_CRUISING:
        if (oldState == ARMED) {
          afterCruiseStart();
        } else {
          // Should never happen
          // Do nothing if not already in ARMED state
          currentState = oldState;  // Revert to the previous state
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
TaskHandle_t telemetryBmsTaskHandle = NULL;
TaskHandle_t trackPowerTaskHandle = NULL;
TaskHandle_t updateDisplayTaskHandle = NULL;
TaskHandle_t watchdogTaskHandle = NULL;
TaskHandle_t spiCommunicationTaskHandle = NULL;

#ifdef CAN_PIO
#define POTENTIOMETER_PIN 8


#endif

unsigned long lastDisarmTime = 0;
const unsigned long DISARM_COOLDOWN = 500; // 500ms cooldown

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
    //blinkLED();  // call blinkLED function
    USBSerial.println("blinkLEDTask");
    delay(500);  // wait for 500ms
  }
  vTaskDelete(NULL); // should never reach this
}

void throttleTask(void *pvParameters) {
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {  // infinite loop
    handleThrottle();  //
    delay(22);  // wait for 22ms
  }
  vTaskDelete(NULL); // should never reach this
}

void telemetryEscTask(void *pvParameters) {
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {  // infinite loop
    readESCTelemetry();
    delay(100);  // wait for 100ms
  }
  vTaskDelete(NULL);  // should never reach this
}

void telemetryBmsTask(void *pvParameters) {
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {  // infinite loop
    updateBMSData();
    delay(200);  // wait for 100ms
  }
  vTaskDelete(NULL);  // should never reach this
}

void trackPowerTask(void *pvParameters) {
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {  // infinite loop
    trackPower();
    delay(250);  // wait for 250ms
  }
  vTaskDelete(NULL);  // should never reach this
}

void refreshDisplay() {
  const float altitude = getAltitude(deviceData);
  bool isArmed = (currentState != DISARMED);
  bool isCruising = (currentState == ARMED_CRUISING);
  updateDisplay(deviceData, escTelemetryData, altitude, isArmed, isCruising, armedAtMillis);
}

void updateDisplayTask(void *pvParameters) {
  (void) pvParameters;  // this is a standard idiom to avoid compiler warnings about unused parameters.

  for (;;) {
    // TODO: separate alt reading out to its own task. Avoid blocking display updates when alt reading is slow etc
    // TODO: use queues to pass data between tasks (xQueueOverwrite)const float altitude = getAltitude(deviceData);
    refreshDisplay();
    delay(250);
  }
  vTaskDelete(NULL);  // should never reach this
}

void spiCommunicationTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(250); // Adjust as needed

  xLastWakeTime = xTaskGetTickCount();

  for(;;) {
      // Update BMS data
      updateBMSData();

      // Update display
      refreshDisplay();

      // Wait for the next cycle.
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void loadHardwareConfig() {
  if (deviceData.revision == 1) {
    board_config = v1_config;
  } else if (deviceData.revision == 2) {
    board_config = v2_config;
  } else if (deviceData.revision == 3) {
    board_config = v3_config;
  } else {
    // Handle other cases or throw an error
  }
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
#ifdef M0_PIO
  uint8_t eepStatus = eep.begin(eep.twiClock100kHz);
#elif RP_PIO
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
  #ifdef M0_PIO
    Watchdog.enable(5000);
  #elif RP_PIO
    watchdog_enable(4000, 1);
  #elif CAN_PIO
    // Initialize Task Watchdog
    ESP_ERROR_CHECK(esp_task_wdt_init(3000, true));  // 3 second timeout, panic on timeout
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

void testTask(void *pvParameters) {
  for (;;) {
    USBSerial.println(".");
    //setESCThrottle(ESC_DISARMED_PWM);
    //dumpESCMessages();
    delay(500);
  }
}

TaskHandle_t testTaskHandle = NULL;

#ifdef CAN_PIO
//just for testing
void setup() {
  USBSerial.begin(115200);
  delay(100);  // Give some time for USB CDC to initialize
  // while (!USBSerial) {  // TODO: remove
  //   delay(100);
  // }
  USBSerial.println("ESP32-S3 is ready!");

  setupEEPROM();
  refreshDeviceData();
  upgradeDeviceRevisionInEEPROM();

  printBootMessage();
  setupBarometer();
  setupEEPROM();
  refreshDeviceData();
  upgradeDeviceRevisionInEEPROM();
  loadHardwareConfig();
  setupLED();
  setupAnalogRead();
  //initButtons();
  setupWatchdog();
  setup140();
  initBMSCAN();

  setLEDColor(LED_YELLOW);
  setupDisplay(deviceData, board_config);
  //initESC(0);
  //setESCThrottle(ESC_DISARMED_PWM);
  xTaskCreate(testTask, "TestTask", 10000, NULL, 1, &testTaskHandle);
  //xTaskCreate(telemetryEscTask, "telemetryEscTask", 4048, NULL, 2, &telemetryEscTaskHandle);
  xTaskCreate(updateDisplayTask, "updateDisplay", 2800, NULL, 1, &updateDisplayTaskHandle);
  //xTaskCreate(blinkLEDTask, "blinkLed", 400, NULL, 1, &blinkLEDTaskHandle);
  //xTaskCreate(throttleTask, "throttle", 4000, NULL, 3, &throttleTaskHandle);
  //xTaskCreate(telemetryBmsTask, "telemetryBmsTask", 8000, NULL, 3, &telemetryBmsTaskHandle);
  xTaskCreatePinnedToCore(spiCommunicationTask, "SPIComm", 4096, NULL, 5, &spiCommunicationTaskHandle, 1);

}

#else
/**
 * Initializes the necessary components and configurations for the device setup.
 * This function is called once at the beginning of the program execution.
 */

void setup() {
  #ifdef USE_TINYUSB
    setupUSBWeb();
  #endif
    printBootMessage();
    setupBarometer();
    setupEEPROM();
    refreshDeviceData();
    upgradeDeviceRevisionInEEPROM();
    loadHardwareConfig();
    setupLED();
    setupAnalogRead();
    initButtons();
    setupTasks();
    setupWatchdog();
    setup140();
  #ifdef M0_PIO
    Watchdog.reset();
  #endif
    setLEDColor(LED_YELLOW);
    setupDisplay(deviceData, board_config);
    if (button_top->isPressedRaw()) {
      modeSwitch(false);
    }
    vTaskResume(updateDisplayTaskHandle);
    setLEDColor(LED_GREEN);
}
#endif

// set up all the main threads/tasks with core 0 affinity
void setupTasks() {
  #ifdef RP_PIO
  xTaskCreateAffinitySet(blinkLEDTask, "blinkLed", 200, NULL, 1, uxCoreAffinityMask1, &blinkLEDTaskHandle);
  xTaskCreateAffinitySet(throttleTask, "throttle", 2048, NULL, 4, uxCoreAffinityMask0, &throttleTaskHandle);
  xTaskCreateAffinitySet(telemetryEscTask, "telemetryEscTask", 4048, NULL, 3, uxCoreAffinityMask0, &telemetryEscTaskHandle);
  xTaskCreateAffinitySet(trackPowerTask, "trackPower", 500, NULL, 2, uxCoreAffinityMask0, &trackPowerTaskHandle);
  xTaskCreateAffinitySet(updateDisplayTask, "updateDisplay", 2000, NULL, 1, uxCoreAffinityMask0, &updateDisplayTaskHandle);
  xTaskCreateAffinitySet(watchdogTask, "watchdog", 1000, NULL, 5, uxCoreAffinityMask0, &watchdogTaskHandle);
 #else
  xTaskCreate(blinkLEDTask, "blinkLed", 400, NULL, 1, &blinkLEDTaskHandle);
  xTaskCreate(throttleTask, "throttle", 1000, NULL, 3, &throttleTaskHandle);
  xTaskCreate(telemetryEscTask, "telemetryEscTask", 4048, NULL, 2, &telemetryEscTaskHandle);
  xTaskCreate(trackPowerTask, "trackPower", 500, NULL, 2, &trackPowerTaskHandle);
  //xTaskCreate(updateDisplayTask, "updateDisplay", 2600, NULL, 1, &updateDisplayTaskHandle);
  xTaskCreatePinnedToCore(watchdogTask, "watchdog", 1000, NULL, 5, &watchdogTaskHandle, 0);  // Run on core 0
  ESP_ERROR_CHECK(esp_task_wdt_add(watchdogTaskHandle));
 #endif
  if (updateDisplayTaskHandle != NULL) {
    vTaskSuspend(updateDisplayTaskHandle);  // Suspend the task immediately after creation
  }

  eepromSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(eepromSemaphore);
  stateMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(spiCommunicationTask, "SPIComm", 4096, NULL, 5, &spiCommunicationTaskHandle, 1);
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

// #ifdef M0_PIO
//   Watchdog.reset(); // reset the watchdog timer (done in task for RP2040)
// #endif

//   // from WebUSB to both Serial & webUSB
// #ifdef USE_TINYUSB
//   if (currentState == DISARMED && usb_web.available()) parse_usb_serial();
// #endif

  // more stable in main loop
  //checkButtons();
  delay(50);
  //USBSerial.println("loop");
}

void checkButtons() {
  button_top->check();
}

void printTime(const char* label) {
  Serial.print(label);
  Serial.println(millis());
}

void disarmESC() {
  #ifdef CAN_PIO
  // TODO: write to CAN bus
  #else
    setESCThrottle(ESC_DISARMED_PWM);
  #endif
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

  delay(500);  // TODO: just disable button thread // dont allow immediate rearming
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
      vTaskSuspend(updateDisplayTaskHandle);
      displayMeta(deviceData);
      vTaskResume(updateDisplayTaskHandle);
      break;
  }
}

// Variable to track if a button was clicked
bool wasClicked = false;

// Timestamp when the button was released
unsigned long releaseTime = 0;

// Time threshold for LongClick after release (in milliseconds)
const unsigned long longClickThreshold = 3500; // adjust as necessary

void handleButtonEvent(AceButton* btn, uint8_t eventType, uint8_t /* st */) {
  switch (eventType) {
  case AceButton::kEventClicked:
    wasClicked = true;
    break;
  case AceButton::kEventReleased:
    if (wasClicked) {
      releaseTime = millis();
      wasClicked = false;
      //Serial.println("Button Released after Click");
    }
    break;
  case AceButton::kEventDoubleClicked:
    break;
  case AceButton::kEventLongPressed:
    if (!wasClicked && (millis() - releaseTime <= longClickThreshold)) {
      toggleArm(); //
      //Serial.println("Long Press after Click and Release");
    } else {
      toggleCruise();
      //Serial.println("Long Press");
    }
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

// read throttle and send to hub
// read throttle
void handleThrottle() {
  //  TODO: remove
  //if (currentState == DISARMED) return;  // safe

  //armedSecs = (millis() - armedAtMillis) / 1000;  // update time while armed

  static int maxPWM = ESC_MAX_PWM;

  pot->update();
  int potRaw = pot->getValue();

  int localThrottlePWM = ESC_DISARMED_PWM;

  if (currentState == ARMED_CRUISING) {
    unsigned long cruisingSecs = (millis() - cruisedAtMillis) / 1000;

    if (cruisingSecs >= CRUISE_GRACE && potRaw > POT_ENGAGEMENT_LEVEL) {
      changeDeviceState(ARMED);  // deactivate cruise
    } else {
      localThrottlePWM = mapd(cruisedPotVal, 0, POT_MAX_VALUE, ESC_MIN_PWM, maxPWM);
    }
  } else {
    // no need to save & smooth throttle etc when in cruise mode (& pot == 0)
    potBuffer.push(potRaw);

    int potLvl = averagePotBuffer();

  // runs ~40x sec
  // 1000 diff in pwm from 0
  // 1000/6/40
    if (deviceData.performance_mode == 0) {  // chill mode
      potLvl = limitedThrottle(potLvl, prevPotLvl, 50);
      maxPWM = 1850;  // 85% interpolated from 1030 to 1990
    } else {
      potLvl = limitedThrottle(potLvl, prevPotLvl, 120);
      maxPWM = ESC_MAX_PWM;
    }
    // mapping val to min and max pwm
    localThrottlePWM = mapd(potLvl, 0, 4095, ESC_MIN_PWM, maxPWM);
  }
#ifdef CAN_PIO
  USBSerial.print("Setting throttle to: ");
  USBSerial.println(localThrottlePWM);
  setESCThrottle(localThrottlePWM);  // using val as the signal to esc
#else
  setESCThrottle(localThrottlePWM);  // using val as the signal to esc
#endif
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

  #ifdef CAN_PIO
  // TODO: write to CAN bus
  #else
    setESCThrottle(ESC_DISARMED_PWM);  // initialize the signal to low
  #endif

  //ledBlinkThread.enabled = false;
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
