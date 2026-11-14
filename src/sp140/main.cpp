// Copyright 2020 <Zach Whitehead>
// OpenPPG
#include "platform/runtime.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "sp140/device_settings.h"
#include "sp140/time_utils.h"
#include "sp140/ble/ota_service.h"
#include "esp_ota_ops.h"

#include <cmath>

#include "../../inc/sp140/esp32s3-config.h"
#include "../../inc/sp140/structs.h"  // data structs
#include "../../inc/sp140/utilities.h"
#include "platform/ring_buffer.h"  // smooth out readings

// ESP32S3 (CAN) specific libraries here
#include "esp_task_wdt.h"

#include "../../inc/sp140/alert_display.h"
#include "../../inc/sp140/altimeter.h"
#include "../../inc/sp140/system_monitors.h"
#include "../../inc/sp140/ble.h"
#include "../../inc/sp140/ble/ble_core.h"
#include "../../inc/sp140/ble/ble_utils.h"
#include "../../inc/sp140/ble/config_service.h"
#include "../../inc/sp140/ble/fastlink_service.h"
#include "../../inc/sp140/bms.h"
#include "../../inc/sp140/esc.h"
#include "../../inc/sp140/esc_config_relay.h"
#include "../../inc/sp140/esc_flasher_relay.h"
#include "../../inc/sp140/globals.h"  // device config
#include "../../inc/sp140/lvgl/lvgl_alerts.h"
#include "../../inc/sp140/lvgl/lvgl_core.h"
#include "../../inc/sp140/lvgl/lvgl_main_screen.h"
#include "../../inc/sp140/lvgl/lvgl_updates.h"
#include "../../inc/sp140/simple_monitor.h"
#include "../../inc/sp140/telemetry_hub.h"

#include "../../inc/sp140/buzzer.h"
#include "../../inc/sp140/device_state.h"
#include "../../inc/sp140/diagnostics.h"
#include "../../inc/sp140/led.h"
#include "../../inc/sp140/mode.h"
#include "../../inc/sp140/throttle.h"
#include "../../inc/sp140/vibration_pwm.h"
#include "sp140/espnow_bridge.h"
#include "sp140/wireless_settings.h"

// FreeRTOS task utilities for stack watermark logging
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#ifdef EPPG_QEMU
extern "C" void eppg_qemu_boot_complete();
#endif

// Forward declarations
void disarmSystem();
bool armSystem();
void afterCruiseEnd();
void setupTasks();
void createAllSyncPrimitives();
void audioTask(void* parameter);
void webSerialTask(void* parameter);
void toggleArm();
void toggleCruise();
void syncESCTelemetry();
uint16_t readControlThrottleRaw();
void processEspNowRemoteUpdates();
void processEspNowReceiverState();

// Global variable for shared SPI


// Store CS pins as globals to avoid accessing protected members
int8_t displayCS = -1;
int8_t bmsCS = MCP_CS;

#define BUTTON_DEBOUNCE_TIME_MS 50
#define FIRST_CLICK_MAX_HOLD_MS                                                \
  500  // Maximum time for first click to be considered a click
#define SECOND_HOLD_TIME_MS 2000  // How long to hold on second press to arm
#define CRUISE_HOLD_TIME_MS 2000
#define BUTTON_SEQUENCE_TIMEOUT_MS 1500  // Time window for arm/disarm sequence
#define PERFORMANCE_MODE_HOLD_MS 3000  // Longer hold time for performance mode
#define BLE_PAIRING_HOLD_MS 10000      // Hold duration to enter BLE pairing mode

// Throttle control constants (incl. CRUISE_MAX_PWM) are in inc/sp140/throttle.h
#define CRUISE_DISENGAGE_POT_THRESHOLD_PERCENTAGE                              \
  0.80  // Current pot must be >= this % of activation value to disengage
#define CRUISE_DISENGAGE_GRACE_PERIOD_MS                                       \
  2000  // Delay before checking pot disengagement after cruise activation

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

UnifiedBatteryData unifiedBatteryData = {0.0f, 0.0f, 0.0f, 0.0f};  // volts, amps, power, soc

// Throttle PWM smoothing buffer is managed in throttle.cpp


uint32_t led_color = STATUS_LED_RED;  // current LED color

// Global variable for device state
volatile DeviceState currentState = DISARMED;
volatile DeviceState wirelessRemoteDesiredState = DISARMED;

// A radio-loss disarm changes the control state synchronously on the 50 Hz
// throttle task. Slow feedback/NVS side effects are deferred to bleNotifyTask.
portMUX_TYPE wirelessDisarmMux = portMUX_INITIALIZER_UNLOCKED;
bool wirelessDisarmSideEffectsPending = false;

SemaphoreHandle_t stateMutex;
SemaphoreHandle_t lvglMutex;

// WebSerial worker
TaskHandle_t webSerialTaskHandle = NULL;

// Variable to track the current screen page
ScreenPage currentScreenPage = MAIN_SCREEN;

// Function to switch screen pages safely
void switchScreenPage(ScreenPage newPage) {
  if (xSemaphoreTake(lvglMutex, portMAX_DELAY) == pdTRUE) {
    currentScreenPage = newPage;
    // Potentially add code here to initialize the new screen if needed
    xSemaphoreGive(lvglMutex);
  } else {
    platform::console.line("Failed to acquire LVGL mutex for screen switch");
  }
}

// -----------------------------------------------
// Periodic FreeRTOS stack high-watermark logger
// -----------------------------------------------
// Forward declarations for task handles used by the logger
extern TaskHandle_t blinkLEDTaskHandle;
extern TaskHandle_t throttleTaskHandle;
extern TaskHandle_t watchdogTaskHandle;
extern TaskHandle_t uiTaskHandle;
extern TaskHandle_t bmsTaskHandle;
extern TaskHandle_t monitoringTaskHandle;
extern TaskHandle_t audioTaskHandle;
extern TaskHandle_t webSerialTaskHandle;
extern TaskHandle_t buttonTaskHandle;
extern TaskHandle_t vibeTaskHandle;

// Forward declarations for tasks defined in other compilation units
extern void vibeTask(void *parameter);
extern void buttonHandlerTask(void *parameter);

void changeDeviceState(DeviceState newState) {
  DeviceState oldState;

  // Fast critical section — update state, then release immediately. Side effects
  // (NVS writes, queue sends, melodies, I2C) run OUTSIDE the mutex to prevent
  // blocking other tasks that read currentState. BLE notify is handled in
  // bleNotifyTask.
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    // Recheck under the same authority lock used to reserve ESC maintenance.
    // This closes the check/arm race before a staging erase can begin.
    if (currentState == DISARMED && newState != DISARMED &&
        (isOtaInProgress() || escConfigRelayIsActive() || escFlasherRelayIsActive())) {
      xSemaphoreGive(stateMutex);
      return;
    }
    oldState = currentState;
    currentState = newState;

    xSemaphoreGive(stateMutex);
  } else {
    return;  // Should never happen with portMAX_DELAY
  }

  // Side effects outside the mutex
  platform::console.text("Device State Changed to: ");
  switch (newState) {
  case DISARMED:
    platform::console.line("DISARMED");
    break;
  case ARMED:
    platform::console.line("ARMED");
    break;
  case ARMED_CRUISING:
    platform::console.line("ARMED_CRUISING");
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
    // Cruise references are recorded in toggleCruise() before the state
    // change, and the throttle task latches its own output on entry, so
    // there is nothing left to do here.
    break;
  }
}

uint32_t armedAtMillis = 0;
uint32_t cruisedAtMillisMilis = 0;
unsigned long armedSecs = 0;

TaskHandle_t blinkLEDTaskHandle = NULL;
TaskHandle_t throttleTaskHandle = NULL;
TaskHandle_t watchdogTaskHandle = NULL;
TaskHandle_t uiTaskHandle = NULL;
TaskHandle_t bmsTaskHandle = NULL;
TaskHandle_t vibeTaskHandle = NULL;       // Vibration motor task
TaskHandle_t monitoringTaskHandle = NULL;  // Sensor monitoring task
TaskHandle_t ctrlSensorTaskHandle = NULL;
TaskHandle_t bleNotifyTaskHandle = NULL;

QueueHandle_t melodyQueue = NULL;
TaskHandle_t audioTaskHandle = NULL;

QueueHandle_t throttleUpdateQueue = NULL;
QueueHandle_t vibeQueue = NULL;  // Vibration motor queue

// Output the throttle task last commanded while ARMED. Written only by the
// throttle task; read by toggleCruise() to gate cruise activation.
static volatile uint16_t armedOutputPwm = ESC_MIN_PWM;

unsigned long lastDisarmTime = 0;
const unsigned long DISARM_COOLDOWN = 500;  // 500ms cooldown

// Timestamps updated by core tasks for watchdog monitoring
volatile uint32_t lastThrottleRunMs = 0;
volatile uint32_t lastUiRunMs = 0;
volatile uint32_t lastBmsRunMs = 0;

#pragma message "Warning: OpenPPG software is in beta"

// Build date/time for logging
const char *buildDate = __DATE__ " " __TIME__;

// Initialize shared SPI and chip select pins
void setupSPI(const HardwareConfig &board_config) {
  platform::console.line("Setting up SPI bus");

  // Store CS pins
  displayCS = board_config.tft_cs;
  bmsCS = MCP_CS;

  // Configure both CS pins as outputs and set 1 (deselected)
  eppg_gpio_config(displayCS, true, false);
  eppg_gpio_write(displayCS, 1);

  eppg_gpio_config(bmsCS, true, false);
  eppg_gpio_write(bmsCS, 1);

  ESP_ERROR_CHECK(eppg_spi_bus_init(board_config.spi_mosi, board_config.spi_miso,
                                   board_config.spi_sclk));
}

void watchdogTask(void *parameter) {
#ifndef OPENPPG_DEBUG
  // Register this task with the hardware Task WDT; only this task feeds it
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
#endif
  for (;;) {
    diagnosticsRefreshHeartbeat();
#ifndef OPENPPG_DEBUG
    esp_task_wdt_reset();
#endif
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void blinkLEDTask(void *pvParameters) {
  (void)pvParameters;  // this is a standard idiom to avoid compiler warnings
                      // about unused parameters.

  for (;;) {
    blinkLED();                     // call blinkLED function
    vTaskDelay(pdMS_TO_TICKS(500));  // wait for 500ms
  }
  vTaskDelete(NULL);  // should never reach this
}

void throttleTask(void *pvParameters) {
  (void)pvParameters;  // this is a standard idiom to avoid compiler warnings
                      // about unused parameters.

#ifndef OPENPPG_DEBUG
  // Subscribe the safety-critical control loop to the hardware Task Watchdog so
  // a genuine stall of this 50 Hz loop reboots the device into a clean,
  // motor-idle state. Deliberately ONLY this task (and the watchdog task) are
  // watched — a stalled UI or BMS task must NOT reboot a flying device.
  esp_task_wdt_add(NULL);
#endif

  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t throttleTicks = pdMS_TO_TICKS(20);  // 50 Hz
  for (;;) {
    handleThrottle();
    lastThrottleRunMs = platform::millis32();
#ifndef OPENPPG_DEBUG
    esp_task_wdt_reset();  // control loop ran this cycle — pet the watchdog
#endif
    vTaskDelayUntil(&lastWake, throttleTicks);
  }
  vTaskDelete(NULL);  // should never reach this
}

// Single 25 Hz barometer producer. Publish one coherent sample to telemetry;
// UI and monitor consumers never start conversions. CPU temperature access
// remains throttled by getCachedCpuTemperature().
void ctrlSensorTask(void *pvParameters) {
  (void)pvParameters;
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t sensorTicks = pdMS_TO_TICKS(40);  // 25 Hz barometer producer

  for (;;) {
    const unsigned long now = platform::millis32();
    sampleBarometer(deviceData);
    const BarometerSnapshot sample = getBarometerSnapshot();
    const float alt = sample.altitude;
    const float bt = sample.temperature;
    const float bp = sample.pressure;
    const float vs = sample.verticalSpeed;
    float mt = getCachedCpuTemperature();
    uint16_t pr = wirelessIsReceiver()
        ? readControlThrottleRaw()
        : getLastThrottleRaw();
    telemetryHubWriteController(alt, bt, bp, vs, mt, pr, now);
    vTaskDelayUntil(&lastWake, sensorTicks);
  }
}

uint16_t readControlThrottleRaw() {
  if (!wirelessIsReceiver()) return readThrottleRaw();

  wireless::ControlPacket control = {};
  uint32_t age_ms = UINT32_MAX;
  if (!espNowGetLatestControl(&control, &age_ms) ||
      age_ms > ESPNOW_CONTROL_TIMEOUT_MS) {
    return 0;
  }
  return control.throttle_raw;
}

void forceImmediateWirelessDisarm(bool failsafe) {
  bool changed = false;
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    changed = currentState != DISARMED;
    currentState = DISARMED;
    xSemaphoreGive(stateMutex);
  }

  espNowSetFailsafeLatched(failsafe);
  if (changed) {
    portENTER_CRITICAL(&wirelessDisarmMux);
    wirelessDisarmSideEffectsPending = true;
    portEXIT_CRITICAL(&wirelessDisarmMux);
  }
}

bool takeWirelessDisarmSideEffects() {
  portENTER_CRITICAL(&wirelessDisarmMux);
  const bool pending = wirelessDisarmSideEffectsPending;
  wirelessDisarmSideEffectsPending = false;
  portEXIT_CRITICAL(&wirelessDisarmMux);
  return pending;
}

void processEspNowReceiverState() {
  if (!wirelessIsReceiver() || !espNowBridgeActive()) return;

  static uint32_t active_session = 0;
  static DeviceState last_requested = DISARMED;
  static bool ready_for_arm = false;

  wireless::ControlPacket control = {};
  uint32_t age_ms = UINT32_MAX;
  if (!espNowGetLatestControl(&control, &age_ms) ||
      age_ms > ESPNOW_CONTROL_TIMEOUT_MS) {
    ready_for_arm = false;
    return;
  }

  const DeviceState requested =
      static_cast<DeviceState>(control.requested_state);
  if (control.session_id != active_session) {
    active_session = control.session_id;
    last_requested = DISARMED;
    ready_for_arm = false;
  }

  deviceData.performance_mode = control.performance_mode;
  if (requested == DISARMED) {
    // An explicit, fresh DISARMED command is the only action that clears the
    // reboot/link-loss latch and permits a later rising arm request.
    ready_for_arm = true;
    last_requested = DISARMED;
    espNowSetFailsafeLatched(false);
    return;
  }

  if (!ready_for_arm || requested == last_requested) return;

  if (requested == ARMED) {
    if (currentState == DISARMED) {
      toggleArm();
    } else if (currentState == ARMED_CRUISING) {
      toggleCruise();
    }
  } else if (requested == ARMED_CRUISING && currentState == ARMED) {
    toggleCruise();
  }
  last_requested = requested;
}

void processEspNowRemoteUpdates() {
  if (!wirelessIsRemote() || !espNowBridgeActive()) return;

  static bool telemetry_marked_lost = false;
  BLE_FastLink_Telemetry telemetry = {};
  if (espNowTakeTelemetry(&telemetry)) {
    espNowApplyMirroredTelemetry(telemetry);
    telemetry_marked_lost = false;
  }

  wireless::AckPacket ack = {};
  if (espNowTakeAck(&ack)) {
    const DeviceState actual = static_cast<DeviceState>(ack.actual_state);
    if ((ack.flags & wireless::ACK_FAILSAFE_LATCHED) != 0) {
      wirelessRemoteDesiredState = DISARMED;
    } else if (actual != wirelessRemoteDesiredState) {
      // The receiver is authoritative (arm rejection, cruise override, etc.).
      wirelessRemoteDesiredState = actual;
    }
    if (actual != currentState) changeDeviceState(actual);
  }

  if (espNowRemoteStatusAgeMs() > ESPNOW_REMOTE_STATUS_TIMEOUT_MS) {
    wirelessRemoteDesiredState = DISARMED;
    if (currentState != DISARMED) changeDeviceState(DISARMED);
  }
  if (!telemetry_marked_lost && espNowRemoteTelemetryAgeMs() > 500) {
    espNowMarkMirroredTelemetryDisconnected();
    telemetry_marked_lost = true;
  }
}

void bleNotifyTask(void *pvParameters) {
  (void)pvParameters;

  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t notifyTicks = pdMS_TO_TICKS(20);  // 50 Hz

  // Mirror state changes to the dedicated device-state characteristic,
  // edge-triggered (fires only on an actual arm/disarm/cruise change).
  uint8_t lastNotifiedState = 0xFF;

  for (;;) {
    processEspNowRemoteUpdates();
    processEspNowReceiverState();
    if (takeWirelessDisarmSideEffects()) disarmSystem();

    TelemetryHub hub = {};
    if (telemetryHubRead(&hub, pdMS_TO_TICKS(2))) {
      publishFastLinkTelemetry(hub, currentState);
      if (wirelessIsReceiver()) {
        // ACK is deliberately queued first so command/state confirmation wins
        // if the single ESP-NOW transmit slot is busy.
        espNowSendAck(currentState);
        espNowSendTelemetry(buildFastLinkTelemetryPacket(hub, currentState));
      }
    }

    // Push ESC config-relay status + result blob over notify (config-service
    // GATT reads return null on this stack; notify is reliable).
    pumpEscRelayNotify();

    if (!isOtaInProgress()) {
      setAndNotifyOnChange(pDeviceStateCharacteristic,
                           static_cast<uint8_t>(currentState), lastNotifiedState);
    }

    // Check for stalled OTA updates
    checkOtaTimeout();

    vTaskDelayUntil(&lastWake, notifyTicks);
  }
}

void refreshDisplay() {
  // Guard against calls before main_screen is set up
  if (main_screen == NULL) {
    return;
  }

  // The sensor task owns periodic conversions. Display work only reads the
  // published snapshot and never waits for an I2C transaction.
  static float lastGoodAltitude = 0.0f;
  const float rawAltitude = getAltitude(deviceData);
  if (rawAltitude != __FLT_MIN__) {
    lastGoodAltitude = rawAltitude;
  }
  float currentRelativeAltitude =
      (rawAltitude != __FLT_MIN__) ? rawAltitude : lastGoodAltitude;

  // Atomic snapshot of ESC/BMS telemetry from the hub. Reading the live globals
  // here would race the throttle task, which writes escTelemetryData
  // field-by-field on the other core (torn read). A function-static snapshot
  // means a rare hub-mutex timeout simply reuses the previous frame's data —
  // this never blocks the UI task and never shows a half-updated struct.
  static TelemetryHub uiHub = {};
  telemetryHubRead(&uiHub, pdMS_TO_TICKS(5));
  if (wirelessIsRemote()) {
    currentRelativeAltitude = uiHub.altitude;
  }

  if (xSemaphoreTake(lvglMutex, pdMS_TO_TICKS(40)) == pdTRUE) {
    // Determine the altitude to show on the display
    float altitudeToShow = 0.0f;  // Default to 0
    if (currentState != DISARMED) {
      altitudeToShow =
          currentRelativeAltitude;  // Show relative altitude when armed/cruising
    }
    bool isArmed = (currentState != DISARMED);
    bool isCruising = (currentState == ARMED_CRUISING);

    // Select the appropriate screen update function based on the current page
    switch (currentScreenPage) {
    case MAIN_SCREEN:
      updateLvglMainScreen(deviceData, uiHub.esc, uiHub.bms,
                           unifiedBatteryData, altitudeToShow, isArmed,
                           isCruising, armedAtMillis);
      break;
    // Add cases for other screens here later
    // case SETTINGS_SCREEN:
    //   updateLvglSettingsScreen(...);
    //   break;
    default:
      // Handle unknown screen state if necessary
      break;
    }

    // Handle synchronized alert updates (counter + display together)
    AlertUIUpdate alertUpdate;
    if (xQueueReceive(alertUIQueue, &alertUpdate, 0) == pdTRUE) {
      // Update counter display
      updateAlertCounterDisplay(alertUpdate.counts);

      // Handle critical display (top, in altitude area)
      if (alertUpdate.showCritical) {
        lv_showAlertTextWithLevel(alertUpdate.criticalId,
                                  alertUpdate.criticalLevel, true);
      } else {
        lv_hideCriticalText();
      }

      // Handle warning display (below critical)
      if (alertUpdate.showWarning) {
        lv_showAlertTextWithLevel(alertUpdate.warningId,
                                  alertUpdate.warningLevel, false);
      } else {
        lv_hideWarningText();
      }

      // Control critical border and vibration based on state
      static bool lastCriticalState = false;
      if (alertUpdate.criticalAlertsActive != lastCriticalState) {
        if (alertUpdate.criticalAlertsActive) {
          platform::console.line("[UI] Starting critical border flash");
          startCriticalBorderFlashDirect();  // Direct control - we already have
                                            // the mutex
        } else {
          platform::console.line("[UI] Stopping critical border flash");
          stopCriticalBorderFlashDirect();  // Direct control - we already have
                                           // the mutex
        }
        lastCriticalState = alertUpdate.criticalAlertsActive;
      }
    }

    // General LVGL update handler
    updateLvgl();

    xSemaphoreGive(lvglMutex);
  } else {
    // Avoid spamming the serial log; skip this frame if mutex busy
  }
}

void monitoringTask(void *pvParameters) {
  for (;;) {
    TelemetryHub hub = {};
    if (telemetryHubRead(&hub, pdMS_TO_TICKS(5)) && monitoringEnabled) {
      checkAllSensorsWithData(hub.esc, hub.bms);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// UI task: fixed ~30 Hz refresh and snapshot publish
void uiTask(void *pvParameters) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t uiTicks = pdMS_TO_TICKS(33);  // ~30 Hz, matches LV_DEF_REFR_PERIOD
  for (;;) {
    refreshDisplay();
    lastUiRunMs = platform::millis32();
    vTaskDelayUntil(&lastWake, uiTicks);
  }
}

// BMS task: 20 Hz polling and unified battery update
void bmsTask(void *pvParameters) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t bmsTicks = pdMS_TO_TICKS(100);  // 10 Hz
  for (;;) {
    if (bmsCanInitialized) {
      updateBMSData();
      // Coherent-snapshot gate: report CONNECTED only after the pack-voltage
      // and cell-voltage frames have both arrived, so monitors and the UI
      // never see the half-populated data a fresh link starts with.
      //
      // Latched on purpose. This orders the first few frames at link-up and
      // has no legitimate work to do afterwards — re-evaluating it every cycle
      // would let a live-but-faulted pack (one collapsed cell) read as "no
      // data" and silently drop the BMS out of the alert system entirely.
      // Genuine link loss is still caught by isConnected() below.
      static bool bmsSnapshotEverCoherent = false;
      if (bms_can->isConnected(BMS_LINK_TIMEOUT_MS)) {
        if (!bmsSnapshotEverCoherent && bmsSnapshotCoherent(bmsTelemetryData)) {
          bmsSnapshotEverCoherent = true;
        }
        bmsTelemetryData.bmsState = bmsSnapshotEverCoherent
            ? TelemetryState::CONNECTED
            : TelemetryState::NOT_CONNECTED;
      } else {
        bmsTelemetryData.bmsState = TelemetryState::NOT_CONNECTED;
      }
    } else {
      bmsTelemetryData.bmsState = TelemetryState::NOT_CONNECTED;
    }
    telemetryHubWriteBms(bmsTelemetryData);

    if (bmsTelemetryData.bmsState == TelemetryState::CONNECTED) {
      unifiedBatteryData.volts = bmsTelemetryData.battery_voltage;
      unifiedBatteryData.amps = bmsTelemetryData.battery_current;
      unifiedBatteryData.soc = bmsTelemetryData.soc;
      unifiedBatteryData.power = bmsTelemetryData.power;
    } else if (escTelemetryData.escState == TelemetryState::CONNECTED) {
      unifiedBatteryData.volts = escTelemetryData.volts;
      unifiedBatteryData.amps = escTelemetryData.amps;
      unifiedBatteryData.power =
          escTelemetryData.amps * escTelemetryData.volts / 1000.0;
      unifiedBatteryData.soc = 0.0;
    } else {
      unifiedBatteryData.volts = 0.0;
      unifiedBatteryData.amps = 0.0;
      unifiedBatteryData.soc = 0.0;
      unifiedBatteryData.power = 0.0;
    }

    lastBmsRunMs = platform::millis32();
    vTaskDelayUntil(&lastWake, bmsTicks);
  }
}

void loadHardwareConfig() {
  board_config = s3_config;  // ESP32S3 is only supported board

  // Throttle input is initialized via initThrottleInput()
}

void printBootMessage() {
  platform::console.text("Booting up V");
  platform::console.text(VERSION_STRING);
  platform::console.text(" git:");
  platform::console.line(GIT_REV);
}

void setupBarometer() {
  const int bmp_enabler = 9;
  eppg_gpio_config(bmp_enabler, true, false);
  eppg_gpio_write(bmp_enabler, 1);  // barometer fix for V2 board
}


void setupLED() {
  eppg_gpio_config(board_config.led_sw, true, false);  // set up the internal LED2 pin
  if (board_config.enable_neopixel) {
    setLEDColor(led_color);
  }
}

void setupAnalogRead() {
  // Initialize throttle ADC input (12-bit range on ESP32)
  initThrottleInput();
}

// Hardware Task Watchdog timeout, in SECONDS. NOTE: esp_task_wdt_init() takes
// seconds, not milliseconds — the previous value of 3000 was a ~50 minute
// timeout that effectively disabled the watchdog. 5 s is long enough that
// normal operation (boot splash, BLE/flash work) can never trip it, yet short
// enough to recover a genuine firmware hang. By the time it fires, the ESC's
// own ~300 ms throttle-loss failsafe has already idled the motor.
#define WDT_TIMEOUT_SECONDS 5

void setupWatchdog() {
#ifndef OPENPPG_DEBUG
  // Initialize Task Watchdog (reboot on timeout). The idle-task checks on both
  // cores are deliberately DISABLED in sdkconfig
  // (CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0/CPU1=n) so the watchdog scope is
  // exactly the tasks that explicitly subscribe (throttleTask + watchdogTask).
  // Otherwise any non-safety task (UI/BMS/BLE) monopolizing a core for >5 s
  // would panic-reboot a flying device — the opposite of the intent.
  esp_task_wdt_config_t config{};
  config.timeout_ms = WDT_TIMEOUT_SECONDS * 1000;
  config.idle_core_mask = 0;
  config.trigger_panic = true;
  ESP_ERROR_CHECK(esp_task_wdt_init(&config));
#endif  // OPENPPG_DEBUG
}

#define TAG "OpenPPG"

/**
 * Phase 2a: Create all RTOS mutexes and semaphores
 * Must be called before any tasks are created
 */
void createAllSyncPrimitives() {
  platform::console.line("Creating sync primitives...");

  // LVGL display mutex - protects all LVGL operations
  lvglMutex = xSemaphoreCreateMutex();
  if (lvglMutex == NULL) {
    platform::console.line("Error creating LVGL mutex");
  }

  // State mutex - protects device state transitions
  stateMutex = xSemaphoreCreateMutex();
  if (stateMutex == NULL) {
    platform::console.line("Error creating state mutex");
  }

  // Create all queues
  throttleUpdateQueue = xQueueCreate(1, sizeof(uint16_t));
  if (throttleUpdateQueue == NULL) {
    platform::console.line("Error creating throttle update queue");
  }

  // Melody queue for audio task - MUST be created before audioTask
  melodyQueue = xQueueCreate(5, sizeof(MelodyRequest));
  if (melodyQueue == NULL) {
    platform::console.line("Error creating melody queue");
  }

  // Telemetry hub mutex - protects shared telemetry data between
  // sensor-producer tasks (ESC/BMS/controller) and the BLE notify task.
  telemetryHubInit();
}

/**
 * Phase 6: Create all FreeRTOS tasks
 * Called AFTER all hardware, UI, and services are initialized
 * Tasks can start running immediately - no sync semaphores needed
 */
void setupTasks() {
  xTaskCreate(blinkLEDTask, "blinkLed", 2560, NULL, 1, &blinkLEDTaskHandle);
  xTaskCreatePinnedToCore(throttleTask, "throttle", 4352, NULL, 3,
                          &throttleTaskHandle, 0);
  xTaskCreatePinnedToCore(uiTask, "UI", 5888, NULL, 2, &uiTaskHandle, 1);
  if (!wirelessIsRemote()) {
    // Native SPI transactions and formatted diagnostics need more headroom
    // than the former wrapper. The instruction-level scenario exercises this
    // path continuously, including the modeled controller register accesses.
    xTaskCreatePinnedToCore(bmsTask, "BMS", 4096, NULL, 2, &bmsTaskHandle, 1);
    xTaskCreate(ctrlSensorTask, "CtrlSensor", 4096, NULL, 2,
                &ctrlSensorTaskHandle);
  }
  // 8 KB stack: bleNotifyTask calls notify() (deep NimBLE GATT/ATT/L2CAP/HCI
  // path) for telemetry and device-state; a smaller stack overflows on notify.
  xTaskCreatePinnedToCore(bleNotifyTask, "BLENotify", 8192, NULL, 1,
                          &bleNotifyTaskHandle, 1);
  xTaskCreatePinnedToCore(buttonHandlerTask, "ButtonHandler", 4096, NULL, 2,
                          &buttonTaskHandle, 0);
  xTaskCreatePinnedToCore(audioTask, "Audio", 1536, NULL, 1, &audioTaskHandle,
                          1);
  xTaskCreatePinnedToCore(watchdogTask, "watchdog", 1536, NULL, 3,
                          &watchdogTaskHandle, 0);
  xTaskCreate(webSerialTask, "WebSerial",
              6144,  // Diagnostics JSON serialization needs more stack headroom
              NULL, 1, &webSerialTaskHandle);
  xTaskCreate(monitoringTask, "Monitoring", 4864, NULL, 1,
              &monitoringTaskHandle);
  xTaskCreatePinnedToCore(vibeTask, "Vibration", 1536, NULL, 2, &vibeTaskHandle,
                          1);
}

// =============================================================================
// MAIN SETUP FUNCTION - PHASED BOOT SEQUENCE
// =============================================================================

/**
 * Initializes the device in deterministic phases.
 * No tasks are created until all their dependencies exist.
 */
extern "C" void app_main() {
  ESP_ERROR_CHECK(nvs_flash_init());
  // =========================================================================
  // PHASE 1: Serial and Early Config
  // =========================================================================
  platform::console.init();
  platform::console.text("Build date/time: ");
  platform::console.line(buildDate);
  diagnosticsInit();

  // Load device config from EEPROM first - may contain pin mappings
  refreshDeviceData();
  loadWirelessSettings();
  printBootMessage();

  // Log OTA partition info
  const esp_partition_t* running = esp_ota_get_running_partition();
  const esp_partition_t* boot = esp_ota_get_boot_partition();
  if (running != nullptr && boot != nullptr) {
    platform::console.format("OTA: Running partition %s @ 0x%06lx, boot %s @ 0x%06lx\n",
                     running->label, (unsigned long)running->address,
                     boot->label, (unsigned long)boot->address);
  }

  // =========================================================================
  // PHASE 2: Create ALL RTOS Primitives (before any tasks)
  // =========================================================================
  createAllSyncPrimitives();

  // =========================================================================
  // PHASE 3: Hardware Initialization
  // =========================================================================
  platform::console.line("Initializing hardware...");

  // Temporary fix for BMP3xx I2C mode - TODO remove
  eppg_gpio_write(42, 1);
  eppg_gpio_config(42, true, false);

  // Load board-specific hardware config
  loadHardwareConfig();

  // I2C bus setup for altimeter
  const int SDA_PIN = 44;
  const int SCL_PIN = 41;
  ESP_ERROR_CHECK(eppg_i2c_init(SDA_PIN, SCL_PIN));
  if (!setupAltimeter()) {
    platform::console.line("Error initializing BMP3xx barometer");
  }

  setupLED();  // Defaults to RED
  setupAnalogRead();
  eppg_gpio_config(board_config.button_top, false, true);  // Button GPIO only, task created later
  setupWatchdog();

  // Buzzer and vibration motor
  if (ENABLE_BUZZ) {
    initBuzz();
  }
  if (ENABLE_VIBE) {
    initVibeMotor();
  }

  setLEDColor(STATUS_LED_YELLOW);  // Indicate boot in progress

  // SPI bus (shared between display and BMS CAN)
  setupSPI(board_config);

  // Display initialization
  setupLvglDisplay(deviceData, board_config.tft_dc, board_config.tft_rst);
  initAlertDisplay();

  // The handheld remote mirrors both CAN buses over ESP-NOW and must never
  // initialize or transmit on its unused local ESC/BMS interfaces.
  if (!wirelessIsRemote()) {
    // BMS CAN interface
#ifndef SCREEN_DEBUG
    initBMSCAN();
#endif

    // ESC interface
    initESC();
    setESCThrottle(ESC_DISARMED_PWM);
  }

  // =========================================================================
  // PHASE 4: UI Startup (blocking - no tasks running yet)
  // =========================================================================
  platform::console.line("Starting UI...");

  // Haptic feedback that boot is progressing
  pulseVibeMotor();

  // Check for performance mode activation (button held during boot)
  if (eppg_gpio_read(board_config.button_top) == 0) {
    perfModeSwitch();
  }

  setLEDColor(STATUS_LED_GREEN);

  // Show splash screen (blocking)
  lv_obj_t* splash_screen = NULL;
  if (xSemaphoreTake(lvglMutex, portMAX_DELAY) == pdTRUE) {
    displayLvglSplash(deviceData, 2000);
    splash_screen = lv_screen_active();
    // Keep mutex held through main screen setup
  } else {
    platform::console.line("Failed to acquire LVGL mutex for splash");
  }

  // Setup main screen after splash
  platform::console.line("Setting up main screen...");
  if (main_screen == NULL) {
    setupMainScreen(deviceData.theme == 1);
  }

  // Force the first main-screen repaint before other tasks can contend for SPI.
  if (main_screen != NULL) {
    lv_obj_invalidate(main_screen);
    lv_refr_now(main_display);
    if (splash_screen != NULL && splash_screen != main_screen) {
      lv_obj_delete(splash_screen);
      splash_screen = NULL;
    }
    platform::console.line("Main screen loaded");
  } else {
    platform::console.line("Error: Main screen object is NULL after setup attempt");
  }

  // Release LVGL mutex
  if (lvglMutex != NULL && xSemaphoreGetMutexHolder(lvglMutex) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(lvglMutex);
  }

  // =========================================================================
  // PHASE 5: Services Initialization
  // =========================================================================
  platform::console.line("Initializing services...");

  // QEMU executes real control/tasks and modeled peripherals. The S3 machine
  // does not emulate the RF hardware; native radio runs only on the board.
#ifndef EPPG_QEMU
  setupBLE();

  if (wirelessSettings.role != WirelessRole::STANDALONE) {
    beginEspNowBridge();
  }
#else
  platform::console.line("[QEMU] BLE and Wi-Fi RF unavailable; peripheral/control test build");
#endif

  // Monitoring system (disabled initially)
  initSimpleMonitor();

  // Send initial device data
  send_device_data();

  // =========================================================================
  // PHASE 6: Create ALL Tasks
  // Tasks can start immediately - all dependencies exist
  // =========================================================================
  setupTasks();

  // =========================================================================
  // PHASE 7: Start External Interfaces
  // =========================================================================
  platform::console.line("Starting external interfaces...");

  // Start BLE advertising (now safe - UI is ready, tasks are running)
#ifndef EPPG_QEMU
  restartBLEAdvertising();
  platform::console.line("BLE services initialized");
#endif

  // Enable sensor monitoring
  enableMonitoring();

  // Check if this is first boot after OTA update and validate if needed
  esp_ota_img_states_t ota_state;
  if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
    if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
      // First boot after OTA - if we made it this far, firmware is good
      platform::console.line("OTA: First boot after update - validating firmware");
      esp_ota_mark_app_valid_cancel_rollback();
      platform::console.line("OTA: Firmware marked valid - Rollback protection active");
    }
  }

  platform::console.line("=== Boot complete ===");
#ifdef EPPG_QEMU
  eppg_qemu_boot_complete();
#endif
}

// Add new button handler task
void buttonHandlerTask(void *parameter) {
  uint32_t lastDebounceTime = 0;
  bool lastButtonState = 1;
  bool buttonState;
  bool pairingHoldHandled = false;

  while (true) {
    buttonState = eppg_gpio_read(board_config.button_top);
    uint32_t currentTime = platform::millis32();

    // Debounce
    if ((currentTime - lastDebounceTime) > BUTTON_DEBOUNCE_TIME_MS) {
      if (buttonState != lastButtonState) {
        lastDebounceTime = currentTime;

        if (buttonState == 0) {  // Button pressed
          buttonPressed = true;
          pairingHoldHandled = false;
          buttonPressStartTime = currentTime;
          platform::console.line("Button pressed");
          platform::console.text("Arm sequence state: ");
          platform::console.line(armSequenceStarted ? "ACTIVE" : "INACTIVE");
        } else {  // Button released
          buttonPressed = false;
          buttonReleaseStartTime = currentTime;

          uint32_t holdDuration = buttonReleaseStartTime - buttonPressStartTime;
          platform::console.text("Button released after ");
          platform::console.text(holdDuration);
          platform::console.line("ms");

          // Disarmed long hold toggles performance mode on release.
          if (currentState == DISARMED &&
              holdDuration >= PERFORMANCE_MODE_HOLD_MS &&
              holdDuration < BLE_PAIRING_HOLD_MS &&
              !pairingHoldHandled) {
            perfModeSwitch();
            lastButtonState = buttonState;
            continue;
          }

          // Only start arm sequence if it was a quick click
          if (!armSequenceStarted && holdDuration < FIRST_CLICK_MAX_HOLD_MS) {
            armSequenceStarted = true;
            platform::console.line("Quick click detected - arm sequence started");
          }
        }

        lastButtonState = buttonState;
      }

      // Handle arm sequence first
      if (armSequenceStarted) {
        // Only check for timeout if we're waiting for the second press
        // (button is not currently pressed)
        if (!buttonPressed && (currentTime - buttonReleaseStartTime) >
                                  BUTTON_SEQUENCE_TIMEOUT_MS) {
          armSequenceStarted = false;
          platform::console.line("Arm sequence timed out - waiting too long between "
                            "click and hold");
        }

        // If button is pressed, check for arm completion
        if (buttonPressed) {
          uint32_t currentHoldTime = currentTime - buttonPressStartTime;

          // Complete arm sequence if held long enough
          if (currentHoldTime >= SECOND_HOLD_TIME_MS) {
            platform::console.line("Arm sequence complete - toggling arm state");
            toggleArm();
            armSequenceStarted = false;
            buttonPressed = false;
            buttonPressStartTime =
                currentTime;  // Reset to prevent immediate cruise activation
          }
        }
      } else if (buttonPressed) {  // Only handle other button actions if we're not in an arm sequence
        uint32_t currentHoldTime = currentTime - buttonPressStartTime;

        // Long hold while disarmed: enter BLE pairing mode (clears bonds).
        if (currentState == DISARMED &&
            currentHoldTime >= BLE_PAIRING_HOLD_MS && !pairingHoldHandled) {
          enterBLEPairingMode();
          pulseVibeMotor();
          platform::console.line("[BLE] Pairing mode activated via button hold");
          pairingHoldHandled = true;
        }

        // Handle cruise control (when armed or cruising and held long enough)
        if ((currentState == ARMED || currentState == ARMED_CRUISING) &&
            currentHoldTime >= CRUISE_HOLD_TIME_MS) {
          platform::console.line("Cruise control button activated");
          toggleCruise();
          buttonPressed = false;
          buttonPressStartTime = currentTime;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void printTime(const char *label) {
  platform::console.text(label);
  platform::console.line(platform::millis32());
}

void disarmESC() {
  // Throttle task sends ESC_DISARMED_PWM on its next 20ms cycle
  // when it sees DISARMED state — no direct CAN write needed here
  // to avoid race condition with throttle task's canard instance
}

// reset smoothing
void resetSmoothing() { throttleFilterClear(); }

void resumeLEDTask() {
  vTaskResume(blinkLEDTaskHandle);  // blink LED while disarmed
}

void runDisarmAlert() {
  u_int16_t disarm_melody[] = {1568, 1047};
  playMelody(disarm_melody, 2);
  queueEscMotorBeepDisarm();
  pulseVibeMotor();
}

void updateArmedTime() {
  uint16_t newArmedTime = round(armedSecs / 60);
  if (newArmedTime <= UINT16_MAX - deviceData.armed_time) {
    deviceData.armed_time += newArmedTime;
  } else {
    // If adding the new time would cause an overflow, set the value to the
    // maximum
    deviceData.armed_time = UINT16_MAX;
  }
}

void disarmSystem() {
  disarmESC();
  resetSmoothing();
  resumeLEDTask();
  runDisarmAlert();

  // Calculate elapsed armed time in seconds before updating hour meter
  if (armedAtMillis > 0) {
    unsigned long currentMillis = platform::millis32();
    armedSecs = (currentMillis - armedAtMillis) / 1000;
  }

  updateArmedTime();
  writeDeviceData();

  // DISARM_COOLDOWN (500ms, checked in toggleArm) prevents immediate re-arm —
  // no need to block this task for 500ms.
  lastDisarmTime = platform::millis32();
}

void handleArmFail() {
  startArmFailIconFlash();
  handleArmFailMelody();
  pulseVibeMotor();
}

void toggleArm() {
  if (wirelessIsRemote()) {
    if (currentState == DISARMED &&
        wirelessRemoteDesiredState == DISARMED) {
      if (isOtaInProgress()) {
        platform::console.line("Remote arm blocked: OTA update in progress");
      } else if (readThrottleRaw() < POT_ENGAGEMENT_LEVEL) {
        wirelessRemoteDesiredState = ARMED;
      } else {
        handleArmFail();
      }
    } else {
      // Disarm is always accepted locally and transmitted as a repeated level.
      wirelessRemoteDesiredState = DISARMED;
    }
    return;
  }

  if (currentState == DISARMED) {
    if (isOtaInProgress()) {
      platform::console.line("Arm blocked: OTA update in progress");
      return;
    }

    // Block arming while an ESC config-relay or firmware-relay session is in
    // flight (the ESC may be mid-reboot / in the bootloader). Mirrors the OTA
    // interlock above.
    if (escConfigRelayIsActive() || escFlasherRelayIsActive()) {
      platform::console.line("Arm blocked: ESC config/firmware relay in progress");
      return;
    }

    // Check if enough time has passed since last disarm
    if (platform::millis32() - lastDisarmTime >= DISARM_COOLDOWN) {
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
  if (wirelessIsRemote()) {
    if (currentState == ARMED) {
      const uint16_t pot = readThrottleRaw();
      // The receiver enforces its actual output cap; reject a remote request
      // here as well when its mode-mapped target is already above that cap.
      if (pot >= POT_ENGAGEMENT_LEVEL &&
          isCruiseOutputAllowed(potRawToModePwm(pot, deviceData.performance_mode))) {
        wirelessRemoteDesiredState = ARMED_CRUISING;
      } else {
        startCruiseIconFlash();
      }
    } else if (currentState == ARMED_CRUISING) {
      wirelessRemoteDesiredState = ARMED;
    }
    return;
  }

  switch (currentState) {
  case ARMED:
    if (isOtaInProgress()) {
      platform::console.line("Cruise blocked: OTA update in progress");
      break;
    }

    // Check if throttle is engaged (not at zero)
    if (throttleEngaged()) {
      // Hold the actual output; refuse activation above the shared cruise cap.
      if (!isCruiseOutputAllowed(armedOutputPwm)) {
        startCruiseIconFlash();
        platform::console.line("Cruise blocked: output above cruise max");
      } else {
        // Record the override reference before the state flips: the throttle
        // task checks it from its first ARMED_CRUISING tick.
        cruisedPotVal = readControlThrottleRaw();
        cruisedAtMillis = platform::millis32();
        changeDeviceState(ARMED_CRUISING);
        pulseVibeMotor();
      }
    } else {
      // Throttle not engaged enough to set level, flash the icon
      startCruiseIconFlash();
      platform::console.line("Cruise activation failed: Throttle not engaged.");
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
  return readControlThrottleRaw() < threshold;
}

/**
 * Checks if cruise control should be disengaged based on potentiometer override
 * @param potVal Current raw potentiometer value
 * @return true if cruise should be disengaged, false otherwise
 */
bool shouldDisengageCruise(int potVal) {
  unsigned long timeSinceCruiseStart = platform::millis32() - cruisedAtMillis;

  // Only check for disengagement *after* the grace period has passed
  if (timeSinceCruiseStart > CRUISE_DISENGAGE_GRACE_PERIOD_MS) {
    // Calculate the disengagement threshold based on the *raw potentiometer
    // value* when cruise was engaged
    int disengageThresholdPotVal =
        (int)(cruisedPotVal * CRUISE_DISENGAGE_POT_THRESHOLD_PERCENTAGE);

    // If the *current raw potentiometer value* is greater than or equal to the
    // threshold
    if (potVal >= disengageThresholdPotVal) {
      platform::console.text("Cruise override: potVal ");
      platform::console.text(potVal);
      platform::console.text(" >= threshold ");
      platform::console.line(disengageThresholdPotVal);
      return true;
    }
  }
  return false;
}

/**
 * Handles throttle control when in ARMED_CRUISING state
 * @param potVal Current raw potentiometer value
 */
void handleCruisingThrottle(int potVal) {
  // Check for cruise disengagement via potentiometer override
  if (shouldDisengageCruise(potVal)) {
    changeDeviceState(ARMED);  // Transition back to normal ARMED state
    pulseVibeMotor();
  }
}

// (moved throttling logic into throttle.cpp helpers)

/**
 * Main throttle handling function - processes potentiometer input and controls
 * ESC based on device state Called every 20ms from throttleTask
 *
 * States handled:
 * - DISARMED: ESC off, reset smoothing buffers
 * - ARMED_CRUISING: Maintain cruise speed, check for potentiometer override
 * - ARMED: Normal throttle control with ramping and performance mode limits
 */
void handleThrottle() {
  static uint16_t currentCruiseThrottlePWM = ESC_MIN_PWM;
  static int prevPwm = ESC_MIN_PWM;  // Previous PWM for ramping
  static bool wasCruising = false;
  uint16_t newPWM;

  if (wirelessIsRemote()) {
    const uint16_t local_throttle = readThrottleRaw();
    espNowSendControl(local_throttle, wirelessRemoteDesiredState,
                      deviceData.performance_mode);
    return;
  }

  uint16_t control_throttle = readControlThrottleRaw();
  if (wirelessIsReceiver()) {
    wireless::ControlPacket control = {};
    uint32_t age_ms = UINT32_MAX;
    const bool fresh = espNowGetLatestControl(&control, &age_ms) &&
                       age_ms <= ESPNOW_CONTROL_TIMEOUT_MS;
    if (!fresh) {
      forceImmediateWirelessDisarm(true);
      control_throttle = 0;
    } else if (control.requested_state == DISARMED) {
      forceImmediateWirelessDisarm(false);
      control_throttle = control.throttle_raw;
    }
  }

  // Sample state once so the latch and output agree across task boundaries.
  const DeviceState state = currentState;
  const bool cruising = state == ARMED_CRUISING;
  if (cruising && !wasCruising) {
    currentCruiseThrottlePWM = std::min<int>(prevPwm, CRUISE_MAX_PWM);
    platform::console.format("Cruise PWM latched at: %u\n", currentCruiseThrottlePWM);
  }
  wasCruising = cruising;

  // Apply BLE cruise setpoint updates after the entry latch.
  if (xQueueReceive(throttleUpdateQueue, &newPWM, 0) == pdTRUE) {
    if (state == ARMED_CRUISING) {
      currentCruiseThrottlePWM = newPWM;
      platform::console.text("Cruise PWM initialized/updated to: ");
      platform::console.line(currentCruiseThrottlePWM);
    }
  }

  int finalPwm = ESC_DISARMED_PWM;

  // Handle throttle based on current device state
  switch (state) {
  case DISARMED:
    resetThrottleState(prevPwm);
    armedOutputPwm = ESC_MIN_PWM;
    finalPwm = ESC_DISARMED_PWM;
    break;

  case ARMED_CRUISING:
    handleCruisingThrottle(control_throttle);
    finalPwm = currentCruiseThrottlePWM;  // Use cruise PWM
    prevPwm = finalPwm;  // Leaving cruise ramps from the output actually held
    break;

  case ARMED:
    int smoothedPwm = wirelessIsReceiver()
        ? getSmoothedThrottlePwmFromRaw(control_throttle,
                                        deviceData.performance_mode)
        : getSmoothedThrottlePwm(deviceData.performance_mode);
    finalPwm =
        applyModeRampClamp(smoothedPwm, prevPwm, deviceData.performance_mode);
    armedOutputPwm = finalPwm;
    break;
  }

  // Suppress the ESC control/telemetry command during a config/firmware relay
  // session. setESCThrottle() pings the ESC every cycle and the ESC answers with
  // a burst of high-rate telemetry; that flood overruns the CAN RX buffer and the
  // SaveConfig reply gets dropped, so the write times out. flash-qc works because
  // its bus is quiet. The device is always DISARMED during a session, so there is
  // no throttle to send anyway. readESCTelemetry() still runs — it pumps the bus
  // and drives the relay state machine.
  if (!escConfigRelayIsActive() && !escFlasherRelayIsActive()) {
    setESCThrottle(finalPwm);
  }

  // Read/Sync ESC Telemetry (runs in all armed states)
  readESCTelemetry();
  syncESCTelemetry();
}

// Declare the static variable from esc.cpp as extern here so sync can access it
// extern unsigned long lastEscTimeUpdateMillis;  // REMOVED - Using struct
// member now

void syncESCTelemetry() {
  // Update ESC state based first on TWAI init, then on time since last update
  // received
  if (!escTwaiInitialized) {
    escTelemetryData.escState = TelemetryState::NOT_CONNECTED;
  }

  const AlertCounts escBmsAlerts = getEscBmsAlertCounts();
  const bool hasCriticalEscOrBmsAlert = escBmsAlerts.criticalCount > 0;
  const bool hasWarningEscOrBmsAlert = escBmsAlerts.warningCount > 0;
  EscStatusLightMode escStatusLightMode = EscStatusLightMode::OFF;
  if (escTelemetryData.escState == TelemetryState::CONNECTED) {
    if (currentState == DISARMED) {
      if (hasCriticalEscOrBmsAlert) {
        escStatusLightMode = EscStatusLightMode::DISARMED_CRITICAL;
      } else if (hasWarningEscOrBmsAlert) {
        escStatusLightMode = EscStatusLightMode::DISARMED_WARNING;
      } else {
        escStatusLightMode = EscStatusLightMode::DISARMED_NOMINAL;
      }
    } else if (hasCriticalEscOrBmsAlert) {
      escStatusLightMode = EscStatusLightMode::FLIGHT_CRITICAL;
    } else if (hasWarningEscOrBmsAlert) {
      escStatusLightMode = EscStatusLightMode::FLIGHT_WARNING;
    } else {
      escStatusLightMode = EscStatusLightMode::FLIGHT_NOMINAL;
    }
  }
  requestEscStatusLightMode(escStatusLightMode);

  telemetryHubWriteEsc(escTelemetryData);
}

bool throttleEngaged() { return !throttleSafe(); }

// average the pot buffer
// averagePotBuffer moved to throttle.cpp as throttleFilterAverage()

// get the PPG ready to fly
bool armSystem() {
  uint16_t arm_melody[] = {1047, 1568};
  // const unsigned int arm_vibes[] = { 1, 85, 1, 85, 1, 85, 1 };
  // Throttle task handles ESC commands exclusively to avoid
  // race condition on the shared canard CAN bus instance

  armedAtMillis = platform::millis32();
  armedSecs = 0;  // Reset armed seconds for new session
  setGroundAltitude(deviceData);

  vTaskSuspend(blinkLEDTaskHandle);
  setLEDs(1);  // solid LED while armed
  playMelody(arm_melody, 2);
  queueEscMotorBeepArm();
  // runVibePattern(arm_vibes, 7);
  pulseVibeMotor();  // Ensure this is the active call
  return true;
}

void afterCruiseEnd() {
  // Instead of clearing the buffer which causes throttle to drop to 0,
  // pre-populate it with the current throttle position to ensure smooth
  // transition
  int currentPotVal = readControlThrottleRaw();
  int currentPwmVal =
      potRawToModePwm(currentPotVal, deviceData.performance_mode);

  // Pre-fill the buffer with current pot value for smooth transition
  throttleFilterReset(currentPwmVal);

  cruisedPotVal = 0;
  // pulseVibeMotor();
}

void playCruiseSound() {
  if (ENABLE_BUZZ) {
    uint16_t notify_melody[] = {1976};
    playMelody(notify_melody, 1);
  }
}

void audioTask(void *parameter) {
  MelodyRequest request;

  for (;;) {
    if (xQueueReceive(melodyQueue, &request, portMAX_DELAY) == pdTRUE) {
      if (!ENABLE_BUZZ)
        continue;

      TickType_t nextWakeTime = xTaskGetTickCount();
      for (int i = 0; i < request.size; i++) {
        startTone(request.notes[i]);
        TickType_t delayTicks = pdMS_TO_TICKS(request.duration);
        if (delayTicks == 0) {
          delayTicks = 1;
        }  // Ensure non-zero delay
        vTaskDelayUntil(&nextWakeTime, delayTicks);
        // Add silence gap between notes for distinct separation
        if (i < request.size - 1) {
          stopTone();
          vTaskDelayUntil(&nextWakeTime, pdMS_TO_TICKS(80));
        }
      }
      stopTone();
    }
  }
}

bool initBMSCAN() {
  platform::console.line("Initializing BMS CAN...");

  // Ensure Display CS is 1 (deselected) before working with BMS
  eppg_gpio_write(displayCS, 1);

  // Create the BMS_CAN instance with the provided SPI
  bms_can = new BMS_CAN(bmsCS, MCP_BAUDRATE);

  if (!bms_can->begin()) {
    platform::console.line("Error initializing BMS_CAN");
    bmsCanInitialized = false;  // BMS initialization failed
    return false;
  }
  platform::console.line("BMS CAN initialized successfully");
  bmsCanInitialized = true;  // BMS successfully initialized
  return true;
}

// WebSerial command handling task
void webSerialTask(void *pvParameters) {
  while (true) {
    if (platform::console.hasInput()) {
      // Check if we're allowed to process commands
      if (currentState == DISARMED) {
        poll_serial_commands();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100ms
  }
}
