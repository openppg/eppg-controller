#include "Arduino.h"
#include "../../inc/sp140/rp2040-config.h"
#include "../../inc/sp140/task_manager.h"
#include "../../inc/sp140/globals.h"
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>

// Message types for the queues
StateChangeRequestType_t stateChangeRequestQueue = NULL;
NotificationType_t notificationQueue = NULL;

// Task handles
TaskHandle_t stateManagerTaskHandle = NULL;
TaskHandle_t notificationTaskHandle = NULL;

// External declarations of global variables
extern volatile DeviceState currentState;
extern TaskHandle_t blinkLEDTaskHandle;
extern TaskHandle_t updateDisplayTaskHandle;
extern unsigned long lastDisarmTime;
extern uint32_t armedAtMillis;
extern uint32_t cruisedAtMillis;
extern int cruisedPotVal;
extern HardwareConfig board_config;
extern ResponsiveAnalogRead* pot;
extern STR_DEVICE_DATA_140_V1 deviceData;
extern SemaphoreHandle_t stateMutex;

// Initialize the task manager
void initTaskManager() {
  // Create queues
  stateChangeRequestQueue = xQueueCreate(5, sizeof(StateChangeRequest));
  notificationQueue = xQueueCreate(5, sizeof(Notification));

  // Create tasks
  xTaskCreateAffinitySet(
    stateManagerTask,
    "stateManager",
    2048,
    NULL,
    3, // Same priority as telemetryTask
    uxCoreAffinityMask0,
    &stateManagerTaskHandle
  );

  xTaskCreateAffinitySet(
    notificationTask,
    "notification",
    1024,
    NULL,
    1, // Lowest priority
    uxCoreAffinityMask0,
    &notificationTaskHandle
  );
}

// Send a state change request from a task
bool sendStateChangeRequest(StateChangeRequest request) {
  if (stateChangeRequestQueue == NULL) {
    return false;
  }
  return xQueueSend(stateChangeRequestQueue, &request, pdMS_TO_TICKS(10)) == pdTRUE;
}

// Send a notification from a task
bool sendNotification(Notification notification) {
  if (notificationQueue == NULL) {
    return false;
  }
  return xQueueSend(notificationQueue, &notification, pdMS_TO_TICKS(10)) == pdTRUE;
}

// State manager task
void stateManagerTask(void *pvParameters) {
  (void)pvParameters;
  StateChangeRequest request;

  for (;;) {
    // Wait for a state change request
    if (xQueueReceive(stateChangeRequestQueue, &request, portMAX_DELAY) == pdTRUE) {
      // Process the request based on its type
      switch (request.type) {
        case ARM_DISARM_REQUEST:
          handleArmDisarmRequest();
          break;

        case CRUISE_REQUEST:
          handleCruiseRequest();
          break;

        default:
          // Unknown request type
          break;
      }
    }
  }

  vTaskDelete(NULL); // Should never get here
}

// Notification task
void notificationTask(void *pvParameters) {
  (void)pvParameters;
  Notification notification;

  for (;;) {
    // Wait for a notification
    if (xQueueReceive(notificationQueue, &notification, portMAX_DELAY) == pdTRUE) {
      // Process the notification based on its type
      switch (notification.type) {
        case MELODY_ARM:
          playArmMelody();
          break;

        case MELODY_DISARM:
          playDisarmMelody();
          break;

        case MELODY_ARM_FAIL:
          playArmFailMelody();
          break;

        case VIBE_ARM:
          triggerArmVibration();
          break;

        case VIBE_DISARM:
          triggerDisarmVibration();
          break;

        case MELODY_CRUISE:
          playCruiseMelody();
          break;

        case VIBE_CRUISE:
          triggerCruiseVibration();
          break;

        default:
          // Unknown notification type
          break;
      }
    }
  }

  vTaskDelete(NULL); // Should never get here
}

// Handle ARM/DISARM request
void handleArmDisarmRequest() {
  // Check the current state
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    if (currentState == DISARMED) {
      // Check if enough time has passed since last disarm
      if (millis() - lastDisarmTime >= DISARM_COOLDOWN) {
        // Check if the throttle is not engaged
        if (!throttleEngaged()) {
          // Change state to ARMED
          currentState = ARMED;

          // Set armed time
          armedAtMillis = millis();

          // Set ground altitude
          setGroundAltitude(deviceData);

          // Suspend LED blinking task
          vTaskSuspend(blinkLEDTaskHandle);

          // Set LED to solid
          setLEDs((byte)HIGH);

          // Ensure ESC is disarmed initially
          setESCThrottle(ESC_DISARMED_PWM);

          // Release the mutex before sending notifications
          xSemaphoreGive(stateMutex);

          // Queue feedback (melody and vibration)
          Notification melodyNotification = { .type = MELODY_ARM };
          Notification vibeNotification = { .type = VIBE_ARM };
          sendNotification(melodyNotification);
          sendNotification(vibeNotification);
        } else {
          // Throttle is engaged, can't arm
          xSemaphoreGive(stateMutex);

          // Queue arm fail melody
          Notification failNotification = { .type = MELODY_ARM_FAIL };
          sendNotification(failNotification);
        }
      } else {
        // Still in cooldown period, release mutex
        xSemaphoreGive(stateMutex);
      }
    } else {
      // Currently ARMED or ARMED_CRUISING, change to DISARMED
      currentState = DISARMED;

      // Disarm ESC
      disarmESC();

      // Reset throttle smoothing
      resetSmoothing();

      // Resume LED blinking task
      vTaskResume(blinkLEDTaskHandle);

      // Update armed time statistics
      updateArmedTime();

      // Save device data
      writeDeviceData();

      // Set the last disarm time
      lastDisarmTime = millis();

      // Release the mutex before sending notifications
      xSemaphoreGive(stateMutex);

      // Queue feedback (melody and vibration)
      Notification melodyNotification = { .type = MELODY_DISARM };
      Notification vibeNotification = { .type = VIBE_DISARM };
      sendNotification(melodyNotification);
      sendNotification(vibeNotification);
    }
  }
}

// Handle CRUISE request
void handleCruiseRequest() {
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    switch (currentState) {
      case ARMED: {
        if (throttleEngaged()) {
          // Change state to ARMED_CRUISING
          currentState = ARMED_CRUISING;

          // Save current throttle position
          cruisedPotVal = pot->getValue();
          cruisedAtMillis = millis();

          // Release the mutex before sending notifications
          xSemaphoreGive(stateMutex);

          // Queue feedback
          Notification melodyNotification = { .type = MELODY_CRUISE };
          Notification vibeNotification = { .type = VIBE_CRUISE };
          sendNotification(melodyNotification);
          sendNotification(vibeNotification);
        } else {
          // Call modeSwitch when armed but throttle not engaged
          xSemaphoreGive(stateMutex);
          modeSwitch(false);
        }
        break;
      }

      case ARMED_CRUISING: {
        // Change state to ARMED
        currentState = ARMED;

        // Reset cruised values
        cruisedPotVal = 0;

        // Release the mutex before sending notifications
        xSemaphoreGive(stateMutex);

        // Queue feedback
        Notification melodyNotification = { .type = MELODY_CRUISE };
        Notification vibeNotification = { .type = VIBE_CRUISE };
        sendNotification(melodyNotification);
        sendNotification(vibeNotification);
        break;
      }

      case DISARMED: {
        // Show stats screen when disarmed
        xSemaphoreGive(stateMutex);
        vTaskSuspend(updateDisplayTaskHandle);
        displayMeta(deviceData, 2000); // 2 second duration
        vTaskResume(updateDisplayTaskHandle);
        break;
      }
    }
  }
}

// Notification helper functions
void playArmMelody() {
  uint16_t arm_melody[] = { 1760, 1976, 2093 };
  playMelody(arm_melody, 3);
}

void playDisarmMelody() {
  uint16_t disarm_melody[] = { 2093, 1976, 880 };
  playMelody(disarm_melody, 3);
}

void playArmFailMelody() {
  uint16_t arm_fail_melody[] = { 820, 640 };
  playMelody(arm_fail_melody, 2);
}

void playCruiseMelody() {
  if (ENABLE_BUZ) {
    uint16_t notify_melody[] = { 900, 900 };
    playMelody(notify_melody, 2);
  }
}

void triggerArmVibration() {
  if (board_config.enable_vib) {
    const unsigned int arm_vibes[] = { 1, 85, 1, 85, 1, 85, 1 };
    runVibePattern(arm_vibes, 7);
  }
}

void triggerDisarmVibration() {
  if (board_config.enable_vib) {
    const unsigned int disarm_vibes[] = { 78, 49 };
    runVibePattern(disarm_vibes, 2);
  }
}

void triggerCruiseVibration() {
  if (board_config.enable_vib) {
    pulseVibeMotor();
  }
}
