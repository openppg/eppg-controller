#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <ResponsiveAnalogRead.h>

// Include necessary configuration and type definitions
#include "../../inc/sp140/rp2040-config.h"
#include "../../inc/sp140/shared-config.h"
#include "../../inc/sp140/structs.h"

// Let's include the actual DeviceState enum from sp140.ino
// Instead of trying to forward-declare it
#include "../../inc/sp140/sp140-defs.h"

// These are defined in sp140.ino
extern UBaseType_t uxCoreAffinityMask0;
extern UBaseType_t uxCoreAffinityMask1;
extern const unsigned long DISARM_COOLDOWN;
extern volatile DeviceState currentState;

// External functions needed from other files
bool throttleEngaged();
void setGroundAltitude(const STR_DEVICE_DATA_140_V1 &deviceData);
void setLEDs(byte state);
void setESCThrottle(int pwm);
void disarmESC();
void resetSmoothing();
void updateArmedTime();
void writeDeviceData();
void modeSwitch(bool update_display);
void displayMeta(const STR_DEVICE_DATA_140_V1 &deviceData, int duration);
bool playMelody(uint16_t melody[], int siz);
bool runVibePattern(const unsigned int pattern[], int patternSize);
void pulseVibeMotor();

// Enums for message types
typedef enum {
  ARM_DISARM_REQUEST,
  CRUISE_REQUEST
} StateChangeRequestType;

typedef enum {
  MELODY_ARM,
  MELODY_DISARM,
  MELODY_ARM_FAIL,
  VIBE_ARM,
  VIBE_DISARM,
  MELODY_CRUISE,
  VIBE_CRUISE
} NotificationType;

// Struct definitions for the messages
typedef struct {
  StateChangeRequestType type;
} StateChangeRequest;

typedef struct {
  NotificationType type;
} Notification;

// Queue typedefs
typedef QueueHandle_t StateChangeRequestType_t;
typedef QueueHandle_t NotificationType_t;

// External declarations for the queues
extern StateChangeRequestType_t stateChangeRequestQueue;
extern NotificationType_t notificationQueue;

// External task handles
extern TaskHandle_t stateManagerTaskHandle;
extern TaskHandle_t notificationTaskHandle;

// Function declarations
void initTaskManager();
bool sendStateChangeRequest(StateChangeRequest request);
bool sendNotification(Notification notification);
void stateManagerTask(void *pvParameters);
void notificationTask(void *pvParameters);
void handleArmDisarmRequest();
void handleCruiseRequest();

// Helper function declarations
void playArmMelody();
void playDisarmMelody();
void playArmFailMelody();
void playCruiseMelody();
void triggerArmVibration();
void triggerDisarmVibration();
void triggerCruiseVibration();

#endif // TASK_MANAGER_H
