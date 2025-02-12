#include "sp140/buzzer.h"
#include <Arduino.h>
#include "sp140/structs.h"      // For MelodyRequest and melodyQueue
#include "sp140/shared-config.h" // For ENABLE_BUZ
#include "sp140/globals.h"       // For extern declaration of melodyQueue

// Forward declaration of the audio task function
static void audioTask(void* parameter) {
  MelodyRequest request;
  for(;;) {
    // Wait indefinitely until a melody is queued
    if(xQueueReceive(melodyQueue, &request, portMAX_DELAY) == pdTRUE) {
      if (!ENABLE_BUZ) continue;
      // Use the current tick count as the reference time
      TickType_t nextWakeTime = xTaskGetTickCount();
      for (int i = 0; i < request.size; i++) {
        // Play the tone for the current note
        tone(BUZZER_PIN, request.notes[i]);
        TickType_t delayTicks = pdMS_TO_TICKS(request.duration);
        if(delayTicks == 0) { delayTicks = 1; } // Ensure non-zero delay
        vTaskDelayUntil(&nextWakeTime, delayTicks);
      }
      noTone(BUZZER_PIN);
    }
  }
}

// Plays a melody by copying the provided melody into a static buffer and queueing a MelodyRequest
bool playMelody(uint16_t melody[], int siz) {
  if (!ENABLE_BUZ)
    return false;

  // Create a static buffer for the melody (adjust size as needed)
  static uint16_t melodyBuffer[32];
  int copySize = (siz < 32) ? siz : 32;
  for (int i = 0; i < copySize; i++) {
    melodyBuffer[i] = melody[i];
  }

  MelodyRequest request = {
    .notes = melodyBuffer,
    .size = (uint8_t)copySize,
    .duration = 125  // Default duration in ms
  };

  // Queue the melody request with a timeout
  if(xQueueSend(melodyQueue, &request, pdMS_TO_TICKS(100)) != pdTRUE) {
    USBSerial.println("Failed to queue melody");
    return false;
  }

  return true;
}

// Plays a specific melody to indicate an arm failure
void handleArmFail() {
  uint16_t arm_fail_melody[] = { 820, 640 };
  playMelody(arm_fail_melody, 2);
}

// Initializes the dedicated buzzer task for playing melodies with precise timing
void initBuzzerTask() {
  // Ensure the melodyQueue is created if not already
  if(melodyQueue == NULL) {
    melodyQueue = xQueueCreate(5, sizeof(MelodyRequest));
    if(melodyQueue == NULL) {
      USBSerial.println("Failed to create melodyQueue in initBuzzerTask");
      return;
    }
  }

  // Create the audio task. You can adjust the stack size and priority as needed.
  xTaskCreatePinnedToCore(
    audioTask,
    "BuzzerAudioTask",
    2048,   // stack size
    NULL,
    2,      // priority
    NULL,
    1       // pinned to core 1. Adjust if needed.
  );
}
