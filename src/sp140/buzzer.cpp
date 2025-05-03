#include "sp140/buzzer.h"

#include <Arduino.h>

#include <algorithm>

#include "sp140/shared-config.h"
#include "sp140/esp32s3-config.h"
#include <FreeRTOS.h>
#include <semphr.h>

#define DEBUG_SERIAL USBSerial

// External global variables needed
extern HardwareConfig board_config;
extern QueueHandle_t melodyQueue;

/**
 * Initialize the buzzer pin for output
 */
void initBuzz() {
  pinMode(board_config.buzzer_pin, OUTPUT);
}

/**
 * Plays a melody using the piezo buzzer
 *
 * @param melody Array of frequencies to play
 * @param siz Size of the melody array
 * @return Returns true if the melody was queued successfully, false otherwise
 */
bool playMelody(uint16_t melody[], int siz) {
  if (!ENABLE_BUZ) return false;

  // Create a static buffer for the melody
  static uint16_t melodyBuffer[32];  // Adjust size as needed

  // Copy melody to static buffer to ensure it persists
  for (int i = 0; i < std::min(siz, 32); i++) {
    melodyBuffer[i] = melody[i];
  }

  MelodyRequest request = {
    .notes = melodyBuffer,
    .size = (uint8_t)std::min(siz, 32),
    .duration = 125  // Default duration
  };

  // Send to queue with timeout
  if (xQueueSend(melodyQueue, &request, pdMS_TO_TICKS(100)) != pdTRUE) {
    DEBUG_SERIAL.println("Failed to queue melody");
    return false;
  }

  return true;
}

/**
 * Plays a melody to indicate arm failure
 */
void handleArmFailMelody() {
  uint16_t arm_fail_melody[] = { 820, 640 };
  playMelody(arm_fail_melody, 2);
}
