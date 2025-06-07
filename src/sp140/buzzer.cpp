#include "sp140/buzzer.h"

#include <Arduino.h>

#include "sp140/shared-config.h"
#include "sp140/esp32s3-config.h"
#include <FreeRTOS.h>
#include <semphr.h>

#define DEBUG_SERIAL USBSerial

// Buzzer PWM configuration - use different channel than vibration motor
// PWM Channel allocation:
// Channel 0: Vibration motor (vibration_pwm.cpp)
// Channel 1: Buzzer (this file)
// Channels 2-7: Available for future use
const int BUZZER_PWM_CHANNEL = 1;
const int BUZZER_PWM_RESOLUTION = 8;
const int BUZZER_PWM_FREQUENCY = 1000;  // Base frequency, will be changed for notes

// External global variables needed
extern HardwareConfig board_config;
extern QueueHandle_t melodyQueue;

static bool buzzerInitialized = false;

/**
 * Initialize the buzzer pin for output using LEDC
 * @return Returns true if initialization was successful, false otherwise
 */
bool initBuzz() {
  // Setup LEDC channel for buzzer
  ledcSetup(BUZZER_PWM_CHANNEL, BUZZER_PWM_FREQUENCY, BUZZER_PWM_RESOLUTION);
  ledcAttachPin(board_config.buzzer_pin, BUZZER_PWM_CHANNEL);
  buzzerInitialized = true;
  return true;
}

/**
 * Start playing a tone at the specified frequency
 */
void startTone(uint16_t frequency) {
  if (!buzzerInitialized || !ENABLE_BUZZ) return;

  // Change the frequency for this channel
  ledcChangeFrequency(BUZZER_PWM_CHANNEL, frequency, BUZZER_PWM_RESOLUTION);
  // Set 50% duty cycle (square wave)
  ledcWrite(BUZZER_PWM_CHANNEL, 128);
}

/**
 * Stop playing the tone
 */
void stopTone() {
  if (!buzzerInitialized) return;

  // Set duty cycle to 0 to stop the tone
  ledcWrite(BUZZER_PWM_CHANNEL, 0);
}

/**
 * Plays a melody using the piezo buzzer
 *
 * @param melody Array of frequencies to play
 * @param siz Size of the melody array
 * @return Returns true if the melody was queued successfully, false otherwise
 */
bool playMelody(uint16_t melody[], int siz) {
  if (!ENABLE_BUZZ) return false;

  // Create a static buffer for the melody
  static uint16_t melodyBuffer[32];  // Adjust size as needed

  // Copy melody to static buffer to ensure it persists
  for (int i = 0; i < std::min(siz, 32); i++) {
    melodyBuffer[i] = melody[i];
  }

  MelodyRequest request = {
    .notes = melodyBuffer,
    .size = (uint8_t)std::min(siz, 32),
    .duration = 100  // Default duration
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
  uint16_t arm_fail_melody[] = { 1760 };
  playMelody(arm_fail_melody, 1);
}
