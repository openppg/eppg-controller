#include "platform/runtime.h"
#include "sp140/vibration_pwm.h"
#include "sp140/esp32s3-config.h"

// Pin is configured via s3_config.vibe_pwm
const int VIBE_PWM_FREQ = 1000;  // Adjust as needed
const int VIBE_PWM_RESOLUTION = 8;  // 8-bit resolution

// PWM Channel allocation:
// Channel 0: Vibration motor (this file)
// Channel 1: Buzzer (buzzer.cpp)
// Channels 2-7: Available for future use
const int VIBE_PWM_CHANNEL = 0;

/**
 * Vibration task - processes vibration requests from queue
 */
void vibeTask(void* parameter) {
  VibeRequest request;

  for (;;) {
    if (xQueueReceive(vibeQueue, &request, portMAX_DELAY) == pdTRUE) {
      if (ENABLE_VIBE) {
        // Turn on vibration with specified intensity
        eppg_pwm_write(VIBE_PWM_CHANNEL, request.intensity);

        // Wait for specified duration
        vTaskDelay(pdMS_TO_TICKS(request.duration_ms));

        // Turn off vibration
        eppg_pwm_write(VIBE_PWM_CHANNEL, 0);
      }
    }
  }
}

/**
 * Initialize the vibration motor pin for output using LEDC
 * @return Returns true if initialization was successful, false otherwise
 */
bool initVibeMotor() {
  extern HardwareConfig board_config;
  if (eppg_pwm_config(VIBE_PWM_CHANNEL, board_config.vibe_pwm, VIBE_PWM_FREQ) != 0) return false;

  // Create vibration queue
  vibeQueue = xQueueCreate(5, sizeof(VibeRequest));
  if (vibeQueue == NULL) {
    return false;
  }

  // Task is created in setupTasks() after init, to centralize task creation
  return true;
}

/**
 * Pulse the vibration motor with a single 400ms pulse (non-blocking)
 */
void pulseVibeMotor() {
  if (!ENABLE_VIBE || vibeQueue == NULL) return;

  VibeRequest request = {
    .duration_ms = 400,
    .intensity = 255
  };

  // Send request to queue (non-blocking)
  if (xQueueSend(vibeQueue, &request, 0) != pdTRUE) {
    // Handle queue full error
    platform::console.line("Vibration queue full!");
  }
}

/**
 * @brief Pulses the vibration motor for a specific duration and intensity.
 */
void pulseVibration(uint16_t duration_ms, uint8_t intensity) {
  if (!ENABLE_VIBE || vibeQueue == NULL) return;

  VibeRequest request = {
    .duration_ms = duration_ms,
    .intensity = intensity
  };
  xQueueSend(vibeQueue, &request, 0);
}

/**
 * @brief Stops all vibration immediately.
 */
void stopVibration() {
  if (vibeQueue != NULL) {
    xQueueReset(vibeQueue);
  }
  eppg_pwm_write(VIBE_PWM_CHANNEL, 0);
}

/**
 * Run a custom vibration pattern using an array of intensities
 * @param pattern Array of intensity values (0-255)
 * @param patternSize Number of elements in the pattern array
 * @return Returns true if pattern was executed successfully, false otherwise
 */
bool runVibePattern(const unsigned int pattern[], int patternSize) {
  if (!ENABLE_VIBE) return false;

  for (int i = 0; i < patternSize; i++) {
    eppg_pwm_write(VIBE_PWM_CHANNEL, pattern[i]);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  eppg_pwm_write(VIBE_PWM_CHANNEL, 0);  // Turn off vibration
  return true;
}

/**
 * Execute a predefined vibration pattern
 * @param pattern The VibePattern enum value to execute
 */
void executeVibePattern(VibePattern pattern) {
  if (!ENABLE_VIBE) return;

  switch (pattern) {
    case VIBE_SHORT_PULSE:
      eppg_pwm_write(VIBE_PWM_CHANNEL, 255);
      vTaskDelay(pdMS_TO_TICKS(100));
      eppg_pwm_write(VIBE_PWM_CHANNEL, 0);
      break;

    case VIBE_LONG_PULSE:
      eppg_pwm_write(VIBE_PWM_CHANNEL, 255);
      vTaskDelay(pdMS_TO_TICKS(500));
      eppg_pwm_write(VIBE_PWM_CHANNEL, 0);
      break;

    case VIBE_DOUBLE_PULSE:
      for (int i = 0; i < 2; i++) {
        eppg_pwm_write(VIBE_PWM_CHANNEL, 255);
        vTaskDelay(pdMS_TO_TICKS(150));
        eppg_pwm_write(VIBE_PWM_CHANNEL, 0);
        vTaskDelay(pdMS_TO_TICKS(150));
      }
      break;

    case VIBE_TRIPLE_PULSE:
      for (int i = 0; i < 3; i++) {
        eppg_pwm_write(VIBE_PWM_CHANNEL, 255);
        vTaskDelay(pdMS_TO_TICKS(100));
        eppg_pwm_write(VIBE_PWM_CHANNEL, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      break;

    case VIBE_RAMP_UP:
      for (int i = 0; i <= 255; i += 5) {
        eppg_pwm_write(VIBE_PWM_CHANNEL, i);
        vTaskDelay(pdMS_TO_TICKS(25));
      }
      eppg_pwm_write(VIBE_PWM_CHANNEL, 0);
      break;

    case VIBE_RAMP_DOWN:
      for (int i = 255; i >= 0; i -= 5) {
        eppg_pwm_write(VIBE_PWM_CHANNEL, i);
        vTaskDelay(pdMS_TO_TICKS(25));
      }
      break;

    case VIBE_WAVE:
      for (int i = 0; i <= 180; i += 5) {
        int intensity = (sin(i * 3.14159265358979323846 / 180.0) + 1) * 127.5;
        eppg_pwm_write(VIBE_PWM_CHANNEL, intensity);
        vTaskDelay(pdMS_TO_TICKS(20));
      }
      eppg_pwm_write(VIBE_PWM_CHANNEL, 0);
      break;
  }
}

/**
 * Execute a custom vibration pattern with specified intensities and durations
 * @param intensities Array of intensity values (0-255)
 * @param durations Array of duration values in milliseconds
 * @param steps Number of steps in the pattern
 */
void customVibePattern(const uint8_t intensities[], const uint16_t durations[], int steps) {
  if (!ENABLE_VIBE) return;

  for (int i = 0; i < steps; i++) {
    eppg_pwm_write(VIBE_PWM_CHANNEL, intensities[i]);
    vTaskDelay(pdMS_TO_TICKS(durations[i]));
  }
  eppg_pwm_write(VIBE_PWM_CHANNEL, 0);
}
