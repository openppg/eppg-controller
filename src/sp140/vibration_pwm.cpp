#include "sp140/vibration_pwm.h"
#include "Arduino.h"
#include "sp140/shared-config.h"

const int VIBE_PWM_PIN = 46;  // TODO: move to config
const int VIBE_PWM_FREQ = 1000;  // Adjust as needed
const int VIBE_PWM_RESOLUTION = 8;  // 8-bit resolution

// PWM Channel allocation:
// Channel 0: Vibration motor (this file)
// Channel 1: Buzzer (buzzer.cpp)
// Channels 2-7: Available for future use
const int VIBE_PWM_CHANNEL = 0;

bool vibeMotorInitialized = false;

/**
 * Vibration task - processes vibration requests from queue
 */
void vibeTask(void* parameter) {
  VibeRequest request;
  
  for (;;) {
    if (xQueueReceive(vibeQueue, &request, portMAX_DELAY) == pdTRUE) {
      if (vibeMotorInitialized && ENABLE_VIBE) {
        // Turn on vibration with specified intensity
        ledcWrite(VIBE_PWM_CHANNEL, request.intensity);
        
        // Wait for specified duration
        vTaskDelay(pdMS_TO_TICKS(request.duration_ms));
        
        // Turn off vibration
        ledcWrite(VIBE_PWM_CHANNEL, 0);
      }
    }
  }
}

/**
 * Initialize the vibration motor pin for output using LEDC
 * @return Returns true if initialization was successful, false otherwise
 */
bool initVibeMotor() {
  ledcSetup(VIBE_PWM_CHANNEL, VIBE_PWM_FREQ, VIBE_PWM_RESOLUTION);
  ledcAttachPin(VIBE_PWM_PIN, VIBE_PWM_CHANNEL);
  
  // Create vibration queue
  vibeQueue = xQueueCreate(5, sizeof(VibeRequest));
  if (vibeQueue == NULL) {
    return false;
  }
  
  // Create vibration task - pin to core 1 to keep it away from throttle task
  xTaskCreatePinnedToCore(vibeTask, "Vibration", 2048, NULL, 2, &vibeTaskHandle, 1);
  if (vibeTaskHandle == NULL) {
    return false;
  }
  
  vibeMotorInitialized = true;
  return true;
}

/**
 * Pulse the vibration motor with a single 400ms pulse (non-blocking)
 */
void pulseVibeMotor() {
  if (!vibeMotorInitialized || !ENABLE_VIBE || vibeQueue == NULL) return;
  
  VibeRequest request = {
    .duration_ms = 400,
    .intensity = 255
  };
  
  // Send request to queue (non-blocking)
  xQueueSend(vibeQueue, &request, 0);  // Don't wait if queue is full
}

/**
 * Run a custom vibration pattern using an array of intensities
 * @param pattern Array of intensity values (0-255)
 * @param patternSize Number of elements in the pattern array
 * @return Returns true if pattern was executed successfully, false otherwise
 */
bool runVibePattern(const unsigned int pattern[], int patternSize) {
  if (!vibeMotorInitialized || !ENABLE_VIBE) return false;

  for (int i = 0; i < patternSize; i++) {
    ledcWrite(VIBE_PWM_CHANNEL, pattern[i]);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  ledcWrite(VIBE_PWM_CHANNEL, 0);  // Turn off vibration
  return true;
}

/**
 * Execute a predefined vibration pattern
 * @param pattern The VibePattern enum value to execute
 */
void executeVibePattern(VibePattern pattern) {
  if (!vibeMotorInitialized || !ENABLE_VIBE) return;

  switch (pattern) {
    case VIBE_SHORT_PULSE:
      ledcWrite(VIBE_PWM_CHANNEL, 255);
      vTaskDelay(pdMS_TO_TICKS(100));
      ledcWrite(VIBE_PWM_CHANNEL, 0);
      break;

    case VIBE_LONG_PULSE:
      ledcWrite(VIBE_PWM_CHANNEL, 255);
      vTaskDelay(pdMS_TO_TICKS(500));
      ledcWrite(VIBE_PWM_CHANNEL, 0);
      break;

    case VIBE_DOUBLE_PULSE:
      for (int i = 0; i < 2; i++) {
        ledcWrite(VIBE_PWM_CHANNEL, 255);
        vTaskDelay(pdMS_TO_TICKS(150));
        ledcWrite(VIBE_PWM_CHANNEL, 0);
        vTaskDelay(pdMS_TO_TICKS(150));
      }
      break;

    case VIBE_TRIPLE_PULSE:
      for (int i = 0; i < 3; i++) {
        ledcWrite(VIBE_PWM_CHANNEL, 255);
        vTaskDelay(pdMS_TO_TICKS(100));
        ledcWrite(VIBE_PWM_CHANNEL, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      break;

    case VIBE_RAMP_UP:
      for (int i = 0; i <= 255; i += 5) {
        ledcWrite(VIBE_PWM_CHANNEL, i);
        vTaskDelay(pdMS_TO_TICKS(25));
      }
      ledcWrite(VIBE_PWM_CHANNEL, 0);
      break;

    case VIBE_RAMP_DOWN:
      for (int i = 255; i >= 0; i -= 5) {
        ledcWrite(VIBE_PWM_CHANNEL, i);
        vTaskDelay(pdMS_TO_TICKS(25));
      }
      break;

    case VIBE_WAVE:
      for (int i = 0; i <= 180; i += 5) {
        int intensity = (sin(i * PI / 180.0) + 1) * 127.5;
        ledcWrite(VIBE_PWM_CHANNEL, intensity);
        vTaskDelay(pdMS_TO_TICKS(20));
      }
      ledcWrite(VIBE_PWM_CHANNEL, 0);
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
  if (!vibeMotorInitialized || !ENABLE_VIBE) return;

  for (int i = 0; i < steps; i++) {
    ledcWrite(VIBE_PWM_CHANNEL, intensities[i]);
    vTaskDelay(pdMS_TO_TICKS(durations[i]));
  }
  ledcWrite(VIBE_PWM_CHANNEL, 0);
}
