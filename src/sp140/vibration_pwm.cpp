#include "sp140/vibration_pwm.h"
#include "sp140/esp32s3-config.h"
#include "sp140/lvgl/lvgl_updates.h"

extern HardwareConfig board_config;

// Pin is configured via s3_config.vibe_pwm
const int VIBE_PWM_FREQ = 1000;  // Adjust as needed
const int VIBE_PWM_RESOLUTION = 8;  // 8-bit resolution

// PWM Channel allocation:
// Channel 0: Vibration motor (this file)
// Channel 1: Buzzer (buzzer.cpp)
// Channels 2-7: Available for future use
const int VIBE_PWM_CHANNEL = 0;

static bool criticalVibrationActive = false;
static TaskHandle_t criticalVibeTaskHandle = NULL;

/**
 * Critical vibration task - provides continuous vibration for critical alerts
 */
void criticalVibeTask(void* parameter) {
  for (;;) {
    if (criticalVibrationActive && ENABLE_VIBE) {
      // Pulse every 1 second for critical alerts
      ledcWrite(board_config.vibe_pwm, 200);  // Medium intensity for continuous
      vTaskDelay(pdMS_TO_TICKS(300));    // 300ms on
      ledcWrite(board_config.vibe_pwm, 0);
      vTaskDelay(pdMS_TO_TICKS(700));    // 700ms off (total 1 second cycle)
    } else {
      // If not active, suspend task to save resources
      vTaskSuspend(NULL);
    }
  }
}

/**
 * Vibration task - processes vibration requests from queue
 */
void vibeTask(void* parameter) {
  VibeRequest request;

  for (;;) {
    if (xQueueReceive(vibeQueue, &request, portMAX_DELAY) == pdTRUE) {
      if (ENABLE_VIBE) {
        // Turn on vibration with specified intensity
        ledcWrite(board_config.vibe_pwm, request.intensity);

        // Wait for specified duration
        vTaskDelay(pdMS_TO_TICKS(request.duration_ms));

        // Turn off vibration
        ledcWrite(board_config.vibe_pwm, 0);
      }
    }
  }
}

/**
 * Initialize the vibration motor pin for output using LEDC
 * @return Returns true if initialization was successful, false otherwise
 */
bool initVibeMotor() {
  // Arduino-ESP32 3.x LEDC API: use ledcAttach(pin, freq, resolution)
  pinMode(board_config.vibe_pwm, OUTPUT);
  ledcAttach(board_config.vibe_pwm, VIBE_PWM_FREQ, VIBE_PWM_RESOLUTION);
  ledcWrite(board_config.vibe_pwm, 0);

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
    USBSerial.println("Vibration queue full!");
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
  ledcWrite(board_config.vibe_pwm, 0);
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
    ledcWrite(board_config.vibe_pwm, pattern[i]);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  ledcWrite(board_config.vibe_pwm, 0);  // Turn off vibration
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
      ledcWrite(board_config.vibe_pwm, 255);
      vTaskDelay(pdMS_TO_TICKS(100));
      ledcWrite(board_config.vibe_pwm, 0);
      break;

    case VIBE_LONG_PULSE:
      ledcWrite(board_config.vibe_pwm, 255);
      vTaskDelay(pdMS_TO_TICKS(500));
      ledcWrite(board_config.vibe_pwm, 0);
      break;

    case VIBE_DOUBLE_PULSE:
      for (int i = 0; i < 2; i++) {
        ledcWrite(board_config.vibe_pwm, 255);
        vTaskDelay(pdMS_TO_TICKS(150));
        ledcWrite(board_config.vibe_pwm, 0);
        vTaskDelay(pdMS_TO_TICKS(150));
      }
      break;

    case VIBE_TRIPLE_PULSE:
      for (int i = 0; i < 3; i++) {
        ledcWrite(board_config.vibe_pwm, 255);
        vTaskDelay(pdMS_TO_TICKS(100));
        ledcWrite(board_config.vibe_pwm, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      break;

    case VIBE_RAMP_UP:
      for (int i = 0; i <= 255; i += 5) {
        ledcWrite(board_config.vibe_pwm, i);
        vTaskDelay(pdMS_TO_TICKS(25));
      }
      ledcWrite(board_config.vibe_pwm, 0);
      break;

    case VIBE_RAMP_DOWN:
      for (int i = 255; i >= 0; i -= 5) {
        ledcWrite(board_config.vibe_pwm, i);
        vTaskDelay(pdMS_TO_TICKS(25));
      }
      break;

    case VIBE_WAVE:
      for (int i = 0; i <= 180; i += 5) {
        int intensity = (sin(i * PI / 180.0) + 1) * 127.5;
        ledcWrite(board_config.vibe_pwm, intensity);
        vTaskDelay(pdMS_TO_TICKS(20));
      }
      ledcWrite(board_config.vibe_pwm, 0);
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
    ledcWrite(board_config.vibe_pwm, intensities[i]);
    vTaskDelay(pdMS_TO_TICKS(durations[i]));
  }
  ledcWrite(board_config.vibe_pwm, 0);
}

// Service state for critical alerts
static bool g_critical_alert_active = false;

/**
 * @brief Initializes the critical alert service.
 */
void initCriticalAlertService() {
  // Initialization can be expanded if needed in the future.
}

/**
 * @brief Starts the critical alert notifications.
 */
void startCriticalAlerts() {
  if (g_critical_alert_active) {
    return;
  }
  g_critical_alert_active = true;

  // Start the single master LVGL timer, which will handle both border and vibration
  startCriticalBorderFlash();
}

/**
 * @brief Stops the critical alert notifications.
 */
void stopCriticalAlerts() {
  if (!g_critical_alert_active) {
    return;
  }
  g_critical_alert_active = false;

  // Stop the master LVGL timer
  stopCriticalBorderFlash();
}

/**
 * @brief Checks if the critical alert system is currently active.
 */
bool isCriticalAlertActive() {
  return g_critical_alert_active;
}
