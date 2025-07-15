#ifndef INC_SP140_VIBRATION_PWM_H_
#define INC_SP140_VIBRATION_PWM_H_

#include <Arduino.h>

// Vibration patterns
enum VibePattern {
  VIBE_SHORT_PULSE,
  VIBE_LONG_PULSE,
  VIBE_DOUBLE_PULSE,
  VIBE_TRIPLE_PULSE,
  VIBE_RAMP_UP,
  VIBE_RAMP_DOWN,
  VIBE_WAVE,
  VIBE_CRITICAL_CONTINUOUS  // New pattern for critical alerts
};

// Vibration request structure for queue
struct VibeRequest {
  uint16_t duration_ms;
  uint8_t intensity;
};

// Task and queue handles
extern TaskHandle_t vibeTaskHandle;
extern QueueHandle_t vibeQueue;

/**
 * Initialize the vibration motor pin for output using LEDC
 * @return Returns true if initialization was successful, false otherwise
 */
bool initVibeMotor();

/**
 * Pulse the vibration motor with a single 400ms pulse
 */
void pulseVibeMotor();

/**
 * Run a custom vibration pattern using an array of intensities
 * @param pattern Array of intensity values (0-255)
 * @param patternSize Number of elements in the pattern array
 * @return Returns true if pattern was executed successfully, false otherwise
 */
bool runVibePattern(const unsigned int pattern[], int patternSize);

/**
 * Execute a predefined vibration pattern
 * @param pattern The VibePattern enum value to execute
 */
void executeVibePattern(VibePattern pattern);

/**
 * Execute a custom vibration pattern with specified intensities and durations
 * @param intensities Array of intensity values (0-255)
 * @param durations Array of duration values in milliseconds
 * @param steps Number of steps in the pattern
 */
void customVibePattern(const uint8_t intensities[], const uint16_t durations[], int steps);

/**
 * Start continuous vibration for critical alerts
 */
void startCriticalVibration();

/**
 * Stop continuous vibration
 */
void stopCriticalVibration();

/**
 * Check if critical vibration is currently active
 */
bool isCriticalVibrationActive();

#endif  // INC_SP140_VIBRATION_PWM_H_
