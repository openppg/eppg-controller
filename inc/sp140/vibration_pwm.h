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
  VIBE_WAVE
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
 * @brief Pulses the vibration motor for a specific duration and intensity.
 *
 * This is a non-blocking function that sends a request to the vibe queue.
 *
 * @param duration_ms The duration of the pulse in milliseconds.
 * @param intensity The vibration intensity (0-255).
 */
void pulseVibration(uint16_t duration_ms, uint8_t intensity);

/**
 * @brief Stops all vibration immediately.
 *
 * This function clears the vibration queue and turns off the PWM signal.
 */
void stopVibration();

/**
 * @brief Initializes the critical alert service.
 *
 * This function sets up the necessary resources for handling synchronized critical alerts.
 * It should be called once during system initialization.
 */
void initCriticalAlertService();

/**
 * @brief Starts the critical alert notifications.
 *
 * Activates the synchronized vibration and border flashing. If alerts are
 * already running, this function has no effect.
 */
void startCriticalAlerts();

/**
 * @brief Stops the critical alert notifications.
 *
 * Deactivates the vibration and border flashing. If alerts are not currently
 * running, this function has no effect.
 */
void stopCriticalAlerts();

/**
 * @brief Checks if the critical alert system is currently active.
 *
 * @return true if critical alerts are active, false otherwise.
 */
bool isCriticalAlertActive();

#endif  // INC_SP140_VIBRATION_PWM_H_
