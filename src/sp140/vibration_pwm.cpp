#ifndef USE_DRV2605

#include "sp140/vibration_pwm.h"
#include "Arduino.h"

const int PWM_PIN = 46;  // TODO: move to config
const int PWM_FREQ = 1000;  // Adjust as needed
const int PWM_RESOLUTION = 8;  // 8-bit resolution
const int PWM_CHANNEL = 0;

bool vibeMotorInitialized = false;

bool initVibeMotor() {
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  vibeMotorInitialized = true;
  return true;
}

void pulseVibeMotor() {
  if (!vibeMotorInitialized) return;
  ledcWrite(PWM_CHANNEL, 255);
  delay(200);
  ledcWrite(PWM_CHANNEL, 0);
}

bool runVibePattern(const unsigned int pattern[], int patternSize) {
  if (!vibeMotorInitialized) return false;

  for (int i = 0; i < patternSize; i++) {
    ledcWrite(PWM_CHANNEL, pattern[i]);
    delay(100);  // Adjust timing as needed
  }
  ledcWrite(PWM_CHANNEL, 0);  // Turn off vibration
  return true;
}

void executeVibePattern(VibePattern pattern) {
  if (!vibeMotorInitialized) return;

  switch (pattern) {
    case VIBE_SHORT_PULSE:
      ledcWrite(PWM_CHANNEL, 255);
      delay(100);
      ledcWrite(PWM_CHANNEL, 0);
      break;

    case VIBE_LONG_PULSE:
      ledcWrite(PWM_CHANNEL, 255);
      delay(500);
      ledcWrite(PWM_CHANNEL, 0);
      break;

    case VIBE_DOUBLE_PULSE:
      for (int i = 0; i < 2; i++) {
        ledcWrite(PWM_CHANNEL, 255);
        delay(150);
        ledcWrite(PWM_CHANNEL, 0);
        delay(150);
      }
      break;

    case VIBE_TRIPLE_PULSE:
      for (int i = 0; i < 3; i++) {
        ledcWrite(PWM_CHANNEL, 255);
        delay(100);
        ledcWrite(PWM_CHANNEL, 0);
        delay(100);
      }
      break;

    case VIBE_RAMP_UP:
      for (int i = 0; i <= 255; i += 5) {
        ledcWrite(PWM_CHANNEL, i);
        delay(25);
      }
      ledcWrite(PWM_CHANNEL, 0);
      break;

    case VIBE_RAMP_DOWN:
      for (int i = 255; i >= 0; i -= 5) {
        ledcWrite(PWM_CHANNEL, i);
        delay(25);
      }
      break;

    case VIBE_WAVE:
      for (int i = 0; i <= 180; i += 5) {
        int intensity = (sin(i * PI / 180.0) + 1) * 127.5;
        ledcWrite(PWM_CHANNEL, intensity);
        delay(20);
      }
      ledcWrite(PWM_CHANNEL, 0);
      break;
  }
}

void customVibePattern(const uint8_t intensities[], const uint16_t durations[], int steps) {
  if (!vibeMotorInitialized) return;

  for (int i = 0; i < steps; i++) {
    ledcWrite(PWM_CHANNEL, intensities[i]);
    delay(durations[i]);
  }
  ledcWrite(PWM_CHANNEL, 0);
}

#endif // !USE_DRV2605
