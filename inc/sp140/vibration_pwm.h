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

bool initVibeMotor();
void pulseVibeMotor();
bool runVibePattern(const unsigned int pattern[], int patternSize);
void executeVibePattern(VibePattern pattern);
void customVibePattern(const uint8_t intensities[], const uint16_t durations[], int steps);

#endif  // INC_SP140_VIBRATION_PWM_H_
