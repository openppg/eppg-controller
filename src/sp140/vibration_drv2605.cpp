#ifdef USE_DRV2605

#include "sp140/vibration_drv2605.h"
#include <Adafruit_DRV2605.h>

Adafruit_DRV2605 vibeMotor;

bool vibeMotorInitialized = false;

bool initVibeMotor() {
  if (!vibeMotor.begin(&Wire1)) {
    return false;
  }
  vibeMotor.selectLibrary(3);
  vibeMotor.setMode(DRV2605_MODE_INTTRIG);
  vibeMotorInitialized = true;

  pulseVibeMotor();  // initial boot vibration
  return true;
}

void pulseVibeMotor() {
  const unsigned int pulsePattern[] = { 14, 1, 14 };
  runVibePattern(pulsePattern, 3);
}

bool runVibePattern(const unsigned int pattern[], int patternSize) {
  if (!vibeMotorInitialized) {
    return false;
  }

  for (int i = 0; i < patternSize; i++) {
    vibeMotor.setWaveform(i, pattern[i]);
  }
  vibeMotor.setWaveform(patternSize, 0);
  vibeMotor.go();
  return true;
}

#endif // USE_DRV2605
