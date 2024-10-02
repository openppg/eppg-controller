#ifndef INC_SP140_VIBRATION_H_
#define INC_SP140_VIBRATION_H_

#include <Arduino.h>

// Initialize the vibration motor
bool initVibeMotor();

// Notify with vibration
void pulseVibeMotor();

// Run a vibration sequence
bool runVibePattern(const unsigned int sequence[], int siz);
#endif  // INC_SP140_VIBRATION_H_
