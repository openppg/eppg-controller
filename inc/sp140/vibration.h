#ifndef INC_SP140_VIBRATION_H_
#define INC_SP140_VIBRATION_H_

#include <Arduino.h>
#include "sp140/structs.h"

#ifdef M0_PIO
  #include "../../inc/sp140/m0-config.h"
#else
  #include "../../inc/sp140/rp2040-config.h"
#endif

// Initialize the vibration motor
bool initVibe();

// Notify with vibration
void vibrateNotify();

// Run a vibration sequence
bool runVibe(const unsigned int sequence[], int siz);
#endif  // INC_SP140_VIBRATION_H_