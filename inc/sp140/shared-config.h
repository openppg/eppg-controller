// Copyright 2021 <Zach Whitehead>
#include "../version.h"

#ifndef INC_SP140_SHARED_CONFIG_H_
#define INC_SP140_SHARED_CONFIG_H_

#define POT_ENGAGEMENT_LEVEL 0.05 * POT_MAX_VALUE  // 5% or less // TODO calibrate for each device

#define DEFAULT_SEA_PRESSURE 1013.25

#define ESC_DISARMED_PWM      1000
#define ESC_MIN_PWM           1035  // ESC min
#define ESC_MAX_PWM           1950  // ESC max 1900

#define ENABLE_BUZZ           true    // enable buzzer
#define ENABLE_VIBE           true    // enable vibration motor
#define POT_MIN_VALUE         0       // 12 bit ADC //TODO: use calibration and store in EEPROM
#define POT_MAX_VALUE         4095    // 12 bit ADC //TODO: use calibration and store in EEPROM

#endif  // INC_SP140_SHARED_CONFIG_H_
