// Copyright 2021 <Zach Whitehead>
#include "../version.h"

#ifndef INC_SP140_SHARED_CONFIG_H_
#define INC_SP140_SHARED_CONFIG_H_

// Batt setting now configurable by user. Read from device data
#define BATT_MIN_V 60.0  // 24 * 2.5V per cell

// Calibration
#define MAMP_OFFSET 200
#define VOLT_OFFSET 1.5


#define CRUISE_GRACE 1.5  // 1.5 sec period to get off throttle
#define POT_ENGAGEMENT_LEVEL 0.05 * POT_MAX_VALUE  // 5% or less // TODO calibrate for each device

#define DEFAULT_SEA_PRESSURE 1013.25

// Library config
#define NO_ADAFRUIT_SSD1306_COLOR_COMPATIBILITY

#define ESC_DISARMED_PWM      1010
#define ESC_MIN_PWM           1080  // ESC min is 1080
#define ESC_MAX_PWM           1900  // ESC max 1900

#define ENABLE_BUZ            true    // enable buzzer
#define ENABLE_VIB_LOW_BAT    false   // vibrate if armed and battery voltage sags below min volts. Gets pilot's attention.
#define POT_MIN_VALUE         0       // 12 bit ADC //TODO: use calibration and store in EEPROM
#define POT_MAX_VALUE         4095    // 12 bit ADC //TODO: use calibration and store in EEPROM

enum DeviceRevision {
  M0 = 0,
  V1 = 1,
  MODULE = 2,
  ESPCAN = 3
};

#endif  // INC_SP140_SHARED_CONFIG_H_
