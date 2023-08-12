// Copyright 2021 <Zach Whitehead>
#ifndef INC_SP140_SHARED_CONFIG_H_
#define INC_SP140_SHARED_CONFIG_H_

// Batt setting now configurable by user. Read from device data
#define BATT_MIN_V 60.0  // 24 * 2.5V per cell

// Calibration
#define MAMP_OFFSET 200
#define VOLT_OFFSET 1.5

#define VERSION_MAJOR 6
#define VERSION_MINOR 0

#define CRUISE_GRACE 1.5  // 1.5 sec period to get off throttle
#define POT_SAFE_LEVEL 0.05 * 4096  // 5% or less

#define DEFAULT_SEA_PRESSURE 1013.25

// Library config
#define NO_ADAFRUIT_SSD1306_COLOR_COMPATIBILITY

#define ESC_DISARMED_PWM      1010
#define ESC_MIN_PWM           1030  // ESC min is 1050
#define ESC_MAX_PWM           1990  // ESC max 1950


#define ESC_BAUD_RATE         115200
#define ESC_DATA_SIZE         20
#define ESC_DATA_V2_SIZE      22
#define READ_INTERVAL         0
#define ESC_TIMEOUT           15
#define ENABLE_BUZ            true    // enable buzzer
#define ENABLE_VIB_LOW_BAT    false   // vibrate if armed and battery voltage sags below min volts. Gets pilot's attention.
#define POT_MAX_VALUE         4095    // 12 bit ADC //TODO use calibration and store in EEPROM

#endif  // INC_SP140_SHARED_CONFIG_H_
