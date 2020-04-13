// Arduino Pins
#define BUTTON_TOP    7   // arm/disarm button_top
#define BUTTON_SIDE   1   // secondary button_top
#define BUZZER_PIN    5   // output for buzzer speaker
#define LED_SW        9   // output for LED on button_top switch
#define LED_2         0   // output for LED 2
#define LED_3         3  // output for LED 3
#define THROTTLE_PIN  A0  // throttle pot input
#define VBAT_PIN      A1  // battery voltage divider

#define CTRL_VER 0x10
#define CTRL2HUB_ID 0x10
#define HUB2CTRL_ID 0x20

#define ARM_VERIFY false
#define CURRENT_DIVIDE 100.0
#define VOLTAGE_DIVIDE 1000.0

// Batt setting now configurable by user. Read from device data
#define BATT_MIN_V 49.0  // 7S min (use 42v for 6S)
#define BATT_MAX_V 58.8  // 7S max (use 50v for 6S)

// Calibration
#define MAMP_OFFSET 200

#define VERSION_MAJOR 5
#define VERSION_MINOR 0

#define CRUISE_GRACE 2  // 2 sec period to get off throttle
#define CRUISE_MAX 300  // 5 min max cruising

#define DEFAULT_SEA_PRESSURE 1013.25

// Library config
#define NO_ADAFRUIT_SSD1306_COLOR_COMPATIBILITY
