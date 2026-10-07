// Copyright 2021 <Zach Whitehead>
#include "../version.h"

#ifndef INC_SP140_SHARED_CONFIG_H_
#define INC_SP140_SHARED_CONFIG_H_

#define CRUISE_GRACE 1.5  // 1.5 sec period to get off throttle
#define POT_ENGAGEMENT_LEVEL 0.05 * POT_MAX_VALUE  // 5% or less // TODO calibrate for each device

#define DEFAULT_SEA_PRESSURE 1013.25

// Library config
#define NO_ADAFRUIT_SSD1306_COLOR_COMPATIBILITY

#define ESC_DISARMED_PWM      1000
#define ESC_MIN_PWM           1035  // ESC min
#define ESC_MAX_PWM           1950  // ESC max 1900

#define ENABLE_BUZZ           true    // enable buzzer
#define ENABLE_VIBE           true    // enable vibration motor
#define POT_MIN_VALUE         0       // 12 bit ADC //TODO: use calibration and store in EEPROM
#define POT_MAX_VALUE         4095    // 12 bit ADC //TODO: use calibration and store in EEPROM

// FIRST_BOOT_QC tuning (see FIRST_BOOT_QC.md). The live throttle mapping does
// NOT use the calibration in v8.1 — capture only; the mapping switch is v8.2.
#define QC_MIN_SPAN                 2000   // reject cal if raw span below this
#define QC_MAX_IDLE                 800    // reject cal if released raw above this
#define QC_MIN_FULL                 3200   // reject cal if full-press raw below this
#define QC_RELEASE_TOLERANCE        100    // re-release must land within this of raw_min
#define QC_STABLE_EPSILON           30     // max-min counts across a stable window
#define QC_STABLE_WINDOW_SAMPLES    25     // ~500 ms at 50 Hz sampling
#define QC_CONFIRM_TIMEOUT_MS       10000  // pot-confirm window for interactive checks
#define QC_SKIP_CONFIRM_TIMEOUT_MS  15000  // button window to confirm an absent device
#define QC_CAN_POLL_ITERATIONS      10     // POST CAN polls (x interval = ~2 s)
#define QC_CAN_POLL_INTERVAL_MS     200
#define QC_CAL_STEP_TIMEOUT_MS      60000  // per calibration step (operator paced)
#define QC_RECORD_JSON_MAX          768    // QC record JSON buffer (IDs + checks)

#endif  // INC_SP140_SHARED_CONFIG_H_
