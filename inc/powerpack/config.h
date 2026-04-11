#ifndef POWERPACK_CONFIG_H_
#define POWERPACK_CONFIG_H_

// CAN bus pins (same hardware as SP140 hand controller)
#define ESC_TX_PIN 2
#define ESC_RX_PIN 3

// Local CAN node ID for this tool
#define LOCAL_NODE_ID 0x01

// Default ESC node ID
#define ESC_NODE_ID 0x20

// Firmware update settings
#define FW_UPDATE_TIMEOUT_MS 150
#define FW_UPDATE_MAX_RETRIES 10
#define FW_CHUNK_SIZE 256

// QC test thresholds
#define QC_TEMP_MIN_C -10.0f   // Minimum reasonable ambient temp
#define QC_TEMP_MAX_C 80.0f    // Max temp for idle ESC
#define QC_MOTOR_TEMP_VALID_MIN_C -20.0f
#define QC_MOTOR_TEMP_VALID_MAX_C 140.0f
#define QC_VBUS_MIN_V 48.0f
#define QC_VBUS_MAX_V 108.0f

// Display (ST7735 via SPI)
#define DISPLAY_CS_PIN 10
#define DISPLAY_DC_PIN 14
#define DISPLAY_RST_PIN 15
#define SPI_MOSI_PIN 11
#define SPI_MISO_PIN 13
#define SPI_SCLK_PIN 12

// Button (GPIO 1 on the SP140 hand controller hardware)
#define BUTTON_PIN 1

// NeoPixel LED
#define NEOPIXEL_PIN 21

// Buzzer
#define BUZZER_PIN 8

// Vibration motor
#define VIBE_PIN 46

#endif // POWERPACK_CONFIG_H_
