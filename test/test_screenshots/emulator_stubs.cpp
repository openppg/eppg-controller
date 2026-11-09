// Stub implementations for functions referenced by the LVGL UI code
// that depend on hardware or other subsystems not available natively.

#include <Arduino.h>
#include <SPI.h>
#include <BMS_CAN.h>
#include <cmath>
#include "sp140/structs.h"
#include "sp140/simple_monitor.h"
#include "sp140/alert_display.h"
#include "sp140/vibration_pwm.h"
#include "sp140/esp32s3-config.h"
#include "sp140/ble.h"
#include "sp140/ble/ble_core.h"

// --- Hardware config ---
HardwareConfig s3_config = {};

// --- BLE globals ---
NimBLECharacteristic* pThrottleCharacteristic = nullptr;
NimBLECharacteristic* pDeviceStateCharacteristic = nullptr;
NimBLEServer* pServer = nullptr;
volatile uint16_t connectedHandle = 0;
volatile bool deviceConnected = false;

// --- Globals from globals.h ---
unsigned long cruisedAtMillis = 0;
int cruisedPotVal = 0;
float watts = 0;
float wattHoursUsed = 0;
STR_DEVICE_DATA_140_V1 deviceData = {};
STR_ESC_TELEMETRY_140 escTelemetryData = {};
UnifiedBatteryData unifiedBatteryData = {};
STR_BMS_TELEMETRY_140 bmsTelemetryData = {};
bool bmsCanInitialized = false;
bool escTwaiInitialized = false;
bool bmpPresent = false;

// --- Monitor globals ---
MultiLogger multiLogger;
std::vector<IMonitor*> monitors;
SerialLogger serialLogger;
bool monitoringEnabled = false;

void SerialLogger::log(SensorID, AlertLevel, float) {}
void SerialLogger::log(SensorID, AlertLevel, bool) {}

// --- Alert display stubs ---
QueueHandle_t alertEventQueue = nullptr;
QueueHandle_t alertCarouselQueue = nullptr;
QueueHandle_t alertUIQueue = nullptr;

void initAlertDisplay() {}
void sendAlertEvent(SensorID, AlertLevel) {}

// --- Vibration stubs ---
TaskHandle_t vibeTaskHandle = nullptr;
QueueHandle_t vibeQueue = nullptr;

bool initVibeMotor() { return true; }
void pulseVibeMotor() {}
bool runVibePattern(const unsigned int[], int) { return true; }
void executeVibePattern(VibePattern) {}
void customVibePattern(const uint8_t[], const uint16_t[], int) {}
void pulseVibration(uint16_t, uint8_t) {}
void stopVibration() {}

// Sensor names (sensorIDToString, sensorIDToAbbreviation...) come from the
// real src/sp140/sensor_names.cpp, so alert labels match the controller.

SensorCategory getSensorCategory(SensorID) { return SensorCategory::ESC; }
void initSimpleMonitor() {}
void checkAllSensors() {}
void checkAllSensorsWithData(const STR_ESC_TELEMETRY_140&, const STR_BMS_TELEMETRY_140&) {}
void addESCMonitors() {}
void addBMSMonitors() {}
void addAltimeterMonitors() {}
void addInternalMonitors() {}
void enableMonitoring() {}

// --- BLE core stubs ---
void setupBLE() {}
void requestFastConnParams() {}
void requestNormalConnParams() {}
void enterBLEPairingMode() {}
bool isBLEPairingModeActive() { return false; }

// --- BMS stubs ---
BMS_CAN* bms_can = nullptr;
bool initBMSCAN(SPIClass*) { return false; }
void updateBMSData() {}
void printBMSData() {}
