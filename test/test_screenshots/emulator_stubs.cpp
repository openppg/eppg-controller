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

// --- Sensor ID string functions ---
const char* sensorIDToString(SensorID id) {
  switch (id) {
    case SensorID::ESC_MOS_Temp:     return "ESC MOS Temp";
    case SensorID::ESC_MCU_Temp:     return "ESC MCU Temp";
    case SensorID::ESC_CAP_Temp:     return "ESC CAP Temp";
    case SensorID::Motor_Temp:       return "Motor Temp";
    case SensorID::BMS_MOS_Temp:     return "BMS MOS Temp";
    case SensorID::BMS_T1_Temp:      return "BMS T1";
    case SensorID::BMS_SOC:          return "BMS SOC";
    case SensorID::CPU_Temp:         return "CPU Temp";
    default:                         return "Unknown";
  }
}

const char* sensorIDToAbbreviation(SensorID id) {
  switch (id) {
    case SensorID::ESC_MOS_Temp:     return "E.MOS";
    case SensorID::ESC_MCU_Temp:     return "E.MCU";
    case SensorID::ESC_CAP_Temp:     return "E.CAP";
    case SensorID::Motor_Temp:       return "MOT";
    case SensorID::BMS_MOS_Temp:     return "B.MOS";
    case SensorID::BMS_Balance_Temp: return "B.BAL";
    case SensorID::BMS_T1_Temp:      return "B.T1";
    case SensorID::BMS_T2_Temp:      return "B.T2";
    case SensorID::BMS_T3_Temp:      return "B.T3";
    case SensorID::BMS_T4_Temp:      return "B.T4";
    case SensorID::BMS_High_Cell_Voltage: return "HiCell";
    case SensorID::BMS_Low_Cell_Voltage:  return "LoCell";
    case SensorID::BMS_SOC:          return "SOC";
    case SensorID::BMS_Total_Voltage: return "BatV";
    case SensorID::BMS_Voltage_Differential: return "Vdiff";
    case SensorID::Baro_Temp:        return "BARO";
    case SensorID::CPU_Temp:         return "CPU";
    default:                         return "???";
  }
}

const char* sensorIDToAbbreviationWithLevel(SensorID id, AlertLevel level) {
  // For the emulator, just return the basic abbreviation with a suffix
  static char buf[16];
  const char* abbr = sensorIDToAbbreviation(id);
  switch (level) {
    case AlertLevel::WARN_HIGH: snprintf(buf, sizeof(buf), "%s Hi", abbr); break;
    case AlertLevel::WARN_LOW:  snprintf(buf, sizeof(buf), "%s Lo", abbr); break;
    case AlertLevel::CRIT_HIGH: snprintf(buf, sizeof(buf), "%s HI", abbr); break;
    case AlertLevel::CRIT_LOW:  snprintf(buf, sizeof(buf), "%s LO", abbr); break;
    default:                    snprintf(buf, sizeof(buf), "%s", abbr); break;
  }
  return buf;
}

SensorCategory getSensorCategory(SensorID) { return SensorCategory::ESC; }
void initSimpleMonitor() {}
void checkAllSensors() {}
void checkAllSensorsWithData(const STR_ESC_TELEMETRY_140&, const STR_BMS_TELEMETRY_140&) {}
void addESCMonitors() {}
void addBMSMonitors() {}
void addAltimeterMonitors() {}
void addInternalMonitors() {}
void enableMonitoring() {}

// --- BMS stubs ---
BMS_CAN* bms_can = nullptr;
bool initBMSCAN(SPIClass*) { return false; }
void updateBMSData() {}
void printBMSData() {}
