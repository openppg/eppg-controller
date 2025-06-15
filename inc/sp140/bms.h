#pragma once

#include <Arduino.h>
#include "driver/twai.h"
#include "sp140/structs.h"

// BMS CAN message IDs (from ANT BMS protocol)
#define BMS_BASIC_INFO_1     0x18FF28F4  // SOC, voltage, current, status flags
#define BMS_BASIC_INFO_2     0x18FE28F4  // Cell voltages, temperatures
#define BMS_CYCLE_INFO       0x18A328F4  // Battery cycle information
#define BMS_BATTERY_ID_BASE  0x18A028F4  // Battery ID parsing (base address)
#define BMS_TEMPERATURE      0x18B428F4  // Battery temperature parsing
#define BMS_CELL_VOLTAGE_BASE 0x18C828F4 // Per-cell voltage frames (base address)

// BMS constants
#define BMS_TIMEOUT_MS 1000              // Timeout for BMS connection
#define BMS_MAX_TEMPERATURES 8           // Maximum number of temperature sensors
#define BMS_MAX_CELLS 24                 // Maximum number of battery cells

// External declarations
extern STR_BMS_TELEMETRY_140 bmsTelemetryData;
extern bool bmsTwaiInitialized;

// BMS functions
void initBMS();
void updateBMSData();
bool isBMSMessage(uint32_t id);
void parseBMSPacket(const twai_message_t* message);
bool isBMSConnected();
void printBMSData();
