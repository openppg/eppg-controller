#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <BMS_CAN.h>

#include "sp140/structs.h"

// BMS-related constants
#define MCP_CS 5        // MCP2515 CS pin
#define MCP_BAUDRATE 250000

// External declarations
extern STR_BMS_TELEMETRY_140 bmsTelemetryData;
extern BMS_CAN* bms_can;

// BMS functions
void updateBMSData();
void printBMSData();
void printBMSMultiBatteryDebug();
