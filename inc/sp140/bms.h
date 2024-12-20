#ifndef INC_SP140_BMS_H_
#define INC_SP140_BMS_H_

#include <Arduino.h>
#include <BMS_CAN.h>

// Define CS_PIN based on your ESP32-S3 setup
#define MCP_CS    5  // Chip Select pin for MCP2515
#define MCP_MOSI  11  // MOSI pin for MCP2515
#define MCP_MISO  13  // MISO pin for MCP2515
#define MCP_SCK   12  // Clock pin for MCP2515
#define MCP_BAUDRATE  250000  // Baud rate for MCP2515

bool initBMSCAN();
void updateBMSData();
void printBMSData();
// Declare the BMS_CAN object as extern so it can be used in other files
extern BMS_CAN bms_can;

#endif  // INC_SP140_BMS_H_
