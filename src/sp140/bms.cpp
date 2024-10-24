#include "sp140/bms.h"

// Initialize the BMS_CAN object
BMS_CAN bms_can(MCP_CS, MCP_BAUDRATE, MCP_MOSI, MCP_MISO, MCP_SCK);

// Define the global flag and battery data
bool isBMSPresent = false;
UnifiedBatteryData batteryData = {0.0f, 0.0f, 0.0f};

bool initBMSCAN() {
  USBSerial.println("Initializing BMS CAN...");
  if (!bms_can.begin()) {
      USBSerial.println("Error initializing BMS_CAN");
      isBMSPresent = false;  // BMS initialization failed
      return false;
  }
  USBSerial.println("BMS CAN initialized successfully");
  isBMSPresent = true;  // BMS successfully initialized
  return true;
}

// Update BMS data and populate unified battery data
void updateBMSData() {
  if (!isBMSPresent) return;  // Exit if BMS is not present

  USBSerial.println("Updating BMS Data");
  bms_can.update();
  //printBMSData();
}

void printBMSData() {
  //USBSerial.println("BMS Data:");
  USBSerial.print("SOC: ");  // State of Charge
  USBSerial.print(bms_can.getSOC());
  USBSerial.println(" %");

  USBSerial.print("Battery Voltage: ");
  USBSerial.print(bms_can.getBatteryVoltage());
  USBSerial.println(" V");

  USBSerial.print("Battery Current: ");
  USBSerial.print(bms_can.getBatteryCurrent());
  USBSerial.println(" A");

  USBSerial.print("Power: ");
  USBSerial.print(bms_can.getPower());
  USBSerial.println(" kW");

  USBSerial.print("Highest Cell Voltage: ");
  USBSerial.print(bms_can.getHighestCellVoltage());
  USBSerial.println(" V");

  // Populate unified battery data
  batteryData.volts = bms_can.getBatteryVoltage();
  batteryData.amps = bms_can.getBatteryCurrent();
  batteryData.soc = bms_can.getSOC();
    // Add other data mappings as needed
}

