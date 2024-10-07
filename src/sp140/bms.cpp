#include "sp140/bms.h"

// Initialize the BMS_CAN object
BMS_CAN bms_can(MCP_CS, MCP_BAUDRATE, MCP_MOSI, MCP_MISO, MCP_SCK);

bool initBMSCAN() {
  if (!bms_can.begin()) {
    USBSerial.println("Error initializing BMS_CAN");
    return false;
  }
  return true;
}

void updateBMSData() {
  bms_can.update();
}

void printBMSData() {
  USBSerial.println("BMS Data:");
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

  USBSerial.print("Lowest Cell Voltage: ");
  USBSerial.print(bms_can.getLowestCellVoltage());
  USBSerial.println(" V");

  USBSerial.print("Battery Charging: ");
  USBSerial.println(bms_can.isBatteryCharging() ? "Yes" : "No");

  USBSerial.print("Battery Low SOC: ");
  USBSerial.println(bms_can.isBatteryLowSOC() ? "Yes" : "No");

  USBSerial.print("Highest Temperature: ");
  USBSerial.print(bms_can.getHighestTemperature());
  USBSerial.println(" °C");

  USBSerial.print("Lowest Temperature: ");
  USBSerial.print(bms_can.getLowestTemperature());
  USBSerial.println(" °C");

  USBSerial.print("Battery Cycle: ");
  USBSerial.println(bms_can.getBatteryCycle());

  USBSerial.print("Energy Cycle: ");
  USBSerial.print(bms_can.getEnergyCycle());
  USBSerial.println(" Wh");

  USBSerial.print("Battery Failure Level: ");
  USBSerial.println(bms_can.getBatteryFailureLevel());
}
