#include "sp140/bms.h"

#include "sp140/structs.h"

STR_BMS_TELEMETRY_140 bmsTelemetryData = {
  .state = TelemetryState::NOT_CONNECTED
};

// Initialize bms_can as nullptr, to be set in initBMSCAN
BMS_CAN* bms_can = nullptr;

// Define the global flag and battery data
bool _isBMSPresent = false;

bool initBMSCAN(SPIClass* spi) {
  USBSerial.println("Initializing BMS CAN...");

  // Create the BMS_CAN instance with the provided SPI
  bms_can = new BMS_CAN(MCP_CS, MCP_BAUDRATE, spi);

  if (!bms_can->begin()) {
    USBSerial.println("Error initializing BMS_CAN");
    _isBMSPresent = false;  // BMS initialization failed
    return false;
  }
  USBSerial.println("BMS CAN initialized successfully");
  _isBMSPresent = true;  // BMS successfully initialized
  return true;
}

// Update BMS data and populate unified battery data
void updateBMSData() {
  if (!_isBMSPresent || bms_can == nullptr) return;  // Exit if BMS is not present or not initialized

  // USBSerial.println("Updating BMS Data");
  bms_can->update();

  // Basic measurements
  bmsTelemetryData.battery_voltage = bms_can->getBatteryVoltage();
  bmsTelemetryData.battery_current = bms_can->getBatteryCurrent();
  bmsTelemetryData.soc = bms_can->getSOC();
  bmsTelemetryData.power = bms_can->getPower();

  // Cell voltages
  bmsTelemetryData.highest_cell_voltage = bms_can->getHighestCellVoltage();
  bmsTelemetryData.lowest_cell_voltage = bms_can->getLowestCellVoltage();

  // Calculated highest cell minus lowest cell voltage
  bmsTelemetryData.voltage_differential = bms_can->getHighestCellVoltage() - bms_can->getLowestCellVoltage();

  // Temperature readings
  bmsTelemetryData.highest_temperature = bms_can->getHighestTemperature();
  bmsTelemetryData.lowest_temperature = bms_can->getLowestTemperature();

  // Battery statistics
  bmsTelemetryData.battery_cycle = bms_can->getBatteryCycle();
  bmsTelemetryData.energy_cycle = bms_can->getEnergyCycle();
  bmsTelemetryData.battery_failure_level = bms_can->getBatteryFailureLevel();

  bmsTelemetryData.lastUpdateMs = millis();

  // printBMSData();
}

void printBMSData() {
  if (!_isBMSPresent || bms_can == nullptr) return;  // Exit if BMS is not present or not initialized

  // USBSerial.println("BMS Data:");
  USBSerial.print("SOC: ");  // State of Charge
  USBSerial.print(bms_can->getSOC());
  USBSerial.println(" %");

  USBSerial.print("Battery Voltage: ");
  USBSerial.print(bms_can->getBatteryVoltage());
  USBSerial.println(" V");

  USBSerial.print("Battery Current: ");
  USBSerial.print(bms_can->getBatteryCurrent());
  USBSerial.println(" A");

  USBSerial.print("Power: ");
  USBSerial.print(bms_can->getPower());
  USBSerial.println(" kW");

  USBSerial.print("Highest Cell Voltage: ");
  USBSerial.print(bms_can->getHighestCellVoltage());
  USBSerial.println(" V");
}

