#include "sp140/bms.h"
#include "sp140/display.h"
#include "sp140/structs.h"

STR_BMS_TELEMETRY_140 bmsTelemetryData = {
  .state = TelemetryState::NOT_CONNECTED
};

// Initialize bms_can as nullptr, to be set in initBMSCAN
BMS_CAN* bms_can = nullptr;

// These are defined in sp140.ino
extern Adafruit_ST7735* display;
extern int8_t displayCS;
extern int8_t bmsCS;
extern bool isBMSPresent;

// BMS initialization is now handled in sp140.ino

// Update BMS data and populate unified battery data
void updateBMSData() {
  if (!isBMSPresent || bms_can == nullptr) return;  // Exit if BMS is not present or not initialized

  // TODO track bms incrementing cycle count
  // Ensure display CS is deselected and BMS CS is selected
  digitalWrite(displayCS, HIGH);
  digitalWrite(bmsCS, LOW);

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

  // Deselect BMS CS when done
  digitalWrite(bmsCS, HIGH);

  // printBMSData();
}

void printBMSData() {
  if (!isBMSPresent || bms_can == nullptr) return;  // Exit if BMS is not present or not initialized

  // Make sure BMS CS is selected before reading values
  digitalWrite(displayCS, HIGH);
  digitalWrite(bmsCS, LOW);

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

  // Deselect BMS CS when done
  digitalWrite(bmsCS, HIGH);
}

