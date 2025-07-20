#include "sp140/bms.h"
#include "sp140/structs.h"
#include "sp140/globals.h"

// Initialize bms_can as nullptr, to be set in initBMSCAN
BMS_CAN* bms_can = nullptr;

// These are defined in sp140.ino
extern int8_t displayCS;
extern int8_t bmsCS;

// BMS initialization is now handled in sp140.ino

// Update BMS data and populate unified battery data
void updateBMSData() {
  // Check if BMS is currently connected and the object exists
  if (bms_can == nullptr || !bmsCanInitialized) {
    // BMS not connected - update with sentinel values
    STR_BMS_TELEMETRY_140 newBmsData = {};
    newBmsData.bmsState = TelemetryState::NOT_CONNECTED;
    
    // Set sentinel values for all numeric fields
    newBmsData.soc = -1.0f;
    newBmsData.battery_voltage = -1.0f;
    newBmsData.battery_current = -1.0f;
    newBmsData.power = -1.0f;
    newBmsData.highest_cell_voltage = -1.0f;
    newBmsData.lowest_cell_voltage = -1.0f;
    newBmsData.highest_temperature = -1.0f;
    newBmsData.lowest_temperature = -1.0f;
    newBmsData.voltage_differential = -1.0f;
    newBmsData.mos_temperature = -1.0f;
    newBmsData.balance_temperature = -1.0f;
    newBmsData.t1_temperature = -1.0f;
    newBmsData.t2_temperature = -1.0f;
    newBmsData.t3_temperature = -1.0f;
    newBmsData.t4_temperature = -1.0f;
    
    // Update the global telemetry data thread-safely
    updateBMSDataThreadSafe(newBmsData);
    
    // Update unified battery data with sentinel values
    UnifiedBatteryData newUnifiedData = {};
    newUnifiedData.volts = -1.0f;
    newUnifiedData.amps = -1.0f;
    newUnifiedData.soc = -1.0f;
    newUnifiedData.power = -1.0f;
    updateUnifiedBatteryDataThreadSafe(newUnifiedData);
    
    return;
  }

  // TODO track bms incrementing cycle count
  // Ensure display CS is deselected and BMS CS is selected
  digitalWrite(displayCS, HIGH);
  digitalWrite(bmsCS, LOW);

  // USBSerial.println("Updating BMS Data");
  bms_can->update();

  // Create a local copy of the telemetry data
  STR_BMS_TELEMETRY_140 newBmsData = {};

  // Basic measurements
  newBmsData.battery_voltage = bms_can->getBatteryVoltage();
  newBmsData.battery_current = bms_can->getBatteryCurrent();
  newBmsData.soc = bms_can->getSOC();
  newBmsData.power = bms_can->getPower();

  // Cell voltages
  newBmsData.highest_cell_voltage = bms_can->getHighestCellVoltage();
  newBmsData.lowest_cell_voltage = bms_can->getLowestCellVoltage();

  // Calculated highest cell minus lowest cell voltage
  newBmsData.voltage_differential = bms_can->getHighestCellVoltage() - bms_can->getLowestCellVoltage();

  // Temperature readings
  newBmsData.highest_temperature = bms_can->getHighestTemperature();
  newBmsData.lowest_temperature = bms_can->getLowestTemperature();

  // Battery statistics
  newBmsData.battery_cycle = bms_can->getBatteryCycle();
  newBmsData.energy_cycle = bms_can->getEnergyCycle();
  newBmsData.battery_failure_level = bms_can->getBatteryFailureLevel();

  // Battery status
  newBmsData.is_charging = bms_can->isBatteryCharging();
  newBmsData.is_charge_mos = bms_can->isChargeMOSStatus();
  newBmsData.is_discharge_mos = bms_can->isDischargeMOSStatus();

  // Set connection state
  newBmsData.bmsState = TelemetryState::CONNECTED;

  // Populate individual cell voltages
  for (uint8_t i = 0; i < BMS_CELLS_NUM; i++) {
    newBmsData.cell_voltages[i] = bms_can->getCellVoltage(i);
  }

  // Populate individual temperature sensors
  newBmsData.mos_temperature = bms_can->getTemperature(0);      // BMS MOSFET
  newBmsData.balance_temperature = bms_can->getTemperature(1);  // BMS Balance resistors
  newBmsData.t1_temperature = bms_can->getTemperature(2);       // Cell probe 1
  newBmsData.t2_temperature = bms_can->getTemperature(3);       // Cell probe 2
  newBmsData.t3_temperature = bms_can->getTemperature(4);       // Cell probe 3
  newBmsData.t4_temperature = bms_can->getTemperature(5);       // Cell probe 4

  newBmsData.lastUpdateMs = millis();

  // Update the global telemetry data thread-safely
  updateBMSDataThreadSafe(newBmsData);

  // Update unified battery data thread-safely
  UnifiedBatteryData newUnifiedData = {};
  newUnifiedData.volts = newBmsData.battery_voltage;
  newUnifiedData.amps = newBmsData.battery_current;
  newUnifiedData.soc = newBmsData.soc;
  newUnifiedData.power = newBmsData.power;
  updateUnifiedBatteryDataThreadSafe(newUnifiedData);

  // Deselect BMS CS when done
  digitalWrite(bmsCS, HIGH);
}

void printBMSData() {
  // Check if BMS is currently connected and the object exists
  if (bms_can == nullptr || !bmsCanInitialized) return;


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
  USBSerial.print(bms_can->getHighestCellVoltage(), 3);
  USBSerial.println(" V");

  USBSerial.print("Lowest Cell Voltage: ");
  USBSerial.print(bms_can->getLowestCellVoltage(), 3);
  USBSerial.println(" V");

  USBSerial.print("Voltage Differential: ");
  USBSerial.print(bms_can->getHighestCellVoltage() - bms_can->getLowestCellVoltage(), 3);
  USBSerial.println(" V");

  USBSerial.print("Discharge MOS Status: ");
  USBSerial.println(bms_can->isDischargeMOSStatus() ? "OPEN" : "CLOSED");

  USBSerial.print("Charge MOS Status: ");
  USBSerial.println(bms_can->isChargeMOSStatus() ? "OPEN" : "CLOSED");

  USBSerial.print("Battery Charging: ");
  USBSerial.println(bms_can->isBatteryCharging() ? "YES" : "NO");

  // Log each temperature sensor value
  // The first 2 are for the BMS (mosfet and balance resistors)
  // The next 4 are for the cell probes
  for (uint8_t i = 0; i < 6; i++) {
    USBSerial.print("Temperature Sensor ");
    USBSerial.print(i);
    USBSerial.print(": ");
    USBSerial.print(bms_can->getTemperature(i));
    USBSerial.println(" °C");
  }

  // Print all cell voltages
  for (uint8_t i = 0; i < 24; i++) {
    USBSerial.print("Cell ");
    USBSerial.print(i + 1);
    USBSerial.print(": ");
    USBSerial.print(bms_can->getCellVoltage(i), 3);
    USBSerial.println(" V");
  }
}
