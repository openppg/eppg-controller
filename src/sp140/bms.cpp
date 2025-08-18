#include "sp140/bms.h"
#include "sp140/structs.h"
#include "sp140/globals.h"
#include "sp140/lvgl/lvgl_core.h"  // for spiBusMutex

STR_BMS_TELEMETRY_140 bmsTelemetryData = {
  .bmsState = TelemetryState::NOT_CONNECTED
};

// Initialize bms_can as nullptr, to be set in initBMSCAN
BMS_CAN* bms_can = nullptr;

// These are defined in sp140.ino
extern int8_t displayCS;
extern int8_t bmsCS;

// BMS initialization is now handled in sp140.ino

// Update BMS data and populate unified battery data
void updateBMSData() {
  // Check if BMS is currently connected and the object exists
  if (bms_can == nullptr || !bmsCanInitialized) return;

  // TODO track bms incrementing cycle count
  // Ensure display CS is deselected and BMS CS is selected
  digitalWrite(displayCS, HIGH);
  // Take the shared SPI mutex to prevent contention with TFT flush
  if (spiBusMutex != NULL) {
    if (xSemaphoreTake(spiBusMutex, pdMS_TO_TICKS(150)) != pdTRUE) {
      // SPI bus timeout - display might be doing long operation
      USBSerial.println("[BMS] SPI bus timeout - skipping BMS update cycle");
      return; // Use stale BMS data this cycle rather than hang
    }
  }
  digitalWrite(bmsCS, LOW);

  // USBSerial.println("Updating BMS Data");
  unsigned long tStart = millis();
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

  // Battery status
  bmsTelemetryData.is_charging = bms_can->isBatteryCharging();
  bmsTelemetryData.is_charge_mos = bms_can->isChargeMOSStatus();
  bmsTelemetryData.is_discharge_mos = bms_can->isDischargeMOSStatus();

  // Populate individual cell voltages
  for (uint8_t i = 0; i < BMS_CELLS_NUM; i++) {
    bmsTelemetryData.cell_voltages[i] = bms_can->getCellVoltage(i);
  }

  // Populate individual temperature sensors
  bmsTelemetryData.mos_temperature = bms_can->getTemperature(0);      // BMS MOSFET
  bmsTelemetryData.balance_temperature = bms_can->getTemperature(1);  // BMS Balance resistors
  bmsTelemetryData.t1_temperature = bms_can->getTemperature(2);       // Cell probe 1
  bmsTelemetryData.t2_temperature = bms_can->getTemperature(3);       // Cell probe 2
  bmsTelemetryData.t3_temperature = bms_can->getTemperature(4);       // Cell probe 3
  bmsTelemetryData.t4_temperature = bms_can->getTemperature(5);       // Cell probe 4

  bmsTelemetryData.lastUpdateMs = millis();
  unsigned long dur = bmsTelemetryData.lastUpdateMs - tStart;
  if (dur > 80) { // warn if BMS update is taking longer than a frame
    USBSerial.print("Warn: BMS update slow ");
    USBSerial.print(dur);
    USBSerial.println("ms");
  }
  // printBMSData();

  // Deselect BMS CS when done and release mutex
  digitalWrite(bmsCS, HIGH);
  if (spiBusMutex != NULL) {
    xSemaphoreGive(spiBusMutex);
  }
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
