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
      return;  // Use stale BMS data this cycle rather than hang
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
  if (dur > 80) {  // warn if BMS update is taking longer than a frame
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

void printBMSMultiBatteryDebug() {
  // Check if BMS is currently connected and the object exists
  if (bms_can == nullptr || !bmsCanInitialized) return;

  // Ensure display CS is deselected and BMS CS is selected
  digitalWrite(displayCS, HIGH);
  // Take the shared SPI mutex to prevent contention with TFT flush
  if (spiBusMutex != NULL) {
    if (xSemaphoreTake(spiBusMutex, pdMS_TO_TICKS(150)) != pdTRUE) {
      // SPI bus timeout - display might be doing long operation
      USBSerial.println("[BMS Debug] SPI bus timeout - skipping debug output");
      return;
    }
  }
  digitalWrite(bmsCS, LOW);

  // Get the number of batteries discovered on the CAN bus
  uint8_t batteryCount = bms_can->getBatteryCount();

  USBSerial.println("========================================");
  USBSerial.print("Batteries Detected: ");
  USBSerial.println(batteryCount);
  USBSerial.println("========================================");

  if (batteryCount == 0) {
    USBSerial.println("No batteries detected. Check CAN bus connections.");
    USBSerial.println();
    digitalWrite(bmsCS, HIGH);
    if (spiBusMutex != NULL) {
      xSemaphoreGive(spiBusMutex);
    }
    return;
  }

  // Print individual battery data
  USBSerial.println("\n--- Individual Battery Data ---");
  for (uint8_t i = 0; i < batteryCount; i++) {
    // Check if this battery is currently connected (has sent data recently)
    if (!bms_can->isBatteryConnected(i, 1000)) {  // 1000ms timeout
      USBSerial.print("Battery ");
      USBSerial.print(i);
      USBSerial.println(": DISCONNECTED");
      continue;
    }

    // Get the Source Address for this battery
    uint8_t sourceAddress = bms_can->getBatterySourceAddress(i);

    USBSerial.println();
    USBSerial.print("Battery ");
    USBSerial.print(i);
    USBSerial.print(" (SA: 0x");
    USBSerial.print(sourceAddress, HEX);
    USBSerial.println("):");

    // Battery identification
    USBSerial.print("  ID: ");
    USBSerial.println(bms_can->getBatteryID(i));

    // Basic telemetry
    USBSerial.print("  SOC: ");
    USBSerial.print(bms_can->getSOC(i));
    USBSerial.println(" %");

    USBSerial.print("  Voltage: ");
    USBSerial.print(bms_can->getBatteryVoltage(i));
    USBSerial.println(" V");

    USBSerial.print("  Current: ");
    USBSerial.print(bms_can->getBatteryCurrent(i));
    USBSerial.println(" A");

    USBSerial.print("  Power: ");
    USBSerial.print(bms_can->getPower(i));
    USBSerial.println(" kW");

    // Cell voltage extremes
    USBSerial.print("  Highest Cell: ");
    USBSerial.print(bms_can->getHighestCellVoltage(i));
    USBSerial.println(" V");

    USBSerial.print("  Lowest Cell: ");
    USBSerial.print(bms_can->getLowestCellVoltage(i));
    USBSerial.println(" V");

    // Temperature extremes
    USBSerial.print("  Highest Temp: ");
    USBSerial.print(bms_can->getHighestTemperature(i));
    USBSerial.println(" °C");

    USBSerial.print("  Lowest Temp: ");
    USBSerial.print(bms_can->getLowestTemperature(i));
    USBSerial.println(" °C");

    // Status flags
    USBSerial.print("  Charging: ");
    USBSerial.println(bms_can->isBatteryCharging(i) ? "Yes" : "No");

    USBSerial.print("  Ready: ");
    USBSerial.println(bms_can->isBatteryReady(i) ? "Yes" : "No");

    // Lifecycle data
    USBSerial.print("  Cycles: ");
    USBSerial.println(bms_can->getBatteryCycle(i));

    USBSerial.print("  Energy Cycled: ");
    USBSerial.print(bms_can->getEnergyCycle(i));
    USBSerial.println(" Wh");
  }

  // Print aggregated system metrics
  USBSerial.println("\n--- Aggregated System Metrics ---");
  USBSerial.println("(Calculated across all connected batteries)");

  USBSerial.print("Average SOC: ");
  USBSerial.print(bms_can->getAverageSOC());
  USBSerial.println(" %");

  USBSerial.print("Total Voltage: ");
  USBSerial.print(bms_can->getTotalVoltage());
  USBSerial.println(" V");

  USBSerial.print("Total Current: ");
  USBSerial.print(bms_can->getTotalCurrent());
  USBSerial.println(" A");

  USBSerial.print("Total Power: ");
  USBSerial.print(bms_can->getTotalPower());
  USBSerial.println(" kW");

  // System-wide cell voltage extremes
  USBSerial.print("System Highest Cell Voltage: ");
  USBSerial.print(bms_can->getSystemHighestCellVoltage());
  USBSerial.println(" V");

  USBSerial.print("System Lowest Cell Voltage: ");
  USBSerial.print(bms_can->getSystemLowestCellVoltage());
  USBSerial.println(" V");

  // System-wide temperature extremes
  USBSerial.print("System Highest Temperature: ");
  USBSerial.print(bms_can->getSystemHighestTemperature());
  USBSerial.println(" °C");

  USBSerial.print("System Lowest Temperature: ");
  USBSerial.print(bms_can->getSystemLowestTemperature());
  USBSerial.println(" °C");

  USBSerial.println();

  // Deselect BMS CS when done and release mutex
  digitalWrite(bmsCS, HIGH);
  if (spiBusMutex != NULL) {
    xSemaphoreGive(spiBusMutex);
  }
}
