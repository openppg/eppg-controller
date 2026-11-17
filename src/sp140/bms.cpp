#include "sp140/bms.h"
#include <Adafruit_SPIDevice.h>
#include "sp140/bms_can_rx.h"
#include "sp140/structs.h"
#include "sp140/globals.h"
#include "sp140/lvgl/lvgl_core.h"  // for spiBusMutex
#include "sp140/time_utils.h"

namespace {

void logBmsCellProbeConnectionTransitions(const float sanitizedCellTemps[BMS_CELL_PROBE_COUNT]) {
  static bool hasPreviousState = false;
  static bool wasConnected[BMS_CELL_PROBE_COUNT] = {false, false, false, false};

  for (uint8_t i = 0; i < BMS_CELL_PROBE_COUNT; i++) {
    const bool connected = !isnan(sanitizedCellTemps[i]);

    if (!hasPreviousState) {
      wasConnected[i] = connected;
      continue;
    }

    if (connected != wasConnected[i]) {
      USBSerial.printf("[BMS] T%u sensor %s (sanitized=%.1fC)\n",
                       i + 1,
                       connected ? "reconnected" : "disconnected",
                       sanitizedCellTemps[i]);
      wasConnected[i] = connected;
    }
  }

  hasPreviousState = true;
}

}  // namespace

STR_BMS_TELEMETRY_140 bmsTelemetryData = {
  .bmsState = TelemetryState::NOT_CONNECTED
};

// Initialize bms_can as nullptr, to be set in initBMSCAN
BMS_CAN* bms_can = nullptr;

// These are defined in sp140.ino
extern int8_t displayCS;
extern int8_t bmsCS;

// BMS initialization is now handled in sp140.ino

// Second handle on the BMS MCP2515 for the receive registers BMS_CAN does not
// expose. Same chip select and SPI settings as the library's own device.
static Adafruit_SPIDevice* bmsMcpRegs = nullptr;
// Receive buffers found overflowed; each means at least one BMS frame dropped.
static uint32_t bmsRxOverflows = 0;

bool configureBMSCANReceive(SPIClass* spi) {
  bmsMcpRegs = new Adafruit_SPIDevice(bmsCS, 10000000, SPI_BITORDER_MSBFIRST, SPI_MODE0, spi);
  return bmsCanRxEnableRollover(*bmsMcpRegs);
}

// Update BMS data and populate unified battery data
void updateBMSData() {
  // Check if BMS is currently connected and the object exists
  if (bms_can == nullptr || !bmsCanInitialized) return;

  // TODO track bms incrementing cycle count
  // Take the shared SPI mutex to prevent contention with TFT flush
  if (spiBusMutex != NULL) {
    if (xSemaphoreTake(spiBusMutex, pdMS_TO_TICKS(150)) != pdTRUE) {
      // SPI bus timeout - display might be doing long operation
      USBSerial.println("[BMS] SPI bus timeout - skipping BMS update cycle");
      return;  // Use stale BMS data this cycle rather than hang
    }
  }
  // Ensure display CS is deselected and BMS CS is selected. This must happen
  // only AFTER the bus mutex is held: writing displayCS while a TFT flush is
  // streaming pixels deselects the panel mid-transfer and the rest of that
  // flush is silently lost, leaving stale bands on screen.
  digitalWrite(displayCS, HIGH);
  digitalWrite(bmsCS, LOW);

  // USBSerial.println("Updating BMS Data");
  unsigned long tStart = timeMillis();
  bms_can->update();
  // The MCP2515 holds two frames between these 100 ms drains; any more are
  // dropped by the chip. That cannot be prevented from here, so count it.
  const uint8_t rxOverflows =
      (bmsMcpRegs != nullptr) ? bmsCanRxTakeOverflows(*bmsMcpRegs) : 0;

  // All SPI traffic happens above — the getters below only read
  // values the library cached in RAM. Release the bus first so a pending
  // display flush waits only for the CAN drain, not the whole copy-out.
  digitalWrite(bmsCS, HIGH);
  if (spiBusMutex != NULL) {
    xSemaphoreGive(spiBusMutex);
  }

  if (rxOverflows > 0) {
    bmsRxOverflows += rxOverflows;
    static unsigned long lastOverflowLogMs = 0;
    if (lastOverflowLogMs == 0 || tStart - lastOverflowLogMs >= 10000) {
      USBSerial.printf("[BMS] CAN receive overflow, frames dropped (%lu total)\n",
                       static_cast<unsigned long>(bmsRxOverflows));
      lastOverflowLogMs = tStart;
    }
  }

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

  // Battery statistics
  bmsTelemetryData.battery_cycle = bms_can->getBatteryCycle();
  bmsTelemetryData.energy_cycle_ah = bms_can->getEnergyCycle();
  bmsTelemetryData.battery_fail_level = bms_can->getBatteryFailureLevel();

  // Battery status
  bmsTelemetryData.is_charging = bms_can->isBatteryCharging();
  bmsTelemetryData.is_charge_mos = bms_can->isChargeMOSStatus();
  bmsTelemetryData.is_discharge_mos = bms_can->isDischargeMOSStatus();
  bmsTelemetryData.charge_wire_connected = bms_can->isChargeWireConnected();
  bmsTelemetryData.low_soc_warning = bms_can->isBatteryLowSOC();
  bmsTelemetryData.battery_ready = bms_can->isBatteryReady();

  // BMS type (auto-detected from cell voltage endianness)
  bmsTelemetryData.bms_type = (uint8_t)bms_can->getBmsType();

  // Battery identity string
  String bid = bms_can->getBatteryID();
  strncpy(bmsTelemetryData.battery_id, bid.c_str(), sizeof(bmsTelemetryData.battery_id) - 1);
  bmsTelemetryData.battery_id[sizeof(bmsTelemetryData.battery_id) - 1] = '\0';

  // Populate individual cell voltages
  for (uint8_t i = 0; i < BMS_CELLS_NUM; i++) {
    bmsTelemetryData.cell_voltages[i] = bms_can->getCellVoltage(i);
  }

  // Populate temperature sensors (library returns NaN for disconnected probes)
  bmsTelemetryData.mos_temperature = bms_can->getTemperature(0);      // BMS MOSFET
  bmsTelemetryData.balance_temperature = bms_can->getTemperature(1);  // BMS Balance resistors

  // Cell probes: library already returns NaN for disconnected. Apply app-level
  // policy (tolerate up to N disconnected before alerting).
  const float cellTemps[BMS_CELL_PROBE_COUNT] = {
    bms_can->getTemperature(2),
    bms_can->getTemperature(3),
    bms_can->getTemperature(4),
    bms_can->getTemperature(5)
  };
  // Latched disconnect policy (see applyCellProbeTempPolicy in bms.h).
  static bool tempFrameSeen = false;
  float sanitizedCellTemps[BMS_CELL_PROBE_COUNT];
  if (applyCellProbeTempPolicy(tempFrameSeen,
                               bmsTelemetryData.mos_temperature,
                               bmsTelemetryData.balance_temperature,
                               cellTemps, sanitizedCellTemps)) {
    // Emit transition logs to help field-debug intermittent probe wiring
    // issues. Only once real temp data exists — otherwise every boot would
    // log a spurious "reconnected" burst when the first frame arrives.
    logBmsCellProbeConnectionTransitions(sanitizedCellTemps);
  }
  bmsTelemetryData.t1_temperature = sanitizedCellTemps[0];
  bmsTelemetryData.t2_temperature = sanitizedCellTemps[1];
  bmsTelemetryData.t3_temperature = sanitizedCellTemps[2];
  bmsTelemetryData.t4_temperature = sanitizedCellTemps[3];

  // Library already excludes disconnected probes from high/low temps
  bmsTelemetryData.highest_temperature = bms_can->getHighestTemperature();
  bmsTelemetryData.lowest_temperature = bms_can->getLowestTemperature();

  bmsTelemetryData.lastUpdateMs = timeMillis();
  unsigned long dur = bmsTelemetryData.lastUpdateMs - tStart;
  if (dur > 80) {  // warn if BMS update is taking longer than a frame
    USBSerial.print("Warn: BMS update slow ");
    USBSerial.print(dur);
    USBSerial.println("ms");
  }
}
