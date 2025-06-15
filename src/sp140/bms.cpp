#include "sp140/bms.h"
#include "sp140/structs.h"
#include "sp140/globals.h"
#include "driver/twai.h"

STR_BMS_TELEMETRY_140 bmsTelemetryData = {
  .bmsState = TelemetryState::NOT_CONNECTED
};

// BMS data storage
static float soc = 0.0;
static float batteryVoltage = 0.0;
static float batteryCurrent = 0.0;
static float highestCellVoltage = 0.0;
static float lowestCellVoltage = 0.0;
static bool batteryCharging = false;
static bool chargeWireConnected = false;
static bool batteryLowSOC = false;
static bool batteryReady = false;
static bool dischargeMOSStatus = false;
static bool chargeMOSStatus = false;
static float highestTemperature = 0.0;
static float lowestTemperature = 0.0;
static uint32_t batteryCycle = 0;
static float energyCycle = 0.0;
static uint8_t batteryFailureLevel = 0;
static unsigned long lastBMSMessageTime = 0;

// Battery ID (24 bytes)
static char batteryID[24] = {0};

// Temperature readings (8 sensors)
static float temperatures[BMS_MAX_TEMPERATURES] = {0};

// Cell voltages (24 cells)
static float cellVoltages[BMS_MAX_CELLS] = {0};

// Mutex for thread safety
#ifdef FREERTOS_H
static SemaphoreHandle_t bmsMutex = NULL;
#endif

/**
 * Initialize BMS communication using shared TWAI bus
 * The TWAI bus should already be initialized by the ESC
 */
void initBMS() {
  USBSerial.println("Initializing BMS on shared TWAI bus...");

  // Check if TWAI is already initialized (should be by ESC)
  if (escTwaiInitialized) {
    bmsTwaiInitialized = true;
    USBSerial.println("BMS using shared TWAI bus - initialized successfully");
  } else {
    bmsTwaiInitialized = false;
    USBSerial.println("BMS initialization failed - TWAI not available");
  }

#ifdef FREERTOS_H
  if (bmsMutex == NULL) {
    bmsMutex = xSemaphoreCreateMutex();
  }
#endif

  // Initialize data
  lastBMSMessageTime = 0;
  memset(batteryID, 0, sizeof(batteryID));
  memset(temperatures, 0, sizeof(temperatures));
  memset(cellVoltages, 0, sizeof(cellVoltages));
}

/**
 * Check if this is a BMS message ID
 */
bool isBMSMessage(uint32_t id) {
  return (id == BMS_BASIC_INFO_1 || id == BMS_BASIC_INFO_2 || id == BMS_CYCLE_INFO ||
          id == BMS_TEMPERATURE ||
          (id >= BMS_BATTERY_ID_BASE && id <= (BMS_BATTERY_ID_BASE + 2)) ||
          ((id >> 24) == 0x18 && ((id >> 8) & 0xFF) == 0x28 &&
           ((id >> 16) & 0xFF) >= 0xC8 && ((id >> 16) & 0xFF) < (0xC8 + BMS_MAX_CELLS/4)));
}

/**
 * Parse BMS CAN packet and update internal data
 */
void parseBMSPacket(const twai_message_t* message) {
  if (message == NULL || message->data_length_code == 0) {
    return;
  }

#ifdef FREERTOS_H
  if (bmsMutex != NULL) {
    xSemaphoreTake(bmsMutex, portMAX_DELAY);
  }
#endif

  // Update last message time for any valid BMS packet
  lastBMSMessageTime = millis();

  uint32_t id = message->identifier;
  uint8_t* data = (uint8_t*)message->data;
  uint8_t len = message->data_length_code;

  if (id == BMS_BASIC_INFO_1 && len >= 7) {
    // BMS Basic Information 1: SOC, voltage, current, status flags
    chargeWireConnected = data[0] & 0x01;
    batteryCharging = data[0] & 0x02;
    batteryLowSOC = data[0] & 0x04;
    batteryReady = data[0] & 0x08;
    dischargeMOSStatus = data[0] & 0x10;
    chargeMOSStatus = data[0] & 0x20;

    soc = data[1];
    batteryCurrent = ((data[2] | (data[3] << 8)) - 5000) * 0.1;
    batteryVoltage = (data[4] | (data[5] << 8)) * 0.1;
    batteryFailureLevel = data[6];

  } else if (id == BMS_BASIC_INFO_2 && len >= 6) {
    // BMS Basic Information 2: cell voltages, temperatures
    highestCellVoltage = (data[0] | (data[1] << 8)) * 0.001;
    lowestCellVoltage = (data[2] | (data[3] << 8)) * 0.001;
    highestTemperature = data[4] - 40;
    lowestTemperature = data[5] - 40;

  } else if (id == BMS_CYCLE_INFO && len == 8) {
    // Battery Cycle Information
    batteryCycle = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
    uint32_t energyCycleRaw = (data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4];
    energyCycle = energyCycleRaw * 0.001; // Convert to kWh

  } else if (id >= BMS_BATTERY_ID_BASE && id <= (BMS_BATTERY_ID_BASE + 2) && len == 8) {
    // Battery ID Parsing (3 frames, 8 bytes each = 24 bytes total)
    int offset = (id - BMS_BATTERY_ID_BASE) * 8;
    if (offset >= 0 && offset < 24) {
      for (int i = 0; i < 8; i++) {
        batteryID[offset + i] = (char)data[i];
      }
    }

  } else if (id == BMS_TEMPERATURE && len == 8) {
    // Battery Temperature Parsing (8 temperature sensors)
    for (int i = 0; i < BMS_MAX_TEMPERATURES; i++) {
      temperatures[i] = (float)data[i] - 40.0;
    }

  } else {
    // Per-cell voltage frames: ID 0x18C828F4 + cell_group
    // Each frame contains 4 cell voltages (2 bytes each)
    uint8_t frameType = (id >> 24) & 0xFF;
    uint8_t pf = (id >> 16) & 0xFF;
    uint8_t sourceAddr = (id >> 8) & 0xFF;

    if (frameType == 0x18 && sourceAddr == 0x28 &&
        pf >= 0xC8 && pf < (0xC8 + BMS_MAX_CELLS/4) && len == 8) {

      int baseCell = (pf - 0xC8) * 4;
      for (int i = 0; i < 4; i++) {
        int cellIdx = baseCell + i;
        if (cellIdx < BMS_MAX_CELLS) {
          // Data is big-endian: first byte is high, second is low
          uint16_t voltage_mV = (data[i*2] << 8) | data[i*2+1];
          cellVoltages[cellIdx] = voltage_mV / 1000.0f;
        }
      }
    }
  }

#ifdef FREERTOS_H
  if (bmsMutex != NULL) {
    xSemaphoreGive(bmsMutex);
  }
#endif
}

/**
 * Check if BMS is connected based on recent message activity
 */
bool isBMSConnected() {
  if (!bmsTwaiInitialized || lastBMSMessageTime == 0) {
    return false;
  }

  return (millis() - lastBMSMessageTime) < BMS_TIMEOUT_MS;
}

/**
 * Update BMS telemetry structure with latest parsed data
 * Called by unified CAN bus after BMS messages are processed
 */
void updateBMSData() {
  if (!bmsTwaiInitialized) {
    return;
  }

  // Update connection status
  bool connected = isBMSConnected();
  if (connected != (bmsTelemetryData.bmsState == TelemetryState::CONNECTED)) {
    bmsTelemetryData.bmsState = connected ? TelemetryState::CONNECTED : TelemetryState::NOT_CONNECTED;
    USBSerial.printf("BMS State: %s\n", connected ? "CONNECTED" : "NOT_CONNECTED");
  }

  if (connected) {
    // Update telemetry structure with latest data
#ifdef FREERTOS_H
    if (bmsMutex != NULL) {
      xSemaphoreTake(bmsMutex, portMAX_DELAY);
    }
#endif

    bmsTelemetryData.battery_voltage = batteryVoltage;
    bmsTelemetryData.battery_current = batteryCurrent;
    bmsTelemetryData.soc = soc;
    bmsTelemetryData.power = (batteryVoltage * batteryCurrent) / 1000.0;
    bmsTelemetryData.highest_cell_voltage = highestCellVoltage;
    bmsTelemetryData.lowest_cell_voltage = lowestCellVoltage;
    bmsTelemetryData.voltage_differential = highestCellVoltage - lowestCellVoltage;
    bmsTelemetryData.highest_temperature = highestTemperature;
    bmsTelemetryData.lowest_temperature = lowestTemperature;
    bmsTelemetryData.battery_cycle = batteryCycle;
    bmsTelemetryData.energy_cycle = energyCycle;
    bmsTelemetryData.battery_failure_level = batteryFailureLevel;
    bmsTelemetryData.is_charging = batteryCharging;
    bmsTelemetryData.lastUpdateMs = millis();

    // Copy cell voltages
    for (uint8_t i = 0; i < BMS_CELLS_NUM; i++) {
      bmsTelemetryData.cell_voltages[i] = (i < BMS_MAX_CELLS) ? cellVoltages[i] : 0.0f;
    }

#ifdef FREERTOS_H
    if (bmsMutex != NULL) {
      xSemaphoreGive(bmsMutex);
    }
#endif
  }
}

void printBMSData() {
  if (!isBMSConnected()) {
    USBSerial.println("BMS not connected");
    return;
  }

#ifdef FREERTOS_H
  if (bmsMutex != NULL) {
    xSemaphoreTake(bmsMutex, portMAX_DELAY);
  }
#endif

  USBSerial.print("SOC: ");
  USBSerial.print(soc);
  USBSerial.println(" %");

  USBSerial.print("Battery Voltage: ");
  USBSerial.print(batteryVoltage);
  USBSerial.println(" V");

  USBSerial.print("Battery Current: ");
  USBSerial.print(batteryCurrent);
  USBSerial.println(" A");

  USBSerial.print("Power: ");
  USBSerial.print((batteryVoltage * batteryCurrent) / 1000.0);
  USBSerial.println(" kW");

  USBSerial.print("Highest Cell Voltage: ");
  USBSerial.print(highestCellVoltage, 3);
  USBSerial.println(" V");

  USBSerial.print("Lowest Cell Voltage: ");
  USBSerial.print(lowestCellVoltage, 3);
  USBSerial.println(" V");

  USBSerial.print("Voltage Differential: ");
  USBSerial.print(highestCellVoltage - lowestCellVoltage, 3);
  USBSerial.println(" V");

  USBSerial.print("Discharge MOS Status: ");
  USBSerial.println(dischargeMOSStatus ? "OPEN" : "CLOSED");

  USBSerial.print("Charge MOS Status: ");
  USBSerial.println(chargeMOSStatus ? "OPEN" : "CLOSED");

  USBSerial.print("Battery Charging: ");
  USBSerial.println(batteryCharging ? "YES" : "NO");

  // Print temperature sensor values
  for (uint8_t i = 0; i < 6; i++) {  // First 6 sensors
    USBSerial.print("Temperature Sensor ");
    USBSerial.print(i);
    USBSerial.print(": ");
    USBSerial.print(temperatures[i]);
    USBSerial.println(" °C");
  }

  // Print all cell voltages
  for (uint8_t i = 0; i < BMS_MAX_CELLS; i++) {
    if (cellVoltages[i] > 0.0) {  // Only print non-zero values
      USBSerial.print("Cell ");
      USBSerial.print(i + 1);
      USBSerial.print(": ");
      USBSerial.print(cellVoltages[i], 3);
      USBSerial.println(" V");
    }
  }

#ifdef FREERTOS_H
  if (bmsMutex != NULL) {
    xSemaphoreGive(bmsMutex);
  }
#endif
}
