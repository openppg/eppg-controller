#ifndef BMS_CAN_H
#define BMS_CAN_H

#include "platform/mcp2515.h"
#include "platform/runtime.h"

enum BmsType {
  BMS_TYPE_UNKNOWN = 0,
  BMS_TYPE_A = 1,  // Older: big-endian cell voltages, no cycle frames, no battery_id
  BMS_TYPE_B = 2,  // Newer: little-endian cell voltages, big-endian cycle count, has battery_id
  // BMS_TYPE_C = 3,  // Future: even newer variants
};

class BMS_CAN {
public:
  BMS_CAN(uint8_t csPin, long baudrate);

  bool begin();
  void update();
  // Transport-independent parser seam used by native protocol regression tests.
  void ingestFrame(uint32_t id, const uint8_t* data, uint8_t length);

  // Add connection status methods
  bool isConnected(unsigned long timeout_ms = 100);
  unsigned long getLastMessageTime();

  // BMS type detection (auto-detected from cell voltage endianness)
  BmsType getBmsType() const;

  float getSOC();
  float getBatteryVoltage();
  float getBatteryCurrent();
  float getPower(); // in kW
  float getHighestCellVoltage();
  float getLowestCellVoltage();
  bool isBatteryCharging();
  bool isChargeWireConnected();
  bool isBatteryLowSOC();
  bool isBatteryReady();
  bool isDischargeMOSStatus();
  bool isChargeMOSStatus();
  float getHighestTemperature();
  float getLowestTemperature();
  uint32_t getBatteryCycle();
  float getEnergyCycle();
  uint8_t getBatteryFailureLevel();

  // Add new method to get Battery ID
  std::string getBatteryID();

  // Temperature probe methods
  float getTemperature(uint8_t index);
  bool isTemperatureConnected(uint8_t index);
  uint8_t getConnectedTemperatureCount(uint8_t startIndex, uint8_t count);

  // Raw value reported by BMS when a temperature probe is disconnected
  static constexpr float TEMP_PROBE_DISCONNECTED = -40.0f;

  // Add support for up to 24 cell voltages
  enum { kMaxCells = 24 };

  // Add getter for cell voltage
  float getCellVoltage(uint8_t index);

private:
  platform::Mcp2515* mcp;
  long baudrate;

  float soc;
  float batteryVoltage;
  float batteryCurrent;
  float highestCellVoltage;
  float lowestCellVoltage;
  bool batteryCharging;
  bool chargeWireConnected;
  bool batteryLowSOC;
  bool batteryReady;
  bool dischargeMOSStatus;
  bool chargeMOSStatus;
  float highestTemperature;
  float lowestTemperature;
  uint32_t batteryCycle;
  float energyCycle;
  uint8_t batteryFailureLevel;

  // Add new member variable for Battery ID
  char batteryID[33]; // 32 characters + null terminator

  // Add new members for Battery Temperatures (simplified for max 8 sensors)
  enum { kMaxTemperatures = 8 }; // Only expect the first frame (8 sensors)
  float temperatures[kMaxTemperatures];

  // Cell voltages are private to enforce read-only access via getter
  float cellVoltages[kMaxCells];

  // Add timestamp tracking
  unsigned long lastMessageTime;


  void initializeMembers();

  BmsType bmsType;
  bool cellEndianDetected;
  bool cellVoltagesLittleEndian;

  // Mutex for thread safety
#ifdef FREERTOS_H
  SemaphoreHandle_t dataMutex;
#endif
};

#endif
