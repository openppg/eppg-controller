#include "sp140/mode.h"
#include "sp140/buzzer.h"
#include "sp140/structs.h"
#include "sp140/device_state.h"
#include "sp140/ble.h"

// Platform-specific hardware config
#include "sp140/esp32s3-config.h"

// External globals
extern STR_DEVICE_DATA_140_V1 deviceData;
extern volatile DeviceState currentState;

// Forward declaration for function in other files
void writeDeviceData();

/**
 * Switches between performance modes (Chill/Sport)
 *
 * @param update_display Whether to update the display after switching modes
 */
void perfModeSwitch() {
  // 0=CHILL 1=SPORT 2=LUDICROUS?!
  if (deviceData.performance_mode == 0) {
    deviceData.performance_mode = 1;
  } else {
    deviceData.performance_mode = 0;
  }

  if (currentState == DISARMED) {
    writeDeviceData();
  }

  // Update BLE characteristic and notify if connected
  if (pPerformanceModeCharacteristic != nullptr) {
    int performanceMode = deviceData.performance_mode;
    pPerformanceModeCharacteristic->setValue(performanceMode);

    if (deviceConnected) {
      pPerformanceModeCharacteristic->notify();
    }
  }

  uint16_t notify_melody[] = { 900, 1976 };
  playMelody(notify_melody, 2);
}
