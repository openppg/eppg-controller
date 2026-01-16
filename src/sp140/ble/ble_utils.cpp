#include "sp140/ble/ble_utils.h"

/**
 * Specialization for uint8_t that uses pointer-based setValue (for binary data).
 */
bool setAndNotifyOnChange(NimBLECharacteristic* characteristic, uint8_t newValue, uint8_t& lastValue) {
  if (characteristic == nullptr) {
    return false;
  }

  characteristic->setValue(&newValue, sizeof(newValue));

  if (newValue != lastValue) {
    lastValue = newValue;
    if (deviceConnected) {
      characteristic->notify();
    }
    return true;  // Value changed
  }
  return false;  // No change
}
