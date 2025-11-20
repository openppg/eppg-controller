#ifndef INC_SP140_BLE_BLE_UTILS_H_
#define INC_SP140_BLE_BLE_UTILS_H_

#include <BLECharacteristic.h>

// Forward declare deviceConnected from ble.h
extern bool deviceConnected;

/**
 * Sets a BLE characteristic value and conditionally notifies only when the value changes.
 * Returns true if the value changed and a notification was sent, false otherwise.
 *
 * @param characteristic The BLE characteristic to update
 * @param newValue The new value to set
 * @param lastValue Reference to the variable tracking the previous value
 * @return true if value changed and notification sent, false if no change
 */
template<typename T>
bool setAndNotifyOnChange(BLECharacteristic* characteristic, T newValue, T& lastValue) {
  if (characteristic == nullptr) {
    return false;
  }

  characteristic->setValue(newValue);

  if (newValue != lastValue) {
    lastValue = newValue;
    if (deviceConnected) {
      characteristic->notify();
    }
    return true;  // Value changed
  }
  return false;  // No change
}

/**
 * Specialization for uint8_t that uses pointer-based setValue (for binary data).
 * Returns true if the value changed and a notification was sent, false otherwise.
 *
 * @param characteristic The BLE characteristic to update
 * @param newValue The new uint8_t value to set
 * @param lastValue Reference to the variable tracking the previous value
 * @return true if value changed and notification sent, false if no change
 */
bool setAndNotifyOnChange(BLECharacteristic* characteristic, uint8_t newValue, uint8_t& lastValue);

#endif  // INC_SP140_BLE_BLE_UTILS_H_
