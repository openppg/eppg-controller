#ifndef INC_SP140_WIRELESS_SETTINGS_H_
#define INC_SP140_WIRELESS_SETTINGS_H_

#include <ArduinoJson.h>
#include <cstdint>

enum class WirelessRole : uint8_t {
  STANDALONE = 0,
  REMOTE = 1,
  RECEIVER = 2,
};

struct WirelessSettings {
  WirelessRole role;
  uint8_t peer_mac[6];
  uint8_t pairing_secret[16];
  uint8_t channel;
  bool peer_configured;
  bool secret_configured;
};

extern WirelessSettings wirelessSettings;

void loadWirelessSettings();
bool applyWirelessSettingsJson(JsonObjectConst settings);
const char* wirelessRoleName(WirelessRole role);
void formatWirelessPeerMac(char* output, size_t output_size);
void formatWirelessLocalMac(char* output, size_t output_size);

inline bool wirelessIsRemote() {
  return wirelessSettings.role == WirelessRole::REMOTE;
}

inline bool wirelessIsReceiver() {
  return wirelessSettings.role == WirelessRole::RECEIVER;
}

#endif  // INC_SP140_WIRELESS_SETTINGS_H_
