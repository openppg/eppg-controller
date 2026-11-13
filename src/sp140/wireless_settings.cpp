#include "sp140/wireless_settings.h"

#include <Arduino.h>
#include <Preferences.h>
#include <esp_mac.h>
#include <cctype>
#include <cstdio>
#include <cstring>

namespace {

constexpr char kNamespace[] = "openppg-radio";
constexpr char kRoleKey[] = "role";
constexpr char kPeerKey[] = "peer";
constexpr char kSecretKey[] = "secret";
constexpr char kChannelKey[] = "channel";
constexpr uint8_t kDefaultChannel = 6;

int hexNibble(char value) {
  if (value >= '0' && value <= '9') return value - '0';
  value = static_cast<char>(std::tolower(static_cast<unsigned char>(value)));
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  return -1;
}

bool parseHexBytes(const char* text, uint8_t* output, size_t output_length,
                   bool allow_separators) {
  if (text == nullptr || output == nullptr) return false;

  size_t output_index = 0;
  int high_nibble = -1;
  for (const char* cursor = text; *cursor != '\0'; ++cursor) {
    if (allow_separators && (*cursor == ':' || *cursor == '-')) continue;
    const int nibble = hexNibble(*cursor);
    if (nibble < 0) return false;
    if (high_nibble < 0) {
      high_nibble = nibble;
    } else {
      if (output_index >= output_length) return false;
      output[output_index++] =
          static_cast<uint8_t>((high_nibble << 4) | nibble);
      high_nibble = -1;
    }
  }
  return high_nibble < 0 && output_index == output_length;
}

bool parseRole(const char* value, WirelessRole* role) {
  if (value == nullptr || role == nullptr) return false;
  if (strcmp(value, "disabled") == 0) {
    *role = WirelessRole::STANDALONE;
  } else if (strcmp(value, "remote") == 0) {
    *role = WirelessRole::REMOTE;
  } else if (strcmp(value, "receiver") == 0) {
    *role = WirelessRole::RECEIVER;
  } else {
    return false;
  }
  return true;
}

bool saveWirelessSettings() {
  Preferences preferences;
  if (!preferences.begin(kNamespace, false)) return false;

  bool ok = true;
  ok &= preferences.putUChar(kRoleKey,
                             static_cast<uint8_t>(wirelessSettings.role)) == 1;
  ok &= preferences.putUChar(kChannelKey, wirelessSettings.channel) == 1;
  if (wirelessSettings.peer_configured) {
    ok &= preferences.putBytes(kPeerKey, wirelessSettings.peer_mac,
                               sizeof(wirelessSettings.peer_mac)) ==
          sizeof(wirelessSettings.peer_mac);
  } else {
    preferences.remove(kPeerKey);
  }
  if (wirelessSettings.secret_configured) {
    ok &= preferences.putBytes(kSecretKey, wirelessSettings.pairing_secret,
                               sizeof(wirelessSettings.pairing_secret)) ==
          sizeof(wirelessSettings.pairing_secret);
  } else {
    preferences.remove(kSecretKey);
  }
  preferences.end();
  return ok;
}

}  // namespace

WirelessSettings wirelessSettings = {
    WirelessRole::STANDALONE, {0}, {0}, kDefaultChannel, false, false};

void loadWirelessSettings() {
  wirelessSettings = {
      WirelessRole::STANDALONE, {0}, {0}, kDefaultChannel, false, false};

  Preferences preferences;
  if (!preferences.begin(kNamespace, true)) return;

  const uint8_t stored_role = preferences.getUChar(kRoleKey, 0);
  if (stored_role <= static_cast<uint8_t>(WirelessRole::RECEIVER)) {
    wirelessSettings.role = static_cast<WirelessRole>(stored_role);
  }

  const uint8_t channel = preferences.getUChar(kChannelKey, kDefaultChannel);
  wirelessSettings.channel = (channel >= 1 && channel <= 13)
      ? channel
      : kDefaultChannel;

  wirelessSettings.peer_configured =
      preferences.getBytesLength(kPeerKey) == sizeof(wirelessSettings.peer_mac) &&
      preferences.getBytes(kPeerKey, wirelessSettings.peer_mac,
                           sizeof(wirelessSettings.peer_mac)) ==
          sizeof(wirelessSettings.peer_mac);
  wirelessSettings.secret_configured =
      preferences.getBytesLength(kSecretKey) ==
          sizeof(wirelessSettings.pairing_secret) &&
      preferences.getBytes(kSecretKey, wirelessSettings.pairing_secret,
                           sizeof(wirelessSettings.pairing_secret)) ==
          sizeof(wirelessSettings.pairing_secret);
  preferences.end();
}

bool applyWirelessSettingsJson(JsonObjectConst settings) {
  WirelessSettings candidate = wirelessSettings;

  if (!settings["role"].isNull() &&
      !parseRole(settings["role"].as<const char*>(), &candidate.role)) {
    return false;
  }
  if (!settings["peer"].isNull()) {
    candidate.peer_configured = parseHexBytes(
        settings["peer"].as<const char*>(), candidate.peer_mac,
        sizeof(candidate.peer_mac), true);
    if (!candidate.peer_configured) return false;
  }
  if (!settings["key"].isNull()) {
    candidate.secret_configured = parseHexBytes(
        settings["key"].as<const char*>(), candidate.pairing_secret,
        sizeof(candidate.pairing_secret), false);
    if (!candidate.secret_configured) return false;
  }
  if (!settings["channel"].isNull()) {
    const int channel = settings["channel"].as<int>();
    if (channel < 1 || channel > 13) return false;
    candidate.channel = static_cast<uint8_t>(channel);
  }

  if (candidate.role != WirelessRole::STANDALONE &&
      (!candidate.peer_configured || !candidate.secret_configured)) {
    return false;
  }

  wirelessSettings = candidate;
  return saveWirelessSettings();
}

const char* wirelessRoleName(WirelessRole role) {
  switch (role) {
    case WirelessRole::REMOTE:
      return "remote";
    case WirelessRole::RECEIVER:
      return "receiver";
    case WirelessRole::STANDALONE:
    default:
      return "disabled";
  }
}

void formatWirelessPeerMac(char* output, size_t output_size) {
  if (output == nullptr || output_size == 0) return;
  if (!wirelessSettings.peer_configured) {
    output[0] = '\0';
    return;
  }
  snprintf(output, output_size, "%02X:%02X:%02X:%02X:%02X:%02X",
           wirelessSettings.peer_mac[0], wirelessSettings.peer_mac[1],
           wirelessSettings.peer_mac[2], wirelessSettings.peer_mac[3],
           wirelessSettings.peer_mac[4], wirelessSettings.peer_mac[5]);
}

void formatWirelessLocalMac(char* output, size_t output_size) {
  if (output == nullptr || output_size == 0) return;
  uint8_t mac[6] = {};
  if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) {
    output[0] = '\0';
    return;
  }
  snprintf(output, output_size, "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
