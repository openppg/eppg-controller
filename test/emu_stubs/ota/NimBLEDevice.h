// Minimal NimBLE-Arduino 2.x surface used by ota_service.cpp. Characteristics
// remember their properties and every notify/indicate payload; emuWrite()
// plays the part of a central's ATT write, including NimBLE's WRITE_ENC check.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "emu_ota_harness.h"

// Values of BLE_GATT_CHR_F_* in NimBLE host/ble_gatt.h.
namespace NIMBLE_PROPERTY {
constexpr uint32_t BROADCAST = 0x0001;
constexpr uint32_t READ = 0x0002;
constexpr uint32_t WRITE_NR = 0x0004;
constexpr uint32_t WRITE = 0x0008;
constexpr uint32_t NOTIFY = 0x0010;
constexpr uint32_t INDICATE = 0x0020;
constexpr uint32_t READ_ENC = 0x0200;
constexpr uint32_t READ_AUTHEN = 0x0400;
constexpr uint32_t READ_AUTHOR = 0x0800;
constexpr uint32_t WRITE_ENC = 0x1000;
constexpr uint32_t WRITE_AUTHEN = 0x2000;
constexpr uint32_t WRITE_AUTHOR = 0x4000;
}  // namespace NIMBLE_PROPERTY

class NimBLEUUID {
 public:
  NimBLEUUID() = default;
  NimBLEUUID(const char* s) : str_(s ? s : "") {}  // NOLINT(runtime/explicit)
  const std::string& toString() const { return str_; }
  // "00008022-0000-1000-8000-00805f9b34fb" -> "8022"
  std::string shortName() const {
    if (str_.size() == 36 && str_.compare(0, 4, "0000") == 0) return str_.substr(4, 4);
    return str_;
  }
  bool operator==(const NimBLEUUID& o) const { return str_ == o.str_; }

 private:
  std::string str_;
};

class NimBLEConnInfo {
 public:
  explicit NimBLEConnInfo(bool encrypted = true) : encrypted_(encrypted) {}
  bool isEncrypted() const { return encrypted_; }
  bool isBonded() const { return encrypted_; }
  uint16_t getConnHandle() const { return 1; }

 private:
  bool encrypted_;
};

class NimBLECharacteristic;

class NimBLECharacteristicCallbacks {
 public:
  virtual ~NimBLECharacteristicCallbacks() = default;
  virtual void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) {
    (void)pCharacteristic;
    (void)connInfo;
  }
  virtual void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) {
    (void)pCharacteristic;
    (void)connInfo;
  }
};

class NimBLECharacteristic {
 public:
  NimBLECharacteristic(const NimBLEUUID& uuid, uint32_t properties)
      : uuid_(uuid), properties_(properties) {}

  void setCallbacks(NimBLECharacteristicCallbacks* cb) { callbacks_ = cb; }
  NimBLECharacteristicCallbacks* getCallbacks() const { return callbacks_; }
  const NimBLEUUID& getUUID() const { return uuid_; }
  uint32_t getProperties() const { return properties_; }

  void setValue(const uint8_t* data, size_t length) {
    value_.assign(reinterpret_cast<const char*>(data), length);
  }
  std::string getValue() const { return value_; }

  bool notify() {
    notifications.emplace_back(value_.begin(), value_.end());
    emu::recordEvent("notify " + uuid_.shortName());
    return true;
  }
  bool indicate() {
    notifications.emplace_back(value_.begin(), value_.end());
    emu::recordEvent("indicate " + uuid_.shortName());
    return true;
  }

  // A central writes `bytes`. Like NimBLE's ATT server, a characteristic with
  // WRITE_ENC rejects the write (insufficient encryption) on an unencrypted
  // link before any application callback runs. Returns false when rejected.
  bool emuWrite(const std::vector<uint8_t>& bytes, bool encryptedLink = true) {
    if ((properties_ & (NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR)) == 0) return false;
    if ((properties_ & NIMBLE_PROPERTY::WRITE_ENC) != 0 && !encryptedLink) {
      emu::recordEvent("att_reject_insufficient_enc " + uuid_.shortName());
      return false;
    }
    value_.assign(bytes.begin(), bytes.end());
    NimBLEConnInfo info(encryptedLink);
    if (callbacks_ != nullptr) callbacks_->onWrite(this, info);
    return true;
  }

  std::vector<std::vector<uint8_t>> notifications;

 private:
  NimBLEUUID uuid_;
  uint32_t properties_;
  NimBLECharacteristicCallbacks* callbacks_ = nullptr;
  std::string value_;
};

class NimBLEService {
 public:
  explicit NimBLEService(const NimBLEUUID& uuid) : uuid_(uuid) {}

  NimBLECharacteristic* createCharacteristic(
      const NimBLEUUID& uuid,
      uint32_t properties = NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE,
      uint16_t maxLen = 512) {
    (void)maxLen;
    characteristics.push_back(std::make_unique<NimBLECharacteristic>(uuid, properties));
    return characteristics.back().get();
  }
  bool start() {
    started = true;
    return true;
  }
  const NimBLEUUID& getUUID() const { return uuid_; }

  NimBLECharacteristic* find(const NimBLEUUID& uuid) const {
    for (const auto& c : characteristics) {
      if (c->getUUID() == uuid) return c.get();
    }
    return nullptr;
  }

  std::vector<std::unique_ptr<NimBLECharacteristic>> characteristics;
  bool started = false;

 private:
  NimBLEUUID uuid_;
};

class NimBLEServer {
 public:
  NimBLEService* createService(const NimBLEUUID& uuid) {
    services.push_back(std::make_unique<NimBLEService>(uuid));
    return services.back().get();
  }
  std::vector<std::unique_ptr<NimBLEService>> services;
};
