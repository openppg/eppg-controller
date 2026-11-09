#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <memory>
using NimBLEUUID = std::string;
struct NimBLEConnInfo {};
class NimBLECharacteristic;
class NimBLECharacteristicCallbacks {
 public:
  virtual ~NimBLECharacteristicCallbacks() = default;
  virtual void onWrite(NimBLECharacteristic*, NimBLEConnInfo&) {}
};
namespace NIMBLE_PROPERTY {
constexpr unsigned WRITE = 1, WRITE_NR = 2, NOTIFY = 4, INDICATE = 8;
}
class NimBLECharacteristic {
 public:
  std::string value;
  std::vector<std::string> notifications;
  NimBLECharacteristicCallbacks* callbacks = nullptr;
  std::string getValue() { return value; }
  void setValue(const uint8_t* bytes, size_t size) { value.assign(reinterpret_cast<const char*>(bytes), size); }
  void notify() { notifications.push_back(value); }
  void setCallbacks(NimBLECharacteristicCallbacks* cb) { callbacks = cb; }
  void write(const std::vector<uint8_t>& bytes) {
    setValue(bytes.data(), bytes.size());
    NimBLEConnInfo connection;
    callbacks->onWrite(this, connection);
  }
};
class NimBLEService {
 public:
  std::vector<std::unique_ptr<NimBLECharacteristic>> chars;
  NimBLECharacteristic* createCharacteristic(const NimBLEUUID&, unsigned) {
    chars.emplace_back(new NimBLECharacteristic);
    return chars.back().get();
  }
  void start() {}
};
class NimBLEServer {
 public:
  NimBLEService service;
  NimBLEService* createService(const NimBLEUUID&) { return &service; }
};
