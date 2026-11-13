#pragma once

// Native IDF NimBLE application objects. No Arduino host or runtime required.
#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <vector>
extern "C" {
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
}

namespace platform {
namespace BleProperty {
constexpr uint32_t READ = BLE_GATT_CHR_F_READ;
constexpr uint32_t WRITE = BLE_GATT_CHR_F_WRITE;
constexpr uint32_t WRITE_NR = BLE_GATT_CHR_F_WRITE_NO_RSP;
constexpr uint32_t NOTIFY = BLE_GATT_CHR_F_NOTIFY;
constexpr uint32_t INDICATE = BLE_GATT_CHR_F_INDICATE;
constexpr uint32_t READ_ENC = BLE_GATT_CHR_F_READ_ENC;
constexpr uint32_t WRITE_ENC = BLE_GATT_CHR_F_WRITE_ENC;
}

class BleUuid {
 public:
  explicit BleUuid(const char* text);
  explicit BleUuid(const std::string& text) : BleUuid(text.c_str()) {}
  explicit BleUuid(uint16_t value);
  const ble_uuid_t* native() const { return &value_.u; }

 private:
  ble_uuid_any_t value_{};
};
class BleAddress {
 public:
  explicit BleAddress(const ble_addr_t& address) : address_(address) {}
  std::string toString() const;
  const ble_addr_t* native() const { return &address_; }

 private:
  ble_addr_t address_{};
};
class BleConnection {
 public:
  explicit BleConnection(uint16_t handle);
  explicit BleConnection(const ble_gap_conn_desc& desc) : desc_(desc) {}
  uint16_t getConnHandle() const { return desc_.conn_handle; }
  bool isBonded() const { return desc_.sec_state.bonded; }
  bool isEncrypted() const { return desc_.sec_state.encrypted; }
  uint8_t getSecKeySize() const { return desc_.sec_state.key_size; }
  BleAddress getAddress() const { return BleAddress(desc_.peer_ota_addr); }
  BleAddress getIdAddress() const { return BleAddress(desc_.peer_id_addr); }

 private:
  ble_gap_conn_desc desc_{};
};
class BleCharacteristic;
class BleServer;
class BleCharacteristicCallbacks {
 public:
  virtual ~BleCharacteristicCallbacks() = default;
  virtual void onRead(BleCharacteristic*, BleConnection&) {}
  virtual void onWrite(BleCharacteristic*, BleConnection&) {}
  virtual void onSubscribe(BleCharacteristic*, BleConnection&, uint16_t) {}
};
class BleServerCallbacks {
 public:
  virtual ~BleServerCallbacks() = default;
  virtual void onConnect(BleServer*, BleConnection&) {}
  virtual void onDisconnect(BleServer*, BleConnection&, int) {}
  virtual void onAuthenticationComplete(BleConnection&) {}
  virtual void onIdentity(BleConnection&) {}
  virtual void onMTUChange(uint16_t, BleConnection&) {}
};
class BleCharacteristic {
 public:
  BleCharacteristic(BleUuid uuid, uint32_t properties);
  void setValue(const uint8_t* bytes, size_t size);
  void setValue(const std::string& value) {
    setValue(reinterpret_cast<const uint8_t*>(value.data()), value.size());
  }
  void setValue(const char* value) { setValue(std::string(value)); }
  template <typename T, typename = std::enable_if_t<std::is_arithmetic<T>::value>>
  void setValue(const T& value) {
    setValue(reinterpret_cast<const uint8_t*>(&value), sizeof(value));
  }
  std::string getValue() const;
  bool notify();
  void setCallbacks(BleCharacteristicCallbacks* callbacks) { callbacks_ = callbacks; }
  uint16_t handle() const { return handle_; }
  static int access(uint16_t connection, uint16_t attribute,
                    ble_gatt_access_ctxt* context, void* argument);

 private:
  friend class BleService;
  friend class BleServer;
  void subscribe(BleConnection& connection, uint16_t flags);
  void disconnect(uint16_t connection);
  BleUuid uuid_;
  uint32_t properties_;
  uint16_t handle_ = 0;
  BleCharacteristicCallbacks* callbacks_ = nullptr;
  mutable std::mutex mutex_;
  std::string value_;
  std::map<uint16_t, uint16_t> subscriptions_;
};
class BleService {
 public:
  explicit BleService(BleUuid uuid) : uuid_(uuid) {}
  BleCharacteristic* createCharacteristic(BleUuid uuid, uint32_t properties);
  void start() {}  // Registration happens atomically with the complete server.

 private:
  friend class BleServer;
  BleUuid uuid_;
  std::vector<std::unique_ptr<BleCharacteristic>> characteristics_;
  std::vector<ble_gatt_chr_def> definitions_;
};
class BleServer {
 public:
  BleServer();
  BleService* createService(BleUuid uuid);
  void setCallbacks(BleServerCallbacks* callbacks) { callbacks_ = callbacks; }
  bool start(const std::string& name);
  bool advertise(BleUuid service, bool openPairing);
  void stopAdvertising();
  bool isAdvertising() const;
  void disconnect(uint16_t handle);
  void updateConnParams(uint16_t handle, uint16_t min, uint16_t max,
                        uint16_t latency, uint16_t timeout);
  size_t bondCount() const;
  bool clearBonds();
  bool deleteBond(const BleAddress& address);
  bool ready() const { return ready_.load(); }
  bool pairingAllowed() const { return pairing_allowed_.load(); }
  void setPairingAllowed(bool allowed) { pairing_allowed_.store(allowed); }
  static int gapEvent(ble_gap_event* event, void* argument);

 private:
  static void onSync();
  static void hostTask(void* argument);
  std::vector<std::unique_ptr<BleService>> services_;
  std::vector<ble_gatt_svc_def> definitions_;
  BleServerCallbacks* callbacks_ = nullptr;
  std::string name_;
  std::atomic<bool> ready_{false};
  std::atomic<bool> pairing_allowed_{false};
  uint8_t address_type_ = BLE_OWN_ADDR_PUBLIC;
  bool registered_ = false;
};
}  // namespace platform
