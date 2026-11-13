#include "platform/ble.h"

#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include "esp_bt.h"
#include "esp_log.h"
extern "C" {
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_hs_pvcy.h"
#include "host/ble_sm.h"
#include "host/ble_store.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
void ble_store_config_init(void);
}

namespace platform {
namespace {
constexpr char kTag[] = "controller_ble";
BleServer* gServer = nullptr;
}
BleUuid::BleUuid(const char* text) {
  if (!text || ble_uuid_from_str(&value_, text) != 0)
    throw std::invalid_argument("Invalid GATT UUID");
}
BleUuid::BleUuid(uint16_t value) {
  value_.u16.u.type = BLE_UUID_TYPE_16;
  value_.u16.value = value;
}
std::string BleAddress::toString() const {
  char text[18];
  std::snprintf(text, sizeof(text), "%02x:%02x:%02x:%02x:%02x:%02x",
      address_.val[5], address_.val[4], address_.val[3],
      address_.val[2], address_.val[1], address_.val[0]);
  return text;
}
BleConnection::BleConnection(uint16_t handle) {
  desc_.conn_handle = handle;
  ble_gap_conn_find(handle, &desc_);
}
BleCharacteristic::BleCharacteristic(BleUuid uuid, uint32_t properties)
    : uuid_(uuid), properties_(properties) {}
void BleCharacteristic::setValue(const uint8_t* bytes, size_t size) {
  if ((!bytes && size) || size > BLE_ATT_ATTR_MAX_LEN) {
    ESP_LOGE(kTag, "Invalid GATT value length %u", static_cast<unsigned>(size));
    return;
  }
  std::lock_guard<std::mutex> guard(mutex_);
  if (!size) value_.clear();
  else value_.assign(reinterpret_cast<const char*>(bytes), size);
}
std::string BleCharacteristic::getValue() const {
  std::lock_guard<std::mutex> guard(mutex_);
  return value_;
}
void BleCharacteristic::subscribe(BleConnection& connection, uint16_t flags) {
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!flags) subscriptions_.erase(connection.getConnHandle());
    else subscriptions_[connection.getConnHandle()] = flags;
  }
  if (callbacks_) callbacks_->onSubscribe(this, connection, flags);
}
void BleCharacteristic::disconnect(uint16_t connection) {
  std::lock_guard<std::mutex> guard(mutex_);
  subscriptions_.erase(connection);
}
bool BleCharacteristic::notify() {
  std::string bytes;
  std::map<uint16_t, uint16_t> subscribers;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    bytes = value_;
    subscribers = subscriptions_;
  }
  bool sent = false;
  for (const auto& subscription : subscribers) {
    BleConnection connection(subscription.first);
    if ((properties_ & (BleProperty::READ_ENC | BleProperty::WRITE_ENC)) &&
        !connection.isEncrypted()) continue;
    const auto mtu = ble_att_mtu(subscription.first);
    if (mtu < 3 || bytes.size() > static_cast<size_t>(mtu - 3)) continue;
    auto* buffer = ble_hs_mbuf_from_flat(bytes.data(), bytes.size());
    if (!buffer) continue;
    // Both APIs consume the mbuf, including on failure.
    const int rc = (subscription.second & 1)
        ? ble_gatts_notify_custom(subscription.first, handle_, buffer)
        : ble_gatts_indicate_custom(subscription.first, handle_, buffer);
    sent = (rc == 0) || sent;
  }
  return sent;
}
int BleCharacteristic::access(uint16_t handle, uint16_t,
    ble_gatt_access_ctxt* context, void* argument) {
  auto* self = static_cast<BleCharacteristic*>(argument);
  BleConnection connection(handle);
  if (context->op == BLE_GATT_ACCESS_OP_READ_CHR) {
    if (self->callbacks_) self->callbacks_->onRead(self, connection);
    const auto bytes = self->getValue();
    return os_mbuf_append(context->om, bytes.data(), bytes.size()) == 0
        ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }
  if (context->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
    const uint16_t length = OS_MBUF_PKTLEN(context->om);
    if (length > BLE_ATT_ATTR_MAX_LEN) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    std::string bytes(length, '\0');
    if (os_mbuf_copydata(context->om, 0, length, bytes.data()) != 0)
      return BLE_ATT_ERR_UNLIKELY;
    self->setValue(bytes);
    if (self->callbacks_) self->callbacks_->onWrite(self, connection);
    return 0;
  }
  return BLE_ATT_ERR_UNLIKELY;
}
BleCharacteristic* BleService::createCharacteristic(BleUuid uuid, uint32_t properties) {
  characteristics_.push_back(std::make_unique<BleCharacteristic>(uuid, properties));
  return characteristics_.back().get();
}
BleServer::BleServer() { gServer = this; }
BleService* BleServer::createService(BleUuid uuid) {
  if (registered_) return nullptr;
  services_.push_back(std::make_unique<BleService>(uuid));
  return services_.back().get();
}
bool BleServer::start(const std::string& name) {
  if (registered_) return true;
  if (nimble_port_init() != ESP_OK) return false;
  name_ = name;
  ble_hs_cfg.sync_cb = onSync;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
  ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
  ble_hs_cfg.sm_bonding = 1;
  ble_hs_cfg.sm_mitm = 0;
  ble_hs_cfg.sm_sc = 1;
  ble_hs_cfg.sm_sc_only = 1;
  ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  ble_svc_gap_init();
  ble_svc_gatt_init();
  ble_svc_gap_device_name_set(name.c_str());
  ble_att_set_preferred_mtu(517);
  ble_gap_set_prefered_default_le_phy(BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
  ble_store_config_init();
  for (auto& service : services_) {
    service->definitions_.reserve(service->characteristics_.size() + 1);
    for (auto& characteristic : service->characteristics_) {
      ble_gatt_chr_def definition{};
      definition.uuid = characteristic->uuid_.native();
      definition.access_cb = BleCharacteristic::access;
      definition.arg = characteristic.get();
      definition.flags = characteristic->properties_;
      definition.val_handle = &characteristic->handle_;
      service->definitions_.push_back(definition);
    }
    service->definitions_.push_back({});
    ble_gatt_svc_def definition{};
    definition.type = BLE_GATT_SVC_TYPE_PRIMARY;
    definition.uuid = service->uuid_.native();
    definition.characteristics = service->definitions_.data();
    definitions_.push_back(definition);
  }
  definitions_.push_back({});
  if (ble_gatts_count_cfg(definitions_.data()) || ble_gatts_add_svcs(definitions_.data())) {
    ESP_LOGE(kTag, "Cannot register GATT services");
    nimble_port_deinit();
    return false;
  }
  registered_ = true;
  nimble_port_freertos_init(hostTask);
  return true;
}
void BleServer::hostTask(void*) {
  nimble_port_run();
  nimble_port_freertos_deinit();
}
void BleServer::onSync() {
  if (!gServer || ble_hs_util_ensure_addr(0) ||
      ble_hs_id_infer_auto(0, &gServer->address_type_)) return;
  ble_hs_pvcy_set_resolve_enabled(1);
  gServer->ready_.store(true);
}
size_t BleServer::bondCount() const {
  ble_addr_t peers[CONFIG_BT_NIMBLE_MAX_BONDS];
  int count = 0;
  if (ble_store_util_bonded_peers(peers, &count, CONFIG_BT_NIMBLE_MAX_BONDS)) return 0;
  return count;
}
bool BleServer::clearBonds() { return ble_store_clear() == 0; }
bool BleServer::deleteBond(const BleAddress& address) {
  return ble_store_util_delete_peer(address.native()) == 0;
}
bool BleServer::advertise(BleUuid service, bool openPairing) {
  if (!ready()) return false;
  stopAdvertising();
  ble_addr_t peers[CONFIG_BT_NIMBLE_MAX_BONDS];
  int count = 0;
  if (ble_store_util_bonded_peers(peers, &count, CONFIG_BT_NIMBLE_MAX_BONDS)) return false;
  if (!openPairing && !count) return false;
  if (ble_gap_wl_set(count ? peers : nullptr, count)) return false;
  ble_hs_adv_fields fields{};
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  const auto* uuid = service.native();
  if (uuid->type == BLE_UUID_TYPE_128) {
    fields.uuids128 = const_cast<ble_uuid128_t*>(reinterpret_cast<const ble_uuid128_t*>(uuid));
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
  } else if (uuid->type == BLE_UUID_TYPE_16) {
    fields.uuids16 = const_cast<ble_uuid16_t*>(reinterpret_cast<const ble_uuid16_t*>(uuid));
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;
  }
  if (ble_gap_adv_set_fields(&fields)) return false;
  ble_hs_adv_fields response{};
  response.name = reinterpret_cast<const uint8_t*>(name_.data());
  response.name_len = std::min<size_t>(name_.size(), 29);
  response.name_is_complete = name_.size() <= 29;
  if (ble_gap_adv_rsp_set_fields(&response)) return false;
  ble_gap_adv_params params{};
  params.conn_mode = BLE_GAP_CONN_MODE_UND;
  params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  params.itvl_min = 32;
  params.itvl_max = 48;
  params.filter_policy = openPairing ? BLE_HCI_ADV_FILT_NONE : BLE_HCI_ADV_FILT_CONN;
  return ble_gap_adv_start(address_type_, nullptr, BLE_HS_FOREVER, &params, gapEvent, this) == 0;
}
void BleServer::stopAdvertising() { if (ready()) ble_gap_adv_stop(); }
bool BleServer::isAdvertising() const { return ready() && ble_gap_adv_active(); }
void BleServer::disconnect(uint16_t handle) {
  ble_gap_terminate(handle, BLE_ERR_REM_USER_CONN_TERM);
}
void BleServer::updateConnParams(uint16_t handle, uint16_t min, uint16_t max,
                                uint16_t latency, uint16_t timeout) {
  ble_gap_upd_params params{};
  params.itvl_min = min;
  params.itvl_max = max;
  params.latency = latency;
  params.supervision_timeout = timeout;
  ble_gap_update_params(handle, &params);
}
int BleServer::gapEvent(ble_gap_event* event, void* argument) {
  auto* self = static_cast<BleServer*>(argument);
  if (!self) return 0;
  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
      if (!event->connect.status && self->callbacks_) {
        BleConnection connection(event->connect.conn_handle);
        self->callbacks_->onConnect(self, connection);
      }
      break;
    case BLE_GAP_EVENT_DISCONNECT: {
      for (auto& service : self->services_)
        for (auto& characteristic : service->characteristics_)
          characteristic->disconnect(event->disconnect.conn.conn_handle);
      if (self->callbacks_) {
        BleConnection connection(event->disconnect.conn);
        self->callbacks_->onDisconnect(self, connection, event->disconnect.reason);
      }
      break;
    }
    case BLE_GAP_EVENT_ENC_CHANGE:
      if (self->callbacks_) {
        BleConnection connection(event->enc_change.conn_handle);
        self->callbacks_->onAuthenticationComplete(connection);
      }
      break;
    case BLE_GAP_EVENT_IDENTITY_RESOLVED:
      if (self->callbacks_) {
        BleConnection connection(event->identity_resolved.conn_handle);
        self->callbacks_->onIdentity(connection);
      }
      break;
    case BLE_GAP_EVENT_MTU:
      if (self->callbacks_) {
        BleConnection connection(event->mtu.conn_handle);
        self->callbacks_->onMTUChange(event->mtu.value, connection);
      }
      break;
    case BLE_GAP_EVENT_SUBSCRIBE:
      for (auto& service : self->services_)
        for (auto& characteristic : service->characteristics_)
          if (characteristic->handle() == event->subscribe.attr_handle) {
            BleConnection connection(event->subscribe.conn_handle);
            characteristic->subscribe(connection, event->subscribe.cur_notify |
                (event->subscribe.cur_indicate << 1));
          }
      break;
    case BLE_GAP_EVENT_REPEAT_PAIRING:
      if (self->pairingAllowed()) {
        BleConnection connection(event->repeat_pairing.conn_handle);
        self->deleteBond(connection.getIdAddress());
        return BLE_GAP_REPEAT_PAIRING_RETRY;
      }
      return BLE_GAP_REPEAT_PAIRING_IGNORE;
    default: break;
  }
  return 0;
}
}
