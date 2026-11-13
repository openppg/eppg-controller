#include "host_can.h"

#include <algorithm>
#include <deque>

#include "Adafruit_MCP2515.h"
#include "driver/twai.h"

namespace {
std::deque<host_can::Frame> mcp_rx;
std::deque<host_can::Frame> twai_rx;
std::vector<host_can::Frame> twai_tx;
}  // namespace

namespace host_can {

void reset() {
  mcp_rx.clear();
  twai_rx.clear();
  twai_tx.clear();
}

void enqueueMcpRx(const Frame& frame) { mcp_rx.push_back(frame); }
void enqueueTwaiRx(const Frame& frame) { twai_rx.push_back(frame); }

std::vector<Frame> takeTwaiTx() {
  std::vector<Frame> result;
  result.swap(twai_tx);
  return result;
}

}  // namespace host_can

Adafruit_MCP2515::Adafruit_MCP2515(std::int8_t, SPIClass*) {}
Adafruit_MCP2515::Adafruit_MCP2515(std::int8_t, std::int8_t, std::int8_t,
                                   std::int8_t) {}
int Adafruit_MCP2515::begin(long) { return 1; }

int Adafruit_MCP2515::parsePacket() {
  if (mcp_rx.empty()) return 0;
  active_ = mcp_rx.front();
  mcp_rx.pop_front();
  read_index_ = 0;
  return active_.len;
}

std::uint32_t Adafruit_MCP2515::packetId() const { return active_.id; }

int Adafruit_MCP2515::read() {
  if (read_index_ >= active_.len) return -1;
  return active_.data[read_index_++];
}

extern "C" esp_err_t twai_read_alerts(uint32_t* alerts, TickType_t) {
  *alerts = twai_rx.empty() ? 0 : TWAI_ALERT_RX_DATA;
  return ESP_OK;
}

extern "C" esp_err_t twai_get_status_info(twai_status_info_t* status) {
  *status = {};
  status->msgs_to_rx = static_cast<uint32_t>(twai_rx.size());
  return ESP_OK;
}

extern "C" esp_err_t twai_receive(twai_message_t* message, TickType_t) {
  if (twai_rx.empty()) return ESP_FAIL;
  const host_can::Frame frame = twai_rx.front();
  twai_rx.pop_front();
  *message = {};
  message->identifier = frame.id;
  message->data_length_code = frame.len;
  message->extd = frame.extended;
  message->rtr = frame.rtr;
  std::copy_n(frame.data.begin(), frame.len, message->data);
  return ESP_OK;
}

extern "C" esp_err_t twai_transmit(const twai_message_t* message,
                                    TickType_t) {
  host_can::Frame frame;
  frame.id = message->identifier;
  frame.len = message->data_length_code;
  frame.extended = message->extd;
  frame.rtr = message->rtr;
  std::copy_n(message->data, frame.len, frame.data.begin());
  twai_tx.push_back(frame);
  return ESP_OK;
}

