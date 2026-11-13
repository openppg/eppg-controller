#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>

#include "BMS_CAN.h"
#include "CanardAdapter.h"
#include "SineEsc.h"
#include "host_can.h"

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

void acceptNoTransfers(CanardInstance*, CanardRxTransfer*) {}
bool acceptNothing(const CanardInstance*, std::uint64_t*, std::uint16_t,
                   CanardTransferType, std::uint8_t) {
  return false;
}

void probeBms() {
  host_can::reset();
  BMS_CAN bms(5, 500000);
  require(bms.begin(), "fake MCP2515 transport should initialize");

  host_can::Frame basic;
  basic.id = 0x18FF28F4;
  basic.len = 8;
  // ready + discharge MOS, 73% SOC, -12.3 A, 52.4 V, failure level 2
  basic.data = {0x18, 73, 0x0D, 0x13, 0x0C, 0x02, 2, 0};
  host_can::enqueueMcpRx(basic);

  host_can::Frame limits;
  limits.id = 0x18FE28F4;
  limits.len = 8;
  limits.data = {0x0A, 0x0E, 0xDE, 0x0D, 85, 65, 0, 0};
  host_can::enqueueMcpRx(limits);

  host_can::Frame cells;
  cells.id = 0x18C828F4;
  cells.len = 8;
  cells.data = {0x0E, 0x0A, 0x0E, 0x08, 0x0E, 0x05, 0x0D, 0xDE};
  host_can::enqueueMcpRx(cells);

  bms.update();
  require(std::fabs(bms.getBatteryVoltage() - 52.4F) < 0.01F,
          "BMS voltage parser");
  require(std::fabs(bms.getBatteryCurrent() - (-12.3F)) < 0.01F,
          "BMS current parser");
  require(std::fabs(bms.getSOC() - 73.0F) < 0.01F, "BMS SOC parser");
  require(bms.isBatteryReady(), "BMS ready flag parser");
  require(std::fabs(bms.getCellVoltage(0) - 3.594F) < 0.001F,
          "BMS big-endian cell parser");
  require(bms.getBmsType() == BMS_TYPE_A, "BMS endianness/type detection");
  std::cout << "PASS: real ANT BMS library parsed queued MCP2515 frames\n";
}

void probeEsc() {
  host_can::reset();

  std::uint8_t controller_pool[4096]{};
  CanardAdapter adapter;
  adapter.begin(controller_pool, sizeof(controller_pool));
  adapter.setLocalNodeId(0x7F);
  SineEsc esc(adapter);
  esc.begin(0x20);

  esc.setThrottleSettings2(12345);
  adapter.processTxRxOnce();
  const auto throttle_frames = host_can::takeTwaiTx();
  require(!throttle_frames.empty(),
          "SINE ESC throttle request should reach fake TWAI queue");
  require(throttle_frames.front().extended, "DroneCAN frame must be extended");

  // Build a genuine multi-frame DroneCAN broadcast as if sent by ESC node 0x20.
  std::uint8_t esc_pool[4096]{};
  CanardInstance esc_canard{};
  canardInit(&esc_canard, esc_pool, sizeof(esc_pool), acceptNoTransfers,
             acceptNothing, nullptr);
  canardSetLocalNodeID(&esc_canard, 0x20);

  sine_esc_BroadcastStatusInfo status{};
  status.recv_pwm = 12000;
  status.comm_pwm = 11950;
  status.speed = 4321;
  status.current = 123;
  status.voltage = 518;
  status.mos_temp = 44;
  status.running_error = 0x0040;

  std::uint8_t payload[SINE_ESC_BROADCASTSTATUSINFO_MAX_SIZE]{};
  const std::uint32_t payload_len =
      sine_esc_BroadcastStatusInfo_encode(&status, payload);
  std::uint8_t transfer_id = 0;
  CanardTxTransfer transfer{};
  transfer.transfer_type = CanardTransferTypeBroadcast;
  transfer.data_type_signature = SINE_ESC_BROADCASTSTATUSINFO_SIGNATURE;
  transfer.data_type_id = SINE_ESC_BROADCASTSTATUSINFO_ID;
  transfer.inout_transfer_id = &transfer_id;
  transfer.priority = CANARD_TRANSFER_PRIORITY_MEDIUM;
  transfer.payload = payload;
  transfer.payload_len = static_cast<std::uint16_t>(payload_len);
  require(canardBroadcastObj(&esc_canard, &transfer) > 0,
          "ESC broadcast should serialize into DroneCAN frames");

  while (const CanardCANFrame* frame = canardPeekTxQueue(&esc_canard)) {
    host_can::Frame rx;
    rx.id = frame->id & CANARD_CAN_EXT_ID_MASK;
    rx.extended = (frame->id & CANARD_CAN_FRAME_EFF) != 0;
    rx.len = frame->data_len;
    for (std::uint8_t i = 0; i < rx.len; ++i) rx.data[i] = frame->data[i];
    host_can::enqueueTwaiRx(rx);
    canardPopTxQueue(&esc_canard);
  }

  adapter.processTxRxOnce();
  const SineEscModel& model = esc.getModel();
  require(model.hasBroadcastStatusInfo,
          "SINE ESC model should accept reassembled broadcast");
  require(model.broadcastStatusInfo.speed == 4321,
          "SINE ESC speed decode");
  require(model.broadcastStatusInfo.voltage == 518,
          "SINE ESC voltage decode");
  require(model.broadcastStatusInfo.running_error == 0x0040,
          "SINE ESC error decode");
  std::cout << "PASS: real SINE ESC/libcanard crossed fake TWAI TX/RX queues\n";
}

}  // namespace

int main() {
  probeBms();
  probeEsc();
  std::cout << "PASS: native CAN protocol seam is feasible\n";
  return 0;
}
