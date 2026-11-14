// Drives the production ESC command path (escQueueThrottle, CanardAdapter,
// SineEsc, libcanard) through CAN bus faults against a host TWAI driver model.
#include <gtest/gtest.h>

#include <CanardAdapter.h>
#include <SineEsc.h>

#include <deque>
#include <vector>

#include "driver/twai.h"
#include "sp140/esc_command.h"

namespace {

// Models the legacy ESP-IDF TWAI driver as the command path sees it:
//  - one frame sits in the controller's transmit buffer and is retried until a
//    node acknowledges it; up to `queueLength` more wait behind it;
//  - msgs_to_tx counts both; twai_clear_transmit_queue() drops only the waiting
//    frames, never the one already in the transmit buffer;
//  - outside RUNNING, twai_transmit() is refused;
//  - initiating bus-off recovery discards everything queued.
struct FakeTwai {
  twai_state_t state = TWAI_STATE_RUNNING;
  bool acknowledged = true;
  size_t queueLength = 5;  // TWAI_GENERAL_CONFIG_DEFAULT tx_queue_len
  std::deque<twai_message_t> pending;  // front() is in the transmit buffer
  std::vector<twai_message_t> wire;    // frames a node acknowledged, in order
  int recoveriesInitiated = 0;
  int starts = 0;

  void drain() {
    if (state != TWAI_STATE_RUNNING || !acknowledged) return;
    for (const auto& frame : pending) wire.push_back(frame);
    pending.clear();
  }
  void setAcknowledged(bool value) {
    acknowledged = value;
    drain();
  }
};

FakeTwai bus;

uint16_t commandOf(const twai_message_t& frame) {
  // SetThrottleSettings2 request: uint16 little-endian, then the tail byte.
  EXPECT_EQ(frame.data_length_code, 3);
  return static_cast<uint16_t>(frame.data[0] | (frame.data[1] << 8));
}

std::vector<uint16_t> commandsOnWire() {
  std::vector<uint16_t> commands;
  for (const auto& frame : bus.wire) commands.push_back(commandOf(frame));
  return commands;
}

}  // namespace

extern "C" esp_err_t twai_read_alerts(uint32_t* alerts, TickType_t) {
  *alerts = 0;
  return ESP_OK;
}
extern "C" esp_err_t twai_get_status_info(twai_status_info_t* status) {
  *status = {};
  status->state = bus.state;
  status->msgs_to_tx = static_cast<uint32_t>(bus.pending.size());
  return ESP_OK;
}
extern "C" esp_err_t twai_receive(twai_message_t*, TickType_t) { return ESP_FAIL; }
extern "C" esp_err_t twai_transmit(const twai_message_t* message, TickType_t) {
  if (bus.state != TWAI_STATE_RUNNING) return ESP_ERR_INVALID_STATE;
  if (bus.pending.size() >= bus.queueLength + 1) return ESP_ERR_TIMEOUT;
  bus.pending.push_back(*message);
  bus.drain();
  return ESP_OK;
}
extern "C" esp_err_t twai_clear_transmit_queue(void) {
  if (bus.pending.size() > 1) bus.pending.erase(bus.pending.begin() + 1, bus.pending.end());
  return ESP_OK;
}
extern "C" esp_err_t twai_initiate_recovery(void) {
  if (bus.state != TWAI_STATE_BUS_OFF) return ESP_ERR_INVALID_STATE;
  bus.state = TWAI_STATE_RECOVERING;
  bus.pending.clear();
  bus.recoveriesInitiated++;
  return ESP_OK;
}
extern "C" esp_err_t twai_start(void) {
  if (bus.state != TWAI_STATE_STOPPED) return ESP_ERR_INVALID_STATE;
  bus.state = TWAI_STATE_RUNNING;
  bus.starts++;
  return ESP_OK;
}

// One controller with the production transport stack, as esc.cpp builds it.
class EscCommandPath : public ::testing::Test {
 protected:
  void SetUp() override {
    bus = FakeTwai{};
    adapter.begin(pool, sizeof(pool));
    adapter.setLocalNodeId(0x01);
    esc.begin(0x20);
  }

  // One 20 ms control cycle: handleThrottle() queues the command, then
  // readESCTelemetry() pumps the bus and services bus-off recovery.
  void cycle(int pwm, bool busReady = true) {
    escQueueThrottle(busReady, adapter, esc, stall, pwm);
    adapter.processTxRxOnce();
    switch (recovery.step(busState())) {
      case EscBusAction::INITIATE_RECOVERY:
        twai_initiate_recovery();
        break;
      case EscBusAction::START:
        if (twai_start() == ESP_OK) recovery.started();
        break;
      case EscBusAction::NONE:
        break;
    }
  }

  static EscBusState busState() {
    switch (bus.state) {
      case TWAI_STATE_STOPPED: return EscBusState::STOPPED;
      case TWAI_STATE_BUS_OFF: return EscBusState::BUS_OFF;
      case TWAI_STATE_RECOVERING: return EscBusState::RECOVERING;
      default: return EscBusState::RUNNING;
    }
  }

  alignas(8) uint8_t pool[1024];  // same size as esc.cpp's memory_pool
  CanardAdapter adapter;
  SineEsc esc{adapter};
  EscTxStall stall;
  EscBusRecovery recovery;
};

TEST(EscCommandPolicy, ThrottleIsNeverWithheldWhileArmed) {
  const bool armed = false, disarmed = true;
  EXPECT_TRUE(escThrottleCommandAllowed(armed, false));
  EXPECT_TRUE(escThrottleCommandAllowed(armed, true))
      << "a relay request queued in flight must not cost a throttle frame";
  EXPECT_TRUE(escThrottleCommandAllowed(disarmed, false));
  EXPECT_FALSE(escThrottleCommandAllowed(disarmed, true))
      << "the bus is kept quiet for a relay session on the ground";
}

TEST(EscCommandPolicy, PwmRange) {
  EXPECT_FALSE(escThrottlePwmValid(999));
  EXPECT_TRUE(escThrottlePwmValid(1000));
  EXPECT_TRUE(escThrottlePwmValid(2000));
  EXPECT_FALSE(escThrottlePwmValid(2001));
  EXPECT_FALSE(escThrottlePwmValid(-1));
}

TEST(EscCommandPolicy, StallNeedsTwoConsecutivePendingCycles) {
  EscTxStall stall;
  EXPECT_FALSE(stall.update(false));
  EXPECT_FALSE(stall.update(true));
  EXPECT_FALSE(stall.update(false)) << "one slow cycle is not a stall";
  EXPECT_FALSE(stall.update(true));
  EXPECT_TRUE(stall.update(true));
  for (int i = 0; i < 1000; i++) EXPECT_TRUE(stall.update(true));
  EXPECT_FALSE(stall.update(false));
  EXPECT_FALSE(stall.update(true)) << "a fresh backlog starts a new window";
}

TEST(EscCommandPolicy, BusOffRecoverySequence) {
  EscBusRecovery recovery;
  EXPECT_EQ(recovery.step(EscBusState::RUNNING), EscBusAction::NONE);
  EXPECT_EQ(recovery.step(EscBusState::BUS_OFF), EscBusAction::INITIATE_RECOVERY);
  EXPECT_EQ(recovery.step(EscBusState::BUS_OFF), EscBusAction::NONE)
      << "recovery is initiated once";
  EXPECT_EQ(recovery.step(EscBusState::RECOVERING), EscBusAction::NONE);
  EXPECT_EQ(recovery.step(EscBusState::STOPPED), EscBusAction::START);
  EXPECT_EQ(recovery.step(EscBusState::STOPPED), EscBusAction::START)
      << "a failed start is retried";
  recovery.started();
  EXPECT_FALSE(recovery.pending());
  EXPECT_EQ(recovery.step(EscBusState::RUNNING), EscBusAction::NONE);
}

TEST(EscCommandPolicy, DriverStoppedOnPurposeIsNotRestarted) {
  EscBusRecovery recovery;
  EXPECT_EQ(recovery.step(EscBusState::STOPPED), EscBusAction::NONE);
  EXPECT_EQ(recovery.step(EscBusState::STOPPED), EscBusAction::NONE);
}

TEST_F(EscCommandPath, HealthyBusSendsEachCommandInItsOwnCycle) {
  for (int pwm = 1035; pwm < 1235; pwm += 10) cycle(pwm);
  const auto sent = commandsOnWire();
  ASSERT_EQ(sent.size(), 20u);
  for (size_t i = 0; i < sent.size(); i++) EXPECT_EQ(sent[i], (1035 + 10 * i) * 10);
  EXPECT_TRUE(bus.pending.empty());
  EXPECT_FALSE(adapter.hasPendingTx());
}

TEST_F(EscCommandPath, InvalidOrEarlyCommandsQueueNothing) {
  EXPECT_FALSE(escQueueThrottle(false, adapter, esc, stall, 1500))
      << "TWAI driver failed to install";
  EXPECT_FALSE(escQueueThrottle(true, adapter, esc, stall, 999));
  EXPECT_FALSE(escQueueThrottle(true, adapter, esc, stall, 2001));
  EXPECT_FALSE(adapter.hasPendingTx());
  EXPECT_TRUE(escQueueThrottle(true, adapter, esc, stall, 1500));
  EXPECT_TRUE(adapter.hasPendingTx());
}

TEST_F(EscCommandPath, SilentEscDoesNotGetOldCommandsReplayed) {
  cycle(1035);
  bus.wire.clear();

  // ESC unpowered or connector open: nothing acknowledges. Five seconds at power.
  bus.setAcknowledged(false);
  for (int i = 0; i < 250; i++) cycle(1900);
  EXPECT_LE(bus.pending.size(), 2u) << "driver backlog must stay bounded";

  // The pilot has released the throttle by the time the ESC comes back.
  cycle(1035);
  cycle(1035);
  bus.setAcknowledged(true);

  // The frame already in the controller's transmit buffer when the ESC went
  // silent cannot be withdrawn, so it still goes out first. Nothing else older
  // than the current command may follow it.
  const auto sent = commandsOnWire();
  ASSERT_FALSE(sent.empty());
  ASSERT_LE(sent.size(), 2u) << "old commands were replayed";
  EXPECT_EQ(sent.back(), 10350) << "the last command the ESC acts on is current";

  // And the path is healthy again afterwards.
  bus.wire.clear();
  cycle(1100);
  cycle(1110);
  EXPECT_EQ(commandsOnWire(), (std::vector<uint16_t>{11000, 11100}));
}

TEST_F(EscCommandPath, BusOffRecoversAndSendsOnlyTheCurrentCommand) {
  cycle(1500);
  bus.wire.clear();

  // Shorted or severed pair drives the controller bus-off mid-flight.
  bus.state = TWAI_STATE_BUS_OFF;
  cycle(1800);
  EXPECT_EQ(bus.recoveriesInitiated, 1);
  for (int i = 0; i < 100; i++) cycle(1800);
  EXPECT_EQ(bus.recoveriesInitiated, 1);
  EXPECT_EQ(bus.starts, 0);

  // The driver finishes recovery once it has seen an idle bus.
  bus.state = TWAI_STATE_STOPPED;
  cycle(1040);
  EXPECT_EQ(bus.starts, 1);
  EXPECT_EQ(bus.state, TWAI_STATE_RUNNING);
  EXPECT_TRUE(bus.wire.empty());

  cycle(1040);
  const auto sent = commandsOnWire();
  ASSERT_FALSE(sent.empty()) << "no command reached the ESC after recovery";
  for (uint16_t command : sent) EXPECT_EQ(command, 10400) << "a bus-off era command was replayed";

  bus.wire.clear();
  cycle(1200);
  EXPECT_EQ(commandsOnWire(), (std::vector<uint16_t>{12000}));
}

TEST_F(EscCommandPath, LongOutageLeavesRoomForTheCurrentCommand) {
  // The transport allocates from a 1 KiB pool. If rejected commands piled up
  // there, new ones would be refused once it ran out and the ESC would get a
  // minute-old command first when the bus returned.
  bus.state = TWAI_STATE_BUS_OFF;
  for (int i = 0; i < 3000; i++) {
    escQueueThrottle(true, adapter, esc, stall, 1700);
    adapter.processTxRxOnce();
  }
  bus.state = TWAI_STATE_RUNNING;
  cycle(1035);
  EXPECT_EQ(commandsOnWire(), (std::vector<uint16_t>{10350}));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
