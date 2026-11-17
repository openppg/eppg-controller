// Drives the production setESCThrottle() / readESCTelemetry() (esc.cpp), with
// the pinned SINE-ESC-CAN CanardAdapter, SineEsc and libcanard, through an ESC
// that stops acknowledging and comes back. The TWAI driver is modelled on the
// ESP-IDF 4.4.7 source this firmware links (framework-espidf 3.40407,
// components/driver/twai.c and components/hal/twai_hal.c):
//
//  - twai_transmit() loads the controller's single transmit buffer directly when
//    nothing is pending, otherwise appends to the TX queue (tx_queue_len 5);
//  - a frame in the transmit buffer is retried until a node acknowledges it;
//  - twai_clear_transmit_queue() empties the queue but leaves the frame in the
//    transmit buffer (twai.c: tx_msg_count = TX_BUFF_OCCUPIED ? 1 : 0);
//  - twai_stop() enters reset mode, which halts any transmission in progress
//    (twai_hal_stop: "Any TX is immediately halted on entering reset mode"),
//    and empties the queue; twai_start() empties both queues again;
//  - twai_initiate_recovery() is only valid from bus-off.
#include <gtest/gtest.h>

#include <deque>
#include <vector>

#include "Arduino.h"
#include "driver/twai.h"

#include "../../src/sp140/esc.cpp"

// Production globals and relay hooks esc.cpp links against (main.cpp and the
// relay modules on target). No relay session runs in these tests.
bool escTwaiInitialized = false;
float watts = 0;
void escConfigRelayInit() {}
void escConfigRelayServiceTick() {}
void escFlasherRelayInit() {}
void escFlasherRelayServiceTick() {}

namespace {

struct FakeTwai {
  twai_state_t state = TWAI_STATE_RUNNING;
  bool acknowledged = true;
  size_t queueLength = 5;           // TWAI_GENERAL_CONFIG_DEFAULT tx_queue_len
  bool txBufferOccupied = false;
  twai_message_t txBuffer = {};
  std::deque<twai_message_t> queue;
  std::vector<twai_message_t> wire;  // frames the ESC acknowledged, in order
  int stops = 0;
  int starts = 0;
  int recoveries = 0;

  uint32_t pending() const {
    return static_cast<uint32_t>(queue.size()) + (txBufferOccupied ? 1u : 0u);
  }

  // The bus: while a node acknowledges, the transmit buffer completes and the
  // TX interrupt loads the next queued frame (twai_handle_tx_buffer_frame).
  void run() {
    while (state == TWAI_STATE_RUNNING && acknowledged && txBufferOccupied) {
      wire.push_back(txBuffer);
      txBufferOccupied = false;
      if (!queue.empty()) {
        txBuffer = queue.front();
        queue.pop_front();
        txBufferOccupied = true;
      }
    }
  }

  void setAcknowledged(bool value) {
    acknowledged = value;
    run();
  }
};

FakeTwai bus;

uint16_t throttleCommandOf(const twai_message_t& frame) {
  // SetThrottleSettings2 request: uint16 pwm_us little-endian, then tail byte.
  EXPECT_EQ(frame.data_length_code, 3);
  return static_cast<uint16_t>(frame.data[0] | (frame.data[1] << 8));
}

std::vector<uint16_t> commandsOnWire() {
  std::vector<uint16_t> out;
  for (const auto& frame : bus.wire) out.push_back(throttleCommandOf(frame));
  return out;
}

// One 50 Hz control cycle as handleThrottle() runs it while armed.
void cycle(int pwm) {
  setESCThrottle(pwm);
  readESCTelemetry();
  emuAdvanceMillis(20);
}

}  // namespace

esp_err_t twai_driver_install(const twai_general_config_t*, const twai_timing_config_t*,
                              const twai_filter_config_t*) {
  return ESP_OK;
}
esp_err_t twai_driver_uninstall(void) { return ESP_OK; }
esp_err_t twai_reconfigure_alerts(uint32_t, uint32_t*) { return ESP_OK; }
esp_err_t twai_read_alerts(uint32_t* alerts, TickType_t) {
  *alerts = 0;
  return ESP_ERR_TIMEOUT;
}
esp_err_t twai_receive(twai_message_t*, TickType_t) { return ESP_ERR_TIMEOUT; }

esp_err_t twai_get_status_info(twai_status_info_t* status) {
  *status = {};
  status->state = bus.state;
  status->msgs_to_tx = bus.pending();
  return ESP_OK;
}

esp_err_t twai_transmit(const twai_message_t* message, TickType_t) {
  if (bus.state != TWAI_STATE_RUNNING) return ESP_ERR_INVALID_STATE;
  if (bus.pending() == 0) {
    bus.txBuffer = *message;
    bus.txBufferOccupied = true;
  } else if (bus.queue.size() < bus.queueLength) {
    bus.queue.push_back(*message);
  } else {
    return ESP_ERR_TIMEOUT;
  }
  bus.run();
  return ESP_OK;
}

esp_err_t twai_clear_transmit_queue(void) {
  bus.queue.clear();
  return ESP_OK;
}

esp_err_t twai_stop(void) {
  if (bus.state != TWAI_STATE_RUNNING) return ESP_ERR_INVALID_STATE;
  bus.txBufferOccupied = false;  // reset mode halts the transmission
  bus.queue.clear();
  bus.state = TWAI_STATE_STOPPED;
  bus.stops++;
  return ESP_OK;
}

esp_err_t twai_start(void) {
  if (bus.state != TWAI_STATE_STOPPED) return ESP_ERR_INVALID_STATE;
  bus.queue.clear();
  bus.state = TWAI_STATE_RUNNING;
  bus.starts++;
  return ESP_OK;
}

esp_err_t twai_initiate_recovery(void) {
  if (bus.state != TWAI_STATE_BUS_OFF) return ESP_ERR_INVALID_STATE;
  bus.queue.clear();
  bus.txBufferOccupied = false;
  bus.state = TWAI_STATE_RECOVERING;
  bus.recoveries++;
  return ESP_OK;
}

class EscTwaiStall : public ::testing::Test {
 protected:
  void SetUp() override {
    // A healthy cycle leaves nothing pending, which also resets the stall
    // counter setESCThrottle() keeps between cycles.
    bus.state = TWAI_STATE_RUNNING;
    bus.setAcknowledged(true);
    cycle(1000);
    ASSERT_EQ(bus.pending(), 0u);
    bus.wire.clear();
    bus.stops = bus.starts = bus.recoveries = 0;
  }
};

TEST_F(EscTwaiStall, HealthyBusSendsEachCommandOnceInOrder) {
  for (int pwm = 1035; pwm < 1235; pwm += 10) cycle(pwm);
  const auto sent = commandsOnWire();
  ASSERT_EQ(sent.size(), 20u);
  for (size_t i = 0; i < sent.size(); i++) EXPECT_EQ(sent[i], (1035 + 10 * i) * 10);
  EXPECT_EQ(bus.stops, 0) << "a healthy bus must never be reset";
}

TEST_F(EscTwaiStall, OneUnacknowledgedCycleIsNotTreatedAsAStall) {
  bus.setAcknowledged(false);
  cycle(1100);
  bus.setAcknowledged(true);
  cycle(1110);
  EXPECT_EQ(commandsOnWire(), (std::vector<uint16_t>{11000, 11100}));
  EXPECT_EQ(bus.stops, 0);
}

TEST_F(EscTwaiStall, NoCommandFromBeforeTheOutageReachesTheEscOnReconnect) {
  // ESC unpowered or connector open while at full power: nothing acknowledges
  // for five seconds.
  bus.setAcknowledged(false);
  for (int i = 0; i < 250; i++) cycle(1900);
  EXPECT_LE(bus.pending(), 2u) << "driver backlog must stay bounded";

  // The pilot has released the throttle by the time the ESC comes back.
  cycle(1035);
  cycle(1035);
  bus.setAcknowledged(true);

  EXPECT_EQ(commandsOnWire(), (std::vector<uint16_t>{10350}))
      << "a full-power command from before the outage was sent on reconnect";
  EXPECT_EQ(bus.state, TWAI_STATE_RUNNING);

  // And the path is healthy afterwards.
  bus.wire.clear();
  const int stopsBefore = bus.stops;
  cycle(1100);
  cycle(1110);
  EXPECT_EQ(commandsOnWire(), (std::vector<uint16_t>{11000, 11100}));
  EXPECT_EQ(bus.stops, stopsBefore);
}

TEST_F(EscTwaiStall, ReconnectDeliversOnlyTheMostRecentCycleCommand) {
  bus.setAcknowledged(false);
  int last = 0;
  for (int i = 0; i < 100; i++) {
    last = 1100 + 7 * i;
    cycle(last);
  }
  bus.setAcknowledged(true);
  EXPECT_EQ(commandsOnWire(), (std::vector<uint16_t>{static_cast<uint16_t>(last * 10)}))
      << "only the command computed in the last control cycle may go out";
}

TEST_F(EscTwaiStall, BusOffRecoveryDoesNotReplayBusOffEraCommands) {
  // Shorted or severed pair drives the controller bus-off at full power.
  bus.state = TWAI_STATE_BUS_OFF;
  for (int i = 0; i < 50; i++) cycle(1800);
  EXPECT_EQ(bus.recoveries, 1);
  EXPECT_EQ(bus.state, TWAI_STATE_RECOVERING);

  // The driver reports STOPPED once recovery completes; readESCTelemetry()
  // restarts it in the same cycle that queues the current command.
  bus.state = TWAI_STATE_STOPPED;
  cycle(1040);
  EXPECT_EQ(bus.state, TWAI_STATE_RUNNING);
  EXPECT_EQ(commandsOnWire(), (std::vector<uint16_t>{10400}))
      << "commands queued while the controller was off the bus were replayed";

  bus.wire.clear();
  cycle(1200);
  EXPECT_EQ(commandsOnWire(), (std::vector<uint16_t>{12000}));
}

TEST_F(EscTwaiStall, LongBusOutageStillLeavesRoomForTheCurrentCommand) {
  // A minute stuck recovering (the pair stays shorted). The transport
  // allocates every queued frame from esc.cpp's 1 KiB pool.
  bus.state = TWAI_STATE_BUS_OFF;
  for (int i = 0; i < 3000; i++) cycle(1700);
  bus.state = TWAI_STATE_STOPPED;
  cycle(1035);
  EXPECT_EQ(commandsOnWire(), (std::vector<uint16_t>{10350}));
}

int main(int argc, char** argv) {
  initESC();  // installs nothing (the model reports the driver installed)
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
