#include <gtest/gtest.h>
#include <vector>

// Exercise production callbacks and wire packets with flash/BLE/RTOS fakes.
#include "../../src/sp140/ble/ota_service.cpp"

volatile DeviceState currentState = DISARMED;
SemaphoreHandle_t stateMutex = nullptr;

namespace {
// Independent protocol encoder for the existing app command/sector format.
uint16_t crc(const std::vector<uint8_t>& bytes) {
  uint16_t result = 0;
  for (uint8_t byte : bytes) {
    result ^= uint16_t(byte) << 8;
    for (int bit = 0; bit < 8; ++bit)
      result = (result & 0x8000) ? (result << 1) ^ 0x1021 : result << 1;
  }
  return result;
}

class OtaServiceTest : public ::testing::Test {
 protected:
  NimBLEServer server;
  void SetUp() override {
    fakeMs = 0;
    ESP.restarts = 0;
    currentState = DISARMED;
    beginResult = writeResult = endResult = bootResult = 0;
    writes = aborts = bootReceipt = 0;
    partitionAvailable = true;
    runningPartition.address = 0x10000;
    otaStatus = OtaStatus{};
    otaInProgress = false;
    renderedRevision = 0;
    resetOtaState();
    stateMutex = xSemaphoreCreateMutex();
    initOtaBleService(&server);
    // Simulate a UI tick: this must work while the OTA mutex is held.
    onDelay = [] {
      OtaStatus status;
      if (readOtaStatus(status)) acknowledgeOtaRendered(status.revision);
    };
  }
  void TearDown() override {
    onDelay = onBegin = onWrite = onEnd = nullptr;
    delete stateMutex;
    delete otaStateMutex;
    delete otaStatusQueue;
    stateMutex = otaStateMutex = nullptr;
    otaStatusQueue = nullptr;
  }
  void command(uint16_t id, uint32_t size = 0) {
    std::vector<uint8_t> packet(18, 0);
    packet[0] = id & 0xff;
    packet[1] = id >> 8;
    for (int i = 0; i < 4; ++i) packet[2 + i] = size >> (8 * i);
    const uint16_t check = crc(packet);
    packet.push_back(check & 0xff);
    packet.push_back(check >> 8);
    pCommandChar->write(packet);
  }
  void sector(uint16_t index, unsigned count, bool validCrc = true) {
    std::vector<uint8_t> bytes(count, 0x42);
    const uint16_t check = crc(bytes) ^ (validCrc ? 0 : 1);
    bytes.insert(bytes.begin(), {uint8_t(index), uint8_t(index >> 8), 0xff});
    bytes.push_back(check & 0xff);
    bytes.push_back(check >> 8);
    pRecvFwChar->write(bytes);
  }
  OtaStatus status() {
    OtaStatus value;
    EXPECT_TRUE(readOtaStatus(value));
    return value;
  }
  void expectCommandAck(uint16_t id, uint16_t result) {
    ASSERT_FALSE(pCommandChar->notifications.empty());
    const std::string& ack = pCommandChar->notifications.back();
    ASSERT_EQ(ack.size(), 20u);
    EXPECT_EQ(uint8_t(ack[0]), 3);
    EXPECT_EQ(uint8_t(ack[2]), id);
    EXPECT_EQ(uint8_t(ack[4]), result);
    std::vector<uint8_t> bytes(ack.begin(), ack.begin() + 18);
    EXPECT_EQ(uint16_t(uint8_t(ack[18])) | (uint16_t(uint8_t(ack[19])) << 8), crc(bytes));
  }
};

TEST_F(OtaServiceTest, SuccessfulTransferPreservesProtocolAndBootReceipt) {
  onBegin = [&] {
    EXPECT_TRUE(isOtaInProgress());
    EXPECT_EQ(status().phase, OtaPhase::Preparing);
    EXPECT_EQ(renderedRevision.load(), status().revision);
  };
  command(CMD_START, 100);
  expectCommandAck(CMD_START, 0);
  sector(0, 42);
  EXPECT_EQ(otaProgressPercent(status()), 42u);
  ASSERT_EQ(pRecvFwChar->notifications.back().size(), 6u);
  EXPECT_EQ(uint8_t(pRecvFwChar->notifications.back()[2]), 0);
  sector(1, 58);
  EXPECT_EQ(status().phase, OtaPhase::Receiving);  // 100% is not success
  onEnd = [&] {
    EXPECT_EQ(status().phase, OtaPhase::Verifying);
    EXPECT_EQ(renderedRevision.load(), status().revision);
  };
  command(CMD_END);
  expectCommandAck(CMD_END, 0);
  EXPECT_EQ(ESP.restarts, 1u);
  EXPECT_EQ(status().phase, OtaPhase::Restarting);
  EXPECT_TRUE(isOtaInProgress());
  EXPECT_EQ(bootReceipt, targetPartition.address);
  abortOta();  // Expected disconnect must not overwrite the restart screen.
  EXPECT_EQ(status().phase, OtaPhase::Restarting);
  EXPECT_EQ(aborts, 0u);
  // Simulate the subsequent boot's validation result.
  otaInProgress = false;
  runningPartition.address = targetPartition.address;
  finishOtaBoot(true);
  EXPECT_EQ(status().phase, OtaPhase::Complete);
  EXPECT_EQ(bootReceipt, 0u);
  const auto revision = status().revision;
  finishOtaBoot(true);
  EXPECT_EQ(status().revision, revision);  // Receipt is shown once.
}

TEST_F(OtaServiceTest, CrcRetryOnlyAdvancesAfterSuccessfulFlashWrite) {
  command(CMD_START, 100);
  sector(0, 42, false);
  EXPECT_EQ(status().written, 0u);
  EXPECT_EQ(writes, 0u);
  EXPECT_EQ(uint8_t(pRecvFwChar->notifications.back()[2]), ACK_ERR_CRC);
  onWrite = [&] { EXPECT_EQ(status().written, 0u); };
  sector(0, 42);
  EXPECT_EQ(status().written, 42u);
  sector(0, 42);  // Duplicate sector does not count twice.
  EXPECT_EQ(status().written, 42u);
}

TEST_F(OtaServiceTest, DisconnectRetainsProgressAndRetryResetsIt) {
  command(CMD_START, 100);
  sector(0, 42);
  abortOta();
  EXPECT_FALSE(isOtaInProgress());
  EXPECT_EQ(status().phase, OtaPhase::Failed);
  EXPECT_EQ(status().error, OtaError::Disconnected);
  EXPECT_EQ(status().written, 42u);
  EXPECT_TRUE(otaBlocksInput());
  fakeMs += 10000;
  EXPECT_FALSE(otaBlocksInput());
  command(CMD_START, 200);
  EXPECT_EQ(status().phase, OtaPhase::Receiving);
  EXPECT_EQ(status().written, 0u);
  EXPECT_EQ(status().total, 200u);
}

TEST_F(OtaServiceTest, TimeoutPublishesErrorAndReleasesInterlock) {
  command(CMD_START, 100);
  fakeMs += 30001;
  checkOtaTimeout();
  EXPECT_FALSE(isOtaInProgress());
  EXPECT_EQ(status().error, OtaError::Timeout);
  EXPECT_EQ(aborts, 1u);
}

TEST_F(OtaServiceTest, FailedWriteDoesNotAdvanceProgress) {
  command(CMD_START, 100);
  writeResult = -1;
  sector(0, 42);
  EXPECT_EQ(status().error, OtaError::Write);
  EXPECT_EQ(status().written, 0u);
  EXPECT_FALSE(isOtaInProgress());
}

TEST_F(OtaServiceTest, IncompleteImageCannotEnterVerification) {
  command(CMD_START, 100);
  sector(0, 42);
  command(CMD_END);
  expectCommandAck(CMD_END, 1);
  EXPECT_EQ(status().error, OtaError::SizeMismatch);
  EXPECT_EQ(ESP.restarts, 0u);
}

TEST_F(OtaServiceTest, InvalidImageDoesNotAbortAnAlreadyEndedHandle) {
  command(CMD_START, 42);
  sector(0, 42);
  endResult = -1;
  command(CMD_END);
  expectCommandAck(CMD_END, 1);
  EXPECT_EQ(status().error, OtaError::Verification);
  EXPECT_EQ(aborts, 0u);
  EXPECT_EQ(bootReceipt, 0u);
  EXPECT_EQ(ESP.restarts, 0u);
}

TEST_F(OtaServiceTest, BootSelectionFailureCannotShowRestarting) {
  command(CMD_START, 42);
  sector(0, 42);
  bootResult = -1;
  command(CMD_END);
  EXPECT_EQ(status().error, OtaError::BootPartition);
  EXPECT_EQ(ESP.restarts, 0u);
  EXPECT_EQ(bootReceipt, 0u);
}

TEST_F(OtaServiceTest, PreparationFailuresReleaseInterlock) {
  partitionAvailable = false;
  command(CMD_START, 100);
  EXPECT_EQ(status().error, OtaError::NoPartition);
  EXPECT_FALSE(isOtaInProgress());
  partitionAvailable = true;
  command(CMD_START, 0);
  EXPECT_EQ(status().error, OtaError::InvalidSize);
  EXPECT_FALSE(isOtaInProgress());
  command(CMD_START, targetPartition.size + 1);
  EXPECT_EQ(status().error, OtaError::InvalidSize);
  beginResult = -1;
  command(CMD_START, 100);
  EXPECT_EQ(status().error, OtaError::Begin);
  EXPECT_FALSE(isOtaInProgress());
}

TEST_F(OtaServiceTest, ArmedStartIsRejectedWithoutReplacingFlightDisplay) {
  currentState = ARMED;
  command(CMD_START, 100);
  expectCommandAck(CMD_START, 1);
  EXPECT_FALSE(isOtaInProgress());
  OtaStatus value;
  EXPECT_FALSE(readOtaStatus(value));
}

TEST_F(OtaServiceTest, DuplicateStartAbortsOldHandleAndStartsFresh) {
  command(CMD_START, 100);
  sector(0, 42);
  command(CMD_START, 200);
  EXPECT_EQ(aborts, 1u);
  EXPECT_EQ(status().written, 0u);
  EXPECT_EQ(status().total, 200u);
  EXPECT_TRUE(isOtaInProgress());
}

TEST_F(OtaServiceTest, MissingUiCannotHangStartOrRestart) {
  onDelay = nullptr;
  command(CMD_START, 42);
  EXPECT_EQ(fakeMs, 250u);
  sector(0, 42);
  command(CMD_END);
  EXPECT_EQ(ESP.restarts, 1u);
  EXPECT_EQ(fakeMs, 1750u);
}

TEST_F(OtaServiceTest, RollbackShowsFailureInsteadOfSuccess) {
  bootReceipt = targetPartition.address;
  finishOtaBoot(true);
  EXPECT_EQ(status().phase, OtaPhase::Failed);
  EXPECT_EQ(status().error, OtaError::RolledBack);
  EXPECT_EQ(bootReceipt, 0u);
}
}  // namespace
