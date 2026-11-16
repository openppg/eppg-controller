// Runs the pinned Adafruit MCP2515 driver and ANT-BMS-CAN BMS_CAN, plus the
// firmware's bms_can_rx.cpp, against a model of the MCP2515 receive buffers
// (test/emu_stubs/can/Adafruit_SPIDevice.h). bmsTask drains the controller once
// every 100 ms; these tests deliver BMS frames between two drains.
#include <gtest/gtest.h>

#include "Arduino.h"
#include "Adafruit_SPIDevice.h"
#include "BMS_CAN.h"
#include "sp140/bms_can_rx.h"

#include "../../src/sp140/bms_can_rx.cpp"

namespace {

constexpr uint8_t kBmsCs = 5;       // MCP_CS
constexpr long kBmsBaud = 250000;   // MCP_BAUDRATE

// BMS Basic Information 1: pack voltage (0.1 V) in bytes 4..5.
void sendBasicInfo1(float packVolts) {
  const uint16_t v = static_cast<uint16_t>(packVolts * 10.0f + 0.5f);
  const uint8_t d[8] = {0x18, 80, 0x88, 0x13, static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8), 0, 0};
  emuMcp2515().receive(0x18FF28F4, d, 8);
}

// BMS Basic Information 2: highest/lowest cell (mV), highest/lowest temp (+40).
void sendBasicInfo2(uint16_t highestMv) {
  const uint16_t lowestMv = highestMv - 20;
  const uint8_t d[8] = {static_cast<uint8_t>(highestMv), static_cast<uint8_t>(highestMv >> 8),
                        static_cast<uint8_t>(lowestMv), static_cast<uint8_t>(lowestMv >> 8),
                        65, 60, 0, 0};
  emuMcp2515().receive(0x18FE28F4, d, 8);
}

// First temperature frame: MOS, balance, then four cell probes (+40).
void sendTemperatures(uint8_t mosC) {
  const uint8_t d[8] = {static_cast<uint8_t>(mosC + 40), 70, 65, 65, 65, 65, 0, 0};
  emuMcp2515().receive(0x18B428F4, d, 8);
}

class BmsCanRx : public ::testing::Test {
 protected:
  void SetUp() override {
    emuMcp2515() = EmuMcp2515{};
    ASSERT_TRUE(bms.begin());
  }

  // One bmsTask drain, in the order updateBMSData() runs it.
  uint8_t poll() {
    bms.update();
    return bmsCanRxTakeOverflows(regs);
  }

  BMS_CAN bms{kBmsCs, kBmsBaud, &SPI};
  Adafruit_SPIDevice regs{kBmsCs, 10000000, SPI_BITORDER_MSBFIRST, SPI_MODE0, &SPI};
};

// The mechanism: with the library's configuration only RXB0 receives, so the
// second frame between two drains is dropped.
TEST_F(BmsCanRx, LibraryConfigurationKeepsOneFrameBetweenDrains) {
  sendBasicInfo1(50.4f);
  sendBasicInfo2(4120);
  poll();
  EXPECT_NEAR(bms.getBatteryVoltage(), 50.4f, 0.01f);
  EXPECT_FLOAT_EQ(bms.getHighestCellVoltage(), 0.0f) << "second frame should have been dropped";
  EXPECT_EQ(emuMcp2515().framesLost, 1u);
}

TEST_F(BmsCanRx, RolloverKeepsTwoFramesBetweenDrains) {
  ASSERT_TRUE(bmsCanRxEnableRollover(regs));
  EXPECT_EQ(emuMcp2515().regs[0x60] & 0x60, 0x60) << "receive-any mode must be kept";

  sendBasicInfo1(50.4f);
  sendBasicInfo2(4120);
  EXPECT_EQ(poll(), 0);
  EXPECT_NEAR(bms.getBatteryVoltage(), 50.4f, 0.01f);
  EXPECT_NEAR(bms.getHighestCellVoltage(), 4.120f, 0.0005f);
  EXPECT_EQ(emuMcp2515().framesLost, 0u);

  // Order is kept across drains: the newer value wins.
  sendBasicInfo1(50.3f);
  sendBasicInfo1(50.2f);
  EXPECT_EQ(poll(), 0);
  EXPECT_NEAR(bms.getBatteryVoltage(), 50.2f, 0.01f);
}

TEST_F(BmsCanRx, LossBeyondTwoFramesIsCountedOnceAndCleared) {
  ASSERT_TRUE(bmsCanRxEnableRollover(regs));
  sendBasicInfo1(50.4f);
  sendBasicInfo2(4120);
  sendTemperatures(31);  // third frame before the drain: RXB1 overflows
  EXPECT_EQ(poll(), 1);
  EXPECT_EQ(emuMcp2515().framesLost, 1u);
  EXPECT_TRUE(isnan(bms.getTemperature(0))) << "the third frame cannot have arrived";
  EXPECT_EQ(emuMcp2515().regs[0x2D] & 0xC0, 0) << "overflow flags must be cleared";

  // Nothing lost since: nothing counted.
  sendTemperatures(31);
  EXPECT_EQ(poll(), 0);
  EXPECT_NEAR(bms.getTemperature(0), 31.0f, 0.01f);
}

TEST_F(BmsCanRx, WithoutRolloverTheOverflowIsStillCounted) {
  // If enabling rollover ever failed to read back, loss is still reported.
  sendBasicInfo1(50.4f);
  sendBasicInfo2(4120);
  EXPECT_EQ(poll(), 1);
  EXPECT_EQ(poll(), 0);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
