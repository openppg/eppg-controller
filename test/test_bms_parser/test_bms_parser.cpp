#include <gtest/gtest.h>
#include "BMS_CAN.h"

// Only the controller transport is mocked; the production ANT parser is linked.
namespace platform {
bool Mcp2515::initialize() { return true; }
bool Mcp2515::receive(CanFrame*) { return false; }
}

TEST(BmsParser, ReassemblesAllThreeIdentityFrames) {
  BMS_CAN bms(5, 250000);
  const uint8_t one[] = {'A','B','C','D','E','F','G','H'};
  const uint8_t two[] = {'I','J','K','L','M','N','O','P'};
  const uint8_t three[] = {'Q','R','S','T','U','V','W','X'};
  bms.ingestFrame(0x18A028F4, one, 8);
  bms.ingestFrame(0x18A128F4, two, 8);
  bms.ingestFrame(0x18A228F4, three, 8);
  EXPECT_EQ(bms.getBatteryID(), "ABCDEFGHIJKLMNOPQRSTUVWX");
  EXPECT_EQ(bms.getBmsType(), BMS_TYPE_B);
}

TEST(BmsParser, IdentityRequiresExactIdentifierAndFullFrame) {
  BMS_CAN bms(5, 250000);
  const uint8_t bytes[] = {'A','B','C','D','E','F','G','H'};
  bms.ingestFrame(0x18A028F5, bytes, 8);
  bms.ingestFrame(0x18A028F4, bytes, 7);
  EXPECT_TRUE(bms.getBatteryID().empty());
  EXPECT_EQ(bms.getBmsType(), BMS_TYPE_UNKNOWN);
}

TEST(BmsParser, RejectsShortBasicFramesWithoutReadingMissingBytes) {
  BMS_CAN bms(5, 250000);
  const uint8_t bytes[] = {0xff};
  bms.ingestFrame(0x18FF28F4, bytes, 1);
  bms.ingestFrame(0x18FE28F4, bytes, 1);
  EXPECT_EQ(bms.getBatteryVoltage(), 0);
  EXPECT_EQ(bms.getHighestCellVoltage(), 0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
