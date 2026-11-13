#include <gtest/gtest.h>

#include "sp140/wireless_protocol.h"
#include "../../src/sp140/wireless_protocol.cpp"

TEST(WirelessProtocol, ValidatesFinalizedControlPacket) {
  wireless::ControlPacket packet = {};
  packet.session_id = 123;
  packet.sequence = 456;
  packet.tx_uptime_ms = 789;
  packet.throttle_raw = 2048;
  packet.requested_state = 1;
  packet.performance_mode = 1;
  wireless::finalize(&packet);

  EXPECT_TRUE(wireless::validate(packet));
}

TEST(WirelessProtocol, RejectsCorruptionAndOutOfRangeControl) {
  wireless::ControlPacket packet = {};
  packet.session_id = 1;
  packet.sequence = 2;
  packet.throttle_raw = 4095;
  packet.requested_state = 0;
  wireless::finalize(&packet);
  ASSERT_TRUE(wireless::validate(packet));

  packet.sequence ^= 0x100;
  EXPECT_FALSE(wireless::validate(packet));

  packet.sequence ^= 0x100;
  packet.throttle_raw = 4096;
  wireless::finalize(&packet);
  EXPECT_FALSE(wireless::validate(packet));
}

TEST(WirelessProtocol, ValidatesAcknowledgement) {
  wireless::AckPacket packet = {};
  packet.session_id = 10;
  packet.accepted_sequence = 20;
  packet.actual_state = 2;
  packet.flags = wireless::ACK_LINK_READY;
  packet.command_age_ms = 30;
  wireless::finalize(&packet);

  EXPECT_TRUE(wireless::validate(packet));
  packet.actual_state = 3;
  wireless::finalize(&packet);
  EXPECT_FALSE(wireless::validate(packet));
}

TEST(WirelessProtocol, SequenceComparisonIsWrapSafe) {
  EXPECT_TRUE(wireless::sequenceNewer(11, 10));
  EXPECT_FALSE(wireless::sequenceNewer(10, 10));
  EXPECT_FALSE(wireless::sequenceNewer(9, 10));
  EXPECT_TRUE(wireless::sequenceNewer(0, UINT32_MAX));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
