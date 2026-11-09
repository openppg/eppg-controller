#include <gtest/gtest.h>
#include "sp140/led_color.h"
#include "sp140/utilities.h"

static void expectRgb(uint32_t color, uint8_t r, uint8_t g, uint8_t b) {
  const RgbColor rgb = unpackRgb(color);
  EXPECT_EQ(rgb.r, r);
  EXPECT_EQ(rgb.g, g);
  EXPECT_EQ(rgb.b, b);
}

TEST(LedColor, StatusColorsUnpackToExpectedChannels) {
  expectRgb(STATUS_LED_RED, 0xFF, 0x00, 0x00);
  expectRgb(STATUS_LED_YELLOW, 0xFF, 0xFF, 0x00);
  expectRgb(STATUS_LED_GREEN, 0x00, 0xFF, 0x00);
}

TEST(LedColor, OffIsAllZero) {
  expectRgb(0, 0, 0, 0);
}

TEST(LedColor, ChannelsAreIndependentAndTopByteIgnored) {
  expectRgb(0x00123456, 0x12, 0x34, 0x56);
  expectRgb(0xAB123456, 0x12, 0x34, 0x56);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
