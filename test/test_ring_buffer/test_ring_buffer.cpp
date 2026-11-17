#include <gtest/gtest.h>
#include "sp140/ring_buffer.h"

TEST(RingBuffer, StartsEmpty) {
  FixedRingBuffer<int, 4> buf;
  EXPECT_TRUE(buf.isEmpty());
  EXPECT_FALSE(buf.isFull());
  EXPECT_EQ(buf.size(), 0u);
  EXPECT_EQ(buf.capacity(), 4u);
}

TEST(RingBuffer, PushKeepsOldestFirst) {
  FixedRingBuffer<int, 4> buf;
  EXPECT_TRUE(buf.push(10));
  EXPECT_TRUE(buf.push(20));
  EXPECT_TRUE(buf.push(30));
  EXPECT_EQ(buf.size(), 3u);
  EXPECT_EQ(buf.first(), 10);
  EXPECT_EQ(buf.last(), 30);
  EXPECT_EQ(buf[0], 10);
  EXPECT_EQ(buf[1], 20);
  EXPECT_EQ(buf[2], 30);
}

TEST(RingBuffer, FullPushOverwritesOldestAndReturnsFalse) {
  FixedRingBuffer<int, 3> buf;
  buf.push(1);
  buf.push(2);
  EXPECT_TRUE(buf.push(3));
  EXPECT_TRUE(buf.isFull());

  EXPECT_FALSE(buf.push(4));  // Overwrites 1
  EXPECT_EQ(buf.size(), 3u);
  EXPECT_EQ(buf.first(), 2);
  EXPECT_EQ(buf.last(), 4);
  EXPECT_EQ(buf[0], 2);
  EXPECT_EQ(buf[1], 3);
  EXPECT_EQ(buf[2], 4);
}

TEST(RingBuffer, WrapsManyTimes) {
  FixedRingBuffer<int, 5> buf;
  for (int i = 0; i < 1003; ++i) buf.push(i);
  EXPECT_EQ(buf.size(), 5u);
  for (size_t i = 0; i < buf.size(); ++i) {
    EXPECT_EQ(buf[i], 998 + static_cast<int>(i));
  }
  EXPECT_EQ(buf.first(), 998);
  EXPECT_EQ(buf.last(), 1002);
}

TEST(RingBuffer, ClearResetsAfterWrap) {
  FixedRingBuffer<int, 3> buf;
  for (int i = 0; i < 7; ++i) buf.push(i);
  buf.clear();
  EXPECT_TRUE(buf.isEmpty());
  EXPECT_EQ(buf.size(), 0u);
  buf.push(42);
  EXPECT_EQ(buf.first(), 42);
  EXPECT_EQ(buf.last(), 42);
  EXPECT_EQ(buf[0], 42);
}

TEST(RingBuffer, FillToCapacityLikeThrottleReset) {
  // Mirrors throttleFilterReset(): clear, then push until full.
  FixedRingBuffer<int, 8> buf;
  buf.push(1);
  buf.push(2);
  buf.clear();
  int pushes = 0;
  while (!buf.isFull()) {
    buf.push(1500);
    ++pushes;
  }
  EXPECT_EQ(pushes, 8);
  for (size_t i = 0; i < buf.size(); ++i) EXPECT_EQ(buf[i], 1500);
}

TEST(RingBuffer, StructElementsLikeAltimeter) {
  struct Reading {
    float altitude;
    unsigned long timestamp;
  };
  FixedRingBuffer<Reading, 2> buf;
  buf.push({1.0f, 100});
  buf.push({2.0f, 200});
  buf.push({3.0f, 300});
  EXPECT_FLOAT_EQ(buf.first().altitude, 2.0f);
  EXPECT_EQ(buf.first().timestamp, 200u);
  EXPECT_FLOAT_EQ(buf.last().altitude, 3.0f);
  EXPECT_EQ(buf.last().timestamp, 300u);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
