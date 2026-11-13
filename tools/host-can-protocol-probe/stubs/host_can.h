#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace host_can {

struct Frame {
  std::uint32_t id = 0;
  std::array<std::uint8_t, 8> data{};
  std::uint8_t len = 0;
  bool extended = true;
  bool rtr = false;
};

void reset();
void enqueueMcpRx(const Frame& frame);
void enqueueTwaiRx(const Frame& frame);
std::vector<Frame> takeTwaiTx();

}  // namespace host_can

