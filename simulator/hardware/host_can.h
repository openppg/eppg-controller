#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace eppg::sim::host_can {

struct Frame {
  std::uint32_t id = 0;
  std::array<std::uint8_t, 8> data{};
  std::uint8_t len = 0;
  bool extended = true;
  bool rtr = false;
};

void reset();
void enqueue_mcp_rx(const Frame& frame);
void enqueue_twai_rx(const Frame& frame);
std::vector<Frame> take_twai_tx();
std::size_t twai_rx_size();

}  // namespace eppg::sim::host_can
