#pragma once

#include "Arduino.h"
#include "host_can.h"

class Adafruit_MCP2515 {
 public:
  explicit Adafruit_MCP2515(std::int8_t cs_pin, SPIClass* spi = nullptr);
  Adafruit_MCP2515(std::int8_t cs_pin, std::int8_t mosi, std::int8_t miso,
                   std::int8_t sck);

  int begin(long baud_rate);
  int parsePacket();
  std::uint32_t packetId() const;
  int read();

 private:
  eppg::sim::host_can::Frame active_{};
  std::uint8_t read_index_ = 0;
};
