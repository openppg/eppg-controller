#include "sp140/wireless_protocol.h"

#include <cstddef>
#include <cstdint>

namespace wireless {
namespace {

template <typename Packet>
uint32_t packetCrc(const Packet& packet) {
  return crc32(reinterpret_cast<const uint8_t*>(&packet),
               offsetof(Packet, crc32));
}

}  // namespace

uint32_t crc32(const uint8_t* data, size_t length) {
  if (data == nullptr) return 0;

  uint32_t crc = 0xFFFFFFFFU;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      const uint32_t mask = -(crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return ~crc;
}

void finalize(ControlPacket* packet) {
  if (packet == nullptr) return;
  packet->magic = kPacketMagic;
  packet->version = kProtocolVersion;
  packet->type = static_cast<uint8_t>(PacketType::CONTROL);
  packet->crc32 = packetCrc(*packet);
}

void finalize(AckPacket* packet) {
  if (packet == nullptr) return;
  packet->magic = kPacketMagic;
  packet->version = kProtocolVersion;
  packet->type = static_cast<uint8_t>(PacketType::ACK);
  packet->crc32 = packetCrc(*packet);
}

bool validate(const ControlPacket& packet) {
  return packet.magic == kPacketMagic &&
         packet.version == kProtocolVersion &&
         packet.type == static_cast<uint8_t>(PacketType::CONTROL) &&
         packet.throttle_raw <= kThrottleRawMax &&
         packet.requested_state <= 2 && packet.performance_mode <= 1 &&
         packet.crc32 == packetCrc(packet);
}

bool validate(const AckPacket& packet) {
  return packet.magic == kPacketMagic &&
         packet.version == kProtocolVersion &&
         packet.type == static_cast<uint8_t>(PacketType::ACK) &&
         packet.actual_state <= 2 && packet.crc32 == packetCrc(packet);
}

bool sequenceNewer(uint32_t candidate, uint32_t previous) {
  return static_cast<int32_t>(candidate - previous) > 0;
}

}  // namespace wireless
