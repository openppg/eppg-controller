#ifndef INC_SP140_WIRELESS_PROTOCOL_H_
#define INC_SP140_WIRELESS_PROTOCOL_H_

#include <cstddef>
#include <cstdint>

namespace wireless {

constexpr uint16_t kPacketMagic = 0x5050;  // "PP" on the wire.
constexpr uint8_t kProtocolVersion = 1;
constexpr uint16_t kThrottleRawMax = 4095;

enum class PacketType : uint8_t {
  CONTROL = 1,
  ACK = 2,
};

enum AckFlags : uint8_t {
  ACK_LINK_READY = 1U << 0,
  ACK_FAILSAFE_LATCHED = 1U << 1,
  ACK_COMMAND_ACCEPTED = 1U << 2,
};

#pragma pack(push, 1)

// Remote -> receiver. The requested state is a level, while sequence/session
// make transitions unambiguous and prevent an old packet from re-arming after
// either unit restarts.
struct ControlPacket {
  uint16_t magic;
  uint8_t version;
  uint8_t type;
  uint32_t session_id;
  uint32_t sequence;
  uint32_t tx_uptime_ms;
  uint16_t throttle_raw;
  uint8_t requested_state;
  uint8_t performance_mode;
  uint8_t flags;
  uint8_t reserved;
  uint32_t crc32;
};

// Receiver -> remote. This is an application-level acknowledgement; an
// ESP-NOW MAC send success alone does not prove that the control application
// accepted a packet.
struct AckPacket {
  uint16_t magic;
  uint8_t version;
  uint8_t type;
  uint32_t session_id;
  uint32_t accepted_sequence;
  uint32_t rx_uptime_ms;
  uint8_t actual_state;
  uint8_t flags;
  uint16_t command_age_ms;
  uint32_t crc32;
};

#pragma pack(pop)

static_assert(sizeof(ControlPacket) == 26,
              "Wireless control packet layout changed");
static_assert(sizeof(AckPacket) == 24,
              "Wireless acknowledgement packet layout changed");

uint32_t crc32(const uint8_t* data, size_t length);
void finalize(ControlPacket* packet);
void finalize(AckPacket* packet);
bool validate(const ControlPacket& packet);
bool validate(const AckPacket& packet);

// Wrap-safe ordering for monotonically increasing uint32_t sequence numbers.
bool sequenceNewer(uint32_t candidate, uint32_t previous);

}  // namespace wireless

#endif  // INC_SP140_WIRELESS_PROTOCOL_H_
