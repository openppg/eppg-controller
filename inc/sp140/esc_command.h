#ifndef INC_SP140_ESC_COMMAND_H_
#define INC_SP140_ESC_COMMAND_H_

// Decisions on the ESC command path, kept free of hardware state so the host
// regressions in test/test_esc_command can drive them through bus faults.

#include <stdint.h>

class CanardAdapter;
class SineEsc;

// The throttle command is also the ESC's keep-alive. It may only be withheld for
// a config or firmware relay session, and those only run while disarmed. A relay
// request reports "active" from the moment it is queued, before the session
// rejects it for being armed, so the armed check has to come first.
inline bool escThrottleCommandAllowed(bool disarmed, bool relaySessionActive) {
  return !disarmed || !relaySessionActive;
}

inline bool escThrottlePwmValid(int throttlePWM) {
  return throttlePWM >= 1000 && throttlePWM <= 2000;
}

// Detects commands that are not leaving the controller. A healthy bus sends each
// frame in well under a millisecond, so frames still waiting on two consecutive
// 20 ms cycles are not going out.
class EscTxStall {
 public:
  static constexpr uint8_t LIMIT_CYCLES = 2;

  // Call once per control cycle. Returns true while the backlog should be discarded.
  bool update(bool pending) {
    if (!pending) {
      cycles_ = 0;
    } else if (cycles_ < LIMIT_CYCLES) {
      cycles_++;
    }
    return cycles_ >= LIMIT_CYCLES;
  }

 private:
  uint8_t cycles_ = 0;
};

enum class EscBusState : uint8_t { STOPPED, RUNNING, BUS_OFF, RECOVERING };
enum class EscBusAction : uint8_t { NONE, INITIATE_RECOVERY, START };

// Bus-off recovery: initiate while BUS_OFF, wait through RECOVERING, restart once
// the driver lands in STOPPED. Only a driver that this recovery took down is
// restarted, never one something else stopped on purpose.
class EscBusRecovery {
 public:
  EscBusAction step(EscBusState state) {
    switch (state) {
      case EscBusState::BUS_OFF:
        if (!pending_) {
          pending_ = true;
          return EscBusAction::INITIATE_RECOVERY;
        }
        return EscBusAction::NONE;
      case EscBusState::RECOVERING:
        return EscBusAction::NONE;
      case EscBusState::STOPPED:
        return pending_ ? EscBusAction::START : EscBusAction::NONE;
      case EscBusState::RUNNING:
      default:
        pending_ = false;
        return EscBusAction::NONE;
    }
  }

  // The caller reports that the START action succeeded.
  void started() { pending_ = false; }
  bool pending() const { return pending_; }

 private:
  bool pending_ = false;
};

// Queue one throttle command (microseconds, 1000-2000) for the next bus pump.
//
// Commands are queued FIFO and retried until acknowledged. When the ESC stops
// acknowledging (unpowered, connector open) or the controller is bus-off, the
// driver queue and the transport behind it fill with old commands, which would be
// replayed oldest-first on reconnect while new ones are dropped. Once the stall
// detector trips, the backlog is discarded so only current commands are kept.
// (During bus-off recovery the driver queue reads empty and rejects frames, so
// the transport queue is checked too.)
//
// Returns true when a command was queued.
bool escQueueThrottle(bool busReady, CanardAdapter& adapter, SineEsc& esc,  // NOLINT(runtime/references)
                      EscTxStall& stall, int throttlePWM);  // NOLINT(runtime/references)

#endif  // INC_SP140_ESC_COMMAND_H_
