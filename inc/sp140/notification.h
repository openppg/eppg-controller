#ifndef INC_SP140_NOTIFICATION_H_
#define INC_SP140_NOTIFICATION_H_

#include <stdint.h>

enum class NotifySeverity {
  INFO,
  CAUTION
};

struct Notification {
  char message[20];
  NotifySeverity severity;
  uint16_t displayMs;
  bool vibrate;
};

// Pure rule helpers (unit-test friendly)
bool shouldNotifyLowBattery(float soc);
bool shouldNotifyBingo(float soc, bool alreadyFired, bool seenAboveThreshold);

// Runtime API (platform-only, not compiled in tests)
void initNotifications();
void processNotificationQueue();
void queueNotification(const char* msg, NotifySeverity sev, uint16_t durationMs, bool vibrate);

// Flight-driven rule checks
void resetFlightNotifications();
void checkArmNotifications();
void checkFlightNotifications();

#endif  // INC_SP140_NOTIFICATION_H_
