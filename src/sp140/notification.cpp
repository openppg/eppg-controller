#include "../../inc/sp140/notification.h"

bool shouldNotifyLowBattery(float soc) {
  return soc > 0.0f && soc < 20.0f;
}

bool shouldNotifyBingo(float soc, bool alreadyFired, bool seenAboveThreshold) {
  return !alreadyFired && seenAboveThreshold && soc > 0.0f && soc < 50.0f;
}
