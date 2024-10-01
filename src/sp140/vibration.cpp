#include "sp140/vibration.h"
#include "sp140/structs.h"
#include <Adafruit_DRV2605.h>

Adafruit_DRV2605 vibe;

bool vibePresent = false;

bool initVibe() {
  if (!vibe.begin(&Wire1)) { return false; }
  vibe.selectLibrary(3);
  vibe.setMode(DRV2605_MODE_INTTRIG);
  vibePresent = true;

  vibrateNotify();  // initial boot vibration
  return true;
}

void vibrateNotify() {
  const unsigned int notify_vibes[] = { 14, 1, 14 };
  runVibe(notify_vibes, 3);
}

bool runVibe(const unsigned int sequence[], int siz) {
  if (!vibePresent) { return false; }

  for (int thisVibe = 0; thisVibe < siz; thisVibe++) {
    vibe.setWaveform(thisVibe, sequence[thisVibe]);
  }
  vibe.go();
  return true;
}
