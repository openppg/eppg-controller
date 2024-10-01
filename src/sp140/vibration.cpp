#include "sp140/vibration.h"
#include "sp140/structs.h"
#include <Adafruit_DRV2605.h>

Adafruit_DRV2605 vibe;

bool vibePresent = false;

bool initVibe() {
  if (!vibe.begin(&Wire1)) { return false; }

  vibe.selectLibrary(1);
  vibe.setMode(DRV2605_MODE_INTTRIG);
  vibrateNotify();  // initial boot vibration

  return true;
}

void vibrateNotify() {
  if (!vibePresent) { return; }

  vibe.setWaveform(0, 15);  // 1 through 117 (see example sketch)
  vibe.setWaveform(1, 0);
  vibe.go();
}

bool runVibe(const unsigned int sequence[], int siz) {
  if (!vibePresent) { return false; }

  for (int thisVibe = 0; thisVibe < siz; thisVibe++) {
    vibe.setWaveform(thisVibe, sequence[thisVibe]);
  }
  vibe.go();
  return true;
}
