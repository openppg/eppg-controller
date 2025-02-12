#include "sp140/buzzer.h"
#include <Arduino.h>
#include "sp140/structs.h"      // For MelodyRequest and melodyQueue
#include "sp140/shared-config.h" // For ENABLE_BUZ
#include "sp140/globals.h"       // For extern declaration of melodyQueue


bool playMelody(uint16_t melody[], int siz) {
  if (!ENABLE_BUZ)
    return false;

  // Create a static buffer for the melody (adjust size as needed)
  static uint16_t melodyBuffer[32];
  int copySize = (siz < 32) ? siz : 32;
  for (int i = 0; i < copySize; i++) {
    melodyBuffer[i] = melody[i];
  }

  MelodyRequest request = {
    .notes = melodyBuffer,
    .size = (uint8_t)copySize,
    .duration = 125  // Default duration in ms
  };

  // Send to queue with timeout
  if(xQueueSend(melodyQueue, &request, pdMS_TO_TICKS(100)) != pdTRUE) {
    USBSerial.println("Failed to queue melody");
    return false;
  }

  return true;
}

void handleArmFail() {
  uint16_t arm_fail_melody[] = { 820, 640 };
  playMelody(arm_fail_melody, 2);
}
