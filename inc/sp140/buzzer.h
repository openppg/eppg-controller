#ifndef SP140_BUZZER_H
#define SP140_BUZZER_H

#include <Arduino.h>

// Plays a melody using the buzzer
// @param melody: array of tone values
// @param siz: size of the melody array
// @return true if the melody was queued successfully, false otherwise
bool playMelody(uint16_t melody[], int siz);

// Plays a specific melody to indicate an arm failure
void handleArmFail();

#endif // SP140_BUZZER_H
