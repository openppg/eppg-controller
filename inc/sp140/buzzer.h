#ifndef INC_SP140_BUZZER_H_
#define INC_SP140_BUZZER_H_

#include <Arduino.h>
#include "sp140/structs.h"

/**
 * Initialize the buzzer pin for output
 */
void initBuzz();

/**
 * Plays a melody using the piezo buzzer
 *
 * @param melody Array of frequencies to play
 * @param siz Size of the melody array
 * @return Returns true if the melody was queued successfully, false otherwise
 */
bool playMelody(uint16_t melody[], int siz);

/**
 * Plays a melody to indicate arm failure
 */
void handleArmFailMelody();

#endif  // INC_SP140_BUZZER_H_
