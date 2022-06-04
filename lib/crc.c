// Copyright 2019 <OpenPPG>
/*
 * crc.c
 *
 *  Created on: Mar 3, 2019
 *      Author: MiguelFAlvarez
 */

#include "crc.h"

uint16_t crc16(uint8_t *buf, uint32_t size) {
  uint16_t crc = 0;
  for (uint32_t i = 0; i < size; i++)
    crc = (crc << 8) ^ crc16table[buf[i] ^ crc>>8];
  return crc;
}

