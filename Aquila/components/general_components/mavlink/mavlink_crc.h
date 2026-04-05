#ifndef MAVLINK_CRC_H
#define MAVLINK_CRC_H

#include <inttypes.h>

 void crc_accumulate(uint8_t data, uint16_t *crcAccum);
 uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length);
 void crc_init(uint16_t* crcAccum);
 void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length);

#endif
