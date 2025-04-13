#include <inttypes.h>

//алгоритм вычисления контрольной суммы по типу Maxim (Dallas)
uint8_t dallas_crc8(uint8_t *input_data, unsigned int size)
{
  uint8_t crc = 0;
  uint8_t i = 0;
  uint8_t inbyte = 0;
  uint8_t j = 0;
  uint8_t mix1 = 0;
  for (  i = 0; i < size; i++ ) {
        inbyte = input_data[i];
      for (  j = 0; j < 8; j++ ) {
          mix1 = (crc ^ inbyte) & 0x01;
          crc = crc >> 1;
          if ( mix1 ) crc ^= 0x8C;                         
          inbyte >>= 1;
      }}
  return crc;
}