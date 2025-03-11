
#include <inttypes.h>

float avg_filter_1d(float* array, float new_data_point, uint8_t filter_length)
{
    uint8_t i = 0;
    float accum = 0;
  
    for (i = 0; i < filter_length - 1; i++) array[i] = array[i+1];
    array[filter_length-1] = new_data_point;
    for (i = 0; i < filter_length; i++) accum += array[i];
    
    return (accum/filter_length);        
}