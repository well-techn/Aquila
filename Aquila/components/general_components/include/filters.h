#ifndef FILTERS_h
#define FILTERS_h

float avg_filter_1d(float* array, float new_data_point, uint8_t filter_length);
float median_filter (float* i_arr, float new_element, int window_size);

#endif