
#include <inttypes.h>

// среднее арифметическое из массива
float avg_filter_1d(float* array, float new_data_point, uint8_t filter_length)
{
    uint8_t i = 0;
    float accum = 0;
  
    for (i = 0; i < filter_length - 1; i++) array[i] = array[i+1];
    array[filter_length-1] = new_data_point;
    for (i = 0; i < filter_length; i++) accum += array[i];
    
    return (accum/filter_length);        
}


// Медианный фильтр с быстрой сортировкой (deep seek)
void swap(float* a, float* b) {
    float temp = *a;
    *a = *b;
    *b = temp;
}

int partition(float* arr, int left, int right) {
    float pivot = arr[right]; // Выбираем последний элемент как pivot
    int i = left - 1;

    for (int j = left; j <= right - 1; j++) {
        if (arr[j] <= pivot) {
            i++;
            swap(&arr[i], &arr[j]);
        }
    }
    swap(&arr[i + 1], &arr[right]);
    return i + 1;
}

float quickselect(float* arr, int left, int right, int k) {
    if (left <= right) {
        int pivot_index = partition(arr, left, right);
        
        if (pivot_index == k) {
            return arr[pivot_index];
        } else if (pivot_index > k) {
            return quickselect(arr, left, pivot_index - 1, k);
        } else {
            return quickselect(arr, pivot_index + 1, right, k);
        }
    }
    return -1; // Ошибка
}

float median_filter (float* i_arr, float new_element, int window_size)
{
    float op_arr[window_size];
    for (int i = 0; i<window_size-1; i++) i_arr[i] = i_arr[i+1];
    i_arr[window_size - 1] = new_element;
    
    for (int i = 0; i<window_size; i++) op_arr[i] = i_arr[i];
    
    float result = quickselect(op_arr, 0, window_size-1, window_size/2);
    return result;
}