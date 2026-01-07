
#include <inttypes.h>
#include <filters.h>
#include <math.h>
#include <stdio.h>

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

void Kalman_2d_predict(float acceleration, KalmanFilter2d_t* this)
{
    //вспомогательная переменная "время в квадрате" для уменьшения операций умножения
    float t2 = this->dt * this->dt;
    
    //printf("%f\n",this->h);

    //прогноз (обычные уравнения)
    this->h = this->h + this->v * this->dt + 0.5f * acceleration * t2;
	this->v = this->v + acceleration * this->dt;
    
    //неопределенность прогноза
    //вспомогательная переменная для уменьшения операций умножения 
    float temp = this->sigma2_accel * t2;
    this->P[0][0] = this->P[0][0] + (this->P[1][0] + this->P[0][1] + (this->P[1][1] + 0.25 * temp) * this->dt) * this->dt;
	this->P[0][1] = this->P[0][1] + (this->P[1][1] + 0.5 * temp) * this->dt;
	this->P[1][0] = this->P[1][0] + (this->P[1][1] + 0.5 * temp) * this->dt;
	this->P[1][1] = this->P[1][1] + temp;   
}

void Kalman_2d_update(float baro_height, KalmanFilter2d_t* this)
{
    float y = baro_height - this->h;
    
    //считаем коэффициент Калмана
    float S_inv = 1.0f / (this->P[0][0] + this->sigma2_baro);
    this->K[0] = this->P[0][0] * S_inv; //для координаты
    this->K[1] = this->P[1][0] * S_inv; //для скорости
    
    //расчет результата
    this->h += this->K[0] * y;
	this->v += this->K[1] * y;
    
    //вычисление точности результата
     this->P[0][0] = this->P[0][0] - this->K[0] * this->P[0][0];
	 this->P[0][1] = this->P[0][1] - this->K[0] * this->P[0][1];
	 this->P[1][0] = this->P[1][0] - this->K[1] * this->P[0][0];
	 this->P[1][1] = this->P[1][1] - this->K[1] * this->P[0][1];
}

void Butterworth_init(float f_sampling, float f_cut, Butterworth_t* this) 
{   
    float omega = 2.0 * M_PI * f_cut / f_sampling; // Нормированная частота
    float theta = omega / 2.0;
    float sn = sinf(theta);
    float cs = cosf(theta);
    float alpha = sn / (2.0 * sqrt(2.0)); // Для порядка 2

    // Коэффициенты передаточной функции
    float b0_raw = (1.0 - cs) / 2.0;
    float b1_raw = 1.0 - cs;
    float b2_raw = (1.0 - cs) / 2.0;
    float a0_raw = 1.0 + alpha;
    float a1_raw = -2.0 * cs;
    float a2_raw = 1.0 - alpha;

    // финально коэффициенты
    this->coef_in_0 = b0_raw / a0_raw;
    this->coef_in_1 = b1_raw / a0_raw;
    this->coef_in_2 = b2_raw / a0_raw;
    this->coef_out_1 = a1_raw / a0_raw;
    this->coef_out_2 = a2_raw/ a0_raw;
    
    //исходные значения буфера
    this->prev_in_1 = 0;
    this->prev_in_2 = 0;
    this->prev_out_1 = 0;
    this->prev_out_2 = 0;
}

float Butterworth_filter(float input, Butterworth_t* state) 
{
    float output = 
        state->coef_in_0 * input + 
        state->coef_in_1 * state->prev_in_1 + 
        state->coef_in_2 * state->prev_in_2 - 
        state->coef_out_1 * state->prev_out_1 - 
        state->coef_out_2 * state->prev_out_2;

    // Обновление состояний
    state->prev_in_2 = state->prev_in_1;
    state->prev_in_1 = input;
    state->prev_out_2 = state->prev_out_1;
    state->prev_out_1 = output;

    return output;
}

uint8_t arrays_are_equal (uint8_t* array_1, uint8_t* array_2, uint8_t length)
{
    for (int i = 0; i<length; i++) 
    {
        if (array_1[i] != array_2[i]) return 0;
    }
    return 1;
}

uint8_t all_array_elements_are_equal(uint8_t* array, uint8_t length) 
{
     for (uint8_t i = 1; i < length; i++) {
        if (array[i] != array[0]) 
            return 0;
    }
    return 1;
}