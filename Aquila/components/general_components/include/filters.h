#ifndef FILTERS_h
#define FILTERS_h

// Структура для хранения параметров Фильтра Калмана для высоты по барометру и акселерометру
typedef struct {
    float dt;                       // интервал времени
    float sigma2_accel;             // дисперсия акселерометра, постоянная величина как характеристика акселя
    float sigma2_baro;             // дисперсия барометра, постоянная величина как характеристика барометра
    float P[2][2];                  // Матрица неопределенности прогноза
    float K[2];                        // коэффициент Калмана
    float h;                        //высота
    float v;                        // скорость
} KalmanFilter2d_t;

typedef struct {
    float coef_in_0;        //коэф для текущего входа                        
    float coef_in_1;        //коэф для пред входа 
    float coef_in_2;        //коэф для пред-пред входа 
    float coef_out_1;       //коэф для пред выхода 
    float coef_out_2;       //коэф для пред пред выхода
    
    float prev_in_1;        //пред вход
    float prev_in_2;        //пред пред вход
    float prev_out_1;       //пред выход
    float prev_out_2;       //пред пред выход
} Butterworth_t;

float avg_filter_1d(float* array, float new_data_point, uint8_t filter_length);
float median_filter (float* i_arr, float new_element, int window_size);
void Kalman_2d_predict(float acceleration, KalmanFilter2d_t* this);
void Kalman_2d_update(float baro_height, KalmanFilter2d_t* this);
void Butterworth_init(float f_sampling, float f_cut, Butterworth_t* this);
float Butterworth_filter(float input, Butterworth_t* state); 

#endif