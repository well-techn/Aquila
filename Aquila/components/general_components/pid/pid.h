#ifndef PID_h
#define PID_h

#include "inttypes.h"

// Структура для хранения состояния PID-регулятора
typedef struct {
    float kp;                       // Коэффициент пропорциональной составляющей
    float ki;                       // Коэффициент интегральной составляющей
    float kd;                       // Коэффициент дифференциальной составляющей
    float kff;                       // Коэффициент FF составляющей
    float error;
    float prev_error;               // Предыдущее значение ошибки
    float prev_measurement;         // Предыдущее значение измерения
    float prev_setpoint;            // Предыдущее значение уставки
    float error_diff;
    float error_diff_filtered;      //для ФНЧ дифференциальной составляющей
    float ff_filtered;              //для ФНЧ FF составляющей
    float alpha;                    //коэффициент для фильтрации дифференциальной составляющей по y[k] = α*y[k-1] + (1-α)*x[k]
    float alpha_ff;                 //коэффициент для фильтрации ff составляющей по y[k] = α*y[k-1] + (1-α)*x[k]
    float integral_error;           // Интегральная составляющая
    float integral_limit;           // Границы интегральной составляющей
    float pid_limit;                // Общий лимит
    float throttle_limit;
    bool is_yaw_angle;
} PIDController_t;

void PID_Init(PIDController_t *pid, float kp, float ki, float kd, float alpha, float integral_limit, float pid_limit);
float PID_Compute(PIDController_t *pid, float setpoint, float measured_value, float throttle);
void send_binary_telemetry(float sp, float input, float output);



#endif