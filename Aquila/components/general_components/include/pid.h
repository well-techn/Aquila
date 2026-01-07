#ifndef PID_h
#define PID_h

#include "inttypes.h"

// Структура для хранения состояния PID-регулятора
typedef struct {
    float kp;          // Коэффициент пропорциональной составляющей
    float ki;          // Коэффициент интегральной составляющей
    float kd;          // Коэффициент дифференциальной составляющей
    float error;
    float prev_error;  // Предыдущее значение ошибки
    float error_diff;
    float error_diff_filtered;  //для ФНЧ дифференциальной составляющей
    float alpha;                //коэффициент для фильтрации дифференциальной составляющей по y[k] = α*y[k-1] + (1-α)*x[k]
    float integral_error;    // Интегральная составляющая
    float integral_limit;    // Границы интегральной составляющей
    float pid_limit;    // Общий лимит
    float throttle_limit;
} PIDController_t;

void PID_Init(PIDController_t *pid, float kp, float ki, float kd, float alpha, float integral_limit, float pid_limit);
float PID_Compute(PIDController_t *pid, float setpoint, float measured_value, float throttle);



#endif