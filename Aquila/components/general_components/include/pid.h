#ifndef PID_h
#define PID_h

#include "inttypes.h"

// Структура для хранения состояния PID-регулятора
typedef struct {
    float kp;          // Коэффициент пропорциональной составляющей
    float ki;          // Коэффициент интегральной составляющей
    float kd;          // Коэффициент дифференциальной составляющей
    float prev_error;  // Предыдущее значение ошибки
    float integral_error;    // Интегральная составляющая
    float integral_limit;    // Границы интегральной составляющей
    float pid_limit;    // Общий лимит
    float throttle_limit;
    uint8_t print_results;
} PIDController_t;

void PID_Init(PIDController_t *pid, float kp, float ki, float kd, float integral_limit, float pid_limit);
float PID_Compute(PIDController_t *pid, float setpoint, float measured_value, float throttle);



#endif