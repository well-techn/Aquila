// Инициализация PID-регулятора
#include "pid.h"
#include "inttypes.h"
#include "stdio.h"

//alpha должна быть больше 0,5!!!
void PID_Init(PIDController_t *pid, float kp, float ki, float kd, float alpha, float integral_limit, float pid_limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error = 0;
    pid->prev_error = 0;
    pid->error_diff = 0;
    pid->alpha = alpha;
    pid->error_diff_filtered = 0;
    pid->integral_error = 0;
    pid->integral_limit = integral_limit;
    pid->pid_limit = pid_limit;
}

// Вычисление управляющего воздействия, alpha должна быть больше 0.5, иначе фильтрация по D составляющей отключается
float PID_Compute(PIDController_t *pid, float setpoint, float measured_value, float throttle) {
    pid->error = setpoint - measured_value;  // Ошибка регулирования
    pid->integral_error += pid->ki * pid->error;         // Интегральная составляющая
    if  (pid->integral_error > pid->integral_limit) pid->integral_error = pid->integral_limit;
    if  (pid->integral_error < -pid->integral_limit) pid->integral_error = -pid->integral_limit;               
        
    pid->error_diff = (pid->error - pid->prev_error);  // Дифференциальная составляющая
//при правильном коэффициенте активируется ФНЧ на входе диф составляющей, в противном случае отключается
    if ((pid->alpha >= 0.5) && (pid->alpha < 1.0))  pid->error_diff_filtered = pid->alpha * pid->error_diff_filtered  + (1 - pid->alpha) * pid->error_diff;
    else pid->error_diff_filtered = pid->error_diff;
//чтобы не накапливалось пока стоит на земле
    if (throttle < pid->throttle_limit) 
    {
        pid->integral_error = 0;              
        pid->error_diff = 0;
        pid->error_diff_filtered = 0;
    }

    // Управляющее воздействие
    float output = pid->kp * pid->error +  pid->integral_error + pid->kd * pid->error_diff_filtered;
    if  (output > pid->pid_limit) output = pid->pid_limit;
    if  (output < -pid->pid_limit) output = -pid->pid_limit; 

     pid->prev_error = pid->error;  // Сохраняем текущую ошибку для следующего вызова
    //if (pid->print_results) printf("%0.2f, %0.2f, %0.2f, %0.2f, %0.2f\n",error, pid->kp * error, derivative, pid->integral_error, output);
    return output;
}

   