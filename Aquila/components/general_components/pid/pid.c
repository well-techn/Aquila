// Инициализация PID-регулятора
#include "pid.h"
#include "inttypes.h"
#include "stdio.h"
#include "wt_alldef.h"

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
    float dt = (float)CYCLE_TIMESTEP_S;
//ошибка регулирования    
    pid->error = setpoint - measured_value;
//ограничиваем ошибку только в случае если это yaw_angle, хз как это сделать по-другому
    if(pid->is_yaw_angle)
    {
        if (pid->error <= -180.0f) pid->error += 360.0f; 
        if (pid->error >= 180.0f)  pid->error -= 360.0f;
    }

// Интегральная составляющая    
    pid->integral_error += pid->ki * pid->error * dt;         
    if  (pid->integral_error > pid->integral_limit) pid->integral_error = pid->integral_limit;
    if  (pid->integral_error < -pid->integral_limit) pid->integral_error = -pid->integral_limit;      

//дифференциальная составляющая       
    pid->error_diff = (pid->prev_measurement - measured_value);
    if(pid->is_yaw_angle)
    {
        if (pid->error_diff <= -180.0f) pid->error_diff += 360.0f; 
        if (pid->error_diff >= 180.0f)  pid->error_diff -= 360.0f;
    }
    pid->error_diff = pid->error_diff / dt;
    
//при правильном коэффициенте активируется ФНЧ на входе диф составляющей, в противном случае отключается
    if ((pid->alpha >= 0.5) && (pid->alpha < 1.0))  pid->error_diff_filtered = pid->alpha * pid->error_diff_filtered  + (1 - pid->alpha) * pid->error_diff;
    else pid->error_diff_filtered = pid->error_diff;

//FF составляющая
    float error_ff = (setpoint - pid->prev_setpoint);
    if (pid->is_yaw_angle) {
        if (error_ff <= -180.0f) error_ff += 360.0f;
        if (error_ff >= 180.0f)  error_ff -= 360.0f;
    }
    error_ff = error_ff / dt;
    pid->ff_filtered = pid->alpha_ff * pid->ff_filtered + (1.0f - pid->alpha_ff) * error_ff;
 
//чтобы не накапливалось пока стоит на земле
    if (throttle < pid->throttle_limit) 
    {
        pid->integral_error = 0;              
        pid->error_diff = 0;
        pid->error_diff_filtered = 0;
        pid->prev_measurement = measured_value;
        pid->prev_setpoint = setpoint;
    }

//вычисление общего управляющего воздействия
    float output = pid->kp * pid->error +  pid->integral_error + pid->kd * pid->error_diff_filtered + pid->kff * pid->ff_filtered;
    if  (output > pid->pid_limit) output = pid->pid_limit;
    if  (output < -pid->pid_limit) output = -pid->pid_limit; 

    pid->prev_measurement = measured_value;
    pid->prev_setpoint = setpoint;
    //if (pid->print_results) printf("%0.2f\n", output);
    return output;
}
