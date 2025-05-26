// Инициализация PID-регулятора
#include "pid.h"
#include "inttypes.h"
#include "stdio.h"

void PID_Init(PIDController_t *pid, float kp, float ki, float kd, float integral_limit, float pid_limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0;
    pid->integral_error = 0;
    pid->integral_limit = integral_limit;
    pid->pid_limit = pid_limit;
}

// Вычисление управляющего воздействия
float PID_Compute(PIDController_t *pid, float setpoint, float measured_value, float throttle) {
    float error = setpoint - measured_value;  // Ошибка регулирования
    pid->integral_error += pid->ki * error;         // Интегральная составляющая
    if  (pid->integral_error > pid->integral_limit) pid->integral_error = pid->integral_limit;
    if  (pid->integral_error < -pid->integral_limit) pid->integral_error = -pid->integral_limit;               
    float derivative = (error - pid->prev_error);  // Дифференциальная составляющая

    if (throttle < pid->throttle_limit) 
    {
        pid->integral_error = 0;              //чтобы не накапливалось пока стоит на земле
        derivative = 0;
    }

    // Управляющее воздействие
    float output = pid->kp * error +  pid->integral_error + pid->kd * derivative;
    if  (output > pid->pid_limit) output = pid->pid_limit;
    if  (output < -pid->pid_limit) output = -pid->pid_limit; 

    pid->prev_error = error;  // Сохраняем текущую ошибку для следующего вызова
    if (pid->print_results) printf("%0.2f, %0.2f, %0.2f, %0.2f, %0.2f\n",error, pid->kp * error, derivative, pid->integral_error, output);
    return output;
}

//outer (angle) pitch cycle
/*    error_pitch_angle = rc_fresh_data.received_pitch - pitch;    
    integral_pitch_error_angle = integral_pitch_error_angle + Ki_pitch_angle * error_pitch_angle;
    if (integral_pitch_error_angle > 1000.0) integral_pitch_error_angle = 1000.0; 
    if (integral_pitch_error_angle < -1000.0) integral_pitch_error_angle =-1000.0;
    if (throttle < 9000.0) integral_pitch_error_angle = 0;              //to avoid accumulation on the ground
    pid_pitch_angle = Kp_pitch_angle * error_pitch_angle + integral_pitch_error_angle + Kd_pitch_angle * (error_pitch_angle - error_pitch_angle_old);     
    if (pid_pitch_angle > 2000.0) {pid_pitch_angle = 2000.0; }
    if (pid_pitch_angle < -2000.0) {pid_pitch_angle = -2000.0; }
    error_pitch_angle_old = error_pitch_angle; 
   float test = PID_Compute(&pitch_pid_angle_1,rc_fresh_data.received_pitch, pitch);
    
    if ((large_counter % 50) == 0) 
    {printf("%0.7f ,", pid_pitch_angle);
     printf("%0.7f\n", test );}
  
//inner (rate) pitch cycle

*/ 


/*    error_pitch_rate = pid_pitch_angle - gyro_pitch;
    //error_pitch_rate = rc_fresh_data.received_pitch - gyro_pitch;         //for inner loop setup
    integral_pitch_error_rate = integral_pitch_error_rate + Ki_pitch_rate * error_pitch_rate;
    if (integral_pitch_error_rate > 1000.0) integral_pitch_error_rate = 1000.0; 
    if (integral_pitch_error_rate < -1000.0) integral_pitch_error_rate = -1000.0;
    diff_pitch_error_rate = Kd_pitch_rate * (error_pitch_rate - error_pitch_rate_old);
    if (throttle < 9000.0) {integral_pitch_error_rate = 0; diff_pitch_error_rate = 0;}             //to avoid accumulation on the ground 
    pid_pitch_rate = Kp_pitch_rate * error_pitch_rate + integral_pitch_error_rate + diff_pitch_error_rate;

    if (pid_pitch_rate > 3000.0) {pid_pitch_rate = 3000.0; }
    if (pid_pitch_rate < -3000.0) {pid_pitch_rate = -3000.0;}
    
    error_pitch_rate_old = error_pitch_rate;

    float test = PID_Compute(&pitch_pid_rate_1,pid_pitch_angle, gyro_pitch, throttle, 9000);
    
    if ((large_counter % 50) == 0) 
    {printf("%0.7f ,", pid_pitch_rate);
     printf("%0.7f\n", test );}
*/

/*    error_roll_angle = rc_fresh_data.received_roll - roll; 
    integral_roll_error_angle = integral_roll_error_angle + Ki_roll_angle * error_roll_angle;
    if (integral_roll_error_angle > 1000.0) integral_roll_error_angle = 1000.0;
    if (integral_roll_error_angle < -1000.0) integral_roll_error_angle =-1000.0;
    if (throttle < 9000.0) integral_roll_error_angle = 0;              //to avoid accumulation on the ground  
    pid_roll_angle = Kp_roll_angle * error_roll_angle + integral_roll_error_angle + Kd_roll_angle * (error_roll_angle - error_roll_angle_old);    
    if (pid_roll_angle > 2000.0) pid_roll_angle = 2000.0;
    if (pid_roll_angle < -2000.0) pid_roll_angle = -2000.0;
    error_roll_angle_old = error_roll_angle;

    float test = PID_Compute(&roll_pid_angle_1,rc_fresh_data.received_roll, roll, throttle, 9000);
    
    if ((large_counter % 50) == 0) 
    {printf("%0.7f ,", pid_roll_angle);
     printf("%0.7f\n", test );}

error_roll_rate = pid_roll_angle - gyro_roll;
    //error_roll_rate = rc_fresh_data.received_roll - gyro_roll;//for inner loop setup
    integral_roll_error_rate = integral_roll_error_rate + Ki_roll_rate * error_roll_rate;
    if (integral_roll_error_rate > 1000.0) integral_roll_error_rate = 1000.0; 
    if (integral_roll_error_rate < -1000.0) integral_roll_error_rate = -1000.0;
    diff_roll_error_rate = Kd_roll_rate * (error_roll_rate - error_roll_rate_old);
    if (throttle < 9000.0) {integral_roll_error_rate = 0; diff_roll_error_rate = 0;}             //to avoid accumulation on the ground 
    pid_roll_rate = Kp_roll_rate * error_roll_rate + integral_roll_error_rate + diff_roll_error_rate;
    if (pid_roll_rate > 3000.0) pid_roll_rate = 3000.0;
    if (pid_roll_rate < -3000.0) pid_roll_rate = -3000.0;
    error_roll_rate_old = error_roll_rate;

    error_yaw_rate = pid_yaw_angle - gyro_yaw;
    //error_yaw_rate = rc_fresh_data.received_yaw - gyro_yaw; //for inner loop setup
    integral_yaw_error_rate = integral_yaw_error_rate + Ki_yaw_rate * error_yaw_rate;
    if (integral_yaw_error_rate > 1000.0) integral_yaw_error_rate = 1000.0; 
    if (integral_yaw_error_rate < -1000.0) integral_yaw_error_rate = -1000.0;
    diff_yaw_error_rate = Kd_yaw_rate * (error_yaw_rate - error_yaw_rate_old); 
    if (throttle < 9000.0) {integral_yaw_error_rate = 0; diff_yaw_error_rate = 0;}            //to avoid accumulation on the ground
    pid_yaw_rate = Kp_yaw_rate * error_yaw_rate + integral_yaw_error_rate + diff_yaw_error_rate;
    if (pid_yaw_rate > 3000.0) pid_yaw_rate = 3000.0;
    if (pid_yaw_rate < -3000.0) pid_yaw_rate = -3000.0;
    error_yaw_rate_old = error_yaw_rate;

    float test = PID_Compute(&yaw_pid_rate_1,pid_yaw_angle, gyro_yaw, throttle, 9000);
    
    if ((large_counter % 50) == 0) 
    {printf("%0.7f ,", pid_yaw_rate );
     printf("%0.7f\n", test );}





            error_altitude = altitude_setpoint - lidar_altitude_corrected;
              integral_alt_error += Ki_alt * error_altitude;                         
              if (integral_alt_error > 250) integral_alt_error = 250;
              if (integral_alt_error < -250) integral_alt_error = -250;

              pid_altitude = Kp_alt * error_altitude + Kd_alt * (error_altitude - previous_error_altitude) + integral_alt_error;
              if (pid_altitude > 500) pid_altitude = 500;
              if (pid_altitude < -500) pid_altitude = -500;
              //printf("%0.2f, %0.2f, %0.2f\n", Kp_alt * error_altitude, Kd_alt * (error_altitude - previous_error_altitude), integral_alt_error);
              previous_error_altitude = error_altitude;
              printf("%0.4f ,",pid_altitude);*/

   