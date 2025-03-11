// Инициализация PID-регулятора
// Структура для хранения состояния PID-регулятора
typedef struct {
    float kp;          // Коэффициент пропорциональной составляющей
    float ki;          // Коэффициент интегральной составляющей
    float kd;          // Коэффициент дифференциальной составляющей
    float prev_error;  // Предыдущее значение ошибки
    float integral;    // Интегральная составляющая
    float integral_limit;    // Границы интегральной составляющей
    float pid_limit;    // Общий лимит
} PIDController_t

void PID_Init(PIDController *pid, float kp, float ki, float kd, float integral_limit, float pid_limit ) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->integral_limit = integral_limit;
    pid->pid_limit = pid_lmit;
}

// Вычисление управляющего воздействия
float PID_Compute(PIDController *pid, float setpoint, float measured_value) {
    float error = setpoint - measured_value;  // Ошибка регулирования
    pid->integral += pid->ki * error;         // Интегральная составляющая 
    if  (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if  (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;               
    float derivative = (error - pid->prev_error);  // Дифференциальная составляющая

    // Управляющее воздействие
    float output = pid->kp * error +  pid->integral + pid->kd * derivative;
    if  (output > pid->pid_limit) pid->integral = pid->pid_limit;
    if  (output < -pid->pid_limit) pid->integral = -pid->pid_limit; 

    pid->prev_error = error;  // Сохраняем текущую ошибку для следующего вызова
    return output;
}