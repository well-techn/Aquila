#ifndef PWM_H 
#define PWM_H

void pwm_esc_control_init(uint8_t engine_0_pin, uint8_t engine_1_pin, uint8_t engine_2_pin, uint8_t engine_3_pin);
void pwm_esc_control_update(float* engines_signal);

#endif