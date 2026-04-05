#ifndef DSHOT_H 
#define DSHOT_H

void dshot_esc_control_init(uint8_t engine_0_pin, uint8_t engine_1_pin, uint8_t engine_2_pin, uint8_t engine_3_pin, int8_t loop_count);
void dshot_esc_control_update(float* engines_signal);

#endif