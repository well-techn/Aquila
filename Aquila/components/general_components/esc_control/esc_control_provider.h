#ifndef ESC_CONTROL_PROVIDER_H
#define ESC_CONTROL_PROVIDER_H

void esc_control_initialize(int8_t loop_count);
void esc_control_update(float* engines_signal);

#endif