#ifndef INIT_H
#define INIT_H

void configure_IOs();
void configuring_timer_for_PWM();
void configuring_channel_for_PWM(uint8_t channel, uint8_t pin);
void init(void * pvParameters);

#endif