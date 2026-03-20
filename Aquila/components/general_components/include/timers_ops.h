#ifndef TIMERS_OPS_H
#define TIMERS_OPS_H

#include "esp_err.h"           // Для esp_err_t
#include "driver/gptimer.h"    // Для gptimer_handle_t и gptimer_alarm_cb_t


esp_err_t create_and_start_timer(uint32_t frequency_hz, gptimer_handle_t* timer_handle);

esp_err_t create_and_start_timer_with_interrupt(gptimer_handle_t *timer_handle,  uint32_t frequency_hz, uint64_t delay_us, gptimer_alarm_cb_t callback_function);

#endif