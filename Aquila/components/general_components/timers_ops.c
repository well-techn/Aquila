#include "driver/gptimer.h"


esp_err_t create_and_start_timer(uint32_t frequency_hz, gptimer_handle_t* timer_handle)                    
{
  gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = frequency_hz,
  };
  esp_err_t err = gptimer_new_timer(&timer_config, timer_handle);
  if (err != ESP_OK) return err;

  err = gptimer_enable(*timer_handle);
  if (err != ESP_OK) return err;
  
   err = gptimer_start(*timer_handle);
   return err;
}

esp_err_t create_and_start_timer_with_interrupt(gptimer_handle_t *timer_handle, uint32_t frequency_hz, uint64_t delay_us, gptimer_alarm_cb_t callback_function) 
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = frequency_hz, // 1MHz = 1us
    };
    esp_err_t err = gptimer_new_timer(&timer_config, timer_handle);
    if (err != ESP_OK) return err;

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = delay_us,
        .flags.auto_reload_on_alarm = false, // если нужно разовое срабатывание
    };
    err = gptimer_set_alarm_action(*timer_handle, &alarm_config);
    if (err != ESP_OK) return err;

    gptimer_event_callbacks_t cbs = {
        .on_alarm = callback_function,
    };
    err = gptimer_register_event_callbacks(*timer_handle, &cbs, NULL);
    if (err != ESP_OK) return err;

    ESP_ERROR_CHECK(gptimer_enable(*timer_handle));
    return gptimer_start(*timer_handle);
}