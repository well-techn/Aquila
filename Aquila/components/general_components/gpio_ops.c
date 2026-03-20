//настройка пина под прерывание
#include "driver/gpio.h"
#include "soc/gpio_reg.h"

esp_err_t сonfigure_pin_for_interrupt(gpio_num_t gpio_number, gpio_pull_mode_t pull_mode, gpio_int_type_t edge)
{
  esp_err_t err = gpio_reset_pin(gpio_number);

  gpio_config_t io_config = {
    .pin_bit_mask = 1ULL << gpio_number,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = (pull_mode == GPIO_PULLUP_ONLY || pull_mode == GPIO_PULLUP_PULLDOWN) ? 1 : 0,
    .pull_down_en = (pull_mode == GPIO_PULLDOWN_ONLY || pull_mode == GPIO_PULLUP_PULLDOWN) ? 1 : 0,
    .intr_type = edge
  }; 

  return (gpio_config(&io_config));
}