#ifndef GPIO_OPS_H
#define GPIO_OPS_H

esp_err_t сonfigure_pin_for_interrupt(gpio_num_t gpio_number, gpio_pull_mode_t pull_mode, gpio_int_type_t edge);

#endif