#include "wt_alldef.h"
#include "driver/ledc.h"
#include "stdio.h"
//статическая переменная, которая хранит состояние
//нужна для того, что если повторно вызывать init поверх предыдущего выдаются warnings 
static bool pwm_already_inited = 0;

void pwm_esc_control_init(uint8_t engine_0_pin, uint8_t engine_1_pin, uint8_t engine_2_pin, uint8_t engine_3_pin)
{
    if (!pwm_already_inited)
    {
//создаем таймер для управления двигателями через LEDC
        ledc_timer_config_t engine_pwm_timer = {
            .speed_mode       = ENGINE_PWM_MODE,
            .timer_num        = ENGINE_PWM_TIMER,
            .duty_resolution  = ENGINE_PWM_DUTY_RESOLUTION,
            .freq_hz          = ENGINE_PWM_FREQUENCY,  
            .clk_cfg          = LEDC_AUTO_CLK 
        };
        ESP_ERROR_CHECK(ledc_timer_config(&engine_pwm_timer));

//создаем 4 канала для LEDC 
        ledc_channel_config_t engine_pwm_channel_0 = {
            .speed_mode     = ENGINE_PWM_MODE,
            .channel        = 0,
            .timer_sel      = ENGINE_PWM_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = engine_0_pin,
            .duty           = ENGINE_MIN_SIGNAL,
            .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&engine_pwm_channel_0));

        ledc_channel_config_t engine_pwm_channel_1 = {
        .speed_mode     = ENGINE_PWM_MODE,
        .channel        = 1,
        .timer_sel      = ENGINE_PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = engine_1_pin,
        .duty           = ENGINE_MIN_SIGNAL,
        .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&engine_pwm_channel_1));

        ledc_channel_config_t engine_pwm_channel_2 = {
        .speed_mode     = ENGINE_PWM_MODE,
        .channel        = 2,
        .timer_sel      = ENGINE_PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = engine_2_pin,
        .duty           = ENGINE_MIN_SIGNAL,
        .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&engine_pwm_channel_2));

        ledc_channel_config_t engine_pwm_channel_3 = {
        .speed_mode     = ENGINE_PWM_MODE,
        .channel        = 3,
        .timer_sel      = ENGINE_PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = engine_3_pin,
        .duty           = ENGINE_MIN_SIGNAL,
        .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&engine_pwm_channel_3)); 

        pwm_already_inited = 1;
    }
}

void pwm_esc_control_update(float* engines_signal)
{
//преобразуем сигнал [0..1]  в [6553..13016] и отправляем на ШИМ    
    ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 0, (uint32_t)(engines_signal[0] * 6553.0f + 6553.0f)));
    ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 0));

    ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 1, (uint32_t)(engines_signal[1] * 6553.0f + 6553.0f)));
    ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 1));

    ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 2, (uint32_t)(engines_signal[2] * 6553.0f + 6553.0f)));
    ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 2));

    ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 3, (uint32_t)(engines_signal[3] * 6553.0f + 6553.0f)));
    ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 3));
}

