#include "driver/ledc.h"
#include "wt_alldef.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

void engines_test(void * pvParameters)
{
    while(1)
    {
        for (uint8_t i = 10; i > 0; i--)
        {
            printf("%d\n",i);
            vTaskDelay(1000/portTICK_PERIOD_MS);
        }   
        ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 0, ENGINE_PWM_MIN_DUTY * 1.5));                  //проверяем двигатели на среднем уровне сигнала
        ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 0));
        vTaskDelay(TIME_TO_KEEP_RUNNING_AT_CHECK_MS/portTICK_PERIOD_MS);

        ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 0, ENGINE_PWM_MIN_DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 0));
        vTaskDelay(1000/portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 1, ENGINE_PWM_MIN_DUTY * 1.5));
        ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 1));
        vTaskDelay(TIME_TO_KEEP_RUNNING_AT_CHECK_MS/portTICK_PERIOD_MS);

        ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 1, ENGINE_PWM_MIN_DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 1));
        vTaskDelay(1000/portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 2, ENGINE_PWM_MIN_DUTY * 1.5));
        ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 2));
        vTaskDelay(TIME_TO_KEEP_RUNNING_AT_CHECK_MS/portTICK_PERIOD_MS);
        
        ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 2, ENGINE_PWM_MIN_DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 2));
        vTaskDelay(1000/portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 3, ENGINE_PWM_MIN_DUTY * 1.5));
        ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 3));
        vTaskDelay(TIME_TO_KEEP_RUNNING_AT_CHECK_MS/portTICK_PERIOD_MS);

        ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 3, ENGINE_PWM_MIN_DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 3));
        
        ESP_LOGI("режим конфигурирования","Проверка двигатетей завершена, нажмите ESC для перезапуска");
        vTaskDelete(NULL);
    }
}