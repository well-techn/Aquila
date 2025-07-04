#include "driver/ledc.h"
#include "wt_alldef.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#ifdef TELNET_CONF_MODE
#include <lwip/sockets.h>
#endif

extern const char *TAG_SERVICE;

void engines_test(void *pvParameters)
{
    int16_t *client_fd = pvParameters;
    char message_to_print[200];
    uint8_t pos = 0;

    char start_message[] = "Запускаем двигатель";
    char stop_message[] = "Останавливаем двигатель";
    char finish_message[] = "Проверка двигателей завершена, нажмите ESC для перезапуска\r\n";

    while (1)
    {
        // даем обратный отсчет 10 секунд
        for (int8_t i = 9; i >= 0; i--)
        {
            printf("%d\n", i);
#ifdef TELNET_CONF_MODE
            pos = sprintf(message_to_print, "\rДо старта %d секунд", i);
            send(*client_fd, message_to_print, pos, 0);
#endif
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

#ifdef TELNET_CONF_MODE
            send(*client_fd, "\r\n", 2, 0);
#endif

        // начинаем последовательно включать и выключать двигатели
        for (uint8_t i = 0; i < 4; i++)
        {
            // сообщение
            pos = sprintf(message_to_print, "%s %d\r\n", start_message, i);
            ESP_LOGI(TAG_SERVICE, "%s", message_to_print);
#ifdef TELNET_CONF_MODE
            send(*client_fd, message_to_print, pos, 0);
#endif
            // запуск
            ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, i, ENGINE_PWM_MIN_DUTY * 1.5)); // проверяем двигатели на среднем уровне сигнала
            ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, i));
            vTaskDelay(TIME_TO_KEEP_RUNNING_AT_CHECK_MS / portTICK_PERIOD_MS);

            // сообщение
            pos = sprintf(message_to_print, "%s %d\r\n", stop_message, i);
            ESP_LOGI(TAG_SERVICE, "%s", message_to_print);
#ifdef TELNET_CONF_MODE
            send(*client_fd, message_to_print, pos, 0);
#endif
            // остановка
            ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, i, ENGINE_PWM_MIN_DUTY));
            ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, i));
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        // проверка завершена, выводим финальное сообщение
        ESP_LOGI(TAG_SERVICE, "%s", finish_message);
#ifdef TELNET_CONF_MODE
        send(*client_fd, finish_message, sizeof(finish_message), 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
#endif

        vTaskDelete(NULL);
    }
}