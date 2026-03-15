#include "driver/ledc.h"
#include "wt_alldef.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#ifdef TELNET_CONF_MODE
    #include <lwip/sockets.h>
#endif
#ifdef USING_DSHOT_ESC_CONTROL
    #include "dshot_esc_encoder.h"
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

#ifdef USING_DSHOT_ESC_CONTROL
  rmt_channel_handle_t esc_dshot_tx_channel[4] = {NULL}; // 4 канала для Dshot RMT
  uint8_t esc_dshot_tx_gpio[4] = {ENGINE_OUTPUT_0_PIN,ENGINE_OUTPUT_1_PIN,ENGINE_OUTPUT_2_PIN,ENGINE_OUTPUT_3_PIN};
  rmt_encoder_handle_t esc_dshot_encoder[4] = {NULL};
  rmt_sync_manager_handle_t esc_dshot_synchro = NULL;
  rmt_transmit_config_t esc_dshot_tx_config = {
        .loop_count = -1,                                                                                    
    };
  dshot_esc_throttle_t dshot_signal[4] = {0};
  
 create_and_configure_dshot_rmt_enviroment(esc_dshot_tx_channel, esc_dshot_tx_gpio, esc_dshot_encoder, &esc_dshot_synchro, 4);
#endif

    while (1)
    {
        // даем обратный отсчет 5 секунд
        for (int8_t i = 5; i >= 0; i--)
        {
            printf("%d\r", i);
#ifdef TELNET_CONF_MODE
            pos = sprintf(message_to_print, "\rДо старта %d секунд", i);
            send(*client_fd, message_to_print, pos, 0);
#endif
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

#ifdef TELNET_CONF_MODE
            send(*client_fd, "\r\n", 2, 0);
#endif

//начинаем последовательно включать и выключать двигатели
        for (uint8_t i = 0; i < 4; i++)
        {
            // сообщение
            pos = sprintf(message_to_print, "%s %d\r\n", start_message, i);
            ESP_LOGI(TAG_SERVICE, "%s", message_to_print);
#ifdef TELNET_CONF_MODE
            send(*client_fd, message_to_print, pos, 0);
#endif
//процесс запуска двигателя
#ifdef USING_PWM_ESC_CONTROL             

            ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, i, ENGINE_MIN_SIGNAL* 1.5)); // проверяем двигатели на среднем уровне сигнала
            ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, i));
#else
//выставляем средний для Dshot газ (из 48..2047)
            dshot_signal[i].throttle = 1024;      
//выдаем сигнал на нужный канал          
            ESP_ERROR_CHECK(rmt_transmit(esc_dshot_tx_channel[i], esc_dshot_encoder[i], &dshot_signal[i], sizeof(dshot_signal[i]), &esc_dshot_tx_config));
            ESP_ERROR_CHECK(rmt_disable(esc_dshot_tx_channel[i]));
            ESP_ERROR_CHECK(rmt_enable(esc_dshot_tx_channel[i]));
#endif
            vTaskDelay(TIME_TO_KEEP_RUNNING_AT_CHECK_MS / portTICK_PERIOD_MS);
// сообщение
            pos = sprintf(message_to_print, "%s %d\r\n", stop_message, i);
            ESP_LOGI(TAG_SERVICE, "%s", message_to_print);
#ifdef TELNET_CONF_MODE
            send(*client_fd, message_to_print, pos, 0);
#endif
//процесс остановки двигателя
#ifdef USING_PWM_ESC_CONTROL 
            ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, i, ENGINE_MIN_SIGNAL));
            ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, i));
#else
//выставляем нулевой для Dshot газ (disarm)
            dshot_signal[i].throttle = 0;      
//выдаем сигнал на нужный канал  
            ESP_ERROR_CHECK(rmt_transmit(esc_dshot_tx_channel[i], esc_dshot_encoder[i], &dshot_signal[i], sizeof(dshot_signal[i]), &esc_dshot_tx_config));
            ESP_ERROR_CHECK(rmt_disable(esc_dshot_tx_channel[i]));
            ESP_ERROR_CHECK(rmt_enable(esc_dshot_tx_channel[i]));
#endif
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