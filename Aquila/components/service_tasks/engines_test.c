#include "driver/ledc.h"
#include "wt_alldef.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#ifdef TELNET_CONF_MODE
    #include <lwip/sockets.h>
#endif
#include "esc_control_provider.h"
#include "dshot_esc_encoder.h"
#include <rom/ets_sys.h>

extern const char *TAG_SERVICE;

//Вспомогательная функция для вывода
void engine_test_log(int16_t fd, const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    uint8_t len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

//Печать ESP_LOG в терминал
    printf("%s\n", buffer);
//печать в telnet
#ifdef TELNET_CONF_MODE
    if (fd >= 0) {
        send(fd, buffer, len, 0);
    }
#endif
}

void engines_test(void *pvParameters) {
    int16_t client_fd = (pvParameters) ? *(int16_t *)pvParameters : -1;
    float signals[4] = {0};
    uint16_t counter = 0;
    engine_test_log(client_fd, "Реинициализируем модуль управления двигателями\r\n");
    esc_control_initialize(0);
    engine_test_log(client_fd, "Модуль управления двигателями реинициализирован\r\n");

//задержка чтобы моторы заармились с поданным сигналом 48
    while(counter < 1000)
    {
    esc_control_update(signals);
    ets_delay_us(500);
    counter++;
    }

//обратный отсчет
    // for (int8_t i = 5; i >= 0; i--) {
    //     engine_test_log(client_fd, "\rДо старта %d секунд", i);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
    // engine_test_log(client_fd, "\r\n");

//включаем по кругу двигатели
    for (uint8_t i = 0; i < 4; i++) {
//Запуск
        engine_test_log(client_fd, "Запускаем двигатель %d\r\n", i);
        signals[i] = 0.6f;
        while(counter < TIME_TO_KEEP_RUNNING_AT_CHECK_MS)
        {
            esc_control_update(signals);
            ets_delay_us(500);
            counter++;
        }
        counter = 0;


//Остановка
        engine_test_log(client_fd, "Останавливаем двигатель %d\r\n", i);
        signals[i] = 0.0f;
        while(counter < 1000)
        {
            esc_control_update(signals);
            ets_delay_us(500);
            counter++;
        }
        counter = 0;
    }

    engine_test_log(client_fd, "Проверка завершена, нажмите ESC для перезапуска или повторите выбор меню\r\n");
    
    vTaskDelete(NULL);
}