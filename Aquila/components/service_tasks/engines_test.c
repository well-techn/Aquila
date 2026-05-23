#include "freertos/FreeRTOS.h"
#include <rom/ets_sys.h>

#include "wt_alldef.h"
#include "logging_operations.h"
#include "esc_control_provider.h"


void engines_test(void *pvParameters) {
    int16_t client_fd = (pvParameters) ? *(int16_t *)pvParameters : -1;
    float signals[4] = {0};
    uint16_t counter = 0;
    print_service_message(client_fd, "Реинициализируем модуль управления двигателями\r\n");
    esc_control_initialize(0);
    print_service_message(client_fd, "Модуль управления двигателями реинициализирован\r\n");

//задержка чтобы моторы заармились с поданным сигналом 48
    while(counter < 2000)
    {
    esc_control_update(signals);
    vTaskDelay(1);
    counter++;
    }

//включаем по кругу двигатели
    for (uint8_t i = 0; i < 4; i++) {
//Запуск
        print_service_message(client_fd, "Запускаем двигатель %d\r\n", i);
        signals[i] = 0.4f;
        while(counter < TIME_TO_KEEP_RUNNING_AT_CHECK_MS)
        {
            esc_control_update(signals);
            vTaskDelay(1);
            counter++;
        }
        counter = 0;


//Остановка
        print_service_message(client_fd, "Останавливаем двигатель %d\r\n", i);
        signals[i] = 0.0f;
        while(counter < 1000)
        {
            esc_control_update(signals);
            vTaskDelay(1);
            counter++;
        }
        counter = 0;
    }

    print_service_message(client_fd, "Проверка завершена, ESC для перезапуска или повторите выбор меню\r\n");
    
    vTaskDelete(NULL);
}