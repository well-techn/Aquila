#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "esp_system.h"
#include "esp_log.h"
#include "string.h"
#include "usb_serial_jtag.h"
#include "reading_logs_from_external_flash.h"
#include "engines_test.h"
#include "imu_calibration.h"
#include "advanced_acc_calibration.h"
#include "advanced_mag_calibration.h"

#define SERIAL_BUF_SIZE (1024)
extern TaskHandle_t task_handle_init;

const char *TAG_SERVICE = "SERVICE";

void configuration_mode(void *arg)
{
    //Настраиваем USB SERIAL JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = SERIAL_BUF_SIZE,
        .tx_buffer_size = SERIAL_BUF_SIZE,
    };

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
    ESP_LOGD(TAG_SERVICE, "USB_SERIAL_JTAG настроен");

    // Создаем буфер
    uint8_t *data = (uint8_t *) malloc(SERIAL_BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG_SERVICE, "ошибка выделения памяти");
        return;
    }
uint8_t len = 0;

printf("\n***** Устройство в режиме конфигурации *****\n\n");
printf("Выберете пункт меню\n");
printf("1 -> скачивание логов\n");
printf("2 -> калибровка IMU\n");
printf("3 -> тестирование двигателей\n");
printf("4 -> калибровка ESC (пока не готово)\n");
printf("5 -> продвинутая калибровка акселерометра (по magnetto)\n");
printf("6 -> продвинутая калибровка магнетометра (по magnetto)\n");
printf("7 -> продолжить загрузку в обычном режиме\n");
printf("ESC -> перезапуск\n\n");

    while (1) {
        len = usb_serial_jtag_read_bytes(data, (SERIAL_BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
//если что-то получено  
        if (len) {
//выполняем действия в зависимости от введенной опции  
            switch (data[0]) 
            {
                case '1': 
                    printf("1 - Запускаем скачивание логов, ESC для прерывания\n");
                    xTaskCreate(reading_logs_from_external_flash,"reading_logs_from_external_flash",4096,NULL,0,NULL);
                    break;
                
                case '2':
                    printf("2 - Запускаем простую калибровку IMU (по уровню)\n\n");
                    xTaskCreate(imu_calibration,"imu_calibration",4096,NULL,0,NULL);   
                    break;
                
                case '3':
                    printf("3 - Запускаем тестирование двигателей через 10 секунд, убедитесь в безопасности операции. ESC для прерывания\n");
                    xTaskCreate(engines_test,"engines_test",4096,NULL,0,NULL);    
                    break;

                case '4':
                    printf("4 - Запускаем калибровку ESC\n");
                    printf("Пока не реализовано\n");   
                    break;

                case '5':
                    printf("5 - Запускаем продвинутую калибровку акселерометра (по magnetto)\n");
                    xTaskCreate(advanced_acc_calibration,"advanced_acc_calibration",16384,NULL,0,NULL);   
                    break;

                case '6':
                    printf("6 - Запускаем продвинутую калибровку магнетометра (по magnetto)\n");
                    xTaskCreate(advanced_mag_calibration,"advanced_mag_calibration",16384,NULL,0,NULL);   
                    break;
                
                case '7':
                    printf("7 - Продолжаем обычную загрузку\n");
                    vTaskResume(task_handle_init);
                    vTaskDelete(NULL);  
                    break;
              
                case 0x1B:                                             //esc
                    printf("ESC - перезапускаемся\n\n");
                    esp_restart();
                    break;
                
                default:
                    printf("Неизвестное меню, повторите ввод\n\n");
                    break;
            }
        }
    }
}