#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "esp_system.h"
#include "esp_log.h"
#include "string.h"
#include "usb_serial_jtag.h"
#include "reading_logs_from_external_flash.h"
#include "engines_test.h"
#include "gyro_calibration.h"
#include "advanced_acc_calibration.h"
#include "advanced_mag_calibration.h"
#include "printing_calibration_coefficients.h"
#include "nvs_flash.h"
#include "wt_alldef.h"

#define SERIAL_BUF_SIZE (1024)
extern TaskHandle_t task_handle_init;
const char *TAG_SERVICE = "SERVICE";


void configuration_mode(void *arg)
{
    nvs_handle_t NVS_handle;
    uint32_t flight_time = 0;    
    
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
// Инициализируем NVS
    esp_err_t ret = nvs_flash_init();
    ESP_ERROR_CHECK(ret);
//открываем на считывание flash   
    ret = nvs_open("storage", NVS_READWRITE, &NVS_handle);
     if (ret != ESP_OK) 
        {
            ESP_LOGE(TAG_SERVICE,"Ошибка (%s) открытия NVS!\n", esp_err_to_name(ret));
        } 
        else 
        {
//считываем flight_time (время налета в секундах)
//если это первый запуск и переменной нет - создаем ее
  ret = nvs_get_u32(NVS_handle, "flight_time", &flight_time); 
  switch (ret) {
      case ESP_OK:
          ESP_LOGD(TAG_SERVICE,"flight time = %ld", flight_time);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_SERVICE,"flight time не определен, используем нулевое значение");
          //ret = nvs_set_u32(NVS_handle, "flight_time", 0);
          break;
      default :
          ESP_LOGE(TAG_SERVICE,"Error (%s) reading!\n", esp_err_to_name(ret));
        }
    }
//разбиваем считанное из flash значение flight_time в сукендах на часы, минуты и секунды
        uint8_t hours = flight_time / 3600;
        uint8_t minutes = (flight_time % 3600) / 60;
        uint8_t seconds = flight_time % 60;

printf("\n***** Устройство в режиме конфигурации *****\n");
printf("*************   Версия  %s ************\n", FW_VERSION);
printf("*************  Налет %02uч %02uм %02uс ***********\n\n", hours, minutes, seconds);
printf("Выберите пункт меню\n");
printf("1 -> скачивание логов\n");
printf("2 -> калибровка гироскопов\n");
printf("3 -> тестирование двигателей\n");
printf("4 -> калибровка ESC (пока не готово)\n");
printf("5 -> продвинутая калибровка акселерометра (по magnetto)\n");
printf("6 -> продвинутая калибровка магнетометра (по magnetto)\n");
printf("7 -> вывести сохраненные калибровочные коэффициенты\r\n");
printf("8 -> продолжить загрузку в обычном режиме\n");
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
                    printf("2 - Запускаем простую калибровку гироскопов\n\n");
                    xTaskCreate(gyro_calibration,"gyro_calibration",4096,NULL,0,NULL);   
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
                    printf("7 - Считываем из flash калибровочные коэффициенты\n");
                    xTaskCreate(printing_calibration_coefficients,"printing_calibration_coefficients",16384,NULL,0,NULL); 
                    break;
                
                case '8':
                    printf("8 - Продолжаем обычную загрузку\n");
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