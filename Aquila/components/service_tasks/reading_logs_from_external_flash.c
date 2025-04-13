
//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "winbondW25N.h"

//Задача считывания данных из внешней flash. Активируется только если обнаруживается установленная перемычка "считать логи."
//Получает данные с памяти и выдает их в UART
void reading_logs_from_external_flash(void * pvParameters)
{
  while(1) {
    W25N_read_and_print_all();
    while (1)                                         //после как закончили считывать моргаем 
    {
      gpio_set_level(LED_RED, 1);
      gpio_set_level(LED_GREEN, 1);
      gpio_set_level(LED_BLUE, 1);
      vTaskDelay(500/portTICK_PERIOD_MS); 
      gpio_set_level(LED_RED, 0);
      gpio_set_level(LED_GREEN, 0);
      gpio_set_level(LED_BLUE, 0);
      vTaskDelay(500/portTICK_PERIOD_MS); 
    }; 
  }
}