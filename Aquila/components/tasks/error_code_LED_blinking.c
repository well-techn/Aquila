//задача моргания светодиодами на плате для отображения кода ошибки. Она создается при возниконовении какой-либо ошибки при прохождении первичной инициализации железа.
//Принимает в качестве параметра при создании <error_code> и моргает светодиодами <error_code> раз для отображения кода ошибки.

//стандартные библиотеки
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

//собственные библиотеки
#include "wt_alldef.h"

void error_code_LED_blinking(void * pvParameters)              
{
  uint8_t i = 0;
  uint8_t *error_code = pvParameters;
  while(1) 
  {
    gpio_set_level(LED_RED, 1);
    gpio_set_level(LED_BLUE, 1);
    gpio_set_level(LED_GREEN, 1);
    vTaskDelay(2000/portTICK_PERIOD_MS);
    for (i=0; i<*error_code; i++) {
      gpio_set_level(LED_RED, 0);
      gpio_set_level(LED_BLUE, 0);
      gpio_set_level(LED_GREEN, 0);
      vTaskDelay(250/portTICK_PERIOD_MS);
      gpio_set_level(LED_RED, 1);
      gpio_set_level(LED_BLUE, 1);
      gpio_set_level(LED_GREEN, 1);
      vTaskDelay(250/portTICK_PERIOD_MS);
    }
  }
}