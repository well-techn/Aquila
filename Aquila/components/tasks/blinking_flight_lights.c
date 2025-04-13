//Задача управления полетными огнями
//В зависимости от состояния мограет полетными огнями с разной задержкой

//стандартные библиотеки
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

//собственные библиотеки
#include "wt_alldef.h"

void blinking_flight_lights(void * pvParameters)
{
  uint32_t blinking_mode = 0;

  while(1) 
  {
    xTaskNotifyWait(0,0,&blinking_mode,NULL);
  
    if (blinking_mode == 0)                 //аварийный режим (1 зеленый 1 красный)
    {
      gpio_set_level(GREEN_FLIGHT_LIGHTS, 1);
      gpio_set_level(RED_FLIGHT_LIGHTS, 1);
      vTaskDelay(50/portTICK_PERIOD_MS);
      
      gpio_set_level(GREEN_FLIGHT_LIGHTS, 0);
      gpio_set_level(RED_FLIGHT_LIGHTS, 0);
      vTaskDelay(250/portTICK_PERIOD_MS);
    }

    if (blinking_mode == 1)               //штатный режим (2 зеленых 1 красный)
    {
      gpio_set_level(GREEN_FLIGHT_LIGHTS, 1);
      vTaskDelay(50/portTICK_PERIOD_MS);
      gpio_set_level(GREEN_FLIGHT_LIGHTS, 0);
      vTaskDelay(100/portTICK_PERIOD_MS);
      gpio_set_level(GREEN_FLIGHT_LIGHTS, 1);
      vTaskDelay(50/portTICK_PERIOD_MS);
      gpio_set_level(GREEN_FLIGHT_LIGHTS, 0);
      vTaskDelay(500/portTICK_PERIOD_MS);

      gpio_set_level(RED_FLIGHT_LIGHTS, 1);
      vTaskDelay(50/portTICK_PERIOD_MS);
      gpio_set_level(RED_FLIGHT_LIGHTS, 0);
      vTaskDelay(750/portTICK_PERIOD_MS);
    } 
  }
}
