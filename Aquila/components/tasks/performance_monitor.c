//Задача печати статистики загруженности системы.
//Получает данные vTaskGetRunTimeStats и петатает их раз в 5 секунд. Только для режима диагностики.

//системные библиотеки
#include "freertos/FreeRTOS.h"

void performance_monitor(void * pvParameters)
{
   char statbuf[800];
  
  while(1)
  {
    vTaskGetRunTimeStats(statbuf);
    printf("%s\n",statbuf);
    vTaskDelay(5000/portTICK_PERIOD_MS);
  } 
}
