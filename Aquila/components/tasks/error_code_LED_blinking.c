/*
задача моргания светодиодами на плате при возниконовении какой-либо ошибки при прохождении первичной инициализации железа.
Принимает в качестве параметра при создании <error_code> и моргает светодиодами <error_code> раз для отображения кода ошибки.
синий отвечает за десятки, зеленый за единицы, 
например, при коде ошибки 25 два раза моргнет синий и 5 раз зеленый

коды ошибок
1	Ошибка создания семафоров или очередей
2	Ошибка создания задач
3	нет связи MCP23017
4	ошибка настройки MCP23017
5	нет связи PCA9685
6	ошибка настройки PCA9685
7	нет связи INA219
8	ошибка настройки INA219
9	нет связи MS5611
10	ошибка настройки MS5611
11	нет связи FL3195
12	ошибка настройки FL3195
13	нет связи IST8310
14	ошибка настройки IST8310
15	ошибка самотестирования IST8310
16	нет связи MPU 1
17	ошибка настройки MPU 1
18	нет связи MPU 2
19	ошибка настройки MPU 2
20	Ошибка JEDEC W25N
21	нет связи с лидаром
22	нет связи с px4flow
23	ошибка калибровочных коэффициентов
24	ошибка системы питания
*/

// стандартные библиотеки
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

// собственные библиотеки
#include "wt_alldef.h"

void error_code_LED_blinking(void *pvParameters)
{
  uint8_t i = 0;
  uint8_t *error_code = pvParameters;

  while (1)
  {
// отмаргиваем синим десятки
    for (i = 0; i < (*error_code / 10); i++)
    {
      gpio_set_level(LED_BLUE, 0);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      gpio_set_level(LED_BLUE, 1);
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);

// отмаргиваем зеленым единицы
    for (i = 0; i < (*error_code % 10); i++)
    {
      gpio_set_level(LED_GREEN, 0);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      gpio_set_level(LED_GREEN, 1);
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
