//Задача считывания данных с INA219 (мониторинг тока и напряжений)
//Активируется из main_flying_cycle, возвращает через очередь данные о напряжении АКБ, токе, мощности и затраченной энергии


//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "wt_i2c.h"
#include "INA219.h"
#include "timing.h"


extern SemaphoreHandle_t semaphore_for_i2c_internal;
extern QueueHandle_t INA219_to_main_queue;
extern char *TAG_INA219;

void INA219_read_and_process_data(void * pvParameters)
{
  float INA219_data[4] = {0.0, 0.0, 0.0, 0.0};  // volt, current, power, consumed
  int64_t prev_time = 0;
  int64_t current_time = 0;
  
  while(1) 
  {
      if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) != 0)
      {
      INA219_data[0] = INA219_read_voltage();
      INA219_data[1] = INA219_read_current();
      INA219_data[2] = INA219_read_power();
      current_time = get_time();
      INA219_data[3] += (INA219_data[2] * ((float)(current_time - prev_time) / 1000000L)) * (1000.0 / 11.1) / 3600.0; //consumed energy in mA*h
      
      ESP_LOGI(TAG_INA219, "V: %0.3fV, I: %0.3fA, P: %0.3fW, A: %0.3fmAh",INA219_data[0], INA219_data[1], INA219_data[2], INA219_data[3]);
      
      prev_time = current_time;
      xQueueSend(INA219_to_main_queue, (void *) INA219_data, NULL); 
      }
  }
}