//Задача управления ШИМ-драйвером PCA9685
//Принимает на вход команду управления (номер выхода + необходимая скважность сигнала) и выполняет ее
//Формат команды - 1 байт, старшие 4 бита - номер выхода, младшие - заполнение в процентах
//например, 0x0103 - выход №1 на 3%

//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "PCA9685.h"

extern  char *TAG_PCA9685;
extern QueueHandle_t PCA9685_queue; //очередь для передачи обработанных данных от пульта в main_flying_cycle
extern SemaphoreHandle_t semaphore_for_i2c_internal;

void PCA9685_control(void * pvParameters)
{
    uint16_t PCA9685_input_command;
    uint8_t pwm_value;
    uint8_t output_number;
    
  while(1) {
    if(xQueueReceive(PCA9685_queue, &PCA9685_input_command, (TickType_t)portMAX_DELAY))
    {
      if (PCA9685_input_command >= 0x1000) ESP_LOGE(TAG_PCA9685,"Received wrong request %04x",PCA9685_input_command);
      else 
      {
        ESP_LOGI(TAG_PCA9685,"Received request %04x",PCA9685_input_command);
        pwm_value = PCA9685_input_command & 0x00FF;
        output_number = (PCA9685_input_command >> 8) & 0x00FF;
        xSemaphoreTake (semaphore_for_i2c_internal,portMAX_DELAY);
        PCA9685_send(pwm_value, output_number);
        xSemaphoreGive (semaphore_for_i2c_internal);
        ESP_LOGI(TAG_PCA9685,"Output #%d set to %d%%",output_number, pwm_value); 
      }  
    }
  }
}