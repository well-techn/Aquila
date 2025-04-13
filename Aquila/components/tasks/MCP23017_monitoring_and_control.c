
//Задача работы с расширителем портов ввода-вывода MCP23017
//принимает через очередь команду (либо считывания входов, либо управление выходами) и выполняет запрошенную команду
//формат команды
// - 0b10000000  - считывание состояния входов
// - 0b010000xx - установить в 1 выход xx
// - 0b001000xx - установить в 0 выход xx

//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "wt_i2c.h"
#include "MCP23017.h"

extern  char *TAG_MCP23017;
extern QueueHandle_t MCP23017_queue; //очередь для передачи обработанных данных от пульта в main_flying_cycle
extern SemaphoreHandle_t semaphore_for_i2c_internal;

void MCP23017_monitoring_and_control(void * pvParameters)
{
  uint8_t MCP23017_external_request;
  uint8_t MCP23017_inputs_state;
  uint8_t MCP23017_current_outputs_state;
    
  while(1) {
    if(xQueueReceive(MCP23017_queue, &MCP23017_external_request, (TickType_t)portMAX_DELAY))
    {
      ESP_LOGI(TAG_MCP23017,"Received request %02x",MCP23017_external_request); 

      if (MCP23017_external_request == 0b10000000) {                //command to read inputs
        xSemaphoreTake(semaphore_for_i2c_internal,portMAX_DELAY);
        MCP23017_inputs_state = MCP23017_get_inputs_state();
        xSemaphoreGive(semaphore_for_i2c_internal); 
        ESP_LOGI(TAG_MCP23017,"Current input state is %02x",MCP23017_inputs_state);  
      }
      
      if (MCP23017_external_request & 0b01000000) {                 //if bit6 is set that means 2 lower bit represents output to be set
        xSemaphoreTake(semaphore_for_i2c_internal,portMAX_DELAY);
        MCP23017_set_output(&MCP23017_current_outputs_state, MCP23017_external_request & 0b00001111);
        xSemaphoreGive(semaphore_for_i2c_internal);
        ESP_LOGI(TAG_MCP23017,"Got request to set output %d, output is set",MCP23017_external_request & 0b00001111);  
      }

      if (MCP23017_external_request & 0b00100000) {                 //if bit5 is set that means 2 lower bit represents output to be cleared
        xSemaphoreTake(semaphore_for_i2c_internal,portMAX_DELAY);
        MCP23017_clear_output(&MCP23017_current_outputs_state, MCP23017_external_request & 0b00001111);
        xSemaphoreGive(semaphore_for_i2c_internal);
        ESP_LOGI(TAG_MCP23017,"Got request to clear output %d, output is cleared",MCP23017_external_request & 0b00001111);
      }
    }
  }
}