//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "math.h"

//собственные библиотеки
#include "wt_alldef.h"

extern QueueHandle_t MCP23017_queue;
extern QueueHandle_t remote_control_to_main_queue; //очередь для передачи обработанных данных от пульта в main_flying_cycle
extern TaskHandle_t task_handle_send_data_to_RC;

//Задача получения и обработки данных от пульта управления. 
//Ждет прерывания по обнаружению символа конца строки, при обнаружении разбираем полученные данные, проводим их обработку и выдаем в очередь в сторону main_flying_cycle.
//Если есть команда на управление сервоприводами выдает команду на PCA9685. По завершении раз через 3 активируем задачу передачи телеметрии обратно на пульт
void RC_emulation(void * pvParameters) 
{

  data_from_rc_to_main_struct RC_data_emulation;
    RC_data_emulation.raw_throttle = 0;
    RC_data_emulation.received_throttle = 0.0;                                                  //значение газа
    RC_data_emulation.received_pitch = 0;                                                     //значение pitch
    RC_data_emulation.received_roll = 0;                                                      //значение roll
    RC_data_emulation.received_yaw = 0;                                                       //значение yaw
    RC_data_emulation.mode = 0;                                                               //режим (два байта с состояниями тумблеров)
    RC_data_emulation.trim_pitch = 0;                                                         //значение trim по pitch 
    RC_data_emulation.trim_roll = 0;                                                         //значение trim по roll
    RC_data_emulation.engines_start_flag = 1;                                               //флаг запуска двигателей
    RC_data_emulation.lidar_altitude_hold_flag = 0;                                         //флаг удержания высоты по лидару
    RC_data_emulation.baro_altitude_hold_flag = 0;                                          //флаг удержания высоты по барометру
    RC_data_emulation.rssi_level = 0;
 
    uint32_t counter = 0;
    uint8_t command_to_enable_sounder = 0b01000001;
    uint8_t command_to_disable_sounder = 0b00100001;
// - 0b010000xx - установить в 1 выход xx
// - 0b001000xx - установить в 0 выход xx
    #define STEP_MS (50)

  while(1) 
  {
    //printf("%d %f %f\n", RC_data_emulation.engines_start_flag, RC_data_emulation.received_throttle, RC_data_emulation.received_pitch);
    xQueueSend(remote_control_to_main_queue, (void *) &RC_data_emulation, 0);
    vTaskDelay(STEP_MS);
    counter++;

//действие 1 - подаем газ
    if (counter >= 5 * 1000 / STEP_MS) 
    {
      RC_data_emulation.received_throttle = (1200.0f / 4095.0f);
      RC_data_emulation.received_throttle = RC_data_emulation.received_throttle - 3.0f * RC_data_emulation.received_throttle * (RC_data_emulation.received_throttle - 0.9f) * (1 - RC_data_emulation.received_throttle);
    }

// //действие 2
//   if (counter == 10 * 1000 / STEP_MS) {RC_data_emulation.received_pitch = 10;}
  
// //действие 3
//   if (counter == 12 * 1000 / STEP_MS) RC_data_emulation.received_pitch = -20;

// //действие 4  
//   if (counter == 14 * 1000 / STEP_MS) RC_data_emulation.received_pitch = 20;

// //действие 5  
//   if (counter == 16 * 1000 / STEP_MS) RC_data_emulation.received_pitch = -25;
  
//действие 5  
  // if (counter == 100 * 1000 / STEP_MS)
  // {
  //   RC_data_emulation.received_pitch = 0;
  //   RC_data_emulation.engines_start_flag = 0;
  //   RC_data_emulation.received_throttle = 0;
  // } 

  }
} 
