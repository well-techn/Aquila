//Задача отправки телеметрии на управления. 
//Активируется из задачи получения данных от пульта, забирает из очереди, в которую шлет main_flying_cycle, данные и отправляем их в UART.

//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "CRCs.h"

extern  char *TAG_RC;
extern QueueHandle_t main_to_rc_queue; 


void send_data_to_RC(void * pvParameters)
{
  data_from_main_to_rc_struct data_to_send_to_rc;
  uint8_t outcoming_message_buffer_remote[NUMBER_OF_BYTES_TO_SEND_TO_RC];
  uint8_t LED_status = 0;

  while(1) 
  {
    if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) != 0)
    {
      if (xQueueReceive(main_to_rc_queue, &data_to_send_to_rc, (TickType_t)portMAX_DELAY)) 
      {
        
        outcoming_message_buffer_remote[0] = RC_MESSAGE_HEADER;
        outcoming_message_buffer_remote[1] = 0; //reserved
        outcoming_message_buffer_remote[2] = 0; //reserved
        outcoming_message_buffer_remote[3] = 0; //reserved
        outcoming_message_buffer_remote[4] = ((int8_t)(data_to_send_to_rc.pitch) & 0xFF00) >> 8;                          //angles in format MSB + LSB    pitch
        outcoming_message_buffer_remote[5] = ((int8_t)(data_to_send_to_rc.pitch) & 0x00FF);
        outcoming_message_buffer_remote[6] = (uint8_t)(((uint16_t)(data_to_send_to_rc.yaw) & 0xFF00) >> 8);                                   //yaw
        outcoming_message_buffer_remote[7] = (uint8_t)((uint16_t)(data_to_send_to_rc.yaw) & 0x00FF);
        outcoming_message_buffer_remote[8] = ((int8_t)(data_to_send_to_rc.roll) & 0xFF00) >> 8;
        outcoming_message_buffer_remote[9] = ((int8_t)(data_to_send_to_rc.roll) & 0x00FF);
        outcoming_message_buffer_remote[10] = (data_to_send_to_rc.power_voltage_value & 0xFF00) >> 8;                  //ADC_power_value
        outcoming_message_buffer_remote[11] = data_to_send_to_rc.power_voltage_value & 0x00FF;
        outcoming_message_buffer_remote[12] = (data_to_send_to_rc.altitude & 0xFF00) >> 8; 
        outcoming_message_buffer_remote[13] = data_to_send_to_rc.altitude & 0x00FF; 
        outcoming_message_buffer_remote[14] = dallas_crc8(outcoming_message_buffer_remote, (NUMBER_OF_BYTES_TO_SEND_TO_RC - 1));
/*        
        outcoming_message_buffer_remote[0] = RC_MESSAGE_HEADER;
        outcoming_message_buffer_remote[1] = (uint8_t)data_to_send_to_rc.power_voltage_value;                 //сантивольты
        outcoming_message_buffer_remote[2] = (uint8_t)data_to_send_to_rc.altitude;                            //метры
        outcoming_message_buffer_remote[3] = dallas_crc8(outcoming_message_buffer_remote, (NUMBER_OF_BYTES_TO_SEND_TO_RC - 1));
        printf("%x, %x, %x, %x\n",outcoming_message_buffer_remote[0],outcoming_message_buffer_remote[1],outcoming_message_buffer_remote[2],outcoming_message_buffer_remote[3]);
*/
        uart_write_bytes(REMOTE_CONTROL_UART, outcoming_message_buffer_remote, NUMBER_OF_BYTES_TO_SEND_TO_RC);
                if (LED_status) {gpio_set_level(LED_BLUE, 0); LED_status=0;}
                      else {gpio_set_level(LED_BLUE, 1);LED_status=1;}
      }                       
    } 
  }
}