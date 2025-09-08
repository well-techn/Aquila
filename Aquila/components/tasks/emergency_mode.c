//задача, которая активируется при аварийной остановке  
//останавливает выполнение остальных задач, периодически включает  световой и звуковой оповещатель (через расширитель портов) 

//стандартные библиотеки
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/uart.h"

//собственные библиотеки
#include "wt_alldef.h"

extern QueueHandle_t MCP23017_queue;
extern QueueHandle_t gps_to_main_queue;
extern QueueHandle_t INA219_to_main_queue;
extern TaskHandle_t task_handle_blinking_flight_lights;
extern TaskHandle_t task_handle_main_flying_cycle;
extern TaskHandle_t task_handle_INA219_read_and_process_data;
extern TaskHandle_t task_handle_MS5611_read_and_process_data;
extern TaskHandle_t task_handle_lidar_read_and_process_data;

static void remote_control_uart_emergency_reconfig(int baud_rate)
{
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(REMOTE_CONTROL_UART, &uart_config));
    uart_flush(REMOTE_CONTROL_UART);                                                                           //сбрасываем буфер
}


void emergency_mode (void * pvParameters)              
{
  uint8_t command_to_enable_emergency_sounder = 0b01000001;      //на выходе 1 MCP23017
  uint8_t command_to_disable_emergency_sounder = 0b00100001;     //на выходе 1 MCP23017

  uint8_t command_to_enable_conf_mode_for_ebyte = 0b01000010;      //на выходе 2 MCP23017
  uint8_t command_to_disable_conf_mode_for_ebyte = 0b00100010;     //на выходе 2 MCP23017

  uint8_t message_to_reconfigure_ebyte_module[] = {0xC2, 0x02,0x01, 0b11000010};        //установить временно эфирную скорость на 2400, остальное без изменений
  //uint8_t message_to_reconfigure_ebyte_module[] = {0xC0, 0x02,0x01, 0b11000101};        //установить постоянно эфирную скорость 19200 
  
  float emergency_INA219_fresh_data[4];

#ifdef USING_GPS
  data_from_gps_to_main_struct_t emergency_gps_beacon;
#endif

data_from_emergency_beacon_to_radio_t emergency_pack;
emergency_pack.latitude = 999999;
emergency_pack.longtitude = 111111;
  
while(1)
  {
    if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) != 0)
    {
//удаляем а всякий случай неактуальные теперь задачи 
        printf("Enter Emergency mode\n");       
        if (task_handle_main_flying_cycle != NULL) vTaskDelete(task_handle_main_flying_cycle);
        if (task_handle_blinking_flight_lights != NULL) vTaskDelete(task_handle_blinking_flight_lights);
        if (task_handle_MS5611_read_and_process_data != NULL) vTaskDelete(task_handle_MS5611_read_and_process_data);
        if (task_handle_lidar_read_and_process_data != NULL) vTaskDelete(task_handle_lidar_read_and_process_data);
  
//вгоняем модуль Ebyte в режим программирования и перенастраиваем UART на 9600, так как программируется он только на этой скорости        
        remote_control_uart_emergency_reconfig(9600);
        xQueueSend(MCP23017_queue,&command_to_enable_conf_mode_for_ebyte,0);
        vTaskDelay(50/portTICK_PERIOD_MS);
//Отправляем на модуль Ebyte команду переключить эфирную скорость на 9600
        uart_write_bytes(REMOTE_CONTROL_UART, message_to_reconfigure_ebyte_module, 4);
        vTaskDelay(500/portTICK_PERIOD_MS);
        xQueueSend(MCP23017_queue,&command_to_disable_conf_mode_for_ebyte,0);
//возвращаем UART на оригинальную скорость
        remote_control_uart_emergency_reconfig(RC_UART_BAUD_RATE);

        
        while(1) 
            {
#ifdef USING_GPS                
                if (xQueueReceive(gps_to_main_queue, &emergency_gps_beacon, 0)) 
                {
                //printf("Широта %lu Долгота %lu\n", emergency_gps_beacon.latitude_d, emergency_gps_beacon.longtitude_d);
                emergency_pack.latitude = emergency_gps_beacon.latitude_d;
                emergency_pack.longtitude = emergency_gps_beacon.longtitude_d;
                }
#endif
                if (xQueueReceive(INA219_to_main_queue, &emergency_INA219_fresh_data, 0))
                {
                //printf("V: %0.4fV\n",emergency_INA219_fresh_data[0]);
                }


                emergency_pack.voltage_dv = (uint8_t)(emergency_INA219_fresh_data[0]*10); 
                
                gpio_set_level(GREEN_FLIGHT_LIGHTS, 1);
                gpio_set_level(RED_FLIGHT_LIGHTS, 1);
                xQueueSend(MCP23017_queue, &command_to_enable_emergency_sounder, 0);

                vTaskDelay(2000/portTICK_PERIOD_MS);

                gpio_set_level(GREEN_FLIGHT_LIGHTS, 0);
                gpio_set_level(RED_FLIGHT_LIGHTS, 0);
                xQueueSend(MCP23017_queue,&command_to_disable_emergency_sounder,0);

                xTaskNotifyGive(task_handle_INA219_read_and_process_data);
//отправляем emergency пакет в эфир 
                uart_write_bytes(REMOTE_CONTROL_UART, &emergency_pack, sizeof(data_from_emergency_beacon_to_radio_t));

                vTaskDelay(10000/portTICK_PERIOD_MS);

                
            }
        }
  }  
}