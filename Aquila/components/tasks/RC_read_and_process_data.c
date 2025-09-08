//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "math.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "CRCs.h"

extern  char *TAG_RC;
extern QueueHandle_t remote_control_queue_for_events;  //очередь для передачи событий от UART пульта управления (pattern detection) в задачу обработки данных пульта
extern QueueHandle_t remote_control_to_main_queue; //очередь для передачи обработанных данных от пульта в main_flying_cycle
extern QueueHandle_t PCA9685_queue;
extern TaskHandle_t task_handle_send_data_to_RC;

static void remote_control_uart_config(void)
{
    int intr_alloc_flags = 0;
    uart_config_t uart_config = {
        .baud_rate = RC_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    //uart_set_rx_timeout(REMOTE_CONTROL_UART, 100);
    //https://esp32.com/viewtopic.php?f=13&t=35116

    ESP_ERROR_CHECK(uart_driver_install(REMOTE_CONTROL_UART, RC_RX_UART_BUFF_SIZE, RC_TX_UART_BUFF_SIZE, RC_UART_PATTERN_DETECTION_QUEUE_SIZE, &remote_control_queue_for_events, intr_alloc_flags)); 
    ESP_ERROR_CHECK(uart_param_config(REMOTE_CONTROL_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(REMOTE_CONTROL_UART, RC_UART_TX_PIN, RC_UART_RX_PIN, RC_UART_RTS_PIN, RC_UART_CTS_PIN));
#ifdef NO_RSSI
    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(REMOTE_CONTROL_UART, 0xFF, 1, 5, 0, 0));                 //активируем режим обнаружения пэттерна, в частности байта 0xFF, который является концом строки 
#endif
    ESP_ERROR_CHECK(uart_pattern_queue_reset(REMOTE_CONTROL_UART, RC_UART_PATTERN_DETECTION_QUEUE_SIZE));      //сбрасываем очередь
#ifdef NO_RSSI
    uart_disable_intr_mask(REMOTE_CONTROL_UART, (0x1 << 8));                                                   //UART_INTR_RXFIFO_TOUT
#endif
    uart_flush(REMOTE_CONTROL_UART);                                                                           //сбрасываем буфер
}



//Задача получения и обработки данных от пульта управления. 
//Ждет прерывания по обнаружению символа конца строки, при обнаружении разбираем полученные данные, проводим их обработку и выдаем в очередь в сторону main_flying_cycle.
//Если есть команда на управление сервоприводами выдает команду на PCA9685. По завершении раз через 3 активируем задачу передачи телеметрии обратно на пульт
void RC_read_and_process_data(void * pvParameters) 
{
  uint16_t i = 0;
  int16_t pos = 0;
  uint8_t remote_packets_counter = 0;
  uart_event_t remote_control_uart_event;
  uint8_t incoming_message_buffer_remote[NUMBER_OF_BYTES_TO_RECEIVE_FROM_RC * 2];
  uint16_t received_throttle = 0;
  uint16_t received_pitch = 2000;                       //среднее положение
  uint16_t received_roll = 2000;
  uint16_t received_yaw = 2000;
  short trim_roll = -1;
  short trim_pitch = 2;
  data_from_rc_to_main_struct remote_control_data;
  
  float rc_throttle_old = 0;
  float rc_pitch_old = 0;
  float rc_roll_old = 0;
  float rc_yaw_old = 0;
  float raw_rc_throttle_old = 0;

  uint8_t lidar_alt_hold_set_allowed = 0;
  uint8_t baro_alt_hold_set_allowed = 0;

  uint16_t command_for_PCA9685;
  uint16_t mode_old = 0;
  uint8_t LED_status = 0;
  uint8_t start_flag_old = 0;

  remote_control_data.lidar_altitude_hold_flag = 0;
  remote_control_data.baro_altitude_hold_flag = 0;
  
  ESP_LOGI(TAG_RC,"Настраиваем UART для пульта управления.....");
  remote_control_uart_config();
  ESP_LOGI(TAG_RC,"UART для пульта управления настроен");

  while(1) 
  {
    if(xQueueReceive(remote_control_queue_for_events, (void * )&remote_control_uart_event, (TickType_t)portMAX_DELAY)) 
    {     
      switch (remote_control_uart_event.type)                   //проверяем тип события, полученного от UART 
      {
//если не включен пакет RSSI на приемнике - работаем по обнаружению символа конца строки
#ifdef NO_RSSI  
        case UART_PATTERN_DET:                                  //если это то что надо - 
          pos = uart_pattern_pop_pos(REMOTE_CONTROL_UART);      //запрашиваем на какой позиции в буфере обнаружен символ конца строки
          ESP_LOGD(TAG_RC, "[UART PATTERN DETECTED] pos: %d", pos);
          if (pos != (NUMBER_OF_BYTES_TO_RECEIVE_FROM_RC-1))    //если это не то место, где он должен быть - значит пакет поступил не полностью из-за какого-то сбоя, 
          {
            uart_flush_input(REMOTE_CONTROL_UART);              //очищаем входящий буфер и очередь  
            xQueueReset(remote_control_queue_for_events);
            ESP_LOGW(TAG_RC, "incorrect pos, %d", pos);  
          }
          else                                                 //если все ок     
          {                                                                                     
            int read_len = uart_read_bytes(REMOTE_CONTROL_UART, incoming_message_buffer_remote, pos+1, 1);      //считываем данные из буфера в локальную переменную
//если получаем байт RSSI - работам просто по UART_data
#else
        case UART_DATA:                                 
                  
          ESP_ERROR_CHECK(uart_get_buffered_data_len(REMOTE_CONTROL_UART, (size_t*)&pos));  //запрашиваем сколько байт получено
          ESP_LOGD(TAG_RC, "Получено %d байт по data", pos);

          if (pos != (NUMBER_OF_BYTES_TO_RECEIVE_FROM_RC + 1))                              //если не столько сколько должно быть
          {
            uart_flush_input(REMOTE_CONTROL_UART);              //очищаем входящий буфер и очередь  
            xQueueReset(remote_control_queue_for_events);
            ESP_LOGW(TAG_RC, "Некорректное кол-во байт, %d", pos);  
          }                    
          else                                                 //если все ок     
            {                                                                                
              int read_len = uart_read_bytes(REMOTE_CONTROL_UART, incoming_message_buffer_remote, pos, 0);  //считываем в локальный буфер    

#endif            
            ESP_LOGD(TAG_RC, "Received in total %d bytes", read_len);

            //проверяем покет на целостность по заголовку и контрольной сумме
            if ((incoming_message_buffer_remote[0] == RC_MESSAGE_HEADER) 
            && (incoming_message_buffer_remote[NUMBER_OF_BYTES_TO_RECEIVE_FROM_RC - 2] == dallas_crc8(incoming_message_buffer_remote, NUMBER_OF_BYTES_TO_RECEIVE_FROM_RC-2)))
            {
              ESP_LOGD(TAG_RC, "CRC passed");
//очищаем буфер
              uart_flush(REMOTE_CONTROL_UART);
//моргаем одним из светодиодов на плате
              if (LED_status) {gpio_set_level(LED_GREEN, 0); LED_status=0;}
              else {gpio_set_level(LED_GREEN, 1);LED_status=1;}
//собираем полученные данные в переменные
           
              received_throttle = (incoming_message_buffer_remote[1] << 8) | incoming_message_buffer_remote[2];
              received_roll = ((incoming_message_buffer_remote[3] << 8) | incoming_message_buffer_remote[4]);               
              received_pitch = ((incoming_message_buffer_remote[5] << 8) | incoming_message_buffer_remote[6]);                
              received_yaw = ((incoming_message_buffer_remote[7] << 8) | incoming_message_buffer_remote[8]);
              remote_control_data.mode = ~incoming_message_buffer_remote[10];
//начинаем приводить их к желаемому виду
              remote_control_data.raw_throttle = received_throttle;              
//remote_control_data.received_throttle = 7836.0f + 48.98f * sqrt((double)received_throttle);
              if(remote_control_data.received_throttle <=0) remote_control_data.received_throttle = 1;//так как логарифм
              remote_control_data.received_throttle = 1760 + 1127 * log((float)received_throttle);


              if ((received_pitch > 360)&&( received_pitch < 1800)) remote_control_data.received_pitch = 0.02083333f*(float)received_pitch - 37.5f;
                else if ((received_pitch > 2245) && (received_pitch < 3741)) remote_control_data.received_pitch  = 0.02005348*(float)received_pitch - 45.020053f;
                else if (received_pitch >= 3741) remote_control_data.received_pitch  = 30.0;  //градусы наклона
                else if (received_pitch <= 360) remote_control_data.received_pitch  = -30.0;
                else remote_control_data.received_pitch = 0;
//reversed signs             
              if ((received_roll > 360)&&( received_roll < 1800)) remote_control_data.received_roll = -0.02083333f*(float)received_roll + 37.5f;              
                else if ((received_roll > 2245) && (received_roll < 3741)) remote_control_data.received_roll = -0.02005348*(float)received_roll + 45.020053f; 
                else if (received_roll >= 3741) remote_control_data.received_roll = -30.0;  //градусы наклона
                else if (received_roll <= 360) remote_control_data.received_roll = 30.0;
                else remote_control_data.received_roll = 0;

              if ((received_yaw > 200)&&( received_yaw < 1848)) remote_control_data.received_yaw = -0.0910194f*(float)received_yaw + 168.203885;
                else if ((received_yaw < 3896)&&( received_yaw > 2248)) remote_control_data.received_yaw = -0.0910194f*(float)received_yaw + 204.61165;
                else if (received_yaw >= 3896) remote_control_data.received_yaw = -150.0;     //градусы в секунду
                else if (received_yaw <= 200) remote_control_data.received_yaw = 150.0;
                else remote_control_data.received_yaw = 0;

//слегка подфильтровываем значения от джойстиков        
              remote_control_data.raw_throttle = remote_control_data.raw_throttle * RC_FILTER_COEFF + raw_rc_throttle_old * (1 - RC_FILTER_COEFF);
              raw_rc_throttle_old = remote_control_data.raw_throttle;

              remote_control_data.received_throttle = remote_control_data.received_throttle * RC_FILTER_COEFF + rc_throttle_old * (1 - RC_FILTER_COEFF);
              rc_throttle_old = remote_control_data.received_throttle;

              remote_control_data.received_pitch = remote_control_data.received_pitch * RC_FILTER_COEFF + rc_pitch_old * (1 - RC_FILTER_COEFF);
              rc_pitch_old = remote_control_data.received_pitch;

              remote_control_data.received_roll = remote_control_data.received_roll * RC_FILTER_COEFF + rc_roll_old * (1 - RC_FILTER_COEFF);
              rc_roll_old = remote_control_data.received_roll;

              remote_control_data.received_yaw = remote_control_data.received_yaw * RC_FILTER_COEFF + rc_yaw_old * (1 - RC_FILTER_COEFF);
              rc_yaw_old = remote_control_data.received_yaw;

//формируем значения trim (не используется далее, но пусть будет)
              if (incoming_message_buffer_remote[9] & 0b00001000) {                            //if roll negative values
              trim_roll = (((~(incoming_message_buffer_remote[9] & 0b00001111)) & 0b00001111) + 1) * -1;}
              else trim_roll = incoming_message_buffer_remote[9] & 0b00001111;

              if (incoming_message_buffer_remote[9] & 0b10000000) {                            //if pitch negative values
              trim_pitch = (((~((incoming_message_buffer_remote[9] >> 4) & 0b00001111)) & 0b00001111)  + 1) * -1;}
              else trim_pitch = (incoming_message_buffer_remote[9] >> 4) & 0b00001111; 

//если ручка газа в нижнем положении и включен тумблер "старт двигателей" установить флаг engines_start_flag        
              if ((remote_control_data.received_throttle < 8400) && (remote_control_data.mode & 0x0001)) remote_control_data.engines_start_flag = 1;
//если тумблер "старт двигателей" выключен - обнуляем бит engines_start_flag (при любом положении ручки газа)                
              if (!(remote_control_data.mode & 0x01)) 
              { 
                remote_control_data.engines_start_flag = 0; 
                remote_control_data.lidar_altitude_hold_flag = 0; 
                remote_control_data.baro_altitude_hold_flag = 0;
                lidar_alt_hold_set_allowed = 0;
                baro_alt_hold_set_allowed = 0;
              }
#ifdef USING_TFMINIS_I2C              
//переменные ***_alt_hold_set_allowed нужны для того, чтобы избежать ситуации, когда мы полетали в ударжании, выключили engines_start_bit оставив включенными тумблера режимов удержания выооты и потом опять включили engines_start - в этом случае режим удержания не включится, пока не выключим тумблер удержания и опять не включим
//если  тумблер "удержание высоты по лидару" включен при запущенных двигателях - устанавливаем флаг lidar_altitude_hold_flag 
              if ((remote_control_data.mode & 0x02) && (lidar_alt_hold_set_allowed == 1) && (remote_control_data.engines_start_flag)) remote_control_data.lidar_altitude_hold_flag = 1;
//в противном случае обнуляем этот флаг              
              if (!(remote_control_data.mode & 0x02)) {remote_control_data.lidar_altitude_hold_flag = 0; lidar_alt_hold_set_allowed = 1;}
#endif
#ifdef USING_MS5611  
//если  тумблер "удержание высоты по барометру" включен при запущенных двигателях - устанавливаем флаг lidar_altitude_hold_flag 
              if ((remote_control_data.mode & 0x04) && (baro_alt_hold_set_allowed == 1) && (remote_control_data.engines_start_flag)) remote_control_data.baro_altitude_hold_flag = 1;
//в противном случае обнуляем этот флаг              
              if (!(remote_control_data.mode & 0x04)) {remote_control_data.baro_altitude_hold_flag = 0; baro_alt_hold_set_allowed = 1;}
#endif
//при изменении положения тумблера 4 отправить в очередь задачи управления PCA9685 команду на изменение сигнала
              if ((remote_control_data.mode & 0x08) ^ (mode_old & 0x08))  {
                if (remote_control_data.mode & 0x08) command_for_PCA9685 = 0x0103;
                else command_for_PCA9685 = 0x011A;
              xQueueSend(PCA9685_queue, &command_for_PCA9685, NULL);
              }
//при изменении состояния флага engine_start отправить в очередь задачи управления PCA9685 команду на изменение сигнала
              if (remote_control_data.engines_start_flag ^ start_flag_old)  {
                if (remote_control_data.engines_start_flag) command_for_PCA9685 = 0x015A;
                else command_for_PCA9685 = 0x0100;
              xQueueSend(PCA9685_queue, &command_for_PCA9685, NULL);
              }
//копируем в структуру байт с уровнем RSSI 
              remote_control_data.rssi_level = (int8_t)incoming_message_buffer_remote[13];
//отправляем сформированную структуру с обработанными данными от пульта в main_flying_cycle                
              xQueueSend(remote_control_to_main_queue, (void *) &remote_control_data, NULL);
//обнуляем на всякий случай буфер
              for (i=0;i<NUMBER_OF_BYTES_TO_RECEIVE_FROM_RC * 2;i++) incoming_message_buffer_remote[i] = 0;           
              
              mode_old = remote_control_data.mode;
              start_flag_old = remote_control_data.engines_start_flag;
//каждый 3й раз пробуждаем задачу отправки телеметрии на пульт
/*
              remote_packets_counter++;
              if (remote_packets_counter == 3)
              {
                remote_packets_counter = 0;
                xTaskNotifyGive(task_handle_send_data_to_RC);        
              }
*/  
            }
            else 
            { 
              ESP_LOGW(TAG_RC, "CRC failed");
              //for (j=0;j<13;j++) printf ("%02x ",incoming_message_buffer_remote[j]);       
              uart_flush(REMOTE_CONTROL_UART);
              xQueueReset(remote_control_queue_for_events);
            }  
          }
          break;

        case UART_FIFO_OVF:
          ESP_LOGW(TAG_RC, "hw fifo overflow");
          uart_flush_input(REMOTE_CONTROL_UART);
          xQueueReset(remote_control_queue_for_events);
          break;

        case UART_BUFFER_FULL:
          ESP_LOGW(TAG_RC, "ring buffer full");
          uart_flush_input(REMOTE_CONTROL_UART);
          xQueueReset(remote_control_queue_for_events);
          break;
#ifdef NO_RSSI
        case UART_DATA: 
          ESP_LOGW(TAG_RC, "data");
        break;
#else
        case UART_PATTERN_DET: 
        ESP_LOGW(TAG_RC, "pattern");
        break;
#endif        

case UART_BREAK:
          ESP_LOGW(TAG_RC, "uart rx break");
          uart_flush(REMOTE_CONTROL_UART);
          break;
        
        case UART_PARITY_ERR:
          ESP_LOGW(TAG_RC, "uart parity error");
          break;
        
        case UART_FRAME_ERR:
          ESP_LOGW(TAG_RC, "uart frame error");
          break;

        default:
          ESP_LOGW(TAG_RC, "unknown uart event type: %d", remote_control_uart_event.type);
          uart_flush(REMOTE_CONTROL_UART);
          xQueueReset(remote_control_queue_for_events);
          break;
      }
    }
  }
} 
