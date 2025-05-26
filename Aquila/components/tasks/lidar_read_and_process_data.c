
//Задача обработки данных от лидара TFSMini (Benewake)
//ждет прерывания от обнаружения символа начала строки, считывает данные, обрабатывает и отправляем в main_flying_cycle 
//опционально можем настраивать частоту выдачи данных с TFSmini 

//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "timing.h"
#include "driver/uart.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "TfminiS.h"
#include "filters.h"
#include "timing.h"


extern SemaphoreHandle_t semaphore_for_i2c_internal;
extern QueueHandle_t lidar_to_main_queue;
extern char *TAG_LIDAR;

#define MEDIAN_FILTER_LENGTH (7)
#define AVG_FILTER_LENGTH (10)

#ifdef USING_TFMINIS_I2C
void lidar_read_and_process_data(void * pvParameters)
{
  uint8_t incoming_message_buffer_lidar[NUMBER_OF_BYTES_TO_RECEIVE_FROM_LIDAR];
  uint16_t i = 0;
  uint8_t CRC_sum = 0;
  uint16_t raw_height = 0;
  //uint16_t raw_height_old = 0;
  uint16_t raw_strength = 0;
  struct data_from_lidar_to_main_struct lidar_data;
  //float initial_height = 0;
  float median_filter_pool[MEDIAN_FILTER_LENGTH];
  float avg_filter_pool[AVG_FILTER_LENGTH];
  float altitude_old = 0;
  uint64_t current_time = 0;
  uint64_t last_cycle_time = 0;

  while(1) 
  {
    if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) != 0)
    {
      ESP_LOGD(TAG_LIDAR,"Отправляем запрос на получение данных");
      xSemaphoreTake(semaphore_for_i2c_internal,portMAX_DELAY); 
      tfs_request_data();
      //xSemaphoreGive(semaphore_for_i2c_internal);                                 //тут должна быть задержка 1ms но вроде работает и без нее
      //vTaskDelay(1/portTICK_PERIOD_MS);     //esp_rom_delay_us(uint32_t us)       //It is recommended to wait for 1ms and then read the result after sending commands.
      //ESP_LOGD(TAG_LIDAR,"Считываем результат");
      //xSemaphoreTake(semaphore_for_i2c_internal,portMAX_DELAY);  
      tfs_read_result(incoming_message_buffer_lidar);
      xSemaphoreGive(semaphore_for_i2c_internal);
      CRC_sum = 0;
      for (i=0;i<8;i++) CRC_sum += incoming_message_buffer_lidar[i];
          
      if ((incoming_message_buffer_lidar[0] == 0x59) && (incoming_message_buffer_lidar[1] == 0x59) && (incoming_message_buffer_lidar[8] == CRC_sum))
      {
        raw_height = (incoming_message_buffer_lidar[3] << 8) + incoming_message_buffer_lidar[2];
        raw_strength = (incoming_message_buffer_lidar[5] << 8) + incoming_message_buffer_lidar[4];
//убираем длинные скачки с которыми не справляется медианный фильтр???? цифра в см
        //if (((raw_height - lidar_data.altitude) > 30 ) || ((raw_height - lidar_data.altitude) < -30)) raw_height = lidar_data.altitude;
        lidar_data.altitude = median_filter(median_filter_pool, raw_height, MEDIAN_FILTER_LENGTH);
        current_time = get_time();
        lidar_data.altitude = avg_filter_1d(avg_filter_pool, lidar_data.altitude, AVG_FILTER_LENGTH);
//вычисляем вертикальную скорость        
        lidar_data.vertical_velocity = ((lidar_data.altitude - altitude_old) / (current_time - last_cycle_time)) * 1000000;
        last_cycle_time = current_time;
        altitude_old = lidar_data.altitude;  
        
        //printf("%0.3f\n",  lidar_data.altitude);

        if ((raw_strength < 100) || (raw_height > 65532)) lidar_data.valid = 0;
        else lidar_data.valid = 1;
        lidar_data.strength = raw_strength;

        xQueueSend(lidar_to_main_queue, (void *) &lidar_data, NULL);
         
      }  
    }
      else if (incoming_message_buffer_lidar[8] != CRC_sum) ESP_LOGW(TAG_LIDAR,"Ошибка CRC, расчетный CRC = %d, принятый %d", CRC_sum, incoming_message_buffer_lidar[8]);
           else ESP_LOGW(TAG_LIDAR,"Ошибка заголовка (%02x, %02x)", incoming_message_buffer_lidar[0], incoming_message_buffer_lidar[1]); 
  }   
}
#endif

#ifdef USING_TFMINIS_UART
void lidar_read_and_process_data(void * pvParameters)
{
  uint8_t incoming_message_buffer_lidar[NUMBER_OF_BYTES_TO_RECEIVE_FROM_LIDAR];
  uint16_t i = 0;
  uint16_t pos = 0;
  uint8_t sum = 0;
  uint16_t raw_height = 0;
  uint16_t raw_height_old = 0;
  uint16_t raw_strength = 0;
  struct data_from_lidar_to_main_struct lidar_data;
  float height_old = 0;
  float height_filter_pool[5] = {0};
  float vertical_velocity_filter_pool[3] = {0};
  uint8_t set_Hz_command[] = {0x5A, 0x06, 0x03, LIDAR_RATE_HZ, 0x00, 0x00};   //5A 06 03 *LL HH* SU format (1-1000Hz)
  uint8_t save[] = {0x5A, 0x04, 0x11, 0x6F};

  uart_event_t lidar_uart_event;
  
  ESP_LOGI(TAG_LIDAR,"Настраиваем UART для лидара.....");
  int intr_alloc_flags = 0;
  uart_config_t lidar_uart_config = {
      .baud_rate = LIDAR_UART_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  ESP_ERROR_CHECK(uart_driver_install(LIDAR_UART, LIDAR_UART_BUF_SIZE, 0, LIDAR_UART_PATTERN_DETECTION_QUEUE_SIZE, &lidar_queue_for_events, intr_alloc_flags)); 
  ESP_ERROR_CHECK(uart_param_config(LIDAR_UART, &lidar_uart_config));
  ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART, LIDAR_UART_TX_PIN, LIDAR_UART_RX_PIN, LIDAR_UART_RTS_PIN, LIDAR_UART_CTS_PIN));

  ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(LIDAR_UART, 0x59, 2, 3, 0, 0));                 //creating pattern detection
  ESP_ERROR_CHECK(uart_pattern_queue_reset(LIDAR_UART, LIDAR_UART_PATTERN_DETECTION_QUEUE_SIZE));          //allocating queue  
  uart_flush(LIDAR_UART);        
  ESP_LOGI(TAG_LIDAR,"UART для лидара настроен");
  
  ESP_LOGI(TAG_LIDAR,"Настраиваем частоту обновления лидара на %dГц",LIDAR_RATE_HZ);
  set_Hz_command[5] = set_Hz_command[0] + set_Hz_command[2]  + set_Hz_command[3]  + set_Hz_command[4];  //калькулируем контрольную сумму для команды установки частоты обновления лидара
  uart_write_bytes(LIDAR_UART, set_Hz_command, 6);                                                      //записываем требуемую частоту
  uart_write_bytes(LIDAR_UART, save, 4);                                                                //сохраняем настройку
  ESP_LOGI(TAG_LIDAR,"Лидар настроен на %dГц",LIDAR_RATE_HZ);
       
  while(1) {
  if(xQueueReceive(lidar_queue_for_events, (void * )&lidar_uart_event, (TickType_t)portMAX_DELAY))
  {
    switch (lidar_uart_event.type) {
        case UART_PATTERN_DET:
                pos = uart_pattern_pop_pos(LIDAR_UART);
                ESP_LOGD(TAG_LIDAR, "[UART PATTERN DETECTED] pos: %d", pos);
                    int read_len = uart_read_bytes(LIDAR_UART, incoming_message_buffer_lidar, pos+9, portMAX_DELAY);
                    ESP_LOGD(TAG_LIDAR, "Received in total %d bytes", read_len);
                    xQueueReset(lidar_queue_for_events);
                    uart_flush_input(LIDAR_UART); 
                    //for (i=0; i<read_len; i++) printf ("%02x", incoming_message_buffer_lidar[i]);
                    //printf("\n");
                    for (i=0;i<8;i++) sum+= incoming_message_buffer_lidar[i];
                    
                    if ((incoming_message_buffer_lidar[0] == 0x59) && (incoming_message_buffer_lidar[1] == 0x59) && (incoming_message_buffer_lidar[8] == sum))
                    {
                      raw_height = (incoming_message_buffer_lidar[3] << 8) + incoming_message_buffer_lidar[2];
                      raw_strength = (incoming_message_buffer_lidar[5] << 8) + incoming_message_buffer_lidar[4];
                  
                      if (((raw_height - raw_height_old) < 2) && ((raw_height  - raw_height_old) > -2)) raw_height = raw_height_old;                  //так как типичный джиттер сигнала = 2см
                      lidar_data.altitude = avg_filter_1d(height_filter_pool, (float)raw_height, sizeof(height_filter_pool) / sizeof(float));         //фильтруем скользящим средним

                      lidar_data.vertical_velocity = (lidar_data.altitude - height_old) / (1.0 / (float)LIDAR_RATE_HZ);       //на основе фильтрованной высоты вычислем скорость в см/с

                      //lidar_data.vertical_velocity = avg_filter_1d(vertical_velocity_filter_pool, lidar_data.vertical_velocity, sizeof(vertical_velocity_filter_pool) / sizeof(float));   //фильтруем скользящим средним
                      
                      height_old = lidar_data.altitude;

                      //printf("%0.3f, %0.3f\n",lidar_data.altitude,lidar_data.vertical_velocity);

                      if ((raw_strength < 100) || (raw_height > 65532)) lidar_data.valid = 0;
                      else lidar_data.valid = 1;

                      xQueueSend(lidar_to_main_queue, (void *) &lidar_data, NULL);
                      height_old = lidar_data.altitude;
                    }
                uart_flush(LIDAR_UART);
                xQueueReset(lidar_queue_for_events);
                sum = 0;
                break;
        case UART_FIFO_OVF:
              ESP_LOGW(TAG_LIDAR, "hw fifo overflow");
              uart_flush_input(LIDAR_UART);
              xQueueReset(lidar_queue_for_events);
              break;

        case UART_BUFFER_FULL:
          ESP_LOGW(TAG_LIDAR, "ring buffer full");
          uart_flush_input(LIDAR_UART);
          xQueueReset(lidar_queue_for_events);
          break;

      case UART_DATA: break;
       
      case UART_BREAK:
          ESP_LOGW(TAG_LIDAR, "uart rx break");
          break;
        
        case UART_PARITY_ERR:
          ESP_LOGW(TAG_LIDAR, "uart parity error");
          break;
        
        case UART_FRAME_ERR:
          ESP_LOGW(TAG_LIDAR, "uart frame error");
          break;
                    
        default:
          ESP_LOGW(TAG_LIDAR, "unknown uart event type: %d", lidar_uart_event.type);
          uart_flush(LIDAR_UART);
          xQueueReset(lidar_queue_for_events);
            break;
    }
  }
}
}
#endif