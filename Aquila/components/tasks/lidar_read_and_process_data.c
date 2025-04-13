
//Задача обработки данных от лидара TFSMini (Benewake)
//ждет прерывания от обнаружения символа начала строки, считывает данные, обрабатывает и отправляем в main_flying_cycle 
//опционально можем настраивать частоту выдачи данных с TFSmini 

//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "TfminiS.h"
#include "filters.h"


extern SemaphoreHandle_t semaphore_for_i2c_internal;
extern QueueHandle_t lidar_to_main_queue;
extern char *TAG_LIDAR;

#define MEDIAN_FILTER_LENGTH (7)
#define AVG_FILTER_LENGTH (10)

void lidar_read_and_process_data(void * pvParameters)
{
  uint8_t incoming_message_buffer_lidar[NUMBER_OF_BYTES_TO_RECEIVE_FROM_LIDAR];
  uint16_t i = 0;
  uint8_t CRC_sum = 0;
  uint16_t raw_height = 0;
  uint16_t raw_height_old = 0;
  uint16_t raw_strength = 0;
  struct data_from_lidar_to_main_struct lidar_data;
  float initial_height = 0;
  float median_filter_pool[MEDIAN_FILTER_LENGTH];
  float avg_filter_pool[AVG_FILTER_LENGTH];

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
        //printf("%0.3f, ", lidar_data.altitude);
        lidar_data.altitude = avg_filter_1d(avg_filter_pool, lidar_data.altitude, AVG_FILTER_LENGTH);
        
        //printf("%0.3f\n", lidar_data.altitude);

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
