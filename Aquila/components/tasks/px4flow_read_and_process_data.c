//Задача считывания и обработки данных от PX4Flow
//считывает по i2c данные и передает в main_flying_cycle

//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "timing.h"
#include "driver/uart.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "px4flow.h"
#include "filters.h"
#include "timing.h"

extern SemaphoreHandle_t semaphore_for_i2c_internal;
extern QueueHandle_t px4flow_to_main_queue;
extern char *TAG_PX4FLOW;

#ifdef USING_PX4FLOW
void px4flow_read_and_process_data(void * pvParameters)
{
  uint32_t height_for_optical_flow_cm = 0;
 
  data_from_px4flow_to_main_struct_t px4flow_data;
  px4flow_i2c_frame_t px4flow_frame;
  px4flow_i2c_integral_frame_t px4flow_int_frame;
  float compensated_flow_x, compensated_flow_y;
  float compensated_pos_x, compensated_pos_y = 0;
 


  while(1) 
  {
    height_for_optical_flow_cm = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (height_for_optical_flow_cm != 0)
    {
      ESP_LOGD(TAG_PX4FLOW,"Отправляем запрос на получение данных");
      xSemaphoreTake(semaphore_for_i2c_internal,portMAX_DELAY); 
      //px4flow_read_frame(&px4flow_frame);
      px4flow_read_integral_frame(&px4flow_int_frame);
      xSemaphoreGive(semaphore_for_i2c_internal);
//формируем поток в метрах      (на сколько переместился между считываниями)
//делим на 10 000 так как in radians*10000 
//height_for_optical_flow в см, по этому делим на 100 для перевода в метры
//меняем местами X и Y и корректируем знак в зависимости от крепления px4flow на борту
//результат в локальных координатах дрона "вперед - вправо"
      px4flow_data.flow_x_meters = ((float)px4flow_int_frame.pixel_flow_y_integral / 10000.0) * height_for_optical_flow_cm / 100.0;
      px4flow_data.flow_y_meters = -((float)px4flow_int_frame.pixel_flow_x_integral / 10000.0) * height_for_optical_flow_cm / 100.0;

      //compensated_flow_x = ((float)(px4flow_int_frame.pixel_flow_y_integral - px4flow_int_frame.gyro_x_rate_integral) / 10000.0) * height_for_optical_flow_cm / 100.0;
      //compensated_flow_y = -((float)(px4flow_int_frame.pixel_flow_x_integral + px4flow_int_frame.gyro_y_rate_integral) / 10000.0) * height_for_optical_flow_cm / 100.0;
//формируем скорость в м/с      
//умножаем на 10^-6 так как integration_timespan в мкс
      px4flow_data.flow_vx = px4flow_data.flow_x_meters/(px4flow_int_frame.integration_timespan * 0.000001);
      px4flow_data.flow_vy = px4flow_data.flow_y_meters/(px4flow_int_frame.integration_timespan * 0.000001);
//формируем положение в м
      px4flow_data.optical_x += px4flow_data.flow_x_meters;
      px4flow_data.optical_y += px4flow_data.flow_y_meters;

      //compensated_pos_x += compensated_flow_x;
      //compensated_pos_y += compensated_flow_y;


//транслируем качество
      px4flow_data.quality = px4flow_int_frame.quality;

      //printf("%d, %d\n",px4flow_int_frame.gyro_x_rate_integral, px4flow_int_frame.gyro_y_rate_integral);
      //printf("%0.2f, %0.2f\n",px4flow_data.flow_x_meters, compensated_flow_x);
      //printf("%0.2f, %0.2f, %d, %ld\n",px4flow_data.optical_x, px4flow_data.optical_y, px4flow_int_frame.quality, height_for_optical_flow);

      xQueueSend(px4flow_to_main_queue, (void *) &px4flow_data, NULL);
    } 
}

}
#endif