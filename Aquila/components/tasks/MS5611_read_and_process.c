//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "math.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "MS5611.h"
#include "filters.h"

extern SemaphoreHandle_t semaphore_for_i2c_external;
extern QueueHandle_t MS5611_to_main_queue;
extern char *TAG_MS5611;

void MS5611_read_and_process_data(void * pvParameters)
{
  uint16_t MS5611_PROM[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  uint32_t MS5611_raw_pressure = 0;
  uint32_t MS5611_raw_temperature = 0;
  int32_t MS5611_dT = 0;
  double MS5611_TEMP = 0;
  double MS5611_OFF = 0;
  double MS5611_SENS = 0;
  double MS5611_P = 0;

  uint8_t first_reads_counter = 0;
  double MS_5611_ground_pressure = 0;;
  float altitude_baro_cm_filter_pool[5] = {0,0,0,0,0};
  float altitude_baro_cm = 0;

  ESP_LOGI(TAG_MS5611,"Считывание PROM MS5611.....");
  if (MS5611_I2C_PROM_read(MS5611_PROM) != ESP_OK) {
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  while(1) 
  {
    if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) != 0)
    {
       xSemaphoreTake (semaphore_for_i2c_external, portMAX_DELAY);
       MS5611_I2C_request_D1();
       xSemaphoreGive (semaphore_for_i2c_external);
       vTaskDelay(20/portTICK_PERIOD_MS);
       xSemaphoreTake (semaphore_for_i2c_external, portMAX_DELAY);
       MS5611_raw_pressure = MS5611_I2C_read();
       //ESP_LOGD(TAG_MS5611,"raw pressure %ld",MS5611_raw_pressure);

       MS5611_I2C_request_D2();
       xSemaphoreGive (semaphore_for_i2c_external);
       vTaskDelay(20/portTICK_PERIOD_MS);
       xSemaphoreTake (semaphore_for_i2c_external, portMAX_DELAY);
       MS5611_raw_temperature = MS5611_I2C_read();
       xSemaphoreGive (semaphore_for_i2c_external);
       ESP_LOGD(TAG_MS5611,"raw temperature %ld",MS5611_raw_temperature);
       
       MS5611_dT = MS5611_raw_temperature - MS5611_PROM[5] * 256; 
       ESP_LOGD(TAG_MS5611,"MS5611_dT %ld", MS5611_dT);            
     
       MS5611_TEMP = (2000 + ((double)MS5611_dT * MS5611_PROM[6]) / 8388608);
       ESP_LOGD(TAG_MS5611,"MS5611_TEMP %f", MS5611_TEMP);  
     
       MS5611_OFF = MS5611_PROM[2] * pow(2,16) + (double)MS5611_PROM[4] * MS5611_dT / pow(2,7);                                
       ESP_LOGD(TAG_MS5611,"MS5611_OFF %f", MS5611_OFF);  
       
       MS5611_SENS = MS5611_PROM[1] * pow(2,15) + (double)MS5611_PROM[3] * MS5611_dT / pow(2,8);
       ESP_LOGD(TAG_MS5611,"MS5611_SENS %f", MS5611_SENS);
     
       if  ((MS5611_TEMP/100.0) < 20.0) 
       {
        double MS5611_T2 = pow(MS5611_dT,2) / pow(2,31);
        double MS5611_OFF2 = 5 * pow(MS5611_TEMP - 2000,2) / 2;
        double MS5611_SENS2 = 5 * pow(MS5611_TEMP - 2000,2) / 4;
     
         if ((MS5611_TEMP/100.0) < -15.0)
         {
           MS5611_OFF2 += 7 * pow(MS5611_TEMP + 1500,2);
           MS5611_SENS2 += 11 * pow(MS5611_TEMP + 1500,2) / 2;
         }
     
         MS5611_TEMP -= MS5611_T2;
         MS5611_OFF -= MS5611_OFF2;
         MS5611_SENS -= MS5611_SENS2;
     
         ESP_LOGD(TAG_MS5611,"MS5611_TEMP_FINAL %f", MS5611_TEMP);
       }
     
       MS5611_P = ((MS5611_raw_pressure * MS5611_SENS)/pow(2,21)- MS5611_OFF)/pow(2,15);
       ESP_LOGD(TAG_MS5611,"pressure %f\n", MS5611_P);
//считаем средний уровень за первые 50 отсчетов
       if (first_reads_counter < 50) 
       {
        MS_5611_ground_pressure += MS5611_P;
        first_reads_counter++;
       }
       else if (first_reads_counter == 50)
            {
              MS_5611_ground_pressure = MS_5611_ground_pressure / 50.0;
              ESP_LOGW(TAG_MS5611,"Давление на уровне точки старта %0.3f",MS_5611_ground_pressure);
              first_reads_counter++;
            } 
            else
            {
              altitude_baro_cm = (MS_5611_ground_pressure - MS5611_P) * 8.33;       //1mBar*100 = 8.3см Относительная высота = разность давлений * 8.3см
              if (altitude_baro_cm < 0) altitude_baro_cm = 0;
              altitude_baro_cm = avg_filter_1d(altitude_baro_cm_filter_pool, altitude_baro_cm, 5);
              xQueueSend(MS5611_to_main_queue, &altitude_baro_cm, NULL); 
            }                         
    } 
  }
}
