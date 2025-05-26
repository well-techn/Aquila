//задача периодического считывания данных с магнетометра IST8310. После создения находится с заблокированном состоянии. Разблокируется из main_flying_cycle.
//Активирует режим однократного измерения, считываем результат после задержки и выдает результат в очередь в сторону main_flying_cycle

//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "inttypes.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "IST8310.h" 

extern SemaphoreHandle_t semaphore_for_i2c_external;
extern QueueHandle_t magnetometer_queue;
extern char *TAG_IST8310;
extern char *TAG_NVS;

void mag_read_and_process_data (void * pvParameters)
{
  uint8_t i = 0;
  float cross_axis[3][3] =  {{0,  0,  0},    
                             {0,  0,  0},
                             {0,  0,  0}};
  uint8_t mag_raw_values[6] = {0,0,0,0,0,0}; 
  int16_t magn_data[3]= {0,0,0};
  float magn_data_axis_corrected[3]= {0,0,0};
  double mag_hard_bias[3] = {0,0,0};   //calibration values with Magneto 1.2
  
  double mag_A_inv[3][3];   //calibration values with Magneto 1.2

  float magn_wo_hb[3] = {0,0,0};
  float magn_data_calibrated[3] = {0,0,0};

  uint64_t temp;
  double* p_double;

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      // если нет места - пробуем стереть и переинициазировать, при этом сотрутся все переменные
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
  }
  ESP_ERROR_CHECK( err );

  ESP_LOGI(TAG_NVS,"Открываем NVS... ");
  nvs_handle_t NVS_handle;
  err = nvs_open("storage", NVS_READWRITE, &NVS_handle);
  if (err != ESP_OK) {
      ESP_LOGE(TAG_NVS,"Ошибка (%s) открытия NVS!\n", esp_err_to_name(err));
  } else {
        ESP_LOGI(TAG_NVS,"NVS открыт");

  // Начинаем считывание сохраненных переменных
  ESP_LOGI(TAG_NVS,"Считываем данные калибровки магнетометра из NVS ... ");

  err = nvs_get_u64(NVS_handle, "mag_h_bias[0]", &temp); 
  p_double = (double*) &temp;
  mag_hard_bias[0] = *p_double;

  err = nvs_get_u64(NVS_handle, "mag_h_bias[1]", &temp); 
  p_double = (double*) &temp;
  mag_hard_bias[1] = *p_double;

  err = nvs_get_u64(NVS_handle, "mag_h_bias[2]", &temp); 
  p_double = (double*) &temp;
  mag_hard_bias[2] = *p_double;


  err = nvs_get_u64(NVS_handle, "mag_A_i[00]", &temp); 
  p_double = (double*) &temp;
  mag_A_inv[0][0] = *p_double;

  err = nvs_get_u64(NVS_handle, "mag_A_i[01]", &temp); 
  p_double = (double*) &temp;
  mag_A_inv[0][1] = *p_double;

  err = nvs_get_u64(NVS_handle, "mag_A_i[02]", &temp); 
  p_double = (double*) &temp;
  mag_A_inv[0][2] = *p_double;

  err = nvs_get_u64(NVS_handle, "mag_A_i[10]", &temp); 
  p_double = (double*) &temp;
  mag_A_inv[1][0] = *p_double;

  err = nvs_get_u64(NVS_handle, "mag_A_i[11]", &temp); 
  p_double = (double*) &temp;
  mag_A_inv[1][1] = *p_double;

  err = nvs_get_u64(NVS_handle, "mag_A_i[12]", &temp); 
  p_double = (double*) &temp;
  mag_A_inv[1][2] = *p_double;

  err = nvs_get_u64(NVS_handle, "mag_A_i[20]", &temp); 
  p_double = (double*) &temp;
  mag_A_inv[2][0] = *p_double;

  err = nvs_get_u64(NVS_handle, "mag_A_i[21]", &temp); 
  p_double = (double*) &temp;
  mag_A_inv[2][1] = *p_double;

  err = nvs_get_u64(NVS_handle, "mag_A_i[22]", &temp); 
  p_double = (double*) &temp;
  mag_A_inv[2][2] = *p_double;
/*
  for (uint8_t i = 0; i<3; i++) printf("%0.3f ", mag_hard_bias[i]);
  printf("\n");
    for (uint8_t i = 0; i<3; i++) 
    {
      for (uint8_t j = 0; j<3; j++)  printf("%0.3f ", mag_A_inv[i][j]);
      printf("\n");
    }
*/
  }

  IST8310_generate_cross_axis_matrix(cross_axis);





  while(1) {
    
    if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) != 0)
    {
      xSemaphoreTake (semaphore_for_i2c_external,portMAX_DELAY);
      IST8310_request_data();
      xSemaphoreGive(semaphore_for_i2c_external);
      vTaskDelay(5/portTICK_PERIOD_MS);   //delay of 5ms as per datasheet min sampling is 200Hz 5ms
      xSemaphoreTake (semaphore_for_i2c_external,portMAX_DELAY);
      IST8310_read_data(mag_raw_values);
      xSemaphoreGive(semaphore_for_i2c_external);
  
      magn_data[0] = mag_raw_values[1] << 8 | mag_raw_values[0]; //X
      magn_data[1] = mag_raw_values[3] << 8 | mag_raw_values[2]; //Y
      magn_data[2] = mag_raw_values[5] << 8 | mag_raw_values[4]; //Z
      //printf ("%d,%d,%d\n",magn_data[0], magn_data[1], magn_data[2]);

      magn_data_axis_corrected[0] = cross_axis[0][0]*magn_data[0] + cross_axis[0][1]*magn_data[1] + cross_axis[0][2]*magn_data[2];
      magn_data_axis_corrected[1] = cross_axis[1][0]*magn_data[0] + cross_axis[1][1]*magn_data[1] + cross_axis[1][2]*magn_data[2];
      magn_data_axis_corrected[2] = cross_axis[2][0]*magn_data[0] + cross_axis[2][1]*magn_data[1] + cross_axis[2][2]*magn_data[2];
      //printf ("%0.4f,%0.4f,%0.4f\n",magn_data_axis_corrected[0], magn_data_axis_corrected[1], magn_data_axis_corrected[2]); // for compass calibration


      for (i=0;i<3;i++) magn_wo_hb[i] = (float)magn_data_axis_corrected[i] - mag_hard_bias[i];
      //printf ("%0.4f,%0.4f,%0.4f\n",magn_wo_hb[0], magn_wo_hb[1], magn_wo_hb[2]);

      magn_data_calibrated[0] = mag_A_inv[0][0]*magn_wo_hb[0] + mag_A_inv[0][1]*magn_wo_hb[1] + mag_A_inv[0][2]*magn_wo_hb[2];
      magn_data_calibrated[1] = mag_A_inv[1][0]*magn_wo_hb[0] + mag_A_inv[1][1]*magn_wo_hb[1] + mag_A_inv[1][2]*magn_wo_hb[2];
      magn_data_calibrated[2] = mag_A_inv[2][0]*magn_wo_hb[0] + mag_A_inv[2][1]*magn_wo_hb[1] + mag_A_inv[2][2]*magn_wo_hb[2];

      //ESP_LOGI(TAG_IST8310,"Mag values are %d, %d, %d, %0.2f",(int16_t)magn_data_calibrated[0],(int16_t)magn_data_calibrated[1],(int16_t)magn_data_calibrated[2], 
      //sqrt(magn_data_calibrated[0]*magn_data_calibrated[0] + magn_data_calibrated[1]*magn_data_calibrated[1] + magn_data_calibrated[2]*magn_data_calibrated[2]));
      //printf ("%0.2f,%0.2f,%0.2f,%0.2f\n",magn_data_calibrated[0], magn_data_calibrated[1], magn_data_calibrated[2], sqrt(magn_data_calibrated[0]*magn_data_calibrated[0] + magn_data_calibrated[1]*magn_data_calibrated[1] + magn_data_calibrated[2]*magn_data_calibrated[2]));
      //heading = atan2(magn_data_calibrated[1],magn_data_calibrated[0]) * (180/M_PI);
      //heading -=90.0;
      //if (heading<0) heading+=360.0;
      //printf ("%d\n",(int16_t) heading);
      //uart_write_bytes(REMOTE_CONTROL_UART, magn_data, NUMBER_OF_BYTES_TO_SEND_TO_RC);
      //length = sprintf(M,"%i,%i,%i\n",magn_data[0],magn_data[1],magn_data[2]);
      //uart_write_bytes(LIDAR_UART, M, length);
                 
      xQueueSend(magnetometer_queue, magn_data_calibrated, NULL); 
    }
  }  
}
