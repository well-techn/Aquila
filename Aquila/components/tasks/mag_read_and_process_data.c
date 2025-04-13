//задача периодического считывания данных с магнетометра IST8310. После создения находится с заблокированном состоянии. Разблокируется из main_flying_cycle.
//Активирует режим однократного измерения, считываем результат после задержки и выдает результат в очередь в сторону main_flying_cycle

//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "IST8310.h" 

extern SemaphoreHandle_t semaphore_for_i2c_external;
extern QueueHandle_t magnetometer_queue;
extern char *TAG_IST8310;

void mag_read_and_process_data (void * pvParameters)
{
  uint8_t i = 0;
  float cross_axis[3][3] = {{0.9800471,  -0.0310357,   -0.0148492},    //calculated manually based on data read from the chip
                            {0.0304362,   1.0342328,   -0.0004612},
                            {-0.0374089,  0.0419651,    1.0106678}};
  uint8_t mag_raw_values[6] = {0,0,0,0,0,0}; 
  int16_t magn_data[3]= {0,0,0};
  float magn_data_axis_corrected[3]= {0,0,0};
  float hard_bias[3] = {-51.782394, 18.121590, -66.742550};   //calibration values with Magneto 1.2
  
  float A_inv[3][3] = {{1.019563,   0.014807,   -0.010553},    //calibration values with Magneto 1.2
                       {0.014807,   1.037489,   0.006424},
                       {-0.010553,  0.006424,   1.111749}};
  
  float magn_wo_hb[3] = {0,0,0};
  float magn_data_calibrated[3] = {0,0,0};

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


      for (i=0;i<3;i++) magn_wo_hb[i] = (float)magn_data_axis_corrected[i] - hard_bias[i];
      //printf ("%0.4f,%0.4f,%0.4f\n",magn_wo_hb[0], magn_wo_hb[1], magn_wo_hb[2]);

      magn_data_calibrated[0] = A_inv[0][0]*magn_wo_hb[0] + A_inv[0][1]*magn_wo_hb[1] + A_inv[0][2]*magn_wo_hb[2];
      magn_data_calibrated[1] = A_inv[1][0]*magn_wo_hb[0] + A_inv[1][1]*magn_wo_hb[1] + A_inv[1][2]*magn_wo_hb[2];
      magn_data_calibrated[2] = A_inv[2][0]*magn_wo_hb[0] + A_inv[2][1]*magn_wo_hb[1] + A_inv[2][2]*magn_wo_hb[2];

      //ESP_LOGI(TAG_IST8310,"Mag values are %d, %d, %d, %0.2f",(int16_t)magn_data_calibrated[0],(int16_t)magn_data_calibrated[1],(int16_t)magn_data_calibrated[2], 
      //sqrt(magn_data_calibrated[0]*magn_data_calibrated[0] + magn_data_calibrated[1]*magn_data_calibrated[1] + magn_data_calibrated[2]*magn_data_calibrated[2]));
      //printf ("%0.2f,%0.2f,%0.2f,%0.2f\n",magn_data_calibrated[0], magn_data_calibrated[1], magn_data_calibrated[2], sqrt(magn_data_calibrated[0]*magn_data_calibrated[0] + magn_data_calibrated[1]*magn_data_calibrated[1] + magn_data_calibrated[2]*magn_data_calibrated[2]));
      //heading = atan2(magn_data_calibrated[1],magn_data_calibrated[0]) * (180/PI);
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
