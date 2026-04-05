//задача периодического считывания данных с магнетометра IST8310. После создения находится с заблокированном состоянии. Разблокируется из main_flying_cycle.
//Активирует режим однократного измерения, считываем результат после задержки и выдает результат в очередь в сторону main_flying_cycle

//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "inttypes.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "advanced_math.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "IST8310.h" 
#include "NVS_operations.h"

extern SemaphoreHandle_t semaphore_for_i2c_external;
extern QueueHandle_t magnetometer_queue;
extern char *TAG_IST8310;
extern char *TAG_NVS;

void mag_read_and_process_data (void * pvParameters)
{
  float cross_axis[3][3] =  {0};
  uint8_t mag_raw_values[6] = {0}; 
  int16_t magn_data[3]= {0};
  float magn_data_axis_corrected[3]= {0};
  float mag_hard_bias[3] = {0};   //calibration values with Magneto 1.2
  
  float mag_A_inv[9] = {0};   //calibration values with Magneto 1.2

  float magn_data_calibrated[3] = {0};
  float magn_data_calibrated_NED[3] = {0};
  float __attribute__((aligned(16))) calibration_temp[9] = {0};

  nvs_handle_t coeff_NVS_handle;

//инициализируем и открываем NVS  
  ESP_ERROR_CHECK(NVS_prepare(&coeff_NVS_handle, "coeff_storage"));

//считываем из NVS массив hard bias магнетометра
  nvs_read_float_array(coeff_NVS_handle, "mag_h_bias", mag_hard_bias, 3);

//считываем из NVS массив Ainv магнетометра
  nvs_read_float_array(coeff_NVS_handle, "mag_Ai", mag_A_inv, 9);

//генерируем cross - axis поправки в соответствии с даташитом
  IST8310_generate_cross_axis_matrix(cross_axis);

  while(1) {
//при получении запроса    
    if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) != 0)
    {
//запрашиваем и через паузу считываем данные с магнетометра
      xSemaphoreTake (semaphore_for_i2c_external,portMAX_DELAY);
      IST8310_request_data();
      xSemaphoreGive(semaphore_for_i2c_external);
      vTaskDelay(5/portTICK_PERIOD_MS);   //5мс так как максимальная частота опроса 200Гц
      xSemaphoreTake (semaphore_for_i2c_external,portMAX_DELAY);
      IST8310_read_data(mag_raw_values);
      xSemaphoreGive(semaphore_for_i2c_external);
//формируем показания  
      magn_data[0] = mag_raw_values[1] << 8 | mag_raw_values[0]; //X
      magn_data[1] = mag_raw_values[3] << 8 | mag_raw_values[2]; //Y
      magn_data[2] = mag_raw_values[5] << 8 | mag_raw_values[4]; //Z
//применяем cross-axis поправки 
      magn_data_axis_corrected[0] = cross_axis[0][0]*magn_data[0] + cross_axis[0][1]*magn_data[1] + cross_axis[0][2]*magn_data[2];
      magn_data_axis_corrected[1] = cross_axis[1][0]*magn_data[0] + cross_axis[1][1]*magn_data[1] + cross_axis[1][2]*magn_data[2];
      magn_data_axis_corrected[2] = cross_axis[2][0]*magn_data[0] + cross_axis[2][1]*magn_data[1] + cross_axis[2][2]*magn_data[2];
//применяем калибровочные коэффициенты считанные с флеш
//убираем offset и одновременно применяем Ainv, в безразмерных величинах     
      vector_3D_calibration(magn_data_axis_corrected, mag_hard_bias, mag_A_inv, magn_data_calibrated);
//разворачиваем магнетометр в соответствии с NED
      magn_data_calibrated_NED[0] = magn_data_calibrated[1];                  //ось магнетометра по N
      magn_data_calibrated_NED[1] = magn_data_calibrated[0];                  //ось магнетометра по E
      magn_data_calibrated_NED[2] = -magn_data_calibrated[2];                 //ось магнетометра по D
//отправляем откалиброванные развернутые данные в main_flying_cycle
//overwrite чтобы не накапливались устаревшие значения если они не были своевременно считаны
      xQueueOverwrite(magnetometer_queue, magn_data_calibrated_NED);
    }
  }  
}
