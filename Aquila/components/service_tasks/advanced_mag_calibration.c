#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "IST8310.h" 
#include "esp_log.h"
#include "wt_alldef.h"
#include "wt_spi.h"
#include <rom/ets_sys.h>
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "advanced_math.h"
#include "inttypes.h"
#include "driver/gpio.h"
#include "nvs.h"

extern SemaphoreHandle_t semaphore_for_i2c_external;
extern QueueHandle_t magnetometer_queue;
extern const char *TAG_SERVICE;

void advanced_mag_calibration (void *pvParameters)
{
   float cross_axis[3][3] =  {{0,  0,  0},    
                              {0,  0,  0},
                              {0,  0,  0}};
  uint8_t mag_raw_values[6] = {0,0,0,0,0,0}; 
  int16_t magn_data[3]= {0,0,0};
  float magn_data_axis_corrected[3]= {0,0,0};

  float input_data[NUMBER_OF_MAG_INPUTS][3];

  double *A_1, *B;
  uint16_t i = 0;

  uint64_t* p_uint64;

  ESP_LOGI(TAG_SERVICE,"Проверка связи с IST8310.....");
  if (IST8310_communication_check() != ESP_OK) vTaskDelete(NULL);

  ESP_LOGI(TAG_SERVICE,"Настройка IST8310.....");
    if (IST8310_configuration() != ESP_OK) vTaskDelete(NULL);

  ESP_LOGI(TAG_SERVICE,"Производим тест IST8310.....");
  if (IST8310_selftest() != ESP_OK) vTaskDelete(NULL);

  ESP_LOGI(TAG_SERVICE,"Считываем данные cross-axis calibration из IST8310.....");
  IST8310_generate_cross_axis_matrix(cross_axis);

  ESP_LOGI(TAG_SERVICE,"Начинаем калибровку магнетометра");

   while (i < NUMBER_OF_MAG_INPUTS)
    {
      printf("Считываем вектор %d\n", i);
     // считываем показания
      IST8310_request_data();
      vTaskDelay(50/portTICK_PERIOD_MS);   //50ms чтобы можно было успевать ворочать
      IST8310_read_data(mag_raw_values);
  
      magn_data[0] = mag_raw_values[1] << 8 | mag_raw_values[0]; //X
      magn_data[1] = mag_raw_values[3] << 8 | mag_raw_values[2]; //Y
      magn_data[2] = mag_raw_values[5] << 8 | mag_raw_values[4]; //Z
      //printf ("%d,%d,%d\n",magn_data[0], magn_data[1], magn_data[2]);

      //выравниваем оси по алгоритму, заложенному в самом магнетометре
      magn_data_axis_corrected[0] = cross_axis[0][0]*magn_data[0] + cross_axis[0][1]*magn_data[1] + cross_axis[0][2]*magn_data[2];
      magn_data_axis_corrected[1] = cross_axis[1][0]*magn_data[0] + cross_axis[1][1]*magn_data[1] + cross_axis[1][2]*magn_data[2];
      magn_data_axis_corrected[2] = cross_axis[2][0]*magn_data[0] + cross_axis[2][1]*magn_data[1] + cross_axis[2][2]*magn_data[2];

      for (uint8_t j = 0; j<3; j++)
      {
        input_data[i][j] = magn_data_axis_corrected[j];
      }
      i++;
    }

    printf("Записанный массив данных \n\n");
    for (i = 0; i < NUMBER_OF_MAG_INPUTS; i++)
        printf("(%0.3f, %0.3f, %0.3f)\n", input_data[i][0], input_data[i][1], input_data[i][2]);


    //выделяем память под матицы для расчетов для акселерометра 1
    A_1 = (double *)malloc(3 * 3 * sizeof(double));
    B = (double *)malloc(3 * sizeof(double));

    printf("\nНачинаем расчет корректировочных коэффициентов для магнетометра\n");
    calculation_B_and_Ainv_with_exclusion(input_data, A_1, B, 150, 0, NUMBER_OF_MAG_INPUTS);

    esp_err_t err = nvs_flash_init();   
    ESP_ERROR_CHECK( err );
    nvs_handle_t NVS_handle;

    err = nvs_open("storage", NVS_READWRITE, &NVS_handle);
    if (err != ESP_OK) ESP_LOGE(TAG_SERVICE,"Ошибка (%s) открытия NVS!\n", esp_err_to_name(err));
    else 
    {
        //Записываем mag_hard_bias[0] - mag_hard_bias[2]
        p_uint64 = (uint64_t*)&B[0];
        err = nvs_set_u64(NVS_handle, "mag_h_bias[0]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "mag_h_bias[1]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "mag_h_bias[2]", *(p_uint64++));

        //Записываем mag_A_i[00]-mag_A_inv[33] 
        p_uint64 = (uint64_t*)&A_1[0]; 
        err = nvs_set_u64(NVS_handle, "mag_A_i[00]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "mag_A_i[01]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "mag_A_i[02]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "mag_A_i[10]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "mag_A_i[11]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "mag_A_i[12]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "mag_A_i[20]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "mag_A_i[21]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "mag_A_i[22]", *(p_uint64++));
    }
    
    printf("Сохраняем данные в NVS ... ");
    err = nvs_commit(NVS_handle);
    if (err == ESP_OK) printf("Данные сохранены\n");
        else printf("Ошибка сохранения данных!\n");
    nvs_close(NVS_handle);
    printf("Для перезапуска нажмите ESC\r\n");

    free(A_1);
    free(B);

    vTaskDelete(NULL);

}


