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
#include "nvs.h"

#include "logging_operations.h"


extern SemaphoreHandle_t semaphore_for_i2c_external;
extern QueueHandle_t magnetometer_queue;
extern const char *TAG_SERVICE;

void advanced_mag_calibration (void *pvParameters)
{
  float cross_axis[3][3] =  {0};
  uint8_t mag_raw_values[6] = {0}; 
  int16_t magn_data[3]= {0};
  float magn_data_axis_corrected[3]= {0};

  float input_data[NUMBER_OF_MAG_INPUTS][3];

  double *A_1, *B;
  float *A_1_f, *B_f;

  uint32_t* p_uint32;

  int16_t client_fd = (pvParameters) ? *(int16_t *)pvParameters : -1;

  print_service_message(client_fd, "Проверка связи с IST8310.....");
  if (IST8310_communication_check() != ESP_OK) vTaskDelete(NULL);

  print_service_message(client_fd, "Настройка IST8310.....");
  if (IST8310_configuration() != ESP_OK) vTaskDelete(NULL);

  print_service_message(client_fd, "Производим тест IST8310.....");
  if (IST8310_selftest() != ESP_OK) vTaskDelete(NULL);

  print_service_message(client_fd, "Считываем данные cross-axis calibration из IST8310.....");
  IST8310_generate_cross_axis_matrix(cross_axis);

  print_service_message(client_fd, "Начинаем калибровку магнетометра, вращайте коптер");

   for (uint16_t i = 0; i<NUMBER_OF_MAG_INPUTS; i++)
    {
      print_service_message(client_fd, "Считываем вектор %d\n", i);
//считываем показания
      IST8310_request_data();
      vTaskDelay(50/portTICK_PERIOD_MS);   //50ms чтобы можно было успевать ворочать
      IST8310_read_data(mag_raw_values);
  
      magn_data[0] = mag_raw_values[1] << 8 | mag_raw_values[0]; //X
      magn_data[1] = mag_raw_values[3] << 8 | mag_raw_values[2]; //Y
      magn_data[2] = mag_raw_values[5] << 8 | mag_raw_values[4]; //Z

//выравниваем оси по алгоритму, заложенному в самом магнетометре
      magn_data_axis_corrected[0] = cross_axis[0][0]*magn_data[0] + cross_axis[0][1]*magn_data[1] + cross_axis[0][2]*magn_data[2];
      magn_data_axis_corrected[1] = cross_axis[1][0]*magn_data[0] + cross_axis[1][1]*magn_data[1] + cross_axis[1][2]*magn_data[2];
      magn_data_axis_corrected[2] = cross_axis[2][0]*magn_data[0] + cross_axis[2][1]*magn_data[1] + cross_axis[2][2]*magn_data[2];

      for (uint8_t j = 0; j<3; j++) input_data[i][j] = magn_data_axis_corrected[j];  
    }

    print_service_message(client_fd, "Записанный массив данных \n\n");
    for (uint16_t i = 0; i < NUMBER_OF_MAG_INPUTS; i++)
        {
          print_service_message(client_fd, "{%0.3f, %0.3f, %0.3f}\n", input_data[i][0], input_data[i][1], input_data[i][2]);
        } 

//выделяем память под матицы для расчетов для акселерометра 1
    A_1 = (double *)malloc(3 * 3 * sizeof(double));
    B = (double *)malloc(3 * sizeof(double));

    print_service_message(client_fd, "\nНачинаем расчет корректировочных коэффициентов для магнетометра\n");
    calculation_B_and_Ainv_with_exclusion(input_data, A_1, B, 150, 0, NUMBER_OF_MAG_INPUTS);

    print_service_message(client_fd,"\r\nКорректировочные значения сдвигов (bias):\r\n");
    print_service_message(client_fd,"%8.6lf %8.6lf %8.6lf\r\n", B[0], B[1], B[2]);
    print_service_message(client_fd,"\r\nКорректирующая матрица Ainv\r\n");
    for (uint8_t i = 0; i < 3; i++)
    {
        print_service_message(client_fd,"%9.6lf %9.6lf %9.6lf\r\n", A_1[i * 3], A_1[i * 3 + 1], A_1[i * 3 + 2]);
    } 

//переводим матрицы во float 
    A_1_f = (float *)malloc(3 * 3 * sizeof(float));
    B_f = (float *)malloc(3 * sizeof(float));
    for (uint8_t i = 0; i<9; i++) A_1_f[i] = (float)A_1[i];
    for (uint8_t i = 0; i<3; i++) B_f[i] = (float)B[i];

    esp_err_t err = nvs_flash_init();   
    ESP_ERROR_CHECK( err );
    nvs_handle_t coeff_NVS_handle;

    err = nvs_open("coeff_storage", NVS_READWRITE, &coeff_NVS_handle);
    if (err != ESP_OK) print_service_message(client_fd,"Ошибка (%s) открытия NVS!\n", esp_err_to_name(err));
    else 
    {
        //Записываем mag_hard_bias[0] - mag_hard_bias[2]
        p_uint32 = (uint32_t*)&B_f[0];
        err = nvs_set_u64(coeff_NVS_handle, "mag_h_bias[0]", *(p_uint32++));
        err = nvs_set_u64(coeff_NVS_handle, "mag_h_bias[1]", *(p_uint32++));
        err = nvs_set_u64(coeff_NVS_handle, "mag_h_bias[2]", *(p_uint32++));

        //Записываем mag_A_i[00]-mag_A_inv[33] 
        p_uint32 = (uint32_t*)&A_1_f[0]; 
        err = nvs_set_u64(coeff_NVS_handle, "mag_Ai[0]", *(p_uint32++));
        err = nvs_set_u64(coeff_NVS_handle, "mag_Ai[1]", *(p_uint32++));
        err = nvs_set_u64(coeff_NVS_handle, "mag_Ai[2]", *(p_uint32++));
        err = nvs_set_u64(coeff_NVS_handle, "mag_Ai[3]", *(p_uint32++));
        err = nvs_set_u64(coeff_NVS_handle, "mag_Ai[4]", *(p_uint32++));
        err = nvs_set_u64(coeff_NVS_handle, "mag_Ai[5]", *(p_uint32++));
        err = nvs_set_u64(coeff_NVS_handle, "mag_Ai[6]", *(p_uint32++));
        err = nvs_set_u64(coeff_NVS_handle, "mag_Ai[7]", *(p_uint32++));
        err = nvs_set_u64(coeff_NVS_handle, "mag_Ai[8]", *(p_uint32++));
    }
    
    print_service_message(client_fd, "Сохраняем данные в NVS ... ");
    err = nvs_commit(coeff_NVS_handle);
    if (err == ESP_OK) print_service_message(client_fd, "Данные сохранены\n");
        else print_service_message(client_fd, "Ошибка сохранения данных!\n");

    nvs_close(coeff_NVS_handle);

    free(A_1);
    free(B);
    free(A_1_f);
    free(B_f);
    print_service_message(client_fd, "Калибровка магнетометра завершена, ESC для перезапуска или повторите выбор меню\r\n"); 



    vTaskDelete(NULL);
}


