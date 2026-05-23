#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "MPU6000.h"
#include "esp_log.h"
#include "wt_alldef.h"
#include "wt_spi.h"
#include <rom/ets_sys.h>
#include "math.h"
#include "string.h"
#ifdef TELNET_CONF_MODE
  #include <lwip/sockets.h>
#endif

#include "logging_operations.h"

extern spi_device_handle_t MPU6000_1;
extern spi_device_handle_t MPU6000_2;
extern const char *TAG_SERVICE;

void gyro_calibration (void *pvParameters)
{
    uint8_t sensor_data_1[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
    uint8_t sensor_data_2[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
  
    int16_t gyro_raw_1[3] = {0}; 
    int16_t gyro_raw_2[3] = {0};
  
    uint16_t counter = 0;
            
    float Gyro_X_cal_1 = 0.0;
    float Gyro_Y_cal_1 = 0.0;
    float Gyro_Z_cal_1 = 0.0;
    
    float Gyro_X_cal_2 = 0.0;
    float Gyro_Y_cal_2 = 0.0;
    float Gyro_Z_cal_2 = 0.0;

    float gyro_1_offset[3] = {0.0};
    float gyro_2_offset[3] = {0.0};

    uint32_t* p_uint32;
    
    int16_t client_fd = (pvParameters) ? *(int16_t *)pvParameters : -1;

  print_service_message(client_fd, "Проверка связи с MPU#1.....\r\n");
  if (MPU6000_communication_check(MPU6000_1) != ESP_OK) {
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);}  }
  
  print_service_message(client_fd, "Проверка связи с MPU#2.....\r\n");
  if (MPU6000_communication_check(MPU6000_2) != ESP_OK) {
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  print_service_message(client_fd, "Настройка MPU#1.....\r\n");
  if (MPU6000_init(MPU6000_1, 
                    IMU_SAMPLING_FREQ_HZ, 
                    IMU_HW_LPF_CUTOFF_HZ,
                    IMU_ACCEL_FULL_SCALE_G,
                    IMU_GYRO_FULL_SCALE_DPS)  != ESP_OK)  {
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  print_service_message(client_fd, "Настройка MPU#2.....\r\n");
  if (MPU6000_init(MPU6000_2, 
                    IMU_SAMPLING_FREQ_HZ, 
                    IMU_HW_LPF_CUTOFF_HZ,
                    IMU_ACCEL_FULL_SCALE_G,
                    IMU_GYRO_FULL_SCALE_DPS)  != ESP_OK)  {
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  print_service_message(client_fd, "Перенастройка SPI на 20МГц.....\r\n");
  SPI_change_MPUs_speed();
  print_service_message(client_fd, "Оба SPI перенастроены\r\n");

  print_service_message(client_fd, "Начинаем калибровку гироскопов\r\n");
  while (1)
  {
//здесь временная точность не нужна, важны лишь показания
    ets_delay_us(1000);
    counter++;
//выводим уровень прогресса
    if ((counter % 500) == 0) 
    {
    uint8_t progress = (counter * 100 / NUMBER_OF_GYRO_INPUTS);
    print_service_message(client_fd, "\r[***** %d%% ******]", progress);
    }
//считываем данные    
    SPI_read_bytes(MPU6000_1, 0, 0, 8, MPU6000_ACCEL_XOUT_H | SPI_READ_FLAG, 0, sensor_data_1, 14);
    SPI_read_bytes(MPU6000_2, 0, 0, 8, MPU6000_ACCEL_XOUT_H | SPI_READ_FLAG, 0, sensor_data_2, 14);

//формируем "сырые" показания гироскопов
    gyro_raw_1[0] = (sensor_data_1[8] << 8) | sensor_data_1[9];            //X                  
    gyro_raw_1[1] = (sensor_data_1[10] << 8) | sensor_data_1[11];          //Y
    gyro_raw_1[2] = (sensor_data_1[12] << 8) | sensor_data_1[13];          //Z
    
    gyro_raw_2[0] = (sensor_data_2[8] << 8) | sensor_data_2[9];            //X                  
    gyro_raw_2[1] = (sensor_data_2[10] << 8) | sensor_data_2[11];          //Y
    gyro_raw_2[2] = (sensor_data_2[12] << 8) | sensor_data_2[13];          //Z

//накапливаем в течение заданного кол-ва отсчетов
    if  (counter < NUMBER_OF_GYRO_INPUTS)
    { 
      Gyro_X_cal_1 += gyro_raw_1[0];
      Gyro_Y_cal_1 += gyro_raw_1[1];
      Gyro_Z_cal_1 += gyro_raw_1[2];

      Gyro_X_cal_2 += gyro_raw_2[0];
      Gyro_Y_cal_2 += gyro_raw_2[1];
      Gyro_Z_cal_2 += gyro_raw_2[2];
      }
//по достижении этого значения вычисляем коэффициенты, сохраняем их в NVS и просим перезапустить систему
      if (counter == NUMBER_OF_GYRO_INPUTS)
      {
        gyro_1_offset[0] = (Gyro_X_cal_1 / (NUMBER_OF_GYRO_INPUTS - 1));
        gyro_1_offset[1] = (Gyro_Y_cal_1 / (NUMBER_OF_GYRO_INPUTS - 1));
        gyro_1_offset[2] = (Gyro_Z_cal_1 / (NUMBER_OF_GYRO_INPUTS - 1));

        gyro_2_offset[0] = (Gyro_X_cal_2 / (NUMBER_OF_GYRO_INPUTS - 1));
        gyro_2_offset[1] = (Gyro_Y_cal_2 / (NUMBER_OF_GYRO_INPUTS - 1));
        gyro_2_offset[2] = (Gyro_Z_cal_2 / (NUMBER_OF_GYRO_INPUTS - 1));
          
    print_service_message(client_fd, "MPU#1 сдвиг нуля гироскопа X: %0.5f, Y: %0.5f, Z: %0.5f\r\n", gyro_1_offset[0], gyro_1_offset[1], gyro_1_offset[2]);
    print_service_message(client_fd, "MPU#2 сдвиг нуля гироскопа X: %0.5f, Y: %0.5f, Z: %0.5f\r\n", gyro_2_offset[0], gyro_2_offset[1], gyro_2_offset[2]);


        printf("\n");

//запись во flash (NVS) калибровочных коэффициентов акселерометров и гироскопов
        esp_err_t err = nvs_flash_init();
        
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) 
        {
          ESP_ERROR_CHECK(nvs_flash_erase());
          err = nvs_flash_init();
        }
        ESP_ERROR_CHECK( err );

        print_service_message(client_fd, "Открываем NVS... \r\n");
        nvs_handle_t coeff_NVS_handle;
        err = nvs_open("coeff_storage", NVS_READWRITE, &coeff_NVS_handle);
        if (err != ESP_OK) ESP_LOGE(TAG_SERVICE,"Ошибка (%s) открытия NVS!\r\n", esp_err_to_name(err));
        else 
        {
          p_uint32 = (uint32_t*)&gyro_1_offset[0];
          print_service_message(client_fd, "Записываем gyro_1_offset\r\n");
          ESP_ERROR_CHECK(nvs_set_u32(coeff_NVS_handle, "gyro_1_off[0]", *(p_uint32++)));
          ESP_ERROR_CHECK(nvs_set_u32(coeff_NVS_handle, "gyro_1_off[1]", *(p_uint32++)));
          ESP_ERROR_CHECK(nvs_set_u32(coeff_NVS_handle, "gyro_1_off[2]", *(p_uint32++)));

          p_uint32 = (uint32_t*)&gyro_2_offset[0];
          print_service_message(client_fd, "Записываем gyro_2_offset\r\n");
          ESP_ERROR_CHECK(nvs_set_u32(coeff_NVS_handle, "gyro_2_off[0]", *(p_uint32++)));
          ESP_ERROR_CHECK(nvs_set_u32(coeff_NVS_handle, "gyro_2_off[1]", *(p_uint32++)));
          ESP_ERROR_CHECK(nvs_set_u32(coeff_NVS_handle, "gyro_2_off[2]", *(p_uint32++)));

          print_service_message(client_fd, "Сохраняем данные в NVS...\r\n");          
          ESP_ERROR_CHECK(nvs_commit(coeff_NVS_handle));

          nvs_close(coeff_NVS_handle);
        }

        print_service_message(client_fd, "Калибровка гироскопов завершена, ESC для перезапуска или повторите выбор меню\r\n"); 
//убиваем задачу
      vTaskDelete (NULL);
    }
  }
}


