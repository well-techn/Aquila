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

extern spi_device_handle_t MPU6000_1;
extern spi_device_handle_t MPU6000_2;
extern const char *TAG_SERVICE;

void gyro_calibration (void *pvParameters)
{
    uint8_t sensor_data_1[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
    uint8_t sensor_data_2[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
  
    int16_t gyro_raw_1[3] = {0,0,0}; 
    int16_t gyro_raw_2[3] = {0,0,0};
  
    uint16_t counter = 0;
    uint8_t i = 0;
            
    float Gyro_X_cal_1 = 0.0;
    float Gyro_Y_cal_1 = 0.0;
    float Gyro_Z_cal_1 = 0.0;
    
    float Gyro_X_cal_2 = 0.0;
    float Gyro_Y_cal_2 = 0.0;
    float Gyro_Z_cal_2 = 0.0;

    int16_t gyro_1_offset[3] = {0,0,0};
    int16_t gyro_2_offset[3] = {0,0,0};

    int16_t *client_fd = pvParameters;

  #ifdef TELNET_CONF_MODE
    send(*client_fd, "Проверка связи с MPU#1.....\r\n", sizeof("Проверка связи с MPU#1.....\r\n"), 0);
  #endif
  ESP_LOGI(TAG_SERVICE,"Проверка связи с MPU#1.....");
  if (MPU6000_communication_check(MPU6000_1) != ESP_OK) {
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);}  }
  
#ifdef TELNET_CONF_MODE
    send(*client_fd, "Проверка связи с MPU#2.....\r\n", sizeof("Проверка связи с MPU#2.....\r\n"), 0);
#endif
  
  ESP_LOGI(TAG_SERVICE,"Проверка связи с MPU#2.....");
  if (MPU6000_communication_check(MPU6000_2) != ESP_OK) {
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

#ifdef TELNET_CONF_MODE
    send(*client_fd, "Настройка MPU#1.....\r\n", sizeof("Настройка MPU#1.....\r\n"), 0);
#endif
  ESP_LOGI(TAG_SERVICE,"Настройка MPU#1.....");
  if (MPU6000_init(MPU6000_1) != ESP_OK) {
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

#ifdef TELNET_CONF_MODE
    send(*client_fd, "Настройка MPU#2.....\r\n", sizeof("Настройка MPU#2.....\r\n"), 0);
#endif

  ESP_LOGI(TAG_SERVICE,"Настройка MPU#2.....");
  if (MPU6000_init(MPU6000_2) != ESP_OK) {
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

#ifdef TELNET_CONF_MODE
    send(*client_fd, "Перенастройка SPI на 20МГц.....\r\n", sizeof("Перенастройка SPI на 20МГц.....\r\n"), 0);
#endif

  ESP_LOGI(TAG_SERVICE,"Перенастройка SPI на 20МГц.....");
  SPI_change_MPUs_speed();
  ESP_LOGI(TAG_SERVICE,"Оба SPI перенастроены");

#ifdef TELNET_CONF_MODE
    send(*client_fd, "Начинаем калибровку IMU\r\n", sizeof("Начинаем калибровку IMU\r\n"), 0);
#endif

  ESP_LOGI(TAG_SERVICE,"Начинаем калибровку IMU");

  while (1)
  {
    //здесь временная точность не нужна, важны лишь показания
    ets_delay_us(1000); 
    
    SPI_read_bytes(MPU6000_1, 0, 0, 8, MPU6000_ACCEL_XOUT_H | SPI_READ_FLAG, 0, sensor_data_1, 14);
    SPI_read_bytes(MPU6000_2, 0, 0, 8, MPU6000_ACCEL_XOUT_H | SPI_READ_FLAG, 0, sensor_data_2, 14);

    //формируем "сырые" показания гироскопов
    gyro_raw_1[0] = (sensor_data_1[8] << 8) | sensor_data_1[9];            //X                  
    gyro_raw_1[1] = (sensor_data_1[10] << 8) | sensor_data_1[11];          //Y
    gyro_raw_1[2] = (sensor_data_1[12] << 8) | sensor_data_1[13];          //Z
    
    gyro_raw_2[0] = (sensor_data_2[8] << 8) | sensor_data_2[9];            //X                  
    gyro_raw_2[1] = (sensor_data_2[10] << 8) | sensor_data_2[11];          //Y
    gyro_raw_2[2] = (sensor_data_2[12] << 8) | sensor_data_2[13];          //Z

    if  (counter < NUMBER_OF_IMU_CALIBRATION_COUNTS)
    { 
      Gyro_X_cal_1 += gyro_raw_1[0];
      Gyro_Y_cal_1 += gyro_raw_1[1];
      Gyro_Z_cal_1 += gyro_raw_1[2];

      Gyro_X_cal_2 += gyro_raw_2[0];
      Gyro_Y_cal_2 += gyro_raw_2[1];
      Gyro_Z_cal_2 += gyro_raw_2[2];
      }
//по достижении этого значения вычисляем коэффициенты, сохраняем их в NVS и просим перезапустить систему
      if (counter == NUMBER_OF_IMU_CALIBRATION_COUNTS)
      {
        gyro_1_offset[0] = (Gyro_X_cal_1 / (NUMBER_OF_IMU_CALIBRATION_COUNTS - 1));
        gyro_1_offset[1] = (Gyro_Y_cal_1 / (NUMBER_OF_IMU_CALIBRATION_COUNTS - 1));
        gyro_1_offset[2] = (Gyro_Z_cal_1 / (NUMBER_OF_IMU_CALIBRATION_COUNTS - 1));

        gyro_2_offset[0] = (Gyro_X_cal_2 / (NUMBER_OF_IMU_CALIBRATION_COUNTS - 1));
        gyro_2_offset[1] = (Gyro_Y_cal_2 / (NUMBER_OF_IMU_CALIBRATION_COUNTS - 1));
        gyro_2_offset[2] = (Gyro_Z_cal_2 / (NUMBER_OF_IMU_CALIBRATION_COUNTS - 1));
          
        for (i=0;i<3;i++) ESP_LOGI(TAG_SERVICE,"MPU#1 сдвиг нуля гироскопа %d",gyro_1_offset[i]);
        for (i=0;i<3;i++) ESP_LOGI(TAG_SERVICE,"MPU#2 сдвиг нуля гироскопа %d",gyro_2_offset[i]);

#ifdef TELNET_CONF_MODE
        char message_to_print[90];
        uint8_t pos = 0;

        pos = sprintf(message_to_print, "MPU#1 сдвиг нуля гироскопа %d, %d, %d\r\n", gyro_1_offset[0], gyro_1_offset[1], gyro_1_offset[2]);
        send(*client_fd, message_to_print, pos, 0);

        pos = sprintf(message_to_print, "MPU#2 сдвиг нуля гироскопа %d, %d, %d\r\n", gyro_2_offset[0], gyro_2_offset[1], gyro_2_offset[2]);
        send(*client_fd, message_to_print, pos, 0);
 #endif

        printf("\n");

  //запись во flash (NVS) калибровочных коэффициентов акселерометров и гироскопов

        ESP_ERROR_CHECK(nvs_flash_erase());
        esp_err_t err = nvs_flash_init();
        
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {

            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        ESP_ERROR_CHECK( err );

        ESP_LOGI(TAG_SERVICE,"Открываем NVS... ");
        nvs_handle_t NVS_handle;
        err = nvs_open("storage", NVS_READWRITE, &NVS_handle);
        if (err != ESP_OK) ESP_LOGE(TAG_SERVICE,"Ошибка (%s) открытия NVS!\n", esp_err_to_name(err));
        else 
        {
          ESP_LOGI(TAG_SERVICE,"Записываем gyro_1_X offset %d... ", gyro_1_offset[0]);
          err = nvs_set_i16(NVS_handle, "gyro_1_off_0", gyro_1_offset[0]);
          
          ESP_LOGI(TAG_SERVICE,"Записываем gyro_1_Y offset %d... ", gyro_1_offset[1]);
          err = nvs_set_i16(NVS_handle, "gyro_1_off_1", gyro_1_offset[1]);
        
          ESP_LOGI(TAG_SERVICE,"Записываем gyro_1_Z offset %d... ", gyro_1_offset[2]);
          err = nvs_set_i16(NVS_handle, "gyro_1_off_2", gyro_1_offset[2]);

          ESP_LOGI(TAG_SERVICE,"Записываем gyro_2_X offset %d... ", gyro_2_offset[0]);
          err = nvs_set_i16(NVS_handle, "gyro_2_off_0", gyro_2_offset[0]);
          
          ESP_LOGI(TAG_SERVICE,"Записываем gyro_2_Y offset %d... ", gyro_2_offset[1]);
          err = nvs_set_i16(NVS_handle, "gyro_2_off_1", gyro_2_offset[1]);
        
          ESP_LOGI(TAG_SERVICE,"Записываем gyro_2_Z offset %d... ", gyro_2_offset[2]);
          err = nvs_set_i16(NVS_handle, "gyro_2_off_2", gyro_2_offset[2]);

          ESP_LOGI(TAG_SERVICE,"Сохраняем данные в NVS... ");
#ifdef TELNET_CONF_MODE
      send(*client_fd, "Сохраняем данные в NVS ... ", sizeof("Сохраняем данные в NVS...\r\n"), 0);
#endif
          err = nvs_commit(NVS_handle);

          nvs_close(NVS_handle);
        }

      ESP_LOGI(TAG_SERVICE,"Калибровка IMU завершена, перезапустите систему.");
#ifdef TELNET_CONF_MODE
      send(*client_fd, "Калибровка IMU завершена, перезапустите систему.", sizeof("Калибровка IMU завершена, перезапустите систему."), 0);
#endif

      vTaskDelete (NULL);
    }

    counter++;
    if ((counter % 500) == 0) 
    {
      printf("%d\n", counter/500);
#ifdef TELNET_CONF_MODE
        char message_to_print[60];
        uint8_t pos = 0;
        pos = sprintf(message_to_print, "\r[***** %d%% ******]",(counter * 100/NUMBER_OF_IMU_CALIBRATION_COUNTS));
        send(*client_fd, message_to_print, pos, 0);
#endif
    }
  }
}


