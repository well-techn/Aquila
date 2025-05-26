#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "MPU6000.h"
#include "esp_log.h"
#include "wt_alldef.h"
#include "wt_spi.h"
#include <rom/ets_sys.h>
#include "math.h"
#include "string.h"
#include "advanced_math.h"
#include "inttypes.h"
#include "driver/gpio.h"
#include "nvs.h"

extern spi_device_handle_t MPU6000_1;
extern spi_device_handle_t MPU6000_2;
extern const char *TAG_SERVICE;

void advanced_acc_calibration()
{
    float input_data_1[NUMBER_OF_ACC_INPUTS][3];
    float input_data_2[NUMBER_OF_ACC_INPUTS][3];

    double *A_1, *B;
    uint16_t i;
    uint64_t* p_uint64;

    uint8_t sensor_data_1[20] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    uint8_t sensor_data_2[20] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};

    ESP_LOGI(TAG_SERVICE, "Проверка связи с MPU#1.....");
    if (MPU6000_communication_check(MPU6000_1) != ESP_OK) vTaskDelete(NULL);
   
    ESP_LOGI(TAG_SERVICE, "Проверка связи с MPU#2.....");
    if (MPU6000_communication_check(MPU6000_2) != ESP_OK) vTaskDelete(NULL);
   

    ESP_LOGI(TAG_SERVICE, "Настройка MPU#1.....");
    if (MPU6000_init(MPU6000_1) != ESP_OK) vTaskDelete(NULL);


    ESP_LOGI(TAG_SERVICE, "Настройка MPU#2.....");
    if (MPU6000_init(MPU6000_2) != ESP_OK) vTaskDelete(NULL);

    ESP_LOGI(TAG_SERVICE, "Перенастройка SPI на 20МГц.....");
    SPI_change_MPUs_speed();
    ESP_LOGI(TAG_SERVICE, "Оба SPI перенастроены");

    ESP_LOGI(TAG_SERVICE, "Начинаем калибровку IMU");
    i = 0;

    while (i < NUMBER_OF_ACC_INPUTS)
    {
        printf("Вектор #%d\n", i);
        // обратный отсчет
        for (uint8_t j = 5; j > 0; j--)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            printf("%d\n", j);
        }

        gpio_set_level(LED_GREEN, 0);
        // считываем показания
        SPI_read_bytes(MPU6000_1, 0, 0, 8, MPU6000_ACCEL_XOUT_H | SPI_READ_FLAG, 0, sensor_data_1, 14);
        SPI_read_bytes(MPU6000_2, 0, 0, 8, MPU6000_ACCEL_XOUT_H | SPI_READ_FLAG, 0, sensor_data_2, 14);

        // формируем "сырые" показания акселерометров
        input_data_1[i][0] = (int16_t)((sensor_data_1[0] << 8) | sensor_data_1[1]); // X
        input_data_1[i][1] = (int16_t)((sensor_data_1[2] << 8) | sensor_data_1[3]); // Y
        input_data_1[i][2] = (int16_t)((sensor_data_1[4] << 8) | sensor_data_1[5]); // Z

        input_data_2[i][0] = (int16_t)((sensor_data_2[0] << 8) | sensor_data_2[1]); // X
        input_data_2[i][1] = (int16_t)((sensor_data_2[2] << 8) | sensor_data_2[3]); // Y
        input_data_2[i][2] = (int16_t)((sensor_data_2[4] << 8) | sensor_data_2[5]); // Z

        i++;
        printf("Сохранен\n\n");
    }

    printf("Записанный массив данных \n\n");
    for (int i = 0; i < NUMBER_OF_ACC_INPUTS; i++)
        printf("{%0.1f, %0.1f, %0.1f},  {%0.1f, %0.1f, %0.1f},\n", input_data_1[i][0], input_data_1[i][1], input_data_1[i][2],
               input_data_2[i][0], input_data_2[i][1], input_data_2[i][2]);

    //выделяем память под матицы для расчетов для акселерометра 1
    A_1 = (double *)malloc(3 * 3 * sizeof(double));
    B = (double *)malloc(3 * sizeof(double));

    // вычисляем корректировочные параметры для акселерометра 1
    printf("\nНачинаем расчет корректировочных коэффициентов для акселерометра 1\n");
    calculation_B_and_Ainv_with_exclusion(input_data_1, A_1, B, 8192, 0, NUMBER_OF_ACC_INPUTS);

    esp_err_t err = nvs_flash_init();   
    ESP_ERROR_CHECK( err );
    nvs_handle_t NVS_handle;

    err = nvs_open("storage", NVS_READWRITE, &NVS_handle);
    if (err != ESP_OK) ESP_LOGE(TAG_SERVICE,"Ошибка (%s) открытия NVS!\n", esp_err_to_name(err));
    else 
    {
        //Записываем mag_bias[0] - mag_bias[2]
        p_uint64 = (uint64_t*)&B[0];
        err = nvs_set_u64(NVS_handle, "accel_1_bias[0]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "accel_1_bias[1]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "accel_1_bias[2]", *(p_uint64++));

        //Записываем mag_A_i[00]-mag_A_inv[33] 
        p_uint64 = (uint64_t*)&A_1[0]; 
        err = nvs_set_u64(NVS_handle, "accel_1_A_i[00]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "accel_1_A_i[01]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "accel_1_A_i[02]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "accel_1_A_i[10]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "accel_1_A_i[11]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "accel_1_A_i[12]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "accel_1_A_i[20]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "accel_1_A_i[21]", *(p_uint64++));
        err = nvs_set_u64(NVS_handle, "accel_1_A_i[22]", *(p_uint64++));
    }
    
    // вычисляем корректировочные параметры для акселерометра 2
    printf("\nНачинаем расчет корректировочных коэффициентов для акселерометра 2\n");
    calculation_B_and_Ainv_with_exclusion(input_data_2, A_1, B, 8192, 0, NUMBER_OF_ACC_INPUTS);

    //Записываем accel_2_bias[0] - accel_2_bias[2]
    p_uint64 = (uint64_t*)&B[0];
    err = nvs_set_u64(NVS_handle, "accel_2_bias[0]", *(p_uint64++));
    err = nvs_set_u64(NVS_handle, "accel_2_bias[1]", *(p_uint64++));
    err = nvs_set_u64(NVS_handle, "accel_2_bias[2]", *(p_uint64++));

    //Записываем accel_2_A_inv[00]-mag_A_inv[33] 
    p_uint64 = (uint64_t*)&A_1[0];
    err = nvs_set_u64(NVS_handle, "accel_2_A_i[00]", *(p_uint64++));
    err = nvs_set_u64(NVS_handle, "accel_2_A_i[01]", *(p_uint64++));
    err = nvs_set_u64(NVS_handle, "accel_2_A_i[02]", *(p_uint64++));
    err = nvs_set_u64(NVS_handle, "accel_2_A_i[10]", *(p_uint64++));
    err = nvs_set_u64(NVS_handle, "accel_2_A_i[11]", *(p_uint64++));
    err = nvs_set_u64(NVS_handle, "accel_2_A_i[12]", *(p_uint64++));
    err = nvs_set_u64(NVS_handle, "accel_2_A_i[20]", *(p_uint64++));
    err = nvs_set_u64(NVS_handle, "accel_2_A_i[21]", *(p_uint64++));
    err = nvs_set_u64(NVS_handle, "accel_2_A_i[22]", *(p_uint64++));
    printf("\n[Результат калибровки] = Ainv * ([Исх. вектор] - bias)\r\n");

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
