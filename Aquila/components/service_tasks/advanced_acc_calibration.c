#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "MPU6000.h"
#include "wt_alldef.h"
#include "wt_spi.h"
#include <rom/ets_sys.h>
#include "math.h"
#include "string.h"
#include "advanced_math.h"
#include "inttypes.h"
#include "nvs.h"

#include "logging_operations.h"

extern spi_device_handle_t MPU6000_1;
extern spi_device_handle_t MPU6000_2;
extern const char *TAG_SERVICE;

void advanced_acc_calibration(void *pvParameters)
{
    float input_data_1[NUMBER_OF_ACC_INPUTS][3];
    float input_data_2[NUMBER_OF_ACC_INPUTS][3];

    double *A_1, *B;
    float  *A_1_f, *B_f;
    uint32_t* p_uint32;

    uint8_t sensor_data_1[20] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    uint8_t sensor_data_2[20] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};

    int16_t client_fd = (pvParameters) ? *(int16_t *)pvParameters : -1;

    print_service_message(client_fd, "Проверка связи с MPU#1.....\r\n");
    if (MPU6000_communication_check(MPU6000_1) != ESP_OK) vTaskDelete(NULL);

    print_service_message(client_fd, "Проверка связи с MPU#2.....\r\n");
    if (MPU6000_communication_check(MPU6000_2) != ESP_OK) vTaskDelete(NULL);

    print_service_message(client_fd, "Настройка MPU#1.....\r\n");
    if (MPU6000_init(MPU6000_1,
                   IMU_SAMPLING_FREQ_HZ,
                   IMU_HW_LPF_CUTOFF_HZ,
                   IMU_ACCEL_FULL_SCALE_G,
                   IMU_GYRO_FULL_SCALE_DPS)  != ESP_OK) vTaskDelete(NULL);

    print_service_message(client_fd, "Настройка MPU#2.....\r\n");
    if (MPU6000_init(MPU6000_2,
                IMU_SAMPLING_FREQ_HZ,
                IMU_HW_LPF_CUTOFF_HZ,
                IMU_ACCEL_FULL_SCALE_G,
                IMU_GYRO_FULL_SCALE_DPS)  != ESP_OK) vTaskDelete(NULL);

    print_service_message(client_fd, "Перенастройка SPI на 20МГц.....\r\n");
    SPI_change_MPUs_speed();
    print_service_message(client_fd, "Оба SPI перенастроены\r\n");
   
    print_service_message(client_fd, "Начинаем калибровку акселерометров\r\n");

    for (uint16_t i = 0; i<NUMBER_OF_ACC_INPUTS; i++)
    {
        print_service_message(client_fd, "Вектор #%d\r\n", i);
//обратный отсчет чтобы успеть поставить статично
        for (uint8_t j = 3; j > 0; j--)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            print_service_message(client_fd, "%d ", j);
        }
//считываем показания
        SPI_read_bytes(MPU6000_1, 0, 0, 8, MPU6000_ACCEL_XOUT_H | SPI_READ_FLAG, 0, sensor_data_1, 14);
        SPI_read_bytes(MPU6000_2, 0, 0, 8, MPU6000_ACCEL_XOUT_H | SPI_READ_FLAG, 0, sensor_data_2, 14);

//формируем "сырые" показания акселерометров
        input_data_1[i][0] = (int16_t)((sensor_data_1[0] << 8) | sensor_data_1[1]); // X
        input_data_1[i][1] = (int16_t)((sensor_data_1[2] << 8) | sensor_data_1[3]); // Y
        input_data_1[i][2] = (int16_t)((sensor_data_1[4] << 8) | sensor_data_1[5]); // Z

        input_data_2[i][0] = (int16_t)((sensor_data_2[0] << 8) | sensor_data_2[1]); // X
        input_data_2[i][1] = (int16_t)((sensor_data_2[2] << 8) | sensor_data_2[3]); // Y
        input_data_2[i][2] = (int16_t)((sensor_data_2[4] << 8) | sensor_data_2[5]); // Z

        print_service_message(client_fd, "сохранен\r\n");
    }
//выводим на всякий случай записанный массив данных    
        print_service_message(client_fd, "Записанный массив данных\r\n");

    for (uint16_t i = 0; i < NUMBER_OF_ACC_INPUTS; i++)
        {
        print_service_message(client_fd, "IMU1: {%0.1f, %0.1f, %0.1f}, IMU2: {%0.1f, %0.1f, %0.1f},\r\n", 
                                input_data_1[i][0], input_data_1[i][1], input_data_1[i][2],
                                input_data_2[i][0], input_data_2[i][1], input_data_2[i][2]);
        }

//выделяем память под матицы для расчетов для акселерометра 1
    A_1 = (double *)malloc(3 * 3 * sizeof(double));
    B = (double *)malloc(3 * sizeof(double));

//вычисляем корректировочные параметры для акселерометра 1
    print_service_message(client_fd, "\nНачинаем расчет корректировочных коэффициентов для акселерометра 1\n");
    calculation_B_and_Ainv_with_exclusion(input_data_1, A_1, B, IMU_ACCEL_BITS_PER_G, 0, NUMBER_OF_ACC_INPUTS);

    print_service_message(client_fd,"\r\nКорректировочные значения сдвигов (offset):\r\n");
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
    ESP_ERROR_CHECK(err);
    nvs_handle_t coeff_NVS_handle;

    err = nvs_open("coeff_storage", NVS_READWRITE, &coeff_NVS_handle);
    if (err != ESP_OK) print_service_message(client_fd,"Ошибка (%s) открытия NVS!\n", esp_err_to_name(err));
    else 
    {
//Записываем accel_1_offset[0]...accel_1_offset[2]
        p_uint32 = (uint32_t*)&B_f[0];
        err = nvs_set_u32(coeff_NVS_handle, "accel_1_off[0]", *(p_uint32++));
        err = nvs_set_u32(coeff_NVS_handle, "accel_1_off[1]", *(p_uint32++));
        err = nvs_set_u32(coeff_NVS_handle, "accel_1_off[2]", *(p_uint32++));

//Записываем accel_1_A_inv[00]...accel_1_A_inv[33] 
        p_uint32 = (uint32_t*)&A_1_f[0]; 
        err = nvs_set_u32(coeff_NVS_handle, "accel_1_Ai[0]", *(p_uint32++));
        err = nvs_set_u32(coeff_NVS_handle, "accel_1_Ai[1]", *(p_uint32++));
        err = nvs_set_u32(coeff_NVS_handle, "accel_1_Ai[2]", *(p_uint32++));
        err = nvs_set_u32(coeff_NVS_handle, "accel_1_Ai[3]", *(p_uint32++));
        err = nvs_set_u32(coeff_NVS_handle, "accel_1_Ai[4]", *(p_uint32++));
        err = nvs_set_u32(coeff_NVS_handle, "accel_1_Ai[5]", *(p_uint32++));
        err = nvs_set_u32(coeff_NVS_handle, "accel_1_Ai[6]", *(p_uint32++));
        err = nvs_set_u32(coeff_NVS_handle, "accel_1_Ai[7]", *(p_uint32++));
        err = nvs_set_u32(coeff_NVS_handle, "accel_1_Ai[8]", *(p_uint32++));
    }
    
//вычисляем корректировочные параметры для акселерометра 2
    print_service_message(client_fd, "\nНачинаем расчет корректировочных коэффициентов для акселерометра 2\n");
    calculation_B_and_Ainv_with_exclusion(input_data_2, A_1, B, IMU_ACCEL_BITS_PER_G, 0, NUMBER_OF_ACC_INPUTS);

    print_service_message(client_fd,"\r\nКорректировочные значения сдвигов (offset):\r\n");
    print_service_message(client_fd,"%8.6lf %8.6lf %8.6lf\r\n", B[0], B[1], B[2]);
    print_service_message(client_fd,"\r\nКорректирующая матрица Ainv\r\n");
    for (uint8_t i = 0; i < 3; i++)
    {
        print_service_message(client_fd,"%9.6lf %9.6lf %9.6lf\r\n", A_1[i * 3], A_1[i * 3 + 1], A_1[i * 3 + 2]);
    } 

//переводим матрицы во float 
    for (uint8_t i = 0; i<9; i++) A_1_f[i] = (float)A_1[i];
    for (uint8_t i = 0; i<3; i++) B_f[i] = (float)B[i];

//Записываем accel_2_offset[0]...accel_2_offset[2]
    p_uint32 = (uint32_t*)&B_f[0];
    err = nvs_set_u32(coeff_NVS_handle, "accel_2_off[0]", *(p_uint32++));
    err = nvs_set_u32(coeff_NVS_handle, "accel_2_off[1]", *(p_uint32++));
    err = nvs_set_u32(coeff_NVS_handle, "accel_2_off[2]", *(p_uint32++));

//Записываем accel_2_A_inv[00]...accel_2_A_inv[33] 
    p_uint32 = (uint32_t*)&A_1_f[0];
    err = nvs_set_u32(coeff_NVS_handle, "accel_2_Ai[0]", *(p_uint32++));
    err = nvs_set_u32(coeff_NVS_handle, "accel_2_Ai[1]", *(p_uint32++));
    err = nvs_set_u32(coeff_NVS_handle, "accel_2_Ai[2]", *(p_uint32++));
    err = nvs_set_u32(coeff_NVS_handle, "accel_2_Ai[3]", *(p_uint32++));
    err = nvs_set_u32(coeff_NVS_handle, "accel_2_Ai[4]", *(p_uint32++));
    err = nvs_set_u32(coeff_NVS_handle, "accel_2_Ai[5]", *(p_uint32++));
    err = nvs_set_u32(coeff_NVS_handle, "accel_2_Ai[6]", *(p_uint32++));
    err = nvs_set_u32(coeff_NVS_handle, "accel_2_Ai[7]", *(p_uint32++));
    err = nvs_set_u32(coeff_NVS_handle, "accel_2_Ai[8]", *(p_uint32++));

    print_service_message(client_fd,"\n[Результат калибровки] = Ainv * ([Исх. вектор] - offset)\r\n");

    print_service_message(client_fd,"Сохраняем данные в NVS ... ");
    err = nvs_commit(coeff_NVS_handle);
    if (err == ESP_OK) print_service_message(client_fd,"Данные сохранены\r\n");
        else print_service_message(client_fd,"Ошибка сохранения данных");
      
    nvs_close(coeff_NVS_handle);
//освобождаем память
    free(A_1);
    free(B);
    free(A_1_f);
    free(B_f);
  
    print_service_message(client_fd, "Калибровка акселерометров завершена, ESC для перезапуска или повторите выбор меню\r\n"); 
//убиваем задачу
    vTaskDelete(NULL);
}
