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
#ifdef TELNET_CONF_MODE
  #include <lwip/sockets.h>
#endif

extern spi_device_handle_t MPU6000_1;
extern spi_device_handle_t MPU6000_2;
extern const char *TAG_SERVICE;

void advanced_acc_calibration(void *pvParameters)
{
    float input_data_1[NUMBER_OF_ACC_INPUTS][3];
    float input_data_2[NUMBER_OF_ACC_INPUTS][3];

    double *A_1, *B;
    uint16_t i;
    uint64_t* p_uint64;

    uint8_t sensor_data_1[20] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    uint8_t sensor_data_2[20] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
   
#ifdef TELNET_CONF_MODE

    char message_to_print[100];
    uint8_t pos = 0;
    int16_t *client_fd = pvParameters;
#endif

#ifdef TELNET_CONF_MODE
    send(*client_fd, "Проверка связи с MPU#1.....\r\n", sizeof("Проверка связи с MPU#1.....\r\n"), 0);
#endif
    ESP_LOGI(TAG_SERVICE, "Проверка связи с MPU#1.....");
    if (MPU6000_communication_check(MPU6000_1) != ESP_OK) vTaskDelete(NULL);

#ifdef TELNET_CONF_MODE
    send(*client_fd, "Проверка связи с MPU#2.....\r\n", sizeof("Проверка связи с MPU#2.....\r\n"), 0);
#endif
    ESP_LOGI(TAG_SERVICE, "Проверка связи с MPU#2.....");
    if (MPU6000_communication_check(MPU6000_2) != ESP_OK) vTaskDelete(NULL);

#ifdef TELNET_CONF_MODE
    send(*client_fd, "Настройка MPU#1.....\r\n", sizeof("Настройка MPU#1.....\r\n"), 0);
#endif
    ESP_LOGI(TAG_SERVICE, "Настройка MPU#1.....");
    if (MPU6000_init(MPU6000_1) != ESP_OK) vTaskDelete(NULL);

#ifdef TELNET_CONF_MODE
    send(*client_fd, "Настройка MPU#2.....\r\n", sizeof("Настройка MPU#2.....\r\n"), 0);
#endif
    ESP_LOGI(TAG_SERVICE, "Настройка MPU#2.....");
    if (MPU6000_init(MPU6000_2) != ESP_OK) vTaskDelete(NULL);

#ifdef TELNET_CONF_MODE
    send(*client_fd, "Перенастройка SPI на 20МГц.....\r\n", sizeof("Перенастройка SPI на 20МГц.....\r\n"), 0);
#endif
    ESP_LOGI(TAG_SERVICE, "Перенастройка SPI на 20МГц.....");
    SPI_change_MPUs_speed();
    ESP_LOGI(TAG_SERVICE, "Оба SPI перенастроены");
   
#ifdef TELNET_CONF_MODE
    send(*client_fd, "Начинаем калибровку IMU\r\n", sizeof("Начинаем калибровку IMU\r\n"), 0);
#endif
 ESP_LOGI(TAG_SERVICE, "Начинаем калибровку IMU");
    i = 0;

    while (i < NUMBER_OF_ACC_INPUTS)
    {
#ifdef TELNET_CONF_MODE
        pos = sprintf(message_to_print, "\r\nВектор #%d\r\n", i);
        send(*client_fd, message_to_print, pos, 0);        
#endif
        printf("Вектор #%d\n", i);
        // обратный отсчет
        for (uint8_t j = 5; j > 0; j--)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
#ifdef TELNET_CONF_MODE
        pos = sprintf(message_to_print, "%d ", j);
        send(*client_fd, message_to_print, pos, 0);        
#endif
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
#ifdef TELNET_CONF_MODE
        send(*client_fd, "Сохранен\r\n", sizeof("Сохранен\r\n"), 0);   
#endif
    }

    printf("Записанный массив данных\n\n");
#ifdef TELNET_CONF_MODE
        send(*client_fd, "Записанный массив данных\r\n", sizeof("Записанный массив данных\r\n"), 0);   
#endif
    for (int i = 0; i < NUMBER_OF_ACC_INPUTS; i++)
        {
            printf("{%0.1f, %0.1f, %0.1f},  {%0.1f, %0.1f, %0.1f},\n", input_data_1[i][0], input_data_1[i][1], input_data_1[i][2],
               input_data_2[i][0], input_data_2[i][1], input_data_2[i][2]);
#ifdef TELNET_CONF_MODE
        pos = sprintf(message_to_print, "{%0.1f, %0.1f, %0.1f},  {%0.1f, %0.1f, %0.1f},\r\n", input_data_1[i][0], input_data_1[i][1], input_data_1[i][2],
                                            input_data_2[i][0], input_data_2[i][1], input_data_2[i][2]);
        send(*client_fd, message_to_print, pos, 0);        
#endif            
        }

    //выделяем память под матицы для расчетов для акселерометра 1
    A_1 = (double *)malloc(3 * 3 * sizeof(double));
    B = (double *)malloc(3 * sizeof(double));

    // вычисляем корректировочные параметры для акселерометра 1
    printf("\nНачинаем расчет корректировочных коэффициентов для акселерометра 1\n");
#ifdef TELNET_CONF_MODE
        send(*client_fd, "\nНачинаем расчет корректировочных коэффициентов для акселерометра 1\n", sizeof("\nНачинаем расчет корректировочных коэффициентов для акселерометра 1\n"), 0);   
#endif
    calculation_B_and_Ainv_with_exclusion(input_data_1, A_1, B, 8192, 0, NUMBER_OF_ACC_INPUTS);
#ifdef TELNET_CONF_MODE
        send(*client_fd, "\r\nКорректировочные значения сдвигов (bias):\r\n", sizeof("\r\nКорректировочные значения сдвигов (bias):\r\n"), 0);
        pos = sprintf(message_to_print, "%8.6lf %8.6lf %8.6lf \r\n", B[0], B[1], B[2]);
        send(*client_fd, message_to_print, pos, 0);
        
       
        send(*client_fd, "\r\nКорректирующая матрица Ainv\r\n", sizeof("\r\nКорректирующая матрица Ainv\r\n"), 0);
            for (uint8_t i = 0; i < 3; i++)
            {
               pos = sprintf(message_to_print, "%9.6lf %9.6lf %9.6lf\r\n", A_1[i * 3], A_1[i * 3 + 1], A_1[i * 3 + 2]);
               send(*client_fd, message_to_print, pos, 0); 
            }
#endif

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
#ifdef TELNET_CONF_MODE
        send(*client_fd, "\nНачинаем расчет корректировочных коэффициентов для акселерометра 2\n", sizeof("\nНачинаем расчет корректировочных коэффициентов для акселерометра 2\n"), 0);   
#endif
    calculation_B_and_Ainv_with_exclusion(input_data_2, A_1, B, 8192, 0, NUMBER_OF_ACC_INPUTS);

    #ifdef TELNET_CONF_MODE
        send(*client_fd, "\r\nКорректировочные значения сдвигов (bias):\r\n", sizeof("\r\nКорректировочные значения сдвигов (bias):\r\n"), 0);
        pos = sprintf(message_to_print, "%8.6lf %8.6lf %8.6lf \r\n", B[0], B[1], B[2]);
        send(*client_fd, message_to_print, pos, 0);
        
       
        send(*client_fd, "\r\nКорректирующая матрица Ainv\r\n", sizeof("\r\nКорректирующая матрица Ainv\r\n"), 0);
            for (uint8_t i = 0; i < 3; i++)
            {
               pos = sprintf(message_to_print, "%9.6lf %9.6lf %9.6lf\r\n", A_1[i * 3], A_1[i * 3 + 1], A_1[i * 3 + 2]);
               send(*client_fd, message_to_print, pos, 0); 
            }
#endif

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
#ifdef TELNET_CONF_MODE
        send(*client_fd, "\r\nСохраняем данные в NVS ...\r\n", sizeof("\r\nСохраняем данные в NVS ...\r\n"), 0);   
#endif
    err = nvs_commit(NVS_handle);
    if (err == ESP_OK) 
    {
        printf("Данные сохранены\n");
#ifdef TELNET_CONF_MODE
        send(*client_fd, "Данные сохранены\r\n", sizeof("Данные сохранены\r\n"), 0);   
#endif
    }
        else 
        {
            printf("Ошибка сохранения данных!\n");
#ifdef TELNET_CONF_MODE
        send(*client_fd, "Ошибка сохранения данных!\r\n", sizeof("Ошибка сохранения данных!\r\n"), 0);   
#endif
        }
    nvs_close(NVS_handle);
  
    printf("Для перезапуска нажмите ESC\r\n");
#ifdef TELNET_CONF_MODE
        send(*client_fd, "Для перезапуска нажмите ESC (придется переподключиться к WiFi)\r\n", sizeof("\r\nДля перезапуска нажмите ESC (придется переподключиться к WiFi)\r\n"), 0);   
#endif
    free(A_1);
    free(B);

    vTaskDelete(NULL);
}
