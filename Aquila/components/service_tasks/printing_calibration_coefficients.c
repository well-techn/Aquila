#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "MPU6000.h"
#include "esp_log.h"
#include "wt_alldef.h"
#include "wt_spi.h"
#include <rom/ets_sys.h>
#include "math.h"
#include "string.h"
#include "NVS_operations.h"
#ifdef TELNET_CONF_MODE
  #include <lwip/sockets.h>
#endif

extern const char *TAG_SERVICE;

void printing_calibration_coefficients (void *pvParameters)
{
  nvs_handle_t coeff_NVS_handle;

  int16_t gyro_1_offset[3] = {0,0,0};
  int16_t gyro_2_offset[3] = {0,0,0};

  double accel_1_bias[3] = {0,0,0};
  double accel_1_A_inv[3][3] = {{0,0,0},
                                {0,0,0},
                                {0,0,0}};
  
  double accel_2_bias[3] = {0,0,0};
  double accel_2_A_inv[3][3] = {{0,0,0},
                                {0,0,0},
                                {0,0,0}};
  double mag_hard_bias[3] = {0,0,0};     
  
  double mag_A_inv[3][3] = {{0,  0,  0},    
                            {0,  0,  0},
                            {0,  0,  0}};   

#ifdef TELNET_CONF_MODE
  int16_t *client_fd = pvParameters;
  char message_to_print[256];
  uint8_t pos = 0;
#endif

//инициализируем и открываем NVS хранения калибровочных коэффициентов  
  ESP_ERROR_CHECK(NVS_prepare(&coeff_NVS_handle, "coeff_storage"));

//считываем массив оффсетов гироскопов 1 и 2
  nvs_read_i16_array(coeff_NVS_handle, "gyro_1_off", gyro_1_offset, 3);
  nvs_read_i16_array(coeff_NVS_handle, "gyro_2_off", gyro_2_offset, 3);

//считываем массивы оффсетов акселерометров 1 и 2
  nvs_read_double_array(coeff_NVS_handle, "accel_1_off", (double*)accel_1_bias, 3);
  nvs_read_double_array(coeff_NVS_handle, "accel_2_off", (double*)accel_2_bias, 3);

//считываем массивы Ainv акселерометров 1 и 2
  nvs_read_double_array(coeff_NVS_handle, "accel_1_Ai", (double*)accel_1_A_inv, 9);
  nvs_read_double_array(coeff_NVS_handle, "accel_2_Ai", (double*)accel_2_A_inv, 9);

//считываем массив hard bias магнетометра
  nvs_read_double_array(coeff_NVS_handle, "mag_h_bias", (double*)mag_hard_bias, 3);

//считываем массив Ainv магнетометра
  nvs_read_double_array(coeff_NVS_handle, "mag_Ai", (double*)mag_A_inv, 9);

//выводим данные на печать
pos = snprintf(message_to_print, sizeof(message_to_print), "Bias гироскопа 1 = [%d, %d, %d]\r\n", gyro_1_offset[0], gyro_1_offset[1], gyro_1_offset[2]);
printf("%s", message_to_print);
#ifdef TELNET_CONF_MODE
        send(*client_fd, message_to_print, pos, 0);
#endif

pos = snprintf(message_to_print, sizeof(message_to_print), "Bias гироскопа 2 = [%d, %d, %d]\r\n\r\n", gyro_2_offset[0], gyro_2_offset[1], gyro_2_offset[2]);
printf("%s", message_to_print);
#ifdef TELNET_CONF_MODE 
        send(*client_fd, message_to_print, pos, 0); 
#endif

pos = snprintf(message_to_print, sizeof(message_to_print), "Корректировочные значения сдвигов (bias) для акселерометра 1 = [%0.3f, %0.3f, %0.3f]\r\n",accel_1_bias[0], accel_1_bias[1], accel_1_bias[2]);
printf("%s", message_to_print);
#ifdef TELNET_CONF_MODE
        send(*client_fd, message_to_print, pos, 0);
#endif

pos = snprintf(message_to_print, sizeof(message_to_print), "Корректирующая матрица Ainv\r\n");
printf("%s", message_to_print);
#ifdef TELNET_CONF_MODE
        send(*client_fd, message_to_print, pos, 0);
#endif

pos = snprintf(message_to_print, sizeof(message_to_print), "[%0.3f, %0.3f, %0.3f\r\n",accel_1_A_inv[0][0], accel_1_A_inv[0][1], accel_1_A_inv[0][2]);
printf("%s", message_to_print);
#ifdef TELNET_CONF_MODE
        send(*client_fd, message_to_print, pos, 0);
#endif

pos = snprintf(message_to_print, sizeof(message_to_print), " %0.3f, %0.3f, %0.3f\r\n",accel_1_A_inv[1][0], accel_1_A_inv[1][1], accel_1_A_inv[1][2]);
printf("%s", message_to_print);
#ifdef TELNET_CONF_MODE
        send(*client_fd, message_to_print, pos, 0);
#endif

pos = snprintf(message_to_print, sizeof(message_to_print), " %0.3f, %0.3f, %0.3f]\r\n\r\n",accel_1_A_inv[2][0], accel_1_A_inv[2][1], accel_1_A_inv[2][2]);
printf("%s", message_to_print);
#ifdef TELNET_CONF_MODE
        send(*client_fd, message_to_print, pos, 0);
#endif


pos = snprintf(message_to_print, sizeof(message_to_print), "Корректировочные значения сдвигов (bias) для акселерометра 2 = [%0.3f, %0.3f, %0.3f]\r\n",accel_2_bias[0], accel_2_bias[1], accel_2_bias[2]);
printf("%s", message_to_print);
#ifdef TELNET_CONF_MODE
        send(*client_fd, message_to_print, pos, 0);
#endif

pos = snprintf(message_to_print, sizeof(message_to_print), "Корректирующая матрица Ainv\r\n"); 
printf("%s", message_to_print);
#ifdef TELNET_CONF_MODE
        send(*client_fd, message_to_print, pos, 0);
#endif

pos = snprintf(message_to_print, sizeof(message_to_print), "[%0.3f, %0.3f, %0.3f\r\n",accel_2_A_inv[0][0], accel_2_A_inv[0][1], accel_2_A_inv[0][2]);
printf("%s", message_to_print);
#ifdef TELNET_CONF_MODE
        send(*client_fd, message_to_print, pos, 0);
#endif

pos = snprintf(message_to_print, sizeof(message_to_print), " %0.3f, %0.3f, %0.3f\r\n",accel_2_A_inv[1][0], accel_2_A_inv[1][1], accel_2_A_inv[1][2]);
printf("%s", message_to_print);
#ifdef TELNET_CONF_MODE
        send(*client_fd, message_to_print, pos, 0);
#endif

pos = snprintf(message_to_print, sizeof(message_to_print), " %0.3f, %0.3f, %0.3f]\r\n\r\n",accel_2_A_inv[2][0], accel_2_A_inv[2][1], accel_2_A_inv[2][2]);
printf("%s", message_to_print);
#ifdef TELNET_CONF_MODE
        send(*client_fd, message_to_print, pos, 0);
#endif


pos = snprintf(message_to_print, sizeof(message_to_print), "Корректировочные значения сдвигов (bias) для магнетометра = [%0.3f, %0.3f, %0.3f]\r\n",mag_hard_bias[0], mag_hard_bias[1], mag_hard_bias[2]);
printf("%s", message_to_print);
#ifdef TELNET_CONF_MODE
        send(*client_fd, message_to_print, pos, 0);
#endif

pos = snprintf(message_to_print, sizeof(message_to_print), "Корректирующая матрица Ainv\r\n"); 
printf("%s", message_to_print);
#ifdef TELNET_CONF_MODE
        send(*client_fd, message_to_print, pos, 0);
#endif

pos = snprintf(message_to_print, sizeof(message_to_print), "[%0.3f, %0.3f, %0.3f\r\n",mag_A_inv[0][0], mag_A_inv[0][1], mag_A_inv[0][2]);
printf("%s", message_to_print);
#ifdef TELNET_CONF_MODE
        send(*client_fd, message_to_print, pos, 0);
#endif

pos = snprintf(message_to_print, sizeof(message_to_print), " %0.3f, %0.3f, %0.3f\r\n",mag_A_inv[1][0], mag_A_inv[1][1], mag_A_inv[1][2]);
printf("%s", message_to_print);
#ifdef TELNET_CONF_MODE
        send(*client_fd, message_to_print, pos, 0);
#endif

pos = snprintf(message_to_print, sizeof(message_to_print), " %0.3f, %0.3f, %0.3f]\r\n\r\n",mag_A_inv[2][0], mag_A_inv[2][1], mag_A_inv[2][2]);
printf("%s", message_to_print);
#ifdef TELNET_CONF_MODE
        send(*client_fd, message_to_print, pos, 0);
#endif

  nvs_close(coeff_NVS_handle);
  vTaskDelete(NULL); 
} 
