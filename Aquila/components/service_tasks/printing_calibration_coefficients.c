#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "string.h"
#include "NVS_operations.h"

#include "logging_operations.h"

void printing_calibration_coefficients (void *pvParameters)
{
  nvs_handle_t coeff_NVS_handle;

  float gyro_1_offset[3] = {0};
  float gyro_2_offset[3] = {0};

  float accel_1_bias[3] = {0};
  float accel_1_A_inv[9] = {0};
  
  float accel_2_bias[3] = {0};
  float accel_2_A_inv[9] = {0};
  float mag_hard_bias[3] = {0};     
  
  float mag_A_inv[9] = {0};
  
  int16_t client_fd = (pvParameters) ? *(int16_t *)pvParameters : -1;

//инициализируем и открываем NVS хранения калибровочных коэффициентов  
  ESP_ERROR_CHECK(NVS_prepare(&coeff_NVS_handle, "coeff_storage"));

//считываем массив оффсетов гироскопов 1 и 2
  nvs_read_float_array(coeff_NVS_handle, "gyro_1_off", gyro_1_offset, 3);
  nvs_read_float_array(coeff_NVS_handle, "gyro_2_off", gyro_2_offset, 3);

//считываем массивы оффсетов акселерометров 1 и 2
  nvs_read_float_array(coeff_NVS_handle, "accel_1_off", accel_1_bias, 3);
  nvs_read_float_array(coeff_NVS_handle, "accel_2_off", accel_2_bias, 3);

//считываем массивы Ainv акселерометров 1 и 2
  nvs_read_float_array(coeff_NVS_handle, "accel_1_Ai", accel_1_A_inv, 9);
  nvs_read_float_array(coeff_NVS_handle, "accel_2_Ai", accel_2_A_inv, 9);

//считываем массив hard bias магнетометра
  nvs_read_float_array(coeff_NVS_handle, "mag_h_bias", mag_hard_bias, 3);

//считываем массив Ainv магнетометра
  nvs_read_float_array(coeff_NVS_handle, "mag_Ai", mag_A_inv, 9);

//выводим данные на печать
print_service_message(client_fd, "Bias гироскопа 1 = [%0.6f, %0.6f, %0.6f]\r\n", gyro_1_offset[0], gyro_1_offset[1], gyro_1_offset[2]);
print_service_message(client_fd, "Bias гироскопа 2 = [%0.6f, %0.6f, %0.6f]\r\n\r\n", gyro_2_offset[0], gyro_2_offset[1], gyro_2_offset[2]);

print_service_message(client_fd, "Корректировочные значения сдвигов (bias) для акселерометра 1 = [%0.6f, %0.6f, %0.6f]\r\n",accel_1_bias[0], accel_1_bias[1], accel_1_bias[2]);

print_service_message(client_fd, "Корректирующая матрица Ainv\r\n");
print_service_message(client_fd, "[%0.6f, %0.6f, %0.6f\r\n",accel_1_A_inv[0], accel_1_A_inv[1], accel_1_A_inv[2]);
print_service_message(client_fd, " %0.6f, %0.6f, %0.6f\r\n",accel_1_A_inv[3], accel_1_A_inv[4], accel_1_A_inv[5]);
print_service_message(client_fd, " %0.6f, %0.6f, %0.6f]\r\n\r\n",accel_1_A_inv[6], accel_1_A_inv[7], accel_1_A_inv[8]);

print_service_message(client_fd, "Корректировочные значения сдвигов (bias) для акселерометра 2 = [%0.6f, %0.6f, %0.6f]\r\n",accel_2_bias[0], accel_2_bias[1], accel_2_bias[2]);

print_service_message(client_fd, "Корректирующая матрица Ainv\r\n"); 
print_service_message(client_fd, "[%0.6f, %0.6f, %0.6f\r\n",accel_2_A_inv[0], accel_2_A_inv[1], accel_2_A_inv[2]);
print_service_message(client_fd, " %0.6f, %0.6f, %0.6f\r\n",accel_2_A_inv[3], accel_2_A_inv[4], accel_2_A_inv[5]);
print_service_message(client_fd, " %0.6f, %0.6f, %0.6f]\r\n\r\n",accel_2_A_inv[6], accel_2_A_inv[7], accel_2_A_inv[8]);

print_service_message(client_fd, "Корректировочные значения сдвигов (bias) для магнетометра = [%0.6f, %0.6f, %0.6f]\r\n",mag_hard_bias[0], mag_hard_bias[1], mag_hard_bias[2]);

print_service_message(client_fd, "Корректирующая матрица Ainv\r\n"); 
print_service_message(client_fd, "[%0.6f, %0.6f, %0.6f\r\n",mag_A_inv[0], mag_A_inv[1], mag_A_inv[2]);
print_service_message(client_fd, " %0.6f, %0.6f, %0.6f\r\n",mag_A_inv[3], mag_A_inv[4], mag_A_inv[5]);
print_service_message(client_fd, " %0.6f, %0.6f, %0.6f]\r\n\r\n",mag_A_inv[6], mag_A_inv[7], mag_A_inv[8]);

  nvs_close(coeff_NVS_handle);

  print_service_message(client_fd, "Вывод коэффициентов завершен, ESC для перезапуска или повторите выбор меню\r\n"); 
  vTaskDelete(NULL); 
} 
