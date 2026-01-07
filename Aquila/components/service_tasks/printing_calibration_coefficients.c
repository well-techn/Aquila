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


extern const char *TAG_SERVICE;

void printing_calibration_coefficients (void *pvParameters)
{
  int16_t temp_int16 = 0;
  uint64_t temp_uint64 = 0;
  double* p_double;
#ifdef TELNET_CONF_MODE
  int16_t *client_fd = pvParameters;
  char message_to_print[150];
  uint8_t pos = 0;
#endif    
  esp_err_t err = nvs_flash_init();
  ESP_ERROR_CHECK( err );

  nvs_handle_t NVS_handle;
  err = nvs_open("storage", NVS_READWRITE, &NVS_handle);
  if (err != ESP_OK) 
  {
      ESP_LOGE(TAG_SERVICE,"Ошибка (%s) открытия NVS!\n", esp_err_to_name(err));
  } 
  else 
  {
//считываем и выводим на печать калибровочные коэффициенты гироскопов из флеш-памяти
   ESP_ERROR_CHECK(nvs_get_i16(NVS_handle, "gyro_1_off_0", &temp_int16)); 
   printf("Bias гироскопа 1 = [%d, ", temp_int16);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "Bias гироскопа 1 = [%d, ", temp_int16);
    send(*client_fd, message_to_print, pos, 0);
 #endif

   ESP_ERROR_CHECK(nvs_get_i16(NVS_handle, "gyro_1_off_1", &temp_int16));
   printf("%d, ", temp_int16);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%d, ", temp_int16);
    send(*client_fd, message_to_print, pos, 0);
#endif 

   ESP_ERROR_CHECK(nvs_get_i16(NVS_handle, "gyro_1_off_2", &temp_int16));
   printf("%d]\n", temp_int16); 
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%d]\r\n", temp_int16);
    send(*client_fd, message_to_print, pos, 0);
#endif 

   ESP_ERROR_CHECK(nvs_get_i16(NVS_handle, "gyro_2_off_0", &temp_int16));
   printf("Bias гироскопа 2 = [%d, ", temp_int16); 
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "Bias гироскопа 2 = [%d, ", temp_int16);
    send(*client_fd, message_to_print, pos, 0);
 #endif

   ESP_ERROR_CHECK(nvs_get_i16(NVS_handle, "gyro_2_off_1", &temp_int16));
   printf("%d, ", temp_int16);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%d, ", temp_int16);
    send(*client_fd, message_to_print, pos, 0);
#endif  

   ESP_ERROR_CHECK(nvs_get_i16(NVS_handle, "gyro_2_off_2", &temp_int16));
   printf("%d]\n\n", temp_int16);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%d]\r\n", temp_int16);
    send(*client_fd, message_to_print, pos, 0);
#endif  

//считываем и выводим на печать калибровочные коэффициенты по магнетто акселерометров из флеш-памяти
  
  printf ("Корректировочные значения сдвигов (bias) для акселерометра 1 = [");
  err = nvs_get_u64(NVS_handle, "accel_1_bias[0]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("%0.3f ", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "\nBias акселерометра 1 = [%0.3f ", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif   

  err = nvs_get_u64(NVS_handle, "accel_1_bias[1]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("%0.3f ", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f ", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif  

  err = nvs_get_u64(NVS_handle, "accel_1_bias[2]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("%0.3f]\n", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f]\r\n", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif   

  printf ("Корректирующая матрица Ainv\n"); 
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "Корректирующая матрица Ainv\r\n");
    send(*client_fd, message_to_print, pos, 0);
#endif 

  err = nvs_get_u64(NVS_handle, "accel_1_A_i[00]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("[%0.3f, ", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "[%0.3f, ", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif   

  err = nvs_get_u64(NVS_handle, "accel_1_A_i[01]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("%0.3f, ", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f, ", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif   

  err = nvs_get_u64(NVS_handle, "accel_1_A_i[02]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("%0.3f\n", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f\r\n", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif 

  err = nvs_get_u64(NVS_handle, "accel_1_A_i[10]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf(" %0.3f, ", *p_double); 
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f, ", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif 

  err = nvs_get_u64(NVS_handle, "accel_1_A_i[11]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("%0.3f, ", *p_double); 
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f, ", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif 

  err = nvs_get_u64(NVS_handle, "accel_1_A_i[12]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("%0.3f\n", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f\r\n", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif  

  err = nvs_get_u64(NVS_handle, "accel_1_A_i[20]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf(" %0.3f, ", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f, ", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif  

  err = nvs_get_u64(NVS_handle, "accel_1_A_i[21]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("%0.3f, ", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f, ", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif  

  err = nvs_get_u64(NVS_handle, "accel_1_A_i[22]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("%0.3f]\n\n", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f]\r\n", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif   

  printf ("Корректировочные значения сдвигов (bias) для акселерометра 2 = [");
  err = nvs_get_u64(NVS_handle, "accel_2_bias[0]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("%0.3f ", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "\nBias акселерометра 2 = [%0.3f ", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif   

  err = nvs_get_u64(NVS_handle, "accel_2_bias[1]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("%0.3f ", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f ", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif  

  err = nvs_get_u64(NVS_handle, "accel_2_bias[2]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("%0.3f]\n", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f]\r\n", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif   

 printf ("Корректирующая матрица Ainv\n"); 
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "Корректирующая матрица Ainv\r\n");
    send(*client_fd, message_to_print, pos, 0);
#endif 

  err = nvs_get_u64(NVS_handle, "accel_2_A_i[00]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("[%0.3f, ", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "[%0.3f, ", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif   

  err = nvs_get_u64(NVS_handle, "accel_2_A_i[01]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("%0.3f, ", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f, ", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif   

  err = nvs_get_u64(NVS_handle, "accel_2_A_i[02]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("%0.3f\n", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f\r\n", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif 

  err = nvs_get_u64(NVS_handle, "accel_2_A_i[10]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf(" %0.3f, ", *p_double); 
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f, ", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif 

  err = nvs_get_u64(NVS_handle, "accel_2_A_i[11]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("%0.3f, ", *p_double); 
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f, ", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif 

  err = nvs_get_u64(NVS_handle, "accel_2_A_i[12]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("%0.3f\n", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f\r\n", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif  

  err = nvs_get_u64(NVS_handle, "accel_2_A_i[20]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf(" %0.3f, ", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f, ", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif  

  err = nvs_get_u64(NVS_handle, "accel_2_A_i[21]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("%0.3f, ", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f, ", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif  

  err = nvs_get_u64(NVS_handle, "accel_2_A_i[22]", &temp_uint64); 
  p_double = (double*) &temp_uint64;
  printf("%0.3f]\n\n", *p_double);
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "%0.3f]\r\n", *p_double);
    send(*client_fd, message_to_print, pos, 0);
#endif   

  nvs_close(NVS_handle);
  vTaskDelete(NULL); 
  } 
}