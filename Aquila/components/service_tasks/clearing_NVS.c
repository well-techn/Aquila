#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "MPU6000.h"
#include "wt_alldef.h"
#include "wt_spi.h"
#include <rom/ets_sys.h>
#include "math.h"
#include "string.h"

#include "logging_operations.h"

void clearing_NVS(void *pvParameters)
{
  int16_t client_fd = (pvParameters) ? *(int16_t *)pvParameters : -1;
  
  esp_err_t err = nvs_flash_init();
  ESP_ERROR_CHECK( err );

  nvs_handle_t coeff_NVS_handle;
  err = nvs_open("coeff_storage", NVS_READWRITE, &coeff_NVS_handle);
  if (err != ESP_OK) 
  {
      //ESP_LOGE(TAG_SERVICE,"Ошибка (%s) открытия NVS!\n", esp_err_to_name(err));
      print_service_message(client_fd, "Ошибка (%s) открытия NVS!\r\n", esp_err_to_name(err));
  } 
  else 
  {
     ESP_ERROR_CHECK(nvs_erase_all(coeff_NVS_handle));
     nvs_commit(coeff_NVS_handle); 
     print_service_message(client_fd, "NVS очищен\r\n");
  }

  nvs_close(coeff_NVS_handle);
  vTaskDelete(NULL); 
} 
