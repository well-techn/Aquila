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

void clearing_NVS(void *pvParameters)
{
#ifdef TELNET_CONF_MODE
  int16_t *client_fd = pvParameters;
  char message_to_print[150];
  uint8_t pos = 0;
#endif    
  
esp_err_t err = nvs_flash_init();
  ESP_ERROR_CHECK( err );

  nvs_handle_t coeff_NVS_handle;
  err = nvs_open("coeff_storage", NVS_READWRITE, &coeff_NVS_handle);
  if (err != ESP_OK) 
  {
      ESP_LOGE(TAG_SERVICE,"Ошибка (%s) открытия NVS!\n", esp_err_to_name(err));
  } 
  else 
  {
     ESP_ERROR_CHECK(nvs_erase_all(coeff_NVS_handle));
     nvs_commit(coeff_NVS_handle); 
     printf("NVS очищен\n\n");
  }
#ifdef TELNET_CONF_MODE
    pos = sprintf(message_to_print, "NVS очищен\r\n");
    send(*client_fd, message_to_print, pos, 0);
#endif   

  nvs_close(coeff_NVS_handle);
  vTaskDelete(NULL); 
} 
