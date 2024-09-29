#include "IST8310.h"
#include "ve_i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

extern i2c_master_dev_handle_t IST8310_dev_handle;
extern i2c_master_bus_handle_t i2c_external_bus_handle;
extern char *TAG_IST8310;

esp_err_t IST8310_communication_check()
{
  esp_err_t err = ESP_FAIL;

  err = i2c_master_probe(i2c_external_bus_handle, IST8310_ADDR, -1);
  if (err == ESP_OK)  ESP_LOGI(TAG_IST8310,"IST8310 is online");
  else ESP_LOGE(TAG_IST8310,"IST8310 is offline\n");

  return err;
}

esp_err_t IST8310_selftest()
{
  esp_err_t err = ESP_FAIL;

  uint8_t normal_reads[6];
  uint8_t inversed_reads[6];
  int16_t normal_values[3] = {0,0,0};
  int16_t inversed_values[3] = {0,0,0};
  uint8_t i = 0;

  //i2c_write_byte_to_address_NEW(i2c_internal_bus_handle, IST8310_CNTL2_REG, IST8310_CNTL2_VAL_SRST);

  i2c_write_byte_to_address_NEW(IST8310_dev_handle, IST8310_TCCNTL_REG, 0x01);      //disabling temp compensation
  i2c_write_byte_to_address_NEW(IST8310_dev_handle, IST8310_CNTL1_REG, IST8310_CNTL1_VAL_SINGLE_MEASUREMENT_MODE);      //enabling single measurenebt mode
  vTaskDelay(10/portTICK_PERIOD_MS);

  i2c_read_bytes_from_address_NEW(IST8310_dev_handle, IST8310_OUTPUT_X_L_REG, 6, normal_reads);     //read_normal values
  for (i = 0; i < 6; i++) printf("%d",normal_reads[i]);

  for (i=0;i<3;i++) normal_values[i] = (normal_reads[i] << 8) | normal_reads[i+1];
  

  i2c_write_byte_to_address_NEW(IST8310_dev_handle, IST8310_SELF_TEST_REG, 0x40);      //enabling axis inversion
  i2c_write_byte_to_address_NEW(IST8310_dev_handle, IST8310_CNTL1_REG, IST8310_CNTL1_VAL_SINGLE_MEASUREMENT_MODE);      //enabling single measurenebt mode

  vTaskDelay(10/portTICK_PERIOD_MS);

  i2c_read_bytes_from_address_NEW(IST8310_dev_handle, IST8310_OUTPUT_X_L_REG, 6, inversed_reads);     //read inversed values
  for (i = 0; i < 6; i++) printf("%d",inversed_reads[i]);

  for (i=0;i<3;i++) inversed_values[i] = (inversed_reads[i] << 8) | inversed_reads[i+1];

 
    if (abs(normal_values[0]) == abs(inversed_values[0]) && 
        abs(normal_values[1]) == abs(inversed_values[1]) &&
        abs(normal_values[2]) == abs(inversed_values[2])) err = ESP_OK;


  i2c_write_byte_to_address_NEW(IST8310_dev_handle, IST8310_SELF_TEST_REG, 0x40);      //disabling axis inversion
  i2c_write_byte_to_address_NEW(IST8310_dev_handle, IST8310_TCCNTL_REG, 0x00);      //enabling temp compensation

  if (err == ESP_OK)  ESP_LOGI(TAG_IST8310,"IST8310 self test passed");
  else ESP_LOGE(TAG_IST8310,"IST8310 self test failed\n");

  return err;
}

