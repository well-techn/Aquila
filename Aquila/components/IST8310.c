#include "IST8310.h"
#include "ve_i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <rom/ets_sys.h>

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

  uint8_t mag_raw_values[6];
  int16_t normal_values[3] = {0,0,0};
  int16_t inversed_values[3] = {0,0,0};

  i2c_write_byte_to_address_NEW(IST8310_dev_handle, IST8310_SELF_TEST_REG, 0x00);      //disabling axis inversion
  i2c_write_byte_to_address_NEW(IST8310_dev_handle, IST8310_TCCNTL_REG, 0x01);      //disabling temp compensation
  i2c_write_byte_to_address_NEW(IST8310_dev_handle, IST8310_CNTL1_REG, IST8310_CNTL1_VAL_SINGLE_MEASUREMENT_MODE);      //enabling single measurenebt mode
  vTaskDelay(10/portTICK_PERIOD_MS);

  i2c_read_bytes_from_address_NEW(IST8310_dev_handle, IST8310_OUTPUT_X_L_REG, 6, mag_raw_values);     //read_normal values

  normal_values[0] = (mag_raw_values[1] << 8) | mag_raw_values[0]; //X
  normal_values[1] = (mag_raw_values[3] << 8) | mag_raw_values[2]; //Y
  normal_values[2] = (mag_raw_values[5] << 8) | mag_raw_values[4]; //Z
  ESP_LOGI(TAG_IST8310,"Self-test normal values reads are %d, %d, %d",normal_values[0],normal_values[1],normal_values[2]);
  
  i2c_write_byte_to_address_NEW(IST8310_dev_handle, IST8310_SELF_TEST_REG, 0x40);      //enabling axis inversion
  i2c_write_byte_to_address_NEW(IST8310_dev_handle, IST8310_CNTL1_REG, IST8310_CNTL1_VAL_SINGLE_MEASUREMENT_MODE);      //enabling single measurenebt mode
  
  vTaskDelay(10/portTICK_PERIOD_MS);

  i2c_read_bytes_from_address_NEW(IST8310_dev_handle, IST8310_OUTPUT_X_L_REG, 6, mag_raw_values);     //read inversed values
  inversed_values[0] = (mag_raw_values[1] << 8) | mag_raw_values[0]; //X
  inversed_values[1] = (mag_raw_values[3] << 8) | mag_raw_values[2]; //Y
  inversed_values[2] = (mag_raw_values[5] << 8) | mag_raw_values[4]; //Z
  ESP_LOGI(TAG_IST8310,"Self-test inversed values reads are %d, %d, %d",inversed_values[0],inversed_values[1],inversed_values[2]);
   
    if (((abs(normal_values[0] + inversed_values[0])) < 20) &&                  //20 expirimental value, never equal
        ((abs(normal_values[1] + inversed_values[1])) < 20) &&
        ((abs(normal_values[2] + inversed_values[2])) < 20)) err = ESP_OK;

  i2c_write_byte_to_address_NEW(IST8310_dev_handle, IST8310_SELF_TEST_REG, 0x00);      //disabling axis inversion
  i2c_write_byte_to_address_NEW(IST8310_dev_handle, IST8310_TCCNTL_REG, 0x00);      //enabling temp compensation

  if (err == ESP_OK)  ESP_LOGI(TAG_IST8310,"IST8310 self test passed");
  else ESP_LOGE(TAG_IST8310,"IST8310 self test failed\n");

  return err;
}

esp_err_t IST8310_configuration()
{
  esp_err_t err = ESP_OK;
  uint8_t i;
  uint8_t received_value = 0;

  uint8_t IST8310_configuration_data[3][2] = {{IST8310_AVGCNTL_REG,                                     0x24},// 16 samples average
                                              {IST8310_PDCNTL_REG,  IST8310_PDCNTL_VAL_PULSE_DURATION_NORMAL}, //Set/Reset pulse duration setup
                                              {IST8310_SELF_TEST_REG,                                   0x00}};       

for (i=0; i<3; i++)   //writing predefined configuration 
{
  i2c_write_byte_to_address_NEW(IST8310_dev_handle,IST8310_configuration_data[i][0], IST8310_configuration_data[i][1]);  
  ets_delay_us(500);
}

ets_delay_us(5000);

for (i=0; i<3; i++) //checking against predefined configuration  
{
  received_value = i2c_read_byte_from_address_NEW(IST8310_dev_handle, IST8310_configuration_data[i][0]);
  if (received_value != IST8310_configuration_data[i][1]) 
  {
    err = ESP_FAIL;
    ESP_LOGE(TAG_IST8310,"IST8310 configuration failed at register %02x, returned value is %02x",IST8310_configuration_data[i][0], received_value);
  }
}

  if (err == ESP_OK) ESP_LOGI(TAG_IST8310,"IST8310 is configured\n");
  else  ESP_LOGE(TAG_IST8310,"IST8310 configuration failed\n");
  return err;
}

void IST8310_request_data()
{
  i2c_write_byte_to_address_NEW(IST8310_dev_handle, IST8310_CNTL1_REG, IST8310_CNTL1_VAL_SINGLE_MEASUREMENT_MODE);      //enabling single measurenebt mode
}

void IST8310_read_data(uint8_t *buffer)
{
  i2c_read_bytes_from_address_NEW(IST8310_dev_handle, IST8310_OUTPUT_X_L_REG, 6, buffer);
}

