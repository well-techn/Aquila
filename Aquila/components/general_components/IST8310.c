#include "IST8310.h"
#include "wt_i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <rom/ets_sys.h>
#include <advanced_math.h>

extern i2c_master_dev_handle_t IST8310_dev_handle;
extern i2c_master_bus_handle_t i2c_external_bus_handle;
extern char *TAG_IST8310;

esp_err_t IST8310_communication_check()
{
  esp_err_t err = ESP_FAIL;

  err = i2c_master_probe(i2c_external_bus_handle, IST8310_ADDR, -1);
  if (err == ESP_OK)  ESP_LOGI(TAG_IST8310,"Связь с IST8310 установлена");
  else ESP_LOGE(TAG_IST8310,"Связь с IST8310 не установлена\n");

  return err;
}

esp_err_t IST8310_selftest()
{
  esp_err_t err = ESP_FAIL;

  uint8_t mag_raw_values[6];
  int16_t normal_values[3] = {0,0,0};
  int16_t inversed_values[3] = {0,0,0};

  i2c_write_byte_to_address(IST8310_dev_handle, IST8310_SELF_TEST_REG, 0x00);      //disabling axis inversion
  i2c_write_byte_to_address(IST8310_dev_handle, IST8310_TCCNTL_REG, 0x01);      //disabling temp compensation
  i2c_write_byte_to_address(IST8310_dev_handle, IST8310_CNTL1_REG, IST8310_CNTL1_VAL_SINGLE_MEASUREMENT_MODE);      //enabling single measurenebt mode
  vTaskDelay(10/portTICK_PERIOD_MS);

  i2c_read_bytes_from_address(IST8310_dev_handle, IST8310_OUTPUT_X_L_REG, 6, mag_raw_values);     //read_normal values

  normal_values[0] = (mag_raw_values[1] << 8) | mag_raw_values[0]; //X
  normal_values[1] = (mag_raw_values[3] << 8) | mag_raw_values[2]; //Y
  normal_values[2] = (mag_raw_values[5] << 8) | mag_raw_values[4]; //Z
  ESP_LOGI(TAG_IST8310,"Данные без инверсии %d, %d, %d",normal_values[0],normal_values[1],normal_values[2]);
  
  i2c_write_byte_to_address(IST8310_dev_handle, IST8310_SELF_TEST_REG, 0x40);      //enabling axis inversion
  i2c_write_byte_to_address(IST8310_dev_handle, IST8310_CNTL1_REG, IST8310_CNTL1_VAL_SINGLE_MEASUREMENT_MODE);      //enabling single measurenebt mode
  
  vTaskDelay(10/portTICK_PERIOD_MS);

  i2c_read_bytes_from_address(IST8310_dev_handle, IST8310_OUTPUT_X_L_REG, 6, mag_raw_values);     //read inversed values
  inversed_values[0] = (mag_raw_values[1] << 8) | mag_raw_values[0]; //X
  inversed_values[1] = (mag_raw_values[3] << 8) | mag_raw_values[2]; //Y
  inversed_values[2] = (mag_raw_values[5] << 8) | mag_raw_values[4]; //Z
  ESP_LOGI(TAG_IST8310,"Данные с инверсией %d, %d, %d",inversed_values[0],inversed_values[1],inversed_values[2]);
   
    if (((abs(normal_values[0] + inversed_values[0])) < 20) &&                  //20 expirimental value, never equal
        ((abs(normal_values[1] + inversed_values[1])) < 20) &&
        ((abs(normal_values[2] + inversed_values[2])) < 20)) err = ESP_OK;

  i2c_write_byte_to_address(IST8310_dev_handle, IST8310_SELF_TEST_REG, 0x00);      //disabling axis inversion
  i2c_write_byte_to_address(IST8310_dev_handle, IST8310_TCCNTL_REG, 0x00);      //enabling temp compensation

  if (err == ESP_OK)  ESP_LOGI(TAG_IST8310,"Самодиагностика IST8310 успешно пройдена");
  else ESP_LOGE(TAG_IST8310,"Самодиагностика IST8310 не пройдена\n");

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
  i2c_write_byte_to_address(IST8310_dev_handle,IST8310_configuration_data[i][0], IST8310_configuration_data[i][1]);  
  ets_delay_us(500);
}

ets_delay_us(5000);

for (i=0; i<3; i++) //checking against predefined configuration  
{
  received_value = i2c_read_byte_from_address(IST8310_dev_handle, IST8310_configuration_data[i][0]);
  if (received_value != IST8310_configuration_data[i][1]) 
  {
    err = ESP_FAIL;
    ESP_LOGE(TAG_IST8310,"Ошибка конфигурирования IST8310 в регистре %02x, считано значение %02x",IST8310_configuration_data[i][0], received_value);
  }
}

  if (err == ESP_OK) ESP_LOGI(TAG_IST8310,"IST8310 настроен\n");
  else  ESP_LOGE(TAG_IST8310,"Ошибка настройки IST8310\n");
  return err;
}

void IST8310_request_data()
{
  i2c_write_byte_to_address(IST8310_dev_handle, IST8310_CNTL1_REG, IST8310_CNTL1_VAL_SINGLE_MEASUREMENT_MODE);      //enabling single measurenebt mode
}

esp_err_t IST8310_read_data(uint8_t *buffer)
{
  return i2c_read_bytes_from_address(IST8310_dev_handle, IST8310_OUTPUT_X_L_REG, 6, buffer);
}

esp_err_t IST8310_generate_cross_axis_matrix(float result[][3])
{
  esp_err_t ret = ESP_FAIL;
  uint8_t cross_axis_raw_values[18];
  int16_t Y_matrix[3][3] = {{0,  0,  0},    
                            {0,  0,  0},
                            {0,  0,  0}};
  float X_matrix[3][3] = {{0,  0,  0},    
                          {0,  0,  0},
                          {0,  0,  0}};

  float X_inv[3][3] = {{0,  0,  0},    
                       {0,  0,  0},
                       {0,  0,  0}};

  float fifty_matrix[3][3] = {{50,  0,  0},    
                              {0,  50,  0},
                              {0,  0,  50}};

    ret = i2c_read_bytes_from_address(IST8310_dev_handle, IST8310_CROSS_AXIS_REG, 18, cross_axis_raw_values);
/*
    Y_matrix[0][0] = cross_axis_raw_values[1] << 8 | cross_axis_raw_values[0];
    Y_matrix[0][1] = cross_axis_raw_values[3] << 8 | cross_axis_raw_values[2];
    Y_matrix[0][2] = cross_axis_raw_values[5] << 8 | cross_axis_raw_values[4];
    Y_matrix[1][0] = cross_axis_raw_values[7] << 8 | cross_axis_raw_values[6];
    Y_matrix[1][1] = cross_axis_raw_values[9] << 8 | cross_axis_raw_values[8];
    Y_matrix[1][2] = cross_axis_raw_values[11] << 8 | cross_axis_raw_values[10];
    Y_matrix[2][0] = cross_axis_raw_values[13] << 8 | cross_axis_raw_values[12];
    Y_matrix[2][1] = cross_axis_raw_values[15] << 8 | cross_axis_raw_values[14];
    Y_matrix[2][2] = cross_axis_raw_values[17] << 8 | cross_axis_raw_values[16];
*/
    Y_matrix[0][0] = cross_axis_raw_values[1] << 8 | cross_axis_raw_values[0];
    Y_matrix[1][0] = cross_axis_raw_values[3] << 8 | cross_axis_raw_values[2];
    Y_matrix[2][0] = cross_axis_raw_values[5] << 8 | cross_axis_raw_values[4];
    Y_matrix[0][1] = cross_axis_raw_values[7] << 8 | cross_axis_raw_values[6];
    Y_matrix[1][1] = cross_axis_raw_values[9] << 8 | cross_axis_raw_values[8];
    Y_matrix[2][1] = cross_axis_raw_values[11] << 8 | cross_axis_raw_values[10];
    Y_matrix[0][2] = cross_axis_raw_values[13] << 8 | cross_axis_raw_values[12];
    Y_matrix[1][2] = cross_axis_raw_values[15] << 8 | cross_axis_raw_values[14];
    Y_matrix[2][2] = cross_axis_raw_values[17] << 8 | cross_axis_raw_values[16];

    for (uint8_t i = 0; i<3; i++)
    {
      for (uint8_t j = 0; j<3; j++) X_matrix[i][j] = (float)Y_matrix[i][j] * 3/20;
    }

    invert3x3(X_matrix[0], X_inv[0]);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int u = 0; u < 3; u++)
                result[i][j] += fifty_matrix[i][u] * X_inv[u][j];
        }
      }

  //ESP_LOGW(TAG_IST8310,"%f,%f,%f",result[0][0],result[1][0],result[2][0]);
  //ESP_LOGW(TAG_IST8310,"%f,%f,%f",result[0][1],result[1][1],result[2][1]);
  //ESP_LOGW(TAG_IST8310,"%f,%f,%f",result[0][2],result[1][2],result[2][2]);

    return ret;    
}

