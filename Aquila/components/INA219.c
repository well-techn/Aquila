#include "INA219.h"
#include "ve_i2c.h"
//#include "driver/i2c.h"
#include "ve_alldef.h"
#include "esp_log.h"
#include <rom/ets_sys.h>

extern char *TAG_INA219;
extern i2c_master_dev_handle_t INA219_dev_handle;
extern i2c_master_bus_handle_t i2c_internal_bus_handle;

esp_err_t INA219_communication_check()
{
  esp_err_t err = ESP_FAIL;

  err = i2c_master_probe(i2c_internal_bus_handle, INA219_ADDR, -1);
  if (err == ESP_OK)  ESP_LOGI(TAG_INA219,"INA219 is online");
  else ESP_LOGE(TAG_INA219,"INA219 is offline\n");

  return err;
}

esp_err_t INA219_configuration()
{
  esp_err_t err = ESP_OK;
  uint8_t i = 0;
  uint16_t settings = INA219_CONFIG_BRNG_16V |
                  INA219_CONFIG_GAIN_8_320MV |
                  INA219_CONFIG_BADC_12BIT |
                  INA219_CONFIG_SADC_12BIT_1S_532US |
                  INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;


                                              //4096
  uint8_t INA219_configuration_data[2][3] = {{INA219_CALIBRATION,               0b00010000,              0b00000000},    //MSB goes first, shifted right 1 bit calibration value       
                                             {INA219_CONFIGURATION,  (settings >> 8), settings & 0x00FF }};   //MSB goes first, 16V, PGA/4, 

  for (i=0;i<2;i++) { i2c_write_2_bytes_to_address_NEW(INA219_dev_handle, INA219_configuration_data[i][0], INA219_configuration_data[i][1], INA219_configuration_data[i][2]);  //pointer to 2D massive
                      ets_delay_us(100);
                    }
  
  for (i=0;i<2;i++) 
    { 
      if ((i2c_read_2_bytes_from_address_NEW(INA219_dev_handle, INA219_configuration_data[i][0])) != ((INA219_configuration_data[i][1] << 8) + INA219_configuration_data[i][2])) 
      {  
        err = ESP_FAIL;
        ESP_LOGE(TAG_INA219,"INA219 configuration failed at register %x",INA219_configuration_data[i][0]);
      }
    }
  
  if (err == ESP_OK) ESP_LOGI(TAG_INA219,"INA219 is configured\n");
    else  ESP_LOGE(TAG_INA219,"INA219 configuration failed\n");

  return err;
}


float INA219_read_voltage()
{
  uint16_t ADC_value;
  float voltage;
  uint8_t raw_data[2];

  i2c_read_bytes_from_address_NEW(INA219_dev_handle, INA219_BUS_VOLTAGE, 2, raw_data);
  ADC_value = ((raw_data[0] << 8) + raw_data[1]) >> 3;
  voltage = ADC_value * 0.004;

  return voltage;
}

float INA219_read_shunt_voltage()
{
  int16_t ADC_value;
  float shunt_voltage;
  uint8_t raw_data[2];

  i2c_read_bytes_from_address_NEW(INA219_dev_handle, INA219_SHUNT_VOLTAGE, 2, raw_data);
  ADC_value = ((raw_data[0] << 8) + raw_data[1]);
  shunt_voltage = ADC_value * 0.001000;   
  //printf("%d ", ADC_value);
  return shunt_voltage;
}

float INA219_read_current()
{
  int16_t ADC_value;
  float current;
  uint8_t raw_data[2];

  i2c_read_bytes_from_address_NEW(INA219_dev_handle, INA219_CURRENT, 2, raw_data);
  ADC_value = (raw_data[0] << 8) + raw_data[1];
  //printf("%d ", ADC_value);
  current = ADC_value * 0.001;//1mA LSB

  return current;
}

float INA219_read_power()
{
  uint16_t ADC_value;
  float power;
  uint8_t raw_data[2];

  i2c_read_bytes_from_address_NEW(INA219_dev_handle, INA219_POWER, 2, raw_data);
  ADC_value = (raw_data[0] << 8) + raw_data[1];
  power = ADC_value * 0.020; //power LSB

  return power;
}