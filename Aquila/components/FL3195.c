#include "FL3195.h"
#include "wt_i2c.h"
//#include "driver/i2c.h"
#include "wt_alldef.h"
#include "esp_log.h"
#include <rom/ets_sys.h>


extern char *TAG_FL3195;
extern i2c_master_dev_handle_t FL3195_dev_handle;
extern i2c_master_bus_handle_t i2c_external_bus_handle;

esp_err_t FL3195_communication_check()
{
  esp_err_t err = ESP_FAIL;

  err = i2c_master_probe(i2c_external_bus_handle, FL3195_ADDR, -1);
  if (err == ESP_OK)  ESP_LOGI(TAG_FL3195,"Связь с FL3195 установлена");
  else ESP_LOGE(TAG_FL3195,"Связь с FL3195 не установлена\n");

  return err;
}

esp_err_t FL3195_configuration()
{
  esp_err_t err = ESP_OK;
  uint8_t i = 0;
  uint8_t received_value = 0;
  uint8_t FL3195_configuration_data[9][2] = {{FL3195_RESET,                0xC5},
                                             {FL3195_SHUTDOWN_CTRL,        0xF1},                                  
                                             {FL3195_MODE_CONFIG,          0xF1},    //pattern RGB+W mode
                                             {FL3195_CHARGE_PUMP1,         0x00},
                                             {FL3195_CURRENT_BAND,         0x55},
                                             {0x1C,                        0x07},
                                             {0x1D,                        0x00},
                                             {0x1E,                        0x11},
                                             {0x1F,                        0x01}}; //Loop Times

      for (i=0; i<9; i++)   //writing predefined configuration 
    {
      i2c_write_byte_to_address(FL3195_dev_handle,FL3195_configuration_data[i][0], FL3195_configuration_data[i][1]);  
      ets_delay_us(500);
    }

    for (i=1; i<9; i++) //checking against predefined configuration except reset
    {
      received_value = i2c_read_byte_from_address(FL3195_dev_handle, FL3195_configuration_data[i][0]);
      if (received_value != FL3195_configuration_data[i][1]) 
      {
        err = ESP_FAIL;
        ESP_LOGE(TAG_FL3195,"Ошибка конфигурирования FL3195 в регистре %02x, считано значение %02x",FL3195_configuration_data[i][0], received_value);
      }
    }

      if (err == ESP_OK) ESP_LOGI(TAG_FL3195,"FL3195 настроен\n");
      else  ESP_LOGE(TAG_FL3195,"Ошибка настройки FL3195\n");
      return err;
}

//устанавливает пэттерн. 
//pulse_length - частота моргания и далее 3 цваета, от 0 до 255
void FL3195_set_pattern(uint8_t pulse_length, uint8_t color_red,uint8_t color_green,uint8_t color_blue) //pulse_length 1 - fast, 5 - slow
{
 //P1 time and cycle
  uint8_t pulse_parameter_1 = (pulse_length & 0x0F) | 0x10;      //T3 fall time
  uint8_t pulse_parameter_2 = (pulse_length & 0x0F);            //T3 fall time

  i2c_write_byte_to_address(FL3195_dev_handle,0x19,0x10);                 //T1 (rise) & TS (start)
  i2c_write_byte_to_address(FL3195_dev_handle,0x1A,pulse_parameter_1);    //T3 (fall) & T2 (hold)  
  i2c_write_byte_to_address(FL3195_dev_handle,0x1B,pulse_parameter_2);    //T4 (pause) & TP (off) 

  //out1 color
  i2c_write_byte_to_address(FL3195_dev_handle,0x10,color_red);    //RED чем больше тем ярче 
  i2c_write_byte_to_address(FL3195_dev_handle,0x11,color_green);    //GREEN
  i2c_write_byte_to_address(FL3195_dev_handle,0x12,color_blue);    //BLUE

  i2c_write_byte_to_address(FL3195_dev_handle,0x50,0xC5);    //upload data  
  i2c_write_byte_to_address(FL3195_dev_handle,0x51,0xC5);
}

