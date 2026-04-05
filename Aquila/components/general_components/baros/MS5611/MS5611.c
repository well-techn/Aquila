#include "MS5611.h"
#include "wt_i2c.h"
//#include "driver/i2c.h"
#include "wt_alldef.h"
#include "esp_log.h"
#include <rom/ets_sys.h>

extern char *TAG_MS5611;
extern i2c_master_dev_handle_t MS5611_dev_handle;
extern i2c_master_bus_handle_t i2c_external_bus_handle;

esp_err_t MS5611_communication_check()
{
  esp_err_t err = ESP_FAIL;

  err = i2c_master_probe(i2c_external_bus_handle, MS5611_ADDRESS, -1);
  if (err == ESP_OK)  ESP_LOGI(TAG_MS5611,"Связь с MS5611 установлена");
  else ESP_LOGE(TAG_MS5611,"Связь с MS5611 не установлена\n");

  return err;
}

esp_err_t MS5611_I2C_reset()
{
    return i2c_write_command(MS5611_dev_handle, MS5611_CMD_RESET); 
}

//считываем PROM и проверяем по CRC, алгоритм расчета CRC из AN520
esp_err_t MS5611_I2C_PROM_read(uint16_t* MS5611_PROM)
{
   esp_err_t ret = ESP_FAIL;   
   uint16_t cnt = 0;;                                                                              // simple counter
   uint16_t n_rem = 0x00;                                                           // crc reminder
   uint16_t crc_read = 0;                                                                // original value of the crc
   uint8_t n_bit = 0;

    for (uint8_t i=0; i < 8; i++)
    {                          
      MS5611_PROM[i] = i2c_read_2_bytes_from_address(MS5611_dev_handle, MS5611_CMD_READ_PROM + 2*i);
      ESP_LOGI(TAG_MS5611, "PROM[%d] = %d ",i,MS5611_PROM[i]);
    }
 
    crc_read = MS5611_PROM[7];                                                            //save read CRC
    MS5611_PROM[7] = (0xFF00 & (MS5611_PROM[7]));                                         //CRC byte is replaced by 0
    for (cnt = 0; cnt < 16; cnt++) 
    {                                                                                       // operation is performed on bytes
         if (cnt % 2) n_rem ^= (unsigned short) ((MS5611_PROM[cnt >> 1]) & 0x00FF);         // choose LSB or MSB
         else n_rem ^= (unsigned short) (MS5611_PROM[cnt >> 1] >> 8);
         for (n_bit = 8; n_bit > 0; n_bit--) {
             if (n_rem & (0x8000)) n_rem = (n_rem << 1) ^ 0x3000;
             else n_rem = (n_rem << 1);
         }
     }
     n_rem = (0x0000F & (n_rem >> 12));                                                         // final 4-bit reminder is CRC code
     MS5611_PROM[7] = crc_read;                                                              // restore the crc_read to its original place
                                                                 
     ESP_LOGI(TAG_MS5611, "CRC = %d ",n_rem);
     ESP_LOGI(TAG_MS5611, "4 младшие бита PROM[7] %d ",(MS5611_PROM[7] & 0x000F));

     if ((MS5611_PROM[7] & 0x000F) == n_rem) ret = ESP_OK;                                   //если 4 последние бита PROM[7] равны подсчитанному CRC Значит все ок
     return (ret);
    }

void MS5611_I2C_request_D1() 
{
    ESP_ERROR_CHECK(i2c_write_command(MS5611_dev_handle, MS5611_CMD_ADC_CONV | MS5611_CMD_ADC_D1 | MS5611_CMD_ADC_4096)); 
}

void MS5611_I2C_request_D2() 
{
    ESP_ERROR_CHECK(i2c_write_command(MS5611_dev_handle, MS5611_CMD_ADC_CONV | MS5611_CMD_ADC_D2 | MS5611_CMD_ADC_4096)); 
}


uint32_t MS5611_I2C_read() 
{
  uint8_t temp[3] = {0,0,0};
  uint32_t result = 0;
  ESP_ERROR_CHECK(i2c_read_bytes_from_address(MS5611_dev_handle, MS5611_CMD_ADC_READ, 3, temp));
  result = (temp[0]<<16) + (temp[1]<<8) + temp[0];
  return (result);
}
