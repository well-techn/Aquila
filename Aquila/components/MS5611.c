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

  err = i2c_master_probe(i2c_external_bus_handle, MS5611_ADDRESS , -1);
  if (err == ESP_OK)  ESP_LOGI(TAG_MS5611,"Связь с MS5611 установлена");
  else ESP_LOGE(TAG_MS5611,"Связь с MS5611 не установлена\n");

  return err;
}

esp_err_t MS5611_I2C_reset()
{
    return i2c_write_command(MS5611_dev_handle, MS5611_CMD_RESET); 
}

//считываем PROM и проверяем по CRC, алгоритм расчета СКС из AN520
esp_err_t MS5611_I2C_PROM_read()
{
   esp_err_t ret = ESP_FAIL;
   uint16_t MS5611_PROM[8] = {2148, 45040, 48704, 28052, 26033, 32077, 28075, 32101};           
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
                                                                 
     ESP_LOGI(TAG_MS5611, "CRC is %d ",n_rem);
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
     
/*
 uint16_t MS5611_PROM[8] = {2116, 51467, 50692, 31930, 28256, 31666, 27448, 5320}; 
 while (1)
 {
  MS5611_I2C_request_D1();
  vTaskDelay(100/portTICK_PERIOD_MS);
  uint32_t MS5611_raw_pressure = MS5611_I2C_read();
  ESP_LOGI(TAG_MS5611,"raw pressure %ld",MS5611_raw_pressure);

 vTaskDelay(100/portTICK_PERIOD_MS);
  
  MS5611_I2C_request_D2();
  vTaskDelay(100/portTICK_PERIOD_MS);
  uint32_t MS5611_raw_temperature = MS5611_I2C_read();
  ESP_LOGI(TAG_MS5611,"raw temperature %ld",MS5611_raw_temperature);
  
  int32_t MS5611_dT = MS5611_raw_temperature - MS5611_PROM[5] * 256; 
  ESP_LOGD(TAG_MS5611,"MS5611_dT %ld", MS5611_dT);            

  double MS5611_TEMP = (2000 + ((double)MS5611_dT * MS5611_PROM[6]) / 8388608);
  ESP_LOGI(TAG_MS5611,"MS5611_TEMP %f", MS5611_TEMP);  

  double MS5611_OFF = MS5611_PROM[2] * pow(2,16) + (double)MS5611_PROM[4] * MS5611_dT / pow(2,7);                                
  ESP_LOGI(TAG_MS5611,"MS5611_OFF %f", MS5611_OFF);  
  
  double MS5611_SENS = MS5611_PROM[1] * pow(2,15) + (double)MS5611_PROM[3] * MS5611_dT / pow(2,8);
  ESP_LOGI(TAG_MS5611,"MS5611_SENS %f", MS5611_SENS);

  if  ((MS5611_TEMP/100.0) < 20.0) 
  {
    double MS5611_T2 = pow(MS5611_dT,2) / pow(2,31);
    double MS5611_OFF2 = 5 * pow(MS5611_TEMP - 2000,2) / 2;
    double MS5611_SENS2 = 5 * pow(MS5611_TEMP - 2000,2) / 4;

    if ((MS5611_TEMP/100.0) < -15.0)
    {
      MS5611_OFF2 += 7 * pow(MS5611_TEMP + 1500,2);
      MS5611_SENS2 += 11 * pow(MS5611_TEMP + 1500,2) / 2;
    }

    MS5611_TEMP -= MS5611_T2;
    MS5611_OFF -= MS5611_OFF2;
    MS5611_SENS -= MS5611_SENS2;

    ESP_LOGI(TAG_MS5611,"MS5611_TEMP_FINAL %f", MS5611_TEMP);
  }

  double MS5611_P = ((MS5611_raw_pressure * MS5611_SENS)/pow(2,21)- MS5611_OFF)/pow(2,15);
  ESP_LOGI(TAG_MS5611,"pressure %f\n", MS5611_P);
  //printf("%d\n", (uint16_t)(MS5611_P *  0.00750062));
  //printf("%d\n", (int16_t)((98300 - MS5611_P) * 8.3));

  vTaskDelay(2000/portTICK_PERIOD_MS);



 }
*/