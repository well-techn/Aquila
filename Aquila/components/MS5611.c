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

void MS5611_I2C_reset()
{
    ESP_ERROR_CHECK(i2c_write_command(MS5611_dev_handle, MS5611_CMD_RESET)); 
}


esp_err_t MS5611_I2C_PROM_read()
{
   esp_err_t ret = ESP_FAIL;
   uint16_t MS5611_PROM[8] = {0, 50725, 52047, 31241, 29137, 31514, 27764, 2};           //Pro board #2
   uint16_t cnt;                                                                              // simple counter
   uint16_t n_rem = 0x00;                                                           // crc reminder
   uint16_t crc_read;                                                                // original value of the crc
   uint8_t n_bit;

    for (cnt = 0; cnt < 8; cnt++) printf("%d ",MS5611_PROM[cnt]);
    printf("\n");  
    for (uint8_t i=0; i < 8; i++)
    {                          
      //MS5611_PROM[i] = i2c_read_2_bytes_from_address(MS5611_dev_handle, MS5611_CMD_READ_PROM + 2*i);
      MS5611_PROM[i] = i2c_read_2_bytes_from_address(MS5611_dev_handle, 0b10101110);
      ESP_LOGI(TAG_MS5611, "%d ",MS5611_PROM[i]);
    }

    //for (cnt = 0; cnt < 8; cnt++) printf("%d ",MS5611_PROM[cnt]);
    
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
     n_rem = (0x0F & (n_rem >> 12));                                                         // final 4-bit reminder is CRC code
     MS5611_PROM[7] = crc_read;
     
                                                                 // restore the crc_read to its original place
     ESP_LOGI(TAG_MS5611, "CRC is %d ",n_rem ^ 0x00);
     return (ret);
    }

void MS5611_I2C_request_D1() 
{
    ESP_ERROR_CHECK(i2c_write_command(MS5611_dev_handle, MS5611_CMD_ADC_CONV | MS5611_CMD_ADC_D1 | MS5611_CMD_ADC_4096)); 
}

void MS5611_I2C_request_D2() 
{
    ESP_ERROR_CHECK(i2c_write_command(MS5611_dev_handle, MS5611_CMD_ADC_CONV | MS5611_CMD_ADC_D1 | MS5611_CMD_ADC_4096)); 
}


uint32_t MS5611_I2C_read() 
{
  uint8_t temp[3];
  uint32_t result = 0;
  ESP_ERROR_CHECK(i2c_write_command(MS5611_dev_handle, MS5611_CMD_ADC_READ));
  i2c_read_bytes_from_address(MS5611_dev_handle, MS5611_CMD_ADC_READ, 3, temp);
  result = (temp[0]>>16) + (temp[1]>>8) + temp[0];
  return (result);
}
     
/*
 void MS5611_convert_data() {

    if (track_MS_data) {
          longwordtostrwithzeros(MS5611_raw_pressure_filtered,txt_12);        //MS5611_raw_pressure_filtered
          for (i=0;i<12;i++) UART4_write(txt_12[i]);
          UART4_write(0x0D);
                        }
    if (track_MS_data) {
          longwordtostrwithzeros(MS5611_raw_temperature_filtered,txt_12);    //MS5611_raw_temperature_filtered
          for (i=0;i<12;i++) UART4_write(txt_12[i]);
          UART4_write(0x0D);
                   }

    MS_5611_dT = (long)MS5611_raw_temperature_filtered - 8067584l;             //prom[5} * 2^8      filtered

    temporary_var = (long long)MS_5611_dT  * (long long)MS5611_PROM[6];
        MS_5611_temperature = 2000l + (long)(temporary_var / 8388608l);      // /100 if in C

    if (track_MS_data) {
          longinttostrwithzeros(MS_5611_temperature,txt_12);
          for (i=0;i<12;i++) UART4_write(txt_12[i]);
          UART4_write(0x0D);
                        }
    MS_5611_temperature = MS_5611_temperature / 100;

    MS_5611_OFF =  ((long long)MS5611_PROM[4] * (long long)MS_5611_dT);                                  //C4
    MS_5611_OFF /=128;
    MS_5611_OFF = 3410952192ll + MS_5611_OFF;                               //C2 * 2^16
    
    MS_5611_SENS = ((long long)MS5611_PROM[3] * (long long)MS_5611_dT);
    MS_5611_SENS /=256;
    MS_5611_SENS = 1662156800ll + MS_5611_SENS;                             //C1 * 2^15

    temporary_var = ((long long)MS5611_raw_pressure_filtered * MS_5611_SENS);                        //filtered
    temporary_var /= 2097152ll;
    temporary_var -= MS_5611_OFF;

    MS_5611_pressure = (long)(temporary_var/32768ll);            //in mBar*100

    relative_height = ((float)MS_5611_ground_pressure - (float)MS_5611_pressure) * 8.33f;       //1mBar*100 = 8.3cm H = result*8.3 = result / 0.117 this is in cm
    if (relative_height < 0) relative_height = 0;

}
*/