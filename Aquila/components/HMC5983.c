#include "HMC5983.h"
#include "ve_alldef.h"
#include "ve_spi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

extern spi_device_handle_t HMC5983;
extern char *TAG_HMC5983;

esp_err_t HMC5983L_SPI_communication_check()
{
      esp_err_t err = ESP_FAIL;

      if (SPI_read_byte(HMC5983, 0, 0, 8, HMC5983_ID_A | SPI_READ_FLAG, 0) == 0b01001000)
      {
            err = ESP_OK;
            ESP_LOGI(TAG_HMC5983, "Связь с HMC5985 установлена");
      }
      else
            ESP_LOGE(TAG_HMC5983, "Связь с HMC5985 не установлена\n");
      return err;
}

esp_err_t HMC5983L_SPI_init()
{ // configure the HMC5983L (magnetometer)

      esp_err_t err = ESP_OK;
      uint8_t reg_value, i;
      uint8_t HMC5983_configuration_data[3][2] = {{HMC5983_CONF_A, 0b11111100}, // 0b11111100 config A register - temp sensor enabled, 8 samples averaging, sample rate = 220Hz, no bias
                                                  {HMC5983_CONF_B, 0b00100000}, // config B register - +-1.3 Gauss
                                                  {HMC5983_MODE, 0b00000000}};  // mode register - 4 wires SPI, continuing measurement mode

      for (i = 0; i < 3; i++)
      {
            SPI_write_byte(HMC5983, 0, 0, 8, HMC5983_configuration_data[i][0], 0, HMC5983_configuration_data[i][1]);
      }

      for (i = 0; i < 3; i++)
      { // checking against predefined configuration
            reg_value = SPI_read_byte(HMC5983, 0, 0, 8, HMC5983_configuration_data[i][0] | SPI_READ_FLAG, 0);
            if (reg_value != HMC5983_configuration_data[i][1])
            {
                  err = ESP_FAIL;
                  ESP_LOGE(TAG_HMC5983, "Ошибка конфигурирования HMC5983 в регистре %x, считано значение %d", HMC5983_configuration_data[i][0], reg_value);
            }
      }

      if (err == ESP_OK)
            ESP_LOGI(TAG_HMC5983, "HMC5983 настроен\n");
      else
            ESP_LOGE(TAG_HMC5983, "Ошибка настройки HMC5983\n");

      return err;
}

esp_err_t HMC5983L_SPI_selftest()
{

      uint8_t raw_mag_data[6];
      int16_t mag_data[3] = {0};
      uint8_t i = 0;
      uint8_t test_result;
      esp_err_t err = ESP_FAIL;

      // positiv bias test
      SPI_write_byte(HMC5983, 0, 0, 8, HMC5983_CONF_A, 0, 0x71); // config A register - (8-average, 15 Hz default, positive self test measurement
      SPI_write_byte(HMC5983, 0, 0, 8, HMC5983_CONF_B, 0, 0xA0); // config B register - (Gain=5)
      SPI_write_byte(HMC5983, 0, 0, 8, HMC5983_MODE, 0, 0x00);   // mode register -  (Continuous-measurement mode)

      for (i = 0; i < 1; i++)
      {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            SPI_read_bytes(HMC5983, 0, 0, 8, HMC5983_OUT_X_MSB | SPI_READ_FLAG, 0, raw_mag_data, 6);
            // for (j=0;j<6;j++) printf ("%d ",raw_mag_data[j]);
            // printf ("\n");
      }

      mag_data[0] = (raw_mag_data[0] << 8) | raw_mag_data[1]; // X values
      mag_data[1] = (raw_mag_data[4] << 8) | raw_mag_data[5]; // Y values
      mag_data[2] = (raw_mag_data[2] << 8) | raw_mag_data[3]; // Z values
      // printf ("Axis values are ");
      // for (i=0;i<3;i++) printf ("%d ",mag_data[i]);
      // printf ("\n");
      if (((mag_data[0] > 243) && (mag_data[0] < 575)) && ((mag_data[1] > 243) && (mag_data[1] < 575)) && ((mag_data[2] > 243) && (mag_data[2] < 575)))
            test_result = 1;
      else
            test_result = 0;

      // negative bias test
      SPI_write_byte(HMC5983, 0, 0, 8, HMC5983_CONF_A, 0, 0x72); // config A register - (8-average, 15 Hz default, negative self test measurement
      SPI_write_byte(HMC5983, 0, 0, 8, HMC5983_CONF_B, 0, 0xA0); // config B register - (Gain=5)
      SPI_write_byte(HMC5983, 0, 0, 8, HMC5983_MODE, 0, 0x00);   // mode register -  (Continuous-measurement mode)

      for (i = 0; i < 1; i++)
      {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            SPI_read_bytes(HMC5983, 0, 0, 8, HMC5983_OUT_X_MSB | SPI_READ_FLAG, 0, raw_mag_data, 6);
            // for (j=0;j<6;j++) printf ("%d ",raw_mag_data[j]);
            // printf ("\n");
      }

      mag_data[0] = (raw_mag_data[0] << 8) | raw_mag_data[1]; // X values
      mag_data[1] = (raw_mag_data[4] << 8) | raw_mag_data[5]; // Y values
      mag_data[2] = (raw_mag_data[2] << 8) | raw_mag_data[3]; // Z values
      // printf ("Axis values are ");
      // for (i=0;i<3;i++) printf ("%d ",mag_data[i]);
      // printf ("\n");
      if (((mag_data[0] > -575) && (mag_data[0] < -243)) && ((mag_data[1] > -575) && (mag_data[1] < -243)) && ((mag_data[2] > -575) && (mag_data[2] < -243)))
            test_result = test_result * 1;
      else
            test_result = test_result * 0;

      if (test_result != 0)
      {
            err = ESP_OK;
            ESP_LOGI(TAG_HMC5983, "Самодиагностика HMC5983 успешно пройдена");
      }
      else
            ESP_LOGE(TAG_HMC5983, "Самодиагностика HMC5983 не пройдена");

      return err;
}

void HMC5983L_SPI_data_read(uint8_t *buffer)
{
      SPI_read_bytes(HMC5983, 0, 0, 8, HMC5983_OUT_X_MSB | HMC5983_AUTOINC_FLAG | SPI_READ_FLAG, 0, buffer, 6);
}



/*

esp_err_t HMC5883_I2C_communication_check()
{
  esp_err_t err = ESP_FAIL;

  err = i2c_master_probe(i2c_internal_bus_handle, HMC5883L_I2C_ADDR, -1);
  if (err == ESP_OK)  ESP_LOGI(TAG_HMC5883_I2C,"HMC5883 is online");
  else ESP_LOGE(TAG_HMC5883_I2C,"HMC5883 is offline\n");

  return err;
}

esp_err_t HMC5883_I2C_configuration()
{
  esp_err_t err = ESP_OK;
  uint8_t i = 0;
  uint16_t settings = HMC5883_I2C_CONFIG_BRNG_16V |
                  HMC5883_I2C_CONFIG_GAIN_8_320MV |
                  HMC5883_I2C_CONFIG_BADC_12BIT |
                  HMC5883_I2C_CONFIG_SADC_12BIT_1S_532US |
                  HMC5883_I2C_CONFIG_MODE_SANDBVOLT_CONTINUOUS;


                                              //4096
  uint8_t HMC5883_I2C_configuration_data[3][2] = {{HMC5983_CONF_A, 0b11111100}, // 0b11111100 config A register - temp sensor enabled, 8 samples averaging, sample rate = 220Hz, no bias
                                                  {HMC5983_CONF_B, 0b00100000}, // config B register - +-1.3 Gauss
                                                  {HMC5983_MODE,   0b00000000}};   //MSB goes first, 16V, PGA/4, 

  for (i=0;i<2;i++) { i2c_write_2_bytes_to_address_NEW(HMC5883_I2C_dev_handle, HMC5883_I2C_configuration_data[i][0], HMC5883_I2C_configuration_data[i][1], HMC5883_I2C_configuration_data[i][2]);  //pointer to 2D massive
                      ets_delay_us(100);
                    }
  
  for (i=0;i<2;i++) 
    { 
      if ((i2c_read_2_bytes_from_address_NEW(HMC5883_I2C_dev_handle, HMC5883_I2C_configuration_data[i][0])) != ((HMC5883_I2C_configuration_data[i][1] << 8) + HMC5883_I2C_configuration_data[i][2])) 
      {  
        err = ESP_FAIL;
        ESP_LOGE(TAG_HMC5883_I2C,"HMC5883_I2C configuration failed at register %x",HMC5883_I2C_configuration_data[i][0]);
      }
    }
  
  if (err == ESP_OK) ESP_LOGI(TAG_HMC5883_I2C,"HMC5883_I2C is configured\n");
    else  ESP_LOGE(TAG_HMC5883_I2C,"HMC5883_I2C configuration failed\n");

  return err;
}

esp_err_t IST8310_read_data(uint8_t *buffer)
{
  return i2c_read_bytes_from_address_NEW(HMC5883_I2C_dev_handle, HMC5983_OUT_X_MSB, 6, buffer);
}
*/




/*
if ((cycle_counter > (int)number_of_calibration_counts) && (cycle_counter < (int)(number_of_calibration_counts + number_of_magnetometer_calibration_reads)))  {
    for (i=0;i<3;i++) {
                if (Magnet_Raw[i] > Magnet_Max[i]) Magnet_Max[i] = Magnet_Raw[i];
                if (Magnet_Raw[i] < Magnet_Min[i]) Magnet_Min[i] = Magnet_Raw[i];
                       }
      if (!(Cycle_counter%20)) UART2_write(0x2A);
      }

   if (cycle_counter == number_of_calibration_counts + number_of_magnetometer_calibration_reads)  {
      for (i=0;i<3;i++) {Magnet_hard_bias[i] = (Magnet_Max[i] + Magnet_Min[i]) / 2.0;
                         Magnet_radius[i] = (Magnet_Max[i] - Magnet_Min[i]) / 2.0;
                         }

      Magnet_Radius_avg = (Magnet_radius[0] + Magnet_radius[1] + Magnet_radius[2]) / 3.0;
      for (i=0;i<3;i++) Magnet_coeff[i] = Magnet_Radius_avg / Magnet_radius[i];

      UART2_write(0x0D);
      UART2_write_text("Hard_biases are ");
      for (i=0;i<3;i++) {
                floattostr(Magnet_hard_bias[i],txt_15);
                for (j=0;j<15;j++) UART2_write (txt_15[j]);
                for (j=0;j<15;j++) txt_15[j] = 0x2A;
                uart2_write(0x20);
                }
      uart2_write(0x0D);
      UART2_write_text("Radiuses are ");
      for (i=0;i<3;i++) {
                floattostr(Magnet_radius[i],txt_15);
                for (j=0;j<15;j++) UART2_write (txt_15[j]);
                for (j=0;j<15;j++) txt_15[j] = 0x2A;
                uart2_write(0x20);
                }
      uart2_write(0x0D);

      UART2_write_text("R_avg is ");
                floattostr(Magnet_Radius_avg,txt_15);
                for (j=0;j<15;j++) UART2_write (txt_15[j]);
                for (j=0;j<15;j++) txt_15[j] = 0x2A;
      uart2_write(0x0D);

      UART2_write_text("Coefficients are ");
      for (i=0;i<3;i++) {
                floattostr(Magnet_coeff[i],txt_15);
                for (j=0;j<15;j++) UART2_write (txt_15[j]);
                for (j=0;j<15;j++) txt_15[j] = 0x2A;
                uart2_write(0x20);
                }
      uart2_write(0x0D);

//   long_beep();
//     long_beep();
//     long_beep();
//     delay_ms(3000);
     for (i=0;i<3;i++) {long_beep();delay_ms(500);}

     }



if (cycle_counter > (number_of_calibration_counts + number_of_magnetometer_calibration_reads))
{

 cycle_counter = number_of_calibration_counts + number_of_magnetometer_calibration_reads + 1;

 for (i=0;i<3;i++) {
         Magnet_converted[i] = ((float)Magnet_raw[i] - Magnet_hard_bias[i]) * Magnet_coeff[i];
*/