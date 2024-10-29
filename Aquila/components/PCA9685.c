#include "ve_i2c.h"
#include "PCA9685.h"
#include "esp_log.h"
#include <rom/ets_sys.h>
#include "ve_alldef.h"


extern char *TAG_PCA9685;
extern i2c_master_dev_handle_t PCA9685_dev_handle;
extern i2c_master_bus_handle_t i2c_internal_bus_handle;

esp_err_t PCA9685_communication_check()
{
  esp_err_t err = ESP_FAIL;

  if (i2c_read_byte_from_address_NEW(PCA9685_dev_handle, PCA9685_SUBADR1) == 0xE2) {
    ESP_LOGI(TAG_PCA9685,"Связь с PCA9685 установлена");
    err = ESP_OK;
  }
  else ESP_LOGE(TAG_PCA9685,"Связь с PCA9685 не установлена\n");

  return err;
}

esp_err_t PCA9685_init()
{
  esp_err_t err = ESP_OK;
  uint8_t i;

  uint8_t PCA9685_configuration_data[4][2] = {{PCA9685_MODE1,     0b00110001},          // enable autoinc, Sleep 
                                              {PCA9685_PRESCALE,  0x7A},                //PWM frequency PRE_SCALE ADDRESS to set pwm at 50Hz [PRECALER = (25000000/(4096*frequency))]
                                              {PCA9685_MODE1,     0b10100001},          // Set MODE1 enable restart, autoinc, normal mode
                                              {PCA9685_MODE2,     0b00000100}};         //Set MODE2 outputs change on ACK, bidi

for (i=0; i<4; i++)   //writing predefined configuration 
{
  i2c_write_byte_to_address_NEW(PCA9685_dev_handle, PCA9685_configuration_data[i][0], PCA9685_configuration_data[i][1]);  
  ets_delay_us(500);
}

for (i=1; i<4; i++) //checking against predefined configuration  
{
  if (i2c_read_byte_from_address_NEW(PCA9685_dev_handle, PCA9685_configuration_data[i][0]) != (PCA9685_configuration_data[i][1] & 0b01111111)) //for MODE1
  {
    err = ESP_FAIL;
    ESP_LOGE(TAG_PCA9685,"Ошибка конфигурирования PCA9685 в регистре %d",PCA9685_configuration_data[i][0]);
  }
}

  if (err == ESP_OK) ESP_LOGI(TAG_PCA9685,"PCA9685 настроена\n");
  else  ESP_LOGE(TAG_PCA9685,"Ошибка настройки PCA9685\n");
  return err;
}

void PCA9685_send(uint8_t value_in_persents, uint8_t output) 
{
  uint16_t pulse_length;// temp variable for PWM
  uint8_t data_to_write[4];

  pulse_length = (uint16_t) (value_in_persents * 40.95);
  data_to_write[0] = 0;
  data_to_write[1] = 0;
  data_to_write[2] = pulse_length;                  //LSB
  data_to_write[3] = pulse_length >> 8;             //MSB

  i2c_write_bytes_to_address_NEW(PCA9685_dev_handle, PCA9685_LED0_ON_L + 4 * output, 5, &data_to_write);
}

