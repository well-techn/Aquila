#include "wt_i2c.h"
#include "PCA9685.h"
#include "esp_log.h"
#include <rom/ets_sys.h>
#include "wt_alldef.h"


extern char *TAG_PCA9685;
extern i2c_master_dev_handle_t PCA9685_dev_handle;
//extern i2c_master_bus_handle_t i2c_internal_bus_handle;

esp_err_t PCA9685_communication_check()
{
  esp_err_t err = ESP_FAIL;

  if (i2c_read_byte_from_address(PCA9685_dev_handle, PCA9685_SUBADR1) == 0xE2) {
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

for (i=0; i<4; i++)   //записываем указанную выше конфигурацию
{
  i2c_write_byte_to_address(PCA9685_dev_handle, PCA9685_configuration_data[i][0], PCA9685_configuration_data[i][1]);  
  ets_delay_us(500);
}

for (i=1; i<4; i++) //проверяем записанные значения, опуская проверку PCA9685_MODE1
{
  if (i2c_read_byte_from_address(PCA9685_dev_handle, PCA9685_configuration_data[i][0]) != (PCA9685_configuration_data[i][1] & 0b01111111)) //for MODE1
  {
    err = ESP_FAIL;
    ESP_LOGE(TAG_PCA9685,"Ошибка конфигурирования PCA9685 в регистре %d",PCA9685_configuration_data[i][0]);
  }
}

  if (err == ESP_OK) ESP_LOGI(TAG_PCA9685,"PCA9685 настроена\n");
  else  ESP_LOGE(TAG_PCA9685,"Ошибка настройки PCA9685\n");
  return err;
}

//функция записыввает на вывод <output> сигнал с коэффициентом заполнения, указанном в <value_in_percents>. Период сигнала зашит при настройке в PCA9685_PRESCALE.
//например, для вывода на вывод 0 импульса 1мс (это 1/20 от периода 20ms, то есть 5%), записываем PCA9685_send(5, 0)
//для вывода на вывод 4 импульса 2мс (это 1/10 от периода 20ms, то есть 10%), записываем PCA9685_send(10, 4)
void PCA9685_send(uint8_t value_in_percents, uint8_t output) 
{
  uint16_t pulse_length;
  uint8_t data_to_write[4];

  pulse_length = (uint16_t) (value_in_percents * 40.95);
  data_to_write[0] = 0;
  data_to_write[1] = 0;
  data_to_write[2] = pulse_length;                  //LSB
  data_to_write[3] = pulse_length >> 8;             //MSB

  i2c_write_bytes_to_address(PCA9685_dev_handle, PCA9685_LED0_ON_L + 4 * output, 5, data_to_write);
}

