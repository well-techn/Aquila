#include "px4flow.h"
#include "wt_i2c.h"
#include "wt_alldef.h"
#include "esp_log.h"
#include <rom/ets_sys.h>

extern char *TAG_PX4FLOW;
extern i2c_master_dev_handle_t PX4FLOW_dev_handle;
extern i2c_master_bus_handle_t i2c_internal_bus_handle;

esp_err_t px4flow_communication_check()
{
  esp_err_t err = ESP_FAIL;

  err = i2c_master_probe(i2c_internal_bus_handle, PX4FLOW_I2C_ADDRESS, -1);
  if (err == ESP_OK)  ESP_LOGI(TAG_PX4FLOW,"Связь с PX4FLOW установлена");
  else ESP_LOGE(TAG_PX4FLOW,"Связь с PX4FLOW не установлена\n");

  return err;
}

void px4flow_read_frame(px4flow_i2c_frame_t* this)
{
  uint8_t raw_data[22];

  i2c_read_bytes_from_address(PX4FLOW_dev_handle, PX4FLOW_READ_FRAME_ADDRESS, sizeof(px4flow_i2c_frame_t), raw_data);
    this->frame_count = (raw_data[1] << 8) + raw_data[0];
    this->pixel_flow_x_sum = (raw_data[3] << 8) + raw_data[2];
    this->pixel_flow_y_sum = (raw_data[5] << 8) + raw_data[4];
    this->flow_comp_m_x = (raw_data[7] << 8) + raw_data[6];
    this->flow_comp_m_y = (raw_data[9] << 8) + raw_data[8];
    this->quality = (raw_data[11] << 8) + raw_data[10];
    this->gyro_x_rate = (raw_data[13] << 8) + raw_data[12];
    this->gyro_y_rate = (raw_data[15] << 8) + raw_data[14];
    this->gyro_z_rate = (raw_data[17] << 8) + raw_data[16];
    this->gyro_range = raw_data[18];
    this->sonar_timestamp = raw_data[19];
    this->ground_distance = (raw_data[21] << 8) + raw_data[20];
}

void px4flow_read_integral_frame(px4flow_i2c_integral_frame_t* this)
{
  uint8_t raw_data[25];

  i2c_read_bytes_from_address(PX4FLOW_dev_handle, PX4FLOW_READ_INTEGRAL_FRAME_ADDRESS, sizeof(px4flow_i2c_integral_frame_t), raw_data);
  this->frame_count_since_last_readout = (raw_data[1] << 8) + raw_data[0];
  this->pixel_flow_x_integral = (raw_data[3] << 8) + raw_data[2];
  this->pixel_flow_y_integral = (raw_data[5] << 8) + raw_data[4];
  this->gyro_x_rate_integral = (raw_data[7] << 8) + raw_data[6];
  this->gyro_y_rate_integral = (raw_data[9] << 8) + raw_data[8];
  this->gyro_z_rate_integral = (raw_data[11] << 8) + raw_data[10];
  this->integration_timespan = (raw_data[15] << 24) + (raw_data[14] << 16) + (raw_data[13] << 8) + raw_data[12];
  this->sonar_timestamp = (raw_data[19] << 24) + (raw_data[18] << 16) + (raw_data[17] << 8) + raw_data[16];
  this->ground_distance = (raw_data[21] << 8) + raw_data[20];
  this->gyro_temperature = (raw_data[23] << 8) + raw_data[22];
  this->quality = raw_data[24];
}

void px4flow_read_all(void)
{
  uint8_t raw_data[47];

  i2c_read_bytes_from_address(PX4FLOW_dev_handle, 0x00, 47, raw_data);

}
