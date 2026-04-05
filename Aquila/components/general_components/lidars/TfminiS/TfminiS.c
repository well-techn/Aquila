
#include "wt_i2c.h"
#include "TfminiS.h"
#include "wt_alldef.h"
#include "esp_log.h"


extern i2c_master_dev_handle_t TFSMINI_dev_handle;
extern i2c_master_bus_handle_t i2c_internal_bus_handle;
extern i2c_master_bus_handle_t i2c_external_bus_handle;

extern char *TAG_LIDAR;

esp_err_t tfs_set_to_UART()
{
    esp_err_t ret = ESP_FAIL;
    uint8_t command[] = {0x5A, 0x05, 0x0A, 0x00, 0x69};
    ret = i2c_write_long_command(TFSMINI_dev_handle, command, 4);
    return ret;
}


esp_err_t tfs_save()
{
    esp_err_t ret = ESP_FAIL;
    uint8_t command[] = {0x5A, 0x04, 0x11, 0x6F};
    ret = i2c_write_long_command(TFSMINI_dev_handle, command, 4);
    return ret;
}


esp_err_t tfs_reset_to_factory()
{
    esp_err_t ret = ESP_FAIL;
    uint8_t command[] = {0x5A, 0x04, 0x10, 0x6E};
    ret = i2c_write_long_command(TFSMINI_dev_handle, command, 4);
    return ret;
}

esp_err_t tfminis_communication_check()
{
    esp_err_t ret = ESP_FAIL;
    ret = checking_address_at_the_bus(i2c_internal_bus_handle, TFMINIS_I2C_ADDRESS);
    if (ret == ESP_OK)  ESP_LOGI(TAG_LIDAR,"Связь с Tfmini-S установлена");
    else ESP_LOGE(TAG_LIDAR,"Связь с Tfmini-S не установлена\n"); 

    return ret;
}

esp_err_t tfs_request_data()
{
    esp_err_t ret = ESP_FAIL;
    uint8_t request_data_frame_command[] = {0x5A, 0x05, 0x00, 0x01, 0x60};   //5A 05 0A MODE SU 
    ret = i2c_write_long_command(TFSMINI_dev_handle, request_data_frame_command, 5);
    return ret;
}

esp_err_t tfs_read_result(uint8_t* where_to_write)
{
    esp_err_t ret = ESP_FAIL;
    ret = i2c_read_bytes(TFSMINI_dev_handle, 9, where_to_write);
    return ret;
}







