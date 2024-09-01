
#include "ve_i2c.h"
#include "TfminiS.h"
#include "ve_alldef.h"
#include "driver/uart.h"

void tfs_i2c_write(uint8_t i2c_port, uint8_t *data_to_write)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, 0x20 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_to_write,sizeof(data_to_write), ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1 / portTICK_PERIOD_MS);
    if (ret == ESP_FAIL) printf ("no req ack\n");
    i2c_cmd_link_delete(cmd);
}

void tfs_i2c_read(uint8_t i2c_port, uint8_t *where_to_read_to)
{
    esp_err_t ret; 
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, 0x20 | READ_BIT, ACK_CHECK_EN);
    
    i2c_master_start(cmd);
    i2c_master_read_byte(cmd, where_to_read_to, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, where_to_read_to, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, where_to_read_to, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, where_to_read_to, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, where_to_read_to, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, where_to_read_to, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, where_to_read_to, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, where_to_read_to, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, where_to_read_to, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(i2c_port, cmd, 1 / portTICK_PERIOD_MS);
    if (ret == ESP_FAIL) printf ("no resp ack\n");
    i2c_cmd_link_delete(cmd);
}

void tfs_set_to_UART()
{
    uint8_t command[] = {0x5A, 0x05, 0x0A, 0x00, 0x69};
    tfs_i2c_write(I2C_EXT_PORT, command);
}

void tfs_save()
{
    uint8_t command[] = {0x5A, 0x04, 0x11, 0x6F};
    tfs_i2c_write(I2C_EXT_PORT, command);
}

void tfs_reset_to_factory()
{
    uint8_t command[] = {0x5A, 0x04, 0x10, 0x6E};
    tfs_i2c_write(I2C_EXT_PORT, command);
}

void tfs_request_data()
{
    uint8_t command[] = {0x5A, 0x05, 0x00, 0x01, 0x06};
    tfs_i2c_write(I2C_EXT_PORT, command);
}

