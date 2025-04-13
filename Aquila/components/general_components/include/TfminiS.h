#ifndef TFMINIS_H
#define TFMINIS_H

#define TFMINIS_I2C_ADDRESS (0x10)

esp_err_t tfs_set_to_UART();
esp_err_t tfs_save();
esp_err_t tfs_request_data();
esp_err_t tfs_read_result(uint8_t* where_to_write);
esp_err_t tfs_reset_to_factory();
esp_err_t tfminis_communication_check();


#endif