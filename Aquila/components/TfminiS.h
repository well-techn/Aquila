#ifndef TFMINIS_H
#define TFMINIS_H



void tfs_i2c_write(uint8_t i2c_port, uint8_t *data_to_write);
void tfs_i2c_read(uint8_t i2c_port, uint8_t *where_to_read_to);
void tfs_set_to_UART();
void tfs_save();
void tfs_request_data();
void tfs_reset_to_factory();


#endif