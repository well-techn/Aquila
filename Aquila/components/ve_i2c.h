#ifndef VE_I2C_H
#define VE_I2C_H

#include <stdio.h>
//#include "driver/i2c.h"
#include "driver/i2c_master.h"

//void i2c_init(uint8_t sda_pin,uint8_t scl_pin, uint32_t freq, uint8_t i2c_port);
//esp_err_t i2c_write_byte_to_address(uint8_t i2c_port, uint8_t device_address, uint8_t reg_address, uint8_t data_to_write);
//esp_err_t i2c_write_bytes_to_address(uint8_t i2c_port, uint8_t device_address, uint8_t reg_address, uint8_t *data_to_write);
//esp_err_t i2c_write_byte_to_address_x16(uint8_t i2c_port, uint8_t device_address, uint16_t reg_address_x16, uint8_t data_to_write);
//esp_err_t i2c_write_2_bytes_to_address_x16(uint8_t i2c_port, uint8_t device_address, uint16_t reg_address_x16, uint16_t data_to_write);
//esp_err_t i2c_write_4_bytes_to_address_x16(uint8_t i2c_port, uint8_t device_address, uint16_t reg_address_x16, uint32_t data_to_write);

//uint8_t i2c_read_byte_from_address(uint8_t i2c_port, uint8_t device_address, uint8_t reg_address);
//uint8_t i2c_read_byte_from_address_x16(uint8_t i2c_port, uint8_t device_address, uint16_t reg_address_x16);
void i2c_init_internal(uint8_t sda_pin,uint8_t scl_pin, uint8_t i2c_port);
void i2c_init_external(uint8_t sda_pin,uint8_t scl_pin, uint8_t i2c_port);
esp_err_t i2c_write_byte_to_address_NEW(i2c_master_dev_handle_t i2c_dev, uint8_t reg_address, uint8_t data_to_write);
esp_err_t i2c_write_2_bytes_to_address_NEW(i2c_master_dev_handle_t i2c_dev, uint8_t reg_address, uint8_t MSB, uint8_t LSB);
esp_err_t i2c_write_bytes_to_address_NEW(i2c_master_dev_handle_t i2c_dev, uint8_t reg_address, uint8_t number_of_bytes, uint8_t *data_to_write);
uint8_t i2c_read_byte_from_address_NEW(i2c_master_dev_handle_t i2c_dev, uint8_t reg_address);
uint16_t i2c_read_2_bytes_from_address_NEW(i2c_master_dev_handle_t i2c_dev, uint8_t reg_address);
esp_err_t i2c_read_bytes_from_address_NEW(i2c_master_dev_handle_t i2c_dev, uint8_t read_start_address, uint8_t number_of_bytes_to_read, uint8_t* where_to_put_to);
void checking_address_at_the_bus(i2c_master_bus_handle_t bus_handle, uint8_t device_address);

#endif
