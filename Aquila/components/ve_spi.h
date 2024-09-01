#ifndef VE_SPI_H
#define VE_SPI_H

#include <stdio.h>
#include "driver/spi_master.h"

void SPI_init ();

void SPI_change_MPUs_speed(); 

void SPI_write_byte(spi_device_handle_t test, 
                        uint8_t number_of_command_bits, uint8_t command, 
                        uint8_t number_of_address_bits, uint16_t address, 
                        uint8_t number_of_dummy_bits, 
                        uint8_t value);

uint8_t SPI_read_byte(spi_device_handle_t test,  
                          uint8_t number_of_command_bits, uint8_t command, 
                          uint8_t number_of_address_bits, uint8_t address, 
                          uint8_t number_of_dummy_bits);

void SPI_write_bytes(spi_device_handle_t test, 
                          uint8_t number_of_command_bits, uint16_t command, 
                          uint8_t number_of_address_bits, uint16_t start_address, 
                          uint8_t number_of_dummy_bits, 
                          uint8_t* where_to_take_from, uint16_t number_of_bytes);

void SPI_read_bytes(spi_device_handle_t test, 
                        uint8_t number_of_command_bits, uint16_t command, 
                        uint8_t number_of_address_bits, uint16_t start_address, 
                        uint8_t number_of_dummy_bits, 
                        uint8_t* where_to_put_to, uint16_t number_of_bytes);
#endif