#ifndef HMC5983_h
#define HMC5983_h

#include <stdio.h>
#include "esp_err.h"

// MEMORY MAPPING
/*
Address Location	Name 		Access
---------------------------------------------------
0x00	Configuration Register A	Read/Write
0x01	Configuration Register B	Read/Write
0x02	Mode Register			Read/Write
0x03	Data Output X MSB Register	Read
0x04	Data Output X LSB Register	Read
0x05	Data Output Z MSB Register	Read
0x06	Data Output Z LSB Register	Read
0x07	Data Output Y MSB Register	Read
0x08	Data Output Y LSB Register	Read
0x09	Status Register			Read
0x0A	Identification Register A	Read
0x0B	Identification Register B	Read
0x0C	Identification Register C	Read
0x31	Temperature Output MSB Register	Read
0x32	Temperature Output LSB Register Read
*/

#define HMC5883L_I2C_ADDR 0x1E 

#define HMC5983_CONF_A		0x00
#define HMC5983_CONF_B		0x01
#define HMC5983_MODE		0x02
#define HMC5983_OUT_X_MSB	0x03            //4 to enable autoinc
#define HMC5983_OUT_X_LSB	0x04
#define HMC5983_OUT_Z_MSB	0x05
#define HMC5983_OUT_Z_LSB	0x06
#define HMC5983_OUT_Y_MSB	0x07
#define HMC5983_OUT_Y_LSB	0x08
#define HMC5983_STATUS		0x09
#define HMC5983_ID_A		0x0A
#define HMC5983_ID_B		0x0B
#define HMC5983_ID_C		0x0C
#define HMC5983_TEMP_OUT_MSB	0x31
#define HMC5983_TEMP_OUT_LSB	0x32

#define HMC5983_AUTOINC_FLAG 0x40

esp_err_t HMC5983L_SPI_communication_check();
esp_err_t HMC5983L_SPI_init();
esp_err_t HMC5983L_SPI_selftest();
void HMC5983L_SPI_data_read(uint8_t* buffer);

#endif