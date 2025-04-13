#ifndef MS5611_h
#define MS5611_h

#include <inttypes.h>
#include "esp_err.h"

#define MS5611_ADDRESS                0x76	// если CBS на +, адрес 0х76. Если CBS на земле адрес 0x77
#define MS5611_CMD_RESET              0x1E
#define MS5611_CMD_READ_PROM          0xA0  //A2
#define MS5611_CMD_ADC_CONV 		  0x40
#define MS5611_CMD_ADC_D1 			  0x00 // ADC D1 conversion
#define MS5611_CMD_ADC_D2 			  0x10 // ADC D2 conversion
#define MS5611_CMD_ADC_READ           0x00
#define MS5611_CMD_ADC_256 			  0x00 // ADC OSR=256
#define MS5611_CMD_ADC_512 			  0x02 // ADC OSR=512
#define MS5611_CMD_ADC_1024 		  0x04 // ADC OSR=1024
#define MS5611_CMD_ADC_2048 		  0x06 // ADC OSR=2048
#define MS5611_CMD_ADC_4096 		  0x08 // ADC OSR=4096

esp_err_t MS5611_communication_check();
esp_err_t MS5611_I2C_reset();
esp_err_t MS5611_I2C_PROM_read();
void MS5611_I2C_request_D1();
void MS5611_I2C_request_D2(); 
uint32_t MS5611_I2C_read(); 

#endif