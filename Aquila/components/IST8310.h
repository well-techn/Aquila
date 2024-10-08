#ifndef IST8310_h
#define IST8310_h

#include <stdio.h>
#include "esp_err.h"

#define IST8310_ADDR 0x0E
#define IST8310_WAI_REG 0x0
#define IST8310_DEVICE_ID 0x10

#define IST8310_OUTPUT_X_L_REG 0x03
#define IST8310_OUTPUT_X_H_REG 0x04
#define IST8310_OUTPUT_Y_L_REG 0x05
#define IST8310_OUTPUT_Y_H_REG 0x06
#define IST8310_OUTPUT_Z_L_REG 0x07
#define IST8310_OUTPUT_Z_H_REG 0x08

#define IST8310_CNTL1_REG 0x0A
#define IST8310_CNTL1_VAL_SINGLE_MEASUREMENT_MODE 0x01

#define IST8310_CNTL2_REG 0x0B
#define IST8310_CNTL2_VAL_SRST 1

#define IST8310_TCCNTL_REG 0x40

#define IST8310_AVGCNTL_REG 0x41
#define IST8310_AVGCNTL_VAL_XZ_0  (0)
#define IST8310_AVGCNTL_VAL_XZ_2  (1)
#define IST8310_AVGCNTL_VAL_XZ_4  (2)
#define IST8310_AVGCNTL_VAL_XZ_8  (3)
#define IST8310_AVGCNTL_VAL_XZ_16 (4)
#define IST8310_AVGCNTL_VAL_Y_0  (0 << 3)
#define IST8310_AVGCNTL_VAL_Y_2  (1 << 3)
#define IST8310_AVGCNTL_VAL_Y_4  (2 << 3)
#define IST8310_AVGCNTL_VAL_Y_8  (3 << 3)
#define IST8310_AVGCNTL_VAL_Y_16 (4 << 3)

#define IST8310_PDCNTL_REG 0x42
#define IST8310_PDCNTL_VAL_PULSE_DURATION_NORMAL 0xC0

#define IST8310_SELF_TEST_REG 0x0C

#define IST8310_CROSS_AXIS_REG 0x9C



#define IST8310_SAMPLING_PERIOD_USEC (10 * AP_USEC_PER_MSEC)

esp_err_t IST8310_selftest();
esp_err_t IST8310_communication_check();
esp_err_t IST8310_configuration();
void IST8310_request_data();
esp_err_t IST8310_read_data(uint8_t *buffer);
esp_err_t IST8310_read_cross_axis_data();




/*

#define IST8310_DEVICE_ID_A 0x10

// IST8310 register map. For details see IST8310 datasheet
#define IST8310_WHO_AM_I 0x00
#define IST8310_R_CONFA 0x0A
#define IST8310_R_CONFB 0x0B
#define IST8310_R_MODE 0x02

#define IST8310_R_XL 0x03
#define IST8310_R_XM 0x04
#define IST8310_R_YL 0x05
#define IST8310_R_YM 0x06
#define IST8310_R_ZL 0x07
#define IST8310_R_ZM 0x08

#define IST8310_AVGCNTL 0x41
#define IST8310_PDCNTL 0x42

#define IST8310_ODR_MODE 0x01 //sigle measure mode

	This device supports multiple addresses depending on the configuration of CAD0 and CAD1 
    The format of these address is ADDR_CAD0_CAD1_nBIT, Where 0 indicates tie to ground, 1 to Vdd 
    If CAD0 and CAD1 are floating, I2C address will be 0x0E / 0x1C.
*/



#endif