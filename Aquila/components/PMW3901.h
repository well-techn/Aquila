#ifndef PMW3901_H
#define PMW3901_H

#include "esp_err.h"


#define PMW3901_PRODUCT_ID        0x00
#define PMW3901_REVISION_ID       0x01
#define PMW3901_MOTION            0x02  //REG_DATA_READY
#define PMW3901_DELTA_X_L         0x03
#define PMW3901_DELTA_X_H         0x04
#define PMW3901_DELTA_Y_L         0x05
#define PMW3901_DELTA_Y_H         0x06
#define PMW3901_SQUAL             0x07
#define PMW3901_RAW_DATA_SUM      0x08
#define PMW3901_MAX_RAW_DATA      0x09
#define PMW3901_MIN_RAW_DATA      0x0A
#define PMW3901_SHUTTER_LOWER     0x0B
#define PMW3901_SHUTTER_UPPER     0x0C

#define PMW3901_OBSERVATION       0x15
#define PMW3901_MOTION_BURST      0x16

#define PMW3901_POWER_UP_RESET    0x3A
#define PMW3901_SHUTDOWN          0x3B

#define PMW3901_RESOLUTION        0x4E  //# PAA5100 only

#define PMW3901_RAWDATA_GRAB      0x58
#define PMW3901_RAWDATA_GRAB_STATUS 0x59

#define PMW3901_ORIENTATION       0x5B

#define PMW3901_INV_PRODUCT_ID    0x5F

esp_err_t PMW3901_communication_check(void);
void PMW3901_config(void);
void PMW3901_get_raw_image(uint8_t* FBuffer);
uint8_t PMW3901_read_address(uint8_t address);
void PMW3901_write_address(uint8_t address, uint8_t value);
void PMW3901_read_motion_burst(uint8_t* buffer);



#endif