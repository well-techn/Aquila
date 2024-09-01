#ifndef MPU6000_h
#define MPU6000_h

#include "driver/spi_master.h"

#define MPU6000_XGOFFS_TC        0x00 // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD
#define MPU6000_YGOFFS_TC        0x01
#define MPU6000_ZGOFFS_TC        0x02
#define MPU6000_X_FINE_GAIN      0x03 // [7:0] fine gain
#define MPU6000_Y_FINE_GAIN      0x04
#define MPU6000_Z_FINE_GAIN      0x05
#define MPU6000_XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define MPU6000_XA_OFFSET_L_TC   0x07
#define MPU6000_YA_OFFSET_H      0x08
#define MPU6000_YA_OFFSET_L_TC   0x09
#define MPU6000_ZA_OFFSET_H      0x0A
#define MPU6000_ZA_OFFSET_L_TC   0x0B
#define MPU6000_SELF_TEST_X      0x0D
#define MPU6000_SELF_TEST_Y      0x0E
#define MPU6000_SELF_TEST_Z      0x0F
#define MPU6000_SELF_TEST_A      0x10
#define MPU6000_XG_OFFS_USRH     0x13  // User-defined trim values for gyroscope; supported in MPU-6050?
#define MPU6000_XG_OFFS_USRL     0x14
#define MPU6000_YG_OFFS_USRH     0x15
#define MPU6000_YG_OFFS_USRL     0x16
#define MPU6000_ZG_OFFS_USRH     0x17
#define MPU6000_ZG_OFFS_USRL     0x18
#define MPU6000_SMPLRT_DIV       0x19
#define MPU6000_CONFIG           0x1A
#define MPU6000_GYRO_CONFIG      0x1B
#define MPU6000_ACCEL_CONFIG     0x1C
#define MPU6000_FF_THR           0x1D  // Free-fall
#define MPU6000_FF_DUR           0x1E  // Free-fall
#define MPU6000_MOT_THR          0x1F  // Motion detection threshold bits [7:0]
#define MPU6000_MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MPU6000_ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define MPU6000_ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
#define MPU6000_FIFO_EN          0x23
#define MPU6000_I2C_MST_CTRL     0x24
#define MPU6000_I2C_SLV0_ADDR    0x25
#define MPU6000_I2C_SLV0_REG     0x26
#define MPU6000_I2C_SLV0_CTRL    0x27
#define MPU6000_I2C_SLV1_ADDR    0x28
#define MPU6000_I2C_SLV1_REG     0x29
#define MPU6000_I2C_SLV1_CTRL    0x2A
#define MPU6000_I2C_SLV2_ADDR    0x2B
#define MPU6000_I2C_SLV2_REG     0x2C
#define MPU6000_I2C_SLV2_CTRL    0x2D
#define MPU6000_I2C_SLV3_ADDR    0x2E
#define MPU6000_I2C_SLV3_REG     0x2F
#define MPU6000_I2C_SLV3_CTRL    0x30
#define MPU6000_I2C_SLV4_ADDR    0x31
#define MPU6000_I2C_SLV4_REG     0x32
#define MPU6000_I2C_SLV4_DO      0x33
#define MPU6000_I2C_SLV4_CTRL    0x34
#define MPU6000_I2C_SLV4_DI      0x35
#define MPU6000_I2C_MST_STATUS   0x36
#define MPU6000_INT_PIN_CFG      0x37
#define MPU6000_INT_ENABLE       0x38
#define MPU6000_DMP_INT_STATUS   0x39  // Check DMP interrupt
#define MPU6000_INT_STATUS       0x3A
#define MPU6000_ACCEL_XOUT_H     0x3B
#define MPU6000_ACCEL_XOUT_L     0x3C
#define MPU6000_ACCEL_YOUT_H     0x3D
#define MPU6000_ACCEL_YOUT_L     0x3E
#define MPU6000_ACCEL_ZOUT_H     0x3F
#define MPU6000_ACCEL_ZOUT_L     0x40
#define MPU6000_TEMP_OUT_H       0x41
#define MPU6000_TEMP_OUT_L       0x42
#define MPU6000_GYRO_XOUT_H      0x43
#define MPU6000_GYRO_XOUT_L      0x44
#define MPU6000_GYRO_YOUT_H      0x45
#define MPU6000_GYRO_YOUT_L      0x46
#define MPU6000_GYRO_ZOUT_H      0x47
#define MPU6000_GYRO_ZOUT_L      0x48
#define MPU6000_EXT_SENS_DATA_00 0x49
#define MPU6000_EXT_SENS_DATA_01 0x4A
#define MPU6000_EXT_SENS_DATA_02 0x4B
#define MPU6000_EXT_SENS_DATA_03 0x4C
#define MPU6000_EXT_SENS_DATA_04 0x4D
#define MPU6000_EXT_SENS_DATA_05 0x4E
#define MPU6000_EXT_SENS_DATA_06 0x4F
#define MPU6000_EXT_SENS_DATA_07 0x50
#define MPU6000_EXT_SENS_DATA_08 0x51
#define MPU6000_EXT_SENS_DATA_09 0x52
#define MPU6000_EXT_SENS_DATA_10 0x53
#define MPU6000_EXT_SENS_DATA_11 0x54
#define MPU6000_EXT_SENS_DATA_12 0x55
#define MPU6000_EXT_SENS_DATA_13 0x56
#define MPU6000_EXT_SENS_DATA_14 0x57
#define MPU6000_EXT_SENS_DATA_15 0x58
#define MPU6000_EXT_SENS_DATA_16 0x59
#define MPU6000_EXT_SENS_DATA_17 0x5A
#define MPU6000_EXT_SENS_DATA_18 0x5B
#define MPU6000_EXT_SENS_DATA_19 0x5C
#define MPU6000_EXT_SENS_DATA_20 0x5D
#define MPU6000_EXT_SENS_DATA_21 0x5E
#define MPU6000_EXT_SENS_DATA_22 0x5F
#define MPU6000_EXT_SENS_DATA_23 0x60
#define MPU6000_MOT_DETECT_STATUS 0x61
#define MPU6000_I2C_SLV0_DO      0x63
#define MPU6000_I2C_SLV1_DO      0x64
#define MPU6000_I2C_SLV2_DO      0x65
#define MPU6000_I2C_SLV3_DO      0x66
#define MPU6000_I2C_MST_DELAY_CTRL 0x67
#define MPU6000_SIGNAL_PATH_RESET  0x68
#define MPU6000_MOT_DETECT_CTRL   0x69
#define MPU6000_USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define MPU6000_PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define MPU6000_PWR_MGMT_2       0x6C
#define MPU6000_DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define MPU6000_DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define MPU6000_DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define MPU6000_DMP_REG_1        0x70
#define MPU6000_DMP_REG_2        0x71
#define MPU6000_FIFO_COUNTH      0x72
#define MPU6000_FIFO_COUNTL      0x73
#define MPU6000_FIFO_R_W         0x74
#define MPU6000_WHO_AM_I         0x75 // Should return 0x68

#define MPU6000_READ_FLAG        0x80

esp_err_t MPU6000_communication_check (spi_device_handle_t MPU_handle);
esp_err_t MPU6000_self_test(spi_device_handle_t MPU_handle);
esp_err_t MPU6000_init(spi_device_handle_t MPU_handle);

#endif