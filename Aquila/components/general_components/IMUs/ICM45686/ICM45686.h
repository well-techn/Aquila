#ifndef ICM45686_h
#define ICM45686_h

#include "driver/spi_master.h"

//https://github.com/SlimeVR/SlimeVR-Tracker-nRF/blob/main/src/sensor/imu/ICM45686.c

#define ICM45686_GYRO_MAX_RATE             250
#define ICM45686_GYRO_SENSITIVITY          (32767.0 / ICM45686_GYRO_MAX_RATE)
#define ICM45686_ACCEL_MAX_SCALE           4
#define ICM45686_ACCEL_SENSITIVITY         (32767.0 / ICM45686_ACCEL_MAX_SCALE)
#define ICM45686_ODR                       800
#define ICM45686_LPF_DIVIDER               128


#define ICM45686_WHO_I_AM_RESPONSE         0xE9

// User Bank 0
#define ICM45686_ACCEL_DATA_X1_UI          0x00
#define ICM45686_GYRO_DATA_X1_UI           0x06
#define ICM45686_TEMP_DATA1_UI             0x0C
#define ICM45686_PWR_MGMT0                 0x10
#define ICM45686_FIFO_COUNT_0              0x12
#define ICM45686_FIFO_DATA                 0x14
#define ICM45686_INT1_CONFIG0              0x16
#define ICM45686_INT1_CONFIG1              0x17
#define ICM45686_INT1_CONFIG2              0x18
#define ICM45686_INT1_STATUS0              0x19
#define ICM45686_ACCEL_CONFIG0             0x1B
#define ICM45686_GYRO_CONFIG0              0x1C
#define ICM45686_FIFO_CONFIG0              0x1D
#define ICM45686_FIFO_CONFIG1_0            0x1E
#define ICM45686_FIFO_CONFIG3              0x21
#define ICM45686_TMST_WOM_CONFIG           0x23
#define ICM45686_RTC_CONFIG                0x26
#define ICM45686_IOC_PAD_SCENARIO_AUX_OVRD 0x30
#define ICM45686_IOC_PAD_SCENARIO_OVRD     0x31 // see application note
#define ICM45686_REG_MISC1                 0x35
#define ICM45686_WHO_I_AM                  0x72
#define ICM45686_IREG_ADDR_15_8            0x7C
#define ICM45686_IREG_DATA                 0x7E
#define ICM45686_REG_MISC2                 0x7F

// User Bank IPREG_BAR
#define ICM45686_IPREG_BAR                 0xA0 // MSB
#define ICM45686_IPREG_BAR_REG_58          0x3A
#define ICM45686_IPREG_BAR_REG_59          0x3B
#define ICM45686_IPREG_BAR_REG_60          0x3C
#define ICM45686_IPREG_BAR_REG_61          0x3D

// User Bank IPREG_TOP1
#define ICM45686_IPREG_TOP1                0xA2 // MSB
#define ICM45686_SMC_CONTROL_0             0x58
#define ICM45686_SREG_CTRL                 0x67
#define ICM45686_ACCEL_WOM_X_THR           0x7E
#define ICM45686_ACCEL_WOM_Y_THR           0x7F
#define ICM45686_ACCEL_WOM_Z_THR           0x80

// User Bank IPREG_SYS1
#define ICM45686_IPREG_SYS1                0xA4 // MSB
#define ICM45686_IPREG_SYS1_REG_166        0xA6
#define ICM45686_IPREG_SYS1_REG_172        0xAC


// User Bank IPREG_SYS2
#define ICM45686_IPREG_SYS2                0xA5 // MSB
#define ICM45686_IPREG_SYS2_REG_123        0x7B
#define ICM45686_IPREG_SYS2_REG_129        0x81
#define ICM45686_IPREG_SYS2_REG_131        0x83




esp_err_t ICM45686_communication_check(spi_device_handle_t MPU_handle);
esp_err_t ICM45686_init(spi_device_handle_t MPU_handle,
                        uint16_t sampling_freq_hz,
                        uint16_t integrated_lpf_value_hz,
                        uint16_t accel_full_scale_g,
                        uint16_t gyro_full_scale_dps);
void ICM45686_indirect_reg_write(spi_device_handle_t MPU_handle, uint8_t shift, uint8_t indir_reg_address, uint8_t data_to_write);
uint8_t ICM45686_indirect_reg_read(spi_device_handle_t MPU_handle, uint8_t shift, uint8_t indir_reg_address);
void ICM45686_soft_reset(spi_device_handle_t MPU_handle);
#endif