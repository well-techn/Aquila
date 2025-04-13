#ifndef FL3195_h
#define FL3195_h
//https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/lights/rgbled_FL3195/rgbled_FL3195.cpp

#include <stdio.h>
#include "esp_err.h"

#define    FL3195_ADDR			                        0x54	/* I2C adress of FL3195 */
#define    FL3195_PRODUCT_ID                        0x00

#define   FL3195_SHUTDOWN_CTRL                     0x01
#define   FL3195_SHUTDOWN_CTRL_SSD_SHUTDOWN      0x00
#define   FL3195_SHUTDOWN_CTRL_SSD_NORMAL        0x01

#define   FL3195_SHUTDOWN_CTRL_SLE_SHIFT         1
#define   FL3195_SHUTDOWN_CTRL_SLE_MASK          0x03
#define   FL3195_SHUTDOWN_CTRL_SLE_DISABLE       (0 << SHUTDOWN_CTRL_SLE_SHIFT)
#define   FL3195_SHUTDOWN_CTRL_SLE_SLEEP1        (1 << SHUTDOWN_CTRL_SLE_SHIFT)
#define   FL3195_SHUTDOWN_CTRL_SLE_SLEEP2        (2 << SHUTDOWN_CTRL_SLE_SHIFT)

#define   FL3195_SHUTDOWN_CTRL_CPPM_SHIFT        3
#define   FL3195_SHUTDOWN_CTRL_CPPM_1X           (0 << SHUTDOWN_CTRL_CPPM_SHIFT)
#define   FL3195_SHUTDOWN_CTRL_CPPM_1P5X         (1 << SHUTDOWN_CTRL_CPPM_SHIFT)
#define   FL3195_SHUTDOWN_CTRL_EN_SHIFT          4
#define   FL3195_SHUTDOWN_CTRL_EN_MASK           0xf
#define   FL3195_SHUTDOWN_CTRL_EN1               (1 << SHUTDOWN_CTRL_EN_SHIFT)
#define   FL3195_SHUTDOWN_CTRL_EN2               (2 << SHUTDOWN_CTRL_EN_SHIFT)
#define   FL3195_SHUTDOWN_CTRL_EN3               (4 << SHUTDOWN_CTRL_EN_SHIFT)
#define   FL3195_SHUTDOWN_CTRL_EN4               (8 << SHUTDOWN_CTRL_EN_SHIFT)

#define   FL3195_MODE_CONFIG                       0x02
#define   FL3195_MODE_CONFIG_LM_SHIFT            0
#define   FL3195_MODE_CONFIG_LM_MASK             0x3
#define   FL3195_MODE_CONFIG_LM_SINGLE           (0 << MODE_CONFIG_LM_SHIFT)
#define   FL3195_MODE_CONFIG_LM_RGBW             (1 << MODE_CONFIG_LM_SHIFT)
#define   FL3195_MODE_CONFIG_LM_SRGBY            (2 << MODE_CONFIG_LM_SHIFT)

#define   FL3195_MODE_CONFIG_OUT_MODE_SHIFT      4
#define   FL3195_MODE_CONFIG_OUT_MODE_MASK       0xf
#define   FL3195_MODE_CONFIG_OUT_MODE_CURRENT    0
#define   FL3195_MODE_CONFIG_OUT_MODE_PATTERN    1
#define   FL3195_MODE_CONFIG_OUT_MODE(n,m)       ((m) << ((n-1) + MODE_CONFIG_OUT_MODE_SHIFT))
#define   FL3195_MODE_CONFIG_OUT1_MODE(m)        MODE_CONFIG_OUT_MODE(1,(m))
#define   FL3195_MODE_CONFIG_OUT2_MODE(m)        MODE_CONFIG_OUT_MODE(2,(m))
#define   FL3195_MODE_CONFIG_OUT3_MODE(m)        MODE_CONFIG_OUT_MODE(3,(m))
#define   FL3195_MODE_CONFIG_OUT4_MODE(m)        MODE_CONFIG_OUT_MODE(4,(m))

#define     FL3195_CHARGE_PUMP1                      0x3
#define     FL3195_CHARGE_PUMP1_CPM_SHIFT        0
#define     FL3195_CHARGE_PUMP1_CPM_MASK         0x3
#define     FL3195_CHARGE_PUMP1_CPM_AUTO         (0 << CHARGE_PUMP1_CPM_SHIFT)
#define     FL3195_CHARGE_PUMP1_CPM_1X           (1 << CHARGE_PUMP1_CPM_SHIFT)
#define     FL3195_CHARGE_PUMP1_CPM_1P5X         (2 << CHARGE_PUMP1_CPM_SHIFT)
#define     FL3195_CHARGE_PUMP1_DEFAULT          (8 << 2)

#define FL3195_CHARGE_PUMP2                      0x4
#define     FL3195_CHARGE_PUMP2_CPDE_SHIFT       0
#define     FL3195_CHARGE_PUMP2_CPDE_MASK        0xf
#define     FL3195_CHARGE_PUMP2_CPDE_ENABLE      0
#define     FL3195_CHARGE_PUMP2_CPDE_DISABLE     1
#define     FL3195_CHARGE_PUMP2_CPDE(n,m)        ((m) << ((n-1) + CHARGE_PUMP2_CPDE_SHIFT))

#define     FL3195_CHARGE_PUMP2_HRT_SHIFT        4
#define     FL3195_CHARGE_PUMP2_MASK             0x3
#define     FL3195_CHARGE_PUMP2_HRT(m)           ((m) << + CHARGE_PUMP2_HRT_SHIFT)
#define     FL3195_CHARGE_PUMP2_HRT_50MV         CHARGE_PUMP2_HRT(0)
#define     FL3195_CHARGE_PUMP2_HRT_100MV        CHARGE_PUMP2_HRT(1)
#define     FL3195_CHARGE_PUMP2_HRT_125MV        CHARGE_PUMP2_HRT(2)
#define     FL3195_CHARGE_PUMP2_HRT_150MV        CHARGE_PUMP2_HRT(3)
#define     FL3195_CHARGE_PUMP2_HRT_175MV        CHARGE_PUMP2_HRT(4)
#define     FL3195_CHARGE_PUMP2_HRT_200MV        CHARGE_PUMP2_HRT(5)
#define     FL3195_CHARGE_PUMP2_HRT_250MV        CHARGE_PUMP2_HRT(6)
#define     FL3195_CHARGE_PUMP2_HRT_300MV        CHARGE_PUMP2_HRT(7)

#define FL3195_CURRENT_BAND  0x5
#define   FL3195_CURRENT_BAND_CB_SHIFT           0
#define   FL3195_CURRENT_BAND_CB_MASK_           0x3
#define   FL3195_CURRENT_BAND_CB2_SHIFT          2

#define   FL3195_CURRENT_BAND_CB_P25             0
#define   FL3195_CURRENT_BAND_CB_P5              1
#define   FL3195_CURRENT_BAND_CB_P75             2
#define   FL3195_CURRENT_BAND_CB_1P0             3
#define FL3195_CURRENT_BAND_CB(n,m)              (m) << ((n-1) * CURRENT_BAND_CB2_SHIFT)

#define FL3195_OUT_CURRENT1                      0x10
#define FL3195_OUT_CURRENT2                      0x21
#define FL3195_OUT_CURRENT3                      0x32
#define FL3195_OUT_CURRENT4                      0x40

#define FL3195_COLOR_UPDATE                      0x50
#define FL3195_COLOR_UPDATE_KEY                  0xC5


#define RUN_MODE (SHUTDOWN_CTRL_SSD_NORMAL  | \
		  SHUTDOWN_CTRL_SLE_DISABLE | \
		  SHUTDOWN_CTRL_CPPM_1P5X)

#define L1_L3_EN  (SHUTDOWN_CTRL_EN1         | \
		   SHUTDOWN_CTRL_EN2         | \
		   SHUTDOWN_CTRL_EN3)

#define     FL3195_RESET            			0x5F


esp_err_t FL3195_communication_check();
esp_err_t FL3195_configuration();
void FL3195_set_pattern(uint8_t pulse_duration, uint8_t color_red,uint8_t color_green,uint8_t color_blue);

#endif