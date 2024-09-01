#ifndef PCA9685_h
#define PCA9685_h

#define PCA9685_address 0x41  // I2C address for PCA9865 with A0 = 1     0x82

#define PCA9685_software_reset 0x06
#define PCA9685_Reset   0x01        // Reset the device
#define PCA9685_MODE1   0x00        // 0x00 location for Mode1 register address
#define PCA9685_MODE2   0x01        // 0x01 location for Mode2 reigster address
#define PCA9685_SUBADR1   0x02        // 0x01 location for Mode2 reigster address
#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H 0x07  /**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALL_CH_ON_L_reg   0xFA
#define PCA9685_ALL_CH_ON_H_reg   0xFB
#define PCA9685_ALL_CH_OFF_L_reg  0xFC
#define PCA9685_ALL_CH_OFF_H_reg  0xFD
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF     /**< defines the test mode to be entered */


esp_err_t PCA9685_communication_check();
esp_err_t PCA9685_init();
void PCA9685_send(uint8_t value_in_persents, uint8_t output);


#endif