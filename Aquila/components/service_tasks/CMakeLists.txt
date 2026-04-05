idf_component_register(SRCS 

"./engines_test.c"
"./reading_logs_from_external_flash.c"
"./gyro_calibration.c"
"./advanced_acc_calibration.c"
"./advanced_mag_calibration.c"
"./printing_calibration_coefficients.c"
"./clearing_NVS.c"

INCLUDE_DIRS
      "include"

REQUIRES         
        esp_common
        esp_driver_i2c
        esp_driver_spi
        esp_driver_gptimer
        esp_driver_gpio
        esp_driver_rmt
        esp_driver_ledc
        esp_driver_uart
        esp_driver_usb_serial_jtag
        freertos
        nvs_flash
        
        wt_libs
        tasks

        MPU6000
        MS5611
        IST8310
        MCP23017
        PCA9685
        px4flow
        PMW3901
        INA219
        TfminiS
        FL3195
        winbondW25N
        #fusion
        mavlink
        math_operations
        esc_control
)
