idf_component_register(SRCS 
"winbondW25N.c"
"madgwick.c"
"HMC5983.c"
"MPU6000.c"
"MCP23017.c"
"PCA9685.c"
"TfminiS.c"
"INA219.c"
"IST8310.c"
"FL3195.c"
"MS5611.c"
#"AES_crypto.c"
"filters.c"
"q_operations.c"
"mavlink_crc.c"
"mavlink_wt.c"
"timing.c"
"pid.c"
"advanced_math.c"
"px4flow.c"
"CRCs.c"
"FusionAhrs.c"
"FusionBias.c"
"FusionCompass.c"
"dshot_esc_encoder.c"
"NVS_operations.c"
"timers_ops.c"
"gpio_ops.c"


INCLUDE_DIRS
    "include" 


REQUIRES 
    nvs_flash
    esp_wifi 
    #mbedtls
    wt_libs
    esp_driver_i2c
    esp_driver_spi
    esp_driver_gptimer
    esp_driver_gpio
    esp_driver_rmt
    esp_driver_ledc
    esp_driver_uart
)
   
