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
"AES_crypto.c"
"filters.c"
"q_operations.c"
"mavlink_crc.c"
"mavlink_wt.c"
"timing.c"
"pid.c"


INCLUDE_DIRS 
"./include"
"${IDF_PATH}/"
"${PROJECT_DIR}/components/wt_libs/include/"
"${IDF_PATH}/components/esp_driver_spi/include/driver/"
"${IDF_PATH}/components/esp_driver_i2c/include/driver/"
REQUIRES mbedtls)
