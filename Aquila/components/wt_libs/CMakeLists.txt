idf_component_register(SRCS 
"wt_i2c.c"
"wt_spi.c"

INCLUDE_DIRS 
"./include" 
"${IDF_PATH}/"
"${PROJECT_DIR}/components/general_components/include/"
"${IDF_PATH}/components/esp_driver_spi/include/driver/"
"${IDF_PATH}/components/esp_driver_i2c/include/driver/"
REQUIRES mbedtls)
