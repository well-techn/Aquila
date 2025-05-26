idf_component_register(SRCS 

"./engines_test.c"
"./reading_logs_from_external_flash.c"
"./imu_calibration.c"
"./advanced_acc_calibration.c"
"./advanced_mag_calibration.c"

INCLUDE_DIRS
"./include" 
"${IDF_PATH}/"
"${PROJECT_DIR}/components/general_components/include/"
"${PROJECT_DIR}/components/service_tasks/include/"
"${PROJECT_DIR}/components/wt_libs/include"
"${IDF_PATH}/components/esp_driver_spi/include/driver/"
"${IDF_PATH}/components/esp_driver_i2c/include/driver/"
"${IDF_PATH}/components/esp_driver_gpio/include/"
"${IDF_PATH}/components/esp_driver_uart/include/"
"${IDF_PATH}/components/esp_driver_gptimer/include/"
"${IDF_PATH}/components/esp_driver_ledc/include/"
"${IDF_PATH}/components/esp_partition/include/"
"${IDF_PATH}/components/nvs_flash/include/"
"${IDF_PATH}/components/esp_driver_usb_serial_jtag/include/driver/"
REQUIRES mbedtls)
