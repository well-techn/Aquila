idf_component_register(SRCS 

"./engines_test.c"
"./reading_logs_from_external_flash.c"
"./gyro_calibration.c"
"./advanced_acc_calibration.c"
"./advanced_mag_calibration.c"
"./printing_calibration_coefficients.c"

INCLUDE_DIRS
      "include"

REQUIRES         
        esp_common
        freertos
        driver
        nvs_flash
        wt_libs
        general_components
        tasks
)
