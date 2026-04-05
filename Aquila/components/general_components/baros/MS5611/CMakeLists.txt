idf_component_register(SRCS 
"MS5611.c"

INCLUDE_DIRS
    "." 

REQUIRES 
    wt_libs
    esp_driver_spi
    esp_driver_gpio
)
   
