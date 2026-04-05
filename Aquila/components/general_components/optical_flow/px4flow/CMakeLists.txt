idf_component_register(SRCS 
"px4flow.c"

INCLUDE_DIRS
    "." 

REQUIRES 
    wt_libs
    esp_driver_spi
    esp_driver_gpio
)
   
