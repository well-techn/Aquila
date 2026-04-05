idf_component_register(SRCS 
"IST8310.c"

INCLUDE_DIRS
    "." 

REQUIRES 
    wt_libs
    esp_driver_spi
    esp_driver_gpio
    math_operations
)
   
