idf_component_register(SRCS 
"winbondW25N.c"

INCLUDE_DIRS
    "." 

REQUIRES 
    wt_libs
    esp_driver_spi
    esp_driver_gpio
)
   
