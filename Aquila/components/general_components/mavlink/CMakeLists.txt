idf_component_register(SRCS 
"mavlink_crc.c"
"mavlink_wt.c"

INCLUDE_DIRS
    "." 

REQUIRES 
    wt_libs
    esp_driver_spi
    esp_driver_gpio
)
   
