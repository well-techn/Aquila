idf_component_register(SRCS 
"advanced_math.c"
"CRCs.c"
"filters.c"
"q_operations.c"

INCLUDE_DIRS
    "." 

REQUIRES 
    wt_libs
    esp_driver_spi
    esp_driver_gpio
    esp-dsp
)
   
