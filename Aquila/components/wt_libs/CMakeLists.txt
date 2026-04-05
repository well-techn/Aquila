idf_component_register(SRCS 
        "wt_i2c.c"
        "wt_spi.c"

INCLUDE_DIRS 
        "include" 

REQUIRES 
        esp_driver_i2c
        esp_driver_spi
        esp_common
)
