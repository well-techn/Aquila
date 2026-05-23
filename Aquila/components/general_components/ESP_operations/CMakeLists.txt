idf_component_register(SRCS 
"gpio_operations.c"
"NVS_operations.c"
"timers_operations.c"
"logging_operations.c"

INCLUDE_DIRS
    "." 

REQUIRES 
    wt_libs
    esp_driver_spi
    esp_driver_gpio
    esp_driver_gptimer
    nvs_flash
)
   
