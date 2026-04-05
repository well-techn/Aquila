idf_component_register(SRCS 
"dshot.c"
"dshot_esc_encoder.c"

INCLUDE_DIRS
    "." 

REQUIRES 
    wt_libs
    esp_driver_rmt
)
   
