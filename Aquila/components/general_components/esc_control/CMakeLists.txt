idf_component_register(
    SRCS "esc_control_provider.c"
    INCLUDE_DIRS "."  
    REQUIRES 
        dshot
        pwm
)