idf_component_register(
    SRCS "ahrs_provider.c"
    INCLUDE_DIRS "."  
    REQUIRES 
        fusion 
        vqf 
        Madgwick
)