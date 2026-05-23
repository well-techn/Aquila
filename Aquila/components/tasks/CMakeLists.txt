idf_component_register(SRCS 

"./MS5611_read_and_process.c"
"./error_code_LED_blinking.c"
"./mag_read_and_process_data.c"
"./gps_read_and_process_data.c"
"./RC_read_and_process_data.c"
"./RC_emulation.c"
"./send_data_to_RC.c"
"./lidar_read_and_process_data.c"
"./MCP23017_monitoring_and_control.c"
"./PCA9685_control.c"
"./writing_logs_to_flash.c"
"./blinking_flight_lights.c"
"./performance_monitor.c"
"./INA219_read_and_process_data.c"
"./send_telemetry_via_mavlink.c"
"./send_telemetry_via_MSP.c"
"./init.c"
"./main_flying_cycle.c"
"./px4flow_read_and_process_data.c"
"./configuration_mode.c"
"./configuration_mode_telnet.c"
"./emergency_mode.c"
"./sending_something_over_wifi.c"
"./fft.c"

INCLUDE_DIRS
"./include" 

REQUIRES         
# Основные системные компоненты
        nvs_flash
        esp_system
        esp_event
        esp_common
        esp_partition
        freertos
        esp-dsp
        esp_ringbuf        
        
# Драйверы
        esp_driver_i2c
        esp_driver_spi
        esp_driver_gptimer
        esp_driver_gpio
        #esp_driver_rmt
        #esp_driver_ledc
        esp_driver_uart
        esp_driver_usb_serial_jtag
        
# Сеть и безопасность
        esp_wifi
        esp_event
        lwip
        mbedtls
        
# Локальные компоненты
        wt_libs
        service_tasks
        MPU6000
        ICM45686
        MS5611
        IST8310
        MCP23017
        PCA9685
        px4flow
        PMW3901
        INA219
        TfminiS
        FL3195
        winbondW25N
        ahrs_algorithms
        mavlink
        ESP_operations
        math_operations
        pid
        esc_control
)
