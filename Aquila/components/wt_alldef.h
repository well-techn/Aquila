#ifndef wt_alldefS_H
#define wt_alldefS_H

//#define USING_W25N 
#define USING_HOLYBRO_M9N
#define USING_LIDAR_UART
//#define USING_PERFORMANCE_MESUREMENT


//defining SPI pins and parameters
#define IMU_SPI_MOSI            (16)
#define IMU_SPI_MISO            (14)
#define IMU_SPI_SCLK            (17)
#define IMU_SPI                 (SPI3_HOST)       //HSPI - 2, VSPI - 3
#define GPIO_CS_MPU6000_1       (18)
#define GPIO_CS_MPU6000_2       (15)
#define GP_SPI                  (SPI2_HOST)        //HSPI - 2, VSPI - 3
#define GP_SPI_MOSI             (39)
#define GP_SPI_MISO             (42)
#define GP_SPI_SCLK             (40)
#define GPIO_CS_W25N01          (41) 
#define GPIO_CS_PMW3901         (7)
#define SPI_READ_FLAG           (0x80)

//defining internal i2c pins and parameters
#define I2C_INT_SDA                       (34) 
#define I2C_INT_SCL                       (48)
#define I2C_PCA9685_FREQ_HZ               (400000)
#define I2C_MCP23017_FREQ_HZ              (400000)
#define I2C_INA219_FREQ_HZ                (400000)
#define I2C_INT_MASTER_TX_BUF_DISABLE     (0)                           
#define I2C_INT_MASTER_RX_BUF_DISABLE     (0)                                           
#define ACK_CHECK_EN                      (0x01)                       
#define ACK_CHECK_DIS                     (0)                      
#define ACK_VAL                           (0)                            
#define NACK_VAL                          (0x01)                           
#define I2C_INT_PORT                      (0)

//defining external i2c pins and parameters

#define I2C_EXT_SDA                       (33) 
#define I2C_EXT_SCL                       (47)
#define I2C_EXT_MASTER_TX_BUF_DISABLE     (0)     
#define I2C_IST8310_FREQ_HZ               (400000)
#define I2C_FL3195_FREQ_HZ                (400000)                      
#define I2C_EXT_MASTER_RX_BUF_DISABLE     (0)                                               
#define I2C_EXT_PORT                      (1)

//defining remote control UART pins and parameters
#define REMOTE_CONTROL_UART                           (2)
#define RC_UART_BAUD_RATE                             (57600) 
#define NUMBER_OF_BYTES_TO_RECEIVE_FROM_RC            (13)
#define NUMBER_OF_BYTES_TO_SEND_TO_RC                 (15)
#define RC_RX_UART_BUFF_SIZE                          (256)
#define RC_TX_UART_BUFF_SIZE                          (256)
#define RC_UART_TX_PIN                                (2) 
#define RC_UART_RX_PIN                                (1) 
#define RC_UART_RTS_PIN                               (UART_PIN_NO_CHANGE)
#define RC_UART_CTS_PIN                               (UART_PIN_NO_CHANGE)
#define RC_UART_PATTERN_DETECTION_QUEUE_SIZE          (10)
#define RC_MESSAGE_HEADER                             (0x4E)
#define RC_NO_COMM_DELAY_MAIN_CYCLES                  (3000)
#define RC_NO_COMM_THROTTLE_HOVER_VALUE               (7500)            
struct data_from_rc_to_main_struct {                          //structure to pass values from RC processing task to main
    float received_throttle;
    float received_pitch;
    float received_roll;
    float received_yaw;
    uint16_t mode;
    uint8_t trim_pitch;
    uint8_t trim_roll;
    uint8_t engines_start_flag;
    uint8_t altitude_hold_flag;
  };
struct data_from_main_to_rc_struct {                   //structure to pass data from main to RC send
    float pitch;
    float roll;
    float yaw;
    uint16_t power_voltage_value;
    uint16_t altitude;
  };

#define RC_FILTER_COEFF                         (0.85) //remote_control_data.received_throttle = remote_control_data.received_throttle * RC_FILTER_COEFF + rc_throttle_old * (1 - RC_FILTER_COEFF);

//defining GPS UART pins and parameters
#define GPS_UART                                (0)
#define GPS_UART_BAUD_RATE                      (38400)
#define NUMBER_OF_BYTES_TO_RECEIVE_FROM_GPS     (140)
#define GPS_UART_BUF_SIZE                       (512)
#define GPS_UART_TX_PIN                         (5) //A1
#define GPS_UART_RX_PIN                         (4) //A0
#define GPS_UART_RTS_PIN                        (UART_PIN_NO_CHANGE)
#define GPS_UART_CTS_PIN                        (UART_PIN_NO_CHANGE)
#define GPS_UART_PATTERN_DETECTION_QUEUE_SIZE   (10)
struct data_from_gps_to_main_struct {                             //structure to pass values from GPS processing task to main
    uint64_t latitude_d;
    uint64_t longtitude_d;
    uint8_t status;
  };
#ifdef USING_LIDAR_UART
//defining tfsmini UART pins and parameters
  #define LIDAR_UART                                (1)
  #define LIDAR_UART_BAUD_RATE                      (115200)    //115200
  #define NUMBER_OF_BYTES_TO_RECEIVE_FROM_LIDAR     (140)
  #define LIDAR_UART_BUF_SIZE                       (256)
  #define LIDAR_UART_TX_PIN                         (43) 
  #define LIDAR_UART_RX_PIN                         (44) 
  #define LIDAR_UART_RTS_PIN                        (UART_PIN_NO_CHANGE)
  #define LIDAR_UART_CTS_PIN                        (UART_PIN_NO_CHANGE)
  #define LIDAR_UART_PATTERN_DETECTION_QUEUE_SIZE   (10)
  struct data_from_lidar_to_main_struct {                             //structure to pass values from tfs processing task to main
      uint16_t height;
      uint16_t strength;
    };
#define LIDAR_RATE_HZ                               (50)
#endif

//defining onboard PWM pins and parameters
#define ENGINE_PWM_DUTY_RESOLUTION              (LEDC_TIMER_14_BIT) // duty resolution to 14 bits
#define ENGINE_PWM_MIN_DUTY                     (6553) // Set duty to 1ms: ((2 ^ 14) - 1) * 400 (Hz) = 6553,2
#define ENGINE_PWM_FREQUENCY                    (400) // Frequency in Hertz. 
#define ENGINE_PWM_TIMER                        (LEDC_TIMER_0)
#define ENGINE_PWM_MODE                         (LEDC_LOW_SPEED_MODE)
#define ENGINE_PWM_OUTPUT_0_PIN                 (9)
#define ENGINE_PWM_OUTPUT_1_PIN                 (10)
#define ENGINE_PWM_OUTPUT_2_PIN                 (11) 
#define ENGINE_PWM_OUTPUT_3_PIN                 (12)
#define TIME_TO_KEEP_RUNNING_AT_CHECK_MS        (10000)
#define LENGTH_OF_ESC_FILTER                    (6)  //10


//defining general pins to use
#define MPU6000_1_INTERRUPT_PIN                 (21)
#define MPU6000_2_INTERRUPT_PIN                 (13) 
#define LED_RED                                 (38)
#define LED_BLUE                                (0)
#define LED_GREEN                               (3)   
#define MCP23017_INTERRUPT_PIN                  (37)
#define GREEN_FLIGHT_LIGHTS                     (8)
#define RED_FLIGHT_LIGHTS                       (36)


//MCP23017 commands
#define MCP23017_READ_COMMAND                   (0b10000000)
#define MCP23017_SET_OUTPUT_COMMAND             (0b01000000)
#define MCP23017_CLEAR_OUTPUT_COMMAND           (0b00100000)

//defining madgwick and general math parameters
#define SMPL                                    (0)                //IMU sample rate = 1000Hz/ 1+SMPL
#define PI                                      (3.1415926536)
#define MADGWICK_BETA                           (0.2) //0.999

//defining logging to external flash memory parameters
#define LOGS_BYTES_PER_STRING                   (79) 

//defining general SW parameters
#define SUSPENSION_TIMER_DELAY_SEC              (1)          //in seconds
#define IMU_SUSPENSION_TIMER_DELAY_MS           (2)          //in mseconds
#define NUMBER_OF_IMU_CALIBRATION_COUNTS        (8000)  
#ifdef USING_MAG_DATA                                      
  #define NUMBER_OF_MAGNETOMETER_CALIBRATION_COUNTS        (15000)  
#else
  #define NUMBER_OF_MAGNETOMETER_CALIBRATION_COUNTS         (1)
#endif
#define PID_LOOPS_RATIO                         (5)                       //relation between internal fast rate loop and external angle loop



#define A0                                      (4)
#define A1                                      (5)
#define A2                                      (6)
#define A3                                      (7)

//tasks' stack sizes
#define MCP23017_MONITORING_AND_CONTROL_STACK_SIZE    (4096)
#define PCA9685_CONTROL_STACK_SIZE                    (4096)
#define MAG_READ_DATA_AND_SEND_TO_MAIN_STACK_SIZE     (4096)
#define MAIN_FLYING_CYCLE_STACK_SIZE                  (8192)
#define MONITORING_PINS_INTERRUPT_QUEUE_STACK_SIZE    (4096)
#define GPS_READ_AND_PROCESS_DATA_STACK_SIZE          (4096)
#define RC_READ_AND_PROCESS_DATA_STACK_SIZE           (4096)
#define SEND_DATA_TO_RC_STACK_SIZE                    (4096)
#define BLINKING_FLIGHT_LIGHTS_STACK_SIZE             (2048)
#define LIDAR_READ_AND_PROCESS_DATA_STACK_SIZE        (4096)
#define INA219_READ_AND_PROCESS_DATA_STACK_SIZE       (4096) 
#define MAG_READ_AND_PROCESS_DATA_STACK_SIZE          (4096)
#define RGB_LED_CONTROL_STACK_SIZE                    (2048)
#define WRITING_LOGS_TO_FLASH_STACK_SIZE              (4096)





#endif