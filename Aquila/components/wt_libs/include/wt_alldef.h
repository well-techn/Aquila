#ifndef WT_ALLDEFS_H
#define WT_ALLDEFS_H

#include <inttypes.h>

#define FW_VERSION          "2025-12"

#define USING_W25N                                                //включаем в код функционал, связанный с записью логов во внешнюю флеш
//#define USING_MAGNETOMETER                                        //активируем использование магнетометра на модуле HOLYBRO M9N  
#define USING_FL3195                                              //активируем использование RGB светодиода на модуле HOLYBRO M9N
#define USING_GPS                                                 //активируем использование GPS на модуле HOLYBRO M9N 
#define USING_TFMINIS_I2C                                         //активируем использование лидара Benewake Tfmini-S
//#define USING_PERFORMANCE_MESUREMENT                            //запускаем задачу, которая выводит на печать процент занимаемого процессорного времени по каждой задаче
#define USING_MS5611
#define BATTERY_COMPENSATION
#define USING_MAVLINK_TELEMETRY
//#define NO_RSSI
//#define MEMORY_CONSUMPTION_MESUREMENT
#define TELNET_CONF_MODE
//#define USING_PX4FLOW
#define PREFLIGHT_POWER_CHECKUP
//#define WIFI_INFLIGHT_TEST

//набор дефайнов, определяющий какие данные будут записываться в "черный ящик" внешней флеш-памяти
#ifdef USING_W25N
  //#define LOGGING_ACCEL_1
  //#define LOGGING_ACCEL_2
 // #define LOGGING_GYRO_1
  //#define LOGGING_GYRO_2
  //#define LOGGING_AVG_ACCEL
  //#define LOGGING_AVG_GYRO
  //#define LOGGING_QUATERNION
  //#define LOGGING_ANGLES

  

  #ifdef USING_TFMINIS_I2C
    #define LOGGING_LIDAR_PID
    #define LOGGING_LIDAR_ALTITUDE_CM
  #endif
  #ifdef USING_MS5611 
    //#define LOGGING_BARO_ALTITUDE_CM
    //#define LOGGING_KALMAN_ALTITUDE_CM
    //#define LOGGING_KALMAN_VELOCITY
    //#define LOGGING_BARO_PID
  #endif
  #define LOGGING_ALTITUDE_SETPOINT_CM
  #define LOGGING_VOLTAGE_mV
  //#define LOGGING_CURRENT_cA
  //#define LOGGING_RC_COMMANDS
  #define LOGGING_ENGINES
  #define LOGGING_ENGINES_FILTERED
  #define LOGGING_FLAGS
  //#define LOGGING_RSSI_LEVEL
#endif

//GPIO и параметры SPI для подключения IMU
#define IMU_SPI                 (SPI3_HOST)                         //HSPI - 2, VSPI - 3
#define IMU_SPI_MOSI            (16)                                //GPIO на который подключен MOSI SPI для IMU
#define IMU_SPI_MISO            (14)                                //GPIO на который подключен MISO SPI для IMU
#define IMU_SPI_SCLK            (17)                                //GPIO на который подключен SCLK SPI для IMU
#define GPIO_CS_MPU6000_1       (18)                                //GPIO для CS IMU1
#define GPIO_CS_MPU6000_2       (15)                                //GPIO для CS IMU2
#define IMU_CONFIG_SPI_FREQ_HZ  (1000000)                           //частота SPI при настройке MPU6000
#define IMU_WORK_SPI_FREQ_HZ    (20000000)                          //частота SPI при чтении данных с MPU6000

//GPIO и параметры SPI для подключения флэш-памяти W25N
#define GP_SPI                  (SPI2_HOST)                         //HSPI - 2, VSPI - 3
#define GP_SPI_MOSI             (39)                                //GPIO на который подключен MOSI SPI для внешней флеш 
#define GP_SPI_MISO             (42)
#define GP_SPI_SCLK             (40)
#define GPIO_CS_W25N01          (41)
#define W25N01_SPI_FREQ_HZ      (75000000)                          //частота SPI для флэш памяти для логов 

#define SPI_READ_FLAG           (0x80)                              //используется для чтения регистров

//GPIO и параметры "внутреннего" I2C, куда подключены PCA9685, MCP23017, INA219, TFMini-S
#define I2C_INT_SDA                       (34)                      //GPIO для SDA 
#define I2C_INT_SCL                       (48)                      //GPIO для SCL
#define I2C_PCA9685_FREQ_HZ               (400000)                  //скорости I2С для соответствующих компонентов
#define I2C_MCP23017_FREQ_HZ              (400000)
#define I2C_INA219_FREQ_HZ                (400000)
#define I2C_TFMINIS_FREQ_HZ               (400000)
#define I2C_PX4FLOW_FREQ_HZ               (100000)
#define I2C_INT_MASTER_TX_BUF_DISABLE     (0)                       //параметры для инициализации интерфейса                           
#define I2C_INT_MASTER_RX_BUF_DISABLE     (0)                                           
#define ACK_CHECK_EN                      (0x01)                       
#define ACK_CHECK_DIS                     (0)                      
#define ACK_VAL                           (0)                            
#define NACK_VAL                          (0x01)                           
#define I2C_INT_PORT                      (0)                       //номер используемого порта I2C для "внутренней" шины 

//GPIO и параметры "внешнего" I2C, куда подключены IST8310 и FL3195 модуля Holybro M9N
#define I2C_EXT_SDA                       (33)                      //GPIO для SDA  
#define I2C_EXT_SCL                       (47)                      //GPIO для SCL
#define I2C_EXT_MASTER_TX_BUF_DISABLE     (0)                       //параметры для инициализации интерфейса 
#define I2C_EXT_MASTER_RX_BUF_DISABLE     (0)       
#define I2C_IST8310_FREQ_HZ               (400000)                  //скорости I2С для соответствующих компонентов
#define I2C_FL3195_FREQ_HZ                (400000)
#define I2C_MS5611_FREQ_HZ                (400000)                                                                   
#define I2C_EXT_PORT                      (1)                       //номер используемого порта I2C для "внешней" шины

//GPIO и параметры UART для канала связи с пультом
#define REMOTE_CONTROL_UART                           (2)                     //номер порта
#define RC_UART_BAUD_RATE                             (57600)                 //скорость порта для канала связи с пультом 
#define NUMBER_OF_CONTROL_BYTES_TO_RECEIVE_FROM_RC    (13)                    //длина пакета данных управления, отправляемого пультом на дрон
#define NUMBER_OF_PID_BYTES_TO_RECEIVE_FROM_RC        (8)                     //длина пакета данных с ПИД коэффициентами, отправляемого пультом на дрон
#define NUMBER_OF_BYTES_TO_SEND_TO_RC                 (15)                    //длина пакета данных, отправляемого дроном на пульт    4
#define RC_RX_UART_BUFF_SIZE                          (256)                   //размер буфера (256 это минимальное значение)
#define RC_TX_UART_BUFF_SIZE                          (256)
#define RC_UART_TX_PIN                                (2)                     //GPIO для TX 
#define RC_UART_RX_PIN                                (1)                     //GPIO для RX 
#define RC_UART_RTS_PIN                               (UART_PIN_NO_CHANGE)
#define RC_UART_CTS_PIN                               (UART_PIN_NO_CHANGE)
#define RC_UART_PATTERN_DETECTION_QUEUE_SIZE          (10)                    //длина очереди для алгоритма детектирования пэттерна (можно и меньше, с запасом)
#define RC_CONTROL_MESSAGE_HEADER                     (0x4E)                  //заголовочный байт отправляемых пультом пакетов, содержащих данные для управления, используется для кросс-проверки целостности данных
#define RC_PID_COEFF_MESSAGE_HEADER                   (0x5E)                  //заголовочный байт отправляемых пультом пакетов, содержащих ПИД коэффициенты, используется для кросс-проверки целостности данных
#define RC_NO_COMM_DELAY_MAIN_CYCLES                  (2000)                  //если в полете дроном не будет получен новый пакет от пульта в течение этого кол-ва циклов - считаем что связь с пультом утеряна и переходим в "автономный режим"
#define RC_NO_COMM_THROTTLE_HOVER_VALUE               (9500)                  //значение уровня "газа" в автономном режиме            

//структура для передачи данных от задачи обработки данных пульта в main_flying_cycle.
//данные, полученные от пульта, обрабатываются в соответствующей задаче (RC_read_and_process_data), собираются в эту структуру
// и отправляются через очередь в main_flying_cycle
typedef struct {                                          
    float raw_throttle;
    float received_throttle;                                                  //значение газа
    float received_pitch;                                                     //значение pitch
    float received_roll;                                                      //значение roll
    float received_yaw;                                                       //значение yaw
    uint16_t mode;                                                            //режим (два байта с состояниями тумблеров)
    int8_t trim_pitch;                                                        //значение trim по pitch 
    int8_t trim_roll;                                                         //значение trim по roll
    uint8_t engines_start_flag;                                               //флаг запуска двигателей
    uint8_t lidar_altitude_hold_flag;                                         //флаг удержания высоты по лидару
    uint8_t baro_altitude_hold_flag;                                          //флаг удержания высоты по барометру
    int8_t rssi_level;
  } data_from_rc_to_main_struct;

//структура для передачи ПИД-коэффициентов, полученных от пульта, от задачи обработки данных пульта в main_flying_cycle.
//данные, полученные от пульта, обрабатываются в соответствующей задаче (RC_read_and_process_data), собираются в эту структуру
// и отправляются через очередь в main_flying_cycle
typedef struct {
    uint16_t kp_alt_hold_coeff;
    uint16_t ki_alt_hold_coeff;  
    uint16_t kd_alt_hold_coeff;                                           
  } pid_coeff_data_from_rc_to_main_struct;

//структура для передачи данных от main_flying_cycle в задачу отправки телеметрии на пульт.
//стрктура наполняется данными в main_flying_cycle и отправляется через очередь в задачу отправки телеметрии на пульт (send_data_to_RC)
typedef struct {                                          
    float pitch;                                                              //текущее значение pitch
    float roll;                                                               //текущее значение roll
    float yaw;                                                                //текущее значение yaw
    uint16_t power_voltage_value;                                             //текущее значение напряжения питания
    uint16_t altitude;                                                        //текущее значение высоты
  } data_from_main_to_rc_struct;

#define RC_FILTER_COEFF                         (0.85)                        //коэффициент фильтрации данных с ручки газа remote_control_data.received_throttle = remote_control_data.received_throttle * RC_FILTER_COEFF + rc_throttle_old * (1 - RC_FILTER_COEFF);

//GPIO и параметры UART для работы с GPS
#define GPS_UART                                (0)                           //номер порта
#define GPS_UART_BAUD_RATE                      (38400)                       //скорость порта для работы с GPS
#define NUMBER_OF_BYTES_TO_RECEIVE_FROM_GPS     (140)                         //ожидаемая длина пакета
#define GPS_UART_BUF_SIZE                       (512)                         //размер FIFO буфера под GPS UART
#define GPS_UART_TX_PIN                         (5)                           //GPIO для TX
#define GPS_UART_RX_PIN                         (4)                           //GPIO для RX
#define GPS_UART_RTS_PIN                        (UART_PIN_NO_CHANGE)
#define GPS_UART_CTS_PIN                        (UART_PIN_NO_CHANGE)
#define GPS_UART_PATTERN_DETECTION_QUEUE_SIZE   (10)                          //длина очереди для обнаружения пэттернов
//структура для передачи данных от задачи обработки GPS в main_flying_cycle
//заполняется значениями в задаче обработки данных от GPS (gps_read_and_process_data) и отправляется через очередь в main_flying_cycle
typedef struct data_from_gps_to_main_struct {                             
    uint32_t latitude_d;
    uint32_t longtitude_d;
    uint8_t status;
  } data_from_gps_to_main_struct_t;

#ifdef USING_TFMINIS_I2C 
  #define NUMBER_OF_BYTES_TO_RECEIVE_FROM_LIDAR    (50)  

//структура для передачи данных от задачи обработки данных лидара в main_flying_cycle.
//Заполняется данными в задаче обработке данных лидара (lidar_read_and_process_data) и отправляется через очередь в main_flying_cycle
  typedef struct  {                             
      float altitude;                                                   //высота в см
      float vertical_velocity;
      uint16_t strength;                                                //качество сигнала                                                         
      uint8_t valid;                                                    //данные по высоте корректны или могут быть не корректны
    } data_from_lidar_to_main_struct;
  #define LIDAR_RATE_HZ                              (25)                       //частота поступления данных с лидара
  #define ALT_PID_DIF_FILTER_LENGTH                  (7)
#endif

#ifdef USING_MAVLINK_TELEMETRY
//GPIO и параметры UART для работы с mavlink
  #define MAV_UART                                (1)                         //номер порта
  #define MAV_UART_BAUD_RATE                      (57600)                    //скорость порта
  #define MAV_RX_UART_BUF_SIZE                    (256)                       //размер RX FIFO буфера под Mavlink UART
  #define MAV_TX_UART_BUF_SIZE                    (512)                       //размер TX FIFO буфера под Mavlink UART
  #define MAV_UART_TX_PIN                         (43)                        //GPIO для TX 
  #define MAV_UART_RX_PIN                         (44)                        //GPIO для RX 
  #define MAV_UART_RTS_PIN                        (UART_PIN_NO_CHANGE)
  #define MAV_UART_CTS_PIN                        (UART_PIN_NO_CHANGE)
  #define MAV_UART_PATTERN_DETECTION_QUEUE_SIZE   (10)                        //длина очереди для обнаружения пэттернов

//Переменные, передаваемые в задачу Mavlink
typedef struct {                             
  uint32_t latitude;
  uint32_t longtitude;
  float angles[3]; //pitch, roll,yaw
  uint16_t altitude_cm;
  uint16_t voltage_mv;
  uint16_t current_ca;    //сантиамреры, чтобы влезло в 2 байта
  int8_t rssi_level;
  uint8_t gps_status;
  uint8_t armed_status;                                                  
} mavlink_data_set_t;
#endif

#ifdef USING_PX4FLOW
  #define NUMBER_OF_BYTES_TO_RECEIVE_FROM_PX4   (50)
  typedef struct {                             
    float flow_x_meters;
    float flow_y_meters;
    float flow_vx;
    float flow_vy; 
    float optical_x;
    float optical_y;
    uint8_t quality;                                                       
  } data_from_px4flow_to_main_struct_t;
  #define PX4FLOW_F_PIXELS                                (666.667)
#endif

//GPIO и параметры для модуля ШИМ (LEDC) для управления моторами
#define ENGINE_PWM_DUTY_RESOLUTION              (LEDC_TIMER_14_BIT)             //разрешение таймера для управления ШИМ
#define ENGINE_PWM_MIN_DUTY                     (6553)                          //минимальная длительности ШИМ сигнала 1мс: ((2 ^ 14) - 1) / 2,5мс (400Гц) = 6553,2
#define ENGINE_PWM_FREQUENCY                    (400)                           //частота ШИМ сигнала, Гц
#define ENGINE_PWM_TIMER                        (LEDC_TIMER_0)                  //используемый таймер
#define ENGINE_PWM_MODE                         (LEDC_LOW_SPEED_MODE)           //режим работы ШИМ
#define ENGINE_PWM_OUTPUT_0_PIN                 (9)                             //GPIO для выхода 0
#define ENGINE_PWM_OUTPUT_1_PIN                 (10)                            //GPIO для выхода 1                        
#define ENGINE_PWM_OUTPUT_2_PIN                 (11)                            //GPIO для выхода 2 
#define ENGINE_PWM_OUTPUT_3_PIN                 (12)                            //GPIO для выхода 3
#define TIME_TO_KEEP_RUNNING_AT_CHECK_MS        (10000)                         //длительность вращения двигателей при поочередном тестировании
#define LENGTH_OF_ESC_FILTER                    (6)                             //глубина выходного фильтра (фильтруем данные скользящим средним перед выдачей на моторы)


//другие GPIO 
#define MPU6000_1_INTERRUPT_PIN                 (21)                            //GPIO для сигнала прерывания от IMU1
#define MPU6000_2_INTERRUPT_PIN                 (13)                            //GPIO для сигнала прерывания от IMU2 
#define LED_RED                                 (38)                            //GPIO для красного светодиода
#define LED_BLUE                                (0)                             //GPIO для синего светодиода
#define LED_GREEN                               (3)                             //GPIO для зеленого светодиода   
#define MCP23017_INTERRUPT_PIN                  (37)                            //GPIO для сигнала прерывания от MCP23017 (пока не используется)
#define GREEN_FLIGHT_LIGHTS                     (8)                             //GPIO для зеленых полетных огней (через транзистор)
#define RED_FLIGHT_LIGHTS                       (36)                            //GPIO для зеленых полетных огней (через транзистор)


//команды для MCP23017, которые отправляются из любой задачи в задачу управления MCP23017 через очередь
#define MCP23017_READ_COMMAND                   (0b10000000)                    //считать состояние входов                    
#define MCP23017_SET_OUTPUT_COMMAND             (0b01000000)                    //выставить выход в лог.1, номер выхода указывается в младших 4 битах
#define MCP23017_CLEAR_OUTPUT_COMMAND           (0b00100000)                    //выcтавить выход в лог.0, номер выхода указывается в младших 4 битах

//Параметры IMU и фильтра Маджвика
#define SMPL                                    (0)                             //IMU sample rate = 1000Hz/ 1+SMPL
#define MADGWICK_BETA                           (0.2) //0.999



//Структура для записи логов во внешнюю флэш
//безусловно в состав входит только timestamp
//остальное набирается define'ами выше, чтобы не писать лишнее и не тратить место и время
#ifdef USING_W25N
struct logging_data_set {                             
  uint32_t timestamp;
  #ifdef LOGGING_ACCEL_1 
    int16_t accel_1[3];
  #endif

  #ifdef LOGGING_ACCEL_2 
    int16_t accel_2[3];
  #endif

  #ifdef LOGGING_GYRO_1 
    int16_t gyro_1[3];
  #endif

  #ifdef LOGGING_GYRO_2 
    int16_t gyro_2[3];
  #endif

  #ifdef LOGGING_AVG_ACCEL
    int16_t accel_avg[3];
  #endif

  #ifdef LOGGING_AVG_GYRO
    int16_t gyro_avg[3];
  #endif

  #ifdef LOGGING_QUATERNION                                                          
    float q[4];
  #endif

  #ifdef LOGGING_ANGLES 
    float angles[3]; //pitch, roll,yaw
  #endif

  #ifdef LOGGING_LIDAR_PID
    float error;
    float P_component;
    float I_component;
    float D_component;
  #endif

  #ifdef LOGGING_LIDAR_ALTITUDE_CM
    uint16_t lidar_altitude_cm;
  #endif
  
  #ifdef LOGGING_BARO_ALTITUDE_CM
    uint16_t baro_altitude_cm;
  #endif

  #ifdef LOGGING_KALMAN_ALTITUDE_CM
    int16_t kalman_altitude_cm;
  #endif

  #ifdef LOGGING_KALMAN_VELOCITY
    int16_t kalman_velocity_cm;
  #endif

  #ifdef LOGGING_ALTITUDE_SETPOINT_CM
    uint16_t altitude_setpoint_cm;
  #endif

  #ifdef LOGGING_VOLTAGE_mV
    uint16_t voltage_mv;
  #endif

  #ifdef LOGGING_CURRENT_cA
    uint16_t current_ca;    //сантиамперы, чтобы влезло в 2 байта
  #endif

  #ifdef LOGGING_RC_COMMANDS
    float throttle_command;
    float pitch_command;
    float roll_command;
    float yaw_command;
    uint16_t mode_command;
  #endif

  #ifdef LOGGING_ENGINES
    uint32_t engines[4];
  #endif

  #ifdef LOGGING_ENGINES_FILTERED
    uint32_t engines_filtered[4];
  #endif

  #ifdef LOGGING_FLAGS
    uint16_t flags;
  #endif

  #ifdef LOGGING_RSSI_LEVEL
  int8_t rssi_level;
  #endif                                                  
};
#endif


//ниже общие параметры

//таймер, который сбрасывается при выполнении основного полетного цикла. 
//если цикл виснет - через это время сработает прерывание и аварийно выключит моторы
#define SUSPENSION_TIMER_DELAY_SEC              (1)
//если в течение столько мс не приходит прерывание от IMU считаем его зависшим                             
#define IMU_SUSPENSION_TIMER_DELAY_MS           (3)                             
#define NUMBER_OF_IMU_CALIBRATION_COUNTS        (8000)                          //кол-во усредняемых при калибровке сэмплов  
#define PID_LOOPS_RATIO                         (5)                             //соотношение между внутренним (угловая скорость) и внешним (угол) циклом PID
#define NUMBER_OF_MAG_INPUTS                    (500)                           //кол-во векторов для расчета калибровки магнетометра по методу magnetto 
#define NUMBER_OF_ACC_INPUTS                    (100)                            //кол-во векторов для расчета калибровки акселерометра по методу magnetto 

//GPIO
#define A0                                            (4)
#define A1                                            (5)
#define A2                                            (6)                       //кнопка аварийной остановки
#define A3                                            (7)                       //светодиод кнопки авариной остановки 

//объем стэков для задач
#define MCP23017_MONITORING_AND_CONTROL_STACK_SIZE    (4096)
#define PCA9685_CONTROL_STACK_SIZE                    (4096)
#define MAG_READ_DATA_AND_SEND_TO_MAIN_STACK_SIZE     (4096)
#define MAIN_FLYING_CYCLE_STACK_SIZE                  (8192)
#define GPS_READ_AND_PROCESS_DATA_STACK_SIZE          (4096)
#define RC_READ_AND_PROCESS_DATA_STACK_SIZE           (4096)
#define SEND_DATA_TO_RC_STACK_SIZE                    (4096)
#define BLINKING_FLIGHT_LIGHTS_STACK_SIZE             (2048)
#define LIDAR_READ_AND_PROCESS_DATA_STACK_SIZE        (4096)
#define INA219_READ_AND_PROCESS_DATA_STACK_SIZE       (4096) 
#define MAG_READ_AND_PROCESS_DATA_STACK_SIZE          (4096)
#define WRITING_LOGS_TO_FLASH_STACK_SIZE              (4096)
#define PERFORMANCE_MEASUREMENT_STACK_SIZE            (8192)
#define MAVLINK_TELEMETRY_STACK_SIZE                  (4096)
#define MS5611_READ_AND_PROCESS_DATA_STACK_SIZE       (4096)
#define PX4FLOW_READ_AND_PROCESS_DATA_STACK_SIZE      (4096)
#define EMERGENCY_MODE_STACK_SIZE                     (4096)
#define SENDING_SOMETHING_OVER_WIFI_STACK_SIZE        (4096)

//приоритеты задач
#define MCP23017_MONITORING_AND_CONTROL_PRIORITY      (1)
#define PCA9685_CONTROL_PRIORITY                      (1)
#define MAG_READ_DATA_AND_SEND_TO_MAIN_PRIORITY       (6)
#define MAIN_FLYING_CYCLE_PRIORITY                    (24)
#define GPS_READ_AND_PROCESS_DATA_PRIORITY            (3)
#define RC_READ_AND_PROCESS_DATA_PRIORITY             (5)
#define SEND_DATA_TO_RC_PRIORITY                      (4)
#define BLINKING_FLIGHT_LIGHTS_PRIORITY               (0)
#define LIDAR_READ_AND_PROCESS_DATA_PRIORITY          (3)
#define INA219_READ_AND_PROCESS_DATA_PRIORITY         (2) 
#define MAG_READ_AND_PROCESS_DATA_PRIORITY            (6)
#define WRITING_LOGS_TO_FLASH_PRIORITY                (0)
#define PERFORMANCE_MEASUREMENT_PRIORITY              (2)
#define MAVLINK_TELEMETRY_PRIORITY                    (0)
#define MS5611_READ_AND_PROCESS_DATA_PRIORITY         (2)
#define PX4FLOW_READ_AND_PROCESS_DATA_PRIORITY        (4)
#define EMERGENCY_MODE_PRIORITY                       (1)
#define SENDING_SOMETHING_OVER_WIFI_PRIORITY          (2)

#define BATTERY_CAPACITY                              (2200) //мА*ч
#define VERT_ACC_FILTER_F_CUT                         (8.0)  

//параметры для фильтра Калмана по высоте
#define ACCEL_SIGMA2                                  (100)//10
#define BARO_SIGMA2                                   (20)
#define VERT_ALT_FILTER_F_CUT                         (4.0)

//структура, хранящее текущее состояние системы
typedef struct system_state {                             
  float acc_local[3];
  float gyro_local[3];
  float velocity_local[3];
  float q[4];
  float acc_ned[3];
  float velocity_ned[3];
  float position_ned[3];
  float pitch;
  float roll;
  float yaw;
  float lidar_h_cm;
  float baro_h_cm;
  float altitude;
} system_state_t;

//структура, которую собирает задача emergency_mode, чтобы отправить по радио на низкой скорости
typedef struct  {                             
      uint32_t latitude;
      uint32_t longtitude;
      uint8_t voltage_dv;       //вольты * 10                                                                                                                                         //данные по высоте корректны или могут быть не корректны
    } __attribute__((packed)) data_from_emergency_beacon_to_radio_t;


#endif