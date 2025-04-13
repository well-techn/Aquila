//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <math.h>
#include <sys/time.h>
#include <rom/ets_sys.h>
#include "soc/gpio_reg.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "wt_spi.h"
#include "PCA9685.h"
#include "MCP23017.h"
#include "winbondW25N.h"
#include "madgwick.h"
#include "PMW3901.h"
#include "MPU6000.h"
#include "TfminiS.h"
#include "INA219.h"
#include "IST8310.h"
#include "FL3195.h"
#include "MS5611.h"
#include "filters.h"
#include "error_code_LED_blinking.h"
#include "timing.h"
#include "pid.h"

//timers handles
extern gptimer_handle_t GP_timer;
extern gptimer_handle_t general_suspension_timer;
extern gptimer_handle_t IMU_1_suspension_timer;
extern gptimer_handle_t IMU_2_suspension_timer;

//spi handles
extern spi_device_handle_t W25N01;
extern spi_device_handle_t MPU6000_1;
extern spi_device_handle_t MPU6000_2;

//tasks handlers
extern TaskHandle_t task_handle_blinking_flight_lights;
extern TaskHandle_t task_handle_main_flying_cycle;
extern TaskHandle_t task_handle_send_data_to_RC;
extern TaskHandle_t task_handle_performace_measurement;
extern TaskHandle_t task_handle_INA219_read_and_process_data;
extern TaskHandle_t task_handle_mag_read_and_process_data;
extern TaskHandle_t task_handle_PCA9685_control;
extern TaskHandle_t task_handle_MCP23017_monitoring_and_control;
extern TaskHandle_t task_handle_writing_logs_to_flash;
extern TaskHandle_t task_handle_mavlink_telemetry;
extern TaskHandle_t task_handle_lidar_read_and_process_data;
extern TaskHandle_t task_handle_MS5611_read_and_process_data;

extern char *TAG_INIT;
extern const char *TAG_FLY;
extern const char *TAG_NVS;

extern QueueHandle_t magnetometer_queue;
extern QueueHandle_t remote_control_to_main_queue;
extern QueueHandle_t lidar_to_main_queue;
extern QueueHandle_t INA219_to_main_queue;
extern QueueHandle_t main_to_rc_queue;
extern QueueHandle_t W25N01_queue;
extern QueueHandle_t main_to_mavlink_queue;
extern QueueHandle_t gps_to_main_queue;
extern QueueHandle_t MS5611_to_main_queue;

/********************************************************************    ПРЕРЫВАНИЯ    ***********************************************************************************************/

//прерывания от GPIO - IMU, аварийная остановка
static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
  uint32_t gpio_intr_status_1 = 0;
  uint32_t gpio_intr_status_2 = 0;

  uint8_t imu_1_interrupt_flag = 0;
  uint8_t imu_2_interrupt_flag = 0;

  uint64_t IMU_1_timer_value = 0;
  uint64_t IMU_2_timer_value = 0;

  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;

  gpio_intr_status_1 = READ_PERI_REG(GPIO_STATUS_REG);     // Чтение регистров статуса прерывания для GPIO0-31
  SET_PERI_REG_MASK(GPIO_STATUS_W1TC_REG , gpio_intr_status_1);      // Очистка регистров прерывания для GPIO0-31 (очистка флагов, запись 1 очищает флаг)
  gpio_intr_status_2 = READ_PERI_REG(GPIO_STATUS1_REG);     // Чтение регистров статуса прерывания для GPIO32-39
  SET_PERI_REG_MASK(GPIO_STATUS1_W1TC_REG , gpio_intr_status_2);      // Очистка регистров прерывания для GPIO32-39 (очистка флагов, запись 1 очищает флаг)
  
  //gpio_intr_status = gpio_intr_status_1 | gpio_intr_status_2;

  if (gpio_intr_status_1 & (1ULL << A2))   //сигнал от кнопки аварийной остановки на Holybro M9N, выключаем двигатели
  {
    ledc_timer_pause(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
    gpio_set_level(ENGINE_PWM_OUTPUT_0_PIN ,0);
    gpio_set_level(ENGINE_PWM_OUTPUT_1_PIN ,0);
    gpio_set_level(ENGINE_PWM_OUTPUT_2_PIN ,0);
    gpio_set_level(ENGINE_PWM_OUTPUT_3_PIN ,0);
    gpio_set_level(A3, 0);                        //включить светодиод на кнопке аварийной остановки
  }  
  
  if (gpio_intr_status_1 & (1ULL << MPU6000_1_INTERRUPT_PIN))   //сигнал от IMU1
  {
    imu_1_interrupt_flag = 1; 
    gptimer_get_raw_count(IMU_1_suspension_timer,&IMU_1_timer_value);
    gptimer_set_raw_count(IMU_1_suspension_timer, 0);
    gptimer_get_raw_count(IMU_2_suspension_timer,&IMU_2_timer_value);
  } 
  
  if (gpio_intr_status_1 & (1ULL << MPU6000_2_INTERRUPT_PIN))   //сигнал от IMU1 
  {
    imu_2_interrupt_flag = 1;
    gptimer_get_raw_count(IMU_1_suspension_timer,&IMU_1_timer_value);
    gptimer_get_raw_count(IMU_2_suspension_timer,&IMU_2_timer_value); 
    gptimer_set_raw_count(IMU_2_suspension_timer, 0);
  } 

  if (imu_1_interrupt_flag && (IMU_1_timer_value < IMU_SUSPENSION_TIMER_DELAY_MS * 1000) && (IMU_2_timer_value < IMU_SUSPENSION_TIMER_DELAY_MS * 1000)) //все в порядке, оба сигнала от IMU пришли вовремя 
  {
    imu_1_interrupt_flag = 0;
    imu_2_interrupt_flag = 0;
    xTaskGenericNotifyFromISR(task_handle_main_flying_cycle, 0, 14, eSetValueWithOverwrite, NULL,  &xHigherPriorityTaskWoken);    //14 - код что все ок
  }

 else if (IMU_2_timer_value > IMU_SUSPENSION_TIMER_DELAY_MS * 1000)  //IMU2 не выдал сигнал
  {
    imu_1_interrupt_flag = 0;
    imu_2_interrupt_flag = 0;
    xTaskGenericNotifyFromISR(task_handle_main_flying_cycle, 0, 15, eSetValueWithOverwrite, NULL, &xHigherPriorityTaskWoken);     //15 - код что 2ой завис
  }

  else if ( IMU_1_timer_value > IMU_SUSPENSION_TIMER_DELAY_MS * 1000)  //IMU1 не выдал сигнал
  {
    imu_1_interrupt_flag = 0;
    imu_2_interrupt_flag = 0;
    xTaskGenericNotifyFromISR(task_handle_main_flying_cycle, 0, 16, eSetValueWithOverwrite, NULL, &xHigherPriorityTaskWoken);     //16 - код что 1ый завис
  }

  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//прерывание от таймера зависания основного полетного цикла
//этот таймер сбрасывается в основном цикле полета. Если он переполнился - значит основной цикл завис, аварийно останавливаем двигетели
static void IRAM_ATTR general_suspension_timer_interrupt_handler(void *args)    
{ 
  ledc_timer_pause(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
  gpio_set_level(ENGINE_PWM_OUTPUT_0_PIN ,0);
  gpio_set_level(ENGINE_PWM_OUTPUT_1_PIN ,0);
  gpio_set_level(ENGINE_PWM_OUTPUT_2_PIN ,0);
  gpio_set_level(ENGINE_PWM_OUTPUT_3_PIN ,0);
  gpio_set_level(A3, 0);                        //включить светодиод на кнопке аварийной остановки
}


/********************************************************************   ОБЩИЕ ФУНКЦИИ  ***********************************************************************************************/

//настройка пинов, которые используются для прерываний
static void configure_pins_for_interrupt()
{
  ESP_ERROR_CHECK(gpio_reset_pin(MPU6000_1_INTERRUPT_PIN));
  gpio_config_t INT_1 = {
    .pin_bit_mask = 1ULL << MPU6000_1_INTERRUPT_PIN,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ENABLE,
    .intr_type = GPIO_INTR_POSEDGE
  }; 
  
  ESP_ERROR_CHECK(gpio_reset_pin(MPU6000_2_INTERRUPT_PIN));
  gpio_config_t INT_2 = {
    .pin_bit_mask = 1ULL << MPU6000_2_INTERRUPT_PIN,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ENABLE,
    .intr_type = GPIO_INTR_POSEDGE
  }; 

  ESP_ERROR_CHECK(gpio_reset_pin(A2));              //на A2 подключен кнопка аварийной остановки на Holybro M9N 
  gpio_config_t INT_3 = {
    .pin_bit_mask = 1ULL << A2,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ENABLE,
    .intr_type = GPIO_INTR_POSEDGE
  };

  ESP_ERROR_CHECK(gpio_config(&INT_1));
  ESP_ERROR_CHECK(gpio_config(&INT_2));
  ESP_ERROR_CHECK(gpio_config(&INT_3));

  ESP_ERROR_CHECK(gpio_isr_register(gpio_interrupt_handler, 0, 0, NULL)); 
}

//создание таймера, контролирующего зависание IMU1
static void Create_and_start_IMU_1_suspension_Timer()           
{
  gptimer_config_t IMU_1_timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&IMU_1_timer_config, &IMU_1_suspension_timer));
  ESP_ERROR_CHECK(gptimer_enable(IMU_1_suspension_timer));
  ESP_ERROR_CHECK(gptimer_start(IMU_1_suspension_timer));
}

//создание таймера, контролирующего зависание IMU2
static void Create_and_start_IMU_2_suspension_Timer()                    
{
  gptimer_config_t IMU_2_timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&IMU_2_timer_config, &IMU_2_suspension_timer));
  ESP_ERROR_CHECK(gptimer_enable(IMU_2_suspension_timer));
  ESP_ERROR_CHECK(gptimer_start(IMU_2_suspension_timer));
}

//создание таймера контроля зависания основного полетного цикла
static void create_and_start_general_suspension_timer()                    
{
  gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &general_suspension_timer));

  gptimer_alarm_config_t alarm_config = {                 //setting alarm threshold
      .alarm_count = SUSPENSION_TIMER_DELAY_SEC * 1000 * 1000,   //1 second
      //.reload_count = NULL,
  };
  ESP_ERROR_CHECK(gptimer_set_alarm_action(general_suspension_timer, &alarm_config));

  gptimer_event_callbacks_t  Suspension_timer_interrupt = {       //this function will be launched when timer alarm occures
      .on_alarm = (gptimer_alarm_cb_t)general_suspension_timer_interrupt_handler,     // register user callback
  };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(general_suspension_timer, &Suspension_timer_interrupt, NULL));

  ESP_ERROR_CHECK(gptimer_enable(general_suspension_timer));
  ESP_ERROR_CHECK(gptimer_start(general_suspension_timer));
}




/********************************************************************      ЗАДАЧА      ***********************************************************************************************/

/*Задача основного полетного цикла. Единственная задача на ядре 1. 
  - Активируется по прерыванию о готовности данных от IMU.
  - при необходимости калибрует и записывает в NVS калибровочные коэффициенты 
  - Считывает данные от IMU, обрабатывает, вычисляет Маджвиком (с учетом магнетометра или без) углы наклона
  - получает из очереди команды от пульта
  - на основе данных по текущим углам и команд от пульта вычисляет необходимые воздействия на двигатели посредством двухконтурного PID
  - выдает эти воздействия на моторы
  - подготавливает данные для записи логов и выдает их в соответствующую очередь 

Сначала идут переменные, потом вспомогательные функции, потом само тело задачи.
*/
void main_flying_cycle(void * pvParameters)
{
  extern float q0;
  extern float q1;
  extern float q2;
  extern float q3;
  
  uint32_t IMU_interrupt_status = 0;
  uint8_t test_for_all_0 = 0;
  uint8_t test_for_all_1 = 0xFF;
  
  uint8_t sensor_data_1[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
  uint8_t sensor_data_2[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};

  int16_t accel_raw_1[3] = {0,0,0}; 
  int16_t gyro_raw_1[3] = {0,0,0}; 
  int16_t accel_raw_2[3] = {0,0,0}; 
  int16_t gyro_raw_2[3] = {0,0,0};

  uint64_t large_counter = 0;
  uint64_t timer_value = 0;
  uint16_t i = 0;
 
  int16_t accel_1_offset[3] = {0,0,0};
  int16_t accel_2_offset[3] = {0,0,0};
  int16_t gyro_1_offset[3] = {0,0,0};
  int16_t gyro_2_offset[3] = {0,0,0};
   
  float gyro_converted_1[3] = {0.0,0.0,0.0};
  float accel_converted_1[3] = {0.0,0.0,0.0};
  float gyro_converted_2[3] = {0.0,0.0,0.0};
  float accel_converted_2[3] = {0.0,0.0,0.0};

  float accel_converted_accumulated_1[3] = {0.0,0.0,0.0};
  float gyro_converted_accumulated_1[3] = {0.0,0.0,0.0};

  float accel_converted_accumulated_2[3] = {0.0,0.0,0.0};
  float gyro_converted_accumulated_2[3] = {0.0,0.0,0.0};

#ifdef USING_MAGNETOMETER
  const uint8_t madgwick_cycles = 1;
#else 
  const uint8_t  madgwick_cycles = 5;
#endif

  static float pitch = 0; 
  static float roll = 0; 
  static float yaw = 0; 

  static float throttle;

// for cascaded PID

  float pid_pitch_angle = 0;
  float pid_pitch_rate = 0;
  float gyro_pitch = 0;
  float gyro_pitch_old = 0;

  PIDController_t pitch_pid_angle;
  pitch_pid_angle.kp = 4.4;
  pitch_pid_angle.kd = 10;
  pitch_pid_angle.ki = 0.0004;
  pitch_pid_angle.prev_error = 0;
  pitch_pid_angle.integral_error = 0;
  pitch_pid_angle.integral_limit = 1000;
  pitch_pid_angle.pid_limit = 2000;

  PIDController_t pitch_pid_rate;
  pitch_pid_rate.kp = 8.8;
  pitch_pid_rate.kd = 860;
  pitch_pid_rate.ki = 0.03;
  pitch_pid_rate.prev_error = 0;
  pitch_pid_rate.integral_error = 0;
  pitch_pid_rate.integral_limit = 1000;
  pitch_pid_rate.pid_limit = 3000;

  float pid_roll_angle = 0;
  float pid_roll_rate = 0;
  float gyro_roll = 0;
  float gyro_roll_old = 0;

  PIDController_t roll_pid_angle;
    roll_pid_angle.kp = 4.2;
    roll_pid_angle.kd = 8;
    roll_pid_angle.ki = 0.0004;
    roll_pid_angle.prev_error = 0;
    roll_pid_angle.integral_error = 0;
    roll_pid_angle.integral_limit = 1000;
    roll_pid_angle.pid_limit = 2000;

  PIDController_t roll_pid_rate;
    roll_pid_rate.kp = 8.8;
    roll_pid_rate.kd = 860;
    roll_pid_rate.ki = 0.03;
    roll_pid_rate.prev_error = 0;
    roll_pid_rate.integral_error = 0;
    roll_pid_rate.integral_limit = 1000;
    roll_pid_rate.pid_limit = 3000;

  float yaw_setpoint = 0.0;
  float Kp_yaw_angle = 3;
  float Kd_yaw_angle = 10;
  float Ki_yaw_angle = 0.001;
  float integral_yaw_error_angle = 0;

  float error_yaw_angle = 0;
  float error_yaw_angle_old = 0;
  float pid_yaw_angle = 0;
  float pid_yaw_rate = 0;
  float gyro_yaw = 0;
/*
  PIDController_t yaw_pid_angle_1;
  yaw_pid_angle_1.kp = 3.0;
  yaw_pid_angle_1.kd = 10;
  yaw_pid_angle_1.ki = 0.001;
  yaw_pid_angle_1.prev_error = 0;
  yaw_pid_angle_1.integral_error = 0;
  yaw_pid_angle_1.integral_limit = 1000;
  yaw_pid_angle_1.pid_limit = 3000;
*/
  PIDController_t yaw_pid_rate;
    yaw_pid_rate.kp = 100.0;
    yaw_pid_rate.kd = 0;
    yaw_pid_rate.ki = 0.01;
    yaw_pid_rate.prev_error = 0;
    yaw_pid_rate.integral_error = 0;
    yaw_pid_rate.integral_limit = 1000;
    yaw_pid_rate.pid_limit = 3000;

  float engine[4] = {ENGINE_PWM_MIN_DUTY,ENGINE_PWM_MIN_DUTY,ENGINE_PWM_MIN_DUTY,ENGINE_PWM_MIN_DUTY}; 
  float engine_filtered[4] = {ENGINE_PWM_MIN_DUTY,ENGINE_PWM_MIN_DUTY,ENGINE_PWM_MIN_DUTY,ENGINE_PWM_MIN_DUTY};

  float engine_0_filter_pool[LENGTH_OF_ESC_FILTER] = {0};
  float engine_1_filter_pool[LENGTH_OF_ESC_FILTER] = {0}; 
  float engine_2_filter_pool[LENGTH_OF_ESC_FILTER] = {0}; 
  float engine_3_filter_pool[LENGTH_OF_ESC_FILTER] = {0}; 

  struct data_from_rc_to_main_struct rc_fresh_data;
    rc_fresh_data.mode = 0;
    rc_fresh_data.engines_start_flag = 0;
    rc_fresh_data.altitude_hold_flag = 0;
  struct data_from_main_to_rc_struct data_to_send_to_rc;
  uint32_t remote_control_lost_comm_counter = 0;

#ifdef USING_GPS
  struct data_from_gps_to_main_struct gps_fresh_data;
#endif  

#ifdef USING_MAGNETOMETER
  float mag_fresh_data[3];
#endif 

//переменные по режиму удержания высоты
  bool altitude_hold_mode_enabled = 0;
  float altitude_setpoint = 0;
  float alt_hold_initial_throttle = 0;
  float alt_error_change_filter_pool[7] = {0};//ALT_PID_DIF_FILTER_LENGTH
  uint8_t new_lidar_data_arrived_flag = 0;
  float pid_altitude = 0;

  PIDController_t alt_hold_pid;
    alt_hold_pid.kp = 10.0;  //10         //30 (на БП)
    alt_hold_pid.kd = 290.0;//290      //400 (на БП)
    alt_hold_pid.ki = 0.4;//0.3       //0.05 (на БП)   x40
    alt_hold_pid.prev_error = 0;
    alt_hold_pid.integral_error = 0;
    alt_hold_pid.integral_limit = 250;
    alt_hold_pid.pid_limit = 500;


#ifdef USING_TFMINIS_I2C 
    struct data_from_lidar_to_main_struct lidar_fresh_data;
      lidar_fresh_data.altitude = 0;
      lidar_fresh_data.strength = 0;
      lidar_fresh_data.valid = 0;
    float lidar_altitude_corrected = 0;

#endif

  float INA219_fresh_data[4];
  float voltage_correction_coeff = 1.0;
  
#ifdef USING_W25N 
  //переменные для логирования
  uint64_t start_time = 0;
  uint8_t flags_byte = 0;
  struct logging_data_set set_to_log;
  struct logging_data_set* p_to_log_structure = &set_to_log;
#endif


#ifdef USING_MAVLINK_TELEMETRY
    mavlink_data_set_t main_to_mavlink_data;
    main_to_mavlink_data.latitude = 321857245;     //стартовые значения, чтобы не было рандомных цифр при отсутствии GPS
    main_to_mavlink_data.longtitude = 441857249;
    mavlink_data_set_t* p_to_mavlink_data = &main_to_mavlink_data;
#endif

#ifdef USING_MS5611
    float baro_altitude = 0;
#endif

/*
  uint8_t long_cycle_flag = 0;
  float q_from_RC[4] = {1,0,0,0};
  float q_current[4] = {0,0,0,0};
  float q_difference[4] = {0,0,0,0};
  float q_angle = 0;
  float q_axis[4];
*/
  void Convert_Q_to_degrees(void) {
    
    float a12,a22,a31,a32,a33 = 0;

    a12 =   2.0f * (q1 * q2 + q0 * q3);
    a22 =   q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    a31 =   2.0f * (q0 * q1 + q2 * q3);
    a32 =   2.0f * (q1 * q3 - q0 * q2);
    a33 =   q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    
    pitch = -asin(a32) * 57.29577951;
    
    roll = atan2(a31, a33) * 57.29577951;
    if (roll > 90) roll -= 360.0;
    
    yaw = -atan2(a12, a22) * 57.29577951;
    yaw -= 9.4; // компенсация наклонения при использовании магнетометра. Если магнетометр не используем - без разницы, вреда не наносит

  }

  void calculate_pids_2(void) {

//наружний pitch по углу   
    pid_pitch_angle = PID_Compute(&pitch_pid_angle,rc_fresh_data.received_pitch, pitch, throttle, 9000);
//внутренний pitch по угловой скорости
    gyro_pitch = 0.1 * (((gyro_converted_1[0] + gyro_converted_2[1]) / 2.0) * (180.0 / (float)PI)) + 0.9 * gyro_pitch_old;
    gyro_pitch_old = gyro_pitch;
    //pid_pitch_rate = PID_Compute(&pitch_pid_rate,rc_fresh_data.received_pitch, gyro_pitch, throttle, 9000);
    pid_pitch_rate = PID_Compute(&pitch_pid_rate,pid_pitch_angle, gyro_pitch, throttle, 9000);

//наружний roll по углу   
    pid_roll_angle = PID_Compute(&roll_pid_angle,rc_fresh_data.received_roll, roll, throttle, 9000);
//внутренний roll по угловой скорости
    gyro_roll = 0.1 * (((gyro_converted_1[1] - gyro_converted_2[0]) / 2.0 ) * (180.0 / (float)PI)) + 0.9 * gyro_roll_old;
    gyro_roll_old = gyro_roll;
    //pid_pitch_rate = PID_Compute(&roll_pid_rate,rc_fresh_data.received_roll, gyro_roll, throttle, 9000);  
    pid_roll_rate = PID_Compute(&roll_pid_rate,pid_roll_angle, gyro_roll, throttle, 9000);
    
//выставляем yaw_setpoint интегрируя данные от пульта
    yaw_setpoint = yaw_setpoint + 0.0016 * rc_fresh_data.received_yaw;  //чем больше этот коэффициент тем выше скорость изменения yaw
    if (yaw_setpoint >= 360) yaw_setpoint = yaw_setpoint - 360.0;
    if (yaw_setpoint < 0) yaw_setpoint = yaw_setpoint + 360.0;

//outer (angle) yaw cycle
    if (throttle < 9000.0) yaw_setpoint = yaw;  //чтобы ошибка не накапливалась пока стоит на земле
    error_yaw_angle = -yaw_setpoint + yaw;
    if (error_yaw_angle <= -180.0) error_yaw_angle = error_yaw_angle + 360.0; 
    if (error_yaw_angle >= 180.0)  error_yaw_angle = error_yaw_angle - 360.0;

    integral_yaw_error_angle = integral_yaw_error_angle + Ki_yaw_angle * error_yaw_angle;
    if (integral_yaw_error_angle > 500.0) integral_yaw_error_angle = 500.0;
    if (integral_yaw_error_angle < -500.0) integral_yaw_error_angle =-500.0;

    pid_yaw_angle = Kp_yaw_angle * error_yaw_angle + integral_yaw_error_angle + Kd_yaw_angle * (error_yaw_angle - error_yaw_angle_old);
    if (pid_yaw_angle > 1000.0)  pid_yaw_angle = 1000.0;
    if (pid_yaw_angle < -1000.0) pid_yaw_angle = -1000.0;
    error_yaw_angle_old = error_yaw_angle;
   
//внутренний yaw по угловой скорости
    gyro_yaw = ((((gyro_converted_1[2] * (-1.0)) - gyro_converted_2[2]) / 2.0) * (180.0 / (float)PI));  
    //pid_yaw_rate = PID_Compute(&yaw_pid_rate, rc_fresh_data.received_yaw, gyro_yaw, throttle, 9000);
    pid_yaw_rate = PID_Compute(&yaw_pid_rate, pid_yaw_angle, gyro_yaw, throttle, 9000);


    engine[0] = (throttle + pid_pitch_rate + pid_roll_rate - pid_yaw_rate);
    engine[1] = (throttle + pid_pitch_rate - pid_roll_rate + pid_yaw_rate);
    engine[2] = (throttle - pid_pitch_rate - pid_roll_rate - pid_yaw_rate);
    engine[3] = (throttle - pid_pitch_rate + pid_roll_rate + pid_yaw_rate);

#ifdef BATTERY_COMPENSATION
    voltage_correction_coeff = 0.9 * voltage_correction_coeff + 0.1 * (11.1 / INA219_fresh_data[0]);           //11.1 - номинальное напряжение 3S АКБ, 12,6 максимальное, 9,9 минимальное
        
    if (voltage_correction_coeff < (11.1/12.6)) voltage_correction_coeff = 11.1/12.6;
    if (voltage_correction_coeff > (11.1/8.8))  voltage_correction_coeff = 11.1/8.8;

    for (i=0;i<4;i++) engine[i] *= sqrt(voltage_correction_coeff);      //так как тяга пропорциональна квадрату оборотов (по идее)
#endif

    for (i=0;i<4;i++) { if (engine[i] > 13106.0) engine[i] = 13106.0;}                                        //верхний предел
    for (i=0;i<4;i++) { if (engine[i] < ENGINE_PWM_MIN_DUTY + 150) engine[i] = ENGINE_PWM_MIN_DUTY + 150;}    //чтобы моторы не останавливались
  }
  
  void ESC_input_data_filter(void) {                                                                                                                    
    
    engine_filtered[0] = avg_filter_1d(engine_0_filter_pool, engine[0], LENGTH_OF_ESC_FILTER);
    engine_filtered[1] = avg_filter_1d(engine_1_filter_pool, engine[1], LENGTH_OF_ESC_FILTER);
    engine_filtered[2] = avg_filter_1d(engine_2_filter_pool, engine[2], LENGTH_OF_ESC_FILTER);
    engine_filtered[3] = avg_filter_1d(engine_3_filter_pool, engine[3], LENGTH_OF_ESC_FILTER);
  }

  void update_engines(void) {
    ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 0, (uint32_t)engine_filtered[0]));
    ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 0));

    ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 1, (uint32_t)engine_filtered[1]));
    ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 1));

    ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 2, (uint32_t)engine_filtered[2]));
    ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 2));

    ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 3, (uint32_t)engine_filtered[3]));
    ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 3));
  }

//функция подготовка логов для записи во флеш память
#ifdef USING_W25N 
void prepare_logs(void) {
  set_to_log.timestamp = get_time() - start_time;
  for (i=0;i<4;i++) set_to_log.accel[i] = accel_raw_1[i];
  for (i=0;i<4;i++) set_to_log.gyro[i] = gyro_raw_1[i];                                                        
  set_to_log.q[0] = q0; 
  set_to_log.q[1] = q1;
  set_to_log.q[2] = q2;
  set_to_log.q[3] = q3;  
  set_to_log.angles[0] = pitch;
  set_to_log.angles[1] = roll; 
  set_to_log.angles[2] = yaw;
  set_to_log.lidar_altitude_cm = (uint16_t)lidar_altitude_corrected;
  set_to_log.lidar_strength = lidar_fresh_data.strength;
#ifdef USING_MS5611
  set_to_log.baro_altitude_m = (uint16_t)baro_altitude;
#endif
  set_to_log.voltage_mv = (uint16_t)(INA219_fresh_data[0] * 1000);
  set_to_log.current_ca = (uint16_t)(INA219_fresh_data[1] * 100);
  set_to_log.throttle_command = throttle;                         //Значение газа либо от пульта либо по altitude_hold
  set_to_log.pitch_command = rc_fresh_data.received_pitch;
  set_to_log.roll_command = rc_fresh_data.received_roll;
  set_to_log.yaw_command = rc_fresh_data.received_yaw;
  set_to_log.mode_command = rc_fresh_data.mode;
  for (i=0;i<4;i++) set_to_log.engines[i] = engine[i];
  
  if (rc_fresh_data.engines_start_flag) flags_byte |= 0b00000001;                                 //bit0 - engine start flag
   else flags_byte &= 0b11111110;
  if (altitude_hold_mode_enabled) flags_byte |= 0b00000010;                                 //bit1 - altitude hold flag
  else flags_byte &= 0b11111101;
  if (remote_control_lost_comm_counter == RC_NO_COMM_DELAY_MAIN_CYCLES) flags_byte |= 0b10000010; //bit8 - comm lost flag
   else flags_byte &= 0b01111111;
  set_to_log.flags = flags_byte;
  set_to_log.rssi_level = rc_fresh_data.rssi_level;
}
#endif

//функция подготовки структуры с данными телеметрии для отправки в мавлинк
#ifdef USING_MAVLINK_TELEMETRY 
void prepare_mavlink_data(void) {
#ifdef USING_GPS  
  main_to_mavlink_data.latitude = gps_fresh_data.latitude_d;
  main_to_mavlink_data.longtitude = gps_fresh_data.longtitude_d;
  main_to_mavlink_data.gps_status = gps_fresh_data.status;
#endif
  main_to_mavlink_data.angles[0] = -pitch;    //для корректного отображения на камере нужны знаки -
  main_to_mavlink_data.angles[1] = -roll; 
  main_to_mavlink_data.angles[2] = -yaw;
  main_to_mavlink_data.voltage_mv = (uint16_t)(INA219_fresh_data[0] * 1000);
  main_to_mavlink_data.current_ca = (uint16_t)(INA219_fresh_data[1] * 100);
  main_to_mavlink_data.altitude_cm = (uint16_t)lidar_altitude_corrected;
  main_to_mavlink_data.rssi_level = rc_fresh_data.rssi_level;
  main_to_mavlink_data.armed_status = rc_fresh_data.engines_start_flag;
}
#endif

void NVS_reading_calibration_values(void)
{
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      // NVS partition was truncated and needs to be erased
      // Retry nvs_flash_init
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
  }
  ESP_ERROR_CHECK( err );

  ESP_LOGI(TAG_NVS,"Открываем NVS... ");
  nvs_handle_t NVS_handle;
  err = nvs_open("storage", NVS_READWRITE, &NVS_handle);
  if (err != ESP_OK) {
      ESP_LOGE(TAG_NVS,"Ошибка (%s) открытия NVS!\n", esp_err_to_name(err));
  } else {
        ESP_LOGI(TAG_NVS,"NVS открыт");

  // Read
  ESP_LOGI(TAG_NVS,"Считываем данные калибровки IMU из NVS ... ");
  
  err = nvs_get_i16(NVS_handle, "accel_1_off_0", &accel_1_offset[0]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGD(TAG_NVS,"accel_1_offset[0] = %d", accel_1_offset[0]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"accel_1_offset[0] не определен!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "accel_1_off_1", &accel_1_offset[1]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGD(TAG_NVS,"accel_1_offset[1] = %d", accel_1_offset[1]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"accel_1_offset[1] не определен!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "accel_1_off_2", &accel_1_offset[2]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGD(TAG_NVS,"accel_1_offset[2] = %d", accel_1_offset[2]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"accel_1_offset[2] не определен!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "gyro_1_off_0", &gyro_1_offset[0]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGD(TAG_NVS,"gyro_1_offset[0] = %d", gyro_1_offset[0]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"gyro_1_offset[0] не определен!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "gyro_1_off_1", &gyro_1_offset[1]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGD(TAG_NVS,"gyro_1_offset[1] = %d", gyro_1_offset[1]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"gyro_1_offset[1] не определен!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "gyro_1_off_2", &gyro_1_offset[2]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGD(TAG_NVS,"gyro_1_offset[2] = %d", gyro_1_offset[2]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"gyro_1_offset[2] не определен!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "accel_2_off_0", &accel_2_offset[0]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGD(TAG_NVS,"accel_2_offset[0] = %d", accel_2_offset[0]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"accel_2_offset[0] не определен!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "accel_2_off_1", &accel_2_offset[1]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGD(TAG_NVS,"accel_2_offset[1] = %d", accel_2_offset[1]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"accel_2_offset[1] не определен!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "accel_2_off_2", &accel_2_offset[2]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGD(TAG_NVS,"accel_2_offset[2] = %d", accel_2_offset[2]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"accel_2_offset[2] не определен!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "gyro_2_off_0", &gyro_2_offset[0]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGD(TAG_NVS,"gyro_2_offset[0] = %d", gyro_2_offset[0]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"gyro_2_offset[0] не определен!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "gyro_2_off_1", &gyro_2_offset[1]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGD(TAG_NVS,"gyro_2_offset[1] = %d", gyro_2_offset[1]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"gyro_2_offset[1] не определен!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "gyro_2_off_2", &gyro_2_offset[2]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGD(TAG_NVS,"gyro_2_offset[2] = %d", gyro_2_offset[2]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"gyro_2_offset[2] не определен!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Ошибка считывания (%s)!\n", esp_err_to_name(err));
  }

  nvs_close(NVS_handle);
  }
}



//*********************************************************НЕПОСРЕДСТВЕННО НАЧИНАЕТСЯ ЗАДАЧА************************************************************************** */

 
//считываем из флеш калибровочные коэффициенты IMU и проверяем что они записаны

 NVS_reading_calibration_values();
    
  if ((gyro_1_offset[0]  || gyro_1_offset[1] || gyro_1_offset[2] || accel_1_offset[0] || accel_1_offset[1] || accel_1_offset[2]) == 0)
  {
    ESP_LOGE(TAG_FLY,"IMUs не откалиброваны, запуститесь в сервисном режиме и проведите калибровку IMU\n");
    uint16_t error_code = 10;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);}
  }

vTaskDelay(100/portTICK_PERIOD_MS);       //without these delay assertion fails

ESP_LOGI(TAG_FLY,"Создаем и запускаем таймер контроля зависания IMU#1.....");
Create_and_start_IMU_1_suspension_Timer();
ESP_LOGI(TAG_FLY,"Таймер контроля зависания IMU#1 запущен\n");

ESP_LOGI(TAG_FLY,"Создаем и запускаем таймер контроля зависания IMU#2.....");
Create_and_start_IMU_2_suspension_Timer();
ESP_LOGI(TAG_FLY,"Таймер контроля зависания IMU#2 запущен\n");

ESP_LOGI(TAG_FLY,"Создаем и запускаем таймер контроля зависания основного полетного цикла.....");
create_and_start_general_suspension_timer();
ESP_LOGI(TAG_FLY,"Таймер контроля зависания основного полетного цикла запущен\n");

ESP_LOGI(TAG_FLY,"Активируем прерывания на входах от IMU и MCP23017.....");
configure_pins_for_interrupt();
ESP_LOGI(TAG_FLY,"Прерывания от IMU и MCP23017 активированы\n");
#ifdef USING_W25N
start_time = get_time();
#endif

ESP_LOGI(TAG_FLY,"К ПОЛЕТУ ГОТОВ!\n");
   
//здесь начинается основной бесконечный цикл main_flying_cycle
while(1) 
{
    
      IMU_interrupt_status = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      
      if (IMU_interrupt_status != 0) {
      
      gpio_set_level(LED_RED, 0);

      large_counter++;                //увеличиваем глобальный счетчик циклов 

      if (IMU_interrupt_status == 14) //all is ok
      {
        SPI_read_bytes(MPU6000_1, 0, 0, 8, MPU6000_ACCEL_XOUT_H | SPI_READ_FLAG, 0, sensor_data_1, 14);
        SPI_read_bytes(MPU6000_2, 0, 0, 8, MPU6000_ACCEL_XOUT_H | SPI_READ_FLAG, 0, sensor_data_2, 14);
      }

      else if (IMU_interrupt_status == 15)  //2nd failed
      {
        SPI_read_bytes(MPU6000_1, 0, 0, 8, MPU6000_ACCEL_XOUT_H | SPI_READ_FLAG, 0, sensor_data_1, 14);
        for (i=0;i<14;i++) sensor_data_2[i] = sensor_data_1[i];
      }

      else if (IMU_interrupt_status == 16) //1st failed
      {
        SPI_read_bytes(MPU6000_2, 0, 0, 8, MPU6000_ACCEL_XOUT_H | SPI_READ_FLAG, 0, sensor_data_2, 14);
        for (i=0;i<14;i++) sensor_data_1[i] = sensor_data_2[i];
      } 
      
      else ledc_timer_pause(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0); //motors emergency stop

//проверяем данные с IMU1 на предмет если они вдруг все 0 или все 1
      test_for_all_0 = 0;
      test_for_all_1 = 0xFF;
      for (i=0;i<14;i++) 
      {
        test_for_all_0 = test_for_all_0 | sensor_data_1[i];  
        test_for_all_1 = test_for_all_1 & sensor_data_1[i];
      }
      if ((test_for_all_0 == 0) || (test_for_all_1 == 0xFF)) //если да - заменяем данными со второго IMU
      {
        for (i=0;i<14;i++) sensor_data_1[i] = sensor_data_2[i]; 
      }
//проверяем данные с IMU2 на предмет если они вдруг все 0 или все 1
      test_for_all_0 = 0;
      test_for_all_1 = 0xFF;
      for (i=0;i<14;i++) 
      {
        test_for_all_0 = test_for_all_0 | sensor_data_2[i]; 
        test_for_all_1 = test_for_all_1 & sensor_data_2[i];
      }
      if ((test_for_all_0 == 0) || (test_for_all_1 == 0xFF)) //если да - заменяем данными со второго IMU
      {
        for (i=0;i<14;i++) sensor_data_2[i] = sensor_data_1[i]; 
      }

//формируем "сырые" показания акселерометров и гироскопов
      accel_raw_1[0] = (sensor_data_1[0] << 8) | sensor_data_1[1];           //X
      accel_raw_1[1] = (sensor_data_1[2] << 8) | sensor_data_1[3];           //Y
      accel_raw_1[2] = (sensor_data_1[4] << 8) | sensor_data_1[5];           //Z
      gyro_raw_1[0] = (sensor_data_1[8] << 8) | sensor_data_1[9];            //X                  
      gyro_raw_1[1] = (sensor_data_1[10] << 8) | sensor_data_1[11];          //Y
      gyro_raw_1[2] = (sensor_data_1[12] << 8) | sensor_data_1[13];          //Z
      
      accel_raw_2[0] = (sensor_data_2[0] << 8) | sensor_data_2[1];           //X
      accel_raw_2[1] = (sensor_data_2[2] << 8) | sensor_data_2[3];           //Y
      accel_raw_2[2] = (sensor_data_2[4] << 8) | sensor_data_2[5];           //Z
      gyro_raw_2[0] = (sensor_data_2[8] << 8) | sensor_data_2[9];            //X                  
      gyro_raw_2[1] = (sensor_data_2[10] << 8) | sensor_data_2[11];          //Y
      gyro_raw_2[2] = (sensor_data_2[12] << 8) | sensor_data_2[13];          //Z

//вычисляем "чистые данные" от IMU с учетом калибровочных коэффициентов и переводим показания акселерометра в G, гироскопа в rad/s
      for (i=0;i<3;i++) 
        {
          accel_converted_1[i] = ((float)accel_raw_1[i] - (float)accel_1_offset[i]) / 8192.0;         // in G    8192
          gyro_converted_1[i]  = (((float)gyro_raw_1[i] - (float)gyro_1_offset[i]) / 131.0) * ((float)PI / 180.0);   //in rads

          accel_converted_2[i] = ((float)accel_raw_2[i] - (float)accel_2_offset[i]) / 8192.0;         // in G    8192
          gyro_converted_2[i]  = (((float)gyro_raw_2[i] - (float)gyro_2_offset[i]) / 131.0) * ((float)PI / 180.0);   //in rads
//накапливаем значения акселерометор и гироскопов для "редкого" вычисления углов Маджвиком          
          accel_converted_accumulated_1[i] += accel_converted_1[i];
          gyro_converted_accumulated_1[i] += gyro_converted_1[i];

          accel_converted_accumulated_2[i] += accel_converted_2[i];
          gyro_converted_accumulated_2[i] += gyro_converted_2[i];
        
        }
//Приступаем к расчетам углов фильтром Маджвика. 
//Расчет этот ведем каждый PID_LOOPS_RATIO цикл относительно получения данных от IMU.
//В данном случае данные от IMU поступают с частотой 1кГц, углы Маджвиком считаем с частотой 200Гц (каждый 5й цикл)        
      if ((large_counter % PID_LOOPS_RATIO) == 0) {

        for (i=0;i<madgwick_cycles;i++) {
//считываем время прошедшее с прошлого цифка расчета 
          ESP_ERROR_CHECK(gptimer_get_raw_count(GP_timer, &timer_value));
          ESP_ERROR_CHECK(gptimer_set_raw_count(GP_timer, 0));
//если используем модуль с компасом и GPS запускаем фильтр Маджвика с учетом данных от магнетометра, используя в качестве входных параметров усредненные накопленные за предыдущие циклы 
//показания гироскопов и акселерометров, а также считав данные из очереди от магнетометра
#ifdef USING_MAGNETOMETER 
        xQueueReceive(magnetometer_queue, mag_fresh_data, 0);

        MadgwickAHRSupdate((gyro_converted_accumulated_1[1] - gyro_converted_accumulated_2[0]) / (2.0 * PID_LOOPS_RATIO), 
                              (gyro_converted_accumulated_1[0] + gyro_converted_accumulated_2[1]) / (2.0 * PID_LOOPS_RATIO), 
                              ((gyro_converted_accumulated_1[2] * (-1.0)) - gyro_converted_accumulated_2[2]) / (2.0 * PID_LOOPS_RATIO), 
                              (-accel_converted_accumulated_1[1] + accel_converted_accumulated_2[0]) / (2.0 * PID_LOOPS_RATIO), 
                              (accel_converted_accumulated_1[0] + accel_converted_accumulated_2[1]) / (-2.0 * PID_LOOPS_RATIO), 
                              ((accel_converted_accumulated_1[2]) + accel_converted_accumulated_2[2]) / (2.0 * PID_LOOPS_RATIO), 
                              mag_fresh_data[1],            
                              mag_fresh_data[0],            
                              mag_fresh_data[2],   
                              timer_value);
//запрашиваем очередную порцию данных от магнетометра
        xTaskNotifyGive(task_handle_mag_read_and_process_data);    

//если не используем модуль с компасом и GPS - запускаем маджвика без учета данных от магнетометра используя в качестве входных параметров усредненные накопленные за предыдущие циклы 
//показания гироскопов и акселерометров      
#else
          MadgwickAHRSupdateIMU((gyro_converted_accumulated_1[1] - gyro_converted_accumulated_2[0]) / (2.0 * PID_LOOPS_RATIO) , 
                                (gyro_converted_accumulated_1[0] + gyro_converted_accumulated_2[1]) / (2.0 * PID_LOOPS_RATIO) ,
                                ((gyro_converted_accumulated_1[2] * (-1.0)) - gyro_converted_accumulated_2[2]) / (2.0 * PID_LOOPS_RATIO) , 
                                (-accel_converted_accumulated_1[1] + accel_converted_accumulated_2[0]) / (2.0 * PID_LOOPS_RATIO), 
                                (accel_converted_accumulated_1[0] + accel_converted_accumulated_2[1]) / (-2.0 * PID_LOOPS_RATIO), 
                                ((accel_converted_accumulated_1[2]) + accel_converted_accumulated_2[2]) / (2.0 * PID_LOOPS_RATIO), 
                                timer_value);
    
#endif
        }
//если не используем модуль с компасом и GPS - запускаем маджвика без учета данных от магнетометра используя в качестве входных параметров усредненные накопленные за предыдущие циклы 
//показания гироскопов и акселерометров          
        
//очищаем накопленные данные от гироскопов и акселерометров         
        for (i=0;i<3;i++)
        {
          accel_converted_accumulated_1[i] = 0;
          gyro_converted_accumulated_1[i] = 0;

          accel_converted_accumulated_2[i] = 0;
          gyro_converted_accumulated_2[i] = 0;
        }
//выполняем преобразование из кватерниона в углы Эйлера   
        Convert_Q_to_degrees();
      }     

//производим запрос данных от INA219 (раз в X циклов, то есть (1000/X) в секунду) 
        if ((large_counter % 25) == 0) xTaskNotifyGive(task_handle_INA219_read_and_process_data);      //по результатам считывания корректируем уровень газа, поэтому очень медленно нельзя
#ifdef USING_TFMINIS_I2C       
        if (((large_counter + 5) % 25) == 0) xTaskNotifyGive(task_handle_lidar_read_and_process_data);
#endif

#ifdef USING_MS5611
        if (((large_counter + 23) % 1000) == 0) xTaskNotifyGive(task_handle_MS5611_read_and_process_data); 
#endif
        //printf("%0.2f\n", voltage_correction_coeff);
//далее место где удобно что-то выводить, печатать 

//if ((large_counter % 1000) == 0) ESP_LOGI(TAG_FLY,"%0.3f, %0.3f, %0.3f, %0.3f", accel_rotated[0], accel_rotated[1], accel_rotated[2], accel_rotated[3]);
//if ((large_counter % 1000) == 0) ESP_LOGI(TAG_FLY,"fil are %0.2f, %0.2f, %0.2f, %0.2f", engine_filtered[0],engine_filtered[1],engine_filtered[2],engine_filtered[3]);
//if ((large_counter % 1000) == 0) ESP_LOGI(TAG_FLY,"new are %0.2f, %0.2f, %0.2f, %0.2f", engine_filtered_new[0],engine_filtered_new[1],engine_filtered_new[2],engine_filtered_new[3]);        
//if ((large_counter % 1000) == 0) ESP_LOGI(TAG_FLY,"%0.3f, %0.3f, %0.3f, %0.3f,    //%0.3f, %0.3f, %0.3f", q0, q1, q2, q3, pitch, roll, yaw);
//if ((large_counter % 1000) == 0) printf("%0.1f\n", lidar_fresh_data.altitude); // баро высота в cm

/*        if ((large_counter % 5000) == 0) 
        {
          UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
          ESP_LOGW(TAG_FLY,"High watermark %d",  uxHighWaterMark);
        }       
            if ((large_counter % 2000) == 0) {
              command_for_PCA9685 +=20;
              if (command_for_PCA9685 > 0x0260) command_for_PCA9685 = 0x0201;
              xQueueSend(PCA9685_queue, &command_for_PCA9685, NULL);}


        //if (large_counter > 10000) {vTaskDelay(1000/portTICK_PERIOD_MS);}   
        if ((large_counter % 1000) == 0)
        {
          uint32_t time_start = get_time();
          //printf("Target %f, %f, %f\n", rc_fresh_data.received_pitch, rc_fresh_data.received_roll,yaw_setpoint);
          //printf("Target Q ");
          calculate_desired_quat_from_3_angles(rc_fresh_data.received_pitch, rc_fresh_data.received_roll, yaw_setpoint, q_from_RC);
          //for (int i = 0; i<4;i++) printf("%f ", q_from_RC[i]);
          //printf("\n");
          //printf("Current Q %f, %f, %f, %f\n", q0, q1, q2, q3);
          q_current[0] = q0;
          q_current[1] = q1;
          q_current[2] = q2;
          q_current[3] = q3;
          //printf("Q_difference ");
          quaternion_difference(q_current, q_from_RC, q_difference);
          //for (int i = 0; i<4;i++) printf("%f ", q_difference[i]);
          //printf("\n");

          angle_and_axis_from_quat(q_difference, &q_angle, q_axis);
          printf("Angle: %f ", q_angle);
          printf("Axis: ");
          for (int i = 1; i<4;i++) printf("%f ", q_axis[i]);
          printf("\n");

          printf("%f, %f, %f", q_angle * q_axis[1],q_angle * q_axis[2], q_angle * q_axis[3]);
          printf("\n\n");

          uint32_t time_finish = get_time();
          //printf("%ld\n", time_finish - time_start);


        }
*/        
        
        //SP_ERROR_CHECK(gpio_reset_pin(MPU6000_2_INTERRUPT_PIN));
        //printf ("%ld\n", IMU_interrupt_status); 
        //printf(" %0.1f, %0.1f\n", yaw_setpoint, yaw);
        //printf("%d\n", data_to_send_to_rc.power_voltage_value);
//        xTaskNotifyGive(task_handle_INA219_read_and_process_data);

        //printf("%0.1f, %0.1f, %0.1f\n", gyro_converted_1[1],(-1)*gyro_converted_2[0], (gyro_converted_1[1] - gyro_converted_2[0]) / 2.0 );  
        //ESP_LOGI(TAG_FLY,"%0.1f", rc_fresh_data.received_yaw);
         
        //ESP_LOGI(TAG_FLY,"ADC is %d ", data_to_send_to_rc.power_voltage_value);
          //ESP_LOGI(TAG_FLY,"fil are %0.2f, %0.2f, %0.2f, %0.2f", engine_filtered[0],engine_filtered[1],engine_filtered[2],engine_filtered[3]);
          //ESP_LOGI(TAG_FLY,"%0.2f, %0.2f, %0.2f, %0.2f", ((((gyro_converted_1[2] * (-1.0)) - gyro_converted_2[2]) / 2.0) * (180.0 / (float)PI)), rc_fresh_data.received_yaw, error_yaw_rate, pid_yaw_rate);
          //ESP_LOGI(TAG_FLY,"%0.2f, %0.2f, %0.2f", rc_fresh_data.received_pitch, rc_fresh_data.received_roll, rc_fresh_data.received_yaw);
        
        
        //printf("%0.1f\n", rc_pitch_filtered);

//получаем свежие данные из очереди от пульта управления. Если данные успешно получены - информируем задачу моргания полетными огнями что моргаем в штатном режиме (режим "1")
    
        if (xQueueReceive(remote_control_to_main_queue, &rc_fresh_data, 0)) 
        {
          remote_control_lost_comm_counter = 0; 
          xTaskNotify(task_handle_blinking_flight_lights,1,eSetValueWithOverwrite);
        }
//в противном случае инкрементируем счетчик, определяющий допустимое время без связи с пультом если были уже в полете - фиксируем уровень газа на некоем уровне и ставим в нули управление по углам.     
        else remote_control_lost_comm_counter++;
//если счетчик превысил порог - фиксируем уровень газа на некоем предустановленном значении и все управляющие сигналы в ноль (то есть в идеале висение на месте) 
        if (remote_control_lost_comm_counter > RC_NO_COMM_DELAY_MAIN_CYCLES)  
        {
          if (rc_fresh_data.engines_start_flag) {
            remote_control_lost_comm_counter = RC_NO_COMM_DELAY_MAIN_CYCLES;
            throttle = RC_NO_COMM_THROTTLE_HOVER_VALUE;
            rc_fresh_data.received_pitch = 0;
            rc_fresh_data.received_roll = 0;
            rc_fresh_data.received_yaw = 0;
            rc_fresh_data.altitude_hold_flag = 0;     //выходим из режима удержания высоты если был включен
          }
//и оповещаем задачу моргания полетными огнями моргать в аварийном режиме (режим "0")
          xTaskNotify(task_handle_blinking_flight_lights,0,eSetValueWithOverwrite);  
        }
#ifdef USING_GPS
//получаем свежие данные из очереди от GPS
        if (xQueueReceive(gps_to_main_queue, &gps_fresh_data, 0)) 
        {
          //ESP_LOGI(TAG_FLY,"Широта %" PRIu64 " Долгота %" PRIu64, gps_fresh_data.latitude_d, gps_fresh_data.longtitude_d);
        } 
#endif

//получаем свежие данные из очереди от лидара
#ifdef USING_TFMINIS_I2C         
        if (xQueueReceive(lidar_to_main_queue, &lidar_fresh_data, 0)) {
          new_lidar_data_arrived_flag = 1;
          ESP_LOGD(TAG_FLY,"высота %0.3f", lidar_fresh_data.altitude); // высота в cm
          if (lidar_fresh_data.altitude < 900) lidar_altitude_corrected = lidar_fresh_data.altitude * cos(pitch * 0.017453292) * cos(roll * 0.017453292);// PI/180
      }
#endif

#ifdef USING_MS5611         
        if (xQueueReceive(MS5611_to_main_queue, &baro_altitude, 0)) {
            //printf("%0.1f\n", baro_altitude); // баро высота в cm
      }
#endif

//получаем свежие данные из очереди от INA219
if (xQueueReceive(INA219_to_main_queue, &INA219_fresh_data, 0))
      {
        //SP_LOGE(TAG_FLY, "V: %0.4fV",INA219_fresh_data[1]);
      }

//далее разбираемся с полетным режимом
// если двигатели запущены - понимаем стоит ли режим удержания высоты. Если да - замещаем полученное от пульта значение газа вычисленным автоматически на основание данных от лидара 
        if (rc_fresh_data.engines_start_flag)
        {
          throttle = rc_fresh_data.received_throttle; //если двигатели запущены устанавливаем уровень газа в соответствие с полученным от пульта
                 
#ifdef USING_TFMINIS_I2C 
        if ((rc_fresh_data.altitude_hold_flag) && (lidar_fresh_data.altitude < 900)) 
          {
            if (altitude_hold_mode_enabled == 0)                                      //если только включили режим, первый раз в цикл вошли
            {
              altitude_hold_mode_enabled = 1;
              altitude_setpoint = lidar_altitude_corrected;                          //запоминаем высоту                          
              alt_hold_initial_throttle = rc_fresh_data.received_throttle;            //запоминаем уровень газа (должен быть около висения)
            }
            if (new_lidar_data_arrived_flag)                                          //если есть свежие данные от лидара            
            {                                                                         //то считаем уровень газа
              new_lidar_data_arrived_flag = 0;
              if (rc_fresh_data.raw_throttle > 3500) altitude_setpoint +=0.25;
              if (rc_fresh_data.raw_throttle < 500) altitude_setpoint -=0.25;

              if (altitude_setpoint >= 800) altitude_setpoint = 800;
              if (altitude_setpoint <= 10) altitude_setpoint = 10;

              //printf("%0.2f\n", altitude_setpoint);

              pid_altitude = PID_Compute(&alt_hold_pid, altitude_setpoint, lidar_altitude_corrected, 0, 0);
            }
            throttle = alt_hold_initial_throttle + pid_altitude;    //заменяем значение throttle от пульта вычисленным при помощи регулятора
          }
        else 
          {
            altitude_hold_mode_enabled = 0; 
            alt_hold_pid.prev_error = 0;
            alt_hold_pid.integral_error = 0;
          }
#endif  

//на основании всех полученных выше данных считаем двухконтурным ПИД-регулятором управляющие воздействия на двигатели по углам
        calculate_pids_2();
        }
//если от пульта команда что двигатели выключены - сбрасываем интегральные составляющие ПИД
        else 
        {
          yaw_setpoint = yaw;
          pitch_pid_angle.integral_error = roll_pid_angle.integral_error = 0;
          pitch_pid_angle.prev_error = roll_pid_angle.prev_error = 0;
          pitch_pid_rate.integral_error = roll_pid_rate.integral_error = yaw_pid_rate.integral_error = 0;
          pitch_pid_rate.prev_error = roll_pid_rate.prev_error = yaw_pid_rate.prev_error = 0;
          integral_yaw_error_angle = error_yaw_angle_old = 0;
          engine[0] = engine[1] = engine[2] = engine[3] = ENGINE_PWM_MIN_DUTY;

          altitude_hold_mode_enabled = 0;
        }
//фильтруем вычисленные ПИД регулятором данные        
        ESC_input_data_filter();
//отправляем данные на двигатели        
        update_engines();
//на этом цикл от считывания данных от IMU до выдачи сигналов на двигатели фактически закончен

//собираем данные телеметрии и отправляем их в очередь на отправку на пульт
        data_to_send_to_rc.pitch = pitch;
        data_to_send_to_rc.roll = roll;
        data_to_send_to_rc.yaw = yaw;
        data_to_send_to_rc.power_voltage_value = (uint16_t)(INA219_fresh_data[0]*10.0);
#ifdef USING_TFMINIS_I2C 
        data_to_send_to_rc.altitude = (uint16_t)lidar_altitude_corrected;
#endif
        xQueueOverwrite(main_to_rc_queue, (void *) &data_to_send_to_rc);         

//подготавливаем данные для записи в логи и записываем по flash
#ifdef USING_W25N
        if ((large_counter % 50) == 0) 
        {       
          prepare_logs();
          xQueueSend(W25N01_queue, &p_to_log_structure, 0);
        }
#endif

#ifdef USING_MAVLINK_TELEMETRY
        if ((large_counter % 200) == 0) 
        {       
          prepare_mavlink_data();
          xQueueSend(main_to_mavlink_queue, &p_to_mavlink_data, 0);
        }
#endif

        gpio_set_level(LED_RED, 1);
        IMU_interrupt_status = 0;
//сбрасываем таймер общего зависания, по сработке которого аварийно останавливаем двигатели        
        ESP_ERROR_CHECK(gptimer_set_raw_count(general_suspension_timer, 0));
      
    } 
  }
}
