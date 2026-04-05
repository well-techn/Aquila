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
#include "freertos/ringbuf.h"
#include "dsps_biquad.h"
#include "dsps_biquad_gen.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "wt_spi.h"
#include "PCA9685.h"
#include "MCP23017.h"
#include "winbondW25N.h"
#include "PMW3901.h"
#include "MPU6000.h"
#include "TfminiS.h"
#include "INA219.h"
#include "IST8310.h"
#include "FL3195.h"
#include "MS5611.h"
#include "filters.h"
#include "error_code_LED_blinking.h"
#include "pid.h"
#include "q_operations.h"
#include "emergency_mode.h"
#include "NVS_operations.h"
#include "timers_operations.h"
#include "gpio_operations.h"
#include "advanced_math.h"
#include "ahrs_provider.h"
#include "esc_control_provider.h"

//handles таймеров
gptimer_handle_t ahrs_timer;
gptimer_handle_t general_suspension_timer;
gptimer_handle_t IMU_1_suspension_timer;
gptimer_handle_t IMU_2_suspension_timer;

//handles SPI
extern spi_device_handle_t W25N01;
extern spi_device_handle_t MPU6000_1;
extern spi_device_handle_t MPU6000_2;

//handlers зачад
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
extern TaskHandle_t task_handle_px4flow_read_and_process_data;
extern TaskHandle_t task_handle_emergency_mode;
extern TaskHandle_t task_handle_fft;

//handlers очередей
extern QueueHandle_t magnetometer_queue;
extern QueueHandle_t remote_control_to_main_queue;
extern QueueHandle_t remote_control_to_main_pid_queue; 
extern QueueHandle_t lidar_to_main_queue;
extern QueueHandle_t INA219_to_main_queue;
extern QueueHandle_t main_to_rc_queue;
extern QueueHandle_t W25N01_queue;
extern QueueHandle_t main_to_mavlink_queue;
extern QueueHandle_t gps_to_main_queue;
extern QueueHandle_t MS5611_to_main_queue;
extern QueueHandle_t px4flow_to_main_queue;
extern QueueHandle_t fft_to_main_queue;

//для логгирования
extern char *TAG_INIT;
extern const char *TAG_FLY;
extern const char *TAG_NVS;

/********************************************************************    ПРЕРЫВАНИЯ    ***********************************************************************************************/
//прерывания от GPIO
//контролирует прерывания от 2х IMU и пина аварйиной остановки.
//для IMU контролирует что сигнал пришел вовремя, без зерержки и передает в основной цикл соответствующий статус
//по полученному статусу main_flying_cycle может замещать данные потенциально неисправного IMU 
//по крнопке аварийной остановки останавливает main_flying_cycle и запускает emergency_task
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
  
   if (gpio_intr_status_1 & (1ULL << A2))   //сигнал от кнопки аварийной остановки на Holybro M9N, выключаем двигатели
  {
    gpio_set_level(A3, 0);                        //включить светодиод на кнопке аварийной остановки
    xTaskGenericNotifyFromISR(task_handle_emergency_mode, 0, 0x01 << 13, eSetValueWithOverwrite, NULL, &xHigherPriorityTaskWoken); //активировать аварийную задачу, передавая ей код вызвавшей ошибки
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken);
  }  
  
  if (gpio_intr_status_1 & (1ULL << MPU6000_1_INTERRUPT_PIN))   //сигнал от IMU1
  {
    imu_1_interrupt_flag = 1; 
    gptimer_get_raw_count(IMU_1_suspension_timer,&IMU_1_timer_value);
    gptimer_set_raw_count(IMU_1_suspension_timer, 0);
    gptimer_get_raw_count(IMU_2_suspension_timer,&IMU_2_timer_value);
  } 
  
  if (gpio_intr_status_1 & (1ULL << MPU6000_2_INTERRUPT_PIN))   //сигнал от IMU2 
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
    xTaskGenericNotifyFromISR(task_handle_main_flying_cycle, 0, 5, eSetValueWithOverwrite, NULL,  &xHigherPriorityTaskWoken);    //5 - код что все ок
  }

  else if (IMU_2_timer_value >= IMU_SUSPENSION_TIMER_DELAY_MS * 1000)  //IMU2 не выдал сигнал
  {
    imu_1_interrupt_flag = 0;
    imu_2_interrupt_flag = 0;
    xTaskGenericNotifyFromISR(task_handle_main_flying_cycle, 0, 2, eSetValueWithOverwrite, NULL, &xHigherPriorityTaskWoken);     //2 - код что 2ой завис
  }

  else if (IMU_1_timer_value >= IMU_SUSPENSION_TIMER_DELAY_MS * 1000)  //IMU1 не выдал сигнал
  {
    imu_1_interrupt_flag = 0;
    imu_2_interrupt_flag = 0;
    xTaskGenericNotifyFromISR(task_handle_main_flying_cycle, 0, 1, eSetValueWithOverwrite, NULL, &xHigherPriorityTaskWoken);     //1 - код что 1ый завис
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//прерывание от таймера зависания основного полетного цикла
//этот таймер сбрасывается в основном цикле полета. Если он переполнился - значит основной цикл завис, аварийно останавливаем двигетели
static bool IRAM_ATTR general_suspension_timer_interrupt_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)  
{ 
  BaseType_t xHigherPriorityTaskWoken  = pdFALSE;
  gpio_set_level(A3, 0);                        //включить светодиод на кнопке аварийной остановки
  xTaskGenericNotifyFromISR(task_handle_emergency_mode, 0, 0x01 << 14, eSetValueWithOverwrite, NULL, &xHigherPriorityTaskWoken); //активировать аварийную задачу, передавая ей код вызвавшей ошибки

  return (xHigherPriorityTaskWoken == pdTRUE); 
}



/********************************************************************      ЗАДАЧА      ***********************************************************************************************/

/*Задача основного полетного цикла. Единственная задача на ядре 1. 
  - Активируется по прерыванию о готовности данных от IMU.
  - Считывает данные от IMU, обрабатывает, вычисляет Маджвиком (с учетом магнетометра или без) углы наклона
  - получает из очереди команды от пульта
  - на основе данных по текущим углам и команд от пульта вычисляет необходимые воздействия на двигатели посредством двухконтурного PID
  - выдает эти воздействия на моторы
  - подготавливает данные для записи логов и выдает их в соответствующую очередь 

Сначала идут переменные, потом вспомогательные функции, потом само тело задачи.
*/
void main_flying_cycle(void * pvParameters)
{
  
  uint32_t IMU_interrupt_status = 0;

  uint8_t sensor_data_1[20] = {0};
  uint8_t sensor_data_2[20] = {0};

  uint8_t sensor_data_1_old[20] = {0};
  uint8_t sensor_data_2_old[20] = {0};

  uint8_t test_for_equal_prev = 0;
  uint8_t test_for_all_equal = 0;
  uint8_t counter_for_equal_prev_IMU1 = 0;
  uint8_t counter_for_equal_prev_IMU2 = 0;
  uint8_t IMU_1_data_fail = 0;
  uint8_t IMU_2_data_fail = 0;

  float accel_raw_1[3] = {0};
  float accel_raw_2[3] = {0};  
  float gyro_raw_1[3] = {0}; 
  float gyro_raw_2[3] = {0};

  uint32_t large_counter = 0;
  uint64_t timer_value = 0;
// __attribute__((aligned(16))) необходимо для использования векторных инстукций библиотеки dsp  
  float __attribute__((aligned(16))) gyro_1_offset[3] = {0};
  float __attribute__((aligned(16))) gyro_2_offset[3] = {0};  
//для гироскопов единичные матрицы, так как хз как их формировать
  float __attribute__((aligned(16))) gyro_1_A_inv[9] = {1,0,0,0,1,0,0,0,1};
  //float __attribute__((aligned(16))) gyro_2_A_inv[9] = {1,0,0,0,1,0,0,0,1};

  float __attribute__((aligned(16))) accel_1_bias[3] = {0};
  float __attribute__((aligned(16))) accel_1_A_inv[9] = {0};
  
  float __attribute__((aligned(16))) accel_2_bias[3] = {0};
  float __attribute__((aligned(16))) accel_2_A_inv[9] = {0};

  float __attribute__((aligned(16))) calibration_temp[9] = {0};

  float __attribute__((aligned(16))) gyro_1_converted[3] = {0};
  float __attribute__((aligned(16))) accel_1_converted[3] = {0};
  float __attribute__((aligned(16))) gyro_2_converted[3] = {0};
  float __attribute__((aligned(16))) accel_2_converted[3] = {0};

  float accel_avg[3] = {0};
  float gyro_avg[3] = {0};
  float mag_fresh_data[3] = {0};
   

#ifdef ACCEL_AND_ANGLES_SAFETY_MEASURES
  float accel_1_total = 0;
  float accel_2_total = 0;
  float accel_1_total_max = 0;
  float accel_2_total_max = 0;
  uint8_t accel_1_exceed_limit_counter = 0;
  uint8_t accel_2_exceed_limit_counter = 0;
  uint8_t pitch_exceed_limit_counter = 0;
  uint8_t roll_exceed_limit_counter = 0;
#endif

  static float pitch = 0; 
  static float roll = 0; 
  static float yaw = 0; 

  static float throttle;

  static float vertical_acceleration = 0;

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
    pitch_pid_angle.alpha = 0.6f;
    pitch_pid_angle.integral_error = 0;
    pitch_pid_angle.integral_limit = 1000.0f;
    pitch_pid_angle.pid_limit = 2000.0f;
    pitch_pid_angle.throttle_limit = 0.3f;

  PIDController_t pitch_pid_rate;
    pitch_pid_rate.kp = 8.8 / 6553.0f;
    pitch_pid_rate.kd = 760  / 6553.0f;      //860
    pitch_pid_rate.ki = 0.03  / 6553.0f;
    pitch_pid_rate.prev_error = 0;
    pitch_pid_rate.alpha = 0.6;
    pitch_pid_rate.integral_error = 0;
    pitch_pid_rate.integral_limit = 1000.0f / 6553.0f;
    pitch_pid_rate.pid_limit = 3000.0f / 6553.0f;
    pitch_pid_rate.throttle_limit = 0.3f;

  float pid_roll_angle = 0;
  float pid_roll_rate = 0;
  float gyro_roll = 0;
  float gyro_roll_old = 0;

  PIDController_t roll_pid_angle;
    roll_pid_angle.kp = 4.2;
    roll_pid_angle.kd = 8;
    roll_pid_angle.ki = 0.0004f;
    roll_pid_angle.prev_error = 0;
    roll_pid_angle.alpha = 0.6f;
    roll_pid_angle.integral_error = 0;
    roll_pid_angle.integral_limit = 1000.0f;
    roll_pid_angle.pid_limit = 2000;
    roll_pid_angle.throttle_limit = 0.3f;

  PIDController_t roll_pid_rate;
    roll_pid_rate.kp = 8.8f / 6553.0f;
    roll_pid_rate.kd = 760.0f / 6553.0f;     //860
    roll_pid_rate.ki = 0.03f / 6553.0f;
    roll_pid_rate.prev_error = 0;
    roll_pid_rate.alpha = 0.6f;
    roll_pid_rate.integral_error = 0;
    roll_pid_rate.integral_limit = 1000.0f / 6553.0f;
    roll_pid_rate.pid_limit = 3000.0f / 6553.0f;
    roll_pid_rate.throttle_limit = 0.3f;

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

  PIDController_t yaw_pid_rate;
    yaw_pid_rate.kp = 100.0f / 6553.0f;
    yaw_pid_rate.kd = 0;
    yaw_pid_rate.ki = 0.01f / 6553.0f;
    yaw_pid_rate.prev_error = 0;
    yaw_pid_rate.alpha = 0;
    yaw_pid_rate.integral_error = 0;
    yaw_pid_rate.integral_limit = 1000.0f / 6553.0f;
    yaw_pid_rate.pid_limit = 3000.0f / 6553.0f;
    yaw_pid_rate.throttle_limit = 0.3f;

  float engine[4] = {0}; 
  float engine_filtered[4] = {0};

  float engines_filter_pool[4][LENGTH_OF_ESC_FILTER] = {0};

  data_from_rc_to_main_struct rc_fresh_data;
    rc_fresh_data.mode = 0;
    rc_fresh_data.engines_start_flag = 0;
    rc_fresh_data.lidar_altitude_hold_flag = 0;
    rc_fresh_data.baro_altitude_hold_flag = 0;
    rc_fresh_data.trim_pitch = 0;
    rc_fresh_data.trim_roll = 0;
  data_from_main_to_rc_struct data_to_send_to_rc;
  uint32_t remote_control_lost_comm_counter = 0;

  pid_coeff_data_from_rc_to_main_struct pid_fresh_data;
    pid_fresh_data.kp_alt_hold_coeff = 0;
    pid_fresh_data.ki_alt_hold_coeff = 0;
    pid_fresh_data.kd_alt_hold_coeff = 0;

#ifdef USING_GPS
 data_from_gps_to_main_struct_t gps_fresh_data;
  gps_fresh_data.latitude_d = 111111111;          //стартовые значения, чтобы не было рандомных цифр при отсутствии GPS
  gps_fresh_data.longtitude_d = 222222222;
#endif  

//переменные по режиму удержания высоты
  bool lidar_altitude_hold_mode_enabled = 0;
  bool baro_altitude_hold_mode_enabled = 0;
  float altitude_setpoint = 0;
  float alt_hold_initial_throttle = 0;
  uint8_t new_lidar_data_arrived_flag = 0;
  uint8_t new_baro_data_arrived_flag = 0;
  float pid_altitude = 0;

  PIDController_t lidar_alt_hold_pid;
    lidar_alt_hold_pid.kp = 4.5f / 6553.0f;  //10       //  раб значения 8, 350, 0.2
    lidar_alt_hold_pid.kd = 200.0f / 6553.0f; //306      //
    lidar_alt_hold_pid.ki = 0.10f / 6553.0f;  //0.4       //
    lidar_alt_hold_pid.prev_error = 0;
    lidar_alt_hold_pid.alpha = 0.75f; //0.6
    lidar_alt_hold_pid.integral_error = 0;
    lidar_alt_hold_pid.integral_limit = 250.0f / 6553.0f;
    lidar_alt_hold_pid.pid_limit = 1000.0f / 6553.0f;    //500
    lidar_alt_hold_pid.throttle_limit = 0;
  
  PIDController_t baro_alt_hold_pid;
    baro_alt_hold_pid.kp = 4.5f / 6553.0f;        //  раб значения 5, 175, 0.25
    baro_alt_hold_pid.kd = 200.0f / 6553.0f;       
    baro_alt_hold_pid.ki = 0.10f / 6553.0f;         
    baro_alt_hold_pid.prev_error = 0;
    baro_alt_hold_pid.alpha = 0;
    baro_alt_hold_pid.integral_error = 0;
    baro_alt_hold_pid.integral_limit = 250.0f / 6553.0f;
    baro_alt_hold_pid.pid_limit = 500.0f / 6553.0f;
    baro_alt_hold_pid.throttle_limit = 0;

#ifdef USING_TFMINIS_I2C 
    data_from_lidar_to_main_struct lidar_fresh_data;
      lidar_fresh_data.altitude = 0;
      lidar_fresh_data.strength = 0;
      lidar_fresh_data.valid = 0;
    float lidar_altitude_corrected = 0;
#endif

  float INA219_fresh_data[4];
  float voltage_correction_coeff = 1.0f;

//переменные для логирования
  uint64_t start_time = 0;
  struct logging_data_set set_to_log = {0};
  struct logging_data_set* p_to_log_structure = &set_to_log;

#ifdef USING_MAVLINK_TELEMETRY
  mavlink_data_set_t main_to_mavlink_data;
    main_to_mavlink_data.latitude = 111111111;     //стартовые значения, чтобы не было рандомных цифр при отсутствии GPS
    main_to_mavlink_data.longtitude = 222222222;
  mavlink_data_set_t* p_to_mavlink_data = &main_to_mavlink_data;
#endif

#ifdef USING_MS5611
  float baro_altitude_cm = 0;
#endif

#ifdef USING_PX4FLOW
  data_from_px4flow_to_main_struct_t px4flow_fresh_data;
#endif

KalmanFilter2d_t Kalm_vert;
  Kalm_vert.dt = 0.001f;                             // интервал времени
  Kalm_vert.sigma2_accel = ACCEL_SIGMA2;            // дисперсия акселерометра, постоянная величина как характеристика акселя
  Kalm_vert.sigma2_baro = BARO_SIGMA2;              // дисперсия барометра, постоянная величина как характеристика барометра

  Kalm_vert.P[0][0] = 1;
  Kalm_vert.P[0][1] = 0;
  Kalm_vert.P[1][0] = 0;
  Kalm_vert.P[1][1] = 1;                  // Матрица неопределенности прогноза
  
  Kalm_vert.K[0] = 0;                     // коэффициент Калмана
  Kalm_vert.K[1] = 0; 
  Kalm_vert.h = 0;                        //высота
  Kalm_vert.v = 0;   

  float q_current[4] = {1,0,0,0};
  float accel_1_full[4] = {0};
  float accel_1_rotated[4] = {0};
  float accel_2_full[4] = {0};
  float accel_2_rotated[4] = {0};

  float accel_z_ave_rotated_filtered = 0;

  Butterworth_t accel_Btw_filter;

  float local_x_velocity = 0;
  float local_x_displacement = 0;

  float local_y_velocity = 0;
  float local_y_displacement = 0;

  uint32_t flight_time = 0;

  typedef enum {
    GROUND_STATE,
    FLIGHT_STATE
  } drone_state;

  drone_state current_state = GROUND_STATE;

  nvs_handle_t coeff_NVS_handle;
  nvs_handle_t flight_time_NVS_handle;

  //переменные для контроля времени выполнения одного цикла
  uint32_t start_CPU_cycles = 0;
  uint32_t current_cycle_us = 0;
  float avg_cycle_us = 0;
  uint32_t max_cycle_us = 0;

#ifdef USING_FFT
//переменные для расчетв FFT
  float gyro_samples_for_fft[FFT_HOP_SIZE] = {0};
  uint16_t samples_since_last_fft = 0;
  extern RingbufHandle_t fft_rb_handle;
  float fft_peak_frequency = 0;

//переменные для фильтрации по результатам FFT
//Состояния фильтра для 3-х осей (память фильтра)
  float gyro_x_w[2] = {0, 0};
  float gyro_y_w[2] = {0, 0};
  float gyro_z_w[2] = {0, 0};

float notch_bandwith = 10.0f;

//Массив коэффициентов: {b0, b1, b2, a1, a2}
  float notch_coeffs[5];

  float gyro_2_converted_filtered[3] = {0};



#endif
 

//*********************************************************************************************************************************************************************** 
//*************************************************************** вспомогательные функции ************************************************************************************ 
  void calculate_pids_2(void) {

//внешний pitch по углу
//в зависимости от ошибки по углу (текущее относительно желаемого) выдает необходимую угловую скорость   
    pid_pitch_angle = PID_Compute(&pitch_pid_angle,rc_fresh_data.received_pitch, pitch, throttle);
//внутренний pitch по угловой скорости
//на основании данных от внешнего цикла (желаемая угловая скорость) и данных от гироскопа (текущая) вычисляет управляющее воздействие
    gyro_pitch = 0.1f * gyro_avg[1] + 0.9f * gyro_pitch_old;
    gyro_pitch_old = gyro_pitch;
    //pid_pitch_rate = PID_Compute(&pitch_pid_rate,rc_fresh_data.received_pitch, gyro_pitch, throttle);
    pid_pitch_rate = PID_Compute(&pitch_pid_rate,pid_pitch_angle, gyro_pitch, throttle);

//внешний roll по углу   
    pid_roll_angle = PID_Compute(&roll_pid_angle,rc_fresh_data.received_roll, roll, throttle);
//внутренний roll по угловой скорости
    gyro_roll = 0.1f * gyro_avg[0] + 0.9f * gyro_roll_old;
    gyro_roll_old = gyro_roll;
    //pid_pitch_rate = PID_Compute(&roll_pid_rate,rc_fresh_data.received_roll, gyro_roll, throttle);  
    pid_roll_rate = PID_Compute(&roll_pid_rate,pid_roll_angle, gyro_roll, throttle);
    
//выставляем yaw_setpoint интегрируя данные от пульта
    yaw_setpoint = yaw_setpoint + 0.0016f * rc_fresh_data.received_yaw;  //чем больше этот коэффициент тем выше скорость изменения yaw
    if (yaw_setpoint >= 360) yaw_setpoint = yaw_setpoint - 360.0;
    if (yaw_setpoint < 0) yaw_setpoint = yaw_setpoint + 360.0;

//внешний yaw по углу
    if (throttle < 0.2f) yaw_setpoint = yaw;  //чтобы ошибка не накапливалась пока стоит на земле
    error_yaw_angle = -yaw_setpoint + yaw;
    if (error_yaw_angle <= -180.0f) error_yaw_angle = error_yaw_angle + 360.0f; 
    if (error_yaw_angle >= 180.0f)  error_yaw_angle = error_yaw_angle - 360.0f;

    integral_yaw_error_angle = integral_yaw_error_angle + Ki_yaw_angle * error_yaw_angle;
    if (integral_yaw_error_angle > 200.0f) integral_yaw_error_angle = 200.0f;
    if (integral_yaw_error_angle < -200.0f) integral_yaw_error_angle = -200.0f;

    pid_yaw_angle = Kp_yaw_angle * error_yaw_angle + integral_yaw_error_angle + Kd_yaw_angle * (error_yaw_angle - error_yaw_angle_old);
    if (pid_yaw_angle > 360.0f)  pid_yaw_angle = 360.0f;
    if (pid_yaw_angle < -360.0f) pid_yaw_angle = -360.0f;
    error_yaw_angle_old = error_yaw_angle;
   
//внутренний yaw по угловой скорости
    gyro_yaw = gyro_avg[2]; 
    //pid_yaw_rate = PID_Compute(&yaw_pid_rate, rc_fresh_data.received_yaw, gyro_yaw, throttle);
    pid_yaw_rate = PID_Compute(&yaw_pid_rate, pid_yaw_angle, gyro_yaw, throttle);

//суммируем результаты для выдачи на моторы
    engine[0] = throttle + pid_pitch_rate + pid_roll_rate - pid_yaw_rate;
    engine[1] = throttle + pid_pitch_rate - pid_roll_rate + pid_yaw_rate;
    engine[2] = throttle - pid_pitch_rate - pid_roll_rate - pid_yaw_rate;
    engine[3] = throttle - pid_pitch_rate + pid_roll_rate + pid_yaw_rate;

//ниже идет компенсация просадки напряжения АКБ
//11.1 - номинальное напряжение 3S АКБ, 12,6 максимальное, 9,9 минимальное 
    voltage_correction_coeff = 0.9f * voltage_correction_coeff + 0.1f * (11.1f / INA219_fresh_data[0]);
//ограничения на коэффициент сверху и снизу                  
    if (voltage_correction_coeff < (11.1f/12.6f)) voltage_correction_coeff = 11.1f/12.6f;
    if (voltage_correction_coeff > (11.1f/8.8f))  voltage_correction_coeff = 11.1f/8.8f;
//применяем вычисленный коэффициент
//так как тяга пропорциональна квадрату оборотов (по идее)
    for (uint8_t i = 0; i < 4; i++) engine[i] *= sqrtf(voltage_correction_coeff);      
//ограничения на выдаваемое значение по верхнему и нижнему порогу
    for (uint8_t i = 0; i < 4; i++) { if (engine[i] > 1.0f) engine[i] = 1.0f;}        //верхний предел
    for (uint8_t i = 0; i < 4; i++) { if (engine[i] < 0.01f) engine[i] = 0.01f;}      //нижний предел, чтобы моторы не останавливались
  }

  
  void ESC_input_data_filter(void) {                                                                                                                     
    engine_filtered[0] = avg_filter_1d(engines_filter_pool[0], engine[0], LENGTH_OF_ESC_FILTER);
    engine_filtered[1] = avg_filter_1d(engines_filter_pool[1], engine[1], LENGTH_OF_ESC_FILTER);
    engine_filtered[2] = avg_filter_1d(engines_filter_pool[2], engine[2], LENGTH_OF_ESC_FILTER);
    engine_filtered[3] = avg_filter_1d(engines_filter_pool[3], engine[3], LENGTH_OF_ESC_FILTER);
  }

//функция подготовка логов для записи во флеш память
void prepare_logs(void) {
  set_to_log.timestamp = get_time() - start_time;
#ifdef LOGGING_ACCEL_1
  memcpy(set_to_log.accel_1, accel_raw_1, sizeof(set_to_log.accel_1));
#endif

#ifdef LOGGING_ACCEL_2
  memcpy(set_to_log.accel_2, accel_raw_2, sizeof(set_to_log.accel_2));
#endif

#ifdef LOGGING_ACCEL_1_MAX
   set_to_log.accel_1_max = accel_1_total_max;
#endif

#ifdef LOGGING_ACCEL_2_MAX
   set_to_log.accel_2_max = accel_2_total_max;
#endif

#ifdef LOGGING_GYRO_1
  memcpy(set_to_log.gyro_1, gyro_raw_1, sizeof(set_to_log.gyro_1));
#endif

#ifdef LOGGING_GYRO_2
  memcpy(set_to_log.gyro_2, gyro_raw_2, sizeof(set_to_log.gyro_2));
#endif

#ifdef LOGGING_AVG_ACCEL
  for (uint8_t i = 0; i < 3; i++) set_to_log.accel_avg[i] = (int16_t)(accel_avg[i] * IMU_ACCEL_BITS_PER_G); //ср.арифм двух скомпенсированных акселерометров
#endif

#ifdef LOGGING_AVG_GYRO
  for (uint8_t i = 0; i < 3; i++) set_to_log.gyro_avg[i] = (int16_t)(gyro_avg[i]); //ср.арифм двух скомпенсированных гироскопов в градусах в секунду   
#endif   

#ifdef LOGGING_QUATERNION
  memcpy(set_to_log.q, q_current, sizeof(set_to_log.q));
#endif  

#ifdef LOGGING_ANGLES
  set_to_log.angles[0] = pitch;
  set_to_log.angles[1] = roll; 
  set_to_log.angles[2] = yaw;
#endif

#ifdef LOGGING_YAW_SETPOINT
    set_to_log.yaw_sp = yaw_setpoint;
#endif

#ifdef LOGGING_LIDAR_PID
  set_to_log.lidar_pid_error = lidar_alt_hold_pid.error;
  set_to_log.lidar_pid_P_component = lidar_alt_hold_pid.error * lidar_alt_hold_pid.kp;
  set_to_log.lidar_pid_I_component = lidar_alt_hold_pid.integral_error;
  set_to_log.lidar_pid_D_component = lidar_alt_hold_pid.kd * lidar_alt_hold_pid.error_diff_filtered;
#endif

#ifdef LOGGING_LIDAR_ALTITUDE_CM
  set_to_log.lidar_altitude_cm = (uint16_t)lidar_altitude_corrected;
#endif

#ifdef LOGGING_BARO_PID
    set_to_log.baro_pid_error = baro_alt_hold_pid.error;
    set_to_log.baro_pid_P_component = baro_alt_hold_pid.error * baro_alt_hold_pid.kp;
    set_to_log.baro_pid_I_component = baro_alt_hold_pid.integral_error;
    set_to_log.baro_pid_D_component = baro_alt_hold_pid.kd * baro_alt_hold_pid.error_diff_filtered;
#endif

#ifdef LOGGING_BARO_ALTITUDE_CM
  set_to_log.baro_altitude_cm = (uint16_t)baro_altitude_cm;
#endif

#ifdef LOGGING_KALMAN_ALTITUDE_CM
  set_to_log.kalman_altitude_cm = (int16_t)Kalm_vert.h;
#endif

#ifdef LOGGING_KALMAN_VELOCITY
  set_to_log.kalman_velocity_cm = (int16_t)Kalm_vert.v;
#endif

#ifdef LOGGING_ALTITUDE_SETPOINT_CM
  set_to_log.altitude_setpoint_cm = (uint16_t)altitude_setpoint;
#endif

#ifdef LOGGING_VOLTAGE_mV
  set_to_log.voltage_mv = (uint16_t)(INA219_fresh_data[0] * 1000);
#endif

#ifdef LOGGING_CURRENT_cA
  set_to_log.current_ca = (uint16_t)(INA219_fresh_data[1] * 100);
#endif

#ifdef LOGGING_RC_COMMANDS
  set_to_log.throttle_command = throttle;                         //Значение газа либо от пульта либо по altitude_hold
  set_to_log.pitch_command = rc_fresh_data.received_pitch;
  set_to_log.roll_command = rc_fresh_data.received_roll;
  set_to_log.yaw_command = rc_fresh_data.received_yaw;
  set_to_log.mode_command = rc_fresh_data.mode & 0x000F;
#endif

#ifdef LOGGING_ENGINES
  memcpy(set_to_log.engines, engine, sizeof(set_to_log.engines));
#endif

#ifdef LOGGING_ENGINES_FILTERED
  memcpy(set_to_log.engines_filtered, engine_filtered, sizeof(set_to_log.engines_filtered));
#endif

#ifdef LOGGING_STATE_FLAGS
  if (rc_fresh_data.engines_start_flag) set_to_log.state_flags |= 0b00000001;                                 //bit0 - engine start flag
   else set_to_log.state_flags &= 0b11111110;
  if (lidar_altitude_hold_mode_enabled) set_to_log.state_flags |= 0b00000010;                                 //bit1 - lidar altitude hold flag
  else set_to_log.state_flags &= 0b11111101;
  if (baro_altitude_hold_mode_enabled) set_to_log.state_flags |= 0b00000100;                                 //bit2 - baro altitude hold flag
  else set_to_log.state_flags &= 0b11111011;
#endif

#ifdef LOGGING_RSSI_LEVEL
  set_to_log.rssi_level = rc_fresh_data.rssi_level;
#endif

#ifdef LOGGING_CYCLE_TIMES
  set_to_log.avg_cycle_time_us = (uint32_t)avg_cycle_us;
  set_to_log.max_cycle_time_us = max_cycle_us;
#endif

#ifdef LOGGING_FFT
  set_to_log.peak_freq_hz = (uint16_t)fft_peak_frequency;
#endif
}


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
#ifdef USING_TFMINIS_I2C 
  main_to_mavlink_data.altitude_cm = (uint16_t)Kalm_vert.h;
#endif
  main_to_mavlink_data.rssi_level = rc_fresh_data.rssi_level;
  main_to_mavlink_data.armed_status = rc_fresh_data.engines_start_flag;
}
#endif


//*********************************************************НЕПОСРЕДСТВЕННО НАЧИНАЕТСЯ ЗАДАЧА************************************************************************** */
 
//считываем из флеш самого ESP калибровочные коэффициенты IMU и проверяем что они записаны
//инициализируем и открываем NVS хранения калибровочных коэффициентов  
  ESP_ERROR_CHECK(NVS_prepare(&coeff_NVS_handle, "coeff_storage"));

//считываем массив оффсетов гироскопов 1 и 2
  nvs_read_float_array(coeff_NVS_handle, "gyro_1_off", gyro_1_offset, 3);
  nvs_read_float_array(coeff_NVS_handle, "gyro_2_off", gyro_2_offset, 3);

//считываем массивы оффсетов акселерометров 1 и 2
  nvs_read_float_array(coeff_NVS_handle, "accel_1_off", accel_1_bias, 3);
  nvs_read_float_array(coeff_NVS_handle, "accel_2_off", accel_2_bias, 3);

//считываем массивы Ainv акселерометров 1 и 2
  nvs_read_float_array(coeff_NVS_handle, "accel_1_Ai", accel_1_A_inv, 9);
  nvs_read_float_array(coeff_NVS_handle, "accel_2_Ai", accel_2_A_inv, 9);

//инициализируем и открываем NVS хранения flight_time 
  ESP_ERROR_CHECK(NVS_prepare(&flight_time_NVS_handle, "perm_storage"));
  esp_err_t err = nvs_get_u32(flight_time_NVS_handle, "flight_time", &flight_time); 
  switch (err) {
      case ESP_OK:
          ESP_LOGD(TAG_NVS,"flight time = %ld", flight_time);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"flight time не определен, прописываем нулевое значение");
          err = nvs_set_u32(flight_time_NVS_handle, "flight_time", flight_time);
          nvs_commit(flight_time_NVS_handle); 
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
    }
    
  if ((gyro_1_offset[0] || gyro_1_offset[1] || gyro_1_offset[2]) == 0)
  {
    ESP_LOGE(TAG_FLY,"Гироскопы не откалиброваны, запуститесь в сервисном режиме и проведите калибровку гироскопов\n");
    uint16_t LED_error_code = 23;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&LED_error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);}
  }

  if ((accel_1_bias[0] || accel_1_bias[1] || accel_1_bias[2] || accel_2_bias[0] || accel_2_bias[1] || accel_2_bias[2]) == 0)
  {
    ESP_LOGE(TAG_FLY,"Акселерометры не откалиброваны, запуститесь в сервисном режиме и проведите калибровку акселерометров\n");
    uint16_t LED_error_code = 23;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&LED_error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);}
  }
//без этой задержки assertion не проходит, хз почему
  vTaskDelay(100/portTICK_PERIOD_MS);

  start_time = get_time();

//инициализируем фильтр Баттерворта для фильтрации показаний акселерометра
  Butterworth_init(1000.0, VERT_ACC_FILTER_F_CUT, &accel_Btw_filter);

  ESP_LOGI(TAG_FLY,"Создаем и запускаем таймер для алгоритмов AHRS.....");
  create_and_start_timer(AHRS_TIMER_RESOLUTION_HZ, &ahrs_timer);
  ESP_LOGI(TAG_FLY,"Таймер для алгоритмов создан и запущен\n");

  ESP_LOGI(TAG_FLY,"Инициализируем выбранный алгоритм AHRS");
  ahrs_initialize();
  ESP_LOGI(TAG_FLY,"Выбранный алгоритм AHRS инициализирован");

  ESP_LOGI(TAG_FLY,"Реинициализируем модуль управления ESC");
  esc_control_initialize(0);
  ESP_LOGI(TAG_FLY,"Модуль управления ESC реинициализирован");

  ESP_LOGI(TAG_FLY,"Создаем и запускаем таймеры контроля зависания IMU#1, IMU2 и активируем прерывания.....");
  create_and_start_timer(1 * 1000 * 1000, &IMU_1_suspension_timer);
  create_and_start_timer(1 * 1000 * 1000, &IMU_2_suspension_timer);

  ESP_LOGI(TAG_FLY,"Создаем и запускаем таймер контроля зависания основного полетного цикла.....");
  ESP_ERROR_CHECK(create_and_start_timer_with_interrupt(&general_suspension_timer, 1 * 1000 * 1000, SUSPENSION_TIMER_DELAY_SEC * 1000 * 1000,  general_suspension_timer_interrupt_handler)); 
  ESP_LOGI(TAG_FLY,"Таймер контроля зависания основного полетного цикла запущен");
  
  ESP_LOGI(TAG_FLY,"Активируем прерывания от IMU#1, IMU2 и аварийной остановки.....");
//активируем прерывание по входу от IMU1
  сonfigure_pin_for_interrupt(MPU6000_1_INTERRUPT_PIN, GPIO_PULLDOWN_ENABLE, GPIO_INTR_POSEDGE);
//активируем прерывание по входу от IMU2  
  сonfigure_pin_for_interrupt(MPU6000_2_INTERRUPT_PIN, GPIO_PULLDOWN_ENABLE, GPIO_INTR_POSEDGE);
//активируем прерывание по кнопке аварийной остановки на Holibro  
  сonfigure_pin_for_interrupt(A2, GPIO_PULLDOWN_ENABLE, GPIO_INTR_POSEDGE);
//активируем обработчик прерываний
  ESP_ERROR_CHECK(gpio_isr_register(gpio_interrupt_handler, 0, 0, NULL)); 

  ESP_LOGI(TAG_FLY,"К ПОЛЕТУ ГОТОВ!\n");

//здесь начинается основной бесконечный цикл main_flying_cycle
while(1) 
{    
//ждем прихода нотификации от обработчика прерываний      
    IMU_interrupt_status = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//на всякий случай проверяем что не нулевой      
    if (IMU_interrupt_status != 0) 
    {
      gpio_set_level(LED_RED, 1);
      gpio_set_level(LED_BLUE, 1);
      start_CPU_cycles = esp_cpu_get_cycle_count();

      large_counter++;                //увеличиваем глобальный счетчик циклов 

      if (IMU_interrupt_status == 5) //если все в порядке, оба IMU работают - считываем показания обоих
      {
        SPI_read_bytes(MPU6000_1, 0, 0, 8, MPU6000_ACCEL_XOUT_H | SPI_READ_FLAG, 0, sensor_data_1, 14);
        SPI_read_bytes(MPU6000_2, 0, 0, 8, MPU6000_ACCEL_XOUT_H | SPI_READ_FLAG, 0, sensor_data_2, 14);
      }
      
      else if (IMU_interrupt_status == 2)  //если ошибка по IMU2 - считываем первый и копируем его данные во второй
      {
        SPI_read_bytes(MPU6000_1, 0, 0, 8, MPU6000_ACCEL_XOUT_H | SPI_READ_FLAG, 0, sensor_data_1, 14);
        memcpy(sensor_data_2, sensor_data_1, 14);
        set_to_log.error_flags |= 0x01 << 1;                                 //прописываем во флаги что имело место зависание IMU2
        //ESP_LOGE(TAG_FLY, "Ошибка времени IMU2");
      }

      else if (IMU_interrupt_status == 1) //если ошибка по IMU1 - считываем второй и копируем его данные во первый
      {
        SPI_read_bytes(MPU6000_2, 0, 0, 8, MPU6000_ACCEL_XOUT_H | SPI_READ_FLAG, 0, sensor_data_2, 14);
        memcpy(sensor_data_1, sensor_data_2, 14);
        set_to_log.error_flags |= 0x01;                                 //прописываем во флаги что имело место зависание IMU1
        //ESP_LOGE(TAG_FLY, "Ошибка времени IMU1");
      } 
      
      else //если статус не 1,2 или 5 то капец всему, хз что происходит
      {
        set_to_log.error_flags |= 0b0000000000000011;                            //прописываем во флаги ошибка проверки по обоим IMU
        set_to_log.error_flags |= (0x01 << 15);                                  //фиксируем аварийный режим

        prepare_logs();
        vTaskPrioritySet(task_handle_writing_logs_to_flash,10);      //повышаем приоритет задачи записи логов чтобы отобразить в логах текущее состояние
        xQueueSend(W25N01_queue, &p_to_log_structure, 0);   //инициируем внеплановую запись в логи зафиксировать текущее состояние
     
        uint8_t LED_error_code = 31;
        xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&LED_error_code,0,NULL);
        xTaskNotify(task_handle_emergency_mode, (uint32_t)set_to_log.error_flags,eSetValueWithOverwrite);
        while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);}
      }

//проверяем данные с IMU1 на предмет равенства всех элементов или равенства текущего пакета предыдущему с инкрементом счетчика одинаковых пакетов
      test_for_equal_prev = arrays_are_equal(sensor_data_1, sensor_data_1_old, 14);
      if (test_for_equal_prev == 1) counter_for_equal_prev_IMU1++;
      else counter_for_equal_prev_IMU1 = 0;
      
      test_for_all_equal = all_array_elements_are_equal(sensor_data_1, 14);

//если более 10 пакетов подряд равны между собой или весь пакет одинаковый
      if ((counter_for_equal_prev_IMU1 > 10) || (test_for_all_equal == 1)) 
      {
        IMU_1_data_fail = 1;
//детализируем проблему для логов 
        if (counter_for_equal_prev_IMU1 > 10) set_to_log.error_flags |= 0x01 << 2;
        if (test_for_all_equal == 1) set_to_log.error_flags |= 0x01 << 3;  
      }    
      else IMU_1_data_fail = 0;

//проверяем данные с IMU2 на предмет равенства всех элементов или равенства текущего пакета предыдущему с инкрементом счетчика одинаковых пакетов
      test_for_equal_prev = arrays_are_equal (sensor_data_2, sensor_data_2_old, 14);
      if (test_for_equal_prev == 1) counter_for_equal_prev_IMU2++;
      else counter_for_equal_prev_IMU2 = 0;
      
      test_for_all_equal = all_array_elements_are_equal(sensor_data_2, 14);
//если более 10 пакетов подряд равны между собой или весь пакет одинаковый             
      if ((counter_for_equal_prev_IMU2 > 10) || (test_for_all_equal == 1)) 
      {
        IMU_2_data_fail = 1;
//детализируем проблему для логов 
        if (counter_for_equal_prev_IMU2 > 10) set_to_log.error_flags |= 0x01 << 4;
        if (test_for_all_equal == 1) set_to_log.error_flags |= 0x01 << 5;
      }    
      else IMU_2_data_fail = 0;

//осуществляем корректирующие действия
//если оба не прошли проверку то фиксируем это во флагах и выключаем моторы
      if (IMU_1_data_fail && IMU_2_data_fail)  
      {
        ESP_LOGE(TAG_FLY,"Ошибка обоих IMU");
        set_to_log.error_flags |= (0x01 << 15);  
        prepare_logs();
        vTaskPrioritySet(task_handle_writing_logs_to_flash,10);      //повышаем приоритет задачи записи логов чтобы отобразить в логах текущее состояние
        xQueueSend(W25N01_queue, &p_to_log_structure, 0);            //инициируем внеплановую запись в логи зафиксировать текущее состояние
        uint8_t LED_error_code = 30;
        xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&LED_error_code,0,NULL);
        xTaskNotify(task_handle_emergency_mode, (uint32_t)set_to_log.error_flags,eSetValueWithOverwrite);
        while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
      }
//если не прошели проверку данные с  IMU1
      else if (IMU_1_data_fail) 
      {
//заменяем их данными с IMU2
        ESP_LOGE(TAG_FLY,"Ошибка по IMU1");
        for (uint8_t i = 0; i < 14; i++) sensor_data_1[i] = sensor_data_2[i];
      }
//если не прошели проверку данные с  IMU2
      else if (IMU_2_data_fail) 
      {
//заменяем их данными с IMU1
        ESP_LOGE(TAG_FLY,"Ошибка по IMU2");
        for (uint8_t i = 0; i < 14; i++) sensor_data_2[i] = sensor_data_1[i];
      }
//сохраняем текущие данные в переменные для хранения старых данных
      for (uint8_t i = 0; i < 14; i++)
      {
        sensor_data_1_old[i] = sensor_data_1[i];
        sensor_data_2_old[i] = sensor_data_2[i];
      }

//формируем "сырые" показания акселерометров и гироскопов
      accel_raw_1[0] = (float)((int16_t)((sensor_data_1[0] << 8) | sensor_data_1[1]));           //X
      accel_raw_1[1] = (float)((int16_t)((sensor_data_1[2] << 8) | sensor_data_1[3]));           //Y
      accel_raw_1[2] = (float)((int16_t)((sensor_data_1[4] << 8) | sensor_data_1[5]));           //Z
      gyro_raw_1[0] = (float)((int16_t)((sensor_data_1[8] << 8) | sensor_data_1[9]));            //X                  
      gyro_raw_1[1] = (float)((int16_t)((sensor_data_1[10] << 8) | sensor_data_1[11]));          //Y
      gyro_raw_1[2] = (float)((int16_t)((sensor_data_1[12] << 8) | sensor_data_1[13]));          //Z
      
      accel_raw_2[0] = (float)((int16_t)((sensor_data_2[0] << 8) | sensor_data_2[1]));           //X
      accel_raw_2[1] = (float)((int16_t)((sensor_data_2[2] << 8) | sensor_data_2[3]));           //Y
      accel_raw_2[2] = (float)((int16_t)((sensor_data_2[4] << 8) | sensor_data_2[5]));           //Z
      gyro_raw_2[0] = (float)((int16_t)((sensor_data_2[8] << 8) | sensor_data_2[9]));            //X                  
      gyro_raw_2[1] = (float)((int16_t)((sensor_data_2[10] << 8) | sensor_data_2[11]));          //Y
      gyro_raw_2[2] = (float)((int16_t)((sensor_data_2[12] << 8) | sensor_data_2[13]));          //Z

//применяем калибровочные корректировки к показаниям гироскопов и акселерометров
//и одновременно переводим эти показания в физические величины (в G для акселерометров или градусы в секунду для гироскопов)
//сначала умножаем однократно матрицу Ainv на чувствительность 
      mul_by_scalar_3x3_matrix(accel_1_A_inv, 1.0f / (float)IMU_ACCEL_BITS_PER_G, calibration_temp);
//затем убираем offset и одновременно приеняем Ainv и чувствительность       
      vector_3D_calibration(accel_raw_1, accel_1_bias, calibration_temp, accel_1_converted);
//то же самое для акселерометра 2       
      mul_by_scalar_3x3_matrix(accel_2_A_inv, 1.0f / (float)IMU_ACCEL_BITS_PER_G, calibration_temp);
      vector_3D_calibration(accel_raw_2, accel_2_bias, calibration_temp, accel_2_converted);

//проделывам то же самое с гироскопами (результат в градусах в секунду)
//так как Ainv матрицы единичные для обоих гироскопов операция mul_by_scalar_3x3_matrix одна
      mul_by_scalar_3x3_matrix(gyro_1_A_inv, (1.0f / (float)IMU_GYRO_BITS_PER_DPS), calibration_temp);
      vector_3D_calibration(gyro_raw_1, gyro_1_offset, calibration_temp, gyro_1_converted);
      vector_3D_calibration(gyro_raw_2, gyro_2_offset, calibration_temp, gyro_2_converted);  

//пока не готово - по результатам FFT рассчитываем динамические коэффициенты фильтра НЧ 
#ifdef USING_FFT
    dsps_biquad_f32_ae32(gyro_2_converted, gyro_2_converted_filtered, 3, notch_coeffs, gyro_x_w);
#endif

//если используем магнетометр считываем данные из очереди (если они пришли) и сразу отправляем запрос на подготовку новой партии
//указатель ниже сбрасываем в NULL каждый прогон и ставим его равным указател на данные магнетометра только если действительно пришли актуальные данные
//это позаоляет без лишних #ifdef вызывать унифицированный ahrs_update который сам разбирается запускаться с данными магнетометра или без
float* pointer_to_fresh_mag_data = NULL;
#ifdef USING_MAGNETOMETER
      if (xQueueReceive(magnetometer_queue, mag_fresh_data, 0) == pdPASS)
      {
        pointer_to_fresh_mag_data = mag_fresh_data;
        xTaskNotifyGive(task_handle_mag_read_and_process_data);
      }       
#endif

//находим усредненные показания акселерометров и гироскопов с учетом их размещения на плате в системе NED
//гироскопы в градусах в секунду
      accel_avg[0] = (accel_1_converted[1] - accel_2_converted[0]) * 0.5f;
      accel_avg[1] = (accel_1_converted[0] + accel_2_converted[1]) * 0.5f;
      accel_avg[2] = (accel_1_converted[2] + accel_2_converted[2]) * (-0.5f);

      gyro_avg[0] = (gyro_1_converted[1] - gyro_2_converted[0]) * 0.5f;
      gyro_avg[1] = (gyro_1_converted[0] + gyro_2_converted[1]) * 0.5f;
      gyro_avg[2] = (gyro_1_converted[2] + gyro_2_converted[2]) * (-0.5f);

//вычисляем время с прошлой итерации
      ESP_ERROR_CHECK(gptimer_get_raw_count(ahrs_timer, &timer_value));
      ESP_ERROR_CHECK(gptimer_set_raw_count(ahrs_timer, 0));
      float deltaTime = (float)timer_value / (float)AHRS_TIMER_RESOLUTION_HZ;

//подаем все собратные данные в алгоритм AHRS (Маджвик или VQF в зависимости от выбранного #define'ом)
//в зависимости от указателя на данные магнетометра (NULL или нет) ahrs_update сам выбирает запускать алгоритм с магнетометром или без
//алгоритм с магнетометром запускается только тогда, когда данные от магнетометра реально получены
//гироскопы подаются в градусах в секунду, конвертируются по необходимости в радианы внутри функции
      ahrs_update(accel_avg, gyro_avg, pointer_to_fresh_mag_data, deltaTime, q_current);

//преобразуем кватернион в углы
      Convert_Q_to_degrees_fast(q_current[0], q_current[1], q_current[2], q_current[3], &pitch, &roll, &yaw);
//далее идут действия по вычислению вектора ускорения в вертикальной плоскости, 
//для чего разворачиваем векторы Z акселерометров имеющимся кватернионом текущей ориентации
//копируем текущий кватернион
      q_current[0] = -q_current[0];     //хз почему минус, по-другому не работает, возможно из-за направления оси

//составляем 4-х компонентный кватернион из вектора гравитации 1 в G в системе NED в соответствие с раcположением IMU 
      accel_1_full[0] = 0;
      accel_1_full[1] = -accel_1_converted[1];
      accel_1_full[2] = -accel_1_converted[0];
      accel_1_full[3] = accel_1_converted[2];
//разворачиваем показания акселерометра 1
      vector_back_rotation(accel_1_full, q_current, accel_1_rotated);
//составляем 4-х компонентный кватернион из вектора гравитации 2 в G в системе NED в соответствие с раcположением IMU 
      accel_2_full[0] = 0;
      accel_2_full[1] = accel_2_converted[0];
      accel_2_full[2] = -accel_2_converted[1];
      accel_2_full[3] = accel_2_converted[2];
//разворачиваем показания акселерометра 2
      vector_back_rotation(accel_2_full, q_current, accel_2_rotated);
//вычисляем средний между 2х акселерометров результат и вычитаем G
//вектор ускорения с минус, так как исходный в NED, то есть вниз положительное
      vertical_acceleration = ((accel_1_rotated[3] + accel_2_rotated[3]) / 2.0f) - 1.0;

//фильтруем показания вертикального ускорения ФНЧ 
      accel_z_ave_rotated_filtered = Butterworth_filter(vertical_acceleration, &accel_Btw_filter);
//        accel_z_ave_rotated_filtered -= 0.00737; //ручная докалибровка

//компенсация магнитного склонения
      yaw -= 9.4;
//применяем к углам триммирующие поправки, получаемые от пульта управления(величина меняется от -7 до 7, коэффициент регулирует чувствительность)
//если, например, коэффициент 0.5, то диапазон регулировки от -3.5 до +3.5 градусов
      pitch += rc_fresh_data.trim_pitch * 0.5;
      roll -= rc_fresh_data.trim_roll * 0.5;
        
//производим запрос данных от INA219 (раз в X циклов, то есть (1000/X) в секунду)
//по результатам считывания корректируем уровень газа, поэтому очень медленно нельзя 
      if ((large_counter % 25) == 0) xTaskNotifyGive(task_handle_INA219_read_and_process_data);      

//даем команду считать показания лидара
#ifdef USING_TFMINIS_I2C       
      if (((large_counter + 5) % 25) == 0) xTaskNotifyGive(task_handle_lidar_read_and_process_data);
#endif

//даем команду считать показания барометра
#ifdef USING_MS5611
      if (((large_counter + 23) % 25) == 0) xTaskNotifyGive(task_handle_MS5611_read_and_process_data); 
#endif

//получаем свежие данные из очереди от пульта управления. Если данные успешно получены 
//- информируем задачу моргания полетными огнями что моргаем в штатном режиме (режим "1")
// или в режиме "низкий заряд аккумулятора" (режим "2")
// или в режиме "совсем низкий заряд аккумулятора" (режим "3")
    
      if (xQueueReceive(remote_control_to_main_queue, &rc_fresh_data, 0)) 
      {
        remote_control_lost_comm_counter = 0;
        if (INA219_fresh_data[0] < 10.3) xTaskNotify(task_handle_blinking_flight_lights,2,eSetValueWithOverwrite);
        else if (INA219_fresh_data[0] < 9.5) xTaskNotify(task_handle_blinking_flight_lights,3,eSetValueWithOverwrite);
        else xTaskNotify(task_handle_blinking_flight_lights,1,eSetValueWithOverwrite);
      }
//в противном случае инкрементируем счетчик, определяющий допустимое время без связи с пультом     
      else remote_control_lost_comm_counter++;
//если счетчик превысил порог когда были уже в полете 
//фиксируем уровень газа на некоем предустановленном значении и все управляющие сигналы в ноль (то есть в идеале висение на месте) 
      if (remote_control_lost_comm_counter > RC_NO_COMM_DELAY_MAIN_CYCLES)  
      {
        if (rc_fresh_data.engines_start_flag) {
          remote_control_lost_comm_counter = RC_NO_COMM_DELAY_MAIN_CYCLES;
          throttle = RC_NO_COMM_THROTTLE_HOVER_VALUE;
          rc_fresh_data.received_pitch = 0;
          rc_fresh_data.received_roll = 0;
          rc_fresh_data.received_yaw = 0;
          rc_fresh_data.lidar_altitude_hold_flag = 0;     //выходим из режима удержания высоты по лидару если был включен
          rc_fresh_data.baro_altitude_hold_flag = 0;      //выходим из режима удержания высоты по барометру если был включен
          rc_fresh_data.rssi_level = -127;                //чтобы в логах было четко видно когда теряется связь
          set_to_log.error_flags |= 0x01 << 7;            //сохраняем ошибку в логи
        }
//и оповещаем задачу моргания полетными огнями моргать в аварийном режиме (режим "0")
        xTaskNotify(task_handle_blinking_flight_lights,0,eSetValueWithOverwrite);  
      }
//получаем свежие данные из очереди от GPS
#ifdef USING_GPS
      if (xQueueReceive(gps_to_main_queue, &gps_fresh_data, 0)) 
      {
        //ESP_LOGI(TAG_FLY,"Широта %ld Долгота %ld", gps_fresh_data.latitude_d, gps_fresh_data.longtitude_d);
      } 
#endif

//получаем свежие данные из очереди от px4flow
#ifdef USING_PX4FLOW
      if (xQueueReceive(px4flow_to_main_queue, &px4flow_fresh_data, 0)) 
        {
          //if (large_counter % 100 == 0) printf("%0.2f, %0.2f, %d\n",px4flow_fresh_data.optical_x, px4flow_fresh_data.optical_y, px4flow_fresh_data.quality);
        }
#endif

//получаем свежие данные из очереди от INA219
      if (xQueueReceive(INA219_to_main_queue, &INA219_fresh_data, 0))
      {
        //SP_LOGE(TAG_FLY, "V: %0.4fV",INA219_fresh_data[1]);
      }

      //получаем свежие данные из очереди от лидара
#ifdef USING_TFMINIS_I2C         
      if (xQueueReceive(lidar_to_main_queue, &lidar_fresh_data, 0)) 
      {
        new_lidar_data_arrived_flag = 1;
        //ESP_LOGI(TAG_FLY,"высота %0.3f", lidar_fresh_data.altitude); // высота в cm
//корректируем показания лидара с учетом текущего наклона по pitch и roll 
        if (lidar_fresh_data.altitude < 1200) lidar_altitude_corrected = lidar_fresh_data.altitude * cosf(pitch * M_PI / 180.0f) * cosf(roll * M_PI / 180.0f);
//передаем актуальную высоту в задачу обработки данных от PX4Flow
#ifdef USING_PX4FLOW 
        xTaskNotify(task_handle_px4flow_read_and_process_data, (uint32_t)lidar_altitude_corrected,eSetValueWithOverwrite);
#endif
      }
#endif

//на основе данных от акселерометра (развернутый отфильтрованный усредненный вектор гравитации) и барометра вычисляем фильтром Калмана текущую высоту
#ifdef USING_MS5611
      if (large_counter > 8000) //задержка так как до 8000 считается урседненное давление в точке старта
      {
        Kalman_2d_predict((accel_z_ave_rotated_filtered) * 981, &Kalm_vert);  //в сантиметрах
        if (xQueueReceive(MS5611_to_main_queue, &baro_altitude_cm, 0)) 
        {
          //printf("%0.1f, ",baro_altitude_cm); 
          new_baro_data_arrived_flag = 1;
          Kalman_2d_update(baro_altitude_cm, &Kalm_vert);
          if (Kalm_vert.h < 0) Kalm_vert.h = 0;
          //printf("%d, %d\n", (uint16_t)Kalm_vert.h, (uint16_t)lidar_altitude_corrected);
          //printf("%0.5f\n", accel_z_ave_rotated_filtered); // баро высота в cm 
        }        
      }
#endif

//получаем из очереди от пульта новые ПИД коэффициенты если они пришли
      if (xQueueReceive(remote_control_to_main_pid_queue, &pid_fresh_data, 0)) 
      {
        //printf("получ Kp: %d, Ki: %d, Kd %d\n", pid_fresh_data.kp_alt_hold_coeff, pid_fresh_data.ki_alt_hold_coeff, pid_fresh_data.kd_alt_hold_coeff);
//копируем их в локальные (избыточный временный шаг)          
        lidar_alt_hold_pid.kp = (float)pid_fresh_data.kp_alt_hold_coeff / 10.0;
        lidar_alt_hold_pid.ki = (float)pid_fresh_data.ki_alt_hold_coeff / 1000.0;
        lidar_alt_hold_pid.kd = (float)pid_fresh_data.kd_alt_hold_coeff;
        //printf("обраб Kp: %0.3f, Ki: %0.3f, Kd %0.3f\n", lidar_alt_hold_pid.kp, lidar_alt_hold_pid.ki, lidar_alt_hold_pid.kd);           
      }

//в этой точке собрано текущее состояние и есть данные от пульта, диктующие желаемое положение и режим полета. Можно переходить к вычислению управляющих воздействий
//разбираемся с полетным режимом
// если двигатели запущены - понимаем стоит ли режим удержания высоты. Если да - замещаем полученное от пульта значение газа вычисленным автоматически на основание данных от лидара или барометра
      if (rc_fresh_data.engines_start_flag)
      {
        throttle = rc_fresh_data.received_throttle; //если двигатели запущены устанавливаем уровень газа в соответствие с полученным от пульта
//если активен режим удержания высоты по барометру или лидару - начинаем танцы с бубном, замещая уровень газа от пульта вычисленным по показаниям барометра или лидара                
      if (rc_fresh_data.lidar_altitude_hold_flag || rc_fresh_data.baro_altitude_hold_flag)
        {
#ifdef USING_MS5611
//если находимся в режиме удержания высоты по барометру            
            if (rc_fresh_data.baro_altitude_hold_flag)
            {
//если только включили режим, первый раз в цикл вошли
              if (baro_altitude_hold_mode_enabled == 0)                                      
              {
//зафиксировали что режим включен
                baro_altitude_hold_mode_enabled = 1;
//запоминаем высоту 
                altitude_setpoint = Kalm_vert.h;
//запоминаем уровень газа (должен быть около висения в момент когда активируем режим удержания) 
                alt_hold_initial_throttle = rc_fresh_data.received_throttle; 
              }
//если есть свежие данные от барометра при включенном режиме работы по барометру то считаем уровень газа 1-контурным ПИДом, сравнивая желаемую высоту setpoint и текущую Kalm_vert.h
              if (new_baro_data_arrived_flag)                                                     
              {                                                                            
                new_baro_data_arrived_flag = 0;
                pid_altitude = PID_Compute(&baro_alt_hold_pid, altitude_setpoint, Kalm_vert.h, 0);
              }
            }
#endif 

#ifdef USING_TFMINIS_I2C                 
//если находимся в режиме удержания высоты по лидару           
            if (rc_fresh_data.lidar_altitude_hold_flag)
            {
//если только включили режим, первый раз в цикл вошли
              if (lidar_altitude_hold_mode_enabled == 0)                                      
              {
//зафиксировали что режим включен
                lidar_altitude_hold_mode_enabled = 1;
//запоминаем высоту 
                altitude_setpoint = lidar_altitude_corrected;
//запоминаем уровень газа (должен быть около висения в момент когда активируем режим удержания) 
                alt_hold_initial_throttle = rc_fresh_data.received_throttle; 
              }
//если есть свежие данные от лидара при включенном режиме работы по лидару то считаем уровень газа 1-контурным ПИДом, сравнивая желаемую высоту setpoint и текущую lidar_altitude_corrected
              if (new_lidar_data_arrived_flag)                                                     
              {                                                                            
                new_lidar_data_arrived_flag = 0;
                pid_altitude = PID_Compute(&lidar_alt_hold_pid, altitude_setpoint, lidar_altitude_corrected, 0);
              }
            }
#endif   
//корректируем точку установки высоты (в эту точку попадаем каждый цикл 1мс без привязки к приходу новых данных от датчиков)
//если ручка газа вверху то увеличиваем установку высоты, если внизу - уменьшаем с с линейной зависимостью от положения ручки газа
//коэффициенты выбраны исходя из того, что макс скорость изменения высоты = 500см/сек, то есть 0,5см за 1мс
            if (rc_fresh_data.raw_throttle > 2448) altitude_setpoint += 0.00018204 * rc_fresh_data.raw_throttle - 0.4456; 
            if (rc_fresh_data.raw_throttle < 1648) altitude_setpoint += 0.00018204 * rc_fresh_data.raw_throttle - 0.3;
//задаем пределы установки высоты (числа в сантиметрах)
            if (altitude_setpoint >= 3000) altitude_setpoint = 3000;
            if (altitude_setpoint <= 1) altitude_setpoint = 1;

//заменяем значение throttle от пульта вычисленным при помощи регулятора
          throttle = alt_hold_initial_throttle + pid_altitude;             
          }

//если режим удержания высоты выключаем (или он постоянно выключен) то обнуляем интегральные составляющие ПИД регуляторов и флаги, чтобы они не выстрелили при следующем запуске
      else 
        {
          lidar_altitude_hold_mode_enabled = 0; 
          lidar_alt_hold_pid.prev_error = 0;
          lidar_alt_hold_pid.integral_error = 0;
          lidar_alt_hold_pid.error_diff_filtered = 0;
          baro_altitude_hold_mode_enabled = 0;
          baro_alt_hold_pid.prev_error = 0;
          baro_alt_hold_pid.integral_error = 0;
          baro_alt_hold_pid.error_diff_filtered = 0;
          altitude_setpoint = 0;
        }

//на основании текущих данных по угловой скорости, углам и желаемым углам от пульта считаем двухконтурным ПИД-регулятором управляющие воздействия на двигатели по углам
      calculate_pids_2();
//раз в секунду увеличиваем счетчик "времени в воздухе"
      if ((large_counter % 1000) == 0) flight_time++;
      current_state = FLIGHT_STATE;
      }
//если от пульта команда что двигатели выключены - сбрасываем интегральные составляющие ПИД, глушим моторы и выключаем режимы удержания
      else 
      {
        yaw_setpoint = yaw;
        pitch_pid_angle.integral_error = roll_pid_angle.integral_error = 0;
        pitch_pid_angle.prev_error = roll_pid_angle.prev_error = 0;
        pitch_pid_rate.integral_error = roll_pid_rate.integral_error = yaw_pid_rate.integral_error = 0;
        pitch_pid_rate.prev_error = roll_pid_rate.prev_error = yaw_pid_rate.prev_error = 0;
        integral_yaw_error_angle = error_yaw_angle_old = 0;
        engine[0] = engine[1] = engine[2] = engine[3] = 0;
        lidar_altitude_hold_mode_enabled = 0;
        baro_altitude_hold_mode_enabled = 0;
//при переходе от состояния с включенным двигателем в выключенное записываем во флеш счетчик времени в полете
        if (current_state == FLIGHT_STATE)
        {
          nvs_set_u32(flight_time_NVS_handle, "flight_time", flight_time);
          nvs_commit(flight_time_NVS_handle);
          current_state = GROUND_STATE;
        }           
      }
//фильтруем вычисленные ПИД-регуляторами данные        
      ESC_input_data_filter();
//отправляем данные на двигатели      
      esc_control_update(engine_filtered);
//на этом цикл от считывания данных от IMU до выдачи сигналов на двигатели фактически закончен
//дальше можно заняться опциональными вещами
//собираем данные телеметрии и отправляем их в очередь на отправку на пульт
/*
      data_to_send_to_rc.pitch = pitch;
      data_to_send_to_rc.roll = roll;
      data_to_send_to_rc.yaw = yaw;
      data_to_send_to_rc.power_voltage_value = (uint16_t)(INA219_fresh_data[0]*10.0); //в сантивольтах
      data_to_send_to_rc.altitude = (uint16_t)baro_altitude_cm / 100;     //переводим из см в метры
      xQueueOverwrite(main_to_rc_queue, (void *) &data_to_send_to_rc);         
*/
//подготавливаем данные для записи в логи и записываем по flash раз в 50 циклов
if (rc_fresh_data.engines_start_flag)
{
      if ((large_counter % 50) == 0) 
      {       
        prepare_logs();
        xQueueSend(W25N01_queue, &p_to_log_structure, 0);
      }
    }        
//2 раза в секунду отправляем данные по UART на minimOSD
#ifdef USING_MAVLINK_TELEMETRY
      if ((large_counter % 500) == 0) 
      {       
        prepare_mavlink_data();
        xQueueSend(main_to_mavlink_queue, &p_to_mavlink_data, 0);
      }
#endif
//опционально раз в 5с выводим данные по потреблению стэка этой задачей
#ifdef MEMORY_CONSUMPTION_MESUREMENT
      if ((large_counter % 5000) == 0) 
      {
        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        ESP_LOGW(TAG_FLY,"High watermark %d",  uxHighWaterMark);
      }       
#endif

//ниже блок, выполняющий проверку обших ускорений и углов на предмет фиксации аварийной ситуации
//если текущние показания превышают установленный предел заданное кол-во раз подряд - значит или перевернулись, или упали, или столкнулись с чем-то в воздухе
//по фиксации превышений аварийно останавливаются моторы 
 #ifdef ACCEL_AND_ANGLES_SAFETY_MEASURES
      if (current_state == FLIGHT_STATE)
      {
//находим полные вектора ускорений (в g)
        accel_1_total = sqrtf(accel_1_converted[0] * accel_1_converted[0] + accel_1_converted[1] * accel_1_converted[1]  + accel_1_converted[2] * accel_1_converted[2]);
        accel_2_total = sqrtf(accel_2_converted[0] * accel_2_converted[0] + accel_2_converted[1] * accel_2_converted[1]  + accel_2_converted[2] * accel_2_converted[2]);

        //сравниваем с заданным порогом по очереди для обоих акселерометров
        if (accel_1_total > accel_1_total_max) accel_1_total_max = accel_1_total; //наблюдаем максиальный уровень для логирования
        if (accel_1_total > MAX_ACCEL_LIMIT)        //если общее ускорение превышает заданный порог 
        {
          accel_1_exceed_limit_counter++;           //увеличиваем счетчик превышений
          set_to_log.error_flags |= (0x01 << 8);    //сохраняем ошибку в логи
        } 
        else accel_1_exceed_limit_counter = 0; 
            
        if (accel_2_total > accel_2_total_max) accel_2_total_max = accel_2_total; //наблюдаем максиальный уровень для логирования
        if (accel_2_total > MAX_ACCEL_LIMIT) 
        {
          accel_2_exceed_limit_counter++;           
          set_to_log.error_flags |= (0x01 << 9);    
        } 
        else accel_2_exceed_limit_counter = 0;

//проверяем углы на превышение максимально допустимых значений. 
//По превышению заносим во флаг и считаем кол-во превышений подряд. Если превышает лимит - фиксируем аварию
        if (fabsf(pitch) >= MAX_ANGLES_LIMIT)            //проверяем pitch 
        {
          set_to_log.error_flags |= (0x01 << 10);        //сохраняем ошибку в логи
          pitch_exceed_limit_counter++;
        }
        else pitch_exceed_limit_counter = 0;

        if (fabsf(roll) >= MAX_ANGLES_LIMIT)             //проверяем roll
        {
          set_to_log.error_flags |= (0x01 << 11);        //сохраняем ошибку в логи
          roll_exceed_limit_counter++;
        }
        else roll_exceed_limit_counter = 0;
//если хоть один из счетчиков превысил порог "срабатываний подряд"
        if ((accel_1_exceed_limit_counter > MAX_ACCEL_EXCEED_LIMIT_COUNTER) || 
            (accel_2_exceed_limit_counter > MAX_ACCEL_EXCEED_LIMIT_COUNTER) ||
            (pitch_exceed_limit_counter > MAX_ANGLE_EXCEED_LIMIT_COUNTER) ||
            (roll_exceed_limit_counter > MAX_ANGLE_EXCEED_LIMIT_COUNTER))    
        {
          set_to_log.error_flags |= (0x01 << 15);                     //фиксируем аварию в логи
          prepare_logs();
          vTaskPrioritySet(task_handle_writing_logs_to_flash,10);      //повышаем приоритет задачи записи логов чтобы отобразить в логах текущее состояние
          xQueueSend(W25N01_queue, &p_to_log_structure, 0);           //инициируем внеплановую запись в логи зафиксировать текущее состояние
          uint8_t LED_error_code = 32;
          xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&LED_error_code,0,NULL);
          xTaskNotify(task_handle_emergency_mode, (uint32_t)set_to_log.error_flags,eSetValueWithOverwrite);
        }
      }  
#endif

        IMU_interrupt_status = 0;

#ifdef LOGGING_CYCLE_TIMES
//вычисляем количество циклов CPU и среднее значение для логгирования
//в данном случае esp_rom_get_cpu_ticks_per_us() вернет 240
        current_cycle_us = (esp_cpu_get_cycle_count() - start_CPU_cycles) / esp_rom_get_cpu_ticks_per_us();
        avg_cycle_us = avg_cycle_us + ((float)current_cycle_us - avg_cycle_us) / (float)large_counter;
        if (current_cycle_us > max_cycle_us) max_cycle_us = current_cycle_us;
#endif

//собираем в кольцевой буфер 128 сэмплов и оповещаем задачу FFT что можно забирать и обрабатывать эти данные
#ifdef USING_FFT
//отправляем в буфер свежий сэмпл выбранной оси какого-то гироскопа или акселерометра (какую ось выбирать?)
          gyro_samples_for_fft[samples_since_last_fft++] = accel_1_converted[0];
//если навколено нужное кол-во - отправляем в кольцевой буфер
          if (samples_since_last_fft >= FFT_HOP_SIZE)
          {
            if (xRingbufferSend(fft_rb_handle, gyro_samples_for_fft, sizeof(float) * FFT_HOP_SIZE, 0) != pdTRUE) 
            {
              ESP_LOGI(TAG_FLY, "Буфер FFT заполнен, сэмпл игнорируется!"); 
            }
            samples_since_last_fft = 0;
            xTaskNotifyGive(task_handle_fft);
          }
//получаем обратно от FFT вычисленную пиковую частоту          
          if (xQueueReceive(fft_to_main_queue, &fft_peak_frequency, 0) == pdPASS)
          {
            //printf("%f\n",fft_peak_frequency);
            dsps_biquad_gen_notch_f32(notch_coeffs, fft_peak_frequency / (float)IMU_SAMPLING_FREQ_HZ, 1.0, notch_bandwith / (float)IMU_SAMPLING_FREQ_HZ);
          }
#endif

//сбрасываем таймер общего зависания, по сработке которого аварийно останавливаем двигатели и входим в emergency режим       
        ESP_ERROR_CHECK(gptimer_set_raw_count(general_suspension_timer, 0));
        gpio_set_level(LED_RED, 0);
        gpio_set_level(LED_BLUE, 0);
        
//if (large_counter > 2000) {while(1) {vTaskDelay(10);}}

//далее удобно что-то выводить, печатать 
//if ((large_counter % 1000) == 0) ESP_LOGI(TAG_FLY,"Trim pitch %d, trim roll %d", rc_fresh_data.trim_pitch, rc_fresh_data.trim_roll);
//if ((large_counter % 1000) == 0) ESP_LOGI(TAG_FLY,"Trim pitch %d, trim roll %d", rc_fresh_data.trim_pitch, rc_fresh_data.trim_roll);
//if ((large_counter % 1000) == 0) ESP_LOGI(TAG_FLY,"new are %0.2f, %0.2f, %0.2f, %0.2f", engine_filtered_new[0],engine_filtered_new[1],engine_filtered_new[2],engine_filtered_new[3]);        
//if ((large_counter % 100) == 0) printf("%0.2f, %0.2f, %0.2f, %0.2f\n", engine[0], engine[1], engine[2], engine[3]);
//if ((large_counter % 50) == 0) printf("%0.2f, %0.2f, %0.2f\n", bias.gyroscopeOffset.axis.x, bias.gyroscopeOffset.axis.y, bias.gyroscopeOffset.axis.z);
//if ((large_counter % 100) == 0) printf("%0.2f, %0.2f, %0.2f\n", pitch, roll, yaw);
//if ((large_counter % 100) == 0) printf("%0.2f, %0.2f, %0.2f, %0.2f\n", q_current[0], q_current[1], q_current[2], q_current[3]);
//if ((large_counter % 20) == 0) printf("%0.2f, %0.2f, %0.2f\n", accelerometer_rotated.axis.x, accelerometer_rotated.axis.y, accelerometer_rotated.axis.z);
//if ((large_counter % 50) == 0) printf("%0.4f\n", vertical_acceleration); // баро высота в cm
//if (large_counter<1000) printf("%lu, %0.2f, %lu\n", current_cycle_us, avg_cycle_us, max_cycle_us);
//if ((large_counter % 1000) == 0) printf("%lu, %lu, %lu\n", current_cycle_us,(uint32_t)avg_cycle_us, max_cycle_us);

//if ((large_counter % 1000) == 0) printf("%0.2f, %lu, %d\n", engine[0], (uint32_t)engine_filtered[0], dshot_signal[0].throttle);

    } 
  }
}
