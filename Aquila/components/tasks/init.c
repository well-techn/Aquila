
//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_log.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "wt_i2c.h"
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
#include "mavlink_wt.h"
#include "mavlink_crc.h"
#include "MS5611_read_and_process.h"
#include "error_code_LED_blinking.h"
#include "mag_read_and_process_data.h"
#include "gps_read_and_process_data.h"
#include "RC_read_and_process_data.h"
#include "send_data_to_RC.h"
#include "lidar_read_and_process_data.h"
#include "MCP23017_monitoring_and_control.h"
#include "PCA9685_control.h"
#include "writing_logs_to_flash.h"
#include "blinking_flight_lights.h"
#include "performance_monitor.h"
#include "INA219_read_and_process_data.h"
#include "send_telemetry_via_mavlink.h"
#include "main_flying_cycle.h"
#include "configuration_mode.h"
#include "configuration_mode_telnet.h"
#include "px4flow.h"

const char *TAG_INIT = "INIT";
const char *TAG_FLY = "FLY";
const char *TAG_GPS = "GPS";
const char *TAG_NVS = "NVS";
const char *TAG_RC = "RC";
const char *TAG_PMW = "PMW";
const char *TAG_W25N = "W25N";
const char *TAG_MPU6000 = "MPU6000";
const char *TAG_MCP23017 = "MCP23017";
const char *TAG_PCA9685 = "PCA9685";
const char *TAG_LIDAR = "LIDAR";
const char *TAG_INA219 = "INA219";
const char *TAG_IST8310 = "IST8310";
const char *TAG_FL3195 = "RGB_LED";
const char *TAG_MS5611 = "MS5611";
const char *TAG_MAV = "MAV";
const char *TAG_PX4FLOW = "PX4FLOW";


//spi handles
extern spi_device_handle_t W25N01;
extern spi_device_handle_t MPU6000_1;
extern spi_device_handle_t MPU6000_2;

//i2c handles
extern i2c_master_bus_handle_t i2c_internal_bus_handle;
extern i2c_master_bus_handle_t i2c_external_bus_handle;
extern i2c_master_dev_handle_t IST8310_dev_handle;
extern i2c_master_dev_handle_t TFSMINI_dev_handle;

//семафоры
SemaphoreHandle_t semaphore_for_i2c_external;
static StaticSemaphore_t semaphore_for_i2c_external_buffer;
SemaphoreHandle_t semaphore_for_i2c_internal;
static StaticSemaphore_t semaphore_for_i2c_internal_buffer;

//таймеры
 gptimer_handle_t GP_timer;
 gptimer_handle_t general_suspension_timer;
 gptimer_handle_t IMU_1_suspension_timer;
 gptimer_handle_t IMU_2_suspension_timer;

//очереди
QueueHandle_t gps_queue_for_events = NULL; //очередь для передачи событий от UART с подключенным GPS (pattern detection) в задачу обработки данных GPS
QueueHandle_t gps_to_main_queue = NULL; //очередь для передачи обработанных GPS данных в main_flying_cycle
QueueHandle_t remote_control_queue_for_events = NULL; //очередь для передачи событий от UART пульта управления (pattern detection) в задачу обработки данных пульта
QueueHandle_t remote_control_to_main_queue = NULL; //очередь для передачи обработанных данных от пульта в main_flying_cycle
QueueHandle_t lidar_queue_for_events = NULL; //очередь для передачи событий от UART с подключенным лидаром (pattern detection) в задачу обработки данных от лидара
QueueHandle_t lidar_to_main_queue = NULL; //очередь для передачи обработанных данных от лидара в main_flying_cycle
QueueHandle_t MCP23017_queue = NULL; //очередь принимающая команды на считывания состояния или управления MCP23017
QueueHandle_t PCA9685_queue = NULL; //очередь принимающая команды управления PCA9685
QueueHandle_t magnetometer_queue = NULL; //очередь для передачи обработанных данных из задачи обработки данных магнетометра в main_flying_cycle
QueueHandle_t main_to_rc_queue = NULL; //очередь для передачи данных из main_flying_cycle в задачу отправки телеметрии на пульт
QueueHandle_t W25N01_queue = NULL;  //очередь для передачи данных из main_flying_cycle в задачу записи логов во внешнюю флэш-память
QueueHandle_t INA219_to_main_queue = NULL; //очередь для передачи данных из задачи обработки данных от INA219 в main_flying_cycle
QueueHandle_t mav_queue_for_events = NULL; //очередь для обработки событий для UART Mavlink
QueueHandle_t main_to_mavlink_queue = NULL; //очередь для передачи данных из main в mavlink
QueueHandle_t MS5611_to_main_queue = NULL; //очередь для передачи данных из задачи считывания MS5611 в main

//параметры выделения памяти для статического размещения задач
StaticTask_t MCP23017_monitoring_and_control_TCB_buffer;
StackType_t MCP23017_monitoring_and_control_stack[MCP23017_MONITORING_AND_CONTROL_STACK_SIZE];

StaticTask_t PCA9685_control_TCB_buffer;
StackType_t PCA9685_control_stack[PCA9685_CONTROL_STACK_SIZE];

StaticTask_t mag_read_and_process_data_TCB_buffer;
StackType_t mag_read_and_process_data_stack[MAG_READ_AND_PROCESS_DATA_STACK_SIZE];

StaticTask_t main_flying_cycle_TCB_buffer;
StackType_t main_flying_cycle_stack[MAIN_FLYING_CYCLE_STACK_SIZE];

StackType_t gps_read_and_process_data_stack[GPS_READ_AND_PROCESS_DATA_STACK_SIZE];
StaticTask_t gps_read_and_process_data_TCB_buffer;

StaticTask_t RC_read_and_process_data_TCB_buffer;
StackType_t RC_read_and_process_data_stack[RC_READ_AND_PROCESS_DATA_STACK_SIZE];

StaticTask_t send_data_to_RC_TCB_buffer;
StackType_t send_data_to_RC_stack[SEND_DATA_TO_RC_STACK_SIZE];

StaticTask_t blinking_flight_lights_TCB_buffer;
StackType_t blinking_flight_lights_stack[BLINKING_FLIGHT_LIGHTS_STACK_SIZE];

StaticTask_t lidar_read_and_process_data_TCB_buffer;
StackType_t lidar_read_and_process_data_stack[LIDAR_READ_AND_PROCESS_DATA_STACK_SIZE];

StaticTask_t INA219_read_and_process_data_TCB_buffer;
StackType_t INA219_read_and_process_data_stack[INA219_READ_AND_PROCESS_DATA_STACK_SIZE];

StaticTask_t writing_logs_to_flash_TCB_buffer;
StackType_t writing_logs_to_flash_stack[WRITING_LOGS_TO_FLASH_STACK_SIZE];

StaticTask_t performance_measurement_TCB_buffer;
StackType_t performance_measurement_stack[PERFORMANCE_MEASUREMENT_STACK_SIZE];

StaticTask_t mavlink_telemetry_TCB_buffer;
StackType_t mavlink_telemetry_stack[MAVLINK_TELEMETRY_STACK_SIZE];

StaticTask_t MS5611_read_and_process_data_TCB_buffer;
StackType_t MS5611_read_and_process_data_stack[MS5611_READ_AND_PROCESS_DATA_STACK_SIZE];

//задачи
TaskHandle_t task_handle_blinking_flight_lights;
TaskHandle_t task_handle_main_flying_cycle;
TaskHandle_t task_handle_send_data_to_RC;
TaskHandle_t task_handle_init;
TaskHandle_t task_handle_performace_measurement;
TaskHandle_t task_handle_INA219_read_and_process_data;
TaskHandle_t task_handle_mag_read_and_process_data;
TaskHandle_t task_handle_PCA9685_control;
TaskHandle_t task_handle_MCP23017_monitoring_and_control;
TaskHandle_t task_handle_writing_logs_to_flash;
TaskHandle_t task_handle_mavlink_telemetry;
TaskHandle_t task_handle_lidar_read_and_process_data;
TaskHandle_t task_handle_MS5611_read_and_process_data;

static void configure_IOs()
{
  gpio_reset_pin(LED_RED);
  gpio_set_direction(LED_RED, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_RED, 1);

  gpio_reset_pin(LED_BLUE);
  gpio_set_direction(LED_BLUE, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_BLUE, 1);

  gpio_reset_pin(LED_GREEN);
  gpio_set_direction(LED_GREEN, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_GREEN, 1);

  gpio_reset_pin(GREEN_FLIGHT_LIGHTS);
  gpio_set_direction(GREEN_FLIGHT_LIGHTS, GPIO_MODE_OUTPUT);
  gpio_set_level(GREEN_FLIGHT_LIGHTS, 0);

  gpio_reset_pin(RED_FLIGHT_LIGHTS);
  gpio_set_direction(RED_FLIGHT_LIGHTS , GPIO_MODE_OUTPUT);
  gpio_set_level(RED_FLIGHT_LIGHTS, 0);

  gpio_reset_pin(GPIO_CS_W25N01);
  gpio_set_direction(GPIO_CS_W25N01, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(GPIO_CS_W25N01, GPIO_PULLUP_ENABLE);
 
  gpio_reset_pin(IMU_SPI_MISO);
  gpio_set_direction(IMU_SPI_MISO, GPIO_MODE_INPUT);
  gpio_set_pull_mode(IMU_SPI_MISO, GPIO_PULLDOWN_ENABLE);

  gpio_reset_pin(IMU_SPI_MOSI);
  gpio_set_direction(IMU_SPI_MOSI, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(IMU_SPI_MOSI, GPIO_PULLUP_ENABLE);

  gpio_reset_pin(GP_SPI_MISO);
  gpio_set_direction(GP_SPI_MISO, GPIO_MODE_INPUT);
  gpio_set_pull_mode(GP_SPI_MISO, GPIO_PULLDOWN_ENABLE);

  gpio_reset_pin(GP_SPI_MOSI);
  gpio_set_direction(GP_SPI_MOSI, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(GP_SPI_MOSI, GPIO_PULLUP_ENABLE);

  gpio_reset_pin(A3);
  gpio_set_direction(A3, GPIO_MODE_OUTPUT);
  gpio_set_level(A3, 1);
    
}

//создание таймера для управления двигателями через LEDC
static void configuring_timer_for_PWM()
{
    ledc_timer_config_t engine_pwm_timer = {
      .speed_mode       = ENGINE_PWM_MODE,
      .timer_num        = ENGINE_PWM_TIMER,
      .duty_resolution  = ENGINE_PWM_DUTY_RESOLUTION,
      .freq_hz          = ENGINE_PWM_FREQUENCY,  
      .clk_cfg          = LEDC_AUTO_CLK
  };
  ESP_ERROR_CHECK(ledc_timer_config(&engine_pwm_timer));
}

//настройка каналов модуля LEDC
static void configuring_channel_for_PWM(uint8_t channel, uint8_t pin)   // подготавливаем каналы (GPIO) куда будут выдаваться ШИМ сигналы
{
  ledc_channel_config_t engine_pwm_channel = {
      .speed_mode     = ENGINE_PWM_MODE,
      .channel        = channel,
      .timer_sel      = ENGINE_PWM_TIMER,
      .intr_type      = LEDC_INTR_DISABLE,
      .gpio_num       = pin,
      .duty           = ENGINE_PWM_MIN_DUTY ,
      .hpoint         = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&engine_pwm_channel));
}

//создание таймера, отсчитывающего временые промежутки для фильтра Маджвика
static void Create_and_start_GP_Timer()                    
{
  gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 10 * 1000 * 1000, // 10MHz, 1 tick = 0,1us
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &GP_timer));
  ESP_ERROR_CHECK(gptimer_enable(GP_timer));
  ESP_ERROR_CHECK(gptimer_start(GP_timer));
}

//непосредственно задача первичной инициализации.
//Проверяет связь со всеми компонентами, производит их настройку. При необходимости калибрует ESC, запускает задачу считывания логов. 
//если все прошло гладко - создает основные рабочие задачи. Если где-то ошибка - запускается задача аварийного моргания светодиодами для визуальной индикации ошибки.
//по завершении выполнения задача инициализации самоликвидируется, так как больше не нужна
void init(void * pvParameters)
{
  uint8_t error_code = 0;

  esp_log_level_set(TAG_INIT,ESP_LOG_INFO);  //WARN ERROR
  esp_log_level_set(TAG_FLY,ESP_LOG_INFO);  //WARN ERROR
  esp_log_level_set(TAG_GPS,ESP_LOG_ERROR);  //WARN ERROR
  esp_log_level_set(TAG_NVS,ESP_LOG_INFO);   //WARN ERROR
  esp_log_level_set(TAG_RC,ESP_LOG_ERROR);    //WARN ERROR
  esp_log_level_set(TAG_PMW,ESP_LOG_WARN);   //WARN ERROR
  esp_log_level_set(TAG_W25N,ESP_LOG_INFO);  //WARN ERROR
  esp_log_level_set(TAG_MPU6000,ESP_LOG_INFO);  //WARN ERROR
  esp_log_level_set(TAG_PCA9685,ESP_LOG_INFO);  //WARN ERROR
  esp_log_level_set(TAG_MCP23017,ESP_LOG_INFO);  //WARN ERROR
  esp_log_level_set(TAG_LIDAR,ESP_LOG_INFO);  //WARN ERROR
  esp_log_level_set(TAG_INA219,ESP_LOG_WARN);  //WARN ERROR
  esp_log_level_set(TAG_IST8310,ESP_LOG_INFO);  //WARN ERROR
  esp_log_level_set(TAG_FL3195,ESP_LOG_WARN); 
  esp_log_level_set(TAG_MAV,ESP_LOG_INFO);
  esp_log_level_set(TAG_MS5611,ESP_LOG_INFO);  

  #ifdef TELNET_CONF_MODE
  esp_log_level_set("wifi",ESP_LOG_WARN);
  esp_log_level_set("wifi_init",ESP_LOG_WARN);
  #endif

  

  printf("\n");
  ESP_LOGI(TAG_INIT,"Старт системы\n");

  ESP_LOGI(TAG_INIT,"Конфигурирование пинов входов - выходов.....");
  configure_IOs();
  ESP_LOGI(TAG_INIT,"Входы - выходы настроены\n");
 
  ESP_LOGI(TAG_INIT,"Настраиваем внутренний i2c.....");
  i2c_init_internal(I2C_INT_SDA, I2C_INT_SCL, I2C_INT_PORT);
  ESP_LOGI(TAG_INIT,"Внутренний i2c настроен\n");

  ESP_LOGI(TAG_INIT,"Настройка внешнего i2c.....");
  i2c_init_external(I2C_EXT_SDA, I2C_EXT_SCL, I2C_EXT_PORT);
  ESP_LOGI(TAG_INIT,"Внешний i2c настроен\n");

  ESP_LOGI(TAG_INIT,"Настраиваем оба SPI.....");
  SPI_init();
  ESP_LOGI(TAG_INIT,"Оба SPI настроены\n");

#ifdef USING_PX4FLOW
  px4flow_i2c_frame_t px4flow_frame;
  px4flow_i2c_integral_frame_t px4flow_int_frame;
  ESP_LOGI(TAG_PX4FLOW,"Проверка связи с PX4FLOW.....");
  if (px4flow_communication_check() != ESP_OK) {
    error_code = 3;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  while(1)
  {
    px4flow_read_frame(&px4flow_frame);
    printf("REG: count %d, qual %d\n", px4flow_frame.frame_count, px4flow_frame.quality);
    vTaskDelay(100/portTICK_PERIOD_MS);

    px4flow_read_integral_frame(&px4flow_int_frame);
    printf("INT: gyro temp %d, qual %d\n\n", px4flow_int_frame.gyro_temperature, px4flow_int_frame.quality);
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
#endif

  ESP_LOGI(TAG_INIT,"Проверка связи с MCP23017.....");
  if (MCP23017_communication_check() != ESP_OK) {
    error_code = 3;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_INIT,"Настройка MCP23017.....");
  if (MCP23017_init() != ESP_OK) {
    error_code = 4;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
 
  ESP_LOGI(TAG_INIT,"Создание очереди для MCP23017.....");
  MCP23017_queue = xQueueCreate(10, sizeof(uint8_t));
  if (MCP23017_queue == NULL) {
    ESP_LOGE(TAG_INIT,"Очередь для MCP23017 не может быть создана\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
  else ESP_LOGI(TAG_INIT,"Очередь для MCP23017 создана\n");

//настраиваем модуль ШИМ на борту ESP32. Делаем это в начале программы, чтобы сразу выдать на ESC управляющие сигналы и они не пищали  
  ESP_LOGI(TAG_INIT,"Настройка ШИМ-модуля (LEDC) для управления двигателями.....");
  configuring_timer_for_PWM();
  configuring_channel_for_PWM(0,ENGINE_PWM_OUTPUT_0_PIN);
  configuring_channel_for_PWM(1,ENGINE_PWM_OUTPUT_1_PIN);
  configuring_channel_for_PWM(2,ENGINE_PWM_OUTPUT_2_PIN);
  configuring_channel_for_PWM(3,ENGINE_PWM_OUTPUT_3_PIN);
  ESP_LOGI(TAG_INIT,"ШИМ-модуль для управления двигателями настроен\n");

  if (!(MCP23017_get_inputs_state() & 0b00000001))        //если стоит DI0 - заходим в меню настойки (запускаем отдельную задачу settings)
  {
    ESP_LOGE(TAG_INIT,"Установлена перемычка DI0, входим в режим конфигурирования.....");
#ifdef TELNET_CONF_MODE
    ESP_LOGW(TAG_INIT,"Запускаем режим конфигурирования по WiFi через Telnet");    
    xTaskCreate(configuration_mode_telnet, "configuration_mode_telnet", 4096, NULL, 10, NULL);
#else
    ESP_LOGW(TAG_INIT,"Запускаем режим конфигурирования по USB-UART");     
    xTaskCreate(configuration_mode, "configuration_mode", 4096, NULL, 10, NULL);
#endif
    vTaskSuspend(NULL); //останавливаем init
  }

  if (!(MCP23017_get_inputs_state() & 0b00000100))                                      //если стоит перемычка DI2 - запускаем калибровку ESC
    { 
      ESP_LOGW(TAG_INIT,"Установлена перемычка DI2, начинаем калибровку ESCs.....");
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 0, ENGINE_PWM_MIN_DUTY*2));        //выдаем на все каналы максимальный уровень сигнала
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 0));
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 1, ENGINE_PWM_MIN_DUTY*2));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 1));
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 2, ENGINE_PWM_MIN_DUTY*2));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 2));
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 3, ENGINE_PWM_MIN_DUTY*2));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 3));
      vTaskDelay(3000/portTICK_PERIOD_MS);                                               //держим его 3 секунды
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 0, ENGINE_PWM_MIN_DUTY));           //выдаем на все каналы минимальный уровень сигнала
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 0));
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 1, ENGINE_PWM_MIN_DUTY));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 1));
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 2, ENGINE_PWM_MIN_DUTY));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 2));
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 3, ENGINE_PWM_MIN_DUTY));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 3));
      vTaskDelay(3000/portTICK_PERIOD_MS);                                                //держим его 3 секунды
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 0, ENGINE_PWM_MIN_DUTY+1));          //выдаем на все каналы минимальный уровень сигнала + 1 (не обязательное действие)
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 0));
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 1, ENGINE_PWM_MIN_DUTY+1));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 1));
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 2, ENGINE_PWM_MIN_DUTY+1));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 2));
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 3, ENGINE_PWM_MIN_DUTY+1));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 3));
      
      ESP_LOGI(TAG_INIT,"Калибровка ESC завершена, снимите перемычку и перезапустите систему.");
      
      while (1) 
      { gpio_set_level(LED_RED, 1);
        vTaskDelay(500/portTICK_PERIOD_MS); 
        gpio_set_level(LED_RED, 0);
        vTaskDelay(500/portTICK_PERIOD_MS);
      }
    }

    ESP_LOGI(TAG_INIT,"Проверка связи с PCA9685.....");
    if (PCA9685_communication_check() != ESP_OK) {
      error_code = 5;
      xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
      while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
    }

    ESP_LOGI(TAG_INIT,"Настройка PCA9685.....");
    if (PCA9685_init() != ESP_OK) {
    error_code = 6;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
  
  ESP_LOGI(TAG_INIT,"Сбрасываем все выходы PCA9685.....");
  for (uint8_t i = 0; i<16; i++) PCA9685_send(0, i);

  ESP_LOGI(TAG_INIT,"Проверка связи с INA219.....");
  if (INA219_communication_check() != ESP_OK) {
    error_code = 7;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_INIT,"Настройка INA219.....");
  if (INA219_configuration() != ESP_OK) {
    error_code = 8;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

#ifdef USING_FL3195
  ESP_LOGI(TAG_INIT,"Проверка связи с FL3195.....");
  if (FL3195_communication_check() != ESP_OK) {
    error_code = 9;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_INIT,"Настройка FL3195.....");
  if (FL3195_configuration() != ESP_OK) {
    error_code = 10;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
#endif

#ifdef USING_MS5611
  ESP_LOGI(TAG_MS5611,"Проверка связи с MS5611.....");
  if (MS5611_communication_check() != ESP_OK) {
    error_code = 3;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_MS5611,"Сброс MS5611.....");
  if (MS5611_I2C_reset() != ESP_OK) {
    error_code = 3;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
#endif


#ifdef USING_MAGNETOMETER
  ESP_LOGI(TAG_INIT,"Проверка связи с IST8310.....");
  if (IST8310_communication_check() != ESP_OK) {
    error_code = 11;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_INIT,"Настройка IST8310.....");
    if (IST8310_configuration() != ESP_OK) {
    error_code = 12;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_INIT,"Производим тест IST8310.....");
  if (IST8310_selftest() != ESP_OK) {
    error_code = 13;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
#endif

#ifdef USING_TFMINIS_I2C
ESP_LOGI(TAG_INIT,"Проверка связи с Tfmini-S (по i2c).....");
if (tfminis_communication_check() != ESP_OK) {
  error_code = 9;
  xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
  while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
}
#endif
/*
  ESP_LOGI(TAG_INIT,"Настраиваем оба SPI.....");
  SPI_init();
  ESP_LOGI(TAG_INIT,"Оба SPI настроены\n");
*/
  ESP_LOGI(TAG_INIT,"Проверка связи с MPU#1.....");
  if (MPU6000_communication_check(MPU6000_1) != ESP_OK) {
    error_code = 14;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);}  }
  
  ESP_LOGI(TAG_INIT,"Проверка связи с MPU#2.....");
  if (MPU6000_communication_check(MPU6000_2) != ESP_OK) {
    error_code = 16;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
/*  
  ESP_LOGI(TAG_INIT,"Performing self-test of MPU#1.....");
  if (MPU6000_self_test(MPU6000_1) != ESP_OK) {
    error_code = 4;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  } 

  ESP_LOGI(TAG_INIT,"Performing self-test of MPU#2.....");
  if (MPU6000_self_test(MPU6000_2) != ESP_OK) {
    error_code = 5;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  } 
*/
  ESP_LOGI(TAG_INIT,"Настройка MPU#1.....");
  if (MPU6000_init(MPU6000_1) != ESP_OK) {
    error_code = 15;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_INIT,"Настройка MPU#2.....");
  if (MPU6000_init(MPU6000_2) != ESP_OK) {
    error_code = 17;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
  
  ESP_LOGI(TAG_INIT,"Перенастройка SPI на 20МГц.....");
  SPI_change_MPUs_speed();
  ESP_LOGI(TAG_INIT,"Оба SPI перенастроены\n");

#ifdef USING_W25N 
  ESP_LOGI(TAG_INIT,"Сбрасываем W25N.....");
  W25N_reset();
  
  ESP_LOGI(TAG_INIT,"Считываем JEDEC ID.....");
  if (W25N_read_JEDEC_ID() != ESP_OK) {
    error_code = 18;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_INIT,"Записываем status-регистры W25N.....");
  W25N_write_status_register(W25N_PROT_REG_SR1, 0x00);         //снимаем защиту от записи
  W25N_write_status_register(W25N_CONFIG_REG_SR2, 0b00011000); //оставляем ECC включенным и Buffer Read Mode (постепенное считывание)

  ESP_LOGI(TAG_INIT,"Удаляем данные с W25N.....");
  W25N_erase_all_new();
  
  W25N01_queue = xQueueCreate(5, sizeof(struct logging_data_set *));
  if (W25N01_queue == NULL) {
    ESP_LOGE(TAG_INIT,"Очередь для записи логов на внешнюю flash-память не создана\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"Очередь для записи логов на внешнюю flash-память успешно создана\n");
#endif  

  ESP_LOGI(TAG_INIT,"Создаем очередь для управления PCA9685.....");       //в эту очередь пишутся команды на управление PCA9685
  PCA9685_queue = xQueueCreate(10, sizeof(uint16_t));
  if (PCA9685_queue == NULL) {
    ESP_LOGE(TAG_INIT,"Очередь для управления PCA9685 не создана\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"Очередь для управления PCA9685 успешно создана\n");

#ifdef USING_MAGNETOMETER
  ESP_LOGI(TAG_INIT,"Создание очереди для передачи данных от магнетометра в main_flying_cycle....."); //очередь через которую магнетометр передает данные в main 
  magnetometer_queue = xQueueCreate(9, 3 * sizeof(float));                                            //9 values 6 bytes each (3 x 2)
  if (magnetometer_queue == NULL) {
    ESP_LOGE(TAG_INIT,"Очередь для передачи данных от магнетометра в main_flying_cycle не создана\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"Очередь для передачи данных от магнетометра в main_flying_cycle успешно создана\n");
  #endif

  ESP_LOGI(TAG_INIT,"Создаем семафор для контроля доступа к внешнему i2c.....");                       //семафор, арбитрирующий доступ к внешней шине i2c 
  semaphore_for_i2c_external = xSemaphoreCreateBinaryStatic(&semaphore_for_i2c_external_buffer);
  if (semaphore_for_i2c_external == NULL) {
    ESP_LOGE(TAG_INIT,"Семафор для контроля доступа к внешнему i2c не создан\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
  else ESP_LOGI(TAG_INIT,"Семафор для контроля доступа к внешнему i2c успешно создан\n");

  xSemaphoreGive(semaphore_for_i2c_external);        //The semaphore is created in the 'empty' state, meaning the semaphore must first be given

  ESP_LOGI(TAG_INIT,"Создаем семафор для контроля доступа к внутреннему i2c.....");                     //семафор, арбитрирующий доступ к внутренней шине i2c 
  semaphore_for_i2c_internal = xSemaphoreCreateBinaryStatic(&semaphore_for_i2c_internal_buffer);
  if (semaphore_for_i2c_internal == NULL) {
    ESP_LOGE(TAG_INIT,"Семафор для контроля доступа к внутреннему i2c не создан\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
  else ESP_LOGI(TAG_INIT,"Семафор для контроля доступа к внутреннему i2c успешно создан\n");

  xSemaphoreGive(semaphore_for_i2c_internal);   //The semaphore is created in the 'empty' state, meaning the semaphore must first be given

  ESP_LOGI(TAG_INIT,"Создаем очередь для передачи данных от пульта управления в main_flying_cycle.....");
  remote_control_to_main_queue = xQueueCreate(10, sizeof(struct data_from_rc_to_main_struct));
  if (remote_control_to_main_queue == NULL) {
    ESP_LOGE(TAG_INIT,"Очередь для передачи данных от пульта управления в main_flying_cycle не создана\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"Очередь для передачи данных от пульта управления в main_flying_cycle успешно создана\n"); 

  ESP_LOGI(TAG_INIT,"Создаем очередь для передачи данных от GPS в main_flying_cycle");
  gps_to_main_queue = xQueueCreate(10, sizeof(struct data_from_gps_to_main_struct));
  if (gps_to_main_queue == NULL) {
    ESP_LOGE(TAG_INIT,"Очередь для передачи данных от GPS в main_flying_cycle не создана\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"Очередь для передачи данных от GPS в main_flying_cycle успешно создана.....\n");
     
  ESP_LOGI(TAG_INIT,"Создаем очередь для передачи телеметрии из main_flying_cycle на пульт управления");
  main_to_rc_queue = xQueueCreate(1, sizeof(struct data_from_main_to_rc_struct));     //size 1 because use xQueueOverwrite
  if (main_to_rc_queue == NULL) {
    ESP_LOGE(TAG_INIT,"Очередь для передачи телеметрии из main_flying_cycle на пульт управления не создана\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"Очередь для передачи телеметрии из main_flying_cycle на пульт управления успешно создана\n"); 

#ifdef USING_TFMINIS_I2C 
  ESP_LOGI(TAG_INIT,"Создаем очередь для передачи данных от лидара в main_flying_cycle.....");
  lidar_to_main_queue = xQueueCreate(10, sizeof(struct data_from_lidar_to_main_struct));
  if ( lidar_to_main_queue == NULL) {
    ESP_LOGE(TAG_INIT,"Очередь для передачи данных от лидара в main_flying_cycle не создана\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"Очередь для передачи данных от лидара в main_flying_cycle успешно создана\n"); 
#endif
  
  ESP_LOGI(TAG_INIT,"Создаем очередь для передачи данных от INA219 в main_flying_cycle.....");
  INA219_to_main_queue = xQueueCreate(10, 4 * sizeof(float));
  if (INA219_to_main_queue == NULL) {
    ESP_LOGE(TAG_INIT,"Очередь для передачи данных от INA219 в main_flying_cycle не создана\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"Очередь для передачи данных от INA219 в main_flying_cycle успешно создана\n");

#ifdef USING_MAVLINK_TELEMETRY
  ESP_LOGI(TAG_INIT,"Создаем очередь для передачи данных от  main_flying_cycle в Mavlink.....");
  main_to_mavlink_queue = xQueueCreate(5, sizeof(struct mavlink_data_set *));
  if (  main_to_mavlink_queue == NULL) {
    ESP_LOGE(TAG_INIT,"Очередь для передачи данных от main_flying_cycle в Mavlink не создана\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"Очередь для передачи данных от от main_flying_cycle в Mavlink успешно создана\n"); 
#endif

#ifdef USING_MS5611
  ESP_LOGI(TAG_INIT,"Создаем очередь для передачи данных от MS5611 в main_flying_cycle.....");
  MS5611_to_main_queue = xQueueCreate(5, sizeof(float));
  if (MS5611_to_main_queue == NULL) {
    ESP_LOGE(TAG_INIT,"Очередь для передачи данных от MS5611 в main_flying_cycle не создана\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"Очередь для передачи данных от MS5611 в main_flying_cycle успешно создана\n"); 
#endif

  ESP_LOGI(TAG_INIT,"Создаем и запускаем GP timer.....");
  Create_and_start_GP_Timer ();
  ESP_LOGI(TAG_INIT,"GP timer создан и запущен\n");

//*************************************************** НАЧИНАЕМ СОЗДАВАТЬ ЗАДАЧИ ****************************************************************************************************** */

  ESP_LOGI(TAG_INIT,"Приступаем к созданию задач\n");

  //core 0 deals with WiFI tasks

 task_handle_MCP23017_monitoring_and_control = xTaskCreateStaticPinnedToCore(MCP23017_monitoring_and_control,"MCP23017_monitoring_and_control",MCP23017_MONITORING_AND_CONTROL_STACK_SIZE,NULL,MCP23017_MONITORING_AND_CONTROL_PRIORITY,MCP23017_monitoring_and_control_stack,&MCP23017_monitoring_and_control_TCB_buffer,0);
  if (task_handle_MCP23017_monitoring_and_control != NULL)
    ESP_LOGI(TAG_INIT,"Задача контроля и управления MCP23017 успешно создана на ядре 0\n");
  else {
    ESP_LOGE(TAG_INIT,"Задача контроля и управления MCP23017 не создана\n");
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_INIT,"Создаем задачу для управления PCA9685 (PCA9685_control)..... ");
  task_handle_PCA9685_control = xTaskCreateStaticPinnedToCore(PCA9685_control,"PCA9685_control",PCA9685_CONTROL_STACK_SIZE,NULL,PCA9685_CONTROL_PRIORITY,PCA9685_control_stack,&PCA9685_control_TCB_buffer,0);
  if (task_handle_PCA9685_control != NULL) 
    ESP_LOGI(TAG_INIT,"Задача для управления PCA9685 успешно создана на ядре 0\n");
  else {
    ESP_LOGE(TAG_INIT,"Задача для управления PCA9685 не создана\n");
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

#ifdef USING_GPS               
  ESP_LOGI(TAG_INIT,"Создаем задачу для считывания данных GPS (gps_read_and_process_data).....");
  if (xTaskCreateStaticPinnedToCore(gps_read_and_process_data, "gps_read_and_process_data", GPS_READ_AND_PROCESS_DATA_STACK_SIZE, NULL, GPS_READ_AND_PROCESS_DATA_PRIORITY, gps_read_and_process_data_stack, &gps_read_and_process_data_TCB_buffer, 0) != NULL)
    ESP_LOGI(TAG_INIT,"Задача для считывания данных GPS успешно создана на ядре 0\n");
  else {
    ESP_LOGE(TAG_INIT,"Задача для считывания данных GPS не создана\n");
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
#endif
  vTaskDelay(50/portTICK_PERIOD_MS);

#ifdef USING_MAGNETOMETER 
  ESP_LOGI(TAG_INIT,"Создаем задачу для считывания данных с магнетометра (mag_read_and_process_data).....");
  task_handle_mag_read_and_process_data = xTaskCreateStaticPinnedToCore(mag_read_and_process_data, "mag_read_and_process_data", MAG_READ_AND_PROCESS_DATA_STACK_SIZE, NULL, MAG_READ_DATA_AND_SEND_TO_MAIN_PRIORITY, mag_read_and_process_data_stack, &mag_read_and_process_data_TCB_buffer, 0);
  if (task_handle_mag_read_and_process_data != NULL)
    ESP_LOGI(TAG_INIT,"Задача для считывания данных с магнетометра успешно создана на ядре 0\n");
  else {
    ESP_LOGE(TAG_INIT,"Задача для считывания данных с магнетометра не создана\n");
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
#endif        
        
  vTaskDelay(50/portTICK_PERIOD_MS);
    
  ESP_LOGI(TAG_INIT,"Создаем задачу для получения данных с пульта управления (RC_read_and_process_data).....");
  if (xTaskCreateStaticPinnedToCore(RC_read_and_process_data,"RC_read_and_process_data",RC_READ_AND_PROCESS_DATA_STACK_SIZE,NULL,RC_READ_AND_PROCESS_DATA_PRIORITY,RC_read_and_process_data_stack,&RC_read_and_process_data_TCB_buffer,0) != NULL)
    ESP_LOGI(TAG_INIT,"Задача для получения данных с пульта управления успешно создана на ядре 0\n");
  else {
    ESP_LOGE(TAG_INIT,"Задача для получения данных с пульта управления не создана\n");
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  vTaskDelay(50/portTICK_PERIOD_MS);

  ESP_LOGI(TAG_INIT,"Создаем задачу для отправки телеметрии на пульт управления.....");
  task_handle_send_data_to_RC = xTaskCreateStaticPinnedToCore(send_data_to_RC,"send_data_to_RC",SEND_DATA_TO_RC_STACK_SIZE ,NULL,SEND_DATA_TO_RC_PRIORITY,send_data_to_RC_stack,&send_data_to_RC_TCB_buffer,0);
  if (task_handle_send_data_to_RC != NULL)
    ESP_LOGI(TAG_INIT,"Задача для отправки телеметрии на пульт управления успешно создана на ядре 0\n");
  else {
    ESP_LOGE(TAG_INIT,"Задача для отправки телеметрии на пульт управления не создана\n");
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

#ifdef USING_PERFORMANCE_MESUREMENT
  ESP_LOGI(TAG_INIT,"Создаем задачу контроля загруженности процессора.....");
  task_handle_performace_measurement = xTaskCreateStaticPinnedToCore(performance_monitor,"performance_monitor",8192 ,NULL,PERFORMANCE_MEASUREMENT_PRIORITY,performance_measurement_stack, &performance_measurement_TCB_buffer,0);
  if (task_handle_performace_measurement != NULL)
    ESP_LOGI(TAG_INIT,"Задача контроля загруженности процессора успешно создана на ядре 0\n");
  else {
  ESP_LOGE(TAG_INIT,"Задача контроля загруженности процессора не создана\n");
  error_code = 2;
  xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
  while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
}
#endif        
        
  vTaskDelay(50/portTICK_PERIOD_MS);

#ifdef USING_TFMINIS_I2C 
  ESP_LOGI(TAG_INIT,"Создавем задачу для получения данных от лидара (lidar_read_and_process_data).....");
  task_handle_lidar_read_and_process_data = xTaskCreateStaticPinnedToCore(lidar_read_and_process_data,"lidar_read_and_process_data",LIDAR_READ_AND_PROCESS_DATA_STACK_SIZE ,NULL,LIDAR_READ_AND_PROCESS_DATA_PRIORITY,lidar_read_and_process_data_stack,&lidar_read_and_process_data_TCB_buffer,0);
  if (task_handle_lidar_read_and_process_data != NULL)  
    ESP_LOGI(TAG_INIT,"Задача для получения данных от лидара успешно создана на ядре 0\n");
  else {
    ESP_LOGE(TAG_INIT,"Задача для получения данных от лидара не создана\n");
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  vTaskDelay(50/portTICK_PERIOD_MS);
#endif
       
  ESP_LOGI(TAG_INIT,"Создаем задачу считывания данных с INA219 (INA219_read_and_process_data).....");
  task_handle_INA219_read_and_process_data = xTaskCreateStaticPinnedToCore(INA219_read_and_process_data,"INA219_read_and_process_data",INA219_READ_AND_PROCESS_DATA_STACK_SIZE,NULL,INA219_READ_AND_PROCESS_DATA_PRIORITY,INA219_read_and_process_data_stack, &INA219_read_and_process_data_TCB_buffer,0);
  if ( task_handle_INA219_read_and_process_data != NULL)
    ESP_LOGI(TAG_INIT,"Задача считывания данных с INA219 успешно создана на ядре 0\n");
  else {
    ESP_LOGE(TAG_INIT,"Задача считывания данных с INA219 не создана\n");
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_INIT,"Создаем задачу моргания полетными огнями (blinking_flight_lights).....");
  task_handle_blinking_flight_lights = xTaskCreateStaticPinnedToCore(blinking_flight_lights,"blinking_flight_lights",BLINKING_FLIGHT_LIGHTS_STACK_SIZE,NULL,BLINKING_FLIGHT_LIGHTS_PRIORITY,blinking_flight_lights_stack, &blinking_flight_lights_TCB_buffer,0);
  if ( task_handle_blinking_flight_lights != NULL)
    ESP_LOGI(TAG_INIT,"Задача моргания полетными огнями успешно создана на ядре 0\n");
  else {
    ESP_LOGE(TAG_INIT,"Задача моргания полетными огнями не создана\n");
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  gpio_set_level(GREEN_FLIGHT_LIGHTS, 1);
  gpio_set_level(RED_FLIGHT_LIGHTS, 1);  

  vTaskDelay(50/portTICK_PERIOD_MS); 

#ifdef USING_W25N
  ESP_LOGI(TAG_INIT,"Создаем задачу записи логов во внешнюю flash-память (writing_logs_to_flash).....");
  task_handle_writing_logs_to_flash = xTaskCreateStaticPinnedToCore(writing_logs_to_flash,"writing_logs_to_flash",WRITING_LOGS_TO_FLASH_STACK_SIZE,NULL,WRITING_LOGS_TO_FLASH_PRIORITY,writing_logs_to_flash_stack, &writing_logs_to_flash_TCB_buffer,0);
  if (task_handle_writing_logs_to_flash !=NULL) 
    {
      ESP_LOGI(TAG_INIT,"Задача записи логов во внешнюю flash-память успешно создана на ядре 0\n");
    }
  else {
    ESP_LOGE(TAG_INIT,"Задача записи логов во внешнюю flash-память не создана\n");
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }       
#endif

#ifdef USING_MS5611
  ESP_LOGI(TAG_INIT,"Создаем задачу для получения данных от барометра MS5611 (MS5611_read_and_process_data).....");
  task_handle_MS5611_read_and_process_data = xTaskCreateStaticPinnedToCore(MS5611_read_and_process_data,"MS5611_read_and_process_data",MS5611_READ_AND_PROCESS_DATA_STACK_SIZE ,NULL,MS5611_READ_AND_PROCESS_DATA_PRIORITY,MS5611_read_and_process_data_stack,&MS5611_read_and_process_data_TCB_buffer,0);
  if (task_handle_MS5611_read_and_process_data != NULL)  
    ESP_LOGI(TAG_INIT,"Задача для получения данных от MS5611 успешно создана на ядре 0\n");
  else {
    ESP_LOGE(TAG_INIT,"Задача для получения данных от MS5611 не создана\n");
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  vTaskDelay(50/portTICK_PERIOD_MS);
#endif

  ESP_LOGI(TAG_INIT,"Создаем задачу основного полетного цикла (Main_flying_cycle)..... ");
  task_handle_main_flying_cycle = xTaskCreateStaticPinnedToCore(main_flying_cycle, "Main_flying_cycle", MAIN_FLYING_CYCLE_STACK_SIZE, NULL, MAIN_FLYING_CYCLE_PRIORITY, main_flying_cycle_stack,&main_flying_cycle_TCB_buffer,1);
  if (task_handle_main_flying_cycle != NULL)
    ESP_LOGI(TAG_INIT,"Задача основного полетного цикла успешно создана на ядре 1\n");
  else {
    ESP_LOGE(TAG_INIT,"Задача основного полетного цикла не создана\n");
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  #ifdef USING_MAVLINK_TELEMETRY
//задержка пока загрузится minim_OSD
  vTaskDelay(3000/portTICK_PERIOD_MS);
  ESP_LOGI(TAG_INIT,"Создаем задачу отправки телеметрии по mavink (send_telemetry_via_mavlink).....");
  task_handle_mavlink_telemetry = xTaskCreateStaticPinnedToCore(send_telemetry_via_mavlink,"send_telemetry_via_mavlink",MAVLINK_TELEMETRY_STACK_SIZE,NULL,MAVLINK_TELEMETRY_PRIORITY,mavlink_telemetry_stack, &mavlink_telemetry_TCB_buffer,0);
  if (task_handle_mavlink_telemetry != NULL) 
    {
      ESP_LOGI(TAG_INIT,"Задача отправки телеметрии по mavink успешно создана на ядре 0\n");
    }
  else {
    ESP_LOGE(TAG_INIT,"Задача отправки телеметрии по mavink не создана\n");
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }       
#endif

//все задачи созданы, убиваем задачу инициализации         
  vTaskDelete(NULL);  
}
