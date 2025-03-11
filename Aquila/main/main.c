//системные библиотеки
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/gptimer.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "nvs_flash.h"
#include "math.h"
#include "esp_adc/adc_oneshot.h"
#include <sys/time.h>
#include "esp_random.h"
#include <rom/ets_sys.h>
#include "soc/gpio_reg.h"
#include "mbedtls/aes.h"

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
#include "AES_crypto.h"
#include "filters.c"
#include "q_operations.c"
#include "mavlink_wt.h"
#include "mavlink_crc.h"

/********************************************************************     СЕКЦИЯ 1     ***********************************************************************************************/
/**************************************************************************************************************************************************************************************/
//таги для логирования
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
static SemaphoreHandle_t semaphore_for_i2c_external;
static StaticSemaphore_t semaphore_for_i2c_external_buffer;

static SemaphoreHandle_t semaphore_for_i2c_internal;
static StaticSemaphore_t semaphore_for_i2c_internal_buffer;

//timers handles
static gptimer_handle_t GP_timer;
static gptimer_handle_t general_suspension_timer;
static gptimer_handle_t IMU_1_suspension_timer;
static gptimer_handle_t IMU_2_suspension_timer;

//queues handles
static QueueHandle_t gps_queue_for_events = NULL; //очередь для передачи событий от UART с подключенным GPS (pattern detection) в задачу обработки данных GPS
static QueueHandle_t gps_to_main_queue = NULL; //очередь для передачи обработанных GPS данных в main_flying_cycle
static QueueHandle_t remote_control_queue_for_events = NULL; //очередь для передачи событий от UART пульта управления (pattern detection) в задачу обработки данных пульта
static QueueHandle_t remote_control_to_main_queue = NULL; //очередь для передачи обработанных данных от пульта в main_flying_cycle
static QueueHandle_t lidar_queue_for_events = NULL; //очередь для передачи событий от UART с подключенным лидаром (pattern detection) в задачу обработки данных от лидара
static QueueHandle_t lidar_to_main_queue = NULL; //очередь для передачи обработанных данных от лидара в main_flying_cycle
static QueueHandle_t MCP23017_queue = NULL; //очередь принимающая команды на считывания состояния или управления MCP23017
static QueueHandle_t PCA9685_queue = NULL; //очередь принимающая команды управления PCA9685
static QueueHandle_t magnetometer_queue = NULL; //очередь для передачи обработанных данных из задачи обработки данных магнетометра в main_flying_cycle
static QueueHandle_t main_to_rc_queue = NULL; //очередь для передачи данных из main_flying_cycle в задачу отправки телеметрии на пульт
static QueueHandle_t W25N01_queue = NULL;  //очередь для передачи данных из main_flying_cycle в задачу записи логов во внешнюю флэш-память
static QueueHandle_t INA219_to_main_queue = NULL; //очередь для передачи данных из задачи обработки данных от INA219 в main_flying_cycle
static QueueHandle_t mav_queue_for_events = NULL; //очередь для побработки событий для UART Mavlink
static QueueHandle_t main_to_mavlink_queue = NULL; //очередь для передачи данных из main в mavlink



//параметры выделения памяти для статического размещения задач
StaticTask_t MCP23017_monitoring_and_control_TCB_buffer;
StackType_t MCP23017_monitoring_and_control_stack[MCP23017_MONITORING_AND_CONTROL_STACK_SIZE];

StaticTask_t PCA9685_control_TCB_buffer;
StackType_t PCA9685_control_stack[PCA9685_CONTROL_STACK_SIZE];

StaticTask_t mag_read_and_process_data_TCB_buffer;
StackType_t mag_read_and_process_data_stack[MAG_READ_AND_PROCESS_DATA_STACK_SIZE];

StaticTask_t main_flying_cycle_TCB_buffer;
StackType_t main_flying_cycle_stack[MAIN_FLYING_CYCLE_STACK_SIZE];

StaticTask_t monitoring_pins_interrupt_queue_TCB_buffer;
StackType_t monitoring_pins_interrupt_queue_stack[MONITORING_PINS_INTERRUPT_QUEUE_STACK_SIZE];

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

//tasks handlers
static TaskHandle_t task_handle_blinking_flight_lights;
static TaskHandle_t task_handle_main_flying_cycle;
static TaskHandle_t task_handle_send_data_to_RC;
static TaskHandle_t task_handle_init;
static TaskHandle_t task_handle_performace_measurement;
static TaskHandle_t task_handle_INA219_read_and_process_data;
static TaskHandle_t task_handle_mag_read_and_process_data;
static TaskHandle_t task_handle_PCA9685_control;
static TaskHandle_t task_handle_MCP23017_monitoring_and_control;
static TaskHandle_t task_handle_writing_logs_to_flash;
static TaskHandle_t task_handle_mavlink_telemetry;
static TaskHandle_t task_handle_lidar_read_and_process_data;

/********************************************************************     СЕКЦИЯ 2     ***********************************************************************************************/
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


/********************************************************************     СЕКЦИЯ 3     ***********************************************************************************************/
/********************************************************************   ОБЩИЕ ФУНКЦИИ  ***********************************************************************************************/

//настройка входов-выходов общего назначения
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


#ifdef USING_GPS
//настройка UART для GPS
static void gps_uart_config()
{
    int intr_alloc_flags = 0;
    uart_config_t gps_uart_config = {
        .baud_rate = GPS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(GPS_UART, GPS_UART_BUF_SIZE, 0, GPS_UART_PATTERN_DETECTION_QUEUE_SIZE, &gps_queue_for_events, intr_alloc_flags)); 
    ESP_ERROR_CHECK(uart_param_config(GPS_UART, &gps_uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART, GPS_UART_TX_PIN, GPS_UART_RX_PIN, GPS_UART_RTS_PIN, GPS_UART_CTS_PIN));

    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(GPS_UART, '*', 1, 9, 0, 0));                 //активируем прерывание по обнаружению пэттерна
    ESP_ERROR_CHECK(uart_pattern_queue_reset(GPS_UART, GPS_UART_PATTERN_DETECTION_QUEUE_SIZE));     //привязываем очередь 
    uart_flush(GPS_UART);                                                                           //очищаем FIFO RX буфер
}
#endif


static void remote_control_uart_config(void)
{
    int intr_alloc_flags = 0;
    uart_config_t uart_config = {
        .baud_rate = RC_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    //uart_set_rx_timeout(REMOTE_CONTROL_UART, 100);
    //https://esp32.com/viewtopic.php?f=13&t=35116

    ESP_ERROR_CHECK(uart_driver_install(REMOTE_CONTROL_UART, RC_RX_UART_BUFF_SIZE, RC_TX_UART_BUFF_SIZE, RC_UART_PATTERN_DETECTION_QUEUE_SIZE, &remote_control_queue_for_events, intr_alloc_flags)); 
    ESP_ERROR_CHECK(uart_param_config(REMOTE_CONTROL_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(REMOTE_CONTROL_UART, RC_UART_TX_PIN, RC_UART_RX_PIN, RC_UART_RTS_PIN, RC_UART_CTS_PIN));
#ifdef NO_RSSI
    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(REMOTE_CONTROL_UART, 0xFF, 1, 5, 0, 0));                 //активируем режим обнаружения пэттерна, в частности байта 0xFF, который является концом строки 
#endif
    ESP_ERROR_CHECK(uart_pattern_queue_reset(REMOTE_CONTROL_UART, RC_UART_PATTERN_DETECTION_QUEUE_SIZE));      //сбрасываем очередь
#ifdef NO_RSSI
    uart_disable_intr_mask(REMOTE_CONTROL_UART, (0x1 << 8));                                                   //UART_INTR_RXFIFO_TOUT
#endif
    uart_flush(REMOTE_CONTROL_UART);                                                                           //сбрасываем буфер
}

#ifdef USING_MAVLINK_TELEMETRY
static void mavlink_uart_config(void)
{
    int intr_alloc_flags = 0;
    uart_config_t uart_config = {
        .baud_rate = MAV_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(MAV_UART, MAV_RX_UART_BUF_SIZE, MAV_TX_UART_BUF_SIZE, MAV_UART_PATTERN_DETECTION_QUEUE_SIZE, &mav_queue_for_events, intr_alloc_flags)); 
    ESP_ERROR_CHECK(uart_param_config(MAV_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(MAV_UART, MAV_UART_TX_PIN, MAV_UART_RX_PIN, MAV_UART_RTS_PIN, MAV_UART_CTS_PIN));
    uart_flush(MAV_UART);                                                                           //сбрасываем буфер
}
#endif

//алгоритм вычисления контрольной суммы по типу Maxim (Dallas)
static uint8_t dallas_crc8(uint8_t *input_data, unsigned int size)
{
  uint8_t crc = 0;
  uint8_t i = 0;
  uint8_t inbyte = 0;
  uint8_t j = 0;
  uint8_t mix1 = 0;
  for (  i = 0; i < size; i++ ) {
        inbyte = input_data[i];
      for (  j = 0; j < 8; j++ ) {
          mix1 = (crc ^ inbyte) & 0x01;
          crc = crc >> 1;
          if ( mix1 ) crc ^= 0x8C;                         
          inbyte >>= 1;
      }}
  return crc;
}

//функция, возвращающая время в микросенундах со старта приложения
static int64_t get_time(void)
{
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  return (int64_t)((int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec);
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

//создание таймера, контролирующего зависание главного цикла. Сбрасывается из этого главного цикла. Если переполняется - по прерыванию аварийно отключаем моторы. 
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
static void Create_and_start_IMU_2_suspension_Timer()                    //timer to control absense of control signal from RC
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

//запись во flash (NVS) калибровочных коэффициентов акселерометров и гироскопов
void NVS_writing_calibration_values(int16_t accel_1_offset[], int16_t gyro_1_offset[], int16_t accel_2_offset[], int16_t gyro_2_offset[])
{
  ESP_ERROR_CHECK(nvs_flash_erase());
  esp_err_t err = nvs_flash_init();
  
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {

      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
  }
  ESP_ERROR_CHECK( err );

  ESP_LOGI(TAG_NVS,"Открываем NVS... ");
  nvs_handle_t NVS_handle;
  err = nvs_open("storage", NVS_READWRITE, &NVS_handle);
  if (err != ESP_OK) ESP_LOGE(TAG_NVS,"Ошибка (%s) открытия NVS!\n", esp_err_to_name(err));
  else 
  {
    ESP_LOGI(TAG_NVS,"Записываем accel_1_X offset %d...", accel_1_offset[0]);
    err = nvs_set_i16(NVS_handle, "accel_1_off_0", accel_1_offset[0]);
    
    ESP_LOGI(TAG_NVS,"Записываем accel_1_Y offset %d...", accel_1_offset[1]);
    err = nvs_set_i16(NVS_handle, "accel_1_off_1", accel_1_offset[1]);
  
    ESP_LOGI(TAG_NVS,"Записываем accel_1_Z offset %d...", accel_1_offset[2]);
    err = nvs_set_i16(NVS_handle, "accel_1_off_2", accel_1_offset[2]);

    ESP_LOGI(TAG_NVS,"Записываем gyro_1_X offset... ");
    err = nvs_set_i16(NVS_handle, "gyro_1_off_0", gyro_1_offset[0]);
    
    ESP_LOGI(TAG_NVS,"Записываем gyro_1_Y offset... ");
    err = nvs_set_i16(NVS_handle, "gyro_1_off_1", gyro_1_offset[1]);
  
    ESP_LOGI(TAG_NVS,"Записываем gyro_1_Z offset... ");
    err = nvs_set_i16(NVS_handle, "gyro_1_off_2", gyro_1_offset[2]);

    ESP_LOGI(TAG_NVS,"Записываем accel_2_X offset... ");
    err = nvs_set_i16(NVS_handle, "accel_2_off_0", accel_2_offset[0]);
    
    ESP_LOGI(TAG_NVS,"Записываем accel_2_Y offset... ");
    err = nvs_set_i16(NVS_handle, "accel_2_off_1", accel_2_offset[1]);
  
    ESP_LOGI(TAG_NVS,"Записываем accel_2_Z offset... ");
    err = nvs_set_i16(NVS_handle, "accel_2_off_2", accel_2_offset[2]);

    ESP_LOGI(TAG_NVS,"Записываем gyro_2_X offset... ");
    err = nvs_set_i16(NVS_handle, "gyro_2_off_0", gyro_2_offset[0]);
    
    ESP_LOGI(TAG_NVS,"Записываем gyro_2_Y offset... ");
    err = nvs_set_i16(NVS_handle, "gyro_2_off_1", gyro_2_offset[1]);
  
    ESP_LOGI(TAG_NVS,"Записываем gyro_2_Z offset... ");
    err = nvs_set_i16(NVS_handle, "gyro_2_off_2", gyro_2_offset[2]);

    ESP_LOGI(TAG_NVS,"Сохраняем данные в NVS ... ");
    err = nvs_commit(NVS_handle);

    nvs_close(NVS_handle);
  }
}

/********************************************************************     СЕКЦИЯ 4     ***********************************************************************************************/
/********************************************************************      ЗАДАЧИ      ***********************************************************************************************/


//задача моргания светодиодами на плате для отображения кода ошибки. Она создается при возниконовении какой-либо ошибки при прохождении первичной инициализации железа.
//Принимает в качестве параметра при создании <error_code> и моргает светодиодами <error_code> раз для отображения кода ошибки.
static void error_code_LED_blinking(void * pvParameters)              
{
  uint8_t i = 0;
  uint8_t *error_code = pvParameters;
  while(1) 
  {
    gpio_set_level(LED_RED, 1);
    gpio_set_level(LED_BLUE, 1);
    gpio_set_level(LED_GREEN, 1);
    vTaskDelay(2000/portTICK_PERIOD_MS);
    for (i=0; i<*error_code; i++) {
      gpio_set_level(LED_RED, 0);
      gpio_set_level(LED_BLUE, 0);
      gpio_set_level(LED_GREEN, 0);
      vTaskDelay(250/portTICK_PERIOD_MS);
      gpio_set_level(LED_RED, 1);
      gpio_set_level(LED_BLUE, 1);
      gpio_set_level(LED_GREEN, 1);
      vTaskDelay(250/portTICK_PERIOD_MS);
    }
  }
}

#ifdef USING_HOLYBRO_M9N
//задача периодического считывания данных с магнетометра IST8310. После создения находится с заблокированном состоянии. Разблокируется из main_flying_cycle.
//Активирует режим однократного измерения, считываем результат после задержки и выдает результат в очередь в сторону main_flying_cycle
static void mag_read_and_process_data (void * pvParameters)
{
  uint8_t i = 0;
  float cross_axis[3][3] = {{0.9800471,  -0.0310357,   -0.0148492},    //calculated manually based on data read from the chip
                            {0.0304362,   1.0342328,   -0.0004612},
                            {-0.0374089,  0.0419651,    1.0106678}};
  uint8_t mag_raw_values[6] = {0,0,0,0,0,0}; 
  int16_t magn_data[3]= {0,0,0};
  float magn_data_axis_corrected[3]= {0,0,0};
  float hard_bias[3] = {-51.782394, 18.121590, -66.742550};   //calibration values with Magneto 1.2
  
  float A_inv[3][3] = {{1.019563,   0.014807,   -0.010553},    //calibration values with Magneto 1.2
                       {0.014807,   1.037489,   0.006424},
                       {-0.010553,  0.006424,   1.111749}};
  
  float magn_wo_hb[3] = {0,0,0};
  float magn_data_calibrated[3] = {0,0,0};

  while(1) {
    
    if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) != 0)
    {
      xSemaphoreTake (semaphore_for_i2c_external,portMAX_DELAY);
      IST8310_request_data();
      xSemaphoreGive(semaphore_for_i2c_external);
      vTaskDelay(5/portTICK_PERIOD_MS);   //delay of 5ms as per datasheet min sampling is 200Hz 5ms
      xSemaphoreTake (semaphore_for_i2c_external,portMAX_DELAY);
      IST8310_read_data(mag_raw_values);
      xSemaphoreGive(semaphore_for_i2c_external);
  
      magn_data[0] = mag_raw_values[1] << 8 | mag_raw_values[0]; //X
      magn_data[1] = mag_raw_values[3] << 8 | mag_raw_values[2]; //Y
      magn_data[2] = mag_raw_values[5] << 8 | mag_raw_values[4]; //Z
      //printf ("%d,%d,%d\n",magn_data[0], magn_data[1], magn_data[2]);

      magn_data_axis_corrected[0] = cross_axis[0][0]*magn_data[0] + cross_axis[0][1]*magn_data[1] + cross_axis[0][2]*magn_data[2];
      magn_data_axis_corrected[1] = cross_axis[1][0]*magn_data[0] + cross_axis[1][1]*magn_data[1] + cross_axis[1][2]*magn_data[2];
      magn_data_axis_corrected[2] = cross_axis[2][0]*magn_data[0] + cross_axis[2][1]*magn_data[1] + cross_axis[2][2]*magn_data[2];
    //printf ("%0.4f,%0.4f,%0.4f\n",magn_data_axis_corrected[0], magn_data_axis_corrected[1], magn_data_axis_corrected[2]); // for compass calibration


      for (i=0;i<3;i++) magn_wo_hb[i] = (float)magn_data_axis_corrected[i] - hard_bias[i];
      //printf ("%0.4f,%0.4f,%0.4f\n",magn_wo_hb[0], magn_wo_hb[1], magn_wo_hb[2]);

      magn_data_calibrated[0] = A_inv[0][0]*magn_wo_hb[0] + A_inv[0][1]*magn_wo_hb[1] + A_inv[0][2]*magn_wo_hb[2];
      magn_data_calibrated[1] = A_inv[1][0]*magn_wo_hb[0] + A_inv[1][1]*magn_wo_hb[1] + A_inv[1][2]*magn_wo_hb[2];
      magn_data_calibrated[2] = A_inv[2][0]*magn_wo_hb[0] + A_inv[2][1]*magn_wo_hb[1] + A_inv[2][2]*magn_wo_hb[2];

      //ESP_LOGI(TAG_IST8310,"Mag values are %d, %d, %d, %0.2f",(int16_t)magn_data_calibrated[0],(int16_t)magn_data_calibrated[1],(int16_t)magn_data_calibrated[2], 
      //sqrt(magn_data_calibrated[0]*magn_data_calibrated[0] + magn_data_calibrated[1]*magn_data_calibrated[1] + magn_data_calibrated[2]*magn_data_calibrated[2]));
      //printf ("%0.2f,%0.2f,%0.2f,%0.2f\n",magn_data_calibrated[0], magn_data_calibrated[1], magn_data_calibrated[2], sqrt(magn_data_calibrated[0]*magn_data_calibrated[0] + magn_data_calibrated[1]*magn_data_calibrated[1] + magn_data_calibrated[2]*magn_data_calibrated[2]));
      //heading = atan2(magn_data_calibrated[1],magn_data_calibrated[0]) * (180/PI);
      //heading -=90.0;
      //if (heading<0) heading+=360.0;
      //printf ("%d\n",(int16_t) heading);
      //uart_write_bytes(REMOTE_CONTROL_UART, magn_data, NUMBER_OF_BYTES_TO_SEND_TO_RC);
      //length = sprintf(M,"%i,%i,%i\n",magn_data[0],magn_data[1],magn_data[2]);
      //uart_write_bytes(LIDAR_UART, M, length);
                 
      xQueueSend(magnetometer_queue, magn_data_calibrated, NULL); 
    }
  }  
}
#endif

#ifdef USING_GPS
//Задача получения и обработки данных от GPS. Подразумеваем что получаем только RMC сообщения.
//Ждет прерывания по обнаружению символа конца строки, при обнаружении разбираем полученную строку, вычленяем координаты и выдаем в очередь в сторону main_flying_cycle.
//Управляем трехцветным светодиодом FL3195 на модуле Holybro M9N для отображения сиатуса GPS. 
static void gps_read_and_process_data(void * pvParameters)
{
  uint8_t incoming_message_buffer_gps[NUMBER_OF_BYTES_TO_RECEIVE_FROM_GPS];
  uint16_t i = 0;
  uint16_t j = 0;
  int16_t pos = 0;
  unsigned char XOR = 0;
  unsigned char XOR_ch = 0;
  uint8_t asteriks_place = 0;
  unsigned char coma_places[13] = {0};
  uart_event_t gps_uart_event;  
  char latitude = 0;
  char longtitude = 0;
  struct data_from_gps_to_main_struct gps_data;
  uint8_t gps_status_old = 5;

  ESP_LOGI(TAG_GPS,"Настраиваем GPS UART.....");
  gps_uart_config();
  ESP_LOGI(TAG_GPS,"UART для GPS настроен");
       
  while(1) {
  if(xQueueReceive(gps_queue_for_events, (void * )&gps_uart_event, (TickType_t)portMAX_DELAY))      //ждем сообщений от UART
  {
    switch (gps_uart_event.type) {
            case UART_PATTERN_DET:
                pos = uart_pattern_pop_pos(GPS_UART);
                ESP_LOGD(TAG_GPS, "[UART PATTERN DETECTED] pos: %d", pos);
                if (pos > 130) {                                                                    //длина не может быть больше 130 байт для RMC
                  uart_flush_input(GPS_UART); 
                  xQueueReset(gps_queue_for_events);
                  }
                else 
                  {
                  uint8_t read_len = uart_read_bytes(GPS_UART, incoming_message_buffer_gps, pos+3, portMAX_DELAY);  //при обнаружении * нужно считать еще 2 символа
                  ESP_LOGD(TAG_GPS, "Из FIFO считано %d байт", read_len);
                    //for (i=0; i<read_len; i++) printf ("%c", incoming_message_buffer_gps[i]);
                 j = 0;
                  while ((incoming_message_buffer_gps[0] != '$') && (j < read_len)) {                               //выравниваем пока первый символ не станет $ (можно опустить)
                    for (i=0; i<read_len; i++) incoming_message_buffer_gps[i] = incoming_message_buffer_gps[i+1];
                      j++;
                     }
                   if (incoming_message_buffer_gps[4] == 'M')        // примитивная проверка на $GNRMC
                      {
                      uart_flush(GPS_UART);                           //очищаем сразу FIFO
                      i = 1;
                      j = 0;
                      XOR = 0;
                      XOR_ch = 0;
                      asteriks_place = 0;
              
                      while((asteriks_place == 0)) {                                                        //пока не обнаружим *
                          if (incoming_message_buffer_gps[i] == ',') {coma_places[j] = i; j++;}             //подсчитываем запятые и заносим их в массив coma_places
                          if (incoming_message_buffer_gps[i] == '*') asteriks_place = i;
                          if (asteriks_place == 0) XOR = XOR^incoming_message_buffer_gps[i];                //вычисляем XOR (контрольную сумму) всего что находится до *  
                          i++;               
                      }
//формируем численную контрольную сумму из полученных чаров   
                      if (incoming_message_buffer_gps[asteriks_place+1] <= 0x39) XOR_ch = (incoming_message_buffer_gps[asteriks_place+1] & 0x0F) << 4;  //если за * следует символ числа преобразуем его в цифру
                      else XOR_ch = (incoming_message_buffer_gps[asteriks_place+1] - 55) << 4;                                                          //если символ буквы
                      if (incoming_message_buffer_gps[asteriks_place+2] <= 0x39) XOR_ch |= (incoming_message_buffer_gps[asteriks_place+2] & 0x0F);      //то же самое со вторым символом контрольной суммы
                      else XOR_ch |= (incoming_message_buffer_gps[asteriks_place+2] - 55);
//если полученная из сообщения и вычисленная контрольная сумма совпадают
                      if (XOR == XOR_ch) {                                                                            
                        //for (i=0;i<90;i++) printf("%c",incoming_message_buffer_gps[i] );
                        //printf("\n");
//проверяем если режим GPS A или D
                        if  ((incoming_message_buffer_gps[asteriks_place-3] == 'A')||(incoming_message_buffer_gps[asteriks_place-3] == 'D')) 
                        { 
                        //Latitude, the format is ddmm.mmmmmmm
                        //Longitude, the format is dddmm.mmmmmmm
//пересчитываем чары в цифры
//сначала целые градусы
                        latitude = (incoming_message_buffer_gps[coma_places[2]+1] & 0x0F)*10 + (incoming_message_buffer_gps[coma_places[2]+2] & 0x0F);
                        longtitude = (incoming_message_buffer_gps[coma_places[4]+2] & 0x0F)*10 + (incoming_message_buffer_gps[coma_places[4]+3] & 0x0F);
//затем минуты
                        gps_data.latitude_d = ((incoming_message_buffer_gps[coma_places[2]+3] & 0x0F)*1000000 + (incoming_message_buffer_gps[coma_places[2]+4] & 0x0F)*(long)100000 + (incoming_message_buffer_gps[coma_places[2]+6] & 0x0F)*10000 + (incoming_message_buffer_gps[coma_places[2]+7] & 0x0F)*1000 + (incoming_message_buffer_gps[coma_places[2]+8] & 0x0F)*100 + (incoming_message_buffer_gps[coma_places[2]+9] & 0x0F)*10 + (incoming_message_buffer_gps[coma_places[2]+10] & 0x0F)) * 10 / 6;
                        gps_data.latitude_d += latitude * 10000000;
//формируем одну переменную типа 123.456789123
                        gps_data.longtitude_d = ((incoming_message_buffer_gps[coma_places[4]+4] & 0x0F)*1000000 + (incoming_message_buffer_gps[coma_places[4]+5] & 0x0F)*100000 + (incoming_message_buffer_gps[coma_places[4]+7] & 0x0F)*(long)10000 + (incoming_message_buffer_gps[coma_places[4]+8] & 0x0F)*1000 + (incoming_message_buffer_gps[coma_places[4]+9] & 0x0F)*100 + (incoming_message_buffer_gps[coma_places[4]+10] & 0x0F)*10 + (incoming_message_buffer_gps[coma_places[4]+11] & 0x0F)) * 10 / 6;
                        gps_data.longtitude_d += longtitude * 10000000;
                        ESP_LOGI(TAG_GPS,"Lat is  %ld, Lon is %ld", gps_data.latitude_d, gps_data.longtitude_d);
//статус ставим в 1 если считаем данные достоверными
                        gps_data.status = 1;
//отправляем сформированную структуру в очередь
                        xQueueSend(gps_to_main_queue, (void *) &gps_data, NULL);
//очищаем локальный буфер
                        for (i = 1;i < NUMBER_OF_BYTES_TO_RECEIVE_FROM_GPS; i++) incoming_message_buffer_gps[i] = 0;
                        }
                        else {ESP_LOGW(TAG_GPS,"Несовпадение режима"); gps_data.status = 0;}            //режим не А и не D
                      } else {ESP_LOGW(TAG_GPS, "Ошибка CRC"); gps_data.status = 0;}            //не сошлась контрольная сумма
                    } else {ESP_LOGW(TAG_GPS,"Сообщение не опознано"); gps_data.status = 0;}       //какое-то левое сообщение или запутались в длине сообщения
//если статус GPS поменялся по отношению к предыдущему, то отправляем соответствующую команду на RGB светодиод на Holybro
//обязательно через семафор, так как на этой шине есть еще устройства                 
                  
#ifdef USING_HOLYBRO_M9N
if (gps_data.status != gps_status_old )
                  {
                    if (gps_data.status)                                    //если статус 1 
                      {
                        if (xSemaphoreTake(semaphore_for_i2c_external, ( TickType_t ) 10) == pdTRUE)
                        {
                          FL3195_set_pattern(3, 0,255,0);                    //то цвет зеленый
                          xSemaphoreGive(semaphore_for_i2c_external);
                        }
                      }
                    
                    else                                                      //если статус 0 
                    {
                      if (xSemaphoreTake(semaphore_for_i2c_external, ( TickType_t ) 10) == pdTRUE)
                        {
                          FL3195_set_pattern(3, 255,0,0);                    //то цвет красный
                          xSemaphoreGive(semaphore_for_i2c_external);
                        }
                    }
                  }
                  gps_status_old = gps_data.status;
#endif                  
                  }
                uart_flush(GPS_UART);
                xQueueReset(gps_queue_for_events);

                break;
//остальные события UART, которые особо не интерсуют            
            case UART_DATA: break;

            case UART_FIFO_OVF:
              ESP_LOGW(TAG_RC, "hw fifo overflow");
              uart_flush_input(REMOTE_CONTROL_UART);
              xQueueReset(remote_control_queue_for_events);
              break;

            case UART_BUFFER_FULL:
              ESP_LOGW(TAG_RC, "ring buffer full");
              uart_flush_input(REMOTE_CONTROL_UART);
              xQueueReset(remote_control_queue_for_events);
              break;
        
            case UART_BREAK:
              ESP_LOGW(TAG_RC, "uart rx break");
              break;
            
            case UART_PARITY_ERR:
              ESP_LOGW(TAG_RC, "uart parity error");
              break;
            
            case UART_FRAME_ERR:
              ESP_LOGW(TAG_RC, "uart frame error");
              break;
                        
            default:
              ESP_LOGW(TAG_GPS, "unknown uart event type: %d", gps_uart_event.type);
              uart_flush(GPS_UART);
              xQueueReset(gps_queue_for_events);
                break;
            }
  }
}
}
#endif

//Задача получения и обработки данных от пульта управления. 
//Ждет прерывания по обнаружению символа конца строки, при обнаружении разбираем полученные данные, проводим их обработку и выдаем в очередь в сторону main_flying_cycle.
//Если есть команда на управление сервоприводами выдает команду на PCA9685. По завершении раз через 3 активируем задачу передачи телеметрии обратно на пульт
static void RC_read_and_process_data(void * pvParameters) 
{
  uint16_t i = 0;
  int16_t pos = 0;
  uint8_t remote_packets_counter = 0;
  uart_event_t remote_control_uart_event;
  uint8_t incoming_message_buffer_remote[NUMBER_OF_BYTES_TO_RECEIVE_FROM_RC * 2];
  uint16_t received_throttle = 0;
  uint16_t received_pitch = 2000;                       //neutral position
  uint16_t received_roll = 2000;
  uint16_t received_yaw = 2000;
  short trim_roll = -1;
  short trim_pitch = 2;
  struct data_from_rc_to_main_struct remote_control_data;
  
  float rc_throttle_old = 0;
  float rc_pitch_old = 0;
  float rc_roll_old = 0;
  float rc_yaw_old = 0;

  uint8_t set_allowed = 0;

  uint16_t command_for_PCA9685;
  uint16_t mode_old = 0;
  uint8_t LED_status = 0;
  uint8_t start_flag_old = 0;

  remote_control_data.altitude_hold_flag = 0;
  
  ESP_LOGI(TAG_RC,"Настраиваем UART для пульта управления.....");
  remote_control_uart_config();
  ESP_LOGI(TAG_RC,"UART для пульта управления настроен");

  while(1) 
  {
    if(xQueueReceive(remote_control_queue_for_events, (void * )&remote_control_uart_event, (TickType_t)portMAX_DELAY)) 
    {     
      switch (remote_control_uart_event.type)                   //проверяем тип события, полученного от UART 
      {
//если не включен пакет RSSI на приемнике - работаем по обнаружению символа конца строки
#ifdef NO_RSSI  
        case UART_PATTERN_DET:                                  //если это то что надо - 
          pos = uart_pattern_pop_pos(REMOTE_CONTROL_UART);      //запрашиваем на какой позиции в буфере обнаружен символ конца строки
          ESP_LOGD(TAG_RC, "[UART PATTERN DETECTED] pos: %d", pos);
          if (pos != (NUMBER_OF_BYTES_TO_RECEIVE_FROM_RC-1))    //если это не то место, где он должен быть - значит пакет поступил не полностью из-за какого-то сбоя, 
          {
            uart_flush_input(REMOTE_CONTROL_UART);              //очищаем входящий буфер и очередь  
            xQueueReset(remote_control_queue_for_events);
            ESP_LOGW(TAG_RC, "incorrect pos, %d", pos);  
          }
          else                                                 //если все ок     
          {                                                                                     
            int read_len = uart_read_bytes(REMOTE_CONTROL_UART, incoming_message_buffer_remote, pos+1, 1);      //считываем данные из буфера в локальную переменную
//если получаем байт RSSI - работам просто по UART_data
#else
        case UART_DATA:                                 
                  
          ESP_ERROR_CHECK(uart_get_buffered_data_len(REMOTE_CONTROL_UART, (size_t*)&pos));  //запрашиваем сколько байт получено
          ESP_LOGD(TAG_RC, "Получено %d байт по data", pos);

          if (pos != (NUMBER_OF_BYTES_TO_RECEIVE_FROM_RC + 1))                              //если не столько сколько должно быть
          {
            uart_flush_input(REMOTE_CONTROL_UART);              //очищаем входящий буфер и очередь  
            xQueueReset(remote_control_queue_for_events);
            ESP_LOGW(TAG_RC, "Некорректное кол-во байт, %d", pos);  
          }                    
          else                                                 //если все ок     
            {                                                                                
              int read_len = uart_read_bytes(REMOTE_CONTROL_UART, incoming_message_buffer_remote, pos, 0);  //считываем в локальный буфер    

#endif            
            ESP_LOGD(TAG_RC, "Received in total %d bytes", read_len);

            //проверяем покет на целостность по заголовку и контрольной сумме
            if ((incoming_message_buffer_remote[0] == RC_MESSAGE_HEADER) 
            && (incoming_message_buffer_remote[NUMBER_OF_BYTES_TO_RECEIVE_FROM_RC - 2] == dallas_crc8(incoming_message_buffer_remote, NUMBER_OF_BYTES_TO_RECEIVE_FROM_RC-2)))
            {
              ESP_LOGD(TAG_RC, "CRC passed");
//очищаем буфер
              uart_flush(REMOTE_CONTROL_UART);
//моргаем одним из светодиодов на плате
              if (LED_status) {gpio_set_level(LED_GREEN, 0); LED_status=0;}
              else {gpio_set_level(LED_GREEN, 1);LED_status=1;}
//собираем полученные данные в переменные
           
              received_throttle = (incoming_message_buffer_remote[1] << 8) | incoming_message_buffer_remote[2];
              received_roll = ((incoming_message_buffer_remote[3] << 8) | incoming_message_buffer_remote[4]);               
              received_pitch = ((incoming_message_buffer_remote[5] << 8) | incoming_message_buffer_remote[6]);                
              received_yaw = ((incoming_message_buffer_remote[7] << 8) | incoming_message_buffer_remote[8]);
              remote_control_data.mode = ~incoming_message_buffer_remote[10];
//начинаем приводить их к желаемому виду
              remote_control_data.received_throttle = 7836.0f + 48.98f * sqrt((double)received_throttle);

              if ((received_pitch > 360)&&( received_pitch < 1800)) remote_control_data.received_pitch = 0.02083333f*(float)received_pitch - 37.5f;
                else if ((received_pitch > 2245) && (received_pitch < 3741)) remote_control_data.received_pitch  = 0.02005348*(float)received_pitch - 45.020053f;
                else if (received_pitch >= 3741) remote_control_data.received_pitch  = 30.0;  //градусы наклона
                else if (received_pitch <= 360) remote_control_data.received_pitch  = -30.0;
                else remote_control_data.received_pitch = 0;
//reversed signs             
              if ((received_roll > 360)&&( received_roll < 1800)) remote_control_data.received_roll = -0.02083333f*(float)received_roll + 37.5f;              
                else if ((received_roll > 2245) && (received_roll < 3741)) remote_control_data.received_roll = -0.02005348*(float)received_roll + 45.020053f; 
                else if (received_roll >= 3741) remote_control_data.received_roll = -30.0;  //градусы наклона
                else if (received_roll <= 360) remote_control_data.received_roll = 30.0;
                else remote_control_data.received_roll = 0;

              if ((received_yaw > 200)&&( received_yaw < 1848)) remote_control_data.received_yaw = -0.0910194f*(float)received_yaw + 168.203885;
                else if ((received_yaw < 3896)&&( received_yaw > 2248)) remote_control_data.received_yaw = -0.0910194f*(float)received_yaw + 204.61165;
                else if (received_yaw >= 3896) remote_control_data.received_yaw = -150.0;     //градусы в секунду
                else if (received_yaw <= 200) remote_control_data.received_yaw = 150.0;
                else remote_control_data.received_yaw = 0;

//слегка подфильтровываем значения от джойстиков        
              remote_control_data.received_throttle = remote_control_data.received_throttle * RC_FILTER_COEFF + rc_throttle_old * (1 - RC_FILTER_COEFF);
              rc_throttle_old = remote_control_data.received_throttle;

              remote_control_data.received_pitch = remote_control_data.received_pitch * RC_FILTER_COEFF + rc_pitch_old * (1 - RC_FILTER_COEFF);
              rc_pitch_old = remote_control_data.received_pitch;

              remote_control_data.received_roll = remote_control_data.received_roll * RC_FILTER_COEFF + rc_roll_old * (1 - RC_FILTER_COEFF);
              rc_roll_old = remote_control_data.received_roll;

              remote_control_data.received_yaw = remote_control_data.received_yaw * RC_FILTER_COEFF + rc_yaw_old * (1 - RC_FILTER_COEFF);
              rc_yaw_old = remote_control_data.received_yaw;

//формируем значения trim (не используется далее, но пусть будет)
              if (incoming_message_buffer_remote[9] & 0b00001000) {                            //if roll negative values
              trim_roll = (((~(incoming_message_buffer_remote[9] & 0b00001111)) & 0b00001111) + 1) * -1;}
              else trim_roll = incoming_message_buffer_remote[9] & 0b00001111;

              if (incoming_message_buffer_remote[9] & 0b10000000) {                            //if pitch negative values
              trim_pitch = (((~((incoming_message_buffer_remote[9] >> 4) & 0b00001111)) & 0b00001111)  + 1) * -1;}
              else trim_pitch = (incoming_message_buffer_remote[9] >> 4) & 0b00001111; 

//если ручка газа в нижнем положении и включен тумблер "старт двигателей" установить флаг engines_start_flag        
              if ((remote_control_data.received_throttle < 8400) && (remote_control_data.mode & 0x0001)) remote_control_data.engines_start_flag = 1;
//если тумблер "старт двигателей" выключен - обнуляем бит engines_start_flag при любом положении ручки газа                
              if (!(remote_control_data.mode & 0x01)) {remote_control_data.engines_start_flag = 0; remote_control_data.altitude_hold_flag = 0; set_allowed = 0;}
//если  тумблер включен "удержание высоты" при запущенных двигателях - устанавливаем флаг altitude_hold_flag 
              if ((remote_control_data.mode & 0x02) && (set_allowed == 1) && (remote_control_data.engines_start_flag)) remote_control_data.altitude_hold_flag = 1;
//в противном случае обнуляем этот флаг              
              if (!(remote_control_data.mode & 0x02)) {remote_control_data.altitude_hold_flag = 0; set_allowed = 1;}
//при изменении положения тумблера 4 отправить в очередь задачи управления PCA9685 команду на изменение сигнала
              if ((remote_control_data.mode & 0x08) ^ (mode_old & 0x08))  {
                if (remote_control_data.mode & 0x08) command_for_PCA9685 = 0x0103;
                else command_for_PCA9685 = 0x011A;
              xQueueSend(PCA9685_queue, &command_for_PCA9685, NULL);
              }
//при изменении состояния флага engine_start отправить в очередь задачи управления PCA9685 команду на изменение сигнала
              if (remote_control_data.engines_start_flag ^ start_flag_old)  {
                if (remote_control_data.engines_start_flag) command_for_PCA9685 = 0x015A;
                else command_for_PCA9685 = 0x0100;
              xQueueSend(PCA9685_queue, &command_for_PCA9685, NULL);
              }
//копируем в структуру байт с уровнем RSSI 
              remote_control_data.rssi_level = (int8_t)incoming_message_buffer_remote[13];
//отправляем сформированную структуру с обработанными данными от пульта в main_flying_cycle                
              xQueueSend(remote_control_to_main_queue, (void *) &remote_control_data, NULL);
//обнуляем на всякий случай буфер
              for (i=0;i<NUMBER_OF_BYTES_TO_RECEIVE_FROM_RC * 2;i++) incoming_message_buffer_remote[i] = 0;           
              
              mode_old = remote_control_data.mode;
              start_flag_old = remote_control_data.engines_start_flag;
//каждый 3й раз пробуждаем задачу отправки телеметрии на пульт
              remote_packets_counter++;
              if (remote_packets_counter == 3)
              {
                remote_packets_counter = 0;
                xTaskNotifyGive(task_handle_send_data_to_RC);        
              }  
            }
            else 
            { 
              ESP_LOGW(TAG_RC, "CRC failed");
              //for (j=0;j<13;j++) printf ("%02x ",incoming_message_buffer_remote[j]);       
              uart_flush(REMOTE_CONTROL_UART);
              xQueueReset(remote_control_queue_for_events);
            }  
          }
          break;

        case UART_FIFO_OVF:
          ESP_LOGW(TAG_RC, "hw fifo overflow");
          uart_flush_input(REMOTE_CONTROL_UART);
          xQueueReset(remote_control_queue_for_events);
          break;

        case UART_BUFFER_FULL:
          ESP_LOGW(TAG_RC, "ring buffer full");
          uart_flush_input(REMOTE_CONTROL_UART);
          xQueueReset(remote_control_queue_for_events);
          break;
#ifdef NO_RSSI
        case UART_DATA: 
          ESP_LOGW(TAG_RC, "data");
        break;
#else
        case UART_PATTERN_DET: 
        ESP_LOGW(TAG_RC, "pattern");
        break;
#endif        

case UART_BREAK:
          ESP_LOGW(TAG_RC, "uart rx break");
          uart_flush(REMOTE_CONTROL_UART);
          break;
        
        case UART_PARITY_ERR:
          ESP_LOGW(TAG_RC, "uart parity error");
          break;
        
        case UART_FRAME_ERR:
          ESP_LOGW(TAG_RC, "uart frame error");
          break;

        default:
          ESP_LOGW(TAG_RC, "unknown uart event type: %d", remote_control_uart_event.type);
          uart_flush(REMOTE_CONTROL_UART);
          xQueueReset(remote_control_queue_for_events);
          break;
      }
    }
  }
} 

//Задача отправки телеметрии на управления. 
//Активируется из задачи получения данных от пульта, забирает из очереди, в которую шлет main_flying_cycle, данные и отправляем их в UART.
static void send_data_to_RC(void * pvParameters)
{
  struct data_from_main_to_rc_struct data_to_send_to_rc;
  uint8_t outcoming_message_buffer_remote[NUMBER_OF_BYTES_TO_SEND_TO_RC];
  uint8_t LED_status = 0;

  while(1) 
  {
    if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) != 0)
    {
      if (xQueueReceive(main_to_rc_queue, &data_to_send_to_rc, (TickType_t)portMAX_DELAY)) 
      {
        outcoming_message_buffer_remote[0] = RC_MESSAGE_HEADER;
        outcoming_message_buffer_remote[1] = 0; //reserved
        outcoming_message_buffer_remote[2] = 0; //reserved
        outcoming_message_buffer_remote[3] = 0; //reserved
        outcoming_message_buffer_remote[4] = ((int8_t)(data_to_send_to_rc.pitch) & 0xFF00) >> 8;                          //angles in format MSB + LSB    pitch
        outcoming_message_buffer_remote[5] = ((int8_t)(data_to_send_to_rc.pitch) & 0x00FF);
        outcoming_message_buffer_remote[6] = (uint8_t)(((uint16_t)(data_to_send_to_rc.yaw) & 0xFF00) >> 8);                                   //yaw
        outcoming_message_buffer_remote[7] = (uint8_t)((uint16_t)(data_to_send_to_rc.yaw) & 0x00FF);
        outcoming_message_buffer_remote[8] = ((int8_t)(data_to_send_to_rc.roll) & 0xFF00) >> 8;
        outcoming_message_buffer_remote[9] = ((int8_t)(data_to_send_to_rc.roll) & 0x00FF);
        outcoming_message_buffer_remote[10] = (data_to_send_to_rc.power_voltage_value & 0xFF00) >> 8;                  //ADC_power_value
        outcoming_message_buffer_remote[11] = data_to_send_to_rc.power_voltage_value & 0x00FF;
        outcoming_message_buffer_remote[12] = (data_to_send_to_rc.altitude & 0xFF00) >> 8; 
        outcoming_message_buffer_remote[13] = data_to_send_to_rc.altitude & 0x00FF; 
        outcoming_message_buffer_remote[14] = dallas_crc8(outcoming_message_buffer_remote, (NUMBER_OF_BYTES_TO_SEND_TO_RC - 1));
        
        uart_write_bytes(REMOTE_CONTROL_UART, outcoming_message_buffer_remote, NUMBER_OF_BYTES_TO_SEND_TO_RC);
        if (LED_status) {gpio_set_level(LED_BLUE, 0); LED_status=0;}
              else {gpio_set_level(LED_BLUE, 1);LED_status=1;}
      }                       
    } 
  }
}

//Задача обработки данных от лидара TFSMini (Benewake)
//ждет прерывания от обнаружения символа начала строки, считывает данные, обрабатывает и отправляем в main_flying_cycle 
//опционально можем настраивать частоту выдачи данных с TFSmini 
#ifdef USING_TFMINIS_I2C 
static void lidar_read_and_process_data(void * pvParameters)
{
  uint8_t incoming_message_buffer_lidar[NUMBER_OF_BYTES_TO_RECEIVE_FROM_LIDAR];
  uint16_t i = 0;
  uint8_t sum = 0;
  uint16_t raw_height = 0;
  uint16_t raw_height_old = 0;
  uint16_t raw_strength = 0;
  struct data_from_lidar_to_main_struct lidar_data;
  float height_old = 0;
  float height_filter_pool[5] = {0};
  float vertical_velocity_filter_pool[3] = {0.1};

  while(1) {

    if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) != 0)
    {
      ESP_LOGD(TAG_LIDAR,"Отправляем запрос на получение данных");
      xSemaphoreTake(semaphore_for_i2c_internal,portMAX_DELAY); 
      tfs_request_data();
      //xSemaphoreGive(semaphore_for_i2c_internal);                                 //тут должна быть задержка 1ms но вроде работает и без нее
      //vTaskDelay(1/portTICK_PERIOD_MS);     //esp_rom_delay_us(uint32_t us)       //It is recommended to wait for 1ms and then read the result after sending commands.
      //ESP_LOGD(TAG_LIDAR,"Считываем результат");
      //xSemaphoreTake(semaphore_for_i2c_internal,portMAX_DELAY);  
      tfs_read_result(incoming_message_buffer_lidar);
      xSemaphoreGive(semaphore_for_i2c_internal);
      sum = 0;
      for (i=0;i<8;i++) sum+= incoming_message_buffer_lidar[i];
          
      if ((incoming_message_buffer_lidar[0] == 0x59) && (incoming_message_buffer_lidar[1] == 0x59) && (incoming_message_buffer_lidar[8] == sum))
      {
        raw_height = (incoming_message_buffer_lidar[3] << 8) + incoming_message_buffer_lidar[2];
        raw_strength = (incoming_message_buffer_lidar[5] << 8) + incoming_message_buffer_lidar[4];
    
        if (((raw_height - raw_height_old) < 2) && ((raw_height  - raw_height_old) > -2)) raw_height = raw_height_old;                  //так как типичный джиттер сигнала = 2см
        lidar_data.altitude = avg_filter_1d(height_filter_pool, (float)raw_height, sizeof(height_filter_pool) / sizeof(float));         //фильтруем скользящим средним

        lidar_data.vertical_velocity = (lidar_data.altitude - height_old) / (1.0 / (float)LIDAR_RATE_HZ);       //на основе фильтрованной высоты вычислем скорость в см/с
        
        height_old = lidar_data.altitude;

        //printf("%0.3f, %0.3f\n",lidar_data.altitude,lidar_data.vertical_velocity);
        //printf("%d\n",raw_height);

        if ((raw_strength < 100) || (raw_height > 65532)) lidar_data.valid = 0;
        else lidar_data.valid = 1;

        xQueueSend(lidar_to_main_queue, (void *) &lidar_data, NULL);
        height_old = lidar_data.altitude;
      }
      else if (incoming_message_buffer_lidar[8] != sum) ESP_LOGW(TAG_LIDAR,"Ошибка CRC, расчетный CRC = %d, принятый %d", sum, incoming_message_buffer_lidar[8]);
           else ESP_LOGW(TAG_LIDAR,"Ошибка заголовка (%02x, %02x)", incoming_message_buffer_lidar[0], incoming_message_buffer_lidar[1]); 
    }   
  }
}
#endif



//Задача работы с расширителем портов ввода-вывода MCP23017
//принимает через очередь команду (либо считывания входов, либо управление выходами) и выполняет запрошенную команду
//формат команды
// - 0b10000000  - считывание состояния входов
// - 0b010000xx - установить в 1 выход xx
// - 0b001000xx - установить в 0 выход xx
static void MCP23017_monitoring_and_control(void * pvParameters)
{
  uint8_t MCP23017_external_request;
  uint8_t MCP23017_inputs_state;
  uint8_t MCP23017_current_outputs_state;
    
  while(1) {
    if(xQueueReceive(MCP23017_queue, &MCP23017_external_request, (TickType_t)portMAX_DELAY))
    {
      ESP_LOGI(TAG_MCP23017,"Received request %02x",MCP23017_external_request); 

      if (MCP23017_external_request == 0b10000000) {                //command to read inputs
        xSemaphoreTake(semaphore_for_i2c_internal,portMAX_DELAY);
        MCP23017_inputs_state = MCP23017_get_inputs_state();
        xSemaphoreGive(semaphore_for_i2c_internal); 
        ESP_LOGI(TAG_MCP23017,"Current input state is %02x",MCP23017_inputs_state);  
      }
      
      if (MCP23017_external_request & 0b01000000) {                 //if bit6 is set that means 2 lower bit represents output to be set
        xSemaphoreTake(semaphore_for_i2c_internal,portMAX_DELAY);
        MCP23017_set_output(&MCP23017_current_outputs_state, MCP23017_external_request & 0b00001111);
        xSemaphoreGive(semaphore_for_i2c_internal);
        ESP_LOGI(TAG_MCP23017,"Got request to set output %d, output is set",MCP23017_external_request & 0b00001111);  
      }

      if (MCP23017_external_request & 0b00100000) {                 //if bit5 is set that means 2 lower bit represents output to be cleared
        xSemaphoreTake(semaphore_for_i2c_internal,portMAX_DELAY);
        MCP23017_clear_output(&MCP23017_current_outputs_state, MCP23017_external_request & 0b00001111);
        xSemaphoreGive(semaphore_for_i2c_internal);
        ESP_LOGI(TAG_MCP23017,"Got request to clear output %d, output is cleared",MCP23017_external_request & 0b00001111);
      }
    }
  }
}

//Задача управления ШИМ-драйвером PCA9685
//Принимает на вход команду управления (номер выхода + необходимая скважность сигнала) и выполняет ее
//Формат команды - 1 байт, старшие 4 бита - номер выхода, младшие - заполнение в процентах
//например, 0x0103 - выход №1 на 3%
static void PCA9685_control(void * pvParameters)
{
    uint16_t PCA9685_input_command;
    uint8_t pwm_value;
    uint8_t output_number;
    
  while(1) {
    if(xQueueReceive(PCA9685_queue, &PCA9685_input_command, (TickType_t)portMAX_DELAY))
    {
      if (PCA9685_input_command >= 0x1000) ESP_LOGE(TAG_PCA9685,"Received wrong request %04x",PCA9685_input_command);
      else 
      {
        ESP_LOGI(TAG_PCA9685,"Received request %04x",PCA9685_input_command);
        pwm_value = PCA9685_input_command & 0x00FF;
        output_number = (PCA9685_input_command >> 8) & 0x00FF;
        xSemaphoreTake (semaphore_for_i2c_internal,portMAX_DELAY);
        PCA9685_send(pwm_value, output_number);
        xSemaphoreGive (semaphore_for_i2c_internal);
        ESP_LOGI(TAG_PCA9685,"Output #%d set to %d%%",output_number, pwm_value); 
      }  
    }
  }
}


#ifdef USING_W25N
//Задача записи логов во внешнюю flash-память
//Принимает на вход данные на запись из main_flyibg_cycle и записывает их в Winbond 
static void writing_logs_to_flash(void * pvParameters)
{
  struct logging_data_set* buffer;
  uint16_t column_address = 0;
  uint16_t page_address = 0;
  while(1) 
  {
    if (xQueueReceive(W25N01_queue, &buffer, portMAX_DELAY))
    {
      //for (int i = 0; i<sizeof(struct logging_data_set);i++) printf("%d",buffer[i]);
      W25N_random_program_data_load(column_address, (uint8_t*)buffer, sizeof(struct logging_data_set));   //загружаем пакет данных в буфер начиная с column_address
      column_address = column_address + sizeof(struct logging_data_set);                        //увеличиваем colunm_address на размер пакета данных 
      if (column_address >= (2048-sizeof(struct logging_data_set)))   //проверяем что в буфер (на эту страницу) еще что-то влезет [79 байт за 1мс, 25 пакетов на странице, 79*25 = 1975]
      {                             //если буфер заполнен - записываем страницу и инкрементируем адрес страницы
        column_address = 0;
        W25N_program_execute(page_address);       //65536 pages, итого на 26 минут макс
        page_address++;
      }
    if (page_address == 65535)                    //если страницы закончились
    {
      ESP_LOGE(TAG_W25N,"Внешняя flash-память для логов переполнена, запись останавливается\n");
      vTaskDelete(NULL);                          //прекращаем запись
    } 
    }
  }
}

//Задача считывания данных из внешней flash. Активируется только если обнаруживается установленная перемычка "считать логи."
//Получает данные с памяти и выдает их в UART
static void reading_logs_from_external_flash(void * pvParameters)
{
  while(1) {
    W25N_read_and_print_all();
    while (1) 
    {
      gpio_set_level(LED_RED, 1);
      gpio_set_level(LED_GREEN, 1);
      gpio_set_level(LED_BLUE, 1);
      vTaskDelay(500/portTICK_PERIOD_MS); 
      gpio_set_level(LED_RED, 0);
      gpio_set_level(LED_GREEN, 0);
      gpio_set_level(LED_BLUE, 0);
      vTaskDelay(500/portTICK_PERIOD_MS); 
    }; 
  }
}
#endif

//Задача управления полетными огнями
//В зависимости от состояния мограет полетными огнями с разной задержкой
static void blinking_flight_lights(void * pvParameters)
{
  uint32_t blinking_mode = 0;

  while(1) 
  {
    xTaskNotifyWait(0,0,&blinking_mode,NULL);
  
    if (blinking_mode == 0)                 //аварийный режим (1 зеленый 1 красный)
    {
      //gpio_set_level(GREEN_FLIGHT_LIGHTS, 1);
      //gpio_set_level(RED_FLIGHT_LIGHTS, 1);
      vTaskDelay(50/portTICK_PERIOD_MS);
      
      gpio_set_level(GREEN_FLIGHT_LIGHTS, 0);
      gpio_set_level(RED_FLIGHT_LIGHTS, 0);
      vTaskDelay(250/portTICK_PERIOD_MS);
    }

    if (blinking_mode == 1)               //штатный режим (2 зеленых 1 красный)
    {
      //gpio_set_level(GREEN_FLIGHT_LIGHTS, 1);
      vTaskDelay(50/portTICK_PERIOD_MS);
      gpio_set_level(GREEN_FLIGHT_LIGHTS, 0);
      vTaskDelay(100/portTICK_PERIOD_MS);
      //gpio_set_level(GREEN_FLIGHT_LIGHTS, 1);
      vTaskDelay(50/portTICK_PERIOD_MS);
      gpio_set_level(GREEN_FLIGHT_LIGHTS, 0);
      vTaskDelay(500/portTICK_PERIOD_MS);

      //gpio_set_level(RED_FLIGHT_LIGHTS, 1);
      vTaskDelay(50/portTICK_PERIOD_MS);
      gpio_set_level(RED_FLIGHT_LIGHTS, 0);
      vTaskDelay(750/portTICK_PERIOD_MS);
    } 
  }
}




//Задача печати статистики загруженности системы.
//Получает данные vTaskGetRunTimeStats и петатает их раз в 5 секунд. Только для режима диагностики.
#ifdef USING_PERFORMANCE_MESUREMENT
static void performace_monitor(void * pvParameters)
{
   char statbuf[800];
  
  while(1)
  {
    vTaskGetRunTimeStats(statbuf);
    printf("%s\n",statbuf);
    vTaskDelay(5000/portTICK_PERIOD_MS);
  } 
}
#endif


//Задача считывания данных с INA219 (мониторинг тока и напряжений)
//Активируется из main_flying_cycle, возвращает через очередь данные о напряжении АКБ, токе, мощности и затраченной энергии
static void INA219_read_and_process_data(void * pvParameters)
{
  float INA219_data[4] = {0.0, 0.0, 0.0, 0.0};  // volt, current, power, consumed
  int64_t prev_time = 0;
  int64_t current_time = 0;
  
  while(1) 
  {
      if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) != 0)
      {
      INA219_data[0] = INA219_read_voltage();
      INA219_data[1] = INA219_read_current();
      INA219_data[2] = INA219_read_power();
      current_time = get_time();
      INA219_data[3] += (INA219_data[2] * ((float)(current_time - prev_time) / 1000000L)) * (1000.0 / 11.1) / 3600.0; //consumed energy in mA*h
      
      ESP_LOGI(TAG_INA219, "V: %0.3fV, I: %0.3fA, P: %0.3fW, A: %0.3fmAh",INA219_data[0], INA219_data[1], INA219_data[2], INA219_data[3]);
      
      prev_time = current_time;
      xQueueSend(INA219_to_main_queue, (void *) INA219_data, NULL); 
      }
  }
}

#ifdef USING_MAVLINK_TELEMETRY
//Задача отправки телеметрии по mavlink. 
//Забирает из очереди, в которую шлет main_flying_cycle, данные и отправляем их в mavlink UART.
static void send_telemetry_via_mavlink(void * pvParameters)
{
  mavlink_data_set_t* p_to_mavlink_data;
  static mavlink_heartbeat_t heartbeat;
  static mavlink_sys_status_t sys_status;
  static mavlink_radio_status_t rssi;
  static mavlink_attitude_t attitude;
  static mavlink_battery_status_t battery;
  static mavlink_global_position_int_t gps;
  static mavlink_gps_raw_int_t gps_raw;
  static mavlink_rc_channels_raw_t rc_channels;
  static mavlink_vfr_hud_t vfr_hud;
  uint8_t seq = 0;

  ESP_LOGI(TAG_MAV, "Настраиваем UART для Mavlink......");
  mavlink_uart_config();
  ESP_LOGI(TAG_MAV, "UART для Mavlink настроен");

  //инициализируем постоянные компоненты каждого из типов заголовков
  //для hearbeat
  heartbeat.header.magic = MAVLINK_V1_MESSAGE_HEADER;
  heartbeat.header.len = MAVLINK_MSG_ID_HEARTBEAT_LEN;
  heartbeat.header.sysid = MAVLINK_MY_SYS_ID;
  heartbeat.header.compid = MAVLINK_MY_COMP_ID;
  heartbeat.header.msgid = MAVLINK_MSG_ID_HEARTBEAT;
  heartbeat.type = 6;
  heartbeat.autopilot = 8;
  heartbeat.type = 2;            //MAV_TYPE_QUADROTOR
  heartbeat.autopilot = 3;       //MAV_AUTOPILOT_ARDUPILOTMEGA 3
  heartbeat.mavlink_version = 3;
  
 //для status
  sys_status.header.magic = MAVLINK_V1_MESSAGE_HEADER;
  sys_status.header.len = MAVLINK_MSG_ID_SYS_STATUS_LEN;
  sys_status.header.sysid = MAVLINK_MY_SYS_ID;
  sys_status.header.compid = MAVLINK_MY_COMP_ID;
  sys_status.header.msgid = MAVLINK_MSG_ID_SYS_STATUS;
  
  //для RSSI
  rssi.header.magic = MAVLINK_V1_MESSAGE_HEADER;
  rssi.header.len = MAVLINK_MSG_ID_RADIO_STATUS_LEN;
  rssi.header.sysid = MAVLINK_MY_SYS_ID;
  rssi.header.compid = MAVLINK_MY_COMP_ID;
  rssi.header.msgid = MAVLINK_MSG_ID_RADIO_STATUS;

  //для ATTITUDE
  attitude.header.magic = MAVLINK_V1_MESSAGE_HEADER;
  attitude.header.len = MAVLINK_MSG_ID_ATTITUDE_LEN;
  attitude.header.sysid = MAVLINK_MY_SYS_ID;
  attitude.header.compid = MAVLINK_MY_COMP_ID;
  attitude.header.msgid = MAVLINK_MSG_ID_ATTITUDE;
  
  //для BATTERY
  battery.header.magic = MAVLINK_V1_MESSAGE_HEADER;
  battery.header.len = MAVLINK_MSG_ID_BATTERY_STATUS_LEN;
  battery.header.sysid = MAVLINK_MY_SYS_ID;
  battery.header.compid = MAVLINK_MY_COMP_ID;
  battery.header.msgid = MAVLINK_MSG_ID_BATTERY_STATUS;  
  
  //для GPS
  gps.header.magic = MAVLINK_V1_MESSAGE_HEADER;
  gps.header.len = MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN;
  gps.header.sysid = MAVLINK_MY_SYS_ID;
  gps.header.compid = MAVLINK_MY_COMP_ID;
  gps.header.msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;

  //для GPS_RAW
  gps_raw.header.magic = MAVLINK_V1_MESSAGE_HEADER;
  gps_raw.header.len = MAVLINK_MSG_ID_GPS_RAW_INT_LEN;
  gps_raw.header.sysid = MAVLINK_MY_SYS_ID;
  gps_raw.header.compid = MAVLINK_MY_COMP_ID;
  gps_raw.header.msgid = MAVLINK_MSG_ID_GPS_RAW_INT;
 
  //для RC_CHANNELS
  rc_channels.header.magic = MAVLINK_V1_MESSAGE_HEADER;
  rc_channels.header.len = MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN;
  rc_channels.header.sysid = MAVLINK_MY_SYS_ID;
  rc_channels.header.compid = MAVLINK_MY_COMP_ID;
  rc_channels.header.msgid = MAVLINK_MSG_ID_RC_CHANNELS_RAW;

  //для VFR_HUD
  vfr_hud.header.magic = MAVLINK_V1_MESSAGE_HEADER;
  vfr_hud.header.len = MAVLINK_MSG_ID_VFR_HUD_LEN;
  vfr_hud.header.sysid = MAVLINK_MY_SYS_ID;
  vfr_hud.header.compid = MAVLINK_MY_COMP_ID;
  vfr_hud.header.msgid = MAVLINK_MSG_ID_VFR_HUD;
  
 while(1)
  {
    if (xQueueReceive(main_to_mavlink_queue, &p_to_mavlink_data, portMAX_DELAY))
    {
    //копируем данные, полученные из очереди, в структуры mavlink
        if (p_to_mavlink_data -> armed_status) heartbeat.base_mode = 128;             // https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG
        else heartbeat.base_mode = 0;                                          
    
        sys_status.voltage_battery = p_to_mavlink_data -> voltage_mv;
        sys_status.current_battery = p_to_mavlink_data -> current_ca;
        sys_status.battery_remaining = (int8_t)(0.037 * (p_to_mavlink_data -> voltage_mv) - 366.3);      //конвертируем линейной зависимостью [9.9..12.6]В в [0 - 100]%
        
        attitude.roll = p_to_mavlink_data -> angles[1] * PI / 180;
        attitude.pitch = p_to_mavlink_data -> angles[0] * PI / 180;
        attitude.yaw = p_to_mavlink_data -> angles[2] * PI / 180;
        //printf("%f\n", attitude.yaw);

        gps_raw.lat = p_to_mavlink_data -> latitude;
        gps_raw.lon = p_to_mavlink_data -> longtitude;
        gps_raw.fix_type = p_to_mavlink_data -> gps_status;

        rc_channels.rssi = (p_to_mavlink_data -> rssi_level) * 2.55 + 255;     //преобразуем сигнал [-100..0] в [0..255]

        vfr_hud.heading = p_to_mavlink_data -> angles[2]; //heading
        vfr_hud.heading = 33; //heading
        vfr_hud.alt = (p_to_mavlink_data -> altitude_cm / 100); //высота в метрах

                
        rssi.remrssi = p_to_mavlink_data -> rssi_level;
        rssi.rssi = p_to_mavlink_data -> rssi_level;

        battery.voltages[0] = p_to_mavlink_data -> voltage_mv;
        battery.current_consumed = p_to_mavlink_data -> current_ca;
        battery.current_battery = p_to_mavlink_data -> current_ca;
        battery.battery_remaining = sys_status.battery_remaining;        //так как это параметр отсылается в нескольких сообщениях

//Отправляем в UART требуемые сообщения        
        prepare_heartbeat(&heartbeat, &seq);
        uart_write_bytes(MAV_UART, &heartbeat, sizeof(mavlink_heartbeat_t));
        //for (int i = 0; i<sizeof(mavlink_heartbeat_t);i++) printf ("%02x ", (uint8_t) *((uint8_t*)&heartbeat + i));

        prepare_status(&sys_status, &seq);
        uart_write_bytes(MAV_UART, &sys_status, sizeof(mavlink_sys_status_t));

        prepare_attitude(&attitude, &seq);
        uart_write_bytes(MAV_UART, &attitude, sizeof(mavlink_attitude_t));  

        prepare_gps_raw(&gps_raw, &seq);
        uart_write_bytes(MAV_UART, &gps_raw, sizeof(mavlink_gps_raw_int_t));

        prepare_rc_channels(&rc_channels, &seq);
        uart_write_bytes(MAV_UART, &rc_channels, sizeof(mavlink_rc_channels_raw_t));

        prepare_vfr_hud(&vfr_hud, &seq);
        uart_write_bytes(MAV_UART, &vfr_hud, sizeof(mavlink_vfr_hud_t));

        prepare_gps(&gps, &seq);
        uart_write_bytes(MAV_UART, &gps, sizeof(mavlink_global_position_int_t)); 
/*        
        prepare_rssi( &rssi, &seq);
        uart_write_bytes(MAV_UART, &rssi, sizeof(mavlink_radio_status_t));
        
        prepare_battery(&battery, &seq);
        uart_write_bytes(MAV_UART, &battery, sizeof(mavlink_battery_status_t));
*/ 
    }
  }
}

#endif


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
static void main_flying_cycle(void * pvParameters)
{
  extern volatile float q0;
  extern volatile float q1;
  extern volatile float q2;
  extern volatile float q3;
  
  uint32_t IMU_interrupt_status = 0;
  uint8_t test_for_all_0 = 0;
  uint8_t test_for_all_1 = 0xFF;
  
  uint8_t sensor_data_1[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
  uint8_t sensor_data_2[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};

  int16_t accel_raw_1[3] = {0,0,0}; 
  int16_t gyro_raw_1[3] = {0,0,0}; 
  int16_t accel_raw_2[3] = {0,0,0}; 
  int16_t gyro_raw_2[3] = {0,0,0};

  uint8_t calibration_flag = 0;
  float number_of_IMU_calibration_counts = NUMBER_OF_IMU_CALIBRATION_COUNTS; 
          
  float Gyro_X_cal_1 = 0.0;
  float Gyro_Y_cal_1 = 0.0;
  float Gyro_Z_cal_1 = 0.0;
  float Accel_X_cal_1 = 0.0;
  float Accel_Y_cal_1 = 0.0;
  float Accel_Z_cal_1 = 0.0;
  
  float Gyro_X_cal_2 = 0.0;
  float Gyro_Y_cal_2 = 0.0;
  float Gyro_Z_cal_2 = 0.0;
  float Accel_X_cal_2 = 0.0;
  float Accel_Y_cal_2 = 0.0;
  float Accel_Z_cal_2 = 0.0;

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

#ifdef USING_HOLYBRO_M9N  
  const uint8_t madgwick_cycles = 1;
#else 
  const uint8_t  madgwick_cycles = 5;
#endif

  static float pitch = 0; 
  static float roll = 0; 
  static float yaw = 0; 

// for cascaded PID
  float Kp_pitch_angle = 4.4;
  float Ki_pitch_angle = 0.0004;
  float Kd_pitch_angle = 10;
  float Kp_pitch_rate = 8.8;
  float Ki_pitch_rate = 0.03;
  float Kd_pitch_rate = 860;
  float diff_pitch_error_rate = 0;
  float error_pitch_rate = 0;
  float error_pitch_angle = 0;
  float error_pitch_angle_old = 0;
  float error_pitch_rate_old = 0;
  float integral_pitch_error_rate = 0;
  float integral_pitch_error_angle = 0;
  float pid_pitch_angle = 0;
  float pid_pitch_rate = 0;
  float gyro_pitch = 0;
  float gyro_pitch_old = 0;

  float Kp_roll_angle = 4.2;
  float Ki_roll_angle = 0.0004;
  float Kd_roll_angle = 8;
  float Kp_roll_rate = 8.8;
  float Ki_roll_rate = 0.03;
  float Kd_roll_rate = 860;
  float diff_roll_error_rate = 0;
  float error_roll_rate = 0;
  float error_roll_angle = 0;
  float error_roll_angle_old = 0;
  float error_roll_rate_old = 0;
  float integral_roll_error_rate = 0;
  float integral_roll_error_angle = 0;
  float pid_roll_angle = 0;
  float pid_roll_rate = 0;
  float gyro_roll = 0;
  float gyro_roll_old = 0;

  float yaw_setpoint = 0.0;
  float Kp_yaw_angle = 3;
  float Kd_yaw_angle = 10;
  float Ki_yaw_angle = 0.001;
  float integral_yaw_error_angle = 0;
  float Kp_yaw_rate = 100;
  float Ki_yaw_rate = 0.01;
  float Kd_yaw_rate = 0;
  float error_yaw_rate = 0;
  float error_yaw_angle = 0;
  float error_yaw_angle_old = 0;
  float error_yaw_rate_old = 0;
  float integral_yaw_error_rate = 0;
  float pid_yaw_angle = 0;
  float pid_yaw_rate = 0;
  float diff_yaw_error_rate = 0.0;

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

#ifdef USING_HOLYBRO_M9N
  float mag_fresh_data[3];
#endif 

//переменные по режиму удержания высоты
  bool altitude_hold_mode_enabled = 0;
  float altitude_setpoint = 0;
  float old_altitude = 0;
  //float count_altitude = 0;
  float error_altitude = 0;
  float previous_error_altitude = 0;
  float Kp_alt = 15;              //30 (на БП)
  float Kd_alt = 180.0;           //400 (на БП)
  float Ki_alt = 0.055;            //0.05 (на БП)
  float pid_altitude = 0;
  float integral_alt_error = 0;
  float alt_hold_initial_throttle = 0;
  float alt_error_change_filter_pool[7] = {0};//ALT_PID_DIF_FILTER_LENGTH


#ifdef USING_TFMINIS_I2C 
    struct data_from_lidar_to_main_struct lidar_fresh_data;
      lidar_fresh_data.altitude = 0;
      lidar_fresh_data.vertical_velocity = 0;
      lidar_fresh_data.valid = 0;
#endif

  float INA219_fresh_data[4];
  float voltage_correction_coeff = 1.0;
  
#ifdef USING_W25N 
  //переменные для логирования
  //static uint8_t logs_buffer[LOGS_BYTES_PER_STRING] = {0x0D}; 
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

  uint8_t long_cycle_flag = 0;

  void Convert_Q_to_degrees(void) {
    
    float a12,a22,a31,a32,a33 = 0;

    a12 =   2.0f * (q1 * q2 + q0 * q3);
    a22 =   q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    a31 =   2.0f * (q0 * q1 + q2 * q3);
    a32 =   2.0f * (q1 * q3 - q0 * q2);
    a33 =   q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    
    pitch = asin(a32) * 57.29577951;
    
    roll = atan2(a31, a33) * 57.29577951;
    roll += 180.0;
    if (roll > 90) roll -= 360.0;
    
    yaw = atan2(a12, a22) * 57.29577951;
    yaw -= 9.4; // компенсация наклонения при использовании магнетометра. Если магнетометр не используем - без разницы, вреда не наносит

  }

  void calculate_pids_2(void) {
//outer (angle) pitch cycle
    error_pitch_angle = rc_fresh_data.received_pitch - pitch;    
    integral_pitch_error_angle = integral_pitch_error_angle + Ki_pitch_angle * error_pitch_angle;
    if (integral_pitch_error_angle > 1000.0) integral_pitch_error_angle = 1000.0; 
    if (integral_pitch_error_angle < -1000.0) integral_pitch_error_angle =-1000.0;
    if (rc_fresh_data.received_throttle < 9000.0) integral_pitch_error_angle = 0;              //to avoid accumulation on the ground
    pid_pitch_angle = Kp_pitch_angle * error_pitch_angle + integral_pitch_error_angle + Kd_pitch_angle * (error_pitch_angle - error_pitch_angle_old);     
    if (pid_pitch_angle > 2000.0) {pid_pitch_angle = 2000.0; }
    if (pid_pitch_angle < -2000.0) {pid_pitch_angle = -2000.0; }
    error_pitch_angle_old = error_pitch_angle; 
  
//inner (rate) pitch cycle
    gyro_pitch = 0.1 * (((gyro_converted_1[0] + gyro_converted_2[1]) / 2.0) * (180.0 / (float)PI)) + 0.9 * gyro_pitch_old;
    gyro_pitch_old = gyro_pitch;

    error_pitch_rate = pid_pitch_angle - gyro_pitch;
    //error_pitch_rate = rc_fresh_data.received_pitch - gyro_pitch;         //for inner loop setup
    integral_pitch_error_rate = integral_pitch_error_rate + Ki_pitch_rate * error_pitch_rate;
    if (integral_pitch_error_rate > 1000.0) integral_pitch_error_rate = 1000.0; 
    if (integral_pitch_error_rate < -1000.0) integral_pitch_error_rate = -1000.0;
    diff_pitch_error_rate = Kd_pitch_rate * (error_pitch_rate - error_pitch_rate_old);
    if (rc_fresh_data.received_throttle < 9000.0) {integral_pitch_error_rate = 0; diff_pitch_error_rate = 0;}             //to avoid accumulation on the ground 
    pid_pitch_rate = Kp_pitch_rate * error_pitch_rate + integral_pitch_error_rate + diff_pitch_error_rate;

    if (pid_pitch_rate > 3000.0) {pid_pitch_rate = 3000.0; }
    if (pid_pitch_rate < -3000.0) {pid_pitch_rate = -3000.0;}
    
    error_pitch_rate_old = error_pitch_rate;

//outer (angle) roll cycle
    error_roll_angle = rc_fresh_data.received_roll - roll; 
    integral_roll_error_angle = integral_roll_error_angle + Ki_roll_angle * error_roll_angle;
    if (integral_roll_error_angle > 1000.0) integral_roll_error_angle = 1000.0;
    if (integral_roll_error_angle < -1000.0) integral_roll_error_angle =-1000.0;
    if (rc_fresh_data.received_throttle < 9000.0) integral_roll_error_angle = 0;              //to avoid accumulation on the ground  
    pid_roll_angle = Kp_roll_angle * error_roll_angle + integral_roll_error_angle + Kd_roll_angle * (error_roll_angle - error_roll_angle_old);    
    if (pid_roll_angle > 2000.0) pid_roll_angle = 2000.0;
    if (pid_roll_angle < -2000.0) pid_roll_angle = -2000.0;
    error_roll_angle_old = error_roll_angle;

//inner (rate) roll cycle
    gyro_roll = 0.1 * (((gyro_converted_1[1] - gyro_converted_2[0]) / 2.0 ) * (180.0 / (float)PI)) + 0.9 * gyro_roll_old;
    gyro_roll_old = gyro_roll;
    error_roll_rate = pid_roll_angle - gyro_roll;
    //error_roll_rate = rc_fresh_data.received_roll - gyro_roll;//for inner loop setup
    integral_roll_error_rate = integral_roll_error_rate + Ki_roll_rate * error_roll_rate;
    if (integral_roll_error_rate > 1000.0) integral_roll_error_rate = 1000.0; 
    if (integral_roll_error_rate < -1000.0) integral_roll_error_rate = -1000.0;
    diff_roll_error_rate = Kd_roll_rate * (error_roll_rate - error_roll_rate_old);
    if (rc_fresh_data.received_throttle < 9000.0) {integral_roll_error_rate = 0; diff_roll_error_rate = 0;}             //to avoid accumulation on the ground 
    pid_roll_rate = Kp_roll_rate * error_roll_rate + integral_roll_error_rate + diff_roll_error_rate;
    if (pid_roll_rate > 3000.0) pid_roll_rate = 3000.0;
    if (pid_roll_rate < -3000.0) pid_roll_rate = -3000.0;
    error_roll_rate_old = error_roll_rate;

    yaw_setpoint = yaw_setpoint + 0.0016 * rc_fresh_data.received_yaw;
    if (yaw_setpoint >= 360) yaw_setpoint = yaw_setpoint - 360.0;
    if (yaw_setpoint < 0) yaw_setpoint = yaw_setpoint + 360.0;

//outer (angle) yaw cycle
    if (rc_fresh_data.received_throttle < 9000.0) yaw_setpoint = yaw;  //to avoid accumulation on the ground
    error_yaw_angle = -yaw_setpoint + yaw;
    if (error_yaw_angle <= -180.0) error_yaw_angle = error_yaw_angle + 360.0; 
    if (error_yaw_angle >= 180.0)  error_yaw_angle = error_yaw_angle - 360.0;

    integral_yaw_error_angle = integral_yaw_error_angle + Ki_yaw_angle * error_yaw_angle;
    if (integral_yaw_error_angle > 500.0) integral_yaw_error_angle = 500.0;
    if (integral_yaw_error_angle < -500.0) integral_yaw_error_angle =-500.0;
    if (rc_fresh_data.received_throttle < 9000) integral_yaw_error_angle = 0;              //to avoid accumulation on the ground
    pid_yaw_angle = Kp_yaw_angle * error_yaw_angle + integral_yaw_error_angle + Kd_yaw_angle * (error_yaw_angle - error_yaw_angle_old);  ;
    if (pid_yaw_angle > 1000.0)  pid_yaw_angle = 1000.0;
    if (pid_yaw_angle < -1000.0) pid_yaw_angle = -1000.0;
    error_yaw_angle_old = error_yaw_angle;
   
//inner yaw cycle
    error_yaw_rate = pid_yaw_angle - ((((gyro_converted_1[2] * (-1.0)) - gyro_converted_2[2]) / 2.0) * (180.0 / (float)PI));
    //error_yaw_rate = rc_fresh_data.received_yaw - ((((gyro_converted_1[2] * (-1.0)) - gyro_converted_2[2]) / 2.0) * (180.0 / (float)PI)); //for inner loop setup
    integral_yaw_error_rate = integral_yaw_error_rate + Ki_yaw_rate * error_yaw_rate;
    if (integral_yaw_error_rate > 1000.0) integral_yaw_error_rate = 1000.0; 
    if (integral_yaw_error_rate < -1000.0) integral_yaw_error_rate = -1000.0;
    diff_yaw_error_rate = Kd_yaw_rate * (error_yaw_rate - error_yaw_rate_old); 
    if (rc_fresh_data.received_throttle < 9000.0) {integral_yaw_error_rate = 0; diff_yaw_error_rate = 0;}            //to avoid accumulation on the ground
    pid_yaw_rate = Kp_yaw_rate * error_yaw_rate + integral_yaw_error_rate + diff_yaw_error_rate;
    if (pid_yaw_rate > 3000.0) pid_yaw_rate = 3000.0;
    if (pid_yaw_rate < -3000.0) pid_yaw_rate = -3000.0;
    error_yaw_rate_old = error_yaw_rate;

    engine[0] = (rc_fresh_data.received_throttle + pid_pitch_rate + pid_roll_rate - pid_yaw_rate);
    engine[1] = (rc_fresh_data.received_throttle + pid_pitch_rate - pid_roll_rate + pid_yaw_rate);
    engine[2] = (rc_fresh_data.received_throttle - pid_pitch_rate - pid_roll_rate - pid_yaw_rate);
    engine[3] = (rc_fresh_data.received_throttle - pid_pitch_rate + pid_roll_rate + pid_yaw_rate);

#ifdef BATTERY_COMPENSATION
    voltage_correction_coeff = 0.9 * voltage_correction_coeff + 0.1 * (11.1 / INA219_fresh_data[0]);           //11.1 - номинальное напряжение 3S АКБ, 12,6 максимальное, 9,9 минимальное
        
    if (voltage_correction_coeff < (11.1/12.6)) voltage_correction_coeff = 11.1/12.6;
    if (voltage_correction_coeff > (11.1/8.8))  voltage_correction_coeff = 11.1/8.8;

    for (i=0;i<4;i++) engine[i] *= sqrt(voltage_correction_coeff);
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
  set_to_log.altitude_cm = (uint16_t)lidar_fresh_data.altitude;
  set_to_log.voltage_mv = (uint16_t)(INA219_fresh_data[0] * 1000);
  set_to_log.current_ca = (uint16_t)(INA219_fresh_data[1] * 100);
  set_to_log.throttle_command = rc_fresh_data.received_throttle;
  set_to_log.pitch_command = rc_fresh_data.received_pitch;
  set_to_log.roll_command = rc_fresh_data.received_roll;
  set_to_log.yaw_command = rc_fresh_data.received_yaw;
  set_to_log.mode_command = rc_fresh_data.mode;
  for (i=0;i<4;i++) set_to_log.engines[i] = engine[i];
  
  if (rc_fresh_data.engines_start_flag) flags_byte |= 0b00000001;                                 //bit0 - engine start flag
   else flags_byte &= 0b11111110;
  if (remote_control_lost_comm_counter == RC_NO_COMM_DELAY_MAIN_CYCLES) flags_byte |= 0b00000010; //bit1 - comm lost flag
   else flags_byte &= 0b11111101;
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
  main_to_mavlink_data.altitude_cm = (uint16_t)lidar_fresh_data.altitude;
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

 
//при запуске сначала понимаем надо ли калибровать IMU (стоит ли перемывка DI4). Если нет - считываем ранее записанные значения калибровочных коеффициентов из NVS

if (!(MCP23017_get_inputs_state() & 0b00010000))  calibration_flag = 1; //DI4 - IMUs calibration
else 
{
  NVS_reading_calibration_values();
    
  if ((gyro_1_offset[0]  || gyro_1_offset[1] || gyro_1_offset[2] || accel_1_offset[0] || accel_1_offset[1] || accel_1_offset[2]) == 0)
  {
    ESP_LOGE(TAG_FLY,"IMUs не откалиброваны, установите джампер J4 для калибровки\n");
    uint16_t error_code = 10;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);}
  }
}

vTaskDelay(100/portTICK_PERIOD_MS);       //without these delay assertion fails

ESP_LOGI(TAG_FLY,"Создаем и запускаем таймер контроля зависания IMU#1.....");
Create_and_start_IMU_1_suspension_Timer();
ESP_LOGI(TAG_FLY,"Таймер контроля зависания IMU#1 запущен\n");

ESP_LOGI(TAG_FLY,"Создаем и запускаем таймер контроля зависания IMU#2.....");
Create_and_start_IMU_2_suspension_Timer();
ESP_LOGI(TAG_FLY,"Таймер контроля зависания IMU#2 запущен\n");

ESP_LOGI(TAG_FLY,"Активируем прерывания на входах от IMU и MCP23017.....");
configure_pins_for_interrupt();
ESP_LOGI(TAG_FLY,"Прерывания от IMU и MCP23017 активированы\n");
#ifdef USING_W25N
start_time = get_time();
#endif

if (calibration_flag) ESP_LOGI(TAG_INIT,"Установлена перемычка DI4, начинаем калибровку IMU....");
else ESP_LOGI(TAG_FLY,"К ПОЛЕТУ ГОТОВ!\n");
   
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

//если требуется калибровка начинаем накапливать эти данные NUMBER_OF_IMU_CALIBRATION_COUNTS раз         
      if ((calibration_flag) && (large_counter < NUMBER_OF_IMU_CALIBRATION_COUNTS))
      { 
        Gyro_X_cal_1 += gyro_raw_1[0];
        Gyro_Y_cal_1 += gyro_raw_1[1];
        Gyro_Z_cal_1 += gyro_raw_1[2];
        Accel_X_cal_1 += accel_raw_1[0];
        Accel_Y_cal_1 += accel_raw_1[1];
        Accel_Z_cal_1 += accel_raw_1[2];

        Gyro_X_cal_2 += gyro_raw_2[0];
        Gyro_Y_cal_2 += gyro_raw_2[1];
        Gyro_Z_cal_2 += gyro_raw_2[2];
        Accel_X_cal_2 += accel_raw_2[0];
        Accel_Y_cal_2 += accel_raw_2[1];
        Accel_Z_cal_2 += accel_raw_2[2];
        }
//по достижении этого значения вычисляем коэффициенты, сохраняем их в NVS и просим перезапустить систему
      if ((calibration_flag) && (large_counter == NUMBER_OF_IMU_CALIBRATION_COUNTS))
      {
        gyro_1_offset[0] = (Gyro_X_cal_1 / (NUMBER_OF_IMU_CALIBRATION_COUNTS - 1));
        gyro_1_offset[1] = (Gyro_Y_cal_1 / (NUMBER_OF_IMU_CALIBRATION_COUNTS - 1));
        gyro_1_offset[2] = (Gyro_Z_cal_1 / (NUMBER_OF_IMU_CALIBRATION_COUNTS - 1));
        accel_1_offset[0] = (Accel_X_cal_1 / (NUMBER_OF_IMU_CALIBRATION_COUNTS - 1));
        accel_1_offset[1] = (Accel_Y_cal_1 / (NUMBER_OF_IMU_CALIBRATION_COUNTS - 1));
        accel_1_offset[2] = (Accel_Z_cal_1 / (NUMBER_OF_IMU_CALIBRATION_COUNTS - 1)) - 8192.0;  


        gyro_2_offset[0] = (Gyro_X_cal_2 / (NUMBER_OF_IMU_CALIBRATION_COUNTS - 1));
        gyro_2_offset[1] = (Gyro_Y_cal_2 / (NUMBER_OF_IMU_CALIBRATION_COUNTS - 1));
        gyro_2_offset[2] = (Gyro_Z_cal_2 / (NUMBER_OF_IMU_CALIBRATION_COUNTS - 1));
        accel_2_offset[0] = (Accel_X_cal_2 / (NUMBER_OF_IMU_CALIBRATION_COUNTS - 1));
        accel_2_offset[1] = (Accel_Y_cal_2 / (NUMBER_OF_IMU_CALIBRATION_COUNTS - 1));
        accel_2_offset[2] = (Accel_Z_cal_2 / (NUMBER_OF_IMU_CALIBRATION_COUNTS - 1)) - 8192.0;
          
        for (i=0;i<3;i++) ESP_LOGI(TAG_INIT,"MPU#2 gyro offset value is %d",gyro_2_offset[i] );
        for (i=0;i<3;i++) ESP_LOGI(TAG_INIT,"MPU#2 accel offset value is %d",accel_2_offset[i] );

        for (i=0;i<3;i++) ESP_LOGI(TAG_INIT,"MPU#1 gyro offset value is %d",gyro_1_offset[i] );
        for (i=0;i<3;i++) ESP_LOGI(TAG_INIT,"MPU#1 accel offset value is %d",accel_1_offset[i] );
        printf("\n");
        
        NVS_writing_calibration_values(accel_1_offset, gyro_1_offset, accel_2_offset, gyro_2_offset);

        ESP_LOGI(TAG_INIT,"Калибровка IMU завершена, снимите перемычку и перезапустите систему.");
      
        while (1) 
        {
          gpio_set_level(LED_RED, 1);
          gpio_set_level(LED_GREEN, 1);
          gpio_set_level(LED_BLUE, 1);
          vTaskDelay(500/portTICK_PERIOD_MS); 
          gpio_set_level(LED_RED, 0);
          gpio_set_level(LED_GREEN, 0);
          gpio_set_level(LED_BLUE, 0);
          vTaskDelay(500/portTICK_PERIOD_MS);
        }
      }
//если калибровка не требуется   
if (!(calibration_flag))
{
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

        long_cycle_flag = 1;
        for (i=0;i<madgwick_cycles;i++) {
//считываем время прошедшее с прошлого цифка расчета 
          ESP_ERROR_CHECK(gptimer_get_raw_count(GP_timer, &timer_value));
          ESP_ERROR_CHECK(gptimer_set_raw_count(GP_timer, 0));
//если используем модуль с компасом и GPS запускаем фильтр Маджвика с учетом данных от магнетометра, используя в качестве входных параметров усредненные накопленные за предыдущие циклы 
//показания гироскопов и акселерометров, а также считав данные из очереди от магнетометра
#ifdef USING_HOLYBRO_M9N      
        xQueueReceive(magnetometer_queue, mag_fresh_data, 0);

        MadgwickAHRSupdate((gyro_converted_accumulated_1[1] - gyro_converted_accumulated_2[0]) / (2.0 * PID_LOOPS_RATIO), 
                              (gyro_converted_accumulated_1[0] + gyro_converted_accumulated_2[1]) / (2.0 * PID_LOOPS_RATIO), 
                              ((gyro_converted_accumulated_1[2] * (-1.0)) - gyro_converted_accumulated_2[2]) / (2.0 * PID_LOOPS_RATIO), 
                              (accel_converted_accumulated_1[1] - accel_converted_accumulated_2[0]) / (2.0 * PID_LOOPS_RATIO), 
                              (accel_converted_accumulated_1[0] + accel_converted_accumulated_2[1]) / (2.0 * PID_LOOPS_RATIO), 
                              ((accel_converted_accumulated_1[2] * (-1.0)) - accel_converted_accumulated_2[2]) / (2.0 * PID_LOOPS_RATIO), 
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
                                (accel_converted_accumulated_1[1] - accel_converted_accumulated_2[0]) / (2.0 * PID_LOOPS_RATIO) , 
                                (accel_converted_accumulated_1[0] + accel_converted_accumulated_2[1]) / (2.0 * PID_LOOPS_RATIO) ,
                                ((accel_converted_accumulated_1[2] * (-1.0)) - accel_converted_accumulated_2[2]) / (2.0 * PID_LOOPS_RATIO), 
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
        
        if (((large_counter + 5) % 25) == 0) xTaskNotifyGive(task_handle_lidar_read_and_process_data);
        //printf("%0.2f\n", voltage_correction_coeff);
//далее место где удобно что-то выводить, печатать 

//if ((large_counter % 1000) == 0) ESP_LOGI(TAG_FLY,"%0.3f, %0.3f, %0.3f, %0.3f", accel_rotated[0], accel_rotated[1], accel_rotated[2], accel_rotated[3]);
//if ((large_counter % 1000) == 0) ESP_LOGI(TAG_FLY,"fil are %0.2f, %0.2f, %0.2f, %0.2f", engine_filtered[0],engine_filtered[1],engine_filtered[2],engine_filtered[3]);
//if ((large_counter % 1000) == 0) ESP_LOGI(TAG_FLY,"new are %0.2f, %0.2f, %0.2f, %0.2f", engine_filtered_new[0],engine_filtered_new[1],engine_filtered_new[2],engine_filtered_new[3]);        
//if ((large_counter % 1000) == 0) ESP_LOGI(TAG_FLY,"%0.3f, %0.3f, %0.3f, %0.3f,    //%0.3f, %0.3f, %0.3f", q0, q1, q2, q3, pitch, roll, yaw);
/*        if ((large_counter % 5000) == 0) 
        {
          UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
          ESP_LOGW(TAG_FLY,"High watermark %d",  uxHighWaterMark);
        }       
            if ((large_counter % 2000) == 0) {
              command_for_PCA9685 +=20;
              if (command_for_PCA9685 > 0x0260) command_for_PCA9685 = 0x0201;
              xQueueSend(PCA9685_queue, &command_for_PCA9685, NULL);}

*/
        //if (large_counter > 10000) {vTaskDelay(1000/portTICK_PERIOD_MS);}   

        
        
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
            rc_fresh_data.received_throttle = RC_NO_COMM_THROTTLE_HOVER_VALUE;
            rc_fresh_data.received_pitch = 0;
            rc_fresh_data.received_roll = 0;
            rc_fresh_data.received_yaw = 0;
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
          ESP_LOGD(TAG_FLY,"высота %0.3f, вертикальная скорость %0.3f", lidar_fresh_data.altitude, lidar_fresh_data.vertical_velocity); // высота в cm
          if (lidar_fresh_data.altitude < 900) lidar_fresh_data.altitude = lidar_fresh_data.altitude * cos(pitch * 0.017453292) * cos(roll * 0.017453292);// PI/180
          old_altitude = lidar_fresh_data.altitude;
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
#ifdef USING_TFMINIS_I2C 
        if ((rc_fresh_data.altitude_hold_flag) && (lidar_fresh_data.altitude < 900)) 
          {
            if (altitude_hold_mode_enabled == 0)                                      //если только включили режим, первый раз в цикл вошли
            {
              altitude_setpoint = lidar_fresh_data.altitude;
              altitude_hold_mode_enabled = 1;
              alt_hold_initial_throttle = rc_fresh_data.received_throttle;
            }
          error_altitude = altitude_setpoint - lidar_fresh_data.altitude;
            integral_alt_error += Ki_alt * error_altitude;                         
            if (integral_alt_error > 500) integral_alt_error = 500;
            if (integral_alt_error < -500) integral_alt_error = -500;

            pid_altitude = Kp_alt * error_altitude + Kd_alt * (error_altitude - previous_error_altitude) + integral_alt_error;
            if (pid_altitude > 3000) pid_altitude = 3000;
            if (pid_altitude < -3000) pid_altitude = -3000;
            //if ((large_counter % 100) == 0) printf("%0.3f, %0.3f, %0.3f\n", Kp_alt * error_altitude, Kd_alt * (error_altitude - previous_error_altitude), integral_alt_error);
            //if ((large_counter % 10) == 0) printf("%0.3f\n",Kd_alt * (error_altitude - previous_error_altitude));
            rc_fresh_data.received_throttle = alt_hold_initial_throttle + pid_altitude;       //заменяем значение throttle от пульта вычисленным при помощт регулятора
            previous_error_altitude = error_altitude;
          }
        else 
          {
            altitude_hold_mode_enabled = 0; 
            previous_error_altitude = 0;
          }
#endif

//на основании всех полученных выше данных считаем двухконтурным ПИД-регулятором управляющие воздействия на двигатели
        calculate_pids_2();
        }
//если от пульта команда что двигатели выключены - сбрасываем интегральные составляющие ПИД
        else 
        {
          yaw_setpoint = yaw;
          error_pitch_rate_old = error_yaw_rate_old = error_roll_rate_old = 0;
          integral_pitch_error_rate = integral_yaw_error_rate = integral_roll_error_rate = 0;
          integral_pitch_error_angle = integral_roll_error_angle = integral_yaw_error_angle = 0;
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
        data_to_send_to_rc.altitude = (uint16_t)lidar_fresh_data.altitude;
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
        long_cycle_flag = 0;
        IMU_interrupt_status = 0;
//сбрасываем таймер общего зависания, по сработке которого аварийно останавливаем двигатели        
        ESP_ERROR_CHECK(gptimer_set_raw_count(general_suspension_timer, 0));
      }
    } 
  }
}


//Задача первичной инициализации.
//Проверяет связь со всем компонентами, производит их настройку. При необходимости калибрует ESC, запускает задачу считывания логов. 
//если все прошло гладко - создает основные рабочие задачи. Если где-то ошибка - запускается задача аварийного моргания светодиодами для визуальной индикации ошибки.
//по завершении выполнения задача инициализации самоликвидируется, так как она больше не нужна

static void init(void * pvParameters)
{
  uint8_t error_code = 0;

  esp_log_level_set(TAG_INIT,ESP_LOG_INFO);  //WARN ERROR
  esp_log_level_set(TAG_FLY,ESP_LOG_INFO);  //WARN ERROR
  esp_log_level_set(TAG_GPS,ESP_LOG_WARN);  //WARN ERROR
  esp_log_level_set(TAG_NVS,ESP_LOG_INFO);   //WARN ERROR
  esp_log_level_set(TAG_RC,ESP_LOG_INFO);    //WARN ERROR
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

 ESP_LOGI(TAG_MS5611,"Считывание PROM MS5611.....");
  if (MS5611_I2C_PROM_read() != ESP_OK) {
    error_code = 3;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

#endif

  ESP_LOGI(TAG_INIT,"Проверка связи с MCP23017.....");
  extern i2c_master_dev_handle_t MCP23017_dev_handle;
  

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

  if (!(MCP23017_get_inputs_state() & 0b00001000))  //если стоит перемычка DI3 - запускаем по очереди каждый из двигателей (использовал для балансировки двигателей)
    { 
      ESP_LOGI(TAG_INIT,"Установлена перемычка DI3, начинаем проверку двигателей.....");
      printf ("%d", MCP23017_get_inputs_state() );
      vTaskDelay(2000/portTICK_PERIOD_MS);
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 0, ENGINE_PWM_MIN_DUTY * 1.5));                  //проверяем двигатели на среднем уровне сигнала
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 0));
      vTaskDelay(TIME_TO_KEEP_RUNNING_AT_CHECK_MS/portTICK_PERIOD_MS);

      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 0, ENGINE_PWM_MIN_DUTY));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 0));
      vTaskDelay(1000/portTICK_PERIOD_MS);
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 1, ENGINE_PWM_MIN_DUTY * 1.5));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 1));
      vTaskDelay(TIME_TO_KEEP_RUNNING_AT_CHECK_MS/portTICK_PERIOD_MS);

      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 1, ENGINE_PWM_MIN_DUTY));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 1));
      vTaskDelay(1000/portTICK_PERIOD_MS);
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 2, ENGINE_PWM_MIN_DUTY * 1.5));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 2));
      vTaskDelay(TIME_TO_KEEP_RUNNING_AT_CHECK_MS/portTICK_PERIOD_MS);
      
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 2, ENGINE_PWM_MIN_DUTY));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 2));
      vTaskDelay(1000/portTICK_PERIOD_MS);
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 3, ENGINE_PWM_MIN_DUTY * 1.5));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 3));
      vTaskDelay(TIME_TO_KEEP_RUNNING_AT_CHECK_MS/portTICK_PERIOD_MS);

      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 3, ENGINE_PWM_MIN_DUTY));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 3));
      
      ESP_LOGI(TAG_INIT,"Проверка двигатетей завершена, снимите перемычку и перезапустите систему.");

      while (1) 
      {
        gpio_set_level(LED_RED, 1);
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

#ifdef USING_HOLYBRO_M9N
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

  ESP_LOGI(TAG_INIT,"Считываем данные cross-axis calibration из IST8310.....");
  IST8310_read_cross_axis_data(); 
#endif

#ifdef USING_TFMINIS_I2C
ESP_LOGI(TAG_INIT,"Проверка связи с Tfmini-S (по i2c).....");
if (tfminis_communication_check() != ESP_OK) {
  error_code = 9;
  xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
  while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
}
#endif

  ESP_LOGI(TAG_INIT,"Настраиваем оба SPI.....");
  SPI_init();
  ESP_LOGI(TAG_INIT,"Оба SPI настроены\n");

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

  if (!(MCP23017_get_inputs_state() & 0b00000010))       //DI1 - считывание логов
  {
    ESP_LOGI(TAG_INIT,"Установлена перемычка DI1, через 10 секунд начинаем считывание логов из памяти.....");
    vTaskDelay(10000/portTICK_PERIOD_MS);
    if (xTaskCreate(reading_logs_from_external_flash,"reading_logs_from_external_flash",4096,NULL,0,NULL) == pdPASS)//задача с приоритетом 0, как IDLE
    ESP_LOGI(TAG_INIT,"Создана задача для считывания логов из внешней flash-памяти\n");
    while (1) {vTaskDelay(500/portTICK_PERIOD_MS);}  
  }
  else {
    ESP_LOGI(TAG_INIT,"Удаляем данные с W25N.....");
    W25N_erase_all_new();
  }
  
  //W25N01_queue = xQueueCreate(5, LOGS_BYTES_PER_STRING);
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

#ifdef USING_HOLYBRO_M9N
  
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

  ESP_LOGI(TAG_INIT,"Создаем и запускаем GP timer.....");
  Create_and_start_GP_Timer ();
  ESP_LOGI(TAG_INIT,"GP timer создан и запущен\n");

//*************************************************** НАЧИНАЕМ СОЗДАВАТЬ ЗАДАЧИ ****************************************************************************************************** */

  ESP_LOGI(TAG_INIT,"Приступаем к созданию задач\n");

  //core 0 deals with WiFI tasks

 task_handle_MCP23017_monitoring_and_control = xTaskCreateStaticPinnedToCore(MCP23017_monitoring_and_control,"MCP23017_monitoring_and_control",MCP23017_MONITORING_AND_CONTROL_STACK_SIZE,NULL,1,MCP23017_monitoring_and_control_stack,&MCP23017_monitoring_and_control_TCB_buffer,0);
  if (task_handle_MCP23017_monitoring_and_control != NULL)
    ESP_LOGI(TAG_INIT,"Задача контроля и управления MCP23017 успешно создана на ядре 0\n");
  else {
    ESP_LOGE(TAG_INIT,"Задача контроля и управления MCP23017 не создана\n");
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_INIT,"Создаем задачу для управления PCA9685 (PCA9685_control)..... ");
  task_handle_PCA9685_control = xTaskCreateStaticPinnedToCore(PCA9685_control,"PCA9685_control",PCA9685_CONTROL_STACK_SIZE,NULL,1,PCA9685_control_stack,&PCA9685_control_TCB_buffer,0);
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
  if (xTaskCreateStaticPinnedToCore(gps_read_and_process_data, "gps_read_and_process_data", GPS_READ_AND_PROCESS_DATA_STACK_SIZE, NULL, 3, gps_read_and_process_data_stack, &gps_read_and_process_data_TCB_buffer, 0) != NULL)
    ESP_LOGI(TAG_INIT,"Задача для считывания данных GPS успешно создана на ядре 0\n");
  else {
    ESP_LOGE(TAG_INIT,"Задача для считывания данных GPS не создана\n");
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
#endif
  vTaskDelay(50/portTICK_PERIOD_MS);

#ifdef USING_HOLYBRO_M9N
  ESP_LOGI(TAG_INIT,"Создаем задачу для считывания данных с магнетометра (mag_read_and_process_data).....");
  task_handle_mag_read_and_process_data = xTaskCreateStaticPinnedToCore(mag_read_and_process_data, "mag_read_and_process_data", MAG_READ_AND_PROCESS_DATA_STACK_SIZE, NULL, 6, mag_read_and_process_data_stack, &mag_read_and_process_data_TCB_buffer, 0);
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
  if (xTaskCreateStaticPinnedToCore(RC_read_and_process_data,"RC_read_and_process_data",RC_READ_AND_PROCESS_DATA_STACK_SIZE,NULL,5,RC_read_and_process_data_stack,&RC_read_and_process_data_TCB_buffer,0) != NULL)
    ESP_LOGI(TAG_INIT,"Задача для получения данных с пульта управления успешно создана на ядре 0\n");
  else {
    ESP_LOGE(TAG_INIT,"Задача для получения данных с пульта управления не создана\n");
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  vTaskDelay(50/portTICK_PERIOD_MS);

  ESP_LOGI(TAG_INIT,"Создаем задачу для отправки телеметрии на пульт управления.....");
  task_handle_send_data_to_RC = xTaskCreateStaticPinnedToCore(send_data_to_RC,"send_data_to_RC",SEND_DATA_TO_RC_STACK_SIZE ,NULL,4,send_data_to_RC_stack,&send_data_to_RC_TCB_buffer,0);
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
  task_handle_performace_measurement = xTaskCreateStaticPinnedToCore(performace_monitor,"performace_monitor",8192 ,NULL,2,performance_measurement_stack, &performance_measurement_TCB_buffer,0);
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
  task_handle_lidar_read_and_process_data = xTaskCreateStaticPinnedToCore(lidar_read_and_process_data,"lidar_read_and_process_data",LIDAR_READ_AND_PROCESS_DATA_STACK_SIZE ,NULL,3,lidar_read_and_process_data_stack,&lidar_read_and_process_data_TCB_buffer,0);
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
  task_handle_INA219_read_and_process_data = xTaskCreateStaticPinnedToCore(INA219_read_and_process_data,"INA219_read_and_process_data",INA219_READ_AND_PROCESS_DATA_STACK_SIZE,NULL,2,INA219_read_and_process_data_stack, &INA219_read_and_process_data_TCB_buffer,0);
  if ( task_handle_INA219_read_and_process_data != NULL)
    ESP_LOGI(TAG_INIT,"Задача считывания данных с INA219 успешно создана на ядре 0\n");
  else {
    ESP_LOGE(TAG_INIT,"Задача считывания данных с INA219 не создана\n");
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_INIT,"Создаем задачу моргания полетными огнями (blinking_flight_lights).....");
  task_handle_blinking_flight_lights = xTaskCreateStaticPinnedToCore(blinking_flight_lights,"blinking_flight_lights",BLINKING_FLIGHT_LIGHTS_STACK_SIZE,NULL,0,blinking_flight_lights_stack, &blinking_flight_lights_TCB_buffer,0);
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
  task_handle_writing_logs_to_flash = xTaskCreateStaticPinnedToCore(writing_logs_to_flash,"writing_logs_to_flash",WRITING_LOGS_TO_FLASH_STACK_SIZE,NULL,0,writing_logs_to_flash_stack, &writing_logs_to_flash_TCB_buffer,0);
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

#ifdef USING_MAVLINK_TELEMETRY
  ESP_LOGI(TAG_INIT,"Создаем задачу отправки телеметрии по mavink (send_telemetry_via_mavlink).....");
  task_handle_mavlink_telemetry = xTaskCreateStaticPinnedToCore(send_telemetry_via_mavlink,"send_telemetry_via_mavlink",MAVLINK_TELEMETRY_STACK_SIZE,NULL,0,mavlink_telemetry_stack, &mavlink_telemetry_TCB_buffer,0);
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

  ESP_LOGI(TAG_INIT,"Создаем задачу основного полетного цикла (Main_flying_cycle)..... ");
  task_handle_main_flying_cycle = xTaskCreateStaticPinnedToCore(main_flying_cycle, "Main_flying_cycle", MAIN_FLYING_CYCLE_STACK_SIZE, NULL, 24, main_flying_cycle_stack,&main_flying_cycle_TCB_buffer,1);
  if (task_handle_main_flying_cycle != NULL)
    ESP_LOGI(TAG_INIT,"Задача основного полетного цикла успешно создана на ядре 1\n");
  else {
    ESP_LOGE(TAG_INIT,"Задача основного полетного цикла не создана\n");
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_INIT,"Создаем и запускаем таймер контроля зависания основного полетного цикла.....");
  create_and_start_general_suspension_timer();
  ESP_LOGI(TAG_INIT,"Таймер контроля зависания основного полетного цикла запущен\n");

//все задачи созданы, убиваем задачу инициализаци         
  vTaskDelete(NULL);
  
}

/********************************************************************     СЕКЦИЯ 5     ***********************************************************************************************/
/********************************************************************  СОБСТВЕННО MAIN ***********************************************************************************************/
void app_main(void) {

  xTaskCreatePinnedToCore(init, "init", 8912, NULL, 5, &task_handle_init, 1);
 
}



