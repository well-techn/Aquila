//system libraries

//the first commit
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

//proprietary libraries
#include "ve_alldef.h"
#include "ve_i2c.h"
#include "VL53L1x.h"
#include "PCA9685.h"
#include "VL53L1X.h"
#include "MCP23017.h"
#include "ve_spi.h"
#include "winbondW25N.h"
#include "madgwick.h"
#include "PMW3901.h"
#include "MPU6000.h"
#include "TfminiS.h"
#include "INA219.h"
#include "IST8310.h"
#include "FL3195.h"


//ESP32 logging labels
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
const char *TAG_LIDAR = "TFSmini";
const char *TAG_INA219 = "INA219";
const char *TAG_IST8310 = "IST8310";
const char *TAG_FL3195 = "RGB_LED";


//spi devices handles
extern spi_device_handle_t W25N01;
extern spi_device_handle_t MPU6000_1;
extern spi_device_handle_t MPU6000_2;
extern i2c_master_bus_handle_t i2c_internal_bus_handle;
extern i2c_master_bus_handle_t i2c_external_bus_handle;

//i2c devices handlwe
extern i2c_master_dev_handle_t IST8310_dev_handle;

//semaphores
SemaphoreHandle_t semaphore_to_read_mag;

//timer handles
gptimer_handle_t GP_timer;
gptimer_handle_t RC_timer;
gptimer_handle_t general_suspension_timer;
gptimer_handle_t IMU_1_suspension_timer;
gptimer_handle_t IMU_2_suspension_timer;

//ADC handles
adc_oneshot_unit_handle_t adc1_handle;

//queues handles
static QueueHandle_t gps_queue_for_events; //queue to handle gps uart events (pattern detection)
static QueueHandle_t gps_to_main_queue; //queue to transfer gps information from gps processing task to main flying task
static QueueHandle_t remote_control_queue_for_events; //queue to handle RC uart events (pattern detection)
static QueueHandle_t remote_control_to_main_queue; //queue to transfer remote control data to main flying task
static QueueHandle_t lidar_queue_for_events; //queue to handle lidar uart events (pattern detection)
static QueueHandle_t MCP23017_queue; //queue to handle any deals with MCP23017
static QueueHandle_t PCA9685_queue; //queue to handle any deals with PCA9685
static QueueHandle_t magnetometer_queue; //queue to transfer data from mag read task to main flying task
static QueueHandle_t main_to_rc_queue; //queue to transfer data from main to rc_send task 
static QueueHandle_t W25N01_queue;  //queue to transfer data from main to logging task
static QueueHandle_t lidar_to_main_queue; //queue to transfer data from lidar to main
static QueueHandle_t INA219_to_main_queue; //queue to transfer data from INA219 to main
static QueueHandle_t any_to_blinking_queue; //queue to transfer data from any task to blinking
static QueueHandle_t IMU_monitoring_timer_to_main_queue; //queue which is used to transfer data about any of 2 IMUs suspension (no interrupt signal at a time)

//static tasks parameters 
StaticTask_t MCP23017_monitoring_and_control_TCB_buffer;
StackType_t MCP23017_monitoring_and_control_stack[MCP23017_MONITORING_AND_CONTROL_STACK_SIZE];

StaticTask_t PCA9685_control_TCB_buffer;
StackType_t PCA9685_control_stack[PCA9685_CONTROL_STACK_SIZE];

StaticTask_t read_and_process_data_from_mag_TCB_buffer;
StackType_t read_and_process_data_from_mag_stack[READ_AND_PROCESS_DATA_FROM_MAG_STACK_SIZE];

StaticTask_t main_flying_cycle_TCB_buffer;
StackType_t main_flying_cycle_stack[MAIN_FLYING_CYCLE_STACK_SIZE];

StaticTask_t monitoring_pins_interrupt_queue_TCB_buffer;
StackType_t monitoring_pins_interrupt_queue_stack[MONITORING_PINS_INTERRUPT_QUEUE_STACK_SIZE];

StackType_t read_and_process_data_from_gps_stack[READ_AND_PROCESS_DATA_FROM_GPS_STACK_SIZE];
StaticTask_t read_and_process_data_from_gps_TCB_buffer;


StaticTask_t read_and_process_data_from_RC_TCB_buffer;
StackType_t read_and_process_data_from_RC_stack[READ_AND_PROCESS_DATA_FROM_RC_STACK_SIZE];

StaticTask_t send_data_to_RC_TCB_buffer;
StackType_t send_data_to_RC_stack[SEND_DATA_TO_RC_STACK_SIZE];

StaticTask_t blinking_flight_lights_TCB_buffer;
StackType_t blinking_flight_lights_stack[BLINKING_FLIGHT_LIGHTS_STACK_SIZE];

StaticTask_t read_and_process_data_from_lidar_TCB_buffer;
StackType_t read_and_process_data_from_lidar_stack[read_and_process_data_from_lidar_STACK_SIZE];

StaticTask_t read_and_process_data_from_INA219_TCB_buffer;
StackType_t read_and_process_data_from_INA219_stack[READ_AND_PROCESS_DATA_FROM_INA219_STACK_SIZE];

TaskHandle_t task_handle_blinking_flight_lights;
TaskHandle_t task_handle_main_flying_cycle;
TaskHandle_t task_handle_send_data_to_RC;
TaskHandle_t task_handle_init;
TaskHandle_t task_handle_performace_measurement;
TaskHandle_t task_handle_read_and_process_data_from_INA219;
TaskHandle_t task_handle_read_and_process_data_from_mag;


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
  
  if (gpio_intr_status_1 & (1ULL << MPU6000_1_INTERRUPT_PIN))   //IMU_1 routine 
  {
    imu_1_interrupt_flag = 1; 
    gptimer_get_raw_count (IMU_1_suspension_timer,&IMU_1_timer_value);
    gptimer_set_raw_count(IMU_1_suspension_timer, 0);
    gptimer_get_raw_count (IMU_2_suspension_timer,&IMU_2_timer_value);
  } 
  
  if (gpio_intr_status_1 & (1ULL << MPU6000_2_INTERRUPT_PIN))   //IMU_2 routine  
  {
    imu_2_interrupt_flag = 1;
    gptimer_get_raw_count (IMU_1_suspension_timer,&IMU_1_timer_value);
    gptimer_get_raw_count (IMU_2_suspension_timer,&IMU_2_timer_value); 
    gptimer_set_raw_count(IMU_2_suspension_timer, 0);
  }

  if (imu_1_interrupt_flag && (IMU_1_timer_value < IMU_SUSPENSION_TIMER_DELAY_MS * 1000) && (IMU_2_timer_value < IMU_SUSPENSION_TIMER_DELAY_MS * 1000)) //all is ok
  {
    //gpio_set_level(LED_GREEN,0);
    imu_1_interrupt_flag = 0;
    imu_2_interrupt_flag = 0;
    xTaskGenericNotifyFromISR(task_handle_main_flying_cycle, 0, 14, eSetValueWithOverwrite, NULL,  &xHigherPriorityTaskWoken);
    //gpio_set_level(LED_GREEN,1);
  }

 else if (IMU_2_timer_value > IMU_SUSPENSION_TIMER_DELAY_MS * 1000)  //2nd failed
  {
    imu_1_interrupt_flag = 0;
    imu_2_interrupt_flag = 0;
    xTaskGenericNotifyFromISR(task_handle_main_flying_cycle, 0, 15, eSetValueWithOverwrite, NULL, &xHigherPriorityTaskWoken);
  }

  else if ( IMU_1_timer_value > IMU_SUSPENSION_TIMER_DELAY_MS * 1000)  //1st failed
  {
    imu_1_interrupt_flag = 0;
    imu_2_interrupt_flag = 0;
    xTaskGenericNotifyFromISR(task_handle_main_flying_cycle, 0, 16, eSetValueWithOverwrite, NULL, &xHigherPriorityTaskWoken);
  }

  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static void IRAM_ATTR general_suspension_timer_interrupt_handler(void *args)    //motor control emergency disable
{
   
   ledc_timer_pause(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);


  //WRITE_PERI_REG((DR_REG_LEDC_BASE + 0x00),0);                    эти регистры переписываются на update_duty

  //REG_CLR_BIT((DR_REG_LEDC_BASE + 0x00),4);  //LEDC_CH0_CONF0_REG
  //REG_CLR_BIT((DR_REG_LEDC_BASE + 0x14),4);  //LEDC_CH1_CONF0_REG
  //REG_CLR_BIT((DR_REG_LEDC_BASE + 0x28),4);  //LEDC_CH2_CONF0_REG
  //REG_CLR_BIT((DR_REG_LEDC_BASE + 0x3C),4);  //LEDC_CH3_CONF0_REG
}

static void IRAM_ATTR IMU_1_suspension_timer_interrupt_handler(void *args)
{
  uint8_t flag = 1;
 
  //gpio_set_level(LED_BLUE, 0);
  //xQueueSendFromISR(IMU_monitoring_timer_to_main_queue, &flag, NULL);
  
}

static void IRAM_ATTR IMU_2_suspension_timer_interrupt_handler(void *args)
{
  uint8_t flag = 2;
  //gpio_set_level(LED_GREEN, 0); 
  //xQueueSendFromISR(IMU_monitoring_timer_to_main_queue, &flag, NULL);
}

//main pins configuration
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
}
//IO interrupts configuration
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

  ESP_ERROR_CHECK(gpio_reset_pin(MCP23017_INTERRUPT_PIN));
  gpio_config_t INT_3 = {
    .pin_bit_mask = 1ULL << MCP23017_INTERRUPT_PIN,
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
//UARTs configuration
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

    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(GPS_UART, '*', 1, 9, 0, 0));                 //creating pattern detection
    ESP_ERROR_CHECK(uart_pattern_queue_reset(GPS_UART, GPS_UART_PATTERN_DETECTION_QUEUE_SIZE));          //allocating queue  
    uart_flush(GPS_UART);                                                                           //resetting incoming buffer
}

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

    ESP_ERROR_CHECK(uart_driver_install(REMOTE_CONTROL_UART, RC_UART_BUFF_SIZE, 0, RC_UART_PATTERN_DETECTION_QUEUE_SIZE, &remote_control_queue_for_events, intr_alloc_flags)); 
    ESP_ERROR_CHECK(uart_param_config(REMOTE_CONTROL_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(REMOTE_CONTROL_UART, RC_UART_TX_PIN, RC_UART_RX_PIN, RC_UART_RTS_PIN, RC_UART_CTS_PIN));

    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(REMOTE_CONTROL_UART, 0xFF, 1, 10, 0, 0));                 //creating pattern detection
    ESP_ERROR_CHECK(uart_pattern_queue_reset(REMOTE_CONTROL_UART, RC_UART_PATTERN_DETECTION_QUEUE_SIZE));          //allocating queue  
    uart_flush(REMOTE_CONTROL_UART);                                                                           //resetting incoming buffer  
}
#ifdef USING_LIDAR_UART
static void lidar_uart_config()
{
    int intr_alloc_flags = 0;
    uart_config_t lidar_uart_config = {
        .baud_rate = LIDAR_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(LIDAR_UART, LIDAR_UART_BUF_SIZE, 0, LIDAR_UART_PATTERN_DETECTION_QUEUE_SIZE, &lidar_queue_for_events, intr_alloc_flags)); 
    ESP_ERROR_CHECK(uart_param_config(LIDAR_UART, &lidar_uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART, LIDAR_UART_TX_PIN, LIDAR_UART_RX_PIN, LIDAR_UART_RTS_PIN, LIDAR_UART_CTS_PIN));

    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(LIDAR_UART, 0x59, 2, 3, 0, 0));                 //creating pattern detection
    ESP_ERROR_CHECK(uart_pattern_queue_reset(LIDAR_UART, LIDAR_UART_PATTERN_DETECTION_QUEUE_SIZE));          //allocating queue  
    uart_flush(LIDAR_UART);                                                                           //resetting incoming buffer
}
#endif
//CRC
static uint8_t dallas_crc8(uint8_t *input_data, unsigned int size)
{
  uint8_t crc = 0;
  unsigned int i;
  unsigned char inbyte;
  unsigned char j;
  unsigned char mix1;
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
//ADC
static void configure_ADC(void) 
{
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  adc_oneshot_chan_cfg_t config_ADC_channel = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN_DB_11,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, A3, &config_ADC_channel));
}

static uint16_t read_ADC(adc_channel_t channel) 
{
  static uint16_t adc_raw;

  ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel, &adc_raw));
  
  return adc_raw;
}
//timers
static int64_t get_time(void)
{
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  return (int64_t)((int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec);
}

static void Create_and_start_GP_Timer()                    //timer for Madgwick
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
/*
static void Create_and_start_test_timer()                    //timer for Madgwick
{
  gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 10 * 1000 * 1000, // 10MHz, 1 tick = 0,1us
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &test_timer));
  ESP_ERROR_CHECK(gptimer_enable(test_timer));
  ESP_ERROR_CHECK(gptimer_start(test_timer));
}
*/
static void create_and_start_general_suspension_timer()                    //timer to control suspension of main cycle
{
  gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &general_suspension_timer));

  gptimer_alarm_config_t alarm_config = {                 //setting alarm threshold
      .alarm_count = SUSPENSION_TIMER_DELAY_SEC * 1000 * 1000,   //10 seconds
      //.reload_count = NULL,
  };
  ESP_ERROR_CHECK(gptimer_set_alarm_action(general_suspension_timer, &alarm_config));

  gptimer_event_callbacks_t  Suspension_timer_interrupt = {       //this function will be launched when timer alarm occures
      .on_alarm = general_suspension_timer_interrupt_handler, // register user callback
  };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(general_suspension_timer, &Suspension_timer_interrupt, NULL));

  ESP_ERROR_CHECK(gptimer_enable(general_suspension_timer));
  ESP_ERROR_CHECK(gptimer_start(general_suspension_timer));
}

static void Create_and_start_IMU_1_suspension_Timer()                    //timer to control absense of control signal from RC
{
  gptimer_config_t IMU_1_timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&IMU_1_timer_config, &IMU_1_suspension_timer));

  gptimer_alarm_config_t alarm_config = {                 //setting alarm threshold
      .alarm_count = IMU_SUSPENSION_TIMER_DELAY_MS * 1000,   //in ms
      .reload_count = 0,
  };
  ESP_ERROR_CHECK(gptimer_set_alarm_action(IMU_1_suspension_timer, &alarm_config));

  gptimer_event_callbacks_t  IMU_1_suspension_timer_interrupt = {       //this function will be launched when timer alarm occures
      .on_alarm = IMU_1_suspension_timer_interrupt_handler, // register user callback
  };
  //ESP_ERROR_CHECK(gptimer_register_event_callbacks(IMU_1_suspension_timer, &IMU_1_suspension_timer_interrupt, NULL));

  ESP_ERROR_CHECK(gptimer_enable(IMU_1_suspension_timer));
  ESP_ERROR_CHECK(gptimer_start(IMU_1_suspension_timer));
}

static void Create_and_start_IMU_2_suspension_Timer()                    //timer to control absense of control signal from RC
{
  gptimer_config_t IMU_2_timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&IMU_2_timer_config, &IMU_2_suspension_timer));

  gptimer_alarm_config_t alarm_config = {                 //setting alarm threshold
      .alarm_count = IMU_SUSPENSION_TIMER_DELAY_MS * 1000,   //in ms
      .reload_count = 0,
  };
  ESP_ERROR_CHECK(gptimer_set_alarm_action(IMU_2_suspension_timer, &alarm_config));

  gptimer_event_callbacks_t  IMU_2_suspension_timer_interrupt = {       //this function will be launched when timer alarm occures
      .on_alarm = IMU_2_suspension_timer_interrupt_handler, // register user callback
  };
  //ESP_ERROR_CHECK(gptimer_register_event_callbacks(IMU_2_suspension_timer, &IMU_2_suspension_timer_interrupt, NULL));

  ESP_ERROR_CHECK(gptimer_enable(IMU_2_suspension_timer));
  ESP_ERROR_CHECK(gptimer_start(IMU_2_suspension_timer));
}

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

static void configuring_channel_for_PWM(uint8_t channel, uint8_t pin) 
{
  // Prepare and then apply the LEDC PWM channel configuration
  ledc_channel_config_t engine_pwm_channel = {
      .speed_mode     = ENGINE_PWM_MODE,
      .channel        = channel,
      .timer_sel      = ENGINE_PWM_TIMER,
      .intr_type      = LEDC_INTR_DISABLE,
      .gpio_num       = pin,
      .duty           = ENGINE_PWM_MIN_DUTY , // Set duty to 5%
      .hpoint         = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&engine_pwm_channel));
}

static void NVS_read_write_test(void * pvParameters)
{
  // Initialize NVS ESP_LOGI(TAG_INIT,
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      // NVS partition was truncated and needs to be erased
      // Retry nvs_flash_init
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
  }
  ESP_ERROR_CHECK( err );

  // Open

  ESP_LOGI(TAG_NVS,"Opening Non-Volatile Storage (NVS) handle... ");
  nvs_handle_t my_handle;
  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
      ESP_LOGE(TAG_NVS,"Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } else {
        ESP_LOGI(TAG_NVS,"Done");

  /*
  // Write
        ESP_LOGI(TAG_NVS,"Generating random number ... ");
        uint32_t random_number = esp_random(); 
        ESP_LOGI(TAG_NVS,"%lu",random_number);
        err = nvs_set_u32(my_handle, "random_number", random_number);
        //ESP_LOGI(TAG_NVS,(err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        ESP_LOGI(TAG_NVS,"Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        //ESP_LOGI(TAG_NVS,(err != ESP_OK) ? "Failed!\n" : "Done\n");
*/
        // Read
        ESP_LOGI(TAG_NVS,"Reading number from NVS ... ");
       uint32_t random_number;
        err = nvs_get_u32(my_handle, "random_number", &random_number); //3841694106
        switch (err) {
            case ESP_OK:
                ESP_LOGI(TAG_NVS,"Done\n");
                ESP_LOGI(TAG_NVS,"Random_number = %lu", random_number);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGW(TAG_NVS,"The value is not initialized yet!\n");
                break;
            default :
                ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
        }

        nvs_close(my_handle);
    }
}

void NVS_writing_calibration_values(int16_t accel_1_offset[], int16_t gyro_1_offset[], int16_t accel_2_offset[], int16_t gyro_2_offset[])
{
  ESP_ERROR_CHECK(nvs_flash_erase());
  esp_err_t err = nvs_flash_init();
  

  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {

      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
  }
  ESP_ERROR_CHECK( err );

  ESP_LOGI(TAG_NVS,"Opening Non-Volatile Storage (NVS) handle... ");
  nvs_handle_t NVS_handle;
  err = nvs_open("storage", NVS_READWRITE, &NVS_handle);
  if (err != ESP_OK) ESP_LOGE(TAG_NVS,"Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  else 
  {
    ESP_LOGI(TAG_NVS,"Writing accel_1_X offset %d...", accel_1_offset[0]);
    err = nvs_set_i16(NVS_handle, "accel_1_off_0", accel_1_offset[0]);
    
    ESP_LOGI(TAG_NVS,"Writing accel_1_Y offset %d...", accel_1_offset[1]);
    err = nvs_set_i16(NVS_handle, "accel_1_off_1", accel_1_offset[1]);
  
    ESP_LOGI(TAG_NVS,"Writing accel_1_Z offset %d...", accel_1_offset[2]);
    err = nvs_set_i16(NVS_handle, "accel_1_off_2", accel_1_offset[2]);

    ESP_LOGI(TAG_NVS,"Writing gyro_1_X offset... ");
    err = nvs_set_i16(NVS_handle, "gyro_1_off_0", gyro_1_offset[0]);
    
    ESP_LOGI(TAG_NVS,"Writing gyro_1_Y offset... ");
    err = nvs_set_i16(NVS_handle, "gyro_1_off_1", gyro_1_offset[1]);
  
    ESP_LOGI(TAG_NVS,"Writing gyro_1_Z offset... ");
    err = nvs_set_i16(NVS_handle, "gyro_1_off_2", gyro_1_offset[2]);
//******************************************* */

    ESP_LOGI(TAG_NVS,"Writing accel_2_X offset... ");
    err = nvs_set_i16(NVS_handle, "accel_2_off_0", accel_2_offset[0]);
    
    ESP_LOGI(TAG_NVS,"Writing accel_2_Y offset... ");
    err = nvs_set_i16(NVS_handle, "accel_2_off_1", accel_2_offset[1]);
  
    ESP_LOGI(TAG_NVS,"Writing accel_2_Z offset... ");
    err = nvs_set_i16(NVS_handle, "accel_2_off_2", accel_2_offset[2]);

    ESP_LOGI(TAG_NVS,"Writing gyro_2_X offset... ");
    err = nvs_set_i16(NVS_handle, "gyro_2_off_0", gyro_2_offset[0]);
    
    ESP_LOGI(TAG_NVS,"Writing gyro_2_Y offset... ");
    err = nvs_set_i16(NVS_handle, "gyro_2_off_1", gyro_2_offset[1]);
  
    ESP_LOGI(TAG_NVS,"Writing gyro_2_Z offset... ");
    err = nvs_set_i16(NVS_handle, "gyro_2_off_2", gyro_2_offset[2]);

    ESP_LOGI(TAG_NVS,"Committing updates in NVS ... ");
    err = nvs_commit(NVS_handle);

    nvs_close(NVS_handle);
  }
}

/*******************************TASKS*************************************/
//displaying error codes at LED
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

static void read_and_process_data_from_mag(void * pvParameters)
{
  uint8_t i = 0;
  uint8_t mag_raw_values[6] = {0,0,0,0,0,0};
  int16_t magn_data[3]= {0,0,0};
  uint16_t total_vector = 0;    
    
  while(1) {
    
    if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) != 0)
    {
      IST8310_request_data();
      vTaskDelay(10/portTICK_PERIOD_MS);
      IST8310_read_data(mag_raw_values);
  
      magn_data[0] = mag_raw_values[1] << 8 | mag_raw_values[0]; //X
      magn_data[1] = mag_raw_values[3] << 8 | mag_raw_values[2]; //Y
      magn_data[2] = mag_raw_values[5] << 8 | mag_raw_values[4]; //Z
      total_vector = sqrt(magn_data[0]*magn_data[0] + magn_data[1] * magn_data[1] + magn_data[2] * magn_data[2]);

      ESP_LOGI(TAG_IST8310,"Mag values are %d, %d, %d    %d",magn_data[0],magn_data[1],magn_data[2],total_vector);
                
      xQueueSend(magnetometer_queue, magn_data, NULL);
    }
  }
}

//GPS UART processing
static void read_and_process_data_from_gps(void * pvParameters)
{
  uint8_t incoming_message_buffer_gps[NUMBER_OF_BYTES_TO_RECEIVE_FROM_GPS];
  uint16_t i = 0;
  uint16_t j = 0;
  int16_t pos = 0;
  unsigned char XOR = 0;
  unsigned char XOR_ch = 0;
  uint8_t asteriks_place = 0;
  unsigned char coma_places[13] = {0};
  UBaseType_t uxHighWaterMark;
  uart_event_t gps_uart_event;
  uint8_t toggle = 0;
  
  char latitude = 0;
  char longtitude = 0;
  bool gps_ok = 0;

  struct data_from_gps_to_main_struct gps_data;
  uint8_t gps_status_old = 0;

  ESP_LOGI(TAG_GPS,"Configuring GPS UART.....");
  gps_uart_config();
  ESP_LOGI(TAG_GPS,"GPS UART configured");
       
  while(1) {
  if(xQueueReceive(gps_queue_for_events, (void * )&gps_uart_event, (TickType_t)portMAX_DELAY))
  {
    switch (gps_uart_event.type) {
            case UART_PATTERN_DET:
                pos = uart_pattern_pop_pos(GPS_UART);
                ESP_LOGD(TAG_GPS, "[UART PATTERN DETECTED] pos: %d", pos);
                if (pos > 130) {
                  uart_flush_input(GPS_UART); 
                  xQueueReset(gps_queue_for_events);
                  }
                else 
                  {
                    int read_len = uart_read_bytes(GPS_UART, incoming_message_buffer_gps, pos+3, portMAX_DELAY);
                    ESP_LOGD(TAG_GPS, "Received in total %d bytes", read_len);
                    //for (i=0; i<read_len; i++) printf ("%c", incoming_message_buffer_gps[i]);
                    j = 0;
                    while ((incoming_message_buffer_gps[0] != '$') && (j < read_len)) {
                      for (i=0; i<read_len; i++) incoming_message_buffer_gps[i] = incoming_message_buffer_gps[i+1];
                      j++;
                     }
                    if (incoming_message_buffer_gps[4] == 'M') 
                      {
                      uart_flush(GPS_UART);
                      i = 1;
                      j = 0;
                      XOR = 0;
                      XOR_ch = 0;
                      asteriks_place = 0;
              
                      while((asteriks_place == 0)) {// && (i <= (read_len-3))) {
                          if (incoming_message_buffer_gps[i] == ',') {coma_places[j] = i; j++;}                   //counting commas and fixinging their numbers 
                          if (incoming_message_buffer_gps[i] == '*') asteriks_place = i;
                          if (asteriks_place == 0) XOR = XOR^incoming_message_buffer_gps[i];                          //calculating XOR of the message until * is found 
                          i++;               
                      }
                      
                      if (incoming_message_buffer_gps[asteriks_place+1] <= 0x39) XOR_ch = (incoming_message_buffer_gps[asteriks_place+1] & 0x0F) << 4;  //discovering if digit or letter
                      else XOR_ch = (incoming_message_buffer_gps[asteriks_place+1] - 55) << 4;
                      if (incoming_message_buffer_gps[asteriks_place+2] <= 0x39) XOR_ch |= (incoming_message_buffer_gps[asteriks_place+2] & 0x0F);
                      else XOR_ch |= (incoming_message_buffer_gps[asteriks_place+2] - 55);
                  
                      if (XOR == XOR_ch) {
                        //for (i=0;i<90;i++) printf("%c",incoming_message_buffer_gps[i] );
                        //printf("\n");
                        if  ((incoming_message_buffer_gps[asteriks_place-3] == 'A')||(incoming_message_buffer_gps[asteriks_place-3] == 'D')) //if mode A or D
                        { 
                        //Latitude, the format is ddmm.mmmmmmm
                        //Longitude, the format is dddmm.mmmmmmm

                        latitude = (incoming_message_buffer_gps[coma_places[2]+1] & 0x0F)*10 + (incoming_message_buffer_gps[coma_places[2]+2] & 0x0F);
                        longtitude = (incoming_message_buffer_gps[coma_places[4]+2] & 0x0F)*10 + (incoming_message_buffer_gps[coma_places[4]+3] & 0x0F);

                        gps_data.latitude_d = ((incoming_message_buffer_gps[coma_places[2]+3] & 0x0F)*1000000 + (incoming_message_buffer_gps[coma_places[2]+4] & 0x0F)*(long)100000 + (incoming_message_buffer_gps[coma_places[2]+6] & 0x0F)*10000 + (incoming_message_buffer_gps[coma_places[2]+7] & 0x0F)*1000 + (incoming_message_buffer_gps[coma_places[2]+8] & 0x0F)*100 + (incoming_message_buffer_gps[coma_places[2]+9] & 0x0F)*10 + (incoming_message_buffer_gps[coma_places[2]+10] & 0x0F)) *10 / 6;
                        gps_data.latitude_d += latitude * 10000000;

                        gps_data.longtitude_d = ((incoming_message_buffer_gps[coma_places[4]+4] & 0x0F)*1000000 + (incoming_message_buffer_gps[coma_places[4]+5] & 0x0F)*100000 + (incoming_message_buffer_gps[coma_places[4]+7] & 0x0F)*(long)10000 + (incoming_message_buffer_gps[coma_places[4]+8] & 0x0F)*1000 + (incoming_message_buffer_gps[coma_places[4]+9] & 0x0F)*100 + (incoming_message_buffer_gps[coma_places[4]+10] & 0x0F)*10 + (incoming_message_buffer_gps[coma_places[4]+11] & 0x0F)) *10 / 6;
                        gps_data.longtitude_d += longtitude * 10000000;
                        ESP_LOGI(TAG_GPS,"Lat is %" PRIu64 " Lon is %" PRIu64, gps_data.latitude_d, gps_data.longtitude_d);

                        gps_data.status = 1;

                        xQueueSend(gps_to_main_queue, (void *) &gps_data, NULL);

                        for (i=1;i<67;i++) incoming_message_buffer_gps[i] = 0;
                        //uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
                        //ESP_LOGD(TAG_GPS,"High watermark %d",  uxHighWaterMark);
                        }
                        else {ESP_LOGW(TAG_GPS,"Mode mismatch"); gps_data.status = 0;}
                      } else {ESP_LOGW(TAG_GPS, "CRC mismatch"); gps_data.status = 0;}
                    } else {ESP_LOGW(TAG_GPS,"Message out of phase"); gps_data.status = 0;}
                  
                  if (gps_data.status != gps_status_old )
                  {
                    if (gps_data.status) FL3195_set_pattern(4, 0,255,0);
                    else FL3195_set_pattern(4, 255,0,0);
                  }
                  gps_status_old = gps_data.status;
                  
                  }
                uart_flush(GPS_UART);
                xQueueReset(gps_queue_for_events);
                break;
            
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
//remote control data processing
static void read_and_process_data_from_RC(void * pvParameters) 
{
  uint16_t i,j;
  int16_t pos = 0;
  uint8_t remote_packets_counter = 0;
  uart_event_t remote_control_uart_event;
  uint8_t incoming_message_buffer_remote[NUMBER_OF_BYTES_TO_RECEIVE_FROM_RC * 2];
  uint16_t received_throttle;
  uint16_t received_pitch = 2000;                       //neutral position
                   //temporary variable to determine fixed angle of pitch when input signal is out of neutral range
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

  remote_control_data.altitude_hold_flag = 0;
  
  ESP_LOGI(TAG_RC,"Configuring RC UART.....");
  remote_control_uart_config();
  ESP_LOGI(TAG_RC,"RC UART configured");

  while(1) 
  {
    if(xQueueReceive(remote_control_queue_for_events, (void * )&remote_control_uart_event, (TickType_t)portMAX_DELAY)) 
    {     
      switch (remote_control_uart_event.type) 
      {
        case UART_PATTERN_DET:
          pos = uart_pattern_pop_pos(REMOTE_CONTROL_UART);
          //ESP_LOGD(TAG_RC, "[UART PATTERN DETECTED] pos: %d", pos);
          if (pos > 30)
          {
            uart_flush_input(REMOTE_CONTROL_UART); 
            xQueueReset(remote_control_queue_for_events);
            ESP_LOGW(TAG_RC, "more than 30, %d", pos);
          }
          else 
          {
            int read_len = uart_read_bytes(REMOTE_CONTROL_UART, incoming_message_buffer_remote, pos+1, 1);
            ESP_LOGD(TAG_RC, "Received in total %d bytes", read_len);
            //for (j=0;j<15;j++) printf ("%02x ",incoming_message_buffer_remote[j]);
            j = 0;
            while ((incoming_message_buffer_remote[0] != RC_MESSAGE_HEADER) && (j < read_len+1)) 
            {
              for (i=0; i<read_len; i++) incoming_message_buffer_remote[i] = incoming_message_buffer_remote[i+1];
              j++;
            }
            //for (j=0;j<13;j++) printf ("%02x ",incoming_message_buffer_remote[j]);
            if (incoming_message_buffer_remote[NUMBER_OF_BYTES_TO_RECEIVE_FROM_RC - 2] == dallas_crc8(incoming_message_buffer_remote, NUMBER_OF_BYTES_TO_RECEIVE_FROM_RC-2)) 
            {
              
              ESP_LOGD(TAG_RC, "CRC passed");
              uart_flush(REMOTE_CONTROL_UART);
              if (LED_status) {gpio_set_level(LED_GREEN, 0); LED_status=0;}
              else {gpio_set_level(LED_GREEN, 1);LED_status=1;}
              received_throttle = (incoming_message_buffer_remote[1] << 8) | incoming_message_buffer_remote[2];
              received_roll = ((incoming_message_buffer_remote[3] << 8) | incoming_message_buffer_remote[4]);               
              received_pitch = ((incoming_message_buffer_remote[5] << 8) | incoming_message_buffer_remote[6]);                
              received_yaw = ((incoming_message_buffer_remote[7] << 8) | incoming_message_buffer_remote[8]);
              remote_control_data.mode = ~incoming_message_buffer_remote[10];

              remote_control_data.received_throttle = 7836.0f + 48.97f * sqrt((double)received_throttle); //7836.0f + 81.86248443f * sqrt((double)received_throttle);

              if ((received_pitch > 360)&&( received_pitch < 1800)) remote_control_data.received_pitch = 0.02083333f*(float)received_pitch - 37.5f;
                else if ((received_pitch > 2245) && (received_pitch < 3741)) remote_control_data.received_pitch  = 0.02005348*(float)received_pitch - 45.020053f;
                else if (received_pitch >= 3741) remote_control_data.received_pitch  = 30.0;  //degrees
                else if (received_pitch <= 360) remote_control_data.received_pitch  = -30.0;
                else remote_control_data.received_pitch = 0;
//reversed signs             
              if ((received_roll > 360)&&( received_roll < 1800)) remote_control_data.received_roll = -0.02083333f*(float)received_roll + 37.5f;              
                else if ((received_roll > 2245) && (received_roll < 3741)) remote_control_data.received_roll = -0.02005348*(float)received_roll + 45.020053f; 
                else if (received_roll >= 3741) remote_control_data.received_roll = -30.0;
                else if (received_roll <= 360) remote_control_data.received_roll = 30.0;
                else remote_control_data.received_roll = 0;

              if ((received_yaw > 200)&&( received_yaw < 1848)) remote_control_data.received_yaw = -0.06068f*(float)received_yaw + 112.13592;
                else if ((received_yaw < 3896)&&( received_yaw > 2248)) remote_control_data.received_yaw = -0.06068f*(float)received_yaw + 136.40777;
                else if (received_yaw >= 3896) remote_control_data.received_yaw = -100.0;     //degrees per s
                else if (received_yaw <= 200) remote_control_data.received_yaw = 100.0;
                else remote_control_data.received_yaw = 0;

//filtering a bit received values to get smoother response              
              remote_control_data.received_throttle = remote_control_data.received_throttle * RC_FILTER_COEFF + rc_throttle_old * (1 - RC_FILTER_COEFF);
              rc_throttle_old = remote_control_data.received_throttle;

              remote_control_data.received_pitch = remote_control_data.received_pitch * RC_FILTER_COEFF + rc_pitch_old * (1 - RC_FILTER_COEFF);
              rc_pitch_old = remote_control_data.received_pitch;

              remote_control_data.received_roll = remote_control_data.received_roll * RC_FILTER_COEFF + rc_roll_old * (1 - RC_FILTER_COEFF);
              rc_roll_old = remote_control_data.received_roll;

              remote_control_data.received_yaw = remote_control_data.received_yaw * RC_FILTER_COEFF + rc_yaw_old * (1 - RC_FILTER_COEFF);
              rc_yaw_old = remote_control_data.received_yaw;


              if (incoming_message_buffer_remote[9] & 0b00001000) {                            //if roll negative values
              trim_roll = (((~(incoming_message_buffer_remote[9] & 0b00001111)) & 0b00001111) + 1) * -1;}
              else trim_roll = incoming_message_buffer_remote[9] & 0b00001111;

              if (incoming_message_buffer_remote[9] & 0b10000000) {                            //if pitch negative values
                trim_pitch = (((~((incoming_message_buffer_remote[9] >> 4) & 0b00001111)) & 0b00001111)  + 1) * -1;}
              else trim_pitch = (incoming_message_buffer_remote[9] >> 4) & 0b00001111; 
            
              if ((remote_control_data.received_throttle < 8400) && (remote_control_data.mode & 0x0001)) remote_control_data.engines_start_flag = 1;
              if (!(remote_control_data.mode & 0x01)) {remote_control_data.engines_start_flag = 0; remote_control_data.altitude_hold_flag = 0; set_allowed = 0;}

              if ((remote_control_data.mode & 0x02) && (set_allowed == 1) && (remote_control_data.engines_start_flag)) remote_control_data.altitude_hold_flag = 1;
              if (!(remote_control_data.mode & 0x02)) {remote_control_data.altitude_hold_flag = 0; set_allowed = 1;}
              
              if ((remote_control_data.mode & 0x08) ^ (mode_old & 0x08))  {
                if (remote_control_data.mode & 0x08) command_for_PCA9685 = 0x0103;
                else command_for_PCA9685 = 0x011A;
              xQueueSend(PCA9685_queue, &command_for_PCA9685, NULL);
              }
                
              xQueueSend(remote_control_to_main_queue, (void *) &remote_control_data, NULL);           
              
              mode_old = remote_control_data.mode;

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

        case UART_DATA: break;
       
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
//sending data to remote control
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
//LIDAR UART processing
#ifdef USING_LIDAR_UART
static void read_and_process_data_from_lidar(void * pvParameters)
{
  uint8_t incoming_message_buffer_lidar[NUMBER_OF_BYTES_TO_RECEIVE_FROM_LIDAR];
  uint16_t i = 0;
  uint16_t j = 0;
  uint16_t pos = 0;
  uint8_t sum = 0;
  struct data_from_lidar_to_main_struct lidar_data;


  uart_event_t lidar_uart_event;
  
  ESP_LOGI(TAG_LIDAR,"Configuring lidar UART.....");
  lidar_uart_config();
  ESP_LOGI(TAG_LIDAR,"lidar UART configured");
  
  //ESP_LOGI(TAG_LIDAR,"Configuring lidar for 20Hz");
  
    //uint8_t set_20_Hz_command[] = {0x5A, 0x06, 0x03, 0x14, 0x00, 0x77};
    //uint8_t save[] = {0x5A, 0x04, 0x11, 0x6F};
    //uart_write_bytes(LIDAR_UART, set_20_Hz_command, 6);
    //uart_write_bytes(LIDAR_UART, save, 4);
  //ESP_LOGI(TAG_LIDAR,"lidar is set for 20Hz");
       
  while(1) {
  if(xQueueReceive(lidar_queue_for_events, (void * )&lidar_uart_event, (TickType_t)portMAX_DELAY))
  {
    switch (lidar_uart_event.type) {
            case UART_PATTERN_DET:
                pos = uart_pattern_pop_pos(LIDAR_UART);
                ESP_LOGD(TAG_LIDAR, "[UART PATTERN DETECTED] pos: %d", pos);
                    int read_len = uart_read_bytes(LIDAR_UART, incoming_message_buffer_lidar, pos+9, portMAX_DELAY);
                    ESP_LOGD(TAG_LIDAR, "Received in total %d bytes", read_len);
                    xQueueReset(lidar_queue_for_events);
                    uart_flush(LIDAR_UART); 
                    //for (i=0; i<read_len; i++) printf ("%02x", incoming_message_buffer_lidar[i]);
                    //printf("\n");
                    j = 0;
                    for (i=0;i<8;i++) sum+= incoming_message_buffer_lidar[i];
                    //printf("%02x\n",sum);
                    if ((incoming_message_buffer_lidar[0] == 0x59) && (incoming_message_buffer_lidar[1] == 0x59) && (incoming_message_buffer_lidar[8] == sum))
                    {
                      lidar_data.height = (incoming_message_buffer_lidar[3] << 8) + incoming_message_buffer_lidar[2];
                      lidar_data.strength = (incoming_message_buffer_lidar[5] << 8) + incoming_message_buffer_lidar[4];
                      ESP_LOGI(TAG_LIDAR, "dist is %d\n",lidar_data.height);
                      xQueueSend(lidar_to_main_queue, (void *) &lidar_data, NULL); 
                    }
                uart_flush(LIDAR_UART);
                xQueueReset(lidar_queue_for_events);
                sum = 0;
                break;
            case UART_FIFO_OVF:
          ESP_LOGW(TAG_LIDAR, "hw fifo overflow");
          uart_flush_input(LIDAR_UART);
          xQueueReset(lidar_queue_for_events);
          break;

        case UART_BUFFER_FULL:
          ESP_LOGW(TAG_LIDAR, "ring buffer full");
          uart_flush_input(LIDAR_UART);
          xQueueReset(lidar_queue_for_events);
          break;

        case UART_DATA: break;
       
        case UART_BREAK:
          ESP_LOGW(TAG_LIDAR, "uart rx break");
          break;
        
        case UART_PARITY_ERR:
          ESP_LOGW(TAG_LIDAR, "uart parity error");
          break;
        
        case UART_FRAME_ERR:
          ESP_LOGW(TAG_LIDAR, "uart frame error");
          break;
                    
        default:
          ESP_LOGW(TAG_LIDAR, "unknown uart event type: %d", lidar_uart_event.type);
          uart_flush(LIDAR_UART);
          xQueueReset(lidar_queue_for_events);
            break;
    }
  }
}
}
#endif
//monitoring MCP23017 
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
        MCP23017_inputs_state = MCP23017_get_inputs_state(); 
        ESP_LOGI(TAG_MCP23017,"Current input state is %02x",MCP23017_inputs_state);  
      }
      
      if (MCP23017_external_request & 0b01000000) {                 //if bit6 is set that means 2 lower bit represents output to be set
        MCP23017_set_output(&MCP23017_current_outputs_state, MCP23017_external_request & 0b00001111);
        ESP_LOGI(TAG_MCP23017,"Got request to set output %d, output is set",MCP23017_external_request & 0b00001111);  
      }

      if (MCP23017_external_request & 0b00100000) {                 //if bit5 is set that means 2 lower bit represents output to be cleared
        MCP23017_clear_output(&MCP23017_current_outputs_state, MCP23017_external_request & 0b00001111);
        ESP_LOGI(TAG_MCP23017,"Got request to clear output %d, output is cleared",MCP23017_external_request & 0b00001111);
      }
    }
  }
}
//control PMW3901
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
        PCA9685_send(pwm_value, output_number);
        ESP_LOGI(TAG_PCA9685,"Output #%d set to %d%%",output_number, pwm_value);
      }  
    
    }
  }
}
//writing logs to flash
static void writing_logs_to_flash(void * pvParameters)
{
  uint8_t buffer[LOGS_BYTES_PER_STRING] = {0x0D}; 
  uint16_t column_address = 0;
  uint16_t page_address = 0;
  while(1) 
  {
    if (xQueueReceive(W25N01_queue, buffer, portMAX_DELAY))
    {
      W25N_random_program_data_load(column_address, buffer, LOGS_BYTES_PER_STRING);   //loading data to page buffer
      column_address = column_address + LOGS_BYTES_PER_STRING;
      if (column_address >= 1975)   //79 bytes per 1ms, 25 samples per page
      {
        column_address = 0;
        W25N_program_execute(page_address);       //65536 pages, total 26 minutes
        page_address++;
      }
    }
  }
}
//reading mag data and sending them to main via queue
static void blinking_flight_lights(void * pvParameters)
{
  uint32_t blinking_mode = 0;

  while(1) 
    {

     xTaskNotifyWait(0,0,&blinking_mode,NULL);
  
      if (blinking_mode == 0)
      {
        gpio_set_level(GREEN_FLIGHT_LIGHTS, 1);
        gpio_set_level(RED_FLIGHT_LIGHTS, 1);
        vTaskDelay(50/portTICK_PERIOD_MS);
        
        gpio_set_level(GREEN_FLIGHT_LIGHTS, 0);
        gpio_set_level(RED_FLIGHT_LIGHTS, 0);
        vTaskDelay(250/portTICK_PERIOD_MS);
      }

      if (blinking_mode == 1)
      {
        gpio_set_level(GREEN_FLIGHT_LIGHTS, 1);
        vTaskDelay(50/portTICK_PERIOD_MS);
        gpio_set_level(GREEN_FLIGHT_LIGHTS, 0);
        vTaskDelay(100/portTICK_PERIOD_MS);
        gpio_set_level(GREEN_FLIGHT_LIGHTS, 1);
        vTaskDelay(50/portTICK_PERIOD_MS);
        gpio_set_level(GREEN_FLIGHT_LIGHTS, 0);
        vTaskDelay(500/portTICK_PERIOD_MS);

        gpio_set_level(RED_FLIGHT_LIGHTS, 1);
        vTaskDelay(50/portTICK_PERIOD_MS);
        gpio_set_level(RED_FLIGHT_LIGHTS, 0);
        vTaskDelay(1500/portTICK_PERIOD_MS);
      } 
    }
}
//reading logs from external memory
static void reading_logs_from_external_flash(void * pvParameters)
{
  while(1) {
    W25N_read_and_print_all();
    while (1) 
    {
      gpio_set_level(LED_RED, 1);
      vTaskDelay(500/portTICK_PERIOD_MS); 
      gpio_set_level(LED_RED, 0);
      vTaskDelay(500/portTICK_PERIOD_MS); 
    }; 
  }
}
//displaying statistics
#ifdef USING_PERFORMANCE_MESUREMENT
static void performace_monitor(void * pvParameters)
{
  char statbuf[512];
  
  while(1)
  {
    vTaskGetRunTimeStats(statbuf);
    printf("%s\n",statbuf);
    vTaskDelay(5000/portTICK_PERIOD_MS);
  } 
}
#endif
//reading INA219 data
static void read_and_process_data_from_INA219(void * pvParameters)
{
  float INA219_data[4] = {0.0, 0.0, 0.0, 0.0};
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

static void main_flying_cycle(void * pvParameters)
{
  extern volatile float q0;// = 34.56; //0x420A3D71
  extern volatile float q1;// = 0.56; //0x3F0F5C29
  extern volatile float q2;// = 0.996; //0x3F7EF9DB
  extern volatile float q3;// = -0.001; //0xBA83126F
  
  uint8_t sensor_data_1[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
  uint8_t sensor_data_2[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};

  uint8_t sensor_data_1_old[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
  uint8_t sensor_data_2_old[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
  
  int16_t accel_raw_1[3] = {31562,13,7613};  //0x7B4A 0x000D 0x1DBD
  int16_t gyro_raw_1[3] = {9568,28999,-31444}; //0x2560 0x7147 0x852C
  int16_t accel_raw_2[3] = {31562,13,7613};  //0x7B4A 0x000D 0x1DBD
  int16_t gyro_raw_2[3] = {9568,28999,-31444}; //0x2560 0x7147 0x852C
          
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

  uint16_t counter = 0;
  uint16_t i = 0;

  int16_t accel_1_offset[3] = {0,0,0};
  int16_t accel_2_offset[3] = {0,0,0};
  int16_t gyro_1_offset[3] = {0,0,0};
  int16_t gyro_2_offset[3] = {0,0,0};

  int16_t magnet_raw[3] = {0,0,0};
  int16_t magnet_max[3] = {0,0,0};
  int16_t magnet_min[3] = {0,0,0};
  float magnet_hard_bias[3] = {0.0,0.0,0.0};
  float magnet_radius[3] = {0.0,0.0,0.0};
  float magnet_avg_radius = 0.0;
  float magnet_coeff[3] = {0.0,0.0,0.0};
  float magnet_converted[3] = {0.0,0.0,0.0};
     
  float gyro_converted_1[3] = {0.0,0.0,0.0};
  float accel_converted_1[3] = {0.0,0.0,0.0};
  float gyro_converted_2[3] = {0.0,0.0,0.0};
  float accel_converted_2[3] = {0.0,0.0,0.0};

  float accel_converted_accumulated_1[3] = {0.0,0.0,0.0};
  float gyro_converted_accumulated_1[3] = {0.0,0.0,0.0};

  float accel_converted_accumulated_2[3] = {0.0,0.0,0.0};
  float gyro_converted_accumulated_2[3] = {0.0,0.0,0.0};

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
  float Kp_yaw_rate = 100;//24;
  float Ki_yaw_rate = 0.01;
  float Kd_yaw_rate = 0;//30;
  float error_yaw_rate = 0;
  float error_yaw_angle = 0;
  float error_yaw_angle_old = 0;
  float error_yaw_rate_old = 0;
  float integral_yaw_error_rate = 0;
  float pid_yaw_angle = 0;
  float pid_yaw_rate = 0;
  float diff_yaw_error = 0.0;


  float engine[4] = {ENGINE_PWM_MIN_DUTY,ENGINE_PWM_MIN_DUTY,ENGINE_PWM_MIN_DUTY,ENGINE_PWM_MIN_DUTY}; //00001ABE 0x00001E83 0x0000000C 0x000025D4
  float engine_filtered[4] = {ENGINE_PWM_MIN_DUTY,ENGINE_PWM_MIN_DUTY,ENGINE_PWM_MIN_DUTY,ENGINE_PWM_MIN_DUTY};
  float engine_filter_pool [LENGTH_OF_ESC_FILTER][6] = {0};                                   //data pool for ESC input filter
  float accum_float;

  uint8_t command_for_MCP23017;
  uint16_t command_for_PCA9685;
  uint8_t blink = 0;
  uint8_t blink_gps = 0;
  uint8_t blink_mag = 0;
  uint8_t blink_rc = 0;
  UBaseType_t uxHighWaterMark;

  struct data_from_rc_to_main_struct rc_fresh_data;
    rc_fresh_data.mode = 0;
    rc_fresh_data.engines_start_flag = 0;
    rc_fresh_data.altitude_hold_flag = 0;
  struct data_from_gps_to_main_struct gps_fresh_data;
  struct data_from_main_to_rc_struct data_to_send_to_rc;
  float rc_throttle_filtered = 0;

  uint32_t remote_control_lost_comm_counter = 0;

  float number_of_IMU_calibration_counts = NUMBER_OF_IMU_CALIBRATION_COUNTS;
  float number_of_magnetometer_calibration_counts = NUMBER_OF_MAGNETOMETER_CALIBRATION_COUNTS; 
  unsigned int cycle_counter = 0;
//logging related variables
  uint8_t logs_buffer[LOGS_BYTES_PER_STRING] = {0x0D}; 
  uint64_t start_time = 0;
  uint32_t timestamp = 0;
  uint64_t timer_value = 0;
  //uint64_t cycle_start_time = 0;
  //uint64_t cycle_end_time = 0;
  uint64_t large_counter = 0;
  uint8_t *p_to_uint8;
  uint8_t flags_byte = 0;
//altitude hold related variables
  bool altitude_hold_mode_enabled = 0;
  float altitude_setpoint = 0;
  float current_altitude = 0;
  float error_altitude = 0;
  float previous_error_altitude = 0;
  float Kp_alt = 5.0;//10
  float Kd_alt = 100.0;//150
  float Ki_alt = 0.000;//0
  float pid_altitude = 0;
  float integral_alt_error = 0;
  float alt_hold_initial_throttle = 0;
  float current_altitude_old = 0;

#ifdef USING_LIDAR_UART
    struct data_from_lidar_to_main_struct lidar_fresh_data;
      lidar_fresh_data.height = 0;
      lidar_fresh_data.strength = 0;
#endif

  float INA219_fresh_data[4];
  uint8_t calibration_flag = 0;

  uint8_t IMU_suspention_event[5] = {0,0,0,0,0};
  uint32_t IMU_interrupt_status = 0;
  uint8_t test_0 = 0;
  uint8_t test_1 = 0xFF;


  void Convert_Q_to_degrees(void) {
    
    float a12,a22,a31,a32,a33 = 0;

    a12 =   2.0f * (q1 * q2 + q0 * q3);
    a22 =   q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    a31 =   2.0f * (q0 * q1 + q2 * q3);
    a32 =   2.0f * (q1 * q3 - q0 * q2);
    a33 =   q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    pitch = - asin(a32);
    roll  = atan2(a31, a33);
    yaw   = atan2(a12, a22);

    pitch *= -57.29577951;    //pitch *= (180.0 / (float)PI) * -1.0;
    roll  *= 57.29577951;             //roll  *= 180.0 / (float)PI;
    roll = roll + 180.0;
    if (roll > 90) roll -= 360.0;
    yaw   *= 57.29577951;             //yaw   *= 180.0 / (float)PI;
    //yaw   -= 9.4; // Declination at Moscow Russia 29.01.2019               9.4 for Smolensk
    yaw   +=180.0;    //making [0..360] from [-180..180]
    //if (yaw < 0) yaw   += 360.0;     
  }

  void calculate_pids_2(void) {
//outer (angle) pitch cycle
    error_pitch_angle = rc_fresh_data.received_pitch - pitch;    
    integral_pitch_error_angle = integral_pitch_error_angle + Ki_pitch_angle * error_pitch_angle;
    if (integral_pitch_error_angle > 1000.0) {integral_pitch_error_angle = 1000.0; }
    if (integral_pitch_error_angle < -1000.0) {integral_pitch_error_angle =-1000.0;}

    pid_pitch_angle = Kp_pitch_angle * error_pitch_angle + integral_pitch_error_angle + Kd_pitch_angle * (error_pitch_angle - error_pitch_angle_old);     
    if (pid_pitch_angle > 2000.0) {pid_pitch_angle = 2000.0; }
    if (pid_pitch_angle < -2000.0) {pid_pitch_angle = -2000.0; }
    error_pitch_angle_old = error_pitch_angle; 
  
//inner (rate) pitch cycle
    gyro_pitch = 0.1 * (((gyro_converted_1[0] + gyro_converted_2[1]) / 2.0) * (180.0 / (float)PI)) + 0.9 * gyro_pitch_old;
    gyro_pitch_old = gyro_pitch;

    error_pitch_rate = pid_pitch_angle - gyro_pitch;
    //error_pitch_rate = rc_fresh_data.received_pitch - gyro_pitch;
    integral_pitch_error_rate = integral_pitch_error_rate + Ki_pitch_rate * error_pitch_rate;
    if (integral_pitch_error_rate > 1000.0) {integral_pitch_error_rate = 1000.0; }
    if (integral_pitch_error_rate < -1000.0) {integral_pitch_error_rate = -1000.0; }
    diff_pitch_error_rate = Kd_pitch_rate * (error_pitch_rate - error_pitch_rate_old); 
    pid_pitch_rate = Kp_pitch_rate * error_pitch_rate + integral_pitch_error_rate + diff_pitch_error_rate;
    if (pid_pitch_rate > 3000.0) {pid_pitch_rate = 3000.0; }
    if (pid_pitch_rate < -3000.0) {pid_pitch_rate = -3000.0;}
    error_pitch_rate_old = error_pitch_rate;

//outer (angle) roll cycle
    error_roll_angle = rc_fresh_data.received_roll - roll; 
    integral_roll_error_angle = integral_roll_error_angle + Ki_roll_angle * error_roll_angle;
    if (integral_roll_error_angle > 1000.0) integral_roll_error_angle = 1000.0;
    if (integral_roll_error_angle < -1000.0) integral_roll_error_angle =-1000.0;   
    pid_roll_angle = Kp_roll_angle * error_roll_angle + integral_roll_error_angle + Kd_roll_angle * (error_roll_angle - error_roll_angle_old);    
    if (pid_roll_angle > 2000.0) pid_roll_angle = 2000.0;
    if (pid_roll_angle < -2000.0) pid_roll_angle = -2000.0;
    error_roll_angle_old = error_roll_angle;

//inner (rate) roll cycle
    gyro_roll = 0.1 * (((gyro_converted_1[1] - gyro_converted_2[0]) / 2.0 ) * (180.0 / (float)PI)) + 0.9 * gyro_roll_old;
    gyro_roll_old = gyro_roll;
    error_roll_rate = pid_roll_angle - gyro_roll;
    //error_roll_rate = rc_fresh_data.received_roll - gyro_roll;
    integral_roll_error_rate = integral_roll_error_rate + Ki_roll_rate * error_roll_rate;
    if (integral_roll_error_rate > 1000.0) integral_roll_error_rate = 1000.0; 
    if (integral_roll_error_rate < -1000.0) integral_roll_error_rate = -1000.0;
    diff_roll_error_rate = Kd_roll_rate * (error_roll_rate - error_roll_rate_old); 
    pid_roll_rate = Kp_roll_rate * error_roll_rate + integral_roll_error_rate + diff_roll_error_rate;
    if (pid_roll_rate > 3000.0) pid_roll_rate = 3000.0;
    if (pid_roll_rate < -3000.0) pid_roll_rate = -3000.0;
    error_roll_rate_old = error_roll_rate;

    yaw_setpoint = yaw_setpoint + 0.0016 * rc_fresh_data.received_yaw;
    if (yaw_setpoint >= 360) yaw_setpoint = yaw_setpoint - 360.0;
    if (yaw_setpoint < 0) yaw_setpoint = yaw_setpoint + 360.0;

//outer (angle) yaw cycle
    error_yaw_angle = -yaw_setpoint + yaw;
    if (error_yaw_angle <= -180.0) error_yaw_angle = error_yaw_angle + 360.0; 
    if (error_yaw_angle >= 180.0)  error_yaw_angle = error_yaw_angle - 360.0;

    integral_yaw_error_angle = integral_yaw_error_angle + Ki_yaw_angle * error_yaw_angle;
    if (integral_yaw_error_angle > 500.0) integral_yaw_error_angle = 500.0;
    if (integral_yaw_error_angle < -500.0) integral_yaw_error_angle =-500.0;
    pid_yaw_angle = Kp_yaw_angle * error_yaw_angle + integral_yaw_error_angle + Kd_yaw_angle * (error_yaw_angle - error_yaw_angle_old);  ;
    if (pid_yaw_angle > 1000.0)  pid_yaw_angle = 1000.0;
    if (pid_yaw_angle < -1000.0) pid_yaw_angle = -1000.0;
    error_yaw_angle_old = error_yaw_angle;
   
//inner yaw cycle (rate only)  
    error_yaw_rate = pid_yaw_angle - ((((gyro_converted_1[2] * (-1.0)) - gyro_converted_2[2]) / 2.0) * (180.0 / (float)PI));
    //error_yaw_rate = rc_fresh_data.received_yaw - ((((gyro_converted_1[2] * (-1.0)) - gyro_converted_2[2]) / 2.0) * (180.0 / (float)PI));
    integral_yaw_error_rate = integral_yaw_error_rate + Ki_yaw_rate * error_yaw_rate;
    if (integral_yaw_error_rate > 1000.0) integral_yaw_error_rate = 1000.0; 
    if (integral_yaw_error_rate < -1000.0) integral_yaw_error_rate = -1000.0;
    diff_yaw_error = Kd_yaw_rate * (error_yaw_rate - error_yaw_rate_old); 
    pid_yaw_rate = Kp_yaw_rate * error_yaw_rate + integral_yaw_error_rate + diff_yaw_error;
    if (pid_yaw_rate > 3000.0) pid_yaw_rate = 3000.0;
    if (pid_yaw_rate < -3000.0) pid_yaw_rate = -3000.0;
    error_yaw_rate_old = error_yaw_rate;

    engine[0] = rc_fresh_data.received_throttle + pid_pitch_rate + pid_roll_rate - pid_yaw_rate;
    engine[1] = rc_fresh_data.received_throttle + pid_pitch_rate - pid_roll_rate + pid_yaw_rate;
    engine[2] = rc_fresh_data.received_throttle - pid_pitch_rate - pid_roll_rate - pid_yaw_rate;
    engine[3] = rc_fresh_data.received_throttle - pid_pitch_rate + pid_roll_rate + pid_yaw_rate;

    //engine[0] *= 2063.0 / data_to_send_to_rc.power_voltage_value; //2063 - ADC value at minimum voltage level 9,9V
    //engine[1] *= 2063.0 / data_to_send_to_rc.power_voltage_value;
    //engine[2] *= 2063.0 / data_to_send_to_rc.power_voltage_value;
    //engine[3] *= 2063.0 / data_to_send_to_rc.power_voltage_value;
    
    for (i=0;i<4;i++) { if (engine[i] > 13106.0) engine[i] = 13106.0;}  //upper limit
    for (i=0;i<4;i++) { if (engine[i] < ENGINE_PWM_MIN_DUTY + 150) engine[i] = ENGINE_PWM_MIN_DUTY + 150;}    //keep motor running
  }
  
  void ESC_input_data_filter(void) {                                                                                                                        // P,T
    uint8_t i,j;
    for (i=0;i<LENGTH_OF_ESC_FILTER;i++) {
        for (j=0;j<4;j++) engine_filter_pool[i][j] = engine_filter_pool[i+1][j];                    //shifting values, new - higher index
      }
    engine_filter_pool[LENGTH_OF_ESC_FILTER-1][0] = engine[0];                            // inputing new values
    engine_filter_pool[LENGTH_OF_ESC_FILTER-1][1] = engine[1];
    engine_filter_pool[LENGTH_OF_ESC_FILTER-1][2] = engine[2];
    engine_filter_pool[LENGTH_OF_ESC_FILTER-1][3] = engine[3];

    for (i=0;i<LENGTH_OF_ESC_FILTER;i++) accum_float +=  engine_filter_pool[i][0];             //calculating averages
    engine_filtered[0] = accum_float / (float)LENGTH_OF_ESC_FILTER;
    accum_float = 0;

    for (i=0;i<LENGTH_OF_ESC_FILTER;i++) accum_float +=  engine_filter_pool[i][1];
    engine_filtered[1] = accum_float / (float)LENGTH_OF_ESC_FILTER;
    accum_float = 0;
    
    for (i=0;i<LENGTH_OF_ESC_FILTER;i++) accum_float +=  engine_filter_pool[i][2];
    engine_filtered[2] = accum_float / (float)LENGTH_OF_ESC_FILTER;
    accum_float = 0;
    
    for (i=0;i<LENGTH_OF_ESC_FILTER;i++) accum_float +=  engine_filter_pool[i][3];
    engine_filtered[3] = accum_float / (float)LENGTH_OF_ESC_FILTER;
    accum_float = 0;

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

  void prepare_logs(void) {
    p_to_uint8 = &timestamp;
    logs_buffer[0] = *p_to_uint8;
    logs_buffer[1] = *(p_to_uint8+1);
    logs_buffer[2] = *(p_to_uint8+2);
    logs_buffer[3] = *(p_to_uint8+3);

    p_to_uint8 = &accel_raw_1[0];       //raw accel_1 X
    logs_buffer[4] = *p_to_uint8;
    logs_buffer[5] = *(p_to_uint8+1);

    p_to_uint8 = &accel_raw_1[1];       //raw accel_1 Y
    logs_buffer[6] = *p_to_uint8;
    logs_buffer[7] = *(p_to_uint8+1);

    p_to_uint8 = &accel_raw_1[2];       //raw accel_1 Z
    logs_buffer[8] = *p_to_uint8;
    logs_buffer[9] = *(p_to_uint8+1);

    p_to_uint8 = &gyro_raw_1[0];        //raw gyro_1 X
    logs_buffer[10] = *p_to_uint8;
    logs_buffer[11] = *(p_to_uint8+1);

    p_to_uint8 = &gyro_raw_1[1];        //raw gyro_1 Y
    logs_buffer[12] = *p_to_uint8;
    logs_buffer[13] = *(p_to_uint8+1);

    p_to_uint8 = &gyro_raw_1[2];        //raw gyro_1 Z
    logs_buffer[14] = *p_to_uint8;
    logs_buffer[15] = *(p_to_uint8+1);

    p_to_uint8 = &q0;                    //q0
    logs_buffer[16] = *p_to_uint8;
    logs_buffer[17] = *(p_to_uint8+1);
    logs_buffer[18] = *(p_to_uint8+2);
    logs_buffer[19] = *(p_to_uint8+3);

    p_to_uint8 = &q1;                     //q1
    logs_buffer[20] = *p_to_uint8;
    logs_buffer[21] = *(p_to_uint8+1);
    logs_buffer[22] = *(p_to_uint8+2);
    logs_buffer[23] = *(p_to_uint8+3);

    p_to_uint8 = &q2;                     //q2
    logs_buffer[24] = *p_to_uint8;
    logs_buffer[25] = *(p_to_uint8+1);
    logs_buffer[26] = *(p_to_uint8+2);
    logs_buffer[27] = *(p_to_uint8+3);

    p_to_uint8 = &q3;                      //q3
    logs_buffer[28] = *p_to_uint8;
    logs_buffer[29] = *(p_to_uint8+1);
    logs_buffer[30] = *(p_to_uint8+2);
    logs_buffer[31] = *(p_to_uint8+3);

    p_to_uint8 = &pitch;
    logs_buffer[32] = *p_to_uint8;
    logs_buffer[33] = *(p_to_uint8+1);
    logs_buffer[34] = *(p_to_uint8+2);
    logs_buffer[35] = *(p_to_uint8+3);

    p_to_uint8 = &roll;
    logs_buffer[36] = *p_to_uint8;
    logs_buffer[37] = *(p_to_uint8+1);
    logs_buffer[38] = *(p_to_uint8+2);
    logs_buffer[39] = *(p_to_uint8+3);

    p_to_uint8 = &yaw;
    logs_buffer[40] = *p_to_uint8;
    logs_buffer[41] = *(p_to_uint8+1);
    logs_buffer[42] = *(p_to_uint8+2);
    logs_buffer[43] = *(p_to_uint8+3);

    p_to_uint8 = &rc_fresh_data.received_throttle;
    logs_buffer[44] = *p_to_uint8;
    logs_buffer[45] = *(p_to_uint8+1);
    logs_buffer[46] = *(p_to_uint8+2);
    logs_buffer[47] = *(p_to_uint8+3);

    p_to_uint8 = &rc_fresh_data.received_pitch;
    logs_buffer[48] = *p_to_uint8;
    logs_buffer[49] = *(p_to_uint8+1);
    logs_buffer[50] = *(p_to_uint8+2);
    logs_buffer[51] = *(p_to_uint8+3);

    p_to_uint8 = &rc_fresh_data.received_roll;
    logs_buffer[52] = *p_to_uint8;
    logs_buffer[53] = *(p_to_uint8+1);
    logs_buffer[54] = *(p_to_uint8+2);
    logs_buffer[55] = *(p_to_uint8+3);

    p_to_uint8 = &rc_fresh_data.received_yaw;
    logs_buffer[56] = *p_to_uint8;
    logs_buffer[57] = *(p_to_uint8+1);
    logs_buffer[58] = *(p_to_uint8+2);
    logs_buffer[59] = *(p_to_uint8+3);

    p_to_uint8 = &rc_fresh_data.mode;        
    logs_buffer[60] = *p_to_uint8;
    logs_buffer[61] = *(p_to_uint8+1);
    
    p_to_uint8 = &engine_filtered[0];
    logs_buffer[62] = *p_to_uint8;
    logs_buffer[63] = *(p_to_uint8+1);
    logs_buffer[64] = *(p_to_uint8+2);
    logs_buffer[65] = *(p_to_uint8+3);

    p_to_uint8 = &engine_filtered[1];
    logs_buffer[66] = *p_to_uint8;
    logs_buffer[67] = *(p_to_uint8+1);
    logs_buffer[68] = *(p_to_uint8+2);
    logs_buffer[69] = *(p_to_uint8+3);

    p_to_uint8 = &engine_filtered[2];
    logs_buffer[70] = *p_to_uint8;
    logs_buffer[71] = *(p_to_uint8+1);
    logs_buffer[72] = *(p_to_uint8+2);
    logs_buffer[73] = *(p_to_uint8+3);

    p_to_uint8 = &engine_filtered[3];
    logs_buffer[74] = *p_to_uint8;
    logs_buffer[75] = *(p_to_uint8+1);
    logs_buffer[76] = *(p_to_uint8+2);
    logs_buffer[77] = *(p_to_uint8+3);

    if (rc_fresh_data.engines_start_flag) flags_byte |= 0b00000001;   //bit0 - engine start flag
      else flags_byte &= 0b11111110;

    if (remote_control_lost_comm_counter == RC_NO_COMM_DELAY_MAIN_CYCLES) flags_byte |= 0b00000010; //bit1 - comm lost flag
      else flags_byte &= 0b11111101;

    logs_buffer[78] = flags_byte;
}

void NVS_reading_calibration_values(void)
{

  // Initialize NVS ESP_LOGI(TAG_INIT,
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      // NVS partition was truncated and needs to be erased
      // Retry nvs_flash_init
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
  }
  ESP_ERROR_CHECK( err );

  ESP_LOGI(TAG_NVS,"Opening Non-Volatile Storage (NVS) handle... ");
  nvs_handle_t NVS_handle;
  err = nvs_open("storage", NVS_READWRITE, &NVS_handle);
  if (err != ESP_OK) {
      ESP_LOGE(TAG_NVS,"Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } else {
        ESP_LOGI(TAG_NVS,"Done");

  // Read
  ESP_LOGI(TAG_NVS,"Reading calibration values from NVS ... ");
  
  err = nvs_get_i16(NVS_handle, "accel_1_off_0", &accel_1_offset[0]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGI(TAG_NVS,"accel_1_offset[0] = %d", accel_1_offset[0]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"accel_1_offset[0] value is not initialized yet!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "accel_1_off_1", &accel_1_offset[1]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGI(TAG_NVS,"accel_1_offset[1] = %d", accel_1_offset[1]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"accel_1_offset[1] value is not initialized yet!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "accel_1_off_2", &accel_1_offset[2]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGI(TAG_NVS,"accel_1_offset[2] = %d", accel_1_offset[2]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"accel_1_offset[2] value is not initialized yet!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "gyro_1_off_0", &gyro_1_offset[0]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGI(TAG_NVS,"gyro_1_offset[0] = %d", gyro_1_offset[0]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"gyro_1_offset[0] value is not initialized yet!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "gyro_1_off_1", &gyro_1_offset[1]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGI(TAG_NVS,"gyro_1_offset[1] = %d", gyro_1_offset[1]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"gyro_1_offset[1] value is not initialized yet!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "gyro_1_off_2", &gyro_1_offset[2]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGI(TAG_NVS,"gyro_1_offset[2] = %d", gyro_1_offset[2]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"gyro_1_offset[2] value is not initialized yet!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "accel_2_off_0", &accel_2_offset[0]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGI(TAG_NVS,"accel_2_offset[0] = %d", accel_2_offset[0]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"accel_2_offset[0] value is not initialized yet!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "accel_2_off_1", &accel_2_offset[1]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGI(TAG_NVS,"accel_2_offset[1] = %d", accel_2_offset[1]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"accel_2_offset[1] value is not initialized yet!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "accel_2_off_2", &accel_2_offset[2]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGI(TAG_NVS,"accel_2_offset[2] = %d", accel_2_offset[2]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"accel_2_offset[2] value is not initialized yet!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "gyro_2_off_0", &gyro_2_offset[0]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGI(TAG_NVS,"gyro_2_offset[0] = %d", gyro_2_offset[0]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"gyro_2_offset[0] value is not initialized yet!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "gyro_2_off_1", &gyro_2_offset[1]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGI(TAG_NVS,"gyro_2_offset[1] = %d", gyro_2_offset[1]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"gyro_2_offset[1] value is not initialized yet!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  err = nvs_get_i16(NVS_handle, "gyro_2_off_2", &gyro_2_offset[2]); 
  switch (err) {
      case ESP_OK:
          ESP_LOGI(TAG_NVS,"gyro_2_offset[2] = %d", gyro_2_offset[2]);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG_NVS,"gyro_2_offset[2] value is not initialized yet!");
          break;
      default :
          ESP_LOGE(TAG_NVS,"Error (%s) reading!\n", esp_err_to_name(err));
  }

  nvs_close(NVS_handle);
  }
}



//**************************************************************************************************************************************************************************************** */

 

if (!(MCP23017_get_inputs_state() & 0b00010000))  //DI4 - IMUs calibration
{
  calibration_flag = 1;
  ESP_LOGI(TAG_FLY,"Jumper #4 is set, starting IMUs calibration .....");
  }
  
else 
{
  NVS_reading_calibration_values();
    
  if ((gyro_1_offset[0]  || gyro_1_offset[1] || gyro_1_offset[2] || accel_1_offset[0] || accel_1_offset[1] || accel_1_offset[2]) == 0)
  {
    ESP_LOGE(TAG_FLY,"IMUs are not calibrated, please set J4 and calibrate\n");
    uint16_t error_code = 10;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);}
  }
}

vTaskDelay(100/portTICK_PERIOD_MS);       //without these delay assertion fails

ESP_LOGI(TAG_FLY,"creating and starting IMU#1 suspension timer.....");
Create_and_start_IMU_1_suspension_Timer();
ESP_LOGI(TAG_FLY,"IMU#1 suspension timer is created and started\n");

ESP_LOGI(TAG_FLY,"creating and starting IMU#2 suspension timer.....");
Create_and_start_IMU_2_suspension_Timer();
ESP_LOGI(TAG_FLY,"IMU#2 suspension timer is created and started\n");

ESP_LOGI(TAG_FLY,"Activating interrupts at IMUs and MCP pins.....");
configure_pins_for_interrupt();
ESP_LOGI(TAG_FLY,"GPIO interrupt pins configured\n");


   

while(1) {
    
      IMU_interrupt_status = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      
      if (IMU_interrupt_status != 0) {
      
      gpio_set_level(LED_RED, 0);

      //cycle_start_time = get_time();
      cycle_counter++;

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

//testing data from sensor_1
      for (i=0;i<14;i++) 
      test_0 = 0;
      test_1 = 0xFF;
      {
        test_0 = test_0 | sensor_data_1[i]; //bitwise operations 
        test_1 = test_1 & sensor_data_1[i];
      }
      if ((test_0 == 0) || (test_1 == 0xFF)) 
      {
        for (i=0;i<14;i++) sensor_data_1[i] = sensor_data_2[i]; //if yes - replaceing with sensor 2 values
      }
//testing data from sensor_2
      test_0 = 0;
      test_1 = 0xFF;
      for (i=0;i<14;i++) 
      {
        test_0 = test_0 | sensor_data_2[i]; //bitwise operations 
        test_1 = test_1 & sensor_data_2[i];
      }
      if ((test_0 == 0) || (test_1 == 0xFF)) 
      {
        for (i=0;i<14;i++) sensor_data_2[i] = sensor_data_1[i]; //if yes - replaceing with sensor 2 values
      }

      //mpu_temp_1 = (sensor_data_1[6] << 8) | sensor_data_1[7];
      //mpu_temp_1 = (mpu_temp_1 / 340.0) + 36.53;
      
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

         
      if ((calibration_flag) && (cycle_counter < NUMBER_OF_IMU_CALIBRATION_COUNTS))
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

      if ((calibration_flag) && (cycle_counter == NUMBER_OF_IMU_CALIBRATION_COUNTS))
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

        ESP_LOGI(TAG_INIT,"Calibration of IMUs is finished. Please remove the jumper and reset the vehicle.");
      
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
   
if (!(calibration_flag))
{

      //if(xQueueReceive(IMU_monitoring_timer_to_main_queue, IMU_suspention_event, 0)) {printf("%d", IMU_suspention_event[0]); gpio_set_level(LED_BLUE, 0);}

      for (i=0;i<3;i++) 
        {
          accel_converted_1[i] = ((float)accel_raw_1[i] - (float)accel_1_offset[i]) / 8192.0;         // in G    8192
          gyro_converted_1[i]  = (((float)gyro_raw_1[i] - (float)gyro_1_offset[i]) / 131.0) * ((float)PI / 180.0);   //in rads

          accel_converted_2[i] = ((float)accel_raw_2[i] - (float)accel_2_offset[i]) / 8192.0;         // in G    8192
          gyro_converted_2[i]  = (((float)gyro_raw_2[i] - (float)gyro_2_offset[i]) / 131.0) * ((float)PI / 180.0);   //in rads
#ifdef USING_MAG_DATA
          magnet_converted[i] = ((float)magnet_raw[i] - magnet_hard_bias[i]) * magnet_coeff[i];
#endif
          
          accel_converted_accumulated_1[i] += accel_converted_1[i];
          gyro_converted_accumulated_1[i] += gyro_converted_1[i];

          accel_converted_accumulated_2[i] += accel_converted_2[i];
          gyro_converted_accumulated_2[i] += gyro_converted_2[i];
        
        }
//********************************************************************************************************************************************************        
      if ((large_counter % PID_LOOPS_RATIO) == 0) { 
        for (i=0;i<MADGWICK_ITERATIONS;i++) {
          ESP_ERROR_CHECK(gptimer_get_raw_count(GP_timer, &timer_value));
          ESP_ERROR_CHECK(gptimer_set_raw_count(GP_timer, 0));



          MadgwickAHRSupdateIMU((gyro_converted_accumulated_1[1] - gyro_converted_accumulated_2[0]) / (2.0 * PID_LOOPS_RATIO) , 
                                (gyro_converted_accumulated_1[0] + gyro_converted_accumulated_2[1]) / (2.0 * PID_LOOPS_RATIO) ,
                                ((gyro_converted_accumulated_1[2] * (-1.0)) - gyro_converted_accumulated_2[2]) / (2.0 * PID_LOOPS_RATIO) , 
                                (accel_converted_accumulated_1[1] - accel_converted_accumulated_2[0]) / (2.0 * PID_LOOPS_RATIO) , 
                                (accel_converted_accumulated_1[0] + accel_converted_accumulated_2[1]) / (2.0 * PID_LOOPS_RATIO) ,
                                ((accel_converted_accumulated_1[2] * (-1.0)) - accel_converted_accumulated_2[2]) / (2.0 * PID_LOOPS_RATIO), 
                                timer_value);   
        
        }
        
        for (i=0;i<3;i++)
        {
          accel_converted_accumulated_1[i] = 0;
          gyro_converted_accumulated_1[i] = 0;

          accel_converted_accumulated_2[i] = 0;
          gyro_converted_accumulated_2[i] = 0;
        }
        
        Convert_Q_to_degrees();
      }
//***********************************************************************************************************************************************************        
        large_counter++;

        if ((large_counter % 1000) == 0) {
        
        //ESP_ERROR_CHECK(gpio_reset_pin(MPU6000_2_INTERRUPT_PIN));
        //printf ("%ld\n", IMU_interrupt_status); 
        //printf(" %0.1f, %0.1f\n", yaw_setpoint, yaw);
        //printf("%d\n", data_to_send_to_rc.power_voltage_value);
        xTaskNotifyGive(task_handle_read_and_process_data_from_INA219);
#ifdef USING_MAG_DATA
        xTaskNotifyGive(task_handle_read_and_process_data_from_mag);
#endif
        //printf("%0.1f, %0.1f, %0.1f\n", gyro_converted_1[1],(-1)*gyro_converted_2[0], (gyro_converted_1[1] - gyro_converted_2[0]) / 2.0 );  
        //ESP_LOGI(TAG_FLY,"%0.1f", rc_fresh_data.received_yaw);
         
        //ESP_LOGI(TAG_FLY,"ADC is %d ", data_to_send_to_rc.power_voltage_value);
          //ESP_LOGI(TAG_FLY,"fil are %0.2f, %0.2f, %0.2f, %0.2f", engine_filtered[0],engine_filtered[1],engine_filtered[2],engine_filtered[3]);
          //ESP_LOGI(TAG_FLY,"%0.2f, %0.2f, %0.2f, %0.2f", ((((gyro_converted_1[2] * (-1.0)) - gyro_converted_2[2]) / 2.0) * (180.0 / (float)PI)), rc_fresh_data.received_yaw, error_yaw_rate, pid_yaw_rate);
          //ESP_LOGI(TAG_FLY,"%0.2f, %0.2f, %0.2f", rc_fresh_data.received_pitch, rc_fresh_data.received_roll, rc_fresh_data.received_yaw);
          //ESP_LOGI(TAG_FLY,"%0.2f, %0.2f, %0.2f", pitch, roll, yaw);
/*        
          blink = ~blink;
          if (blink == 0) {
            command_for_MCP23017 = MCP23017_SET_OUTPUT_COMMAND | 0;
            command_for_PCA9685 = 0x0103;
          }
          else {command_for_MCP23017 = MCP23017_CLEAR_OUTPUT_COMMAND | 0;
                command_for_PCA9685 = 0x011A;
          }
          //xQueueSend(MCP23017_queue, &command_for_MCP23017, NULL);
          xQueueSend(PCA9685_queue, &command_for_PCA9685, NULL);
*/         
        //uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
          //ESP_LOGW(TAG_FLY,"High watermark %d",  uxHighWaterMark);
        //printf("%0.1f\n", rc_pitch_filtered);
        }



        if (xQueueReceive(remote_control_to_main_queue, &rc_fresh_data, 0)) 
        {
          remote_control_lost_comm_counter = 0;
          
          xTaskNotify(task_handle_blinking_flight_lights,1,eSetValueWithOverwrite);
          //ESP_LOGW(TAG_FLY,"RC comm restored"); 
        }
        else remote_control_lost_comm_counter++;

        if (remote_control_lost_comm_counter > RC_NO_COMM_DELAY_MAIN_CYCLES)  
        {
            if (rc_fresh_data.engines_start_flag) {
              remote_control_lost_comm_counter = RC_NO_COMM_DELAY_MAIN_CYCLES;
              rc_fresh_data.received_throttle = RC_NO_COMM_THROTTLE_HOVER_VALUE;
              rc_fresh_data.received_pitch = 0;
              rc_fresh_data.received_roll = 0;
              rc_fresh_data.received_yaw = 0;
              }
            xTaskNotify(task_handle_blinking_flight_lights,0,eSetValueWithOverwrite);  
        }

        if (xQueueReceive(gps_to_main_queue, &gps_fresh_data, 0)) 
        {
          //ESP_LOGI(TAG_FLY,"Lat is %" PRIu64 " Lon is %" PRIu64, gps_fresh_data.latitude_d, gps_fresh_data.longtitude_d);
        }; 
 
#ifdef USING_LIDAR_UART        
        if (xQueueReceive(lidar_to_main_queue, &lidar_fresh_data, 0)) {
          //ESP_LOGD(TAG_FLY,"height is %d, strength is %d", lidar_fresh_data.height, lidar_fresh_data.strength);
          if (lidar_fresh_data.height < 900) current_altitude = lidar_fresh_data.height * cos(pitch * 0.017453292) * cos(roll * 0.017453292);// PI/180
          //printf ("raw %d, real %02f\n", lidar_fresh_data.height,current_altitude); 
          if (((current_altitude - current_altitude_old) < 5) && ((current_altitude - current_altitude_old) > -5)) current_altitude = current_altitude_old;
          current_altitude_old = current_altitude;
          }; 
#endif

if (xQueueReceive(INA219_to_main_queue, &INA219_fresh_data, 0)) {
        //ESP_LOGE(TAG_FLY, "V: %0.4fV, I: %0.4fA, P: %0.4fW, A: %0.8f",INA219_fresh_data[0], INA219_fresh_data[1], INA219_fresh_data[2], INA219_fresh_data[3]);
      };
    
        if (rc_fresh_data.engines_start_flag)
        {
        if ((rc_fresh_data.altitude_hold_flag) && (lidar_fresh_data.height < 900)) 
          {
            if (altitude_hold_mode_enabled == 0)
            {
              altitude_setpoint = current_altitude;
              altitude_hold_mode_enabled = 1;
              alt_hold_initial_throttle = rc_fresh_data.received_throttle;
            }
            error_altitude = altitude_setpoint - current_altitude;
            integral_alt_error = Ki_alt * error_altitude;
            if (integral_alt_error > 500) integral_alt_error = 500;
            if (integral_alt_error < -500) integral_alt_error = -500;

            //pid_p_alt = Kp_alt * error_altitude;
            //pid_d_alt = Kd_alt * (error_altitude - previous_error_altitude);
            pid_altitude = Kp_alt * error_altitude + Kd_alt * (error_altitude - previous_error_altitude) + integral_alt_error;
            if (pid_altitude > 3000) pid_altitude = 3000;
            if (pid_altitude < -3000) pid_altitude = -3000;
            rc_fresh_data.received_throttle = alt_hold_initial_throttle + pid_altitude;
            previous_error_altitude = error_altitude;
          }
        else 
          {
            altitude_hold_mode_enabled = 0; 
            previous_error_altitude = 0;
          }

        calculate_pids_2();
        }
        else 
        {
          yaw_setpoint = yaw;
          error_pitch_rate_old = error_yaw_rate_old = error_roll_rate_old = 0;
          integral_pitch_error_rate = integral_yaw_error_rate = integral_roll_error_rate = 0;
          integral_pitch_error_angle = integral_roll_error_angle = integral_yaw_error_angle = 0;
          engine[0] = engine[1] = engine[2] = engine[3] = ENGINE_PWM_MIN_DUTY;

          altitude_hold_mode_enabled = 0;
        }
        
        ESC_input_data_filter();
        
        update_engines();
//collecting data to be returned to RC
        data_to_send_to_rc.pitch = pitch;
        data_to_send_to_rc.roll = roll;
        data_to_send_to_rc.yaw = yaw;
        data_to_send_to_rc.power_voltage_value = (uint16_t)(INA219_fresh_data[0]*10.0);
        data_to_send_to_rc.altitude = (uint16_t)current_altitude;;
        xQueueOverwrite(main_to_rc_queue, (void *) &data_to_send_to_rc);         //sending data to rc_send task

//*********************writing logs*****************************//
#ifdef USING_W25N       
        timestamp = get_time() - start_time;
        prepare_logs();
        xQueueSend(W25N01_queue, logs_buffer, NULL);
#endif
        //cycle_end_time = get_time();
        //if ((cycle_end_time - cycle_start_time) > 800) ESP_LOGW(TAG_FLY,"%lld", (cycle_end_time - cycle_start_time));
      gpio_set_level(LED_RED, 1);

      IMU_interrupt_status = 0;
        
        ESP_ERROR_CHECK(gptimer_set_raw_count(general_suspension_timer, 0));    //resetting general_suspension timer at this point
      }
    } 
  }
}

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
  esp_log_level_set(TAG_PCA9685,ESP_LOG_WARN);  //WARN ERROR
  esp_log_level_set(TAG_MCP23017,ESP_LOG_WARN);  //WARN ERROR
  esp_log_level_set(TAG_LIDAR,ESP_LOG_WARN);  //WARN ERROR
  esp_log_level_set(TAG_INA219,ESP_LOG_WARN);  //WARN ERROR
  esp_log_level_set(TAG_IST8310,ESP_LOG_INFO);  //WARN ERROR

  printf("\n");
  ESP_LOGI(TAG_INIT,"System starts\n");  

  //NVS_read_write_test();
  ESP_LOGI(TAG_INIT,"Configuring input-output pins.....");
  configure_IOs();
  ESP_LOGI(TAG_INIT,"IO pins configured\n");
 
  ESP_LOGI(TAG_INIT,"Configuring internal i2c.....");
  i2c_init_internal(I2C_INT_SDA, I2C_INT_SCL, I2C_INT_PORT);
  ESP_LOGI(TAG_INIT,"internal i2c is configured\n");

  ESP_LOGI(TAG_INIT,"Configuring external i2c.....");
  i2c_init_external(I2C_EXT_SDA, I2C_EXT_SCL, I2C_EXT_PORT);
  ESP_LOGI(TAG_INIT,"external i2c is configured\n");

  ESP_LOGI(TAG_INIT,"Checking communication with MCP23017.....");
  if (MCP23017_communication_check() != ESP_OK) {
    error_code = 10;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_INIT,"Configuring MCP23017.....");
  if (MCP23017_init() != ESP_OK) {
    error_code = 8;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
 
  ESP_LOGI(TAG_INIT,"Creating queue for MCP23017.....");
  MCP23017_queue = xQueueCreate(10, sizeof(uint8_t));
  if (MCP23017_queue == NULL) {
    ESP_LOGE(TAG_INIT,"queue for MCP23017 could not be created\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
  else ESP_LOGI(TAG_INIT,"Queue for MCP23017 created\n");

//place it here to avoid engines beeping
  ESP_LOGI(TAG_INIT,"configuring PWM module for engines control.....");
  configuring_timer_for_PWM();
  configuring_channel_for_PWM(0,ENGINE_PWM_OUTPUT_0_PIN);
  configuring_channel_for_PWM(1,ENGINE_PWM_OUTPUT_1_PIN);
  configuring_channel_for_PWM(2,ENGINE_PWM_OUTPUT_2_PIN);
  configuring_channel_for_PWM(3,ENGINE_PWM_OUTPUT_3_PIN);
  ESP_LOGI(TAG_INIT,"PWM for engines control is configured\n");

  if (!(MCP23017_get_inputs_state() & 0b00000100))  //DI2 - calibrating ESC
    { 
      ESP_LOGI(TAG_INIT,"Calibrating ESCs.....");
      //vTaskDelay(2000/portTICK_PERIOD_MS);
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 0, ENGINE_PWM_MIN_DUTY*2));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 0));
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 1, ENGINE_PWM_MIN_DUTY*2));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 1));
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 2, ENGINE_PWM_MIN_DUTY*2));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 2));
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 3, ENGINE_PWM_MIN_DUTY*2));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 3));
      vTaskDelay(3000/portTICK_PERIOD_MS);
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 0, ENGINE_PWM_MIN_DUTY));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 0));
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 1, ENGINE_PWM_MIN_DUTY));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 1));
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 2, ENGINE_PWM_MIN_DUTY));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 2));
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 3, ENGINE_PWM_MIN_DUTY));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 3));
      vTaskDelay(3000/portTICK_PERIOD_MS);
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 0, ENGINE_PWM_MIN_DUTY+1));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 0));
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 1, ENGINE_PWM_MIN_DUTY+1));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 1));
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 2, ENGINE_PWM_MIN_DUTY+1));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 2));
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 3, ENGINE_PWM_MIN_DUTY+1));
      ESP_ERROR_CHECK(ledc_update_duty(ENGINE_PWM_MODE, 3));
      
      ESP_LOGI(TAG_INIT,"Calibration ESCs is finished, please remove jumper and reset the vehicle.");
      
      while (1) {vTaskDelay(1000/portTICK_PERIOD_MS);}
    }

  if (!(MCP23017_get_inputs_state() & 0b00001000))  //DI3 - checking engines
    { 
      ESP_LOGI(TAG_INIT,"Starting engines test.....");
      printf ("%d", MCP23017_get_inputs_state() );
      vTaskDelay(2000/portTICK_PERIOD_MS);
      ESP_ERROR_CHECK(ledc_set_duty(ENGINE_PWM_MODE, 0, ENGINE_PWM_MIN_DUTY * 1.5));
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
      
      ESP_LOGI(TAG_INIT,"Engines test is finished. Please remove jumper and reset the vehicle.");

      while (1) 
      {
        gpio_set_level(LED_RED, 1);
        vTaskDelay(500/portTICK_PERIOD_MS); 
        gpio_set_level(LED_RED, 0);
        vTaskDelay(500/portTICK_PERIOD_MS);
      }
    }

    ESP_LOGI(TAG_INIT,"Checking communication with PCA9685.....");
    if (PCA9685_communication_check() != ESP_OK) {
      error_code = 10;
      xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
      while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
    }

    ESP_LOGI(TAG_INIT,"Configuring PCA9685.....");
    if (PCA9685_init() != ESP_OK) {
    error_code = 11;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  
  ESP_LOGI(TAG_INIT,"Checking communicaion with INA219.....");
  if (INA219_communication_check() != ESP_OK) {
    error_code = 10;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_INIT,"Configuring INA219.....");
  if (INA219_configuration() != ESP_OK) {
    error_code = 8;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
#ifdef USING_MAG_DATA
  ESP_LOGI(TAG_INIT,"Checking communicaion with FL3195.....");
  if (FL3195_communication_check() != ESP_OK) {
    error_code = 10;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_INIT,"Configuring FL3195.....");
  if (FL3195_configuration() != ESP_OK) {
    error_code = 8;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  FL3195_set_pattern(3, 255,0,0);

  ESP_LOGI(TAG_INIT,"Checking communicaion with IST8310.....");
  if (IST8310_communication_check() != ESP_OK) {
    error_code = 10;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_INIT,"Configuring IST8310.....");
    if (IST8310_configuration() != ESP_OK) {
    error_code = 11;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }


  ESP_LOGI(TAG_INIT,"Performing self-test of IST8310.....");
  if (IST8310_selftest() != ESP_OK) {
    error_code = 4;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  } 

#endif

  ESP_LOGI(TAG_INIT,"Configuring both SPIs.....");
  SPI_init();
  ESP_LOGI(TAG_INIT,"Both SPIs initialized\n");

  ESP_LOGI(TAG_INIT,"Checking communication with MPU#1.....");
  if (MPU6000_communication_check(MPU6000_1) != ESP_OK) {
    error_code = 2;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);}  }
  
  ESP_LOGI(TAG_INIT,"Checking communication with MPU#2.....");
  if (MPU6000_communication_check(MPU6000_2) != ESP_OK) {
    error_code = 3;
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
  ESP_LOGI(TAG_INIT,"Configuring MPU#1.....");
  if (MPU6000_init(MPU6000_1) != ESP_OK) {
    error_code = 6;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_INIT,"Configuring MPU#2.....");
  if (MPU6000_init(MPU6000_2) != ESP_OK) {
    error_code = 7;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
  
  ESP_LOGI(TAG_INIT,"Reinitializing SPIs to increase IMU SPI freq up to 20MHz.....");
  SPI_change_MPUs_speed();
  ESP_LOGI(TAG_INIT,"Both SPIs reinitialized\n");

 
  ESP_LOGI(TAG_INIT,"Resetting W25N.....");
  W25N_reset();
  ESP_LOGI(TAG_INIT,"W25N reset done");

  ESP_LOGI(TAG_INIT,"Reading JEDEC ID.....");
  if (W25N_read_JEDEC_ID() != ESP_OK) {
    error_code = 9;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

#ifdef USING_W25N
  ESP_LOGI(TAG_INIT,"Writing W25N status registers.....");
  W25N_write_status_register(W25N_PROT_REG_SR1, 0x00);         //disable protection
  W25N_write_status_register(W25N_CONFIG_REG_SR2, 0b00011000); //as default

  if (!(MCP23017_get_inputs_state() & 0b00000010))    //DI1 - reading memory
  {
    ESP_LOGI(TAG_INIT,"Starting task reading_logs_from_external_flash in 10 secs.....");
    vTaskDelay(10000/portTICK_PERIOD_MS);
    if (xTaskCreate(reading_logs_from_external_flash,"reading_logs_from_external_flash",4096,NULL,0,NULL) == pdPASS)//task with 0 priority, same as idle
    ESP_LOGI(TAG_INIT,"reading_logs_from_external_flash task is created\n");
    while (1) {vTaskDelay(500/portTICK_PERIOD_MS);}  
  }
  else {
    //ESP_LOGI(TAG_INIT,"Performing W25N read-write test.....");
    //winbond_read_write_test();
    ESP_LOGI(TAG_INIT,"Clearing W25N.....");
    W25N_erase_all();
  }
  
  ESP_LOGI(TAG_INIT,"Creating queue for logging to external flash.....");
  W25N01_queue = xQueueCreate(5, LOGS_BYTES_PER_STRING);
  if (W25N01_queue == NULL) {
    ESP_LOGE(TAG_INIT,"Queue for logging to external flash could not be created\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"Queue for logging created\n");
#endif  

  ESP_LOGI(TAG_INIT,"Creating queue for PCA9685.....");
  PCA9685_queue = xQueueCreate(10, sizeof(uint16_t));
  if (PCA9685_queue == NULL) {
    ESP_LOGE(TAG_INIT,"Queue for PCA9685 could not be created\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"Queue for PCA9685 created\n");

#ifdef USING_MAG_DATA
  
  ESP_LOGI(TAG_INIT,"Creating queue to transfer data from magnetometer to main.....");
  magnetometer_queue = xQueueCreate(9, 6);       //9 values 6 bytes each (3 x 2)
  if (magnetometer_queue == NULL) {
    ESP_LOGE(TAG_INIT,"queue to transfer data from magnetometer to main could not be created\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"queue to transfer data from magnetometer to main created\n");
  
  ESP_LOGI(TAG_INIT,"Creating semaphore to start mag data reading.....");
  semaphore_to_read_mag = xSemaphoreCreateBinary();
  if (semaphore_to_read_mag == NULL) {
    ESP_LOGE(TAG_INIT,"Semaphore_to_read_mag could not be created\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
  else ESP_LOGI(TAG_INIT,"Semaphore to read mag data created\n");

#endif 

  ESP_LOGI(TAG_INIT,"Creating queue to send data from remote control to main task");
  remote_control_to_main_queue = xQueueCreate(10, sizeof(struct data_from_rc_to_main_struct));
  if (remote_control_to_main_queue == NULL) {
    ESP_LOGE(TAG_INIT,"queue to send data from remote control to main task could not be created\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"Queue to send data from remote control to main created\n"); 

  ESP_LOGI(TAG_INIT,"Creating queue to send data from GPS to main task");
  gps_to_main_queue = xQueueCreate(10, sizeof(struct data_from_gps_to_main_struct));
  if (gps_to_main_queue == NULL) {
    ESP_LOGE(TAG_INIT,"queue to send data from GPS to main task could not be created\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"Queue to send data from GPS to main created\n");
     
  ESP_LOGI(TAG_INIT,"Creating queue to send data from main to RC_Send task");
  main_to_rc_queue = xQueueCreate(1, sizeof(struct data_from_main_to_rc_struct));     //size 1 because use xQueueOverwrite
  if (main_to_rc_queue == NULL) {
    ESP_LOGE(TAG_INIT,"queue to send data from main to RC_Send task could not be created\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"Queue to send data from main to RC_send task created\n"); 
#ifdef USING_LIDAR_UART
  ESP_LOGI(TAG_INIT,"Creating queue to send data from lidar to main task");
  lidar_to_main_queue = xQueueCreate(10, sizeof(struct data_from_lidar_to_main_struct));
  if ( lidar_to_main_queue == NULL) {
    ESP_LOGE(TAG_INIT,"queue to send data from lidar to main task could not be created\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"Queue to send data from lidar to main created\n"); 
#endif
  
  ESP_LOGI(TAG_INIT,"Creating queue for INA219.....");
  INA219_to_main_queue = xQueueCreate(10, 4 * sizeof(float));
  if (INA219_to_main_queue == NULL) {
    ESP_LOGE(TAG_INIT,"Queue for INA219 could not be created\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"Queue for INA219 created\n");

  ESP_LOGI(TAG_INIT,"Creating queue for blinking.....");
  any_to_blinking_queue = xQueueCreate(2, sizeof(uint8_t));
  if (any_to_blinking_queue == NULL) {
    ESP_LOGE(TAG_INIT,"any_to_blinking_queue could not be created\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"any_to_blinking_queue is created\n"); 

  ESP_LOGI(TAG_INIT,"creating and starting GP timer.....");
  Create_and_start_GP_Timer ();
  ESP_LOGI(TAG_INIT,"GP timer created and started\n");

  ESP_LOGI(TAG_INIT,"Creating queue to monitor IMU suspention.....");
  IMU_monitoring_timer_to_main_queue = xQueueCreate(2, sizeof(uint8_t));
  if (IMU_monitoring_timer_to_main_queue == NULL) {
    ESP_LOGE(TAG_INIT,"IMU_monitoring_timer_to_main_queue could not be created\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }
    else ESP_LOGI(TAG_INIT,"IMU_monitoring_timer_to_main_queue is created\n"); 
/*
  ESP_LOGI(TAG_INIT,"creating and starting test timer.....");
  Create_and_start_test_timer();
  ESP_LOGI(TAG_INIT,"test timer created and started\n");
*/ 
  ESP_LOGI(TAG_INIT,"configuring ADC.....");
  configure_ADC();
  ESP_LOGI(TAG_INIT,"ADC is configured");

//********************************************************************************************************************************************************* */

  
  ESP_LOGI(TAG_INIT,"Starting tasks creation\n");

  //core 0 deals with WiFI tasks
    ESP_LOGI(TAG_INIT,"Creating MCP23017_monitoring_and_control task..... ");
  if (xTaskCreateStaticPinnedToCore(MCP23017_monitoring_and_control,"MCP23017_monitoring_and_control",MCP23017_MONITORING_AND_CONTROL_STACK_SIZE,NULL,1,MCP23017_monitoring_and_control_stack,&MCP23017_monitoring_and_control_TCB_buffer,0) != NULL)
    ESP_LOGI(TAG_INIT,"MCP23017_monitoring_and_control is created\n");
  else {
    ESP_LOGE(TAG_INIT,"MCP23017_monitoring_and_control task could not be created\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  ESP_LOGI(TAG_INIT,"Creating PCA9685_control task..... ");
  if (xTaskCreateStaticPinnedToCore(PCA9685_control,"PCA9685_control",PCA9685_CONTROL_STACK_SIZE,NULL,1,PCA9685_control_stack,&PCA9685_control_TCB_buffer,0) != NULL) 
    ESP_LOGI(TAG_INIT,"PCA9685_control is created\n");
  else {
    ESP_LOGE(TAG_INIT,"PCA9685_control task could not be created\n");
    error_code = 1;
    xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
    while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
  }

  #ifdef USING_GPS                
        ESP_LOGI(TAG_INIT,"Creating read_and_process_data_from_GPS task.....");
        if (xTaskCreateStaticPinnedToCore(read_and_process_data_from_gps, "read_and_process_data_from_gps", READ_AND_PROCESS_DATA_FROM_GPS_STACK_SIZE, NULL, 3, read_and_process_data_from_gps_stack, &read_and_process_data_from_gps_TCB_buffer, 0) != NULL)
          ESP_LOGI(TAG_INIT,"read_and_process_data_from_GPS task is created at core 0");
#endif        
        vTaskDelay(50/portTICK_PERIOD_MS);

#ifdef USING_MAG_DATA               
        ESP_LOGI(TAG_INIT,"Creating read_and_process_data_from_magnetometer task.....");
        task_handle_read_and_process_data_from_mag = xTaskCreateStaticPinnedToCore(read_and_process_data_from_mag, "read_and_process_data_from_mag", READ_AND_PROCESS_DATA_FROM_MAG_STACK_SIZE, NULL, 3, read_and_process_data_from_mag_stack, &read_and_process_data_from_mag_TCB_buffer, 0);
        if (task_handle_read_and_process_data_from_mag != NULL)
          ESP_LOGI(TAG_INIT,"read_and_process_data_from_mag task is created at core 0");
#endif        
        
        vTaskDelay(50/portTICK_PERIOD_MS);
         
        ESP_LOGI(TAG_INIT,"Creating read_and_process_data_from_RC task.....");
        if (xTaskCreateStaticPinnedToCore(read_and_process_data_from_RC,"read_and_process_data_from_RC",READ_AND_PROCESS_DATA_FROM_RC_STACK_SIZE,NULL,5,read_and_process_data_from_RC_stack,&read_and_process_data_from_RC_TCB_buffer,0) != NULL)
          ESP_LOGI(TAG_INIT,"read_and_process_data_from_RC task is created at core 0");

        vTaskDelay(50/portTICK_PERIOD_MS);

        ESP_LOGI(TAG_INIT,"Creating send_data_to_RC task.....");
        task_handle_send_data_to_RC = xTaskCreateStaticPinnedToCore(send_data_to_RC,"send_data_to_RC",SEND_DATA_TO_RC_STACK_SIZE ,NULL,4,send_data_to_RC_stack,&send_data_to_RC_TCB_buffer,0);
        if (task_handle_send_data_to_RC != NULL)
          ESP_LOGI(TAG_INIT,"send_data_to_RC task is created at core 0");
#ifdef USING_PERFORMANCE_MESUREMENT
        ESP_LOGI(TAG_INIT,"Creating performance measuring task.....");
        task_handle_performace_measurement = xTaskCreatePinnedToCore(performace_monitor,"performace_monitor",8192 ,NULL,4,&task_handle_performace_measurement,0);
        if (task_handle_performace_measurement != NULL)
          ESP_LOGI(TAG_INIT,"performance measurement task is created at core 0");
#endif        
        vTaskDelay(50/portTICK_PERIOD_MS);

#ifdef USING_LIDAR_UART
        ESP_LOGI(TAG_INIT,"Creating read_and_process_data_from_lidar task.....");
        if (xTaskCreateStaticPinnedToCore(read_and_process_data_from_lidar,"read_and_process_data_from_lidar",read_and_process_data_from_lidar_STACK_SIZE ,NULL,3,read_and_process_data_from_lidar_stack,&read_and_process_data_from_lidar_TCB_buffer,0) != NULL)
          ESP_LOGI(TAG_INIT,"read_and_process_data_from_lidar task is created at core 0");

        vTaskDelay(50/portTICK_PERIOD_MS);
#endif
        
     
        ESP_LOGI(TAG_INIT,"Creating read_and_process_data_from_INA219 task.....");
        task_handle_read_and_process_data_from_INA219 = xTaskCreateStaticPinnedToCore(read_and_process_data_from_INA219,"read_and_process_data_from_INA219",READ_AND_PROCESS_DATA_FROM_INA219_STACK_SIZE,NULL,2,read_and_process_data_from_INA219_stack, &read_and_process_data_from_INA219_TCB_buffer,0);
        if ( task_handle_read_and_process_data_from_INA219 != NULL)
          ESP_LOGI(TAG_INIT,"read_and_process_data_from_INA219 task created");

        ESP_LOGI(TAG_INIT,"Creating blinking_flight_lights task.....");
        task_handle_blinking_flight_lights = xTaskCreateStaticPinnedToCore(blinking_flight_lights,"blinking_flight_lights",BLINKING_FLIGHT_LIGHTS_STACK_SIZE,NULL,0,blinking_flight_lights_stack, &blinking_flight_lights_TCB_buffer,0);
        if ( task_handle_blinking_flight_lights != NULL)
          ESP_LOGI(TAG_INIT,"blinking flight lights task created");
        
        //vTaskSuspend(task_handle_blinking_flight_lights); 
        //ESP_LOGI(TAG_INIT,"blinking flight lights task is suspended until establishing communication with RC");      

        gpio_set_level(GREEN_FLIGHT_LIGHTS, 1);
        gpio_set_level(RED_FLIGHT_LIGHTS, 1);  

        vTaskDelay(50/portTICK_PERIOD_MS); 

#ifdef USING_W25N
        xTaskCreatePinnedToCore(writing_logs_to_flash,"writing_logs_to_flash",4096,NULL,3,NULL,0);
        ESP_LOGI(TAG_INIT,"Logging task created\n");
        start_time = get_time();
#endif

        ESP_LOGI(TAG_INIT,"Creating Main_flying_cycle task..... ");
        task_handle_main_flying_cycle = xTaskCreateStaticPinnedToCore(main_flying_cycle, "Main_flying_cycle", MAIN_FLYING_CYCLE_STACK_SIZE, NULL, 24, main_flying_cycle_stack,&main_flying_cycle_TCB_buffer,1);
        if (task_handle_main_flying_cycle != NULL)
          ESP_LOGI(TAG_INIT,"Main_flying_cycle is created\n");
        else {
          ESP_LOGE(TAG_INIT,"Main_flying_cycle could not be created\n");
          error_code = 1;
          xTaskCreate(error_code_LED_blinking,"error_code_LED_blinking",2048,(void *)&error_code,0,NULL);
          while(1) {vTaskDelay(1000/portTICK_PERIOD_MS);} 
        }

        ESP_LOGI(TAG_INIT,"creating and starting general suspension timer.....");
        create_and_start_general_suspension_timer();
        ESP_LOGI(TAG_INIT,"general suspension timer is created and started\n");
        

  vTaskDelete(NULL);
  
}




void app_main(void) {

  xTaskCreatePinnedToCore(init, "init", 8912, NULL, 5, &task_handle_init, 1);
 
}



