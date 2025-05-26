//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "math.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "mavlink_wt.h"
#include "mavlink_crc.h"

extern  char *TAG_MAV;
extern QueueHandle_t mav_queue_for_events;
extern QueueHandle_t main_to_mavlink_queue;

#ifdef USING_MAVLINK_TELEMETRY

void mavlink_uart_config(void)
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

//Задача отправки телеметрии по mavlink. 
//Забирает из очереди, в которую шлет main_flying_cycle, данные и отправляем их в mavlink UART.
void send_telemetry_via_mavlink(void * pvParameters)
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
        
        attitude.roll = p_to_mavlink_data -> angles[1] * M_PI / 180;
        attitude.pitch = p_to_mavlink_data -> angles[0] * M_PI / 180;
        attitude.yaw = p_to_mavlink_data -> angles[2] * M_PI / 180;
        //printf("%f\n", attitude.yaw);

        gps_raw.lat = p_to_mavlink_data -> latitude;
        gps_raw.lon = p_to_mavlink_data -> longtitude;
        gps_raw.fix_type = p_to_mavlink_data -> gps_status;

        rc_channels.rssi = (p_to_mavlink_data -> rssi_level) * 2.55 + 255;     //преобразуем сигнал [-100..0] в [0..255]

        vfr_hud.heading = p_to_mavlink_data -> angles[2]; //heading
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