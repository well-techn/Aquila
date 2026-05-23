//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "math.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "mavlink_wt.h"
#include "mavlink_crc.h"

extern  char *TAG_OSD;
extern QueueHandle_t mav_queue_for_events;
extern QueueHandle_t main_to_mavlink_queue;
extern QueueHandle_t MCP23017_queue;


void MSP_uart_config(void)
{
    int intr_alloc_flags = 0;
    uart_config_t uart_config = {
        .baud_rate = OSD_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(OSD_UART, OSD_RX_UART_BUF_SIZE, OSD_TX_UART_BUF_SIZE , OSD_UART_PATTERN_DETECTION_QUEUE_SIZE, &mav_queue_for_events, intr_alloc_flags)); 
    ESP_ERROR_CHECK(uart_param_config(OSD_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(OSD_UART, OSD_UART_TX_PIN, OSD_UART_RX_PIN, OSD_UART_RTS_PIN, OSD_UART_CTS_PIN));
    uart_flush(OSD_UART);                                                                           //сбрасываем буфер
}

//Задача отправки телеметрии по mavlink. 
//Забирает из очереди, в которую шлет main_flying_cycle, данные и отправляем их в mavlink UART.
void send_telemetry_via_MSP(void * pvParameters)
{
  ESP_LOGI(TAG_OSD, "Настраиваем UART для MSP......");
  MSP_uart_config();
  ESP_LOGI(TAG_OSD, "UART для MSP настроен");

  uart_event_t osd_uart_event;

  uint8_t counter = 0;

  
  char incoming_message_buffer_osd[20];
  float voltage = 12.4;
  uint8_t msp_ident_reply[] = {0x24, 0x4D, 0x3E, 0x07, 0x02, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26};//ok
  uint8_t msp_analog_reply[] = {
    0x24, 0x4D, 0x3E, // $M>
    0x07,             // Size: 7 байт
    0x6E,             // ID: 110
    0x7E,             // Data[0]: VBat (12.6V)
    0x00, 0x00,       // Data[1-2]: Power (мАч)
    0xFF, 0x03,       // Data[3-4]: RSSI = 1023 (0x03FF в Little Endian)
    0x00, 0x00,       // Data[5-6]: Current (ток)
    0xEB              // ВЕРНЫЙ CRC (07^6E^7E^FF^03 = 0xEB)
};
  uint8_t msp_misc_reply[] = {0x24, 0x4D, 0x3E, 0x16, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15};//ok

  uint8_t msp_status_reply[] = {
    0x24, 0x4D, 0x3E, // Заголовок $M> (символ '>' означает ответ от контроллера)
    0x0B,             // Размер полезной нагрузки (11 байт)
    0x65,             // ID команды (101 - MSP_STATUS)
    0x00, 0x00,       // cycleTime
    0x00, 0x00,       // i2c_errors_count
    0x01, 0x00,       // sensor (акселерометр включен)
    0x00, 0x00, 0x00, 0x00, // flight_mode (дизармлен, стабилизация)
    0x00,             // profile
    0x6F              // CRC (рассчитан для 0x0B^0x65^0x01 = 0x6D)
};
uint8_t msp_status_armed_reply[] = {
    0x24, 0x4D, 0x3E, // $M>
    0x0B,             // Size
    0x65,             // ID (101)
    0x00, 0x00,       // cycleTime
    0x00, 0x00,       // i2c_errors
    0x01, 0x00,       // sensor (ACC)
    0x01, 0x00, 0x00, 0x00, // flight_mode: БИТ 0 установлен (ARMED)
    0x00,             // profile
    0x6E              // Новый CRC (0x0B ^ 0x65 ^ 0x01 ^ 0x01 = 0x6C)
};
uint8_t msp_osd_hb[] = {0x24, 0x4D, 0x3C, 0x01, 0xB8, 0x02, 0xBB};//ok

  // сonst uint8_t msp_osd_clear[] = {0x24, 0x4D, 0x3C, 0x01, 0xB8, 0x00, 0xB9};//ok 
  // uint8_t msp_osd_write_A[] = {0x24, 0x4D, 0x3C, 0x06, 0xB8, 0x01, 0x08, 0x0F, 0x00, 0x41, 0x00, 0xF9};//ok
  // uint8_t msp_osd_draw[] = {0x24, 0x4D, 0x3C, 0x01, 0xB8, 0x03, 0xBA};//ok
  // uint8_t subcommand_4[] = {0x24, 0x4D, 0x3C, 0x01, 0xB8, 0x04, 0xBD};

  uint8_t msp_set_canvas_hd[] = {0x24, 0x4D, 0x3C, 0x02, 0xBC, 0x32, 0x12, 0x9E};
  uint8_t msp_request_canvas_hd[] = {0x24, 0x4D, 0x3C, 0x00, 0xBD, 0xBD};

  uint8_t msp_osd_hb_v2[] = {0x24, 0x4D, 0x3C, 0x01, 0xB6, 0x00, 0xB7};
  uint8_t msp_osd_clear_v2[] = {0x24, 0x4D, 0x3C, 0x01, 0xB6, 0x02, 0xB5};
  uint8_t msp_osd_write_A_v2[] = {0x24, 0x4D, 0x3C, 0x06, 0xB6, 0x03, 0x08, 0x0F, 0x00, 0x41, 0x00, 0xF5};
  uint8_t msp_osd_draw_v2[] = {0x24, 0x4D, 0x3C, 0x01, 0xB6, 0x04, 0xB3};
  uint8_t msp_dp_release[] = {
    0x24, 0x4D, 0x3C, // Заголовок $M<
    0x01,             // Размер данных
    0xB6,             // ID команды (182)
    0x01,             // Субкоманда (RELEASE)
    0xB6              // CRC (0x01 ^ 0xB6 ^ 0x01 = 0xB6)
};
 
  //uart_write_bytes(OSD_UART,msp_set_canvas_hd, sizeof(msp_set_canvas_hd));
  uart_write_bytes(OSD_UART,msp_dp_release, sizeof(msp_dp_release));
  vTaskDelay(100);

while(1)
{
  if(xQueueReceive(mav_queue_for_events, (void * )&osd_uart_event, (TickType_t)portMAX_DELAY)) 
    {     
      switch(osd_uart_event.type)                   //проверяем тип события, полученного от UART 
      {
        case UART_DATA:
          int16_t pos = 0;
          uint8_t incoming_message_buffer_osd[20];
          ESP_ERROR_CHECK(uart_get_buffered_data_len(OSD_UART, (size_t*)&pos));  //запрашиваем сколько байт получено
          int read_len = uart_read_bytes(OSD_UART, incoming_message_buffer_osd, pos, 0);  //считываем в локальный буфер 
            //ESP_LOGD(TAG_OSD, "Всего получено %d байт", read_len);         
          //ESP_LOG_BUFFER_HEX(TAG_OSD, incoming_message_buffer_osd, read_len);

          
    if (incoming_message_buffer_osd[0] == 0x24 && incoming_message_buffer_osd[1] == 0x4D) { // Проверка заголовка $M
    
    uint8_t msp_id = incoming_message_buffer_osd[4]; // ID команды всегда в 5-м байте

    switch (msp_id) {
        case 0x65: // Запрос MSP_STATUS (101)
            uart_write_bytes(OSD_UART, (const char*)msp_status_armed_reply, sizeof(msp_status_armed_reply));
            //printf("st\n");
            break;

        case 0x02: // Запрос MSP_IDENT (2)
            uart_write_bytes(OSD_UART, (const char*)msp_ident_reply, sizeof(msp_ident_reply));
            //printf("iden\n");
            break;

        case 0x6E: // Запрос MSP_ANALOG (110)
            uart_write_bytes(OSD_UART, (const char*)msp_analog_reply, sizeof(msp_analog_reply));
            //printf("anal\n");
            break;

        case 0x03: // Запрос MSP_MISC (3)
            uart_write_bytes(OSD_UART, (const char*)msp_misc_reply, sizeof(msp_misc_reply));
            //printf("misc\n");
            break;

        default:
            // Неизвестный запрос - можно игнорировать
            printf("U %d\n",msp_id);
            ESP_LOG_BUFFER_HEX(TAG_OSD, incoming_message_buffer_osd, read_len);
            gpio_set_level(LED_BLUE,0);
            break;
    }

    //  uart_write_bytes(OSD_UART, (const char*)msp_osd_hb, sizeof(msp_osd_hb));
    counter++;
    if ((counter % 10) == 0)
    {
    uart_write_bytes(OSD_UART, (const char*)msp_osd_hb_v2, sizeof(msp_osd_hb_v2));
    uart_write_bytes(OSD_UART, (const char*)msp_osd_write_A_v2, sizeof(msp_osd_write_A_v2));
    uart_write_bytes(OSD_UART, (const char*)msp_osd_draw_v2, sizeof(msp_osd_draw_v2));
    //uart_write_bytes(OSD_UART, (const char*)msp_osd_clear, sizeof(msp_osd_clear));

    }

    // if ((counter % 53) == 0)
    // {
    // uart_write_bytes(OSD_UART, (const char*)msp_osd_write_A_v2, sizeof(msp_osd_write_A_v2));
    // uart_write_bytes(OSD_UART, (const char*)msp_osd_draw_v2, sizeof(msp_osd_draw_v2));
    // }

// if ((counter % 53) == 0)
// {
//   uart_write_bytes(OSD_UART,msp_request_canvas_hd, sizeof(msp_request_canvas_hd));
// }

    




          uart_flush_input((uart_port_t)OSD_UART);
          break;

        case UART_FIFO_OVF:
          ESP_LOGW(TAG_OSD, "hw fifo overflow");
          uart_flush_input(OSD_UART);
          xQueueReset(LED_BLUE);
          break;

        case UART_BUFFER_FULL:
          ESP_LOGW(TAG_OSD, "ring buffer full");
          uart_flush_input(OSD_UART);
          xQueueReset(LED_BLUE);
          break;

        case UART_PATTERN_DET:
          ESP_LOGW(TAG_OSD, "pattern det");
          uart_flush_input(OSD_UART);
          xQueueReset(LED_BLUE);
          break;
        
        default: break;

      
    }
   
      //uart_write_bytes(OSD_UART, (const char*)req, sizeof(req));


        //gpio_set_level(LED_BLUE, 0);
        
        // uart_write_bytes(OSD_UART,msp_status_armed_reply, sizeof(msp_status_armed_reply));
        // uart_write_bytes(OSD_UART,msp_ident_reply, sizeof(msp_ident_reply));
        // uart_write_bytes(OSD_UART,msp_misc_reply, sizeof(msp_misc_reply));
        // uart_write_bytes(OSD_UART,msp_hb, sizeof(msp_hb));

        // uart_write_bytes(OSD_UART,msp_clear, sizeof(msp_clear));


        // snprintf(incoming_message_buffer_osd, sizeof(incoming_message_buffer_osd), "VOLTS: %.1fV", voltage);
        // msp_dp_write_string(OSD_UART, 5, 5, incoming_message_buffer_osd); // Координаты X=2, Y=2

        // // 3. Команда на отрисовку (ОБЯЗАТЕЛЬНО для отображения изменений)
        // uint8_t draw_frame[] = {'$', 'M', '<', 1, MSP_DISPLAYPORT, MSP_DP_DRAW_SCREEN, (1 ^ MSP_DISPLAYPORT ^ MSP_DP_DRAW_SCREEN)};
        // uart_write_bytes(OSD_UART, (const char*)draw_frame, sizeof(draw_frame));
        // gpio_set_level(LED_BLUE, 1); 

        // vTaskDelay(pdMS_TO_TICKS(100)); // Обновление 10 Гц
  }
}
}}
