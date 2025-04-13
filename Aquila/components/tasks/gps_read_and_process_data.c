//стандартные библиотеки
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/uart.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "FL3195.h"

extern  char *TAG_GPS;
extern QueueHandle_t gps_queue_for_events;
extern QueueHandle_t gps_to_main_queue;
extern SemaphoreHandle_t semaphore_for_i2c_external;

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



//Задача получения и обработки данных от GPS. Подразумеваем что получаем только RMC сообщения.
//Ждет прерывания по обнаружению символа конца строки, при обнаружении разбираем полученную строку, вычленяем координаты и выдаем в очередь в сторону main_flying_cycle.
//Управляем трехцветным светодиодом FL3195 на модуле Holybro M9N для отображения сиатуса GPS. 
void gps_read_and_process_data(void * pvParameters)
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
                  
#ifdef USING_FL3195
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
//остальные события UART, которые особо не интересуют            
            case UART_DATA: break;

            case UART_FIFO_OVF:
              ESP_LOGW(TAG_GPS, "hw fifo overflow");
              uart_flush_input(REMOTE_CONTROL_UART);
              xQueueReset(gps_queue_for_events);
              break;

            case UART_BUFFER_FULL:
              ESP_LOGW(TAG_GPS, "ring buffer full");
              uart_flush_input(REMOTE_CONTROL_UART);
              xQueueReset(gps_queue_for_events);
              break;
        
            case UART_BREAK:
              ESP_LOGW(TAG_GPS, "uart rx break");
              break;
            
            case UART_PARITY_ERR:
              ESP_LOGW(TAG_GPS, "uart parity error");
              break;
            
            case UART_FRAME_ERR:
              ESP_LOGW(TAG_GPS, "uart frame error");
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
