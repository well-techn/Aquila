
// системные библиотеки
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <lwip/sockets.h>

// собственные библиотеки
#include "wt_alldef.h"
#include "winbondW25N.h"

extern const char *TAG_W25N;

// Задача считывания данных из внешней flash. Активируется только если обнаруживается установленная перемычка "считать логи."
// Получает данные с памяти и выдает их по telnet по установленному wifi соединению
void reading_logs_from_external_flash(void *pvParameters)
{
  uint16_t page_address = 0;
  uint16_t column_address = 0;
  uint8_t i = 0;
  // сюда считывается содержимое 1го пакета из flash-памяти
  uint8_t receiving_logs_buffer[sizeof(struct logging_data_set)];
  uint8_t empty_timestamp_flag = 0;
  struct logging_data_set *p_to_set_to_log;

  int16_t *client_fd = pvParameters;
  // сюда содержимое пакета печатается через sprintf (длина с запасом)
  char message_to_print[200];
  uint16_t next = 0;
  char header[] = "time_us|ax|ay|az|gx|gy|gz|q0|q1|q2|q3|pitch|roll|yaw|lid_h|baro_h|Kalman_h|Kalman_v|alt_setp|optical_X|optical_Y|optical_quality|v_mV|I_cA|thr_c|pitch_c|roll_c|yaw_c|mode_c|e1|e2|e3|e4|flags|rssi|EOL\r\n";
  char end_message[] = "Считывание логов из внешней flash-памяти завершено, перезапустите систему\r\n";

  while (1)
  {
    // печатаем строку заголовков
    printf("%s", header);
#ifdef TELNET_CONF_MODE
    send(*client_fd, header, strlen(header), 0);
#endif

    while ((page_address < 65535) && (empty_timestamp_flag == 0)) // 65365 страниц на микросхеме памяти
    {
      W25N_page_data_read(page_address); // копируем содержимое страницы в буфер микросхемы
      column_address = 0;

      while ((column_address < (2048 - sizeof(struct logging_data_set))) && (empty_timestamp_flag == 0)) // пока в буфере помещается целый пакет данных
      {
        W25N_read(column_address, receiving_logs_buffer, sizeof(struct logging_data_set)); // считываем первый пакет из буфера

        p_to_set_to_log = (struct logging_data_set *)receiving_logs_buffer;

        if (p_to_set_to_log->timestamp == 4294967295)
          empty_timestamp_flag = 1; // в такое число считывается таймстэмп если ячейкм не запроганы (все FF)
// в случае если не достигли конца записанных в микруху данных начинаем переводить числа в печатный вид при помощи sprintf
        if (!empty_timestamp_flag)
        {
          next += sprintf(message_to_print, "%lu|", p_to_set_to_log->timestamp);
          for (i = 0; i < 3; i++)
            next += sprintf(message_to_print + next, "%d|", p_to_set_to_log->accel[i]);
          for (i = 0; i < 3; i++)
            next += sprintf(message_to_print + next, "%d|", p_to_set_to_log->gyro[i]);
          for (i = 0; i < 4; i++)
            next += sprintf(message_to_print + next, "%f|", p_to_set_to_log->q[i]);
          for (i = 0; i < 3; i++)
            next += sprintf(message_to_print + next, "%0.2f|", p_to_set_to_log->angles[i]);
          next += sprintf(message_to_print + next, "%d|", p_to_set_to_log->lidar_altitude_cm);
          // next += sprintf(message_to_print + next, "%d|", p_to_set_to_log->lidar_strength);
          next += sprintf(message_to_print + next, "%d|", p_to_set_to_log->baro_altitude_cm);
          next += sprintf(message_to_print + next, "%d|", p_to_set_to_log->kalman_altitude_cm);
          next += sprintf(message_to_print + next, "%d|", p_to_set_to_log->kalman_velocity_cm);
          next += sprintf(message_to_print + next, "%d|", p_to_set_to_log->altitude_setpoint_cm);
          
          next += sprintf(message_to_print + next, "%d|", p_to_set_to_log->px4flow_position_x_cm);
          next += sprintf(message_to_print + next, "%d|", p_to_set_to_log->px4flow_position_y_cm);
          next += sprintf(message_to_print + next, "%d|", p_to_set_to_log->px4flow_quality);

          next += sprintf(message_to_print + next, "%d|", p_to_set_to_log->voltage_mv);
          next += sprintf(message_to_print + next, "%d|", p_to_set_to_log->current_ca);
          next += sprintf(message_to_print + next, "%0.2f|", p_to_set_to_log->throttle_command);
          next += sprintf(message_to_print + next, "%0.2f|", p_to_set_to_log->pitch_command);
          next += sprintf(message_to_print + next, "%0.2f|", p_to_set_to_log->roll_command);
          next += sprintf(message_to_print + next, "%0.2f|", p_to_set_to_log->yaw_command);
          next += sprintf(message_to_print + next, "%d|", p_to_set_to_log->mode_command);
          for (i = 0; i < 4; i++) next += sprintf(message_to_print + next, "%ld|", p_to_set_to_log->engines[i]);
          next += sprintf(message_to_print + next, "%d|", p_to_set_to_log->flags);
          next += sprintf(message_to_print + next, "%d|", p_to_set_to_log->rssi_level);
          next += sprintf(message_to_print + next, "*\r\n"); // конец строки

// Печаем или отправляем сформированную строку в telnet
#ifdef TELNET_CONF_MODE
          send(*client_fd, message_to_print, next, 0);
#endif
          printf("%s", message_to_print);
          next = 0;
          // переход на следующий пакет в буфере микросхемы
          column_address += sizeof(struct logging_data_set);
        }
      }
      page_address++; // переход к считыванию следующей страницы памяти
    }

    ESP_LOGI(TAG_W25N, "Считывание логов из внешней flash-памяти завершено, перезапустите систему");
#ifdef TELNET_CONF_MODE
    send(*client_fd, end_message, strlen(end_message), 0);
#endif

    while (1) // после как закончили считывать моргаем
    {
      gpio_set_level(LED_RED, 1);
      gpio_set_level(LED_GREEN, 1);
      gpio_set_level(LED_BLUE, 1);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      gpio_set_level(LED_RED, 0);
      gpio_set_level(LED_GREEN, 0);
      gpio_set_level(LED_BLUE, 0);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    };
  }
}