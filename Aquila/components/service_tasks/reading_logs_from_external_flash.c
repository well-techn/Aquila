
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
  uint8_t receiving_logs_buffer[sizeof(struct logging_data_set)]; // сюда считывается содержимое 1го пакета из flash-памяти
  uint8_t empty_timestamp_flag = 0;
  uint8_t emergency_mode_flag = 0;
  struct logging_data_set *p_to_set_to_log;

  int16_t *client_fd = pvParameters;
  char message_to_print[300]; // сюда содержимое пакета печатается через snprintf (длина с запасом)
  uint16_t next = 0;
  char end_message[] = "Считывание логов из внешней flash-памяти завершено, перезапустите систему\r\n";
  char header_new[300];

//формируем строку заголовка исходя из того, что включено в логи
  next += snprintf(header_new, sizeof(header_new), "time_us|");
#ifdef LOGGING_ACCEL_1 
  next += snprintf(header_new + next, sizeof(header_new), "ax_raw_1|");
  next += snprintf(header_new + next, sizeof(header_new), "ay_raw_1|");
  next += snprintf(header_new + next, sizeof(header_new), "az_raw_1|");
#endif

#ifdef LOGGING_ACCEL_2 
  next += snprintf(header_new + next, sizeof(header_new), "ax_raw_2|");
  next += snprintf(header_new + next, sizeof(header_new), "ay_raw_2|");
  next += snprintf(header_new + next, sizeof(header_new), "az_raw_3|");
#endif

#ifdef LOGGING_ACCEL_1_MAX
  next += snprintf(header_new + next, sizeof(header_new), "accel_1_max|");
#endif

#ifdef LOGGING_ACCEL_2_MAX
  next += snprintf(header_new + next, sizeof(header_new), "accel_2_max|");
#endif

#ifdef LOGGING_GYRO_1
  next += snprintf(header_new + next, sizeof(header_new), "gx_raw_1|");
  next += snprintf(header_new + next, sizeof(header_new), "gy_raw_1|");
  next += snprintf(header_new + next, sizeof(header_new), "gz_raw_1|");
#endif

#ifdef LOGGING_GYRO_2 
  next += snprintf(header_new + next, sizeof(header_new), "gx_raw_2|");
  next += snprintf(header_new + next, sizeof(header_new), "gy_raw_2|");
  next += snprintf(header_new + next, sizeof(header_new), "gz_raw_3|");
#endif

#ifdef LOGGING_AVG_ACCEL
  next += snprintf(header_new + next, sizeof(header_new), "ax_avg|");
  next += snprintf(header_new + next, sizeof(header_new), "ay_avg|");
  next += snprintf(header_new + next, sizeof(header_new), "az_avg|");
#endif

#ifdef LOGGING_AVG_GYRO
  next += snprintf(header_new + next, sizeof(header_new), "gx_avg|");
  next += snprintf(header_new + next, sizeof(header_new), "gy_avg|");
  next += snprintf(header_new + next, sizeof(header_new), "gz_avg|");
#endif

#ifdef LOGGING_QUATERNION 
  next += snprintf(header_new + next, sizeof(header_new), "q0|q1|q2|q3|");
#endif

#ifdef LOGGING_ANGLES 
  next += snprintf(header_new + next, sizeof(header_new), "pitch|roll|yaw|");
#endif

#ifdef LOGGING_LIDAR_PID
  next += snprintf(header_new + next, sizeof(header_new), "lid_er|lid_P|lid_I|lid_D|");
#endif

#ifdef LOGGING_LIDAR_ALTITUDE_CM
  next += snprintf(header_new + next, sizeof(header_new), "lid_h|");
#endif

#ifdef LOGGING_BARO_PID
  next += snprintf(header_new + next, sizeof(header_new), "baro_er|baro_P|baro_I|baro_D|");
#endif

#ifdef LOGGING_BARO_ALTITUDE_CM
  next += snprintf(header_new + next, sizeof(header_new), "baro_h|");
#endif

#ifdef LOGGING_KALMAN_ALTITUDE_CM
  next += snprintf(header_new + next, sizeof(header_new), "Kalman_h|");
#endif

#ifdef LOGGING_KALMAN_VELOCITY_CM
  next += snprintf(header_new + next, sizeof(header_new), "Kalman_v|");
#endif

#ifdef LOGGING_ALTITUDE_SETPOINT_CM
  next += snprintf(header_new + next, sizeof(header_new), "alt_set|");
#endif

#ifdef LOGGING_VOLTAGE_mV
  next += snprintf(header_new + next, sizeof(header_new), "V_mV|");
#endif

#ifdef LOGGING_CURRENT_cA
  next += snprintf(header_new + next, sizeof(header_new), "I_cA|");
#endif

#ifdef LOGGING_RC_COMMANDS
  next += snprintf(header_new + next, sizeof(header_new), "thr_c|pitch_c|roll_c|yaw_c|mode_c|");
#endif

#ifdef LOGGING_ENGINES
  next += snprintf(header_new + next, sizeof(header_new), "e1|e2|e3|e4|");
#endif

#ifdef LOGGING_ENGINES_FILTERED
  next += snprintf(header_new + next, sizeof(header_new), "e1_f|e2_f|e3_f|e4_f|");
#endif

#ifdef LOGGING_STATE_FLAGS
  next += snprintf(header_new + next, sizeof(header_new), "state_flags|");
#endif

  next += snprintf(header_new + next, sizeof(header_new), "error_flags|");

#ifdef LOGGING_RSSI_LEVEL
  next += snprintf(header_new + next, sizeof(header_new), "RSSI|");
#endif

  next += snprintf(header_new + next, sizeof(header_new), "EOL\r\n");
  next = 0;

  while (1)
  {
    // печатаем строку заголовков
    printf("%s", header_new);
#ifdef TELNET_CONF_MODE
    send(*client_fd, header_new, strlen(header_new), 0);
#endif
//пока не дошли до конца памяти (65365 страниц на микросхеме памяти), не нашли конец данных и не обнаружили флаг аварийной записи
    while ((page_address < 65535) && (empty_timestamp_flag == 0) && (emergency_mode_flag == 0)) 
    {
// копируем содержимое страницы в буфер микросхемы      
      W25N_page_data_read(page_address); 
      column_address = 0;
// пока в буфере помещается целый пакет данных
      while ((column_address < (2048 - sizeof(struct logging_data_set))) && (empty_timestamp_flag == 0) && (emergency_mode_flag == 0)) 
      {
// считываем первый пакет из буфера микросхемы себе         
        W25N_read(column_address, receiving_logs_buffer, sizeof(struct logging_data_set)); 

        p_to_set_to_log = (struct logging_data_set *)receiving_logs_buffer;
//если достигли конца записанных данных (в такое число считывается таймстэмп если ячейкм не запроганы (все FF))
        if (p_to_set_to_log->timestamp == 4294967295) empty_timestamp_flag = 1; 
//в случае если не достигли конца записанных в микруху данных 
//и не обнаружили флаг экстренной записи начинаем переводить числа в печатный вид при помощи snprintf
//второе условие нужно для того, чтобы после аварийно записанного пакета не дублировались данные с предыдущей страницы
//так как по emergency пишем в микруху внепланово, а у нее при этом в буфере остаются данные предыдущей считанной страницы
        if ((empty_timestamp_flag == 0) && (emergency_mode_flag == 0))
        {
          next += snprintf(message_to_print, sizeof(message_to_print), "%lu|", p_to_set_to_log->timestamp);
#ifdef LOGGING_ACCEL_1
          for (i = 0; i < 3; i++) next += snprintf(message_to_print + next, sizeof(message_to_print), "%d|", p_to_set_to_log->accel_1[i]);
#endif

#ifdef LOGGING_ACCEL_2
          for (i = 0; i < 3; i++) next += snprintf(message_to_print + next, sizeof(message_to_print), "%d|", p_to_set_to_log->accel_2[i]);
#endif

#ifdef LOGGING_ACCEL_1_MAX
          next += snprintf(message_to_print + next, sizeof(message_to_print), "%0.2f|", p_to_set_to_log->accel_1_max);
#endif

#ifdef LOGGING_ACCEL_2_MAX
          next += snprintf(message_to_print + next, sizeof(message_to_print), "%0.2f|", p_to_set_to_log->accel_2_max);
#endif

#ifdef LOGGING_GYRO_1
          for (i = 0; i < 3; i++) next += snprintf(message_to_print + next, sizeof(message_to_print), "%d|", p_to_set_to_log->gyro_1[i]);
#endif

#ifdef LOGGING_GYRO_2
          for (i = 0; i < 3; i++) next += snprintf(message_to_print + next, sizeof(message_to_print), "%d|", p_to_set_to_log->gyro_2[i]);
#endif

#ifdef LOGGING_AVG_ACCEL
          for (i = 0; i < 3; i++) next += snprintf(message_to_print + next, sizeof(message_to_print), "%d|", p_to_set_to_log->accel_avg[i]);
#endif

#ifdef LOGGING_AVG_GYRO
          for (i = 0; i < 3; i++) next += snprintf(message_to_print + next, sizeof(message_to_print), "%d|", p_to_set_to_log->gyro_avg[i]);
#endif
          
#ifdef LOGGING_QUATERNION
          for (i = 0; i < 4; i++) next += snprintf(message_to_print + next, sizeof(message_to_print), "%f|", p_to_set_to_log->q[i]);
#endif
          
#ifdef LOGGING_ANGLES
          for (i = 0; i < 3; i++) next += snprintf(message_to_print + next, sizeof(message_to_print), "%0.2f|", p_to_set_to_log->angles[i]);
#endif

#ifdef LOGGING_LIDAR_PID
          next += snprintf(message_to_print + next, sizeof(message_to_print), "%0.2f|%0.2f|%0.2f|%0.2f|",p_to_set_to_log->lidar_pid_error, p_to_set_to_log->lidar_pid_P_component, p_to_set_to_log->lidar_pid_I_component, p_to_set_to_log->lidar_pid_D_component);
#endif

#ifdef LOGGING_LIDAR_ALTITUDE_CM
          next += snprintf(message_to_print + next, sizeof(message_to_print), "%d|", p_to_set_to_log->lidar_altitude_cm);
#endif

#ifdef LOGGING_BARO_PID
          next += snprintf(message_to_print + next, sizeof(message_to_print), "%0.2f|%0.2f|%0.2f|%0.2f|",p_to_set_to_log->baro_pid_error, p_to_set_to_log->baro_pid_P_component, p_to_set_to_log->baro_pid_I_component, p_to_set_to_log->baro_pid_D_component);
#endif

#ifdef LOGGING_BARO_ALTITUDE_CM
          next += snprintf(message_to_print + next, sizeof(message_to_print), "%d|", p_to_set_to_log->baro_altitude_cm);
#endif

#ifdef LOGGING_KALMAN_ALTITUDE_CM
          next += snprintf(message_to_print + next, sizeof(message_to_print), "%d|", p_to_set_to_log->kalman_altitude_cm);
#endif

#ifdef LOGGING_KALMAN_VELOCITY_CM
          next += snprintf(message_to_print + next, sizeof(message_to_print), "%d|", p_to_set_to_log->kalman_velocity_cm);
#endif

#ifdef LOGGING_ALTITUDE_SETPOINT_CM
          next += snprintf(message_to_print + next, sizeof(message_to_print), "%d|", p_to_set_to_log->altitude_setpoint_cm);
#endif          

#ifdef LOGGING_VOLTAGE_mV
          next += snprintf(message_to_print + next, sizeof(message_to_print), "%d|", p_to_set_to_log->voltage_mv);
#endif

#ifdef LOGGING_CURRENT_cA
          next += snprintf(message_to_print + next, sizeof(message_to_print), "%d|", p_to_set_to_log->current_ca);
#endif

#ifdef LOGGING_RC_COMMANDS
          next += snprintf(message_to_print + next, sizeof(message_to_print), "%0.2f|", p_to_set_to_log->throttle_command);
          next += snprintf(message_to_print + next, sizeof(message_to_print), "%0.2f|", p_to_set_to_log->pitch_command);
          next += snprintf(message_to_print + next, sizeof(message_to_print), "%0.2f|", p_to_set_to_log->roll_command);
          next += snprintf(message_to_print + next, sizeof(message_to_print), "%0.2f|", p_to_set_to_log->yaw_command);
          next += snprintf(message_to_print + next, sizeof(message_to_print), "%d|", p_to_set_to_log->mode_command);
#endif

#ifdef LOGGING_ENGINES
          for (i = 0; i < 4; i++) next += snprintf(message_to_print + next, sizeof(message_to_print), "%ld|", p_to_set_to_log->engines[i]);
#endif

#ifdef LOGGING_ENGINES_FILTERED
          for (i = 0; i < 4; i++) next += snprintf(message_to_print + next, sizeof(message_to_print), "%ld|", p_to_set_to_log->engines_filtered[i]);
#endif

#ifdef LOGGING_STATE_FLAGS
          next += snprintf(message_to_print + next, sizeof(message_to_print), "%d|", p_to_set_to_log->state_flags);
#endif

          next += snprintf(message_to_print + next, sizeof(message_to_print), "%d|", p_to_set_to_log->error_flags);

#ifdef LOGGING_RSSI_LEVEL
          next += snprintf(message_to_print + next, sizeof(message_to_print), "%d|", p_to_set_to_log->rssi_level);
#endif
          next += snprintf(message_to_print + next, sizeof(message_to_print), "*\r\n"); // конец строки

// Печаем или отправляем сформированную строку в telnet
#ifdef TELNET_CONF_MODE
          send(*client_fd, message_to_print, next, 0);
#endif
          printf("%s", message_to_print);
          next = 0;
//переход на следующий пакет в буфере микросхемы
          column_address += sizeof(struct logging_data_set);
        }
//проверяем на emergency_mode_flag текущий только что выведенный пакет данных.
        if ((p_to_set_to_log->error_flags)&(0x01 << 15)) emergency_mode_flag = 1;
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
    }
  }
}
