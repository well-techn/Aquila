#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wt_alldef.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include <lwip/sockets.h>
#include "reading_logs_from_external_flash.h"
#include "gyro_calibration.h"
#include "advanced_acc_calibration.h"
#include "advanced_mag_calibration.h"
#include "engines_test.h"
#include "printing_calibration_coefficients.h"


#define ESP_WIFI_SSID "aquila"
#define ESP_WIFI_PASS "1234567890!"
#define ESP_WIFI_CHANNEL 1
#define MAX_STA_CONN 2
#define TELNET_PORT 23
#define BUFFER_SIZE 128

static const char *TAG = "WiFi";
extern TaskHandle_t task_handle_init;


char flight_time_buffer[57] = "********** Налет 00ч 00м 00с ***********\r\n";
char *welcome_messages[] =
    {
        "\r\n\nдобро пожаловать в telnet проекта Aquila\r\n",
        "************ Версия " FW_VERSION " ************\r\n",
        flight_time_buffer, 
        "* Устройство в режиме конфигурирования *\r\n\n",
        "Выберите пункт меню\r\n",
        "1 -> скачивание логов\r\n",
        "2 -> калибровка гироскопов\r\n",
        "3 -> тестирование двигателей\r\n",
        "4 -> калибровка ESC (пока не готово)\r\n",
        "5 -> продвинутая калибровка акселерометра (по magnetto)\r\n",
        "6 -> продвинутая калибровка магнетометра (по magnetto)\r\n",
        "7 -> вывести сохраненные калибровочные коэффициенты\r\n",
        "8 -> продолжить загрузку в обычном режиме (не использовать)\r\n",
        "ESC -> перезапуск\r\n\n",
        NULL};

char r_1[] = "1 - Запускаем скачивание логов, ESC для прерывания\r\n";
char r_2[] = "2 - Запускаем простую калибровку гироскопов\r\n";
char r_3[] = "3 - Запускаем тестирование двигателей через 10 секунд, убедитесь в безопасности операции. ESC для прерывания\r\n";
char r_5[] = "5 - Запускаем продвинутую калибровку акселерометра (по magnetto)\r\n";
char r_6[] = "6 - Запускаем продвинутую калибровку магнетометра (по magnetto)\r\n";
char r_7[] = "7 - Считываем из flash калибровочные коэффициенты\r\n";
char r_8[] = "8 - Выключаем WiFi и продолжаем обычную загрузку\r\n";
char n_1[] = "Неизвестное меню, повторите ввод\r\n";
char esc[] = "ESC - перезапускаемся\n\n";

void configuration_mode_telnet(void *arg)
{
    nvs_handle_t NVS_handle;
    uint32_t flight_time;

// Инициализируем NVS
    esp_err_t ret = nvs_flash_init();
    ESP_ERROR_CHECK(ret);
//открываем на считывание flash   
    ret = nvs_open("storage", NVS_READWRITE, &NVS_handle);
     if (ret != ESP_OK) 
        {
            ESP_LOGE(TAG,"Ошибка (%s) открытия NVS!\n", esp_err_to_name(ret));
        } 
        else 
        {
//считываем flight_time (время налета в секундах)
//если это первый запуск и переменной нет - создаем ее
  ret = nvs_get_u32(NVS_handle, "flight_time", &flight_time); 
  switch (ret) {
      case ESP_OK:
          ESP_LOGD(TAG,"flight time = %ld", flight_time);
          break;
      case ESP_ERR_NVS_NOT_FOUND:
          ESP_LOGW(TAG,"flight time не определен, используем нулевое значение");
          //ret = nvs_set_u32(NVS_handle, "flight_time", 0);
          break;
      default :
          ESP_LOGE(TAG,"Error (%s) reading!\n", esp_err_to_name(ret));
        }
//разбиваем считанное из flash значение flight_time в сукендах на часы, минуты и секунды
        uint8_t hours = flight_time / 3600;
        uint8_t minutes = (flight_time % 3600) / 60;
        uint8_t seconds = flight_time % 60;
//модифицируем стринг welcome_message, внося соответствующие изменения
        sprintf(flight_time_buffer, "********** Налет %02uч %02uм %02uс ***********\r\n", hours, minutes, seconds);
        }
    // Инициализируем WiFi
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_SSID,
            .ssid_len = strlen(ESP_WIFI_SSID),
            .channel = ESP_WIFI_CHANNEL,
            .password = ESP_WIFI_PASS,
            .max_connection = MAX_STA_CONN,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
            .authmode = WIFI_AUTH_WPA3_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
            .authmode = WIFI_AUTH_WPA2_PSK,
#endif
            .pmf_cfg = {
                .required = true,
            },
        },
    };
    if (strlen(ESP_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Настройка WiFi завершена. SSID:%s пароль:%s канал:%d", ESP_WIFI_SSID, ESP_WIFI_PASS, ESP_WIFI_CHANNEL);
//Отсюда и ниже активация сокета и запуск telnet 
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    char buffer[BUFFER_SIZE];

    // Создаем TCP-сокет
    server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_fd < 0)
    {
        printf("Ошибка создания сокета\n");
        vTaskDelete(NULL);
    }

// Настраиваем серверный адрес
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(TELNET_PORT);

// Привязываем сокет к адресу
    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)))
    {
        printf("Ошибка привязки сокета к адресу\n");
        close(server_fd);
        vTaskDelete(NULL);
    }

// Слушаем входящие подключения
    if (listen(server_fd, 1))
    {
        printf("Ошибка прослушивания\n");
        close(server_fd);
        vTaskDelete(NULL);
    }
    printf("Telnet-сервер запущен на порту %d\n", TELNET_PORT);

    while (1)
    {
        // Принимаем новое подключение (блокирующая функция, находимся в этой строке пока не примем подключение)
        client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &addr_len);
        if (client_fd < 0)
        {
            printf("Ошибка принятия подключения\n");
            continue;
        }
        printf("Telnet клиент подключен\n");

// Если клиент подключен отправляем сообщения с возможными опциями меню
        for (uint8_t i = 0; welcome_messages[i] != NULL; i++)
        {
            send(client_fd, welcome_messages[i], strlen(welcome_messages[i]), 0);
        }

        while (1)
        {
//ожидаем прихода ответа в блокирующем режиме
//можно поиграться с получением ровно len байт recv(sock, buf, len, MSG_WAITALL);            
            int16_t len = recv(client_fd, buffer, BUFFER_SIZE, 0);
            //printf("%d\n", len);

            if (len < 0) {printf("Ошибка recv\n"); break;}
            if (len == 0) {printf("Telnet cоединение закрыто\n"); break;}
            if (len <= 3)
            {
                switch (buffer[0])
                {
                case '1':
                    send(client_fd, r_1, sizeof(r_1), 0);
                    xTaskCreate(reading_logs_from_external_flash,"reading_logs_from_external_flash",4096,(void *)&client_fd,0,NULL);
                    break;

                case '2':
                    send(client_fd, r_2, sizeof(r_2), 0);
                    xTaskCreate(gyro_calibration,"gyro_calibration",4096,(void *)&client_fd,0,NULL);  
                    break;

                case '3':
                    send(client_fd, r_3, sizeof(r_3), 0);
                    xTaskCreate(engines_test,"engines_test",4096,(void *)&client_fd,0,NULL);  
                    break;

                case '5':
                    send(client_fd, r_5, sizeof(r_5), 0);
                    xTaskCreate(advanced_acc_calibration,"advanced_acc_calibration",16384,(void *)&client_fd,0,NULL);    
                    break;

                case '6':
                    send(client_fd, r_6, sizeof(r_6), 0);
                    xTaskCreate(advanced_mag_calibration,"advanced_mag_calibration",16384,(void *)&client_fd,0,NULL);    
                    break;

                case '7':
                    send(client_fd, r_7, sizeof(r_7), 0);
                    xTaskCreate(printing_calibration_coefficients,"printing_calibration_coefficients",16384,(void *)&client_fd,0,NULL); 
                    break;
                 
                case '8':
                    send(client_fd, r_8, sizeof(r_8), 0);
                    ESP_ERROR_CHECK(esp_wifi_stop());
                    ESP_ERROR_CHECK(esp_wifi_deinit());
                    //ESP_ERROR_CHECK(esp_netif_deinit());
                    vTaskDelete(NULL);
                    vTaskResume(task_handle_init);
                    break;
                
//                case '9':
//                    send(client_fd, r_9, sizeof(r_9), 0);
//                    xTaskCreate(sending_something_during_flight,"sending_something_during_flight",16384,(void *)&client_fd,0,NULL); 
//                    break;

                case 0x1B:  //ESC
                    send(client_fd, esc, sizeof(esc), 0);
                    vTaskDelay(200/portTICK_PERIOD_MS);
                    esp_restart();
                    break;

                default:
                    //send(client_fd, n_1, sizeof(n_1), 0);
                    break;
                    }
            }
        }
        close(client_fd);
    }
}
