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

#define ESP_WIFI_SSID "aquila"
#define ESP_WIFI_PASS "1234567890!"
#define ESP_WIFI_CHANNEL 1
#define MAX_STA_CONN 2
#define TELNET_PORT 23
#define BUFFER_SIZE 128

extern const char *TAG_SERVICE;

void sending_something_over_wifi(void *pvParameters)
{
    nvs_handle_t NVS_handle;
// Инициализируем NVS
    esp_err_t ret = nvs_flash_init();
    ESP_ERROR_CHECK(ret);
//открываем на считывание flash   
    ret = nvs_open("storage", NVS_READWRITE, &NVS_handle);
     if (ret != ESP_OK) 
        {
            ESP_LOGE(TAG_SERVICE,"Ошибка (%s) открытия NVS!\n", esp_err_to_name(ret));
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

//Отсюда и ниже активация сокета и запуск telnet 
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);

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
        
        int pos = 0;
        uint8_t P = 10;
        uint8_t I = 20;
        uint8_t D = 30;
        char message_to_print[100];
        char incoming_buffer[100];
        while (1)
        {
    /*    recv(client_fd, incoming_buffer, BUFFER_SIZE, 0);

        switch (incoming_buffer[0])
                {
                case 'P':
                    if (incoming_buffer[1] == '+') P++;
                    if (incoming_buffer[1] == '-') P--;
                    break;

                case 'I':
                    if (incoming_buffer[1] == '+') I++;
                    if (incoming_buffer[1] == '-') I--;
                    break;
                
                case 'D':
                    if (incoming_buffer[1] == '+') D++;
                    if (incoming_buffer[1] == '-') D--;
                    break;

                    default: break;
                }
*/
        pos = sprintf(message_to_print, "P %d, I %d, D %dP\r\n", P++, I--, D++);
        send(client_fd, message_to_print, pos, 0);  
        vTaskDelay(100/portTICK_PERIOD_MS);           
        }
    }
        close(client_fd);
}
