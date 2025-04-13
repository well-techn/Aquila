//Задача записи логов во внешнюю flash-память
//Принимает на вход данные на запись из main_flyibg_cycle и записывает их в Winbond 

//системные библиотеки
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "winbondW25N.h"

extern  char *TAG_W25N;
extern QueueHandle_t W25N01_queue; //очередь для передачи обработанных данных от пульта в main_flying_cycle

void writing_logs_to_flash(void * pvParameters)
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