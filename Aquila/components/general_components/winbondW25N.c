#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <ctype.h>
#include "winbondW25N.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wt_spi.h"
#include "esp_log.h"
#include <rom/ets_sys.h>
#include "wt_alldef.h"
#include "esp_random.h"
#ifdef TELNET_CONF_MODE
  #include "lwip/err.h"
  #include "lwip/sys.h"
  #include <lwip/sockets.h>
#endif

extern spi_device_handle_t W25N01;
extern char *TAG_W25N;

void W25N_reset(void) {
  SPI_write_byte(W25N01, 0, 0, 0, 0, 0, W25N_RESET);
  vTaskDelay(1/portTICK_PERIOD_MS);
  ESP_LOGI(TAG_W25N,"W25N сброшена");
}

esp_err_t W25N_read_JEDEC_ID(void) {

  esp_err_t err = ESP_FAIL;
  uint8_t buf[3] = "0";

  SPI_read_bytes (W25N01, 8, W25N_READ_JEDEC_ID, 0, 0, 8, buf, 3);
  if ((buf[0] == WINBOND_MAN_ID) && (buf[1] == W25N01GV_DEV_ID_HI) && (buf[2] == W25N01GV_DEV_ID_LO)) 
  {
    err = ESP_OK;
    ESP_LOGI(TAG_W25N,"Проверка JEDEC пройдена");
  } 
  else ESP_LOGE(TAG_W25N,"Проверка JEDEC не пройдена, считаны значения %02x, %02x, %02x\n",buf[0],buf[1],buf[2]);
  
  return err;
}

void W25N_read_all_status_registers(void) {

uint8_t buff;

  SPI_read_bytes (W25N01, 8, W25N_READ_STATUS_REG, 8, W25N_PROT_REG_SR1, 0, &buff, 1);
  printf ("SR-1 is %02x\n",buff);
  SPI_read_bytes (W25N01, 8, W25N_READ_STATUS_REG, 8, W25N_CONFIG_REG_SR2, 0, &buff, 1);
  printf ("SR-2 is %02x\n",buff);
  SPI_read_bytes (W25N01, 8, W25N_READ_STATUS_REG, 8, W25N_STAT_REG_SR3, 0, &buff, 1);
  printf ("SR-3 is %02x\n",buff);
}

uint8_t W25N_read_STATUS_register(void) {

  uint8_t buff;

  SPI_read_bytes (W25N01, 8, W25N_READ_STATUS_REG, 8, W25N_STAT_REG_SR3, 0, &buff, 1);
  return (buff);
}

void W25N_write_status_register(uint8_t status_register_address, uint8_t reg_value) {

  SPI_write_byte(W25N01, 8, W25N_WRITE_STATUS_REG, 8, status_register_address, 0, reg_value);
}

void W25N_write_enable(void) {
  SPI_write_byte(W25N01, 0, 0, 0, 0, 0, W25N_WRITE_ENABLE);
  ets_delay_us(1);
}

void W25N_write_disable(void) {
  SPI_write_byte(W25N01, 0, 0, 0, 0, 0, W25N_WRITE_DISABLE);
  ets_delay_us(1);
}

void W25N_program_data_load(uint16_t column_address, uint8_t* data_to_write, uint16_t number_of_bytes) {
 
  W25N_write_enable();
  SPI_write_bytes (W25N01, 8, W25N_PROG_DATA_LOAD, 16, column_address, 0, data_to_write, number_of_bytes);
 }

void W25N_random_program_data_load(uint16_t column_address, uint8_t* data_to_write, uint16_t number_of_bytes) {
  W25N_write_enable(); 
  SPI_write_bytes (W25N01, 8, W25N_RAND_PROG_DATA_LOAD, 16, column_address, 0, data_to_write, number_of_bytes);
}

void W25N_program_execute(uint16_t page_address) {
  
  W25N_write_enable();
  
  uint8_t buf[2];
  buf[0] = (uint8_t)((page_address & 0xFF00) >> 8); //MSB
  buf[1] = (uint8_t) page_address;  //LSB
  
  SPI_write_bytes (W25N01, 8, W25N_PROG_EXECUTE, 0, 0, 8, buf, 2);
  vTaskDelay(1/portTICK_PERIOD_MS); //700us delay tPP
}

void W25N_page_data_read(uint16_t page_address) {
  uint8_t buf[2];
  buf[0] = (uint8_t)((page_address & 0xFF00) >> 8); //MSB
  buf[1] = (uint8_t) page_address;  //LSB
  SPI_write_bytes (W25N01, 8, W25N_PAGE_DATA_READ, 0, 0, 8, buf, 2);
  ets_delay_us(50); //verified practically
  }

void W25N_read(uint16_t column_address, uint8_t* where_to_put_to, uint16_t number_of_bytes) {

  SPI_read_bytes(W25N01, 8, W25N_READ, 16, column_address, 8, where_to_put_to, number_of_bytes);
  
}

void W25N_block_erase(uint16_t page_address) {
  
  W25N_write_enable();
    uint8_t buf[2];
  buf[0] = (uint8_t)((page_address & 0xFF00) >> 8); //MSB
  buf[1] = (uint8_t) page_address;  //LSB
  SPI_write_bytes (W25N01, 8, W25N_BLOCK_ERASE, 0, 0, 8, buf, 2);
  vTaskDelay(10/portTICK_PERIOD_MS); //tBE = 2..10ms
}

void W25N_erase_all(void) 
{
  uint8_t buf[4];
  uint16_t page_address = 0;
  uint8_t empty_block_flag = 0;

  while ((page_address < 65471) && (!(empty_block_flag)))
  {
    ESP_LOGI(TAG_W25N,"Считываем страницу  %d",page_address);
    W25N_page_data_read(page_address);                                    //копируем содержимое страницы в буфер микросхемы
    W25N_read(0, buf, 4);                                                 //считываем первые 4 байта с начала буфера
    
    ESP_LOGI(TAG_W25N,"Первые 4 байта %d, %d, %d, %d",buf[0],buf[1],buf[2],buf[3]);
    
    if ((buf[0] == 0xFF) && (buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF)) empty_block_flag = 1;  //если 4 первые байта страницы пустые - значит все что дальше пустое 
    if (!(empty_block_flag))
    {
    ESP_LOGI(TAG_W25N,"Стираем блок начиная со страницы %d",page_address);
    W25N_write_enable();
    buf[0] = (uint8_t)((page_address & 0xFF00) >> 8); //MSB
    buf[1] = (uint8_t) page_address;  //LSB
    SPI_write_bytes (W25N01, 8, W25N_BLOCK_ERASE, 0, 0, 8, buf, 2);
    vTaskDelay(10/portTICK_PERIOD_MS); //tBE = 2..10ms
    if (W25N_read_STATUS_register() & 0b00000100) ESP_LOGE(TAG_W25N,"Ошибка стирания на странице %d\n",page_address); //проверяем флаг Erase_fail
    page_address+=64;
    }
  }
  ESP_LOGI(TAG_W25N,"Полная очистка успешно завершена, было стерто %d страниц (%d блоков)\n", page_address, page_address/64);     //компенсируем крайний инкремент
}

esp_err_t winbond_read_write_test(void)
{
  esp_err_t err = ESP_FAIL;
  uint8_t i = 0;
  uint16_t result = 0;
  uint8_t receiving_logs_buffer[10];
  
  ESP_LOGD(TAG_W25N,"Erasing block.....");
  W25N_block_erase(0x0000);
 
  ESP_LOGD(TAG_W25N,"Load data to buffer.....");
  esp_fill_random(receiving_logs_buffer, 50);
  ESP_LOGD(TAG_W25N,"Printing original values.....");
  for (i=0;i<10;i++) ESP_LOGD(TAG_W25N,"%02x ",receiving_logs_buffer[i]);
  
  ESP_LOGD(TAG_W25N,"Loading data from column 0....");
  W25N_program_data_load(0x0000, receiving_logs_buffer, 10);
  ESP_LOGD(TAG_W25N,"Executing programming to page 0");
  W25N_program_execute(0x0000);
  
  vTaskDelay(10/portTICK_PERIOD_MS);

  ESP_LOGD(TAG_W25N,"Page data read.....");
  W25N_page_data_read(0x0000);
  
  ESP_LOGD(TAG_W25N,"Data read.....");
  W25N_read(0x0000, receiving_logs_buffer, 10);

  ESP_LOGD(TAG_W25N,"Printing received values.....");
  for (i=0;i<10;i++) ESP_LOGD(TAG_W25N,"%02x ",receiving_logs_buffer[i]);

  for (i=0;i<10;i++) result+=(receiving_logs_buffer[i] - receiving_logs_buffer[i]);
  
  if (result == 0) {err = ESP_OK; ESP_LOGI(TAG_W25N,"W25N01 write - read test successfully performed");}
  else ESP_LOGE(TAG_W25N,"Write - Read test failed, result of operaion is %d while has to be 0", result);
 
  return err;
}