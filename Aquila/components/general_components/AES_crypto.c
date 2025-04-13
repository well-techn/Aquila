#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/aes.h"

extern char *TAG_CRYPTO;

esp_err_t encrypt_16_bytes_string(const uint8_t *input, const uint8_t *key, uint8_t *iv, uint8_t *encrypted_message) {
  
  esp_err_t ret = ESP_FAIL;
  uint8_t iv_copy[16] = {0};
//копируем IV  
  memcpy(iv_copy, iv, 16);
// создаем переменную нужного типа
  mbedtls_aes_context aes; 
// инициализируем AES                               
  mbedtls_aes_init(&aes);
// устанавливаем ключ                                 
  mbedtls_aes_setkey_enc(&aes, key, 256);                 
//криптуем данные
  ret = (esp_err_t)mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, 16, iv_copy, (unsigned char *)input, (unsigned char *)encrypted_message);

return ret;
}

esp_err_t encrypt_any_length_string(const uint8_t *input, uint16_t input_len, const uint8_t *key, const uint8_t *iv, uint8_t *encrypted_message) {
  
  esp_err_t ret = ESP_FAIL;
  uint16_t padded_input_len = 0;
  uint8_t iv_copy[16] = {0};

//сначала надо понять какая будет длина выровненных данных
  if (input_len < 16)			                                //если исходная длина меньше 16
    padded_input_len = 16;		                            //дополняем до 16
  else
    {padded_input_len = (input_len / 16 + 1) * 16;}	      // в противном случае округляем до минимального значения кратного 16
  
  ESP_LOGI(TAG_CRYPTO,"длина входных данных %d байт", input_len);
  ESP_LOGI(TAG_CRYPTO,"Пэддируем до %d", padded_input_len);

  uint8_t padded_input[padded_input_len];                 //создаем переменную для хранения дополненного массива

  memcpy(padded_input, input, input_len);	                //копируем в выделенную память полезную часть данных
  uint8_t pkcs5_value = (padded_input_len - input_len);    //определяем какой цифрой заполнять
  ESP_LOGI(TAG_CRYPTO,"Значение pkcs5 %02x", pkcs5_value);
  for (int i = input_len; i < padded_input_len; i++) {
    padded_input[i] = pkcs5_value;							            //заполняем
  }
  ESP_LOGI(TAG_CRYPTO,"Пэддированное исходящее сообщение: ");
  ESP_LOG_BUFFER_HEX(TAG_CRYPTO, padded_input, (uint16_t)sizeof(padded_input));

  memcpy(iv_copy, iv, 16);                                // копируем вектор iv 

  mbedtls_aes_context aes;                                //типовой процесс шифрования
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, key, 256);
  ret = (esp_err_t)mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, padded_input_len, iv_copy,
                        (unsigned char *)padded_input, encrypted_message);

   return(ret);
}

esp_err_t decrypt_string(uint8_t *encrypted_input, uint16_t input_len, const uint8_t *key, const uint8_t *iv, uint8_t *decrypted_message)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t iv_copy[16] = {0};
    
    memcpy(iv_copy, iv, 16);
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_dec(&aes, key, 256);
    ret = (esp_err_t)mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, input_len, iv_copy, (const unsigned char *)encrypted_input, (unsigned char *)decrypted_message);
    
    return ret;
}