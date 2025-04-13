#ifndef AES_CRYPTO_H
#define AES_CRYPTO_H

esp_err_t encrypt_16_bytes_string(const unsigned char *input, const uint8_t *key, uint8_t *iv, uint8_t *encrypted_message);
esp_err_t encrypt_any_length_string(uint8_t *input, uint16_t input_len, const uint8_t *key, const uint8_t *iv, uint8_t *encrypted_message);
esp_err_t decrypt_string(unsigned char *encrypted_input, uint16_t input_len, const uint8_t *key, const uint8_t *iv, uint8_t *decrypted_message);

#endif