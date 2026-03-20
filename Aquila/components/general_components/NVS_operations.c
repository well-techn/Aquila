#include "nvs_flash.h"
#include "esp_log.h"

extern const char *TAG_NVS;

esp_err_t NVS_prepare(nvs_handle_t* NVS_handle, const char* name_space)
{
    static bool nvs_initialized = false;

    if (!nvs_initialized) 
    {
//инициализируем NVS    
        esp_err_t err = nvs_flash_init();
// если нет места - пробуем стереть и переинициазировать, при этом сотрутся все переменные
        if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) 
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
//если не помогло
        if (err != ESP_OK) return err;

        nvs_initialized = true;
    }
    ESP_LOGI(TAG_NVS,"Открываем NVS... ");
    esp_err_t err = nvs_open(name_space, NVS_READWRITE, NVS_handle);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG_NVS,"Ошибка (%s) открытия NVS!\n", esp_err_to_name(err));
        return(err);
    } 

        ESP_LOGI(TAG_NVS,"NVS [%s] открыт",name_space);
        return(ESP_OK);        
}

//функция для чтения массива int16
esp_err_t nvs_read_i16_array(nvs_handle_t handle, const char* prefix, int16_t* array, size_t size) {
    char key[16]; 
    
    for (size_t i = 0; i < size; i++) 
    {
        snprintf(key, sizeof(key), "%s[%d]", prefix, (uint8_t)i);
        esp_err_t err = nvs_get_i16(handle, key, &array[i]);

        if(err != ESP_OK)
        {
            ESP_LOGW("NVS", "Ключ %s не найден или ошибка: %s", key, esp_err_to_name(err));
            return(err);
        }
        ESP_LOGD("NVS", "Считано %s = %d", key, array[i]);
    }
    return(ESP_OK);
}

//функция для чтения массива double, хранимых как u64, так как в IDF нет функции для записи/считывания double из NVS
esp_err_t nvs_read_double_array(nvs_handle_t handle, const char* prefix, double* array, size_t size) {
    char key[16];
    uint64_t temp;
    
    for (size_t i = 0; i < size; i++) 
    {
        snprintf(key, sizeof(key), "%s[%d]", prefix, (uint8_t)i);
        esp_err_t err = nvs_get_u64(handle, key, &temp);
        
         if(err != ESP_OK)
        {
            ESP_LOGW("NVS", "Ключ %s не найден или ошибка: %s", key, esp_err_to_name(err));
            return(err);
        }
        array[i] = *(double*)&temp;
        ESP_LOGD("NVS", "Считано %s = %f", key, array[i]);
    }
    return(ESP_OK);
}