#ifndef NVS_OPERATIONS_h
#define NVS_OPERATIONS_h

esp_err_t NVS_prepare(nvs_handle_t* NVS_handle, const char* name_space);
esp_err_t nvs_read_i16_array(nvs_handle_t handle, const char* prefix, int16_t* array, size_t size);
esp_err_t nvs_read_double_array(nvs_handle_t handle, const char* prefix, double* array, size_t size);

#endif
