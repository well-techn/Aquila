#include "ICM45686.h"
#include "wt_spi.h" 
#include "esp_log.h"
#include <rom/ets_sys.h>
#include "math.h"
#include "wt_alldef.h"

extern char *TAG_ICM45686;


esp_err_t ICM45686_communication_check(spi_device_handle_t MPU_handle) {

    esp_err_t err = ESP_FAIL;
    uint8_t response_value = SPI_read_byte(MPU_handle, 0, 0, 8, ICM45686_WHO_I_AM | 0x80, 0);
    if (response_value == ICM45686_WHO_I_AM_RESPONSE) 
    {
        ESP_LOGI(TAG_ICM45686, "Связь с ICM45686 установлена, получен ответ %02X", response_value);
        err = ESP_OK;
    }
    else 
    {
        ESP_LOGE(TAG_ICM45686, "Связь с ICM45686 не установлена, получен ответ %02X", response_value);
    }
    
    return err;
}

//так как регистры для адреса непрямого регистра ICM45686_IREG_ADDR_15_8, ICM45686_IREG_ADDR_7_0 и регистр ICM45686_IREG_DATA для значения идут друг за другом 
//можно просто записать последовательно 3 байта начиная с ICM45686_IREG_ADDR_15_8
void ICM45686_indirect_reg_write(spi_device_handle_t MPU_handle, uint8_t shift, uint8_t indir_reg_address, uint8_t data_to_write)
{
    uint8_t temp[3]= {shift, indir_reg_address, data_to_write};
    SPI_write_bytes(MPU_handle, 0, 0, 8, ICM45686_IREG_ADDR_15_8, 0, temp, 3);
}

uint8_t ICM45686_indirect_reg_read(spi_device_handle_t MPU_handle, uint8_t shift, uint8_t indir_reg_address)
{
    uint8_t temp[2]= {shift, indir_reg_address};
//первой транзакцией прописываем адрес запрашиваемого регистра в ICM45686_IREG_ADDR_15_8, ICM45686_IREG_ADDR_7_0
    SPI_write_bytes(MPU_handle, 0, 0, 8, ICM45686_IREG_ADDR_15_8, 0, temp, 2);
//второй считываем значение из ICM45686_IREG_DATA
    return SPI_read_byte(MPU_handle, 0, 0, 8, ICM45686_IREG_DATA | SPI_READ_FLAG, 0);
}

void ICM45686_soft_reset(spi_device_handle_t MPU_handle) {

    esp_err_t err = ESP_FAIL;
    SPI_write_byte(MPU_handle, 0, 0, 8, ICM45686_REG_MISC2, 0, 0b00000010);
    ets_delay_us(1000);
}



esp_err_t ICM45686_init(spi_device_handle_t MPU_handle,
                        uint16_t sampling_freq_hz,
                        uint16_t integrated_lpf_ODR_divider,
                        uint16_t accel_full_scale_g,
                        uint16_t gyro_full_scale_dps) 
{
    uint8_t odr_register = 0;       // Совмещенный ODR для акселя и гиро
    uint8_t accel_scale_reg = 0;
    uint8_t gyro_scale_reg = 0;
    uint8_t filter_reg = 0;

//настройка ODR (Sampling Frequency)
    switch (sampling_freq_hz) {
        case 6400: odr_register = 0x03; break;
        case 3200: odr_register = 0x04; break;
        case 1600: odr_register = 0x05; break;
        case 800:  odr_register = 0x06; break;
        case 400:  odr_register = 0x07; break;
        case 200:  odr_register = 0x08; break;
        case 100:  odr_register = 0x09; break;
        default: { ESP_LOGE(TAG_ICM45686, "Ошибка частоты ODR"); return ESP_FAIL; }
    }

//настройка шкалы акселерометра
    switch (accel_full_scale_g) {
        case 2:  accel_scale_reg = 0x04; break; // +-2g
        case 4:  accel_scale_reg = 0x03; break; // +-4g
        case 8:  accel_scale_reg = 0x02; break; // +-8g
        case 16: accel_scale_reg = 0x01; break; // +-16g
        case 32: accel_scale_reg = 0x00; break; // +-32g
        default: { ESP_LOGE(TAG_ICM45686, "Ошибка шкалы акселя"); return ESP_FAIL; }
    }

//настройка шкалы гироскопа
    switch (gyro_full_scale_dps) {
        case 15:  gyro_scale_reg = 0x08; break;
        case 31:  gyro_scale_reg = 0x07; break;
        case 62: gyro_scale_reg = 0x06; break;
        case 125: gyro_scale_reg = 0x05; break;
        case 250:  gyro_scale_reg = 0x04; break;
        case 500:  gyro_scale_reg = 0x03; break;
        case 1000: gyro_scale_reg = 0x02; break;
        case 2000: gyro_scale_reg = 0x01; break;
        case 4000: gyro_scale_reg = 0x00; break;
        default: { ESP_LOGE(TAG_ICM45686, "Ошибка шкалы гиро"); return ESP_FAIL; }
    }

//настройка встроенного LPF - выбирается относительно ODR

    // switch (integrated_lpf_value_hz) {
    //     case 0: filter_reg = 0x00; break; 
    //     case (sampling_freq_hz / 4):  filter_reg = 0x01; break; 
    //     case (sampling_freq_hz / 8):  filter_reg = 0x02; break;
    //     case (sampling_freq_hz / 16):  filter_reg = 0x03; break;
    //     case (sampling_freq_hz / 32):  filter_reg = 0x04; break;
    //     case (sampling_freq_hz / 64):  filter_reg = 0x05; break;
    //     case (sampling_freq_hz / 128):  filter_reg = 0x06; break;
    //     case (sampling_freq_hz / 128):  filter_reg = 0x07; break;
    //     default:  { ESP_LOGE(TAG_ICM45686, "Ошибка параметра LPF фильтра, должно быть кратно ODR"); return ESP_FAIL; }
    // }

        switch (integrated_lpf_ODR_divider) {
        case 0: filter_reg = 0x00; break; 
        case (4):  filter_reg = 0x01; break; 
        case (8):  filter_reg = 0x02; break;
        case (16):  filter_reg = 0x03; break;
        case (32):  filter_reg = 0x04; break;
        case (64):  filter_reg = 0x05; break;
        case (128):  filter_reg = 0x06; break;
        default:  { ESP_LOGE(TAG_ICM45686, "Ошибка параметра LPF фильтра, должно быть степень 2 или 0 (выкл)"); return ESP_FAIL; }
    }

    esp_err_t err = ESP_OK;
    uint8_t reg_value = 0;

//последовательность записи настроек для ICM45686
uint8_t config_data[9][2] = {
    {ICM45686_PWR_MGMT0,  0b00001111}, // Настройка режимов гироскопа
    {ICM45686_ACCEL_CONFIG0, (accel_scale_reg << 4) | odr_register},
    {ICM45686_GYRO_CONFIG0,  (gyro_scale_reg << 4) | odr_register},
    {ICM45686_INT1_CONFIG0,    0b00000100}, // настойка прерывания по data ready
    {ICM45686_INT1_CONFIG2,    0b00000001}, // настойка уровней прерывания (активный 1, pulse)
    {ICM45686_IPREG_SYS2_REG_123, 0x14 | 0b00000010}, // включение interpolator и FIR для гироскопа (0x14 дефолтное значени регистра)  
    {ICM45686_IPREG_SYS2_REG_131, filter_reg}, // встроеный LPF для акселерометра
    {ICM45686_IPREG_SYS1_REG_166, 0x1B | 0b01000000}, // включение interpolator и FIR для гироскопа (0x1B дефолтное значени регистра)            
    {ICM45686_IPREG_SYS1_REG_172, 0x80 | filter_reg}, // встроеный LPF для гироскопа 
};

//запись настроек в прямые регистры
    for (uint8_t i = 0; i < 5; i++) {
        SPI_write_byte(MPU_handle, 0, 0, 8, config_data[i][0], 0, config_data[i][1]);
        ets_delay_us(100);
    }
//запись настроек в непрямые регистры
    for (uint8_t i = 5; i < 7; i++) {
        ICM45686_indirect_reg_write(MPU_handle, ICM45686_IPREG_SYS2, config_data[i][0], config_data[i][1]);
        ets_delay_us(100);
    }
    for (uint8_t i = 7; i < 9; i++) {
        ICM45686_indirect_reg_write(MPU_handle, ICM45686_IPREG_SYS1, config_data[i][0], config_data[i][1]);
        ets_delay_us(100);
    }

//проверка записанных настроек в прямых регистрах
    for (uint8_t i = 0; i < 5; i++) {
        reg_value = SPI_read_byte(MPU_handle, 0, 0, 8, config_data[i][0] | 0x80, 0);
        if (reg_value != config_data[i][1]) {
            err = ESP_FAIL;
            ESP_LOGE(TAG_ICM45686, "Ошибка в рег %02X, считано %d", config_data[i][0], reg_value);
        }
    }

//проверка записанных настроек в непрямых регистрах
    for (uint8_t i = 5; i < 7; i++) {
        reg_value = ICM45686_indirect_reg_read(MPU_handle, ICM45686_IPREG_SYS2, config_data[i][0]);
        if (reg_value != config_data[i][1]) {
            err = ESP_FAIL;
            ESP_LOGE(TAG_ICM45686, "Ошибка в рег %02X, считано %d", config_data[i][0], reg_value);
        }
    }

    for (uint8_t i = 7; i < 9; i++) {
        reg_value = ICM45686_indirect_reg_read(MPU_handle, ICM45686_IPREG_SYS1, config_data[i][0]);
        if (reg_value != config_data[i][1]) {
            err = ESP_FAIL;
            ESP_LOGE(TAG_ICM45686, "Ошибка в рег %x, считано %d", config_data[i][0], reg_value);
        }
    }

    //if (err == ESP_OK) ESP_LOGI(TAG_ICM45686, "ICM45686 настроен");
    
    return err;
}


