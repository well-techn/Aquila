#include <stdio.h>
#include "i2c_master.h"
#include "wt_alldef.h"
#include "PCA9685.h"
#include "MCP23017.h"
#include "INA219.h"
#include "IST8310.h"
#include "FL3195.h"
#include "MS5611.h"
#include "inttypes.h"
#include "TfminiS.h"

i2c_master_bus_handle_t i2c_internal_bus_handle;
i2c_master_bus_handle_t i2c_external_bus_handle;

i2c_master_dev_handle_t PCA9685_dev_handle;
i2c_master_dev_handle_t MCP23017_dev_handle;
i2c_master_dev_handle_t INA219_dev_handle;
i2c_master_dev_handle_t IST8310_dev_handle;
i2c_master_dev_handle_t FL3195_dev_handle;
i2c_master_dev_handle_t MS5611_dev_handle;
i2c_master_dev_handle_t TFSMINI_dev_handle;

//инициализация i2c
void i2c_init_internal(uint8_t sda_pin,uint8_t scl_pin, uint8_t i2c_port)
{
   i2c_master_bus_config_t i2c_configuration = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = i2c_port,
    .scl_io_num = scl_pin,
    .sda_io_num = sda_pin,
    .glitch_ignore_cnt = 7,
    .intr_priority = 0,                         //I2C interrupt priority, if set to 0, driver will select the default priority (1,2,3).
    .flags.enable_internal_pullup = true,
  };

  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_configuration, &i2c_internal_bus_handle));

    i2c_device_config_t PCA9685_dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = PCA9685_address,
    .scl_speed_hz = I2C_PCA9685_FREQ_HZ,
    };

    i2c_device_config_t MCP23017_dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = MCP23017_ADDR,
    .scl_speed_hz = I2C_MCP23017_FREQ_HZ,
    };

    i2c_device_config_t INA219_dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = INA219_ADDR,
    .scl_speed_hz = I2C_INA219_FREQ_HZ,
    };

    i2c_device_config_t TFSMINI_dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = TFMINIS_I2C_ADDRESS,
    .scl_speed_hz = I2C_TFMINIS_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_internal_bus_handle, &PCA9685_dev_config, &PCA9685_dev_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_internal_bus_handle, &MCP23017_dev_config, &MCP23017_dev_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_internal_bus_handle, &INA219_dev_config, &INA219_dev_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_internal_bus_handle, &TFSMINI_dev_config, &TFSMINI_dev_handle));
}

void i2c_init_external(uint8_t sda_pin,uint8_t scl_pin, uint8_t i2c_port)
{
   i2c_master_bus_config_t i2c_configuration = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = i2c_port,
    .scl_io_num = scl_pin,
    .sda_io_num = sda_pin,
    .glitch_ignore_cnt = 7,
    .intr_priority = 0,                         //I2C interrupt priority, if set to 0, driver will select the default priority (1,2,3).
    .flags.enable_internal_pullup = true,
  };

   ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_configuration, &i2c_external_bus_handle));

    i2c_device_config_t FL3195_dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = FL3195_ADDR,
    .scl_speed_hz = I2C_FL3195_FREQ_HZ,
    };

    i2c_device_config_t IST8310_dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = IST8310_ADDR,
    .scl_speed_hz = I2C_IST8310_FREQ_HZ,
    };

    i2c_device_config_t MS5611_dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = MS5611_ADDRESS,
    .scl_speed_hz = I2C_MS5611_FREQ_HZ,
    };


    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_external_bus_handle, &FL3195_dev_config, &FL3195_dev_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_external_bus_handle, &IST8310_dev_config, &IST8310_dev_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_external_bus_handle, &MS5611_dev_config, &MS5611_dev_handle));

}


//запись в i2c
esp_err_t i2c_write_command(i2c_master_dev_handle_t i2c_dev, uint8_t command)
{
    esp_err_t ret;
    uint8_t write_buf;
    
    write_buf = command;
    ret = i2c_master_transmit(i2c_dev, &write_buf, 1, 0);
    
    return ret;
}

esp_err_t i2c_write_long_command(i2c_master_dev_handle_t i2c_dev, uint8_t* command, uint8_t length)
{
    esp_err_t ret;

    ret = i2c_master_transmit(i2c_dev, command, length, -1);
    
    return ret;
}

esp_err_t i2c_write_byte_to_address(i2c_master_dev_handle_t i2c_dev, uint8_t reg_address, uint8_t data_to_write)
{
    esp_err_t ret;
    uint8_t write_buf[2];
    
    write_buf[0] = reg_address;
    write_buf[1] = data_to_write;
    ret = i2c_master_transmit(i2c_dev, write_buf, 2, -1);
    
    return ret;
}

esp_err_t i2c_write_2_bytes_to_address(i2c_master_dev_handle_t i2c_dev, uint8_t reg_address, uint8_t MSB, uint8_t LSB)
{
    esp_err_t ret;
    uint8_t write_buf[3];
    
    write_buf[0] = reg_address;
    write_buf[1] = MSB;
    write_buf[2] = LSB;
    ret = i2c_master_transmit(i2c_dev, write_buf, 3, -1);
    
    return ret;
}

esp_err_t i2c_write_bytes_to_address(i2c_master_dev_handle_t i2c_dev, uint8_t reg_address, uint8_t number_of_bytes, uint8_t *data_to_write)
{
    esp_err_t ret;
    uint8_t i = 0;
    uint8_t write_buf[number_of_bytes+1];

    write_buf[0] = reg_address;
    for (i = 1; i < number_of_bytes+1; i++) {
        write_buf[i] = data_to_write[i-1];
    }
    
    ret = i2c_master_transmit(i2c_dev, write_buf, number_of_bytes + 1, -1);
    return ret;
}

//чтение i2c
uint8_t i2c_read_byte_from_address(i2c_master_dev_handle_t i2c_dev, uint8_t reg_address)
{
    uint8_t where_to_read_to;
    esp_err_t ret; 
    
    ret = i2c_master_transmit_receive(i2c_dev, &reg_address, 1, &where_to_read_to, 1, -1);

    return where_to_read_to;
}

uint16_t i2c_read_2_bytes_from_address(i2c_master_dev_handle_t i2c_dev, uint8_t reg_address)
{
    uint8_t where_to_read_to[2];
    uint16_t result = 0;
    esp_err_t ret; 
    
    ret = i2c_master_transmit_receive(i2c_dev, &reg_address, 1, where_to_read_to, 2, -1);
    result = (where_to_read_to[0] << 8) + where_to_read_to[1];  

    return result;
}

esp_err_t i2c_read_bytes_from_address(i2c_master_dev_handle_t i2c_dev, uint8_t read_start_address, uint8_t number_of_bytes_to_read, uint8_t* where_to_put_to)
{ 
    return i2c_master_transmit_receive(i2c_dev, &read_start_address, 1, where_to_put_to, number_of_bytes_to_read, -1);
}

esp_err_t i2c_read_bytes(i2c_master_dev_handle_t i2c_dev, uint8_t number_of_bytes_to_read, uint8_t* where_to_put_to)
{ 
    return i2c_master_receive(i2c_dev, where_to_put_to, number_of_bytes_to_read, -1);
}


esp_err_t checking_address_at_the_bus(i2c_master_bus_handle_t bus_handle, uint8_t device_address)
{
    esp_err_t ret = ESP_FAIL;
    ret = i2c_master_probe(bus_handle, device_address, -1);
    if (ret == ESP_OK) printf ("device with address 0x%02x is discovered\n", device_address);
    else printf ("device with address 0x%02x is not discovered\n", device_address);

    return ret;
}

