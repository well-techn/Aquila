#include "ve_spi.h"
#include "ve_alldef.h"

spi_device_handle_t MPU9255;
spi_device_handle_t HMC5983; //8MHz
spi_device_handle_t SD_card; 
spi_device_handle_t W25N01; //104MHz
spi_device_handle_t PMW3901;  //2MHz max
spi_device_handle_t MPU6000_1; //1MHz for all, 20MHz for data and interrupt
spi_device_handle_t MPU6000_2;

//uint8_t spi_RX_buffer[32] = {0};
//uint8_t spi_TX_buffer[32] = {0};

void SPI_init()
{
  spi_bus_config_t IMUs_SPI_buscfg = {
      .mosi_io_num=IMU_SPI_MOSI,
      .miso_io_num=IMU_SPI_MISO,
      .sclk_io_num=IMU_SPI_SCLK,
      .quadwp_io_num=-1,
      .quadhd_io_num=-1
  };
  ESP_ERROR_CHECK(spi_bus_initialize(IMU_SPI, &IMUs_SPI_buscfg, SPI_DMA_CH_AUTO));

 spi_device_interface_config_t devcfg_MPU6000_1 = {         //1MHz for config, 20MHz for sensor registers
    .clock_speed_hz=1000000,
    .duty_cycle_pos=128,                                    //Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty). Setting this to 0 (=not setting it) is equivalent to setting this to 128.
    .mode=3,                                                //SPI mode, representing a pair of (CPOL, CPHA) configuration
    .spics_io_num=GPIO_CS_MPU6000_1,
    .cs_ena_posttrans=3,                                    //Keep the CS low 3 cycles after transaction
    .queue_size=3,
    .input_delay_ns = 0
 };
 ESP_ERROR_CHECK(spi_bus_add_device(IMU_SPI, &devcfg_MPU6000_1, &MPU6000_1));
 
  spi_device_interface_config_t devcfg_MPU6000_2 = {
    .clock_speed_hz=1000000,
    .duty_cycle_pos=128,                                    //Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty). Setting this to 0 (=not setting it) is equivalent to setting this to 128.
    .mode=3,                                                //SPI mode, representing a pair of (CPOL, CPHA) configuration
    .spics_io_num=GPIO_CS_MPU6000_2,
    .cs_ena_posttrans=3,                                    //Keep the CS low 3 cycles after transaction
    .queue_size=3,
    .input_delay_ns = 0
 };
 ESP_ERROR_CHECK(spi_bus_add_device(IMU_SPI, &devcfg_MPU6000_2, &MPU6000_2));

 spi_bus_config_t GP_SPI_buscfg = {
    .mosi_io_num=GP_SPI_MOSI,
    .miso_io_num=GP_SPI_MISO,
    .sclk_io_num=GP_SPI_SCLK,
    .quadwp_io_num=-1,
    .quadhd_io_num=-1
 };
 ESP_ERROR_CHECK(spi_bus_initialize(GP_SPI, &GP_SPI_buscfg, SPI_DMA_CH_AUTO));

/*
 spi_device_interface_config_t devcfg_SD_card = {
    .command_bits=8,
    .address_bits=0,
    .dummy_bits=0,
    .clock_speed_hz=1000000,
    .duty_cycle_pos=128,                                    //Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty). Setting this to 0 (=not setting it) is equivalent to setting this to 128.
    .mode=3,                                                //SPI mode, representing a pair of (CPOL, CPHA) configuration
    .spics_io_num=GPIO_CS_SD_card,
    .cs_ena_posttrans=3,                                    //Keep the CS low 3 cycles after transaction
    .queue_size=3,
    .input_delay_ns = 0
 };

 ESP_ERROR_CHECK(spi_bus_add_device(IMU_SPI, &devcfg_SD_card, &SD_card));
 //assert(ret==ESP_OK);
*/

 spi_device_interface_config_t devcfg_W25N01 = {
    //.command_bits=8,
    //.address_bits=8,
    //.dummy_bits=8,
    .clock_speed_hz=75000000,
    .duty_cycle_pos=128,                                    //Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty). Setting this to 0 (=not setting it) is equivalent to setting this to 128.
    .mode=3,                                                //SPI mode, representing a pair of (CPOL, CPHA) configuration
    .spics_io_num=GPIO_CS_W25N01,
    .cs_ena_posttrans=3,                                    //Keep the CS low 3 cycles after transaction
    .queue_size=3,
    .input_delay_ns = 0
 };
 ESP_ERROR_CHECK(spi_bus_add_device(GP_SPI, &devcfg_W25N01, &W25N01));
/* 
  spi_device_interface_config_t devcfg_PMW3901 = {//2MHz max
    //.command_bits=NULL,
    .address_bits= 8,
    //.dummy_bits=NULL,
    .clock_speed_hz=1500000,
    .duty_cycle_pos=128,                                    //Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty). Setting this to 0 (=not setting it) is equivalent to setting this to 128.
    .mode=3,                                                //SPI mode, representing a pair of (CPOL, CPHA) configuration
    .spics_io_num=-1,                          //GPIO_CS_PMW3901
    //.cs_ena_pretrans=5,                                     //no
    //.cs_ena_posttrans=5,                                    //Keep the CS low 3 cycles after transaction 30
    .queue_size=3,
    .input_delay_ns = 50,
    //.flags = SPI_DEVICE_HALFDUPLEX,
 };
 ESP_ERROR_CHECK(spi_bus_add_device(GP_SPI, &devcfg_PMW3901, &PMW3901));

 spi_device_interface_config_t devcfg_HMC5983 = {           
    .command_bits=0,
    .address_bits=8,
    .dummy_bits=0,
    .clock_speed_hz=8000000,
    .duty_cycle_pos=128,                                    //Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty). Setting this to 0 (=not setting it) is equivalent to setting this to 128.
    .mode=3,                                                //SPI mode, representing a pair of (CPOL, CPHA) configuration
    .spics_io_num=GPIO_CS_HMC5983,
    .cs_ena_posttrans=3,                                    //Keep the CS low 3 cycles after transaction
    .queue_size=3,
    .input_delay_ns = 0
 };
 ESP_ERROR_CHECK(spi_bus_add_device(GP_SPI, &devcfg_HMC5983, &HMC5983));
*/
}

void SPI_change_MPUs_speed() 
{
  ESP_ERROR_CHECK(spi_bus_remove_device(MPU6000_1));
  ESP_ERROR_CHECK(spi_bus_remove_device(MPU6000_2));

  spi_device_interface_config_t devcfg_MPU6000_1 = {         //1MHz for config, 20MHz for sensor registers
    .clock_speed_hz=20000000,
    .duty_cycle_pos=128,                                    //Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty). Setting this to 0 (=not setting it) is equivalent to setting this to 128.
    .mode=3,                                                //SPI mode, representing a pair of (CPOL, CPHA) configuration
    .spics_io_num=GPIO_CS_MPU6000_1,
    .cs_ena_posttrans=3,                                    //Keep the CS low 3 cycles after transaction
    .queue_size=3,
    .input_delay_ns = 0
 };
 ESP_ERROR_CHECK(spi_bus_add_device(IMU_SPI, &devcfg_MPU6000_1, &MPU6000_1));
 
  spi_device_interface_config_t devcfg_MPU6000_2 = {
    .clock_speed_hz=20000000,
    .duty_cycle_pos=128,                                    //Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty). Setting this to 0 (=not setting it) is equivalent to setting this to 128.
    .mode=3,                                                //SPI mode, representing a pair of (CPOL, CPHA) configuration
    .spics_io_num=GPIO_CS_MPU6000_2,
    .cs_ena_posttrans=3,                                    //Keep the CS low 3 cycles after transaction
    .queue_size=3,
    .input_delay_ns = 0
 };
 ESP_ERROR_CHECK(spi_bus_add_device(IMU_SPI, &devcfg_MPU6000_2, &MPU6000_2));
};

void SPI_write_byte(spi_device_handle_t spi_handle, 
                        uint8_t number_of_command_bits, uint8_t command, 
                        uint8_t number_of_address_bits, uint16_t address, 
                        uint8_t number_of_dummy_bits, 
                        uint8_t value) 
{
  spi_transaction_t t = {
        .cmd       = command ,           
        .addr      = address ,                                  
        .length    = 8 ,
        .rxlength  = NULL ,                                      //Total data length received, should be not greater than length in full-duplex mode (0 defaults this to the value of length).
        .flags     = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY ,                  //Bitwise OR of SPI_TRANS_* flags.
        .tx_buffer = &value,                    //??
  };
    spi_transaction_ext_t e = {
        .base = t ,
        .command_bits = number_of_command_bits ,
        .address_bits = number_of_address_bits ,
        .dummy_bits = number_of_dummy_bits ,
    }; 
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_handle, &e));
}

uint8_t SPI_read_byte(spi_device_handle_t spi_handle,  
                          uint8_t number_of_command_bits, uint8_t command, 
                          uint8_t number_of_address_bits, uint8_t address, 
                          uint8_t number_of_dummy_bits) 
{
  uint8_t value = 0;
  spi_transaction_t t = {
          .cmd       = command, 
          .addr       = address,           //could be used only if set properly in config_t MPU9250_PRODUCT_ID | READ_FLAG, 
          .length    = 8 ,
          .rxlength  = 8 ,                                      //Total data length received, should be not greater than length in full-duplex mode (0 defaults this to the value of length).
          .flags     = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY ,                  //Bitwise OR of SPI_TRANS_* flags.
          .rx_buffer = &value ,
      };

  spi_transaction_ext_t e = {
      .base = t ,
      .command_bits = number_of_command_bits ,
      .address_bits = number_of_address_bits ,
      .dummy_bits = number_of_dummy_bits ,
    }; 
  ESP_ERROR_CHECK(spi_device_polling_transmit(spi_handle, &e));

  return value;
}

void SPI_write_bytes(spi_device_handle_t spi_handle, 
                          uint8_t number_of_command_bits, uint16_t command, 
                          uint8_t number_of_address_bits, uint16_t start_address, 
                          uint8_t number_of_dummy_bits, 
                          uint8_t* where_to_take_from, uint16_t number_of_bytes)
{      
    spi_transaction_t t = {
      .cmd       = command,           //could be used only if set properly in config_t MPU9250_PRODUCT_ID | READ_FLAG, 
      .addr      = start_address,                                    //could be used only if set properly in config_t
      .length    = 8 * number_of_bytes ,
      .rxlength  = NULL ,                                      //Total data length received, should be not greater than length in full-duplex mode (0 defaults this to the value of length).
      .flags     = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY ,
      .tx_buffer = where_to_take_from,                    //??
      //.rx_buffer = spi_RX_buffer,
      //.tx_data[] = {0,1,2,3},
      //.rx_data[] = {0,0,0,0};
  };
  spi_transaction_ext_t e = {
      .base = t ,
      .command_bits = number_of_command_bits,
      .address_bits = number_of_address_bits,
      .dummy_bits = number_of_dummy_bits
    }; 
  ESP_ERROR_CHECK(spi_device_polling_transmit(spi_handle, &e));


}

void SPI_read_bytes(spi_device_handle_t spi_handle, 
                        uint8_t number_of_command_bits, uint16_t command, 
                        uint8_t number_of_address_bits, uint16_t start_address, 
                        uint8_t number_of_dummy_bits, 
                        uint8_t* where_to_put_to, uint16_t number_of_bytes) 
{    
  spi_transaction_t t = {
        .cmd       = command,           
        .addr      = start_address ,                                    
        .length    = 8 * number_of_bytes ,
        .rxlength  = 8 * number_of_bytes ,                                      //Total data length received, should be not greater than length in full-duplex mode (0 defaults this to the value of length).
        .flags     = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY ,
        .rx_buffer = where_to_put_to,

    };
    spi_transaction_ext_t e = {
        .base = t ,
        .command_bits = number_of_command_bits ,  
        .address_bits = number_of_address_bits ,  
        .dummy_bits = number_of_dummy_bits 
      }; 
    //ESP_ERROR_CHECK(spi_device_acquire_bus(spi_handle, portMAX_DELAY));  
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_handle, &e));
    //spi_device_release_bus(spi_handle); 
}



