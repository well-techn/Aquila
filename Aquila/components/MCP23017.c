#include "ve_i2c.h"
#include "MCP23017.h"
//#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "ve_alldef.h"
#include "esp_log.h"
#include <rom/ets_sys.h>

extern char *TAG_MCP23017;
extern uint8_t MCP23017_outputs;
extern i2c_master_dev_handle_t MCP23017_dev_handle;
extern i2c_master_bus_handle_t i2c_internal_bus_handle;


esp_err_t MCP23017_communication_check() {
    
    esp_err_t err; 

    err = i2c_master_probe(i2c_internal_bus_handle, MCP23017_ADDR, -1);

    if (err == ESP_OK) ESP_LOGI(TAG_MCP23017,"MCP23017 is online\n");
    else  ESP_LOGE(TAG_MCP23017,"MCP23017 is offline\n");

    return err;
}

esp_err_t MCP23017_init() {

    esp_err_t err = ESP_OK; 
    uint8_t reg_value, i;
    uint8_t MCP23017_configuration_data[13][2] = {{MCP23017_IOCON,    0b00000010},      //general config, interrupt polarity high
                                                  {MCP23017_IODIRA,   0b11110000},      //direction A, low uotputs, high inputs (not in use)          
                                                  {MCP23017_IODIRB,   0xFF},            // direction B, all inputs                
                                                  {MCP23017_IPOLA,    0x00},            //no inversion         
                                                  {MCP23017_IPOLB,    0x00},            //no inversion
                                                  {MCP23017_GPPUA,    0b11110000},      //pulling up unused pins
                                                  {MCP23017_GPPUB,    0b11111111},      //pulling up unused pins
                                                  {MCP23017_GPINTENA, 0x00},            //interrupt on change disable A            
                                                  {MCP23017_GPINTENB, 0b00111111},      //interrupt on change enable B
                                                  {MCP23017_DEFVALA,  0b11110000},      //default values to compare with A
                                                  {MCP23017_DEFVALB,  0b11111111},      //default values to compare with B
                                                  {MCP23017_INTCONA,  0x00},            //Pin value is compared against the previous pin value
                                                  {MCP23017_INTCONB,  0x00}};           //Pin value is compared against the previous pin value
                                                           
    for (i=0;i<13;i++) {
        i2c_write_byte_to_address_NEW(MCP23017_dev_handle, MCP23017_configuration_data[i][0], MCP23017_configuration_data[i][1]);
        ets_delay_us(500);
    }

    for (i=0; i<13; i++) {                              //checking against predefined configuration  
        reg_value = i2c_read_byte_from_address_NEW(MCP23017_dev_handle, MCP23017_configuration_data[i][0]);
        if (reg_value != MCP23017_configuration_data[i][1]) 
        {
            err = ESP_FAIL;
            ESP_LOGE(TAG_MCP23017,"MCP23017 configuration failed at register %02x, returned value is %d",MCP23017_configuration_data[i][0], reg_value);
        }
}

if (err == ESP_OK) ESP_LOGI(TAG_MCP23017,"MCP23017 is configured\n");
    else  ESP_LOGE(TAG_MCP23017,"MCP23017 configuration failed\n");

    return err;
}    
/*
uint8_t MCP23017_read_register(uint8_t reg_to_read_from)
{
    return i2c_read_byte_from_address(I2C_INT_PORT, MCP23017_ADDR, reg_to_read_from);
}
*/
esp_err_t MCP23017_set_output(uint8_t * previous_output_state, uint8_t out_pin)
{
    esp_err_t ret;
    uint8_t MCP23017_new_outputs;
    uint8_t MCP23017_previous_outputs;

    MCP23017_previous_outputs = &previous_output_state;
    MCP23017_new_outputs = MCP23017_previous_outputs | (1 << out_pin);
    ret = i2c_write_byte_to_address_NEW(MCP23017_dev_handle, MCP23017_OLATA, MCP23017_new_outputs);

    return ret;
}

esp_err_t MCP23017_clear_output(uint8_t * previous_output_state, uint8_t out_pin)
{
    esp_err_t ret;
    uint8_t MCP23017_new_outputs;
    uint8_t MCP23017_previous_outputs;

    MCP23017_previous_outputs = &previous_output_state; 
    MCP23017_new_outputs = MCP23017_previous_outputs & (~( 0b00000001 << out_pin));
    ret = i2c_write_byte_to_address_NEW(MCP23017_dev_handle, MCP23017_OLATA, MCP23017_new_outputs);

    return ret;
}

uint8_t MCP23017_get_inputs_state()
{
    return i2c_read_byte_from_address_NEW(MCP23017_dev_handle, MCP23017_INTCAPB);
} 