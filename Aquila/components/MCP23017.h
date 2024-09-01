#ifndef MCP23017_h
#define MCP23017_h

#define MCP23017_ADDR             0x20 //!< Default I2C Address 0x40   7 bits or 8 bits addressing!!!

#define MCP23017_IODIRA           0x00   //!< I/O direction register
#define MCP23017_IODIRB           0x01   //!< I/O direction register
#define MCP23017_IPOLA            0x02    //!< Input polarity register
#define MCP23017_IPOLB            0x03    //!< Input polarity register
#define MCP23017_GPINTENA         0x04 //!< Interrupt-on-change control register
#define MCP23017_GPINTENB         0x05 //!< Interrupt-on-change control register
#define MCP23017_DEFVALA          0x06 //!< Default compare register for interrupt-on-change
#define MCP23017_DEFVALB          0x07 //!< Default compare register for interrupt-on-change
#define MCP23017_INTCONA          0x08 //!< Interrupt control register
#define MCP23017_INTCONB          0x09 //!< Interrupt control register
#define MCP23017_IOCON            0x0A  //!< Configuration register

#define MCP23017_GPPUA            0x0C   //!< Pull-up resistor configuration register
#define MCP23017_GPPUB            0x0D   //!< Pull-up resistor configuration register
#define MCP23017_INTFA            0x0E   //!< Interrupt flag register
#define MCP23017_INTFB            0x0F   //!< Interrupt flag register
#define MCP23017_INTCAPA          0x10 //!< Interrupt capture register
#define MCP23017_INTCAPB          0x11 //!< Interrupt capture register
#define MCP23017_GPIOA            0x12   //!< Port register
#define MCP23017_GPIOB            0x13   //!< Port register
#define MCP23017_OLATA            0x14   //!< Output latch register
#define MCP23017_OLATB            0x15   //!< Output latch register


esp_err_t MCP23017_communication_check();
esp_err_t MCP23017_init();
//uint8_t MCP23017_read_register(uint8_t reg_to_read_from);
esp_err_t MCP23017_set_output(uint8_t * previous_output_state, uint8_t out_pin);
esp_err_t MCP23017_clear_output(uint8_t * previous_output_state, uint8_t out_pin);
uint8_t MCP23017_get_inputs_state();


#endif