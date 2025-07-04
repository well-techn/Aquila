//

#ifndef winbondW25N_H
#define winbondW25N_H
 
#include "esp_err.h"

#define W25M_DIE_SELECT           0xC2

#define W25N_RESET                0xFF
#define W25N_READ_JEDEC_ID        0x9F
#define W25N_READ_STATUS_REG      0x05
#define W25N_WRITE_STATUS_REG     0x01
#define W25N_WRITE_ENABLE         0x06
#define W25N_WRITE_DISABLE        0x04
#define W25N_BB_MANAGE            0xA1
#define W25N_READ_BBM             0xA5
#define W25N_LAST_ECC_FAIL        0xA9
#define W25N_BLOCK_ERASE          0xD8
#define W25N_PROG_DATA_LOAD       0x02
#define W25N_RAND_PROG_DATA_LOAD  0x84
#define W25N_PROG_EXECUTE         0x10
#define W25N_PAGE_DATA_READ       0x13
#define W25N_READ                 0x03
#define W25N_FAST_READ            0x0B

#define W25N_PROT_REG_SR1         0xA0
#define W25N_CONFIG_REG_SR2       0xB0
#define W25N_STAT_REG_SR3         0xC0
 
#define WINBOND_MAN_ID            0xEF
#define W25N01GV_DEV_ID           0xAA21
#define W25N01GV_DEV_ID_HI        0xAA
#define W25N01GV_DEV_ID_LO        0x21

#define W25M02GV_DEV_ID           0xAB21

#define W25N01GV_MAX_PAGE         65535
#define W25N_MAX_COLUMN           2112
#define W25M02GV_MAX_PAGE         131071
#define W25M02GV_MAX_DIES         2


void W25N_reset();
esp_err_t W25N_read_JEDEC_ID();
void W25N_write_enable();
void W25N_write_disable();

void W25N_program_data_load (uint16_t column_address, uint8_t* data_to_write, uint16_t number_of_bytes);
void W25N_random_program_data_load (uint16_t column_address, uint8_t* data_to_write, uint16_t number_of_bytes);
void W25N_program_execute(uint16_t page_address);

void W25N_page_data_read (uint16_t page_address);
void W25N_read (uint16_t column_address, uint8_t* where_to_put_to, uint16_t number_of_bytes);
void W25N_read_all_status_registers();
void W25N_write_status_register(uint8_t status_register_address, uint8_t reg_value);
esp_err_t winbond_read_write_test(void);
void W25N_block_erase(uint16_t page_address);
void W25N_erase_all(void);
void W25N_erase_all_new(void);
uint8_t W25N_read_STATUS_register(void);




#endif