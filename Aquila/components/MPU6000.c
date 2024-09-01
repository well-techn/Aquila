#include "MPU6000.h"
#include "ve_spi.h" 
#include "esp_log.h"
#include <rom/ets_sys.h>
#include "math.h"
#include "ve_alldef.h"



extern char *TAG_MPU6000;;

esp_err_t MPU6000_communication_check(spi_device_handle_t MPU_handle) {

    esp_err_t err = ESP_FAIL;

    if ((SPI_read_byte (MPU_handle, 0, 0, 8, MPU6000_WHO_AM_I | MPU6000_READ_FLAG, 0)) == 0x68) 
    {
        ESP_LOGI(TAG_MPU6000,"MPU6000 is online");
        err = ESP_OK;
    }
    else ESP_LOGE(TAG_MPU6000,"MPU6000 is offline\n");
    
    return err;
}     

esp_err_t MPU6000_self_test(spi_device_handle_t MPU_handle) {

 uint8_t buf[4];
 int8_t factory_trim_values_accel[3];
 float factory_trim_calculated_values_accel[3];
 int8_t factory_trim_values_gyro[3];
 float factory_trim_calculated_values_gyro[3];
 uint8_t i;
 uint8_t sensor_data[14];
 int16_t accel_raw[3];
 int16_t gyro_raw[3];
 int16_t accel_raw_test[3];
 int16_t gyro_raw_test[3];
 float str_accel[3];
 float str_gyro[3];
 float final_value_accel[3];
 float final_value_gyro[3];
 
 esp_err_t err = ESP_OK;

    SPI_read_bytes(MPU_handle, 0, 0, 8, MPU6000_SELF_TEST_X | MPU6000_READ_FLAG, 0, buf, 4); //reading Factory trim values from accel and gyro reristers

    factory_trim_values_accel[0] = ((buf[0] >> 3) & 0b00011100) | ((buf[3] >> 4) & 0b00000011); //assembling AX trim value
    factory_trim_values_accel[1] = ((buf[1] >> 3) & 0b00011100) | ((buf[3] >> 2) & 0b00000011); //assembling AY trim value 
    factory_trim_values_accel[2] = ((buf[2] >> 3) & 0b00011100) | (buf[3] & 0b00000011); //assembling AZ trim value

    factory_trim_values_gyro[0] = buf[0] & 0b00011111; //assembling GX trim value
    factory_trim_values_gyro[1] = buf[1] & 0b00011111; //assembling GY trim value
    factory_trim_values_gyro[2] = buf[2] & 0b00011111; //assembling GZ trim value

    ESP_LOGI(TAG_MPU6000,"Accel factory trim values are %d, %d, %d",factory_trim_values_accel[0],factory_trim_values_accel[1],factory_trim_values_accel[2]);
    ESP_LOGI(TAG_MPU6000,"Gyro factory trim values are %d, %d, %d",factory_trim_values_gyro[0],factory_trim_values_gyro[1],factory_trim_values_gyro[2]);
    
    for (i=0;i<3;i++) {
        if (factory_trim_values_accel[i] == 0) factory_trim_calculated_values_accel[i] = 0;
        else factory_trim_calculated_values_accel[i] = 4096.0 * 0.34 * powf((0.92 / 0.34),((float)(factory_trim_values_accel[i] - 1.0) / 30.0));
        
        if (factory_trim_values_gyro[i] == 0) factory_trim_calculated_values_gyro[i] = 0;
        else factory_trim_calculated_values_gyro[i] = 25.0 * 131.0 * powf(1.046,(float)(factory_trim_values_gyro[i] - 1));
    }

    factory_trim_calculated_values_gyro[1] *= -1.0;     //as Y should be -25, not 25 as above  
    
    ESP_LOGI(TAG_MPU6000,"Calculated accel factory trim values are %0.2f, %0.2f, %0.2f",factory_trim_calculated_values_accel[0],factory_trim_calculated_values_accel[1],factory_trim_calculated_values_accel[2]);
    ESP_LOGI(TAG_MPU6000,"Calculated gyro factory trim values are %0.2f, %0.2f, %0.2f",factory_trim_calculated_values_gyro[0],factory_trim_calculated_values_gyro[1],factory_trim_calculated_values_gyro[2]);
    
    //settings up minimum registers to read data (accel +-8G, gyro +-250dps)
    SPI_write_byte (MPU_handle, 0, 0, 8, MPU6000_PWR_MGMT_1, 0, 0x80);
    ets_delay_us(10000); 
    SPI_write_byte (MPU_handle, 0, 0, 8, MPU6000_SIGNAL_PATH_RESET, 0, 0x07);
    ets_delay_us(10000); 
    SPI_write_byte (MPU_handle, 0, 0, 8, MPU6000_PWR_MGMT_1, 0, 0x00);
    ets_delay_us(10000); 
    SPI_write_byte (MPU_handle, 0, 0, 8, MPU6000_GYRO_CONFIG, 0, 0x00);          //+-250dps
    ets_delay_us(10000);
    SPI_write_byte (MPU_handle, 0, 0, 8, MPU6000_ACCEL_CONFIG, 0, 0b00010000);    //8G
    ets_delay_us(10000);

    //reading values without test enabled
    SPI_read_bytes(MPU_handle, 0, 0, 8, MPU6000_ACCEL_XOUT_H | MPU6000_READ_FLAG, 0, sensor_data, 14);
    accel_raw[0] = (sensor_data[0] << 8) | sensor_data[1];           //X
    accel_raw[1] = (sensor_data[2] << 8) | sensor_data[3];           //Y
    accel_raw[2] = (sensor_data[4] << 8) | sensor_data[5];           //Z
    gyro_raw[0] = (sensor_data[8] << 8) | sensor_data[9];            //X                  
    gyro_raw[1] = (sensor_data[10] << 8) | sensor_data[11];          //Y
    gyro_raw[2] = (sensor_data[12] << 8) | sensor_data[13];          //Z  

    ESP_LOGI(TAG_MPU6000,"Accel values w/o test are %d, %d, %d",accel_raw[0],accel_raw[1],accel_raw[2]);
    ESP_LOGI(TAG_MPU6000,"Gyro values w/o test are %d, %d, %d",gyro_raw[0],gyro_raw[1],gyro_raw[2]);

    //enabling test bits
    SPI_write_byte (MPU_handle, 0, 0, 8, MPU6000_GYRO_CONFIG, 0, 0b11100000);     //+-250dps
    ets_delay_us(10000);
    SPI_write_byte (MPU_handle, 0, 0, 8, MPU6000_ACCEL_CONFIG, 0, 0b11110000);    //8G
    ets_delay_us(500000);
    //reading data with test enabled
    SPI_read_bytes(MPU_handle, 0, 0, 8, MPU6000_ACCEL_XOUT_H | MPU6000_READ_FLAG, 0, sensor_data, 14);
    accel_raw_test[0] = (sensor_data[0] << 8) | sensor_data[1];           //X
    accel_raw_test[1] = (sensor_data[2] << 8) | sensor_data[3];           //Y
    accel_raw_test[2] = (sensor_data[4] << 8) | sensor_data[5];           //Z
    gyro_raw_test[0] = (sensor_data[8] << 8) | sensor_data[9];            //X                  
    gyro_raw_test[1] = (sensor_data[10] << 8) | sensor_data[11];          //Y
    gyro_raw_test[2] = (sensor_data[12] << 8) | sensor_data[13];          //Z  

    ESP_LOGI(TAG_MPU6000,"Accel values with test are %d, %d, %d",accel_raw_test[0],accel_raw_test[1],accel_raw_test[2]);
    ESP_LOGI(TAG_MPU6000,"Gyro values with test are %d, %d, %d",gyro_raw_test[0],gyro_raw_test[1],gyro_raw_test[2]);

    //calculating difference
    for (i=0;i<3;i++) {
        str_accel[i] = accel_raw_test[i] - accel_raw[i]; //fabs as per https://github.com/natanaeljr/esp32-MPU-driver/blob/master/src/MPU.cpp????
        str_gyro[i] = gyro_raw_test[i] - gyro_raw[i];    //fabs????

        final_value_accel[i] = (str_accel[i] - factory_trim_calculated_values_accel[i]) / factory_trim_calculated_values_accel[i];
        final_value_gyro[i] =   (str_gyro[i] - factory_trim_calculated_values_gyro[i]) / factory_trim_calculated_values_gyro[i]; 

        if (fabs(final_value_accel[i]) > 0.14)
            {ESP_LOGE(TAG_MPU6000,"Self test accel[%d] failed, calculated value is %0.2f but should be within [-0.14...0.14] range",i,final_value_accel[i]); err = ESP_FAIL;}
            else ESP_LOGI(TAG_MPU6000,"Self test accel[%d] is ok, calculated value is %0.2f",i,final_value_accel[i]); 

        if ((final_value_gyro[i] > 0.14) || (final_value_gyro[i] < -0.14))
            {ESP_LOGE(TAG_MPU6000,"Self test gyro[%d] failed, calculated value is %0.2f but should be within [-0.14...0.14] range",i,final_value_gyro[i]); err = ESP_FAIL;}
            else ESP_LOGI(TAG_MPU6000,"Self test gyro[%d] is ok, calculated value is %0.2f",i,final_value_gyro[i]);   
    }

    return err;
}     

esp_err_t MPU6000_init(spi_device_handle_t MPU_handle) {

    esp_err_t err = ESP_OK;
    uint8_t i = 0;
    uint8_t reg_value = 0;
    uint8_t MPU6000_configuration_data[9][2] = {{MPU6000_PWR_MGMT_1,         0x80},         //resetting device         
                                                {MPU6000_SIGNAL_PATH_RESET,  0x07},         //resetting device                 
                                                {MPU6000_PWR_MGMT_1,         0x00},         //internal clock         
                                                {MPU6000_CONFIG,             0b00000110},   //5Hz filter, was 20Hz all the time       
                                                {MPU6000_SMPLRT_DIV,         SMPL},         //sampling frequency         
                                                {MPU6000_GYRO_CONFIG,        0x00},         //250 deg per second                
                                                {MPU6000_ACCEL_CONFIG,       0x08},          
                                                {MPU6000_USER_CTRL,          0b00010000},
                                                {MPU6000_INT_ENABLE,         0x01}};         

    for (i=0;i<9;i++) {
        SPI_write_byte (MPU_handle, 0, 0, 8, MPU6000_configuration_data[i][0], 0, MPU6000_configuration_data[i][1]);
        ets_delay_us(10000);  
    }

    for (i=2; i<9; i++) { //checking against predefined configuration
        reg_value = SPI_read_byte(MPU_handle, 0, 0, 8, MPU6000_configuration_data[i][0] | MPU6000_READ_FLAG, 0);
        if ( reg_value != MPU6000_configuration_data[i][1]) {
            err = ESP_FAIL;
            ESP_LOGE(TAG_MPU6000,"MPU6000 configuration failed at register %x, returned value is %d",MPU6000_configuration_data[i][0], reg_value);
        }
    }

    if (err == ESP_OK) ESP_LOGI(TAG_MPU6000,"MPU6000 is configured\n");
    else  ESP_LOGE(TAG_MPU6000,"MPU6000 configuration failed\n");

    return err;
}









