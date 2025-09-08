#ifndef _PX4FLOW_H
#define _PX4FLOW_H

#include <inttypes.h> 
#include "esp_err.h"
#include "px4flow.h"

// 7 Bit I2C Address of the Flow Module: Default 0x42 (user selectable bits 0,1,2) 
#define PX4FLOW_I2C_ADDRESS 0x42

// timeout in milliseconds for PX4Flow read
#define PX4FLOW_TIMEOUT 10

//адрес для чтения обычного регистра
#define PX4FLOW_READ_FRAME_ADDRESS 0x00

//адрес для чтения "интегрального" регистра
#define PX4FLOW_READ_INTEGRAL_FRAME_ADDRESS 0x16

//фокусное расстояние в пикселях 
//focal_length_px = (global_data.param[PARAM_FOCAL_LENGTH_MM]) / (4.0f * 6.0f) * 1000.0f; //original focal lenght: 12mm pixelsize: 6um, binning 4 enabled

//разница между кадрами в пикселях 64 пикселя макс

//коррекция по гироскопу 
/* calc pixel of gyro 
	float y_rate_pixel = y_rate * (get_time_between_images() / 1000000.0f) * focal_length_px;
	float comp_x = histflowx + y_rate_pixel;
*/

typedef struct px4flow_i2c_frame
{
    uint16_t frame_count;// counts created I2C frames                                   постоянно инкрементирующееся значение
    int16_t pixel_flow_x_sum;// accumulated x flow in pixels*10 since last I2C frame    суммарное перемещение в пикселях
    int16_t pixel_flow_y_sum;// accumulated y flow in pixels*10 since last I2C frame    суммарное перемещение в пикселях
    int16_t flow_comp_m_x;// x velocity*1000 in meters / timestep                       скорость в м/с
    int16_t flow_comp_m_y;// y velocity*1000 in meters / timestep                       скорость в м/с
    int16_t quality;// Optical flow quality / confidence 0: bad, 255: maximum quality
    int16_t gyro_x_rate; //gyro x rate
    int16_t gyro_y_rate; //gyro y rate
    int16_t gyro_z_rate; //gyro z rate
    uint8_t gyro_range; // gyro range
    uint8_t sonar_timestamp;// timestep in milliseconds between I2C frames
    int16_t ground_distance;// Ground distance in meters*1000. Positive value: distance known. Negative value: Unknown distance
} px4flow_i2c_frame_t;

typedef struct px4flow_i2c_integral_frame
{
    uint16_t frame_count_since_last_readout;//number of flow measurements since last I2C readout [#frames]
    int16_t pixel_flow_x_integral;//accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]      accumulated_flow_x += pixel_flow_y  / focal_length_px * 1.0f;
    int16_t pixel_flow_y_integral;//accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
    int16_t gyro_x_rate_integral;//accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000] 
    int16_t gyro_y_rate_integral;//accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000] 
    int16_t gyro_z_rate_integral;//accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000] 
    uint32_t integration_timespan;//accumulation timespan in microseconds since last I2C readout [microseconds]
    uint32_t sonar_timestamp;// time since last sonar update [microseconds]
    int16_t ground_distance;// Ground distance in meters*1000 [meters*1000]
    int16_t gyro_temperature;// Temperature * 100 in centi-degrees Celsius [degcelsius*100]
    uint8_t quality;// averaged quality of accumulated flow values [0:bad quality;255: max quality]
} px4flow_i2c_integral_frame_t;

esp_err_t px4flow_communication_check();
void px4flow_read_integral_frame(px4flow_i2c_integral_frame_t* this);
void px4flow_read_frame(px4flow_i2c_frame_t* this);
void px4flow_read_all(void);


#endif
