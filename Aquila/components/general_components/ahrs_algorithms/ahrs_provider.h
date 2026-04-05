#ifndef AHRS_PROVIDER_H
#define AHRS_PROVIDER_H

void ahrs_initialize(void);
void ahrs_update(float* accel_data, float* gyro_data, float* magn_data, float delta_time, float* q_result);

#endif