#ifndef MADGWICK_H
#define MADGWICK_H

float invSqrt(float x);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, uint64_t timer_value);
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, uint64_t timer_value);
void MadgwickQuaternionUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, uint64_t timer_value);

#endif