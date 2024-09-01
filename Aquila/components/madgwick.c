#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <ctype.h>
#include "math.h"
#include "ve_alldef.h"

volatile float q0 = 0.0; 
volatile float q1 = 1.0; 
volatile float q2 = 0.0; 
volatile float q3 = 0.0; 
float beta = MADGWICK_BETA;

static float invSqrt(float x) {
  union {
    float    f;
    uint32_t i;
  } conv;

  float       x2;
  const float threehalfs = 1.5F;

  x2     = x * 0.5F;
  conv.f = x;
  conv.i = 0x5f3759df - (conv.i >> 1);
  conv.f = conv.f * (threehalfs - (x2 * conv.f * conv.f));
  return conv.f;
}

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, uint64_t timer_value) {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
        
        
        
        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5 * ((-1.0)* q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);
       
        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

                // Normalise accelerometer measurement
                recipNorm = invSqrt(ax * ax + ay * ay + az * az);
                                       
                ax *= recipNorm;
                ay *= recipNorm;
                az *= recipNorm;

               // Auxiliary variables to avoid repeated arithmetic
                _2q0 = 2.0 * q0;
                _2q1 = 2.0 * q1;
                _2q2 = 2.0 * q2;
                _2q3 = 2.0 * q3;
                _4q0 = 4.0 * q0;
                _4q1 = 4.0 * q1;
                _4q2 = 4.0 * q2;
                _8q1 = 8.0 * q1;
                _8q2 = 8.0 * q2;
                q0q0 = q0 * q0;
                q1q1 = q1 * q1;
                q2q2 = q2 * q2;
                q3q3 = q3 * q3;

                // Gradient decent algorithm corrective step
                s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
                s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
                s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
                s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
   

                // Apply feedback step
                qDot1 -= beta * s0;
                qDot2 -= beta * s1;
                qDot3 -= beta * s2;
                qDot4 -= beta * s3;

    }
        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * (float)timer_value * 0.1 / 1000000.0;           //     (1.0 / (float)((float)SMPL_FREQ * (float)iteration_number));
        q1 += qDot2 * (float)timer_value * 0.1 / 1000000.0;
        q2 += qDot3 * (float)timer_value * 0.1 / 1000000.0;
        q3 += qDot4 * (float)timer_value * 0.1 / 1000000.0;
        
        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

        
        
}

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, uint64_t timer_value) {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        volatile float hx, hy;
        volatile float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3,_8bx, _8bz;

        // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
//        if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
//                MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
//                return;
//        }

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

                // Normalise accelerometer measurement
                //recipNorm = 1.0 / sqrt(ax * ax + ay * ay + az * az);
                recipNorm = invSqrt(ax * ax + ay * ay + az * az);

                ax *= recipNorm;
                ay *= recipNorm;
                az *= recipNorm;

                // Normalise magnetometer measurement
                //recipNorm = 1.0 / sqrt(mx * mx + my * my + mz * mz);
                recipNorm = invSqrt(mx * mx + my * my + mz * mz);
                
                
                mx *= recipNorm;
                my *= recipNorm;
                mz *= recipNorm;

               // Auxiliary variables to avoid repeated arithmetic
                _2q0mx = 2.0 * q0 * mx;
                _2q0my = 2.0 * q0 * my;
                _2q0mz = 2.0 * q0 * mz;
                _2q1mx = 2.0 * q1 * mx;
                _2q0 = 2.0 * q0;
                _2q1 = 2.0 * q1;
                _2q2 = 2.0 * q2;
                _2q3 = 2.0 * q3;
                _2q0q2 = 2.0 * q0 * q2;
                _2q2q3 = 2.0 * q2 * q3;
                q0q0 = q0 * q0;
                q0q1 = q0 * q1;
                q0q2 = q0 * q2;
                q0q3 = q0 * q3;
                q1q1 = q1 * q1;
                q1q2 = q1 * q2;
                q1q3 = q1 * q3;
                q2q2 = q2 * q2;
                q2q3 = q2 * q3;
                q3q3 = q3 * q3;

                // Reference direction of Earth's magnetic field
                hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
                hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
                _2bx = sqrt(hx * hx + hy * hy);
                _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
                _4bx = 2.0 * _2bx;
                _4bz = 2.0 * _2bz;
                //*************************************888
                _8bx = 2.0 * _4bx;
                _8bz = 2.0 * _4bz;

                // Gradient decent algorithm corrective step     https://diydrones.com/forum/topics/madgwick-imu-ahrs-and-fast-inverse-square-root?id=705844%3ATopic%3A1018435&page=4#comments
                s0 = -_2q2 * (2.0f*(q1q3 - q0q2) - ax) + _2q1*(2.0f*(q0q1 + q2q3) - ay)   +  -_4bz*q2*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)   +   (-_4bx*q3+_4bz*q1)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)    +   _4bx*q2*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
                s1 = _2q3 * (2.0f*(q1q3 - q0q2) - ax) + _2q0*(2.0f*(q0q1 + q2q3) - ay) +   -4.0f*q1*(2.0f*(0.5 - q1q1 - q2q2) - az)    +   _4bz*q3*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)   + (_4bx*q2+_4bz*q0)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)   +   (_4bx*q3-_8bz*q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
                s2 = -_2q0 * (2.0f*(q1q3 - q0q2) - ax) + _2q3*(2.0f*(q0q1 + q2q3) - ay)   +   (-4.0f*q2)*(2.0f*(0.5 - q1q1 - q2q2) - az) +   (-_8bx*q2-_4bz*q0)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)+(_4bx*q1+_4bz*q3)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)+(_4bx*q0-_8bz*q2)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
                s3 = _2q1*(2.0f*(q1q3 - q0q2) - ax) + _2q2*(2.0f*(q0q1 + q2q3) - ay)+(-_8bx*q3+_4bz*q1)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)+(-_4bx*q0+_4bz*q2)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)+(_4bx*q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);


                recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude

                s0 *= recipNorm;
                s1 *= recipNorm;
                s2 *= recipNorm;
                s3 *= recipNorm;
         // Apply feedback step
                qDot1 -= beta * s0;
                qDot2 -= beta * s1;
                qDot3 -= beta * s2;
                qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * (float)timer_value * 0.1 / 1000000.0;           //     (1.0 / (float)((float)SMPL_FREQ * (float)iteration_number));
        q1 += qDot2 * (float)timer_value * 0.1 / 1000000.0;
        q2 += qDot3 * (float)timer_value * 0.1 / 1000000.0;
        q3 += qDot4 * (float)timer_value * 0.1 / 1000000.0;
 
        // Normalise quaternion
        //recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

}



