#include "wt_alldef.h"
#include "math.h"
#include "stdio.h"

void quaternion_conjugation(float* q, float* result)
{
    result[0] = q[0];
    result[1] = -q[1];
    result[2] = -q[2];
    result[3] = -q[3];		
}

void quaternion_normalization(float* q, float* result)
{
    float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    
    result[0] = q[0] / norm;
    result[1] = q[1] / norm;
    result[2] = q[2] / norm;
    result[3] = q[3] / norm;		
}

void quaternion_sum(float* q, float* p, float* result)
{
    result[0] = q[0] + p[0];
    result[1] = q[1] + p[1];
    result[2] = q[2] + p[2];
    result[3] = q[3] + p[3];		
}

void quaternions_multiplication(float* q1, float* q2, float* result)
{
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2]- q1[3]*q2[3];
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3]- q1[3]*q2[2];
    result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0]+ q1[3]*q2[1];
    result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1]+ q1[3]*q2[0];		
}

// Vновый = (q*) * (Vстарый) * (q)
void vector_back_rotation(float* vector, float* q, float* result)
{
 float q_conj[4];
 float q_temp[4];
 quaternion_conjugation(q, q_conj);
 quaternions_multiplication(q_conj,vector,q_temp);
 quaternions_multiplication(q_temp,q,result);
}

void quaternion_difference(float* q_current, float* q_desired, float* q_diff)
{
 float q_conj[4];
 quaternion_conjugation(q_current, q_conj);
 quaternions_multiplication(q_desired,q_conj,q_diff);
}

void quat_from_angle_and_axis(float angle, float* axis, float* q)
{
    q[0] = cos((angle/2) * M_PI / 180.0);
    q[1] = axis[0] * sin((angle/2) * M_PI / 180.0);
    q[2] = axis[1] * sin((angle/2) * M_PI / 180.0);
    q[3] = axis[2] * sin((angle/2) * M_PI / 180.0);
}

void angle_and_axis_from_quat(float* q, float* angle, float* axis)
{
    *angle = 2 * acos(q[0]) * 180 / M_PI;
    axis[1] = q[1] / sin((*angle/2) * M_PI / 180.0);
    axis[2] = q[2] / sin((*angle/2) * M_PI / 180.0);
    axis[3] = q[3] / sin((*angle/2) * M_PI / 180.0);
}

void calculate_desired_quat_from_angles_and_yaw_rate(float pitch, float roll, float yaw_rate_degrees, float time, float* q_desired)
{
    float axis_pitch[3] = {0,1,0};
    float axis_roll[3] = {1,0,0};

    float q_pitch[4];
    float q_roll[4];
     
    quat_from_angle_and_axis(pitch, axis_pitch, q_pitch);
    //for (int i = 0; i<4;i++) printf("%0.3f ", q_pitch[i]);
    //printf("\n");
    
    quat_from_angle_and_axis(roll, axis_roll, q_roll);
    //for (int i = 0; i<4;i++) printf("%0.3f ", q_roll[i]);
    //printf("\n");
    
    float angular_rate_rads = yaw_rate_degrees * 3.14/180.0;
    //printf("%0.3f\n", angular_rate_rads);
    float half_dt = time / 2.0;
    float q_omega[4] = {0,0,0,0};
    q_omega[3] = angular_rate_rads * half_dt;
    //printf("Angular rate quat ");
    //for (int i = 0; i<4;i++) printf("%f ", q_omega[i]);
    //printf("\n");
    
    
    quaternions_multiplication(q_pitch, q_roll, q_desired);
    //printf("First multiplication ");
    //for (int i = 0; i<4;i++) printf("%f ", q_desired[i]);
    //printf("\n");
    //quaternions_multiplication(q_desired, q_yaw, q_desired);
    quaternion_sum(q_desired, q_omega, q_desired);
    //printf("Sum ");
    //for (int i = 0; i<4;i++) printf("%f ", q_desired[i]);
    //printf("\n");
    
    quaternion_normalization(q_desired, q_desired);
    //printf("Normalized ");
    //for (int i = 0; i<4;i++) printf("%f ", q_desired[i]);
    //printf("\n");
}

void calculate_desired_quat_from_3_angles(float pitch, float roll, float yaw, float* q_desired)
{
    float axis_pitch[3] = {0,1,0};
    float axis_roll[3] = {1,0,0};
    float axis_yaw[3] = {0,0,1};

    float q_pitch[4];
    float q_roll[4];
    float q_yaw[4];
    float q_temp[4];
    
    
    quat_from_angle_and_axis(pitch, axis_pitch, q_pitch);
    //printf("First ");
    //for (int i = 0; i<4;i++) printf("%0.3f ", q_pitch[i]);
    //printf("\n");
    
    quat_from_angle_and_axis(roll, axis_roll, q_roll);
    //printf("Second ");
    //for (int i = 0; i<4;i++) printf("%0.3f ", q_roll[i]);
    //printf("\n");
    
    quat_from_angle_and_axis(yaw, axis_yaw, q_yaw);
    //printf("Third ");
    //for (int i = 0; i<4;i++) printf("%0.3f ", q_yaw[i]);
    //printf("\n");
    
    
    quaternions_multiplication(q_pitch, q_roll, q_temp);
    //printf("First multiplication: ");
    //for (int i = 0; i<4;i++) printf("%f ", q_temp[i]);
    //printf("\n");
    quaternions_multiplication(q_temp, q_yaw, q_desired);
    //printf("Second multiplication: ");
    //for (int i = 0; i<4;i++) printf("%f ", q_desired[i]);
    //printf("\n");
    
    quaternion_normalization(q_desired, q_desired);
    //printf("Normalized ");
    //for (int i = 0; i<4;i++) printf("%f ", q_desired[i]);
    //printf("\n");
}


void Convert_Q_to_degrees(float q0, float q1, float q2, float q3, float* pitch, float* roll, float* yaw)
{
    float a12,a22,a31,a32,a33 = 0;

    a12 =   2.0f * (q1 * q2 + q0 * q3);
    a22 =   q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    a31 =   2.0f * (q0 * q1 + q2 * q3);
    a32 =   2.0f * (q1 * q3 - q0 * q2);
    a33 =   q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    *pitch = -asin(a32) * 57.29577951;  // 180/пи

    *roll = atan2(a31, a33) * 57.29577951;
    //if (*roll > 90) *roll -= 360.0;     

    *yaw = -atan2(a12, a22) * 57.29577951;
    *yaw -= 9.4; // компенсация наклонения при использовании магнетометра. Если магнетометр не используем - без разницы, вреда не наносит
}






/*
float axis_pitch[3] = {0,1,0};
float axis_roll[3] = {1,0,0};
float axis_yaw[3] = {0,0,1};

float pitch = 30;
float roll = 10;
float yaw = -5;

float q_pitch[4];
float q_roll[4];
float q_yaw[4];

float q_desired[4];
float q_current[4] = {0.9952,0.08707,0.003801,-0.04345};
float q_difference[4];

quat_from_angle_and_axis(pitch, axis_pitch, q_pitch);
for (int i = 0; i<4;i++) printf("%0.3f ", q_pitch[i]);
printf("\n");

quat_from_angle_and_axis(roll, axis_roll, q_roll);
for (int i = 0; i<4;i++) printf("%0.3f ", q_roll[i]);
printf("\n");

quat_from_angle_and_axis(yaw, axis_yaw, q_yaw);
for (int i = 0; i<4;i++) printf("%0.3f ", q_yaw[i]);
printf("\n");

quaternions_multiplication(q_pitch, q_roll, q_desired);
quaternions_multiplication(q_desired, q_yaw, q_desired);
for (int i = 0; i<4;i++) printf("%f ", q_desired[i]);
printf("\n");

quaternion_difference(q_current, q_desired, q_difference);
for (int i = 0; i<4;i++) printf("%f ", q_difference[i]);
printf("\n");
*/