#ifndef Q_OPERATIONS
#define Q_OPERATIONS

void quaternion_conjugation(float* q, float* result);
void quaternion_sum(float* q, float* p, float* result);
void quaternions_multiplication(float* q1, float* q2, float* result);
void vector_back_rotation(float* vector, float* q, float* result);
void quaternion_difference(float* q_current, float* q_desired, float* q_diff);
void quat_from_angle_and_axis(float angle, float* axis, float* q);
void angle_and_axis_from_quat(float* q, float* angle, float* axis);
void calculate_desired_quat_from_angles_and_yaw_rate(float pitch, float roll, float yaw_rate_degrees, float time, float* q_desired);
void calculate_desired_quat_from_3_angles(float pitch, float roll, float yaw, float* q_desired);
void Convert_Q_to_degrees(float q0, float q1, float q2, float q3, float* pitch, float* roll, float* yaw);

#endif