#ifndef Q_OPERATIONS
#define Q_OPERATIONS

void quaternion_conjugation(float* q, float* result);
void quaternions_multiplication(float* q1, float* q2, float* result);
void vector_back_rotation(float* vector, float* q, float* result);

#endif