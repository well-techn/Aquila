void quaternion_conjugation(float* q, float* result)
{
    result[0] = q[0];
    result[1] = -q[1];
    result[2] = -q[2];
    result[3] = -q[3];		
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