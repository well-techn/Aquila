#ifndef ADVANCED_MATH_H
#define ADVANCED_MATH_H

#include <esp_err.h>

//функция для поэлементного сложения двух 3D векторов
inline void add_2_3D_vectors(const float* a, const float* b, float* sum) {
    sum[0] = a[0] + b[0];
    sum[1] = a[1] + b[1];
    sum[2] = a[2] + b[2];
}

//функция для поэлементного вычитания двух 3D векторов
inline void sub_2_3D_vectors(const float* a, const float* b, float* sub) {
    sub[0] = a[0] - b[0];
    sub[1] = a[1] - b[1];
    sub[2] = a[2] - b[2];
}

//функция для поэлементного умножения двух 3D векторов
inline void mul_elementvise_2_3D_vectors(const float* a, const float* b, float* result) {
    result[0] = a[0] * b[0];
    result[1] = a[1] * b[1];
    result[2] = a[2] * b[2];    
}

//умножение на скаляр 3D вектора 
inline void mul_by_scalar_3D_vector(const float* a, const float scalar, float* result) {
    result[0] = a[0] * scalar;
    result[1] = a[1] * scalar;
    result[2] = a[2] * scalar;    
}

//умножение на скаляр матрицы 3х3 
inline void mul_by_scalar_3x3_matrix(const float* m, const float scalar, float* result) {
     for (uint8_t i = 0; i < 9; i++) result[i] = m[i] * scalar;
}
esp_err_t vector_3D_calibration(const float* raw_data, const float* offset, const float* Ainv_sens, float* result);


void invert3x3(const float* src, float* dst);
void Interchange_Rows(double *A, int row1, int row2, int ncols);
void Interchange_Columns(double *A, int col1, int col2, int nrows, int ncols);
void Copy_Vector(double *d, double *s, int n);
void Multiply_Self_Transpose(double *C, double *A, int nrows, int ncols);
void Get_Submatrix(double *S, int mrows, int mcols,
                   double *A, int ncols, int row, int col);
int Choleski_LU_Decomposition(double *A, int n);
int Lower_Triangular_Inverse(double *L, int n);
int Choleski_LU_Inverse(double *LU, int n);
void Multiply_Matrices(double *C, double *A, int nrows, int ncols,
                       double *B, int mcols);
void Identity_Matrix(double *A, int n);
int Hessenberg_Form_Elementary(double *A, double *S, int n);
void Hessenberg_Elementary_Transform(double *H, double *S, int perm[],
                                     int n);
int QR_Hessenberg_Matrix(double *H, double *S, double eigen_real[],
                         double eigen_imag[], int n, int max_iteration_count);
void One_Real_Eigenvalue(double Hrow[], double eigen_real[],
                         double eigen_imag[], int row, double shift);

void Two_Eigenvalues(double *H, double *S, double eigen_real[],
                     double eigen_imag[], int n, int row, double shift);
void Update_Row(double *Hrow, double cos, double sin, int n, int row);
void Update_Column(double *H, double cos, double sin, int n, int col);
void Update_Transformation(double *S, double cos, double sin,
                           int n, int k);
void Double_QR_Iteration(double *H, double *S, int min_row, int max_row,
                         int n, double *shift, int iteration);
void Product_and_Sum_of_Shifts(double *H, int n, int max_row,
                               double *shift, double *trace, double *det, int iteration);
int Two_Consecutive_Small_Subdiagonal(double *H, int min_row,
                                      int max_row, int n, double trace, double det);
void Double_QR_Step(double *H, int min_row, int max_row, int min_col,
                    double trace, double det, double *S, int n);

void BackSubstitution(double *H, double eigen_real[],
                      double eigen_imag[], int n);
void BackSubstitute_Real_Vector(double *H, double eigen_real[],
                                double eigen_imag[], int row, double zero_tolerance, int n);
void BackSubstitute_Complex_Vector(double *H, double eigen_real[],
                                   double eigen_imag[], int row, double zero_tolerance, int n);
void Calculate_Eigenvectors(double *H, double *S, double eigen_real[],
                            double eigen_imag[], int n);
void Complex_Division(double x, double y, double u, double v,
                      double *a, double *b);
void Transpose_Square_Matrix(double *A, int n);
void calculation_mean_and_sigma_of_vector_array(float array[][3], double *mean, double *sigma, uint16_t number_of_elements);
void calculation_B_and_Ainv(float array[][3], double *A_1, double *B, double *mean, double *sigma, uint16_t number_of_elements);
uint8_t calculation_B_and_Ainv_with_exclusion(float array[][3], double *A_1, double *B, double vector_norm, double rej_threshold, uint16_t number_of_elements);

float sin_approx(float x);
float cos_approx(float x);
float atan2_approx(float y, float x);
float acos_approx(float x);
float asin_approx(float x);





#endif