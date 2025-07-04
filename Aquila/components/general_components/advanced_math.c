#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include <string.h>
#include <float.h>


void invert3x3(const float* src, float* dst)
{
    float det = 0;

    /* Compute adjoint: */

    dst[0] = + src[4] * src[8] - src[5] * src[7];
    dst[1] = - src[1] * src[8] + src[2] * src[7];
    dst[2] = + src[1] * src[5] - src[2] * src[4];
    dst[3] = - src[3] * src[8] + src[5] * src[6];
    dst[4] = + src[0] * src[8] - src[2] * src[6];
    dst[5] = - src[0] * src[5] + src[2] * src[3];
    dst[6] = + src[3] * src[7] - src[4] * src[6];
    dst[7] = - src[0] * src[7] + src[1] * src[6];
    dst[8] = + src[0] * src[4] - src[1] * src[3];

    /* Compute determinant: */

    det = src[0] * dst[0] + src[1] * dst[3] + src[2] * dst[6];

    /* Multiply adjoint with reciprocal of determinant: */

    det = 1.0f / det;

    dst[0] *= det;
    dst[1] *= det;
    dst[2] *= det;
    dst[3] *= det;
    dst[4] *= det;
    dst[5] *= det;
    dst[6] *= det;
    dst[7] *= det;
    dst[8] *= det;
}

void Interchange_Rows(double *A, int row1, int row2, int ncols)
{
    int i;
    double *pA1, *pA2;
    double temp;

    pA1 = A + row1 * ncols;
    pA2 = A + row2 * ncols;
    for (i = 0; i < ncols; i++)
    {
        temp = *pA1;
        *pA1++ = *pA2;
        *pA2++ = temp;
    }
}

void Interchange_Columns(double *A, int col1, int col2, int nrows, int ncols)
{
    int i;
    double *pA1, *pA2;
    double temp;

    pA1 = A + col1;
    pA2 = A + col2;
    for (i = 0; i < nrows; pA1 += ncols, pA2 += ncols, i++)
    {
        temp = *pA1;
        *pA1 = *pA2;
        *pA2 = temp;
    }
}

void Copy_Vector(double *d, double *s, int n)
{
    memcpy(d, s, sizeof(double) * n);
}

static void Update_Row(double *Hrow, double cos, double sin, int n, int row)
{
    double x;
    double *Hnextrow = Hrow + n;
    int i;

    for (i = row; i < n; i++)
    {
        x = Hrow[i];
        Hrow[i] = cos * x + sin * Hnextrow[i];
        Hnextrow[i] = cos * Hnextrow[i] - sin * x;
    }
}

static void Update_Column(double *H, double cos, double sin, int n, int col)
{
    double x;
    int i;
    int next_col = col + 1;

    for (i = 0; i <= next_col; i++, H += n)
    {
        x = H[col];
        H[col] = cos * x + sin * H[next_col];
        H[next_col] = cos * H[next_col] - sin * x;
    }
}

static void Update_Transformation(double *S, double cos, double sin, int n, int k)
{
    double x;
    int i;
    int k1 = k + 1;

    for (i = 0; i < n; i++, S += n)
    {
        x = S[k];
        S[k] = cos * x + sin * S[k1];
        S[k1] = cos * S[k1] - sin * x;
    }
}

static void Complex_Division(double x, double y, double u, double v, double *a, double *b)
{
    double q = u * u + v * v;

    *a = (x * u + y * v) / q;
    *b = (y * u - x * v) / q;
}

void Multiply_Self_Transpose(double *C, double *A, int nrows, int ncols)
{
    int i, j, k;
    double *pA;
    double *p_A = A;
    double *pB;
    double *pCdiag = C;
    double *pC = C;
    double *pCt;

    for (i = 0; i < nrows; pCdiag += nrows + 1, p_A = pA, i++)
    {
        pC = pCdiag;
        pCt = pCdiag;
        pB = p_A;
        for (j = i; j < nrows; pC++, pCt += nrows, j++)
        {
            pA = p_A;
            *pC = 0.0;
            for (k = 0; k < ncols; k++)
                *pC += *(pA++) * *(pB++);
            *pCt = *pC;
        }
    }
}

void Transpose_Square_Matrix(double *A, int n)
{
    double *pA, *pAt;
    double temp;
    int i, j;

    for (i = 0; i < n; A += n + 1, i++)
    {
        pA = A + 1;
        pAt = A + n;
        for (j = i + 1; j < n; pA++, pAt += n, j++)
        {
            temp = *pAt;
            *pAt = *pA;
            *pA = temp;
        }
    }
}

void Get_Submatrix(double *S, int mrows, int mcols, double *A, int ncols, int row, int col)
{
    int number_of_bytes = sizeof(double) * mcols;

    for (A += row * ncols + col; mrows > 0; A += ncols, S += mcols, mrows--)
        memcpy(S, A, number_of_bytes);
}

void Calculate_Eigenvectors(double *H, double *S, double eigen_real[], double eigen_imag[], int n)
{
    double *pH;
    double *pS;
    double x, y;
    int i, j, k;

    for (k = n - 1; k >= 0; k--)
    {
        if (eigen_imag[k] < 0.0)
        {
            for (i = 0, pS = S; i < n; pS += n, i++)
            {
                x = 0.0;
                y = 0.0;
                for (j = 0, pH = H; j <= k; pH += n, j++)
                {
                    x += pS[j] * pH[k - 1];
                    y += pS[j] * pH[k];
                }
                pS[k - 1] = x;
                pS[k] = y;
            }
        }
        else if (eigen_imag[k] == 0.0)
        {
            for (i = 0, pS = S; i < n; i++, pS += n)
            {
                x = 0.0;
                for (j = 0, pH = H; j <= k; j++, pH += n)
                    x += pS[j] * pH[k];
                pS[k] = x;
            }
        }
    }
}

int Choleski_LU_Decomposition(double *A, int n)
{
    int i, k, p;
    double *p_Lk0; // pointer to L[k][0]
    double *p_Lkp; // pointer to L[k][p]
    double *p_Lkk; // pointer to diagonal element on row k.
    double *p_Li0; // pointer to L[i][0]
    double reciprocal;

    for (k = 0, p_Lk0 = A; k < n; p_Lk0 += n, k++)
    {

        //            Update pointer to row k diagonal element.

        p_Lkk = p_Lk0 + k;

        //            Calculate the difference of the diagonal element in row k
        //            from the sum of squares of elements row k from column 0 to
        //            column k-1.

        for (p = 0, p_Lkp = p_Lk0; p < k; p_Lkp += 1, p++)
            *p_Lkk -= *p_Lkp * *p_Lkp;

        //            If diagonal element is not positive, return the error code,
        //            the matrix is not positive definite symmetric.

        if (*p_Lkk <= 0.0)
            return -1;

        //            Otherwise take the square root of the diagonal element.

        *p_Lkk = sqrt(*p_Lkk);
        reciprocal = 1.0 / *p_Lkk;

        //            For rows i = k+1 to n-1, column k, calculate the difference
        //            between the i,k th element and the inner product of the first
        //            k-1 columns of row i and row k, then divide the difference by
        //            the diagonal element in row k.
        //            Store the transposed element in the upper triangular matrix.

        p_Li0 = p_Lk0 + n;
        for (i = k + 1; i < n; p_Li0 += n, i++)
        {
            for (p = 0; p < k; p++)
                *(p_Li0 + k) -= *(p_Li0 + p) * *(p_Lk0 + p);
            *(p_Li0 + k) *= reciprocal;
            *(p_Lk0 + i) = *(p_Li0 + k);
        }
    }
    return 0;
}

int Lower_Triangular_Inverse(double *L, int n)
{
    int i, j, k;
    double *p_i, *p_j, *p_k;
    double sum;

    //         Invert the diagonal elements of the lower triangular matrix L.

    for (k = 0, p_k = L; k < n; p_k += (n + 1), k++)
    {
        if (*p_k == 0.0)
            return -1;
        else
            *p_k = 1.0 / *p_k;
    }

    //         Invert the remaining lower triangular matrix L row by row.

    for (i = 1, p_i = L + n; i < n; i++, p_i += n)
    {
        for (j = 0, p_j = L; j < i; p_j += n, j++)
        {
            sum = 0.0;
            for (k = j, p_k = p_j; k < i; k++, p_k += n)
                sum += *(p_i + k) * *(p_k + j);
            *(p_i + j) = -*(p_i + i) * sum;
        }
    }

    return 0;
}

int Choleski_LU_Inverse(double *LU, int n)
{
    int i, j, k;
    double *p_i, *p_j, *p_k;
    double sum;

    if (Lower_Triangular_Inverse(LU, n) < 0)
        return -1;

    //         Premultiply L inverse by the transpose of L inverse.

    for (i = 0, p_i = LU; i < n; i++, p_i += n)
    {
        for (j = 0, p_j = LU; j <= i; j++, p_j += n)
        {
            sum = 0.0;
            for (k = i, p_k = p_i; k < n; k++, p_k += n)
                sum += *(p_k + i) * *(p_k + j);
            *(p_i + j) = sum;
            *(p_j + i) = sum;
        }
    }

    return 0;
}

void Multiply_Matrices(double *C, double *A, int nrows, int ncols, double *B, int mcols)
{
    double *pB;
    double *p_B;
    int i, j, k;

    for (i = 0; i < nrows; A += ncols, i++)
        for (p_B = B, j = 0; j < mcols; C++, p_B++, j++)
        {
            pB = p_B;
            *C = 0.0;
            for (k = 0; k < ncols; pB += mcols, k++)
                *C += *(A + k) * *pB;
        }
}

void Identity_Matrix(double *A, int n)
{
    int i, j;

    for (i = 0; i < n - 1; i++)
    {
        *A++ = 1.0;
        for (j = 0; j < n; j++)
            *A++ = 0.0;
    }
    *A = 1.0;
}

void Hessenberg_Elementary_Transform(double *H, double *S, int perm[], int n)
{
    int i, j;
    double *pS, *pH;
    // double x;

    Identity_Matrix(S, n);
    for (i = n - 2; i >= 1; i--)
    {
        pH = H + n * (i + 1);
        pS = S + n * (i + 1);
        for (j = i + 1; j < n; pH += n, pS += n, j++)
        {
            *(pS + i) = *(pH + i - 1);
            *(pH + i - 1) = 0.0;
        }
        if (perm[i] != i)
        {
            pS = S + n * i;
            pH = S + n * perm[i];
            for (j = i; j < n; j++)
            {
                *(pS + j) = *(pH + j);
                *(pH + j) = 0.0;
            }
            *(pH + i) = 1.0;
        }
    }
}

int Hessenberg_Form_Elementary(double *A, double *S, int n)
{
    // int i, j, k, col, row;
    int i, j, col, row;
    int *perm;
    double *p_row, *pS_row;
    double max;
    double s;
    double *pA, *pB, *pC, *pS;

    // n x n matrices for which n <= 2 are already in Hessenberg form

    if (n <= 1)
    {
        *S = 1.0;
        return 0;
    }
    if (n == 2)
    {
        *S++ = 1.0;
        *S++ = 0.0;
        *S++ = 1.0;
        *S = 0.0;
        return 0;
    }

    // Allocate working memory

    perm = (int *)malloc(n * sizeof(int));
    if (perm == NULL)
        return -1; // not enough memory

    // For each column use Elementary transformations
    //   to zero the entries below the subdiagonal.

    p_row = A + n;
    pS_row = S + n;
    for (col = 0; col < (n - 2); p_row += n, pS_row += n, col++)
    {

        // Find the row in column "col" with maximum magnitude where
        // row >= col + 1.

        row = col + 1;
        perm[row] = row;
        for (pA = p_row + col, max = 0.0, i = row; i < n; pA += n, i++)
            if (fabs(*pA) > max)
            {
                perm[row] = i;
                max = fabs(*pA);
            }

        // If perm[row] != row, then interchange row "row" and row
        // perm[row] and interchange column "row" and column perm[row].

        if (perm[row] != row)
        {
            Interchange_Rows(A, row, perm[row], n);
            Interchange_Columns(A, row, perm[row], n, n);
        }

        // Zero out the components lying below the subdiagonal.

        pA = p_row + n;
        pS = pS_row + n;
        for (i = col + 2; i < n; pA += n, pS += n, i++)
        {
            s = *(pA + col) / *(p_row + col);
            for (j = 0; j < n; j++)
                *(pA + j) -= *(p_row + j) * s;
            *(pS + col) = s;
            for (j = 0, pB = A + col + 1, pC = A + i; j < n; pB += n, pC += n, j++)
                *pB += s * *pC;
        }
    }
    pA = A + n + n;
    pS = S + n + n;
    for (i = 2; i < n; pA += n, pS += n, i++)
        Copy_Vector(pA, pS, i - 1);

    Hessenberg_Elementary_Transform(A, S, perm, n);

    free(perm);
    return 0;
}

static void One_Real_Eigenvalue(double Hrow[], double eigen_real[], double eigen_imag[], int row, double shift)
{
    Hrow[row] += shift;
    eigen_real[row] = Hrow[row];
    eigen_imag[row] = 0.0;
}

static void Two_Eigenvalues(double *H, double *S, double eigen_real[], double eigen_imag[], int n, int row, double shift)
{
    double p, q, x, discriminant, r;
    double cos, sin;
    double *Hrow = H + n * row;
    double *Hnextrow = Hrow + n;
    int nextrow = row + 1;

    p = 0.5 * (Hrow[row] - Hnextrow[nextrow]);
    x = Hrow[nextrow] * Hnextrow[row];
    discriminant = p * p + x;
    Hrow[row] += shift;
    Hnextrow[nextrow] += shift;
    if (discriminant > 0.0)
    { // pair of real roots
        q = sqrt(discriminant);
        if (p < 0.0)
            q = p - q;
        else
            q += p;
        eigen_real[row] = Hnextrow[nextrow] + q;
        eigen_real[nextrow] = Hnextrow[nextrow] - x / q;
        eigen_imag[row] = 0.0;
        eigen_imag[nextrow] = 0.0;
        r = sqrt(Hnextrow[row] * Hnextrow[row] + q * q);
        sin = Hnextrow[row] / r;
        cos = q / r;
        Update_Row(Hrow, cos, sin, n, row);
        Update_Column(H, cos, sin, n, row);
        Update_Transformation(S, cos, sin, n, row);
    }
    else
    { // pair of complex roots
        eigen_real[nextrow] = eigen_real[row] = Hnextrow[nextrow] + p;
        eigen_imag[row] = sqrt(fabs(discriminant));
        eigen_imag[nextrow] = -eigen_imag[row];
    }
}

int Two_Consecutive_Small_Subdiagonal(double *H, int min_row, int max_row, int n, double trace, double det)
{
    double x, y, z, s;
    double *pH;
    int i, k;

    for (k = max_row - 2, pH = H + k * n; k >= min_row; pH -= n, k--)
    {
        x = (pH[k] * (pH[k] - trace) + det) / pH[n + k] + pH[k + 1];
        y = pH[k] + pH[n + k + 1] - trace;
        z = pH[n + n + k + 1];
        s = fabs(x) + fabs(y) + fabs(z);
        x /= s;
        y /= s;
        z /= s;
        if (k == min_row)
            break;
        if ((fabs(pH[k - 1]) * (fabs(y) + fabs(z))) <=
            DBL_EPSILON * fabs(x) *
                (fabs(pH[k - 1 - n]) + fabs(pH[k]) + fabs(pH[n + k + 1])))
            break;
    }
    for (i = k + 2, pH = H + i * n; i <= max_row; pH += n, i++)
        pH[i - 2] = 0.0;
    for (i = k + 3, pH = H + i * n; i <= max_row; pH += n, i++)
        pH[i - 3] = 0.0;
    return k;
}

void Product_and_Sum_of_Shifts(double *H, int n, int max_row, double *shift, double *trace, double *det, int iteration)
{
    double *pH = H + max_row * n;
    double *p_aux;
    int i;
    int min_col = max_row - 1;

    if ((iteration % 10) == 0)
    {
        *shift += pH[max_row];
        for (i = 0, p_aux = H; i <= max_row; p_aux += n, i++)
            p_aux[i] -= pH[max_row];
        p_aux = pH - n;
        *trace = fabs(pH[min_col]) + fabs(p_aux[min_col - 1]);
        *det = *trace * *trace;
        *trace *= 1.5;
    }
    else
    {
        p_aux = pH - n;
        *trace = p_aux[min_col] + pH[max_row];
        *det = p_aux[min_col] * pH[max_row] - p_aux[max_row] * pH[min_col];
    }
};

static void Double_QR_Step(double *H, int min_row, int max_row, int min_col, double trace, double det, double *S, int n)
{
    double s, x, y, z;
    double a, b, c;
    double *pH;
    double *tH;
    double *pS;
    int i, j, k;
    int last_test_row_col = max_row - 1;

    x = 0;

    k = min_col;
    pH = H + min_col * n;
    a = (pH[k] * (pH[k] - trace) + det) / pH[n + k] + pH[k + 1];
    b = pH[k] + pH[n + k + 1] - trace;
    c = pH[n + n + k + 1];
    s = fabs(a) + fabs(b) + fabs(c);
    a /= s;
    b /= s;
    c /= s;

    for (; k <= last_test_row_col; k++, pH += n)
    {
        if (k > min_col)
        {
            c = (k == last_test_row_col) ? 0.0 : pH[n + n + k - 1];
            x = fabs(pH[k - 1]) + fabs(pH[n + k - 1]) + fabs(c);
            if (x == 0.0)
                continue;
            a = pH[k - 1] / x;
            b = pH[n + k - 1] / x;
            c /= x;
        }
        s = sqrt(a * a + b * b + c * c);
        if (a < 0.0)
            s = -s;
        if (k > min_col)
            pH[k - 1] = -s * x;
        else if (min_row != min_col)
            pH[k - 1] = -pH[k - 1];
        a += s;
        x = a / s;
        y = b / s;
        z = c / s;
        b /= a;
        c /= a;

        // Update rows k, k+1, k+2
        for (j = k; j < n; j++)
        {
            a = pH[j] + b * pH[n + j];
            if (k != last_test_row_col)
            {
                a += c * pH[n + n + j];
                pH[n + n + j] -= a * z;
            }
            pH[n + j] -= a * y;
            pH[j] -= a * x;
        }

        // Update column k+1

        j = k + 3;
        if (j > max_row)
            j = max_row;
        for (i = 0, tH = H; i <= j; i++, tH += n)
        {
            a = x * tH[k] + y * tH[k + 1];
            if (k != last_test_row_col)
            {
                a += z * tH[k + 2];
                tH[k + 2] -= a * c;
            }
            tH[k + 1] -= a * b;
            tH[k] -= a;
        }

        // Update transformation matrix

        for (i = 0, pS = S; i < n; pS += n, i++)
        {
            a = x * pS[k] + y * pS[k + 1];
            if (k != last_test_row_col)
            {
                a += z * pS[k + 2];
                pS[k + 2] -= a * c;
            }
            pS[k + 1] -= a * b;
            pS[k] -= a;
        }
    };
}

void Double_QR_Iteration(double *H, double *S, int min_row, int max_row, int n, double *shift, int iteration)
{
    int k;
    double trace, det;

    Product_and_Sum_of_Shifts(H, n, max_row, shift, &trace, &det, iteration);
    k = Two_Consecutive_Small_Subdiagonal(H, min_row, max_row, n, trace, det);
    Double_QR_Step(H, min_row, max_row, k, trace, det, S, n);
}

void BackSubstitute_Real_Vector(double *H, double eigen_real[], double eigen_imag[], int row, double zero_tolerance, int n)
{
    double *pH;
    double *pV;
    // double x, y;
    double x;
    double u[4];
    double v[2];
    int i, j, k;

    k = row;
    pH = H + row * n;
    pH[row] = 1.0;
    for (i = row - 1, pH -= n; i >= 0; i--, pH -= n)
    {
        u[0] = pH[i] - eigen_real[row];
        v[0] = pH[row];
        pV = H + n * k;
        for (j = k; j < row; j++, pV += n)
            v[0] += pH[j] * pV[row];
        if (eigen_imag[i] < 0.0)
        {
            u[3] = u[0];
            v[1] = v[0];
        }
        else
        {
            k = i;
            if (eigen_imag[i] == 0.0)
            {
                if (u[0] != 0.0)
                    pH[row] = -v[0] / u[0];
                else
                    pH[row] = -v[0] / zero_tolerance;
            }
            else
            {
                u[1] = pH[i + 1];
                u[2] = pH[n + i];
                x = (eigen_real[i] - eigen_real[row]);
                x *= x;
                x += eigen_imag[i] * eigen_imag[i];
                pH[row] = (u[1] * v[1] - u[3] * v[0]) / x;
                if (fabs(u[1]) > fabs(u[3]))
                    pH[n + row] = -(v[0] + u[0] * pH[row]) / u[1];
                else
                    pH[n + row] = -(v[1] + u[2] * pH[row]) / u[3];
            }
        }
    }
}

static void BackSubstitute_Complex_Vector(double *H, double eigen_real[], double eigen_imag[], int row, double zero_tolerance, int n)
{
    double *pH;
    double *pV;
    double x, y;
    double u[4];
    double v[2];
    double w[2];
    int i, j, k;

    k = row - 1;
    pH = H + n * row;
    if (fabs(pH[k]) > fabs(pH[row - n]))
    {
        pH[k - n] = -(pH[row] - eigen_real[row]) / pH[k];
        pH[row - n] = -eigen_imag[row] / pH[k];
    }
    else
        Complex_Division(-pH[row - n], 0.0,
                         pH[k - n] - eigen_real[row], eigen_imag[row], &pH[k - n], &pH[row - n]);
    pH[k] = 1.0;
    pH[row] = 0.0;
    for (i = row - 2, pH = H + n * i; i >= 0; pH -= n, i--)
    {
        u[0] = pH[i] - eigen_real[row];
        w[0] = pH[row];
        w[1] = 0.0;
        pV = H + k * n;
        for (j = k; j < row; j++, pV += n)
        {
            w[0] += pH[j] * pV[row - 1];
            w[1] += pH[j] * pV[row];
        }
        if (eigen_imag[i] < 0.0)
        {
            u[3] = u[0];
            v[0] = w[0];
            v[1] = w[1];
        }
        else
        {
            k = i;
            if (eigen_imag[i] == 0.0)
            {
                Complex_Division(-w[0], -w[1], u[0], eigen_imag[row], &pH[row - 1],
                                 &pH[row]);
            }
            else
            {
                u[1] = pH[i + 1];
                u[2] = pH[n + i];
                x = eigen_real[i] - eigen_real[row];
                y = 2.0 * x * eigen_imag[row];
                x = x * x + eigen_imag[i] * eigen_imag[i] - eigen_imag[row] * eigen_imag[row];
                if (x == 0.0 && y == 0.0)
                    x = zero_tolerance * (fabs(u[0]) + fabs(u[1]) + fabs(u[2]) + fabs(u[3]) + fabs(eigen_imag[row]));
                Complex_Division(u[1] * v[0] - u[3] * w[0] + w[1] * eigen_imag[row],
                                 u[1] * v[1] - u[3] * w[1] - w[0] * eigen_imag[row],
                                 x, y, &pH[row - 1], &pH[row]);
                if (fabs(u[1]) > (fabs(u[3]) + fabs(eigen_imag[row])))
                {
                    pH[n + row - 1] = -w[0] - u[0] * pH[row - 1] + eigen_imag[row] * pH[row] / u[1];
                    pH[n + row] = -w[1] - u[0] * pH[row] - eigen_imag[row] * pH[row - 1] / u[1];
                }
                else
                {
                    Complex_Division(-v[0] - u[2] * pH[row - 1], -v[1] - u[2] * pH[row],
                                     u[3], eigen_imag[row], &pH[n + row - 1], &pH[n + row]);
                }
            }
        }
    }
}

void BackSubstitution(double *H, double eigen_real[], double eigen_imag[], int n)
{
    double zero_tolerance;
    double *pH;
    int i, j, row;

    // Calculate the zero tolerance

    pH = H;
    zero_tolerance = fabs(pH[0]);
    for (pH += n, i = 1; i < n; pH += n, i++)
        for (j = i - 1; j < n; j++)
            zero_tolerance += fabs(pH[j]);
    zero_tolerance *= DBL_EPSILON;

    // Start Backsubstitution

    for (row = n - 1; row >= 0; row--)
    {
        if (eigen_imag[row] == 0.0)
            BackSubstitute_Real_Vector(H, eigen_real, eigen_imag, row,
                                       zero_tolerance, n);
        else if (eigen_imag[row] < 0.0)
            BackSubstitute_Complex_Vector(H, eigen_real, eigen_imag, row,
                                          zero_tolerance, n);
    }
}

int QR_Hessenberg_Matrix(double *H, double *S, double eigen_real[], double eigen_imag[], int n, int max_iteration_count)
{
    int i;
    int row;
    int iteration;
    int found_eigenvalue;
    double shift = 0.0;
    double *pH;

    for (row = n - 1; row >= 0; row--)
    {
        found_eigenvalue = 0;
        for (iteration = 1; iteration <= max_iteration_count; iteration++)
        {

            // Search for small subdiagonal element

            for (i = row, pH = H + row * n; i > 0; i--, pH -= n)
                if (fabs(*(pH + i - 1)) <= DBL_EPSILON *
                                               (fabs(*(pH - n + i - 1)) + fabs(*(pH + i))))
                    break;

            // If the subdiagonal element on row "row" is small, then
            // that row element is an eigenvalue.  If the subdiagonal
            // element on row "row-1" is small, then the eigenvalues
            // of the 2x2 diagonal block consisting rows "row-1" and
            // "row" are eigenvalues.  Otherwise perform a double QR
            // iteration.

            switch (row - i)
            {
            case 0: // One real eigenvalue
                One_Real_Eigenvalue(pH, eigen_real, eigen_imag, i, shift);
                found_eigenvalue = 1;
                break;
            case 1: // Either two real eigenvalues or a complex pair
                row--;
                Two_Eigenvalues(H, S, eigen_real, eigen_imag, n, row, shift);
                found_eigenvalue = 1;
                break;
            default:
                Double_QR_Iteration(H, S, i, row, n, &shift, iteration);
            }
            if (found_eigenvalue)
                break;
        }
        if (iteration > max_iteration_count)
            return -1;
    }

    BackSubstitution(H, eigen_real, eigen_imag, n);
    Calculate_Eigenvectors(H, S, eigen_real, eigen_imag, n);

    return 0;
}

void calculation_mean_and_sigma_of_vector_array(float array[][3], double *mean, double *sigma, uint16_t number_of_elements)
{
    double x2 = 0;
    *sigma = 0;
    *mean = 0;
    for (uint16_t i = 0; i < number_of_elements; i++)
    {
        x2 = array[i][0] * array[i][0] + array[i][1] * array[i][1] + array[i][2] * array[i][2];
        *mean += sqrt(x2);
        *sigma += x2;
    }
    *mean = *mean / number_of_elements;                           // mean vector length
    *sigma = sqrt(*sigma / number_of_elements - (*mean * *mean)); // std. dev.
}

void calculation_B_and_Ainv(float array[][3], double *A_1, double *B, double *mean, double *sigma, uint16_t number_of_elements)
{
    double *D, *S, *C, *S11, *S12, *S12t, *S22, *S22_1, *S22a, *S22b, *SS, *E, *U, *SSS;
    double *eigen_real, *eigen_imag, *v1, *v2, *v, *Q, *Q_1, *QB, J, hmb, *SSSS;
    int i, j, index;
    double maxval, norm, btqb, *eigen_real3, *eigen_imag3, *Dz, *vdz, *SQ, vector_norm, norm1, norm2, norm3;
    double *raw;
    double x2;

    D = (double *)malloc(10 * number_of_elements * sizeof(double));
    raw = (double *)malloc(3 * number_of_elements * sizeof(double));

    j = 0; // array index for good measurements

    for (i = 0; i < number_of_elements; i++)
    {
        x2 = sqrt(array[i][0] * array[i][0] + array[i][1] * array[i][1] + array[i][2] * array[i][2]); // vector length
        x2 = fabs(x2 - *mean) / *sigma;                                                               // standard deviation from mean

        raw[3 * j] = array[i][0];
        raw[3 * j + 1] = array[i][1];
        raw[3 * j + 2] = array[i][2];
        D[j] = array[i][0] * array[i][0];
        D[number_of_elements + j] = array[i][1] * array[i][1];
        D[number_of_elements * 2 + j] = array[i][2] * array[i][2];
        D[number_of_elements * 3 + j] = 2.0 * array[i][1] * array[i][2];
        D[number_of_elements * 4 + j] = 2.0 * array[i][0] * array[i][2];
        D[number_of_elements * 5 + j] = 2.0 * array[i][0] * array[i][1];
        D[number_of_elements * 6 + j] = 2.0 * array[i][0];
        D[number_of_elements * 7 + j] = 2.0 * array[i][1];
        D[number_of_elements * 8 + j] = 2.0 * array[i][2];
        D[number_of_elements * 9 + j] = 1.0;
        j++; // count good measurements
    }
    // number_of_elements = number_of_elements; // number to process

    vector_norm = *mean;
    printf("\r\nУстанавливаем нормальное значение вектора = %8.6f\r\n", vector_norm);

    // allocate memory for matrix S
    S = (double *)malloc(10 * 10 * sizeof(double));
    Multiply_Self_Transpose(S, D, 10, number_of_elements);

    // Create pre-inverted constraint matrix C
    C = (double *)malloc(6 * 6 * sizeof(double));
    C[0] = 0.0;
    C[1] = 0.5;
    C[2] = 0.5;
    C[3] = 0.0;
    C[4] = 0.0;
    C[5] = 0.0;
    C[6] = 0.5;
    C[7] = 0.0;
    C[8] = 0.5;
    C[9] = 0.0;
    C[10] = 0.0;
    C[11] = 0.0;
    C[12] = 0.5;
    C[13] = 0.5;
    C[14] = 0.0;
    C[15] = 0.0;
    C[16] = 0.0;
    C[17] = 0.0;
    C[18] = 0.0;
    C[19] = 0.0;
    C[20] = 0.0;
    C[21] = -0.25;
    C[22] = 0.0;
    C[23] = 0.0;
    C[24] = 0.0;
    C[25] = 0.0;
    C[26] = 0.0;
    C[27] = 0.0;
    C[28] = -0.25;
    C[29] = 0.0;
    C[30] = 0.0;
    C[31] = 0.0;
    C[32] = 0.0;
    C[33] = 0.0;
    C[34] = 0.0;
    C[35] = -0.25;

    S11 = (double *)malloc(6 * 6 * sizeof(double));
    Get_Submatrix(S11, 6, 6, S, 10, 0, 0);
    S12 = (double *)malloc(6 * 4 * sizeof(double));
    Get_Submatrix(S12, 6, 4, S, 10, 0, 6);
    S12t = (double *)malloc(4 * 6 * sizeof(double));
    Get_Submatrix(S12t, 4, 6, S, 10, 6, 0);
    S22 = (double *)malloc(4 * 4 * sizeof(double));
    Get_Submatrix(S22, 4, 4, S, 10, 6, 6);

    S22_1 = (double *)malloc(4 * 4 * sizeof(double));
    for (i = 0; i < 16; i++)
        S22_1[i] = S22[i];
    Choleski_LU_Decomposition(S22_1, 4);
    Choleski_LU_Inverse(S22_1, 4);

    // Calculate S22a = S22_1 * S12t   4*6 = 4x4 * 4x6   C = AB
    S22a = (double *)malloc(4 * 6 * sizeof(double));
    Multiply_Matrices(S22a, S22_1, 4, 4, S12t, 6);

    // Then calculate S22b = S12 * S22a      ( 6x6 = 6x4 * 4x6)
    S22b = (double *)malloc(6 * 6 * sizeof(double));
    Multiply_Matrices(S22b, S12, 6, 4, S22a, 6);

    // Calculate SS = S11 - S22b
    SS = (double *)malloc(6 * 6 * sizeof(double));
    for (i = 0; i < 36; i++)
        SS[i] = S11[i] - S22b[i];
    E = (double *)malloc(6 * 6 * sizeof(double));
    Multiply_Matrices(E, C, 6, 6, SS, 6);

    SSS = (double *)malloc(6 * 6 * sizeof(double));
    Hessenberg_Form_Elementary(E, SSS, 6);

    eigen_real = (double *)malloc(6 * sizeof(double));
    eigen_imag = (double *)malloc(6 * sizeof(double));

    QR_Hessenberg_Matrix(E, SSS, eigen_real, eigen_imag, 6, 100);

    index = 0;
    maxval = eigen_real[0];
    for (i = 1; i < 6; i++)
    {
        if (eigen_real[i] > maxval)
        {
            maxval = eigen_real[i];
            index = i;
        }
    }

    v1 = (double *)malloc(6 * sizeof(double));

    v1[0] = SSS[index];
    v1[1] = SSS[index + 6];
    v1[2] = SSS[index + 12];
    v1[3] = SSS[index + 18];
    v1[4] = SSS[index + 24];
    v1[5] = SSS[index + 30];

    // normalize v1
    norm = sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2] + v1[3] * v1[3] + v1[4] * v1[4] + v1[5] * v1[5]);
    v1[0] /= norm;
    v1[1] /= norm;
    v1[2] /= norm;
    v1[3] /= norm;
    v1[4] /= norm;
    v1[5] /= norm;

    if (v1[0] < 0.0)
    {
        v1[0] = -v1[0];
        v1[1] = -v1[1];
        v1[2] = -v1[2];
        v1[3] = -v1[3];
        v1[4] = -v1[4];
        v1[5] = -v1[5];
    }

    // Calculate v2 = S22a * v1      ( 4x1 = 4x6 * 6x1)
    v2 = (double *)malloc(4 * sizeof(double));
    Multiply_Matrices(v2, S22a, 4, 6, v1, 1);

    v = (double *)malloc(10 * sizeof(double));

    v[0] = v1[0];
    v[1] = v1[1];
    v[2] = v1[2];
    v[3] = v1[3];
    v[4] = v1[4];
    v[5] = v1[5];
    v[6] = -v2[0];
    v[7] = -v2[1];
    v[8] = -v2[2];
    v[9] = -v2[3];

    Q = (double *)malloc(3 * 3 * sizeof(double));

    Q[0] = v[0];
    Q[1] = v[5];
    Q[2] = v[4];
    Q[3] = v[5];
    Q[4] = v[1];
    Q[5] = v[3];
    Q[6] = v[4];
    Q[7] = v[3];
    Q[8] = v[2];

    U = (double *)malloc(3 * sizeof(double));

    U[0] = v[6];
    U[1] = v[7];
    U[2] = v[8];
    Q_1 = (double *)malloc(3 * 3 * sizeof(double));
    for (i = 0; i < 9; i++)
        Q_1[i] = Q[i];
    Choleski_LU_Decomposition(Q_1, 3);
    Choleski_LU_Inverse(Q_1, 3);

    // Calculate B = Q-1 * U   ( 3x1 = 3x3 * 3x1)

    Multiply_Matrices(B, Q_1, 3, 3, U, 1);
    B[0] = -B[0]; // x-axis combined bias
    B[1] = -B[1]; // y-axis combined bias
    B[2] = -B[2]; // z-axis combined bias

    // for(i = 0; i < 3; i++)
    printf("\r\nКоректировочные значения сдвигов (bias):\r\n");
    printf("%8.6lf %8.6lf %8.6lf \r\n", B[0], B[1], B[2]);

    // First calculate QB = Q * B   ( 3x1 = 3x3 * 3x1)
    QB = (double *)malloc(3 * sizeof(double));
    Multiply_Matrices(QB, Q, 3, 3, B, 1);

    // Then calculate btqb = BT * QB    ( 1x1 = 1x3 * 3x1)
    Multiply_Matrices(&btqb, B, 1, 3, QB, 1);

    // Calculate hmb = sqrt(btqb - J).
    J = v[9];
    hmb = sqrt(btqb - J);

    // Calculate SQ, the square root of matrix Q
    SSSS = (double *)malloc(3 * 3 * sizeof(double));
    Hessenberg_Form_Elementary(Q, SSSS, 3);

    eigen_real3 = (double *)malloc(3 * sizeof(double));
    eigen_imag3 = (double *)malloc(3 * sizeof(double));
    QR_Hessenberg_Matrix(Q, SSSS, eigen_real3, eigen_imag3, 3, 100);

    // normalize eigenvectors
    norm1 = sqrt(SSSS[0] * SSSS[0] + SSSS[3] * SSSS[3] + SSSS[6] * SSSS[6]);
    SSSS[0] /= norm1;
    SSSS[3] /= norm1;
    SSSS[6] /= norm1;
    norm2 = sqrt(SSSS[1] * SSSS[1] + SSSS[4] * SSSS[4] + SSSS[7] * SSSS[7]);
    SSSS[1] /= norm2;
    SSSS[4] /= norm2;
    SSSS[7] /= norm2;
    norm3 = sqrt(SSSS[2] * SSSS[2] + SSSS[5] * SSSS[5] + SSSS[8] * SSSS[8]);
    SSSS[2] /= norm3;
    SSSS[5] /= norm3;
    SSSS[8] /= norm3;

    Dz = (double *)malloc(3 * 3 * sizeof(double));
    for (i = 0; i < 9; i++)
        Dz[i] = 0.0;
    Dz[0] = sqrt(eigen_real3[0]);
    Dz[4] = sqrt(eigen_real3[1]);
    Dz[8] = sqrt(eigen_real3[2]);

    vdz = (double *)malloc(3 * 3 * sizeof(double));
    Multiply_Matrices(vdz, SSSS, 3, 3, Dz, 3);

    Transpose_Square_Matrix(SSSS, 3);

    SQ = (double *)malloc(3 * 3 * sizeof(double));
    Multiply_Matrices(SQ, vdz, 3, 3, SSSS, 3);

    for (i = 0; i < 9; i++)
        A_1[i] = SQ[i] * vector_norm / hmb;

    printf("\r\nКорректирующая матрица Ainv (для V_norm = %8.6f):\r\n", vector_norm);
    for (i = 0; i < 3; i++)
        printf("%9.6lf %9.6lf %9.6lf\r\n", A_1[i * 3], A_1[i * 3 + 1], A_1[i * 3 + 2]);

    free(D);
    free(S);
    free(C);
    free(S11);
    free(S12);
    free(S12t);
    free(S22);
    free(S22_1);
    free(S22a);
    free(S22b);
    free(SS);
    free(E);
    free(U);
    free(SSS);
    free(eigen_real);
    free(eigen_imag);
    free(v1);
    free(v2);
    free(v);
    free(Q);
    free(Q_1);
    free(QB);
    free(SSSS);
    free(eigen_real3);
    free(eigen_imag3);
    free(Dz);
    free(vdz);
    free(SQ);
}

uint8_t calculation_B_and_Ainv_with_exclusion(float array[][3], double *A_1, double *B, double vector_norm, double rej_threshold, uint16_t number_of_elements)
{
    double *D, *S, *C, *S11, *S12, *S12t, *S22, *S22_1, *S22a, *S22b, *SS, *E, *U, *SSS;
    double *eigen_real, *eigen_imag, *v1, *v2, *v, *Q, *Q_1, *QB, J, hmb, *SSSS;
    int i, j, index;
    double maxval, norm, btqb, *eigen_real3, *eigen_imag3, *Dz, *vdz, *SQ, norm1, norm2, norm3;
    double *raw;
    double x2;
    double mean = 0;
    double sigma = 0;
    uint16_t n_good_vectors;
    
    //сначала ищем среднее значение модуля массива векторов (mean) и стандартное отклонение (sigma) 
    for (uint16_t i = 0; i < number_of_elements; i++)
    {
        x2 = array[i][0] * array[i][0] + array[i][1] * array[i][1] + array[i][2] * array[i][2];
        mean += sqrt(x2);
        sigma += x2;
    }

    mean = mean / number_of_elements;                         // среднее значение
    sigma = sqrt(sigma / number_of_elements - (mean * mean)); // стандартное отклонение

    printf("Среднее значение (mean) и среднеквадратичное отклонение (sigma) обработанного массива из %d векторов = %6.6lf, %6.6lf\r\n", number_of_elements, mean, sigma);
    printf("Выбранный допустимый уровень отклонения от среднеквадратичного отклонения = %0.3f\n", rej_threshold);
    if (rej_threshold == 0) printf("если уровень отклонения указан равным нулю, в расчет идут все вектора\n");
    else printf("отбрасываем вектор если |длина вектора - mean|/(sigma) > %5.1f\r\n", rej_threshold);

    //далее надо исключить из расчета вектора, отклонение которых больше среднеквадратичного отклонения более чем в rej_threshold раз
    n_good_vectors = number_of_elements; // пока все вектора считаем хорошими   
    if (rej_threshold > 0)
        {
            // подсчитываем сколько векторов отбрасываются из общего количества
            for (uint16_t i = 0; i < number_of_elements; i++)
            {
                // еще раз считаем длину вектора
                x2 = sqrt(array[i][0] * array[i][0] + array[i][1] * array[i][1] + array[i][2] * array[i][2]);
                // находим отношение текушего отклонения от среднеквадратичного по всему объему
                x2 = fabs(x2 - mean) / sigma;
                //если текущее превышает                                                                                               
                if (x2 > rej_threshold) 
                {
                    n_good_vectors--; // уменьшаем кол-во хороших векторов
                    printf("отбрасываем из расчетов вектор #%d, %8.1f %8.1f %8.1f\n, отношение его отклонения к sigma %0.2f\n", i, array[i][0], array[i][1], array[i][2], x2);
                }
            }
            printf("\r\nколичество отброшенных векторов: %d, принятых: %d\r\n", number_of_elements - n_good_vectors, n_good_vectors);
        }

    // выделяем память под обработку "хороших" векторов
    D = (double *)malloc(10 * n_good_vectors * sizeof(double));
    raw = (double *)malloc(3 * n_good_vectors * sizeof(double));

    j = 0;

    for (i = 0; i < number_of_elements; i++)
    {
        // 3й раз считаем модуль вектора
        x2 = sqrt(array[i][0] * array[i][0] + array[i][1] * array[i][1] + array[i][2] * array[i][2]);
        // и его отклонение
        x2 = fabs(x2 - mean) / sigma;
        // если проходит то копируем в выделенную область памяти для дальнейшей обработки
        if ((x2 <= rej_threshold) || (rej_threshold == 0)) 
        {
            raw[3 * j] = array[i][0];
            raw[3 * j + 1] = array[i][1];
            raw[3 * j + 2] = array[i][2];
            D[j] = array[i][0] * array[i][0];
            D[number_of_elements + j] = array[i][1] * array[i][1];
            D[number_of_elements * 2 + j] = array[i][2] * array[i][2];
            D[number_of_elements * 3 + j] = 2.0 * array[i][1] * array[i][2];
            D[number_of_elements * 4 + j] = 2.0 * array[i][0] * array[i][2];
            D[number_of_elements * 5 + j] = 2.0 * array[i][0] * array[i][1];
            D[number_of_elements * 6 + j] = 2.0 * array[i][0];
            D[number_of_elements * 7 + j] = 2.0 * array[i][1];
            D[number_of_elements * 8 + j] = 2.0 * array[i][2];
            D[number_of_elements * 9 + j] = 1.0;
            j++; // подсчитываем "хорошие" вектора для кросс-проверки
        }
    }

    printf("\r\nОбработано %3d векторов, ожидалось %d\r\n", j, n_good_vectors);
    if (j != n_good_vectors)
    {
        printf("Ошибка индексирования!\r\n");
        return 0;
    }
    
    printf("Выбранное нормальное значение вектора = %8.2f\r\n", vector_norm);

    //allocate memory for matrix S
    S = (double *)malloc(10 * 10 * sizeof(double));
    Multiply_Self_Transpose(S, D, 10, number_of_elements);

    // Create pre-inverted constraint matrix C
    C = (double *)malloc(6 * 6 * sizeof(double));
    C[0] = 0.0;
    C[1] = 0.5;
    C[2] = 0.5;
    C[3] = 0.0;
    C[4] = 0.0;
    C[5] = 0.0;
    C[6] = 0.5;
    C[7] = 0.0;
    C[8] = 0.5;
    C[9] = 0.0;
    C[10] = 0.0;
    C[11] = 0.0;
    C[12] = 0.5;
    C[13] = 0.5;
    C[14] = 0.0;
    C[15] = 0.0;
    C[16] = 0.0;
    C[17] = 0.0;
    C[18] = 0.0;
    C[19] = 0.0;
    C[20] = 0.0;
    C[21] = -0.25;
    C[22] = 0.0;
    C[23] = 0.0;
    C[24] = 0.0;
    C[25] = 0.0;
    C[26] = 0.0;
    C[27] = 0.0;
    C[28] = -0.25;
    C[29] = 0.0;
    C[30] = 0.0;
    C[31] = 0.0;
    C[32] = 0.0;
    C[33] = 0.0;
    C[34] = 0.0;
    C[35] = -0.25;

    S11 = (double *)malloc(6 * 6 * sizeof(double));
    Get_Submatrix(S11, 6, 6, S, 10, 0, 0);
    S12 = (double *)malloc(6 * 4 * sizeof(double));
    Get_Submatrix(S12, 6, 4, S, 10, 0, 6);
    S12t = (double *)malloc(4 * 6 * sizeof(double));
    Get_Submatrix(S12t, 4, 6, S, 10, 6, 0);
    S22 = (double *)malloc(4 * 4 * sizeof(double));
    Get_Submatrix(S22, 4, 4, S, 10, 6, 6);

    S22_1 = (double *)malloc(4 * 4 * sizeof(double));
    for (i = 0; i < 16; i++)
        S22_1[i] = S22[i];
    Choleski_LU_Decomposition(S22_1, 4);
    Choleski_LU_Inverse(S22_1, 4);

    // Calculate S22a = S22_1 * S12t   4*6 = 4x4 * 4x6   C = AB
    S22a = (double *)malloc(4 * 6 * sizeof(double));
    Multiply_Matrices(S22a, S22_1, 4, 4, S12t, 6);

    // Then calculate S22b = S12 * S22a      ( 6x6 = 6x4 * 4x6)
    S22b = (double *)malloc(6 * 6 * sizeof(double));
    Multiply_Matrices(S22b, S12, 6, 4, S22a, 6);

    // Calculate SS = S11 - S22b
    SS = (double *)malloc(6 * 6 * sizeof(double));
    for (i = 0; i < 36; i++)
        SS[i] = S11[i] - S22b[i];
    E = (double *)malloc(6 * 6 * sizeof(double));
    Multiply_Matrices(E, C, 6, 6, SS, 6);

    SSS = (double *)malloc(6 * 6 * sizeof(double));
    Hessenberg_Form_Elementary(E, SSS, 6);

    eigen_real = (double *)malloc(6 * sizeof(double));
    eigen_imag = (double *)malloc(6 * sizeof(double));

    QR_Hessenberg_Matrix(E, SSS, eigen_real, eigen_imag, 6, 100);

    index = 0;
    maxval = eigen_real[0];
    for (i = 1; i < 6; i++)
    {
        if (eigen_real[i] > maxval)
        {
            maxval = eigen_real[i];
            index = i;
        }
    }

    v1 = (double *)malloc(6 * sizeof(double));

    v1[0] = SSS[index];
    v1[1] = SSS[index + 6];
    v1[2] = SSS[index + 12];
    v1[3] = SSS[index + 18];
    v1[4] = SSS[index + 24];
    v1[5] = SSS[index + 30];

    // normalize v1
    norm = sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2] + v1[3] * v1[3] + v1[4] * v1[4] + v1[5] * v1[5]);
    v1[0] /= norm;
    v1[1] /= norm;
    v1[2] /= norm;
    v1[3] /= norm;
    v1[4] /= norm;
    v1[5] /= norm;

    if (v1[0] < 0.0)
    {
        v1[0] = -v1[0];
        v1[1] = -v1[1];
        v1[2] = -v1[2];
        v1[3] = -v1[3];
        v1[4] = -v1[4];
        v1[5] = -v1[5];
    }

    // Calculate v2 = S22a * v1      ( 4x1 = 4x6 * 6x1)
    v2 = (double *)malloc(4 * sizeof(double));
    Multiply_Matrices(v2, S22a, 4, 6, v1, 1);

    v = (double *)malloc(10 * sizeof(double));

    v[0] = v1[0];
    v[1] = v1[1];
    v[2] = v1[2];
    v[3] = v1[3];
    v[4] = v1[4];
    v[5] = v1[5];
    v[6] = -v2[0];
    v[7] = -v2[1];
    v[8] = -v2[2];
    v[9] = -v2[3];

    Q = (double *)malloc(3 * 3 * sizeof(double));

    Q[0] = v[0];
    Q[1] = v[5];
    Q[2] = v[4];
    Q[3] = v[5];
    Q[4] = v[1];
    Q[5] = v[3];
    Q[6] = v[4];
    Q[7] = v[3];
    Q[8] = v[2];

    U = (double *)malloc(3 * sizeof(double));

    U[0] = v[6];
    U[1] = v[7];
    U[2] = v[8];
    Q_1 = (double *)malloc(3 * 3 * sizeof(double));
    for (i = 0; i < 9; i++)
        Q_1[i] = Q[i];
    Choleski_LU_Decomposition(Q_1, 3);
    Choleski_LU_Inverse(Q_1, 3);

    // Calculate B = Q-1 * U   ( 3x1 = 3x3 * 3x1)

    Multiply_Matrices(B, Q_1, 3, 3, U, 1);
    B[0] = -B[0]; // x-axis combined bias
    B[1] = -B[1]; // y-axis combined bias
    B[2] = -B[2]; // z-axis combined bias

    printf("\r\nКоректировочные значения сдвигов (bias):\r\n");
    printf("%8.6lf %8.6lf %8.6lf \r\n", B[0], B[1], B[2]);

    // First calculate QB = Q * B   ( 3x1 = 3x3 * 3x1)
    QB = (double *)malloc(3 * sizeof(double));
    Multiply_Matrices(QB, Q, 3, 3, B, 1);

    // Then calculate btqb = BT * QB    ( 1x1 = 1x3 * 3x1)
    Multiply_Matrices(&btqb, B, 1, 3, QB, 1);

    // Calculate hmb = sqrt(btqb - J).
    J = v[9];
    hmb = sqrt(btqb - J);

    // Calculate SQ, the square root of matrix Q
    SSSS = (double *)malloc(3 * 3 * sizeof(double));
    Hessenberg_Form_Elementary(Q, SSSS, 3);

    eigen_real3 = (double *)malloc(3 * sizeof(double));
    eigen_imag3 = (double *)malloc(3 * sizeof(double));
    QR_Hessenberg_Matrix(Q, SSSS, eigen_real3, eigen_imag3, 3, 100);

    // normalize eigenvectors
    norm1 = sqrt(SSSS[0] * SSSS[0] + SSSS[3] * SSSS[3] + SSSS[6] * SSSS[6]);
    SSSS[0] /= norm1;
    SSSS[3] /= norm1;
    SSSS[6] /= norm1;
    norm2 = sqrt(SSSS[1] * SSSS[1] + SSSS[4] * SSSS[4] + SSSS[7] * SSSS[7]);
    SSSS[1] /= norm2;
    SSSS[4] /= norm2;
    SSSS[7] /= norm2;
    norm3 = sqrt(SSSS[2] * SSSS[2] + SSSS[5] * SSSS[5] + SSSS[8] * SSSS[8]);
    SSSS[2] /= norm3;
    SSSS[5] /= norm3;
    SSSS[8] /= norm3;

    Dz = (double *)malloc(3 * 3 * sizeof(double));
    for (i = 0; i < 9; i++)
        Dz[i] = 0.0;
    Dz[0] = sqrt(eigen_real3[0]);
    Dz[4] = sqrt(eigen_real3[1]);
    Dz[8] = sqrt(eigen_real3[2]);

    vdz = (double *)malloc(3 * 3 * sizeof(double));
    Multiply_Matrices(vdz, SSSS, 3, 3, Dz, 3);

    Transpose_Square_Matrix(SSSS, 3);

    SQ = (double *)malloc(3 * 3 * sizeof(double));
    Multiply_Matrices(SQ, vdz, 3, 3, SSSS, 3);

    for (i = 0; i < 9; i++)
        A_1[i] = SQ[i] * vector_norm / hmb;

    printf("\r\nКорректирующая матрица Ainv (для V_norm = %8.6f):\r\n", vector_norm);
    for (i = 0; i < 3; i++)
        printf("%9.6lf %9.6lf %9.6lf\r\n", A_1[i * 3], A_1[i * 3 + 1], A_1[i * 3 + 2]);

    free(D);
    free(S);
    free(C);
    free(S11);
    free(S12);
    free(S12t);
    free(S22);
    free(S22_1);
    free(S22a);
    free(S22b);
    free(SS);
    free(E);
    free(U);
    free(SSS);
    free(eigen_real);
    free(eigen_imag);
    free(v1);
    free(v2);
    free(v);
    free(Q);
    free(Q_1);
    free(QB);
    free(SSSS);
    free(eigen_real3);
    free(eigen_imag3);
    free(Dz);
    free(vdz);
    free(SQ);

    return 0;
}
