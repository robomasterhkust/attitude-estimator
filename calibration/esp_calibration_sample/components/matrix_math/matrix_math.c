/**
 * 20170514, Beck Pang
 * build necessary tools for matrix operation
 */

#include "matrix_math.h"

/**
 * @ brief: inverse know square matrix using functions in LAPACK
 * @ dependency:s
 * * LU decomoposition of a general matrix
 * void dgetrf_(int* M, int *N, double* A, int* lda, int* IPIV, int* INFO);
 *
 * * generate inverse of a matrix given its LU decomposition
 * void dgetri_(int* N, double* A, int* lda, int* IPIV, double* WORK, int* lwork, int* INFO);
 */
bool matrix_invert3(float src[3][3], float dst[3][3])
{
    float det = src[0][0] * (src[1][1] * src[2][2] - src[1][2] * src[2][1]) -
                src[0][1] * (src[1][0] * src[2][2] - src[1][2] * src[2][0]) +
                src[0][2] * (src[1][0] * src[2][1] - src[1][1] * src[2][0]);

    if (fabsf(det) < FLT_EPSILON)
    {
        return false;           // Singular matrix
    }

    dst[0][0] = (src[1][1] * src[2][2] - src[1][2] * src[2][1]) / det;
    dst[1][0] = (src[1][2] * src[2][0] - src[1][0] * src[2][2]) / det;
    dst[2][0] = (src[1][0] * src[2][1] - src[1][1] * src[2][0]) / det;
    dst[0][1] = (src[0][2] * src[2][1] - src[0][1] * src[2][2]) / det;
    dst[1][1] = (src[0][0] * src[2][2] - src[0][2] * src[2][0]) / det;
    dst[2][1] = (src[0][1] * src[2][0] - src[0][0] * src[2][1]) / det;
    dst[0][2] = (src[0][1] * src[1][2] - src[0][2] * src[1][1]) / det;
    dst[1][2] = (src[0][2] * src[1][0] - src[0][0] * src[1][2]) / det;
    dst[2][2] = (src[0][0] * src[1][1] - src[0][1] * src[1][0]) / det;

    return true;
}


/**
 * @brief matrix subtraction with known size, no check
 * A = input matrix (m x n)
 * B = input matrix (m x n)
 * m = number of rows in A = number of rows in B
 * n = number of columns in A = number of columns in B
 * C = output matrix = A-B (m x n)
 */
void matrix_subtract(float *A, float *B, int m, int n, float *C)
{
    int i, j;

    for (i = 0; i < m; i++)
    {
        for (j = 0; j < n; j++)
        {
            C[n * i + j] = A[n * i + j] - B[n * i + j];
        }
    }
}


/**
 * @source from MatrixMath.cpp from Arduino
 * @brief for known size matrix multiplication, no check
 * A = input matrix (m x p)
 * B = input matrix (p x n)
 * m = number of rows in A
 * p = number of columns in A = number of rows in B
 * n = number of columns in B
 * C = output matrix = A*B (m x n)
 */
void matrix_multiply(float *A, float *B, int m, int p, int n, float *C)
{
    int i, j, k;

    for (i = 0; i < m; i++)
    {
        for (j = 0; j < n; j++)
        {
            C[n * i + j] = 0;
            for (k = 0; k < p; k++)
            {
                C[n * i + j] = C[n * i + j] + A[p * i + k] * B[n * k + j];
            }
        }
    }
}


/**
 * @brief for Euler angle rotation
 */
void rotx(float angle, float *rotx_mat)
{
    float rotx_temp[9] =            \
    {                               \
        1,          0,           0, \
        0, cos(angle), -sin(angle), \
        0, sin(angle), cos(angle)   \
    };

    for (unsigned short i = 0; i < 9; i++)
    {
        rotx_mat[i] = rotx_temp[i];
    }
}


void roty(float angle, float *roty_mat)
{
    float roty_temp[9] =            \
    {                               \
        cos(angle),  0, sin(angle), \
        0,           1,          0, \
        -sin(angle), 0, cos(angle)  \
    };

    for (unsigned short i = 0; i < 9; i++)
    {
        roty_mat[i] = roty_temp[i];
    }
}


void rotz(float angle, float *rotz_mat)
{
    float rotz_temp[9] =            \
    {                               \
        cos(angle), -sin(angle), 0, \
        sin(angle), cos(angle),  0, \
        0,                    0, 1  \
    };

    for (unsigned short i = 0; i < 9; i++)
    {
        rotz_mat[i] = rotz_temp[i];
    }
}
