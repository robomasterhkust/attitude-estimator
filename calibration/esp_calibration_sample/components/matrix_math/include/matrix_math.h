/**
 * 20170514, Beck Pang
 * build necessary tools for matrix operation
 */

#ifndef _MATRIX_MATH_H_
#define _MATRIX_MATH_H_


#include <stdio.h>
#include <stdbool.h>
#include <math.h>
// TODO: update to "Eigen.h" and C++ concurrency in the future

/**
 * @source NuttX/nuttx/include/nuttx/lib/float.h
 * The difference between 1 and the least value greater than 1 that is
 * representable in the given floating-point type, b1-p.
 */
#define FLT_EPSILON        1.1920929e-07F

/**
 * @brief: inverse know square matrix
 * @return false if the matrix is singular
 */
bool matrix_invert3(float src[3][3], float dst[3][3]);

void matrix_subtract(float *A, float *B, int m, int n, float *C);

void matrix_multiply(float *A, float *B, int m, int p, int n, float *C);

/**
 * @brief for Euler angle rotation
 */
void rotx(float angle, float* rotx_mat);
void roty(float angle, float* roty_mat);
void rotz(float angle, float* rotz_mat);

#endif
