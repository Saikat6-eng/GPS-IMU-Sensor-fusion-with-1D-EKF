#ifndef _matrix_h
#define _matrix_h

#define SQUARE(x) ((x)*(x))

float invSqrt(float x);
void mul_matrix(double *A, double *B, double *C, int R_a, int C_a, int C_b);
void mat_transmul(float *A, float *B, float *C, int R_1,int C_1, int R_2);
char mat_inv_3(double* A, double* AInverse);
void mat_vectmul(float *A, float *V, float *C,  int rowsInA, int colsInA);

#endif
