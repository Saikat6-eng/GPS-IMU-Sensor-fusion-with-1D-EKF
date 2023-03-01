#include <matrix.h>
#include <math.h>


float invSqrt(float x)
{
	float y = x;
	long i = *(long*)&y;
		i = 0x5f3759df - (i>>1);
		y = *(float*)&i;
		y = y * (1.5f - ((0.5f * x) * y * y));
	return y;
	// 	float y;
	// 	y=sqrt(x);
	// 	y=1/y;
	// 	return y;
}	


void mul_matrix(double *A, double *B, double *C, int R_a, int C_a, int C_b) 
{
	 unsigned char rowNum, colNum, multIndex;

   for(rowNum = 0; rowNum < R_a; rowNum++) {
        for (colNum = 0; colNum < C_b; colNum++) {
            *(C + rowNum*C_b + colNum) = 0.0;
            for (multIndex = 0; multIndex < C_a; multIndex++) {
                *(C + rowNum*C_b + colNum) = *(C + rowNum*C_b + colNum) +
                    *(A + rowNum*C_a + multIndex) * *(B + colNum + C_b*multIndex);
            }
        }
    }	 
}

void mat_transmul(float *A, float *B, float *C, int R_1,int C_1, int R_2)
{
	  int	i, j, k;

		for (i=0; i<R_1; i++)
		for (j=0; j<R_2; j++)
		{
		C[i*R_2+j] = 0;
		for (k=0; k<C_1; k++) {
			//C[i][j] += A[i][k] * B[j][k];
			C[i*R_2+j] += A[i*C_1+k] * B[j*C_1+k];
		}
	}
}

void mat_vectmul(float *A, float *V, float *C,  int rowsInA, int colsInA)
{
	int rowNum, multIndex;

  for (rowNum = 0; rowNum < rowsInA; rowNum++) 
	{
   *(C + rowNum) = 0.0;
   for (multIndex = 0; multIndex < colsInA; multIndex++) {
      *(C + rowNum) = *(C + rowNum) + *(A + rowNum*colsInA + multIndex) * *(V + multIndex);
    }
  }
}

char mat_inv_3(double* A, double* AInverse)
{
    double temp[3];
    double detInv;

    temp[0] =  *(A + 8) * *(A + 4) - *(A + 7) * *(A + 5);
    temp[1] = -*(A + 8) * *(A + 1) + *(A + 7) * *(A + 2);
    temp[2] =  *(A + 5) * *(A + 1) - *(A + 4) * *(A + 0*3 + 2);
    detInv = 1.0f / ( *(A + 0*3 + 0) * temp[0] + *(A + 1*3 + 0) * temp[1] + *(A + 2*3 + 0) * temp[2] );

    *(AInverse + 0*3 + 0) = temp[0] * detInv;
    *(AInverse + 0*3 + 1) = temp[1] * detInv;
    *(AInverse + 0*3 + 2) = temp[2] * detInv;

    temp[0] = -*(A + 2*3 + 2) * *(A + 1*3 + 0) + *(A + 2*3 + 0) * *(A + 1*3 + 2);
    temp[1] =  *(A + 2*3 + 2) * *(A + 0*3 + 0) - *(A + 2*3 + 0) * *(A + 0*3 + 2);
    temp[2] = -*(A + 1*3 + 2) * *(A + 0*3 + 0) + *(A + 1*3 + 0) * *(A + 0*3 + 2);
    *(AInverse + 1*3 + 0) = temp[0] * detInv;
    *(AInverse + 1*3 + 1) = temp[1] * detInv;
    *(AInverse + 1*3 + 2) = temp[2] * detInv;

    temp[0] =  *(A + 2*3 + 1) * *(A + 1*3 + 0) - *(A + 2*3 + 0) * *(A + 1*3 + 1);
    temp[1] = -*(A + 2*3 + 1) * *(A + 0*3 + 0) + *(A + 2*3 + 0) * *(A + 0*3 + 1);
    temp[2] =  *(A + 1*3 + 1) * *(A + 0*3 + 0) - *(A + 1*3 + 0) * *(A + 0*3 + 1);
    *(AInverse + 2*3 + 0) = temp[0] * detInv;
    *(AInverse + 2*3 + 1) = temp[1] * detInv;
    *(AInverse + 2*3 + 2) = temp[2] * detInv;

    return 1;
}
