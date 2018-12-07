/**
 *   @file  gtrackMatrixMath.c
 *
 *   @brief
 *      This is a set of matrix functions used by GTRACK Algorithm
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2017 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <math.h>

#include <ti/alg/gtrack/gtrack.h>
#include <ti/alg/gtrack/include/gtrack_int.h>

/**
*  @b Description
*  @n
*		This function is used to multiply two matrices. 
*		Matrices are all real, single precision floating point.
*		Matrices are in row-major order
*
*  @param[in]  rows
*		Outer dimension, number of rows
*  @param[in]  m
*		Inner dimension
*  @param[in]  cols
*		Outer dimension, number of cols
*  @param[in]  A
*		Matrix A
*  @param[in]  B
*		Matrix B
*  @param[in]  C
*		Matrix C(rows,cols) = A(rows,m) X B(m,cols)
*
*  \ingroup GTRACK_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/

void gtrack_matrixMultiply(uint16_t rows, uint16_t m, uint16_t cols, float *A, float *B, float *C)
{
	//C(rows*cols) = A(rows*m)*B(m*cols)
	uint16_t i,j, k;
	for (i = 0; i < rows; i++)
	{
		for (j = 0; j < cols; j++)
		{			
			C[i*cols + j] = 0;
			for (k = 0; k < m; k++)
				C[i*cols+j] += A[i*m+k] * B[k*cols + j];
		}
	}
}

void gtrack_matrixMultiplyM(uint16_t rows, uint16_t mid, uint16_t cols, float *A, float *B, float *C)
{
	//C(rows*cols) = A(rows*mid)*B(mid*cols)
	uint16_t r, rm, c, m, mm;
	for (r = 0, rm = 0; r < rows*cols; r+=cols, rm+=mid) 
	{
		for (c = 0; c < cols; c++)
		{			
			C[r + c] = 0;
			for (m = 0, mm = 0; m < mid; m++, mm+=cols)
				C[r + c] += A[rm + m] * B[c + mm];
		}
	}
}
void gtrack_matrixComputePJT(float *P, float *J, float *PJ)
{
	//PJ[6x3] = P[6x6]xJ[6x3]'
	// We compute PJ' first because it is re-used in Kalman Gain calculations 
	uint16_t i,j, k, n;
	
	//PJ[6x3] = P[6x6] x J[3x6]'
	for (i = 0, k = 0; i < 18; i+= 3, k+=6)
	{
		for (j = 0, n = 0; j < 3; j++, n+=6)
		{
            PJ[i + j] = (P[k + 0] * J[n + 0]) +
						(P[k + 1] * J[n + 1]) +
                        (P[k + 2] * J[n + 2]) +
                        (P[k + 3] * J[n + 3]) +
                        (P[k + 4] * J[n + 4]) +
                        (P[k + 5] * J[n + 5]);	
		}
	}
}
void gtrack_matrixComputeCov(float *J, float *P, float *R, float *PJ, float *C)
{
	//C[3x3] = J[3x6]*(P[6x6]xJ[6x3]')+R[3x3]
	// We compute PJ' first because it is re-used in Kalman Gain calculations 
	uint16_t i,j, k, n;
	
	//PJ[6x3] = P[6x6] x J[3x6]'
	for (i = 0, k = 0; i < 18; i+= 3, k+=6)
	{
		for (j = 0, n = 0; j < 3; j++, n+=6)
		{
            PJ[i + j] = (P[k + 0] * J[n + 0]) +
						(P[k + 1] * J[n + 1]) +
                        (P[k + 2] * J[n + 2]) +
                        (P[k + 3] * J[n + 3]) +
                        (P[k + 4] * J[n + 4]) +
                        (P[k + 5] * J[n + 5]);	
		}
	}

	//JPJ'[3x3] = J[3x6]*PJ[6x3] + R[3x3]
	for (i = 0, k = 0; i < 9; i+= 3, k+=6)
	{
		for (j = 0; j < 3; j++)
		{
            C[i + j] =	(J[k + 0] * PJ[j +  0]) +
						(J[k + 1] * PJ[j +  3]) +
						(J[k + 2] * PJ[j +  6]) +	
						(J[k + 3] * PJ[j +  9]) +
						(J[k + 4] * PJ[j + 12]) +
						(J[k + 5] * PJ[j + 15]) +

						R[i + j];	
		}
	}
}

void gtrack_matrixMultiply66(float *A, float *B, float *C)
{
	uint16_t i,j;
    for (i = 0; i < 36; i += 6) 
	{
        for (j = 0; j < 6; j++) {
            C[i + j] = 	(A[i + 0] * B[j +  0]) +
						(A[i + 1] * B[j +  6]) +
                        (A[i + 2] * B[j + 12]) +
                        (A[i + 3] * B[j + 18]) +
                        (A[i + 4] * B[j + 24]) +
                        (A[i + 5] * B[j + 30]);	
		}
	}
}

void gtrack_matrixMultiply66M(float *A, float *B, float *C)
{
    uint16_t i,j;

    for (i = 0; i < 36; i += 6)
    {
        C[i+0] = A[i+0]*B[0];
        C[i+1] = A[i+1]*B[1];
        C[i+2] = A[i+2]*B[2];
        C[i+3] = A[i+3]*B[3];
        C[i+4] = A[i+4]*B[4];
        C[i+5] = A[i+5]*B[5];

        for (j = 1; j < 6; j++) {
            C[i+0] += A[i+0]*B[j+0];
            C[i+1] += A[i+1]*B[j+1];
            C[i+2] += A[i+2]*B[j+2];
            C[i+3] += A[i+3]*B[j+3];
            C[i+4] += A[i+4]*B[j+4];
            C[i+5] += A[i+5]*B[j+5];
        }
    }
}

void gtrack_matrixMultiply66T(float *A, float *B, float *C)
{
	uint16_t i,j, k;
    for (i = 0; i < 36; i += 6) 
	{
        for (j = 0, k = 0; j < 6; j++, k+=6) {
            C[i + j] = 	(A[i + 0] * B[k + 0]) +
						(A[i + 1] * B[k + 1]) +
                        (A[i + 2] * B[k + 2]) +
                        (A[i + 3] * B[k + 3]) +
                        (A[i + 4] * B[k + 4]) +
                        (A[i + 5] * B[k + 5]);	
		}
	}
}

/**
*  @b Description
*  @n
*		This function is used to multiply two matrices. Second Matrix is getting transposed first
*		Matrices are all real, single precision floating point.
*		Matrices are in row-major order
*
*  @param[in]  rows
*		Outer dimension, number of rows
*  @param[in]  m
*		Inner dimension
*  @param[in]  cols
*		Outer dimension, number of cols
*  @param[in]  A
*		Matrix A
*  @param[in]  B
*		Matrix B
*  @param[in]  C
*		Matrix C(rows,cols) = A(rows,m) X B(cols,m)T
*
*  \ingroup GTRACK_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void gtrack_matrixTransposeMultiply(uint16_t rows, uint16_t m, uint16_t cols, float *A, float *B, float *C)
{
	//C(rows*cols) = A(rows*m)*B(cols*m)T
	uint16_t i,j, k;
	for (i = 0; i < rows; i++)
	{
		for (j = 0; j < cols; j++)
		{			
			C[i*cols + j] = 0;
			for (k = 0; k < m; k++)
				C[i*cols+j] += A[i*m+k] * B[k + j*m];
		}
	}
}

/**
*  @b Description
*  @n
*		This function is used to multiply matrix by a scaller.
*		Matrices are all real, single precision floating point.
*		Matrices are in row-major order
*
*  @param[in]  rows
*		Number of rows
*  @param[in]  cols
*		Number of cols
*  @param[in]  A
*		Matrix A
*  @param[in]  C
*		Scaller C
*  @param[in]  B
*		Matrix B(rows,cols) = A(rows,cols) X C
*
*  \ingroup GTRACK_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void gtrack_matrixScallerMultiply(uint16_t rows, uint16_t cols, float *A, float C, float *B)
{
	//B(rows*cols) = A(rows*cols)*C
	uint16_t i;
	for (i = 0; i < rows*cols; i++)
	{
		B[i] = A[i] * C;
	}
}

/**
*  @b Description
*  @n
*		This function is used to add two matrices.
*		Matrices are all real, single precision floating point.
*		Matrices are in row-major order
*
*  @param[in]  rows
*		Number of rows
*  @param[in]  cols
*		Number of cols
*  @param[in]  A
*		Matrix A
*  @param[in]  B
*		Matrix B
*  @param[in]  C
*		Matrix C(rows,cols) = A(rows,cols) + B(rows,cols)
*
*  \ingroup GTRACK_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void gtrack_matrixAdd(uint16_t rows, uint16_t cols, float *A, float *B, float *C)
{
	//C(rows*cols) = A(rows*cols) + B(rows*cols)
	uint16_t i;
	for (i = 0; i < rows*cols; i++)
	{
		C[i] = A[i] + B[i];
	}
}

/**
*  @b Description
*  @n
*		This function is used to subtract two matrices.
*		Matrices are all real, single precision floating point.
*		Matrices are in row-major order
*
*  @param[in]  rows
*		Number of rows
*  @param[in]  cols
*		Number of cols
*  @param[in]  A
*		Matrix A
*  @param[in]  B
*		Matrix B
*  @param[in]  C
*		Matrix C(rows,cols) = A(rows,cols) - B(rows,cols)
*
*  \ingroup GTRACK_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void gtrack_matrixSub(uint16_t rows, uint16_t cols, float *A, float *B, float *C)
{
	//C(rows*cols) = A(rows*cols) - B(rows*cols)
	uint16_t i;
	for (i = 0; i < rows*cols; i++)
	{
		C[i] = A[i] - B[i];
	}
}

/**
*  @b Description
*  @n
*		This function is used to force matrix symmetry by averaging off-diagonal elements
*		Matrices are squared, real, single precision floating point.
*		Matrices are in row-major order
*
*  @param[in]  m (m=rows=cols)
*		Number of rows and cols
*  @param[in]  A
*		Matrix A
*  @param[in]  B
*		Matrix B
*
*  \ingroup GTRACK_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void gtrack_matrixMakeSymmetrical(uint16_t m, float *A, float *B)
{
	// Make square matrix symmetrical by averaging off-diagonal elements
	uint16_t i,j;
	for (i = 0; i < m-1; i++)
	{
		B[i*m + i] = A[i*m + i];
		for (j = i+1; j < m; j++)
			B[i*m+j] = B[j*m+i] = 0.5f*(A[i*m+j]+A[j*m+i]);
	}
	B[i*m + i] = A[i*m + i];
}

/**
*  @b Description
*  @n
*		This function performs cholesky decomposition of 3x3 matrix.
*		Matrix are squared, real, single precision floating point.
*		Matrix are in row-major order
*
*  @param[in]  A
*		Matrix A
*  @param[in]  G
*		Matrix G = cholseky(A);
*
*  \ingroup GTRACK_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void gtrack_matrixCholesky3(float *A, float *G)
{
	float v[3] = {0, 0, 0};
	float temp;
    uint16_t i, j, k;
	uint16_t dim = 3;

    for (j = 0; j < dim; j++)
    {
		//v(j:n,1) = A(j:n,j);
		for (i = j; i < dim; i++)
		    v[i] = A[i * dim + j];
        
		for (k = 0; k < j; k++)
        {
			//v(j:n,1) = v(j:n,1) - G(j,k)*G(j:n,k);
			for (i = j; i < dim; i++)
				v[i] = v[i] - G[j*dim + k] * G[i*dim + k];
		}
    
        //G(j:n,j) = v(j:n,1)/sqrt(v(j));
		temp = 1.0f/sqrtf(v[j]);
		for (i = j; i < dim; i++)
			G[i*dim + j] = v[i]*temp;
	}
	G[1] = G[2] = G[5] = 0;
}
/**
*  @b Description
*  @n
*		This function computes the determinant of 3x3 matrix.
*		Matrix is real, single precision floating point.
*		Matrix is in row-major order
*
*  @param[in]  A
*		Matrix A
*  @param[in]  det
*		det = det(A);
*
*  \ingroup GTRACK_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void gtrack_matrixDet3(float *A, float *det)
{
	*det = A[0] * (A[4]*A[8] - A[7]*A[5]) -
		A[1] * (A[3]*A[8] - A[5]*A[6]) +
		A[2] * (A[3]*A[7] - A[4]*A[6]);
}

/**
*  @b Description
*  @n
*		This function computes the inverse of 3x3 matrix.
*		Matrix is real, single precision floating point.
*		Matrix is in row-major order
*
*  @param[in]  A
*		Matrix A
*  @param[in]  inv
*		inv = inverse(A);
*
*  \ingroup GTRACK_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/

void gtrack_matrixInv3(float *A, float *inv)
{
	float det;
	float invdet;

	det = A[0] * (A[4]*A[8] - A[7]*A[5]) -
		A[1] * (A[3]*A[8] - A[5]*A[6]) +
		A[2] * (A[3]*A[7] - A[4]*A[6]);

	invdet = 1 / det;

	inv[0] = (A[4] * A[8] - A[7] * A[5]) * invdet;
	inv[1] = (A[2] * A[7] - A[1] * A[8]) * invdet;
	inv[2] = (A[1] * A[5] - A[2] * A[4]) * invdet;
	inv[3] = (A[5] * A[6] - A[3] * A[8]) * invdet;
	inv[4] = (A[0] * A[8] - A[2] * A[6]) * invdet;
	inv[5] = (A[3] * A[2] - A[0] * A[5]) * invdet;
	inv[6] = (A[3] * A[7] - A[6] * A[4]) * invdet;
	inv[7] = (A[6] * A[1] - A[0] * A[7]) * invdet;
	inv[8] = (A[0] * A[4] - A[3] * A[1]) * invdet;
}

void gtrack_matrixPrint(uint16_t rows, uint16_t cols, float *A)
{
	uint16_t i,j;
	for (i = 0; i < rows; i++)
	{
		for (j = 0; j < cols-1; j++)
			gtrack_log(GTRACK_VERBOSE_DEBUG, "%6.3f\t",A[i*cols +j]); 
		gtrack_log(GTRACK_VERBOSE_DEBUG, "%6.3f\n",A[i*cols +j]);
	}
	gtrack_log(GTRACK_VERBOSE_DEBUG, "\n");
}
void gtrack_matrixPrint2(uint16_t rows, uint16_t cols, float *A, float *B)
{
	// print 2 matrices side by side
	uint16_t i,j;
	for (i = 0; i < rows; i++)
	{
		for (j = 0; j < cols-1; j++)
			gtrack_log(GTRACK_VERBOSE_DEBUG, "%6.2f\t",A[i*cols +j]); 
		gtrack_log(GTRACK_VERBOSE_DEBUG, "%6.2f\t\t",A[i*cols +j]);

		for (j = 0; j < cols-1; j++)
			gtrack_log(GTRACK_VERBOSE_DEBUG, "%6.2f\t",B[i*cols +j]); 
		gtrack_log(GTRACK_VERBOSE_DEBUG, "%6.2f\n",B[i*cols +j]);
	}
	gtrack_log(GTRACK_VERBOSE_DEBUG, "\n");
}
