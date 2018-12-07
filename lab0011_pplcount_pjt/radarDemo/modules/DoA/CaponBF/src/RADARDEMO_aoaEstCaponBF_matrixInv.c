/**
 *  \file   RADARDEMO_aoaEstCaponBF_matrixInv.c
 *
 *   \brief   Matrix inversion functions used in Capon BF.
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
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
 *
*/
#include "RADARDEMO_aoaEstCaponBF_priv.h"

#define DEBUGOUTPUT (0)


/** 
 *  \fn     void MATRIX_single8x8MatInv(
 *                            IN      Cplx32  * restrict Input,
 *                            IN      int32_t * RESTRICT scratch,
 *                            OUT     cplx32_t  * Output);
 *
 *  \brief   Single 8x8 matrix (complex positive semi-definitive Hermitian) inversion using block wise method.
 * 
 *  \param[in]    Input
 *              Input 8x8 matrix that needs to be inversed, stored sequentially 
 *              in format (1,1), (1,2).... (1,8), (2,1), (2,2)...(8,8). 
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[out]    scratch
 *              Input pointer to the scratch memory. Must be of size 7 * 2 * 16 = 288 32-bit words.
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[out]   Output
 *              Output 8x8 matrix. Stored sequentially as the input.
 *              Must be aligned to 8-byte boundary.
 *
 *  \pre     Input matrix must be complex positive semi-definitive Hermitian matrix
 *
 *  \post  
 * 
 *  \sa   
 * 
 */

void MATRIX_single8x8MatInv (  
                             IN      cplxf_t * RESTRICT Input,
                             IN      int32_t * RESTRICT scratch,
                             OUT     cplxf_t  * output)
{
	cplxf_t    * RESTRICT A;
	cplxf_t    * RESTRICT B;
	cplxf_t    * RESTRICT C;
	cplxf_t    * RESTRICT D;
	cplxf_t    * RESTRICT T;
	cplxf_t    * RESTRICT invT;
	cplxf_t    * RESTRICT invA;
	int32_t		jj, offset;
	int32_t		scratchIndx;
	__float2_t		dtemp1;


	scratchIndx		=	0;
	A				=	(cplxf_t *) &scratch[scratchIndx];
	scratchIndx		+=	16 * 2;
	B				=	(cplxf_t *) &scratch[scratchIndx];
	scratchIndx		+=	16 * 2;
	C				=	(cplxf_t *) &scratch[scratchIndx];
	scratchIndx		+=	16 * 2;
	D				=	(cplxf_t *) &scratch[scratchIndx];
	scratchIndx		+=	16 * 2;
	T				=	(cplxf_t *) &scratch[scratchIndx];
	scratchIndx		+=	16 * 2;
	invA			=	(cplxf_t *) &scratch[scratchIndx];
	scratchIndx		+=	16 * 2;
	invT			=	(cplxf_t *) &scratch[scratchIndx];
	scratchIndx		+=	16 * 2;


	/*Copy A */
	_amem8_f2(&A[0])	=	_amem8_f2(&Input[0]);
	dtemp1				=	_amem8_f2(&Input[1]);
	_amem8_f2(&A[1])	=	dtemp1;
	_amem8_f2(&A[4])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	dtemp1				=	_amem8_f2(&Input[2]);
	_amem8_f2(&A[2])	=	dtemp1;
	_amem8_f2(&A[8])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	dtemp1				=	_amem8_f2(&Input[3]);
	_amem8_f2(&A[3])	=	dtemp1;
	_amem8_f2(&A[12])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	_amem8_f2(&A[5])	=	_amem8_f2(&Input[8]);
	dtemp1				=	_amem8_f2(&Input[9]);
	_amem8_f2(&A[6])	=	dtemp1;
	_amem8_f2(&A[9])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	dtemp1				=	_amem8_f2(&Input[10]);
	_amem8_f2(&A[7])	=	dtemp1;
	_amem8_f2(&A[13])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	_amem8_f2(&A[10])	=	_amem8_f2(&Input[15]);
	dtemp1				=	_amem8_f2(&Input[16]);
	_amem8_f2(&A[11])	=	dtemp1;
	_amem8_f2(&A[14])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	_amem8_f2(&A[15])	=	_amem8_f2(&Input[21]);

	/*copy C */
	offset = 4;
	for ( jj = 0; jj < 4; jj++ )
	{
		dtemp1				=	_amem8_f2(&Input[jj + offset]);
		_amem8_f2(&C[4 * jj])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	}
	offset = 11;
	for ( jj = 0; jj < 4; jj++ )
	{
		dtemp1				=	_amem8_f2(&Input[jj + offset]);
		_amem8_f2(&C[4 * jj + 1])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	}
	offset = 17;
	for ( jj = 0; jj < 4; jj++ )
	{
		dtemp1				=	_amem8_f2(&Input[jj + offset]);
		_amem8_f2(&C[4 * jj + 2])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	}
	offset = 22;
	for ( jj = 0; jj < 4; jj++ )
	{
		dtemp1				=	_amem8_f2(&Input[jj + offset]);
		_amem8_f2(&C[4 * jj + 3])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	}

	/*Copy D */
	_amem8_f2(&D[0])	=	_amem8_f2(&Input[26]);
	dtemp1				=	_amem8_f2(&Input[27]);
	_amem8_f2(&D[1])	=	dtemp1;
	_amem8_f2(&D[4])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	dtemp1				=	_amem8_f2(&Input[28]);
	_amem8_f2(&D[2])	=	dtemp1;
	_amem8_f2(&D[8])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	dtemp1				=	_amem8_f2(&Input[29]);
	_amem8_f2(&D[3])	=	dtemp1;
	_amem8_f2(&D[12])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	_amem8_f2(&D[5])	=	_amem8_f2(&Input[30]);
	dtemp1				=	_amem8_f2(&Input[31]);
	_amem8_f2(&D[6])	=	dtemp1;
	_amem8_f2(&D[9])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	dtemp1				=	_amem8_f2(&Input[32]);
	_amem8_f2(&D[7])	=	dtemp1;
	_amem8_f2(&D[13])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	_amem8_f2(&D[10])	=	_amem8_f2(&Input[33]);
	dtemp1				=	_amem8_f2(&Input[34]);
	_amem8_f2(&D[11])	=	dtemp1;
	_amem8_f2(&D[14])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	_amem8_f2(&D[15])	=	_amem8_f2(&Input[35]);

#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;

		/* A matrix */ 
		fid			=	fopen("input_A.dat", "w");
		for (jj = 0; jj < 16; jj++)
			fprintf(fid, "\t%f\t\t%f\n", A[jj].real, A[jj].imag);
		fclose(fid);

		/* B matrix */ 
		fid			=	fopen("input_B.dat", "w");
		for (jj = 0; jj < 16; jj++)
			fprintf(fid, "\t%f\t\t%f\n", B[jj].real, B[jj].imag);
		fclose(fid);


		/* C matrix */ 
		fid			=	fopen("input_C.dat", "w");
		for (jj = 0; jj < 16; jj++)
			fprintf(fid, "\t%f\t\t%f\n", C[jj].real, C[jj].imag);
		fclose(fid);

		/* D matrix */ 
		fid			=	fopen("input_D.dat", "w");
		for (jj = 0; jj < 16; jj++)
			fprintf(fid, "\t%f\t\t%f\n", D[jj].real, D[jj].imag);
		fclose(fid);
	}
#endif


	/* calculate inv(A)*/
	MATRIX_4x4_BWInversionfp (A, invA);

#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;

		/* A matrix */ 
		fid			=	fopen("invA.dat", "w");
		for (jj = 0; jj < 16; jj++)
			fprintf(fid, "\t%e\t\t%e\n", invA[jj].real, invA[jj].imag);
		fclose(fid);

	}
#endif


	/* calculate tempC = C*inv(A) */
	//tsc_in			=	clock();
	MATRIX_Mult4x4fp(C, invA, B);
	//tsc_out			=	clock();
	//CycleCounts		=	tsc_out - tsc_in;
#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;

		/* A matrix */ 
		fid			=	fopen("CinvA.dat", "w");
		for (jj = 0; jj < 16; jj++)
			fprintf(fid, "\t%e\t\t%e\n", B[jj].real, B[jj].imag);
		fclose(fid);

	}
#endif

	/* calculate T = D - C*inv(A)*B */
	//results			=	_cmpysp(_amem8_f2(&C[0]), _amem8_f2(&B[0]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&C[0]), _amem8_f2(&B[0]));

	//results			=	_cmpysp(_amem8_f2(&C[1]), _amem8_f2(&B[1]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[1]), _amem8_f2(&B[1])));

	//results			=	_cmpysp(_amem8_f2(&C[2]), _amem8_f2(&B[2]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[2]), _amem8_f2(&B[2])));

	//results			=	_cmpysp(_amem8_f2(&C[3]), _amem8_f2(&B[3]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[3]), _amem8_f2(&B[3])));

	dtemp1			=	_dsubsp(_amem8_f2(&D[0]), dtemp1);
	_amem8_f2(&T[0])	=	dtemp1;

	//results			=	_cmpysp(_amem8_f2(&C[4]), _amem8_f2(&B[0]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&C[4]), _amem8_f2(&B[0]));

	//results			=	_cmpysp(_amem8_f2(&C[5]), _amem8_f2(&B[1]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[5]), _amem8_f2(&B[1])));

	//results			=	_cmpysp(_amem8_f2(&C[6]), _amem8_f2(&B[2]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[6]), _amem8_f2(&B[2])));

	//results			=	_cmpysp(_amem8_f2(&C[7]), _amem8_f2(&B[3]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[7]), _amem8_f2(&B[3])));

	dtemp1			=	_dsubsp(_amem8_f2(&D[1]), dtemp1);
	_amem8_f2(&T[1])	=	dtemp1;
	_amem8_f2(&T[4])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	//_amem8_f2(&T[4])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
	
	//results			=	_cmpysp(_amem8_f2(&C[4]), _amem8_f2(&B[4]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&C[4]), _amem8_f2(&B[4]));

	//results			=	_cmpysp(_amem8_f2(&C[5]), _amem8_f2(&B[5]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[5]), _amem8_f2(&B[5])));

	//results			=	_cmpysp(_amem8_f2(&C[6]), _amem8_f2(&B[6]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[6]), _amem8_f2(&B[6])));

	//results			=	_cmpysp(_amem8_f2(&C[7]), _amem8_f2(&B[7]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[7]), _amem8_f2(&B[7])));

	dtemp1			=	_dsubsp(_amem8_f2(&D[5]), dtemp1);
	_amem8_f2(&T[5])	=	dtemp1;
	
	//results			=	_cmpysp(_amem8_f2(&C[8]), _amem8_f2(&B[0]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&C[8]), _amem8_f2(&B[0]));

	//results			=	_cmpysp(_amem8_f2(&C[9]), _amem8_f2(&B[1]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[9]), _amem8_f2(&B[1])));

	//results			=	_cmpysp(_amem8_f2(&C[10]), _amem8_f2(&B[2]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[10]), _amem8_f2(&B[2])));

	//results			=	_cmpysp(_amem8_f2(&C[11]), _amem8_f2(&B[3]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[11]), _amem8_f2(&B[3])));

	dtemp1			=	_dsubsp(_amem8_f2(&D[2]), dtemp1);
	_amem8_f2(&T[2])	=	dtemp1;
	_amem8_f2(&T[8])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	//_amem8_f2(&T[8])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
	
	//results			=	_cmpysp(_amem8_f2(&C[8]), _amem8_f2(&B[4]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&C[8]), _amem8_f2(&B[4]));

	//results			=	_cmpysp(_amem8_f2(&C[9]), _amem8_f2(&B[5]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[9]), _amem8_f2(&B[5])));

	//results			=	_cmpysp(_amem8_f2(&C[10]), _amem8_f2(&B[6]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[10]), _amem8_f2(&B[6])));

	//results			=	_cmpysp(_amem8_f2(&C[11]), _amem8_f2(&B[7]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[11]), _amem8_f2(&B[7])));

	dtemp1			=	_dsubsp(_amem8_f2(&D[6]), dtemp1);
	_amem8_f2(&T[6])	=	dtemp1;
	_amem8_f2(&T[9])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	//_amem8_f2(&T[9])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
	
	//results			=	_cmpysp(_amem8_f2(&C[8]), _amem8_f2(&B[8]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&C[8]), _amem8_f2(&B[8]));

	//results			=	_cmpysp(_amem8_f2(&C[9]), _amem8_f2(&B[9]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[9]), _amem8_f2(&B[9])));

	//results			=	_cmpysp(_amem8_f2(&C[10]), _amem8_f2(&B[10]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[10]), _amem8_f2(&B[10])));

	//results			=	_cmpysp(_amem8_f2(&C[11]), _amem8_f2(&B[11]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[11]), _amem8_f2(&B[11])));

	dtemp1			=	_dsubsp(_amem8_f2(&D[10]), dtemp1);
	_amem8_f2(&T[10])	=	dtemp1;

	//results			=	_cmpysp(_amem8_f2(&C[12]), _amem8_f2(&B[0]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&C[12]), _amem8_f2(&B[0]));

	//results			=	_cmpysp(_amem8_f2(&C[13]), _amem8_f2(&B[1]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[13]), _amem8_f2(&B[1])));

	//results			=	_cmpysp(_amem8_f2(&C[14]), _amem8_f2(&B[2]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[14]), _amem8_f2(&B[2])));

	//results			=	_cmpysp(_amem8_f2(&C[15]), _amem8_f2(&B[3]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[15]), _amem8_f2(&B[3])));

	dtemp1			=	_dsubsp(_amem8_f2(&D[3]), dtemp1);
	_amem8_f2(&T[3])	=	dtemp1;
	_amem8_f2(&T[12])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	//_amem8_f2(&T[12])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
	
	//results			=	_cmpysp(_amem8_f2(&C[12]), _amem8_f2(&B[4]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&C[12]), _amem8_f2(&B[4]));

	//results			=	_cmpysp(_amem8_f2(&C[13]), _amem8_f2(&B[5]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[13]), _amem8_f2(&B[5])));

	//results			=	_cmpysp(_amem8_f2(&C[14]), _amem8_f2(&B[6]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[14]), _amem8_f2(&B[6])));

	//results			=	_cmpysp(_amem8_f2(&C[15]), _amem8_f2(&B[7]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[15]), _amem8_f2(&B[7])));

	dtemp1			=	_dsubsp(_amem8_f2(&D[7]), dtemp1);
	_amem8_f2(&T[7])	=	dtemp1;
	_amem8_f2(&T[13])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	//_amem8_f2(&T[13])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
	
	//results			=	_cmpysp(_amem8_f2(&C[12]), _amem8_f2(&B[8]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&C[12]), _amem8_f2(&B[8]));

	//results			=	_cmpysp(_amem8_f2(&C[13]), _amem8_f2(&B[9]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[13]), _amem8_f2(&B[9])));

	//results			=	_cmpysp(_amem8_f2(&C[14]), _amem8_f2(&B[10]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[14]), _amem8_f2(&B[10])));

	//results			=	_cmpysp(_amem8_f2(&C[15]), _amem8_f2(&B[11]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[15]), _amem8_f2(&B[11])));

	dtemp1			=	_dsubsp(_amem8_f2(&D[11]), dtemp1);
	_amem8_f2(&T[11])	=	dtemp1;
	_amem8_f2(&T[14])	=	_ftof2(_hif2(dtemp1), -_lof2(dtemp1));
	//_amem8_f2(&T[14])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
	
	//results			=	_cmpysp(_amem8_f2(&C[12]), _amem8_f2(&B[12]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&C[12]), _amem8_f2(&B[12]));

	//results			=	_cmpysp(_amem8_f2(&C[13]), _amem8_f2(&B[13]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[13]), _amem8_f2(&B[13])));

	//results			=	_cmpysp(_amem8_f2(&C[14]), _amem8_f2(&B[14]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[14]), _amem8_f2(&B[14])));

	//results			=	_cmpysp(_amem8_f2(&C[15]), _amem8_f2(&B[15]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[15]), _amem8_f2(&B[15])));

	dtemp1			=	_dsubsp(_amem8_f2(&D[15]), dtemp1);
	_amem8_f2(&T[15])	=	dtemp1;

/*	MATRIX_Mult4x4fp(tempC, B, C);

	for ( jj = 0; jj < 16; jj++ )
	{
		_amem8_f2(&T[jj])		=	_dsubsp(_amem8_f2(&D[jj]), _amem8_f2(&C[jj]));
	}
*/
#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;

		/* A matrix */ 
		fid			=	fopen("Tmat.dat", "w");
		for (jj = 0; jj < 16; jj++)
			fprintf(fid, "\t%e\t\t%e\n", T[jj].real, T[jj].imag);
		fclose(fid);

	}
#endif

	/* Calculate T = inv(T) */
	MATRIX_4x4_BWInversionfp (T, invT);
#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;

		/* A matrix */ 
		fid			=	fopen("invT.dat", "w");
		for (jj = 0; jj < 16; jj++)
			fprintf(fid, "\t%e\t\t%e\n", invT[jj].real, invT[jj].imag);
		fclose(fid);

	}
#endif


	/* output SE 4x4 matrix */
	_amem8_f2(&output[26])	=	_amem8_f2(&invT[0]);
	_amem8_f2(&output[27])	=	_amem8_f2(&invT[1]);
	_amem8_f2(&output[28])	=	_amem8_f2(&invT[2]);
	_amem8_f2(&output[29])	=	_amem8_f2(&invT[3]);
	_amem8_f2(&output[30])	=	_amem8_f2(&invT[5]);
	_amem8_f2(&output[31])	=	_amem8_f2(&invT[6]);
	_amem8_f2(&output[32])	=	_amem8_f2(&invT[7]);
	_amem8_f2(&output[33])	=	_amem8_f2(&invT[10]);
	_amem8_f2(&output[34])	=	_amem8_f2(&invT[11]);
	_amem8_f2(&output[35])	=	_amem8_f2(&invT[15]);

	/* output SW and NE 4x4 matrix = - invA*B*invT */
	MATRIX_Mult4x4fp(invT, B, C);
	jj						=	0;
	dtemp1					=	_amem8_f2(&C[jj * 4 + 0]);
	_amem8_f2(&output[4])	=	_ftof2(-_hif2(dtemp1), _lof2(dtemp1));
	dtemp1					=	_amem8_f2(&C[jj * 4 + 1]);
	_amem8_f2(&output[11])	=	_ftof2(-_hif2(dtemp1), _lof2(dtemp1));
	dtemp1					=	_amem8_f2(&C[jj * 4 + 2]);
	_amem8_f2(&output[17])	=	_ftof2(-_hif2(dtemp1), _lof2(dtemp1));
	dtemp1					=	_amem8_f2(&C[jj * 4 + 3]);
	_amem8_f2(&output[22])	=	_ftof2(-_hif2(dtemp1), _lof2(dtemp1));
	jj						=	1;
	dtemp1					=	_amem8_f2(&C[jj * 4 + 0]);
	_amem8_f2(&output[5])	=	_ftof2(-_hif2(dtemp1), _lof2(dtemp1));
	dtemp1					=	_amem8_f2(&C[jj * 4 + 1]);
	_amem8_f2(&output[12])	=	_ftof2(-_hif2(dtemp1), _lof2(dtemp1));
	dtemp1					=	_amem8_f2(&C[jj * 4 + 2]);
	_amem8_f2(&output[18])	=	_ftof2(-_hif2(dtemp1), _lof2(dtemp1));
	dtemp1					=	_amem8_f2(&C[jj * 4 + 3]);
	_amem8_f2(&output[23])	=	_ftof2(-_hif2(dtemp1), _lof2(dtemp1));
	jj						=	2;
	dtemp1					=	_amem8_f2(&C[jj * 4 + 0]);
	_amem8_f2(&output[6])	=	_ftof2(-_hif2(dtemp1), _lof2(dtemp1));
	dtemp1					=	_amem8_f2(&C[jj * 4 + 1]);
	_amem8_f2(&output[13])	=	_ftof2(-_hif2(dtemp1), _lof2(dtemp1));
	dtemp1					=	_amem8_f2(&C[jj * 4 + 2]);
	_amem8_f2(&output[19])	=	_ftof2(-_hif2(dtemp1), _lof2(dtemp1));
	dtemp1					=	_amem8_f2(&C[jj * 4 + 3]);
	_amem8_f2(&output[24])	=	_ftof2(-_hif2(dtemp1), _lof2(dtemp1));
	jj						=	3;
	dtemp1					=	_amem8_f2(&C[jj * 4 + 0]);
	_amem8_f2(&output[7])	=	_ftof2(-_hif2(dtemp1), _lof2(dtemp1));
	dtemp1					=	_amem8_f2(&C[jj * 4 + 1]);
	_amem8_f2(&output[14])	=	_ftof2(-_hif2(dtemp1), _lof2(dtemp1));
	dtemp1					=	_amem8_f2(&C[jj * 4 + 2]);
	_amem8_f2(&output[20])	=	_ftof2(-_hif2(dtemp1), _lof2(dtemp1));
	dtemp1					=	_amem8_f2(&C[jj * 4 + 3]);
	_amem8_f2(&output[25])	=	_ftof2(-_hif2(dtemp1), _lof2(dtemp1));


#if 0
	zeros		=	_ftof2(0.f, 0.f);
	for ( jj = 0; jj < 4; jj++ )
	{
		dtemp1								=	_dsubsp(zeros, _amem8_f2(&C[jj * 4 + 0]));
		_amem8_f2(&output[(jj + 4) * 8 + 0])	=	dtemp1;
		_amem8_f2(&output[jj + 4])		=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);

		dtemp1								=	_dsubsp(zeros, _amem8_f2(&C[jj * 4 + 1]));
		_amem8_f2(&output[(jj + 4) * 8 + 1])	=	dtemp1;
		_amem8_f2(&output[jj + 12])		=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);

		dtemp1								=	_dsubsp(zeros, _amem8_f2(&C[jj * 4 + 2]));
		_amem8_f2(&output[(jj + 4) * 8 + 2])	=	dtemp1;
		_amem8_f2(&output[jj + 20])		=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);

		dtemp1								=	_dsubsp(zeros, _amem8_f2(&C[jj * 4 + 3]));
		_amem8_f2(&output[(jj + 4) * 8 + 3])	=	dtemp1;
		_amem8_f2(&output[jj + 28])		=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
	}
#endif

	/* output NW 4x4 matrix = invA + invA*B*invT*C*invA */
	/* output NW 4x4 matrix = A + conj(B)*C */
	//results			=	_cmpysp(_amem8_f2(&B[0]), _amem8_f2(&C[0]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&B[0]), _amem8_f2(&C[0]));

	//results			=	_cmpysp(_amem8_f2(&B[4]), _amem8_f2(&C[4]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[4]), _amem8_f2(&C[4])));

	//results			=	_cmpysp(_amem8_f2(&B[8]), _amem8_f2(&C[8]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[8]), _amem8_f2(&C[8])));

	//results			=	_cmpysp(_amem8_f2(&B[12]), _amem8_f2(&C[12]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[12]), _amem8_f2(&C[12])));

	dtemp1			=	_daddsp(_amem8_f2(&invA[0]), dtemp1);
	_amem8_f2(&output[0])	=	dtemp1;

	//results			=	_cmpysp(_amem8_f2(&B[0]), _amem8_f2(&C[1]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&B[0]), _amem8_f2(&C[1]));

	//results			=	_cmpysp(_amem8_f2(&B[4]), _amem8_f2(&C[5]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[4]), _amem8_f2(&C[5])));

	//results			=	_cmpysp(_amem8_f2(&B[8]), _amem8_f2(&C[9]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[8]), _amem8_f2(&C[9])));

	//results			=	_cmpysp(_amem8_f2(&B[12]), _amem8_f2(&C[13]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[12]), _amem8_f2(&C[13])));

	dtemp1			=	_daddsp(_amem8_f2(&invA[1]), dtemp1);
	_amem8_f2(&output[1])	=	dtemp1;
	//_amem8_f2(&output[8])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
	
	//results			=	_cmpysp(_amem8_f2(&B[1]), _amem8_f2(&C[1]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&B[1]), _amem8_f2(&C[1]));

	//results			=	_cmpysp(_amem8_f2(&B[5]), _amem8_f2(&C[5]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[5]), _amem8_f2(&C[5])));

	//results			=	_cmpysp(_amem8_f2(&B[9]), _amem8_f2(&C[9]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[9]), _amem8_f2(&C[9])));

	//results			=	_cmpysp(_amem8_f2(&B[13]), _amem8_f2(&C[13]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[13]), _amem8_f2(&C[13])));

	dtemp1			=	_daddsp(_amem8_f2(&invA[5]), dtemp1);
	//_amem8_f2(&output[9])	=	dtemp1;
	_amem8_f2(&output[8])	=	dtemp1;
	
	//results			=	_cmpysp(_amem8_f2(&B[0]), _amem8_f2(&C[2]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&B[0]), _amem8_f2(&C[2]));

	//results			=	_cmpysp(_amem8_f2(&B[4]), _amem8_f2(&C[6]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[4]), _amem8_f2(&C[6])));

	//results			=	_cmpysp(_amem8_f2(&B[8]), _amem8_f2(&C[10]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[8]), _amem8_f2(&C[10])));

	//results			=	_cmpysp(_amem8_f2(&B[12]), _amem8_f2(&C[14]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[12]), _amem8_f2(&C[14])));

	dtemp1			=	_daddsp(_amem8_f2(&invA[2]), dtemp1);
	_amem8_f2(&output[2])	=	dtemp1;
	//_amem8_f2(&output[16])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
	
	//results			=	_cmpysp(_amem8_f2(&B[1]), _amem8_f2(&C[2]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&B[1]), _amem8_f2(&C[2]));

	//results			=	_cmpysp(_amem8_f2(&B[5]), _amem8_f2(&C[6]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[5]), _amem8_f2(&C[6])));

	//results			=	_cmpysp(_amem8_f2(&B[9]), _amem8_f2(&C[10]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[9]), _amem8_f2(&C[10])));

	//results			=	_cmpysp(_amem8_f2(&B[13]), _amem8_f2(&C[14]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[13]), _amem8_f2(&C[14])));

	dtemp1			=	_daddsp(_amem8_f2(&invA[6]), dtemp1);
	_amem8_f2(&output[9])	=	dtemp1;
	//_amem8_f2(&output[10])	=	dtemp1;
	//_amem8_f2(&output[17])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
	
	//results			=	_cmpysp(_amem8_f2(&B[2]), _amem8_f2(&C[2]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&B[2]), _amem8_f2(&C[2]));

	//results			=	_cmpysp(_amem8_f2(&B[6]), _amem8_f2(&C[6]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[6]), _amem8_f2(&C[6])));

	//results			=	_cmpysp(_amem8_f2(&B[10]), _amem8_f2(&C[10]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[10]), _amem8_f2(&C[10])));

	//results			=	_cmpysp(_amem8_f2(&B[14]), _amem8_f2(&C[14]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[14]), _amem8_f2(&C[14])));

	dtemp1			=	_daddsp(_amem8_f2(&invA[10]), dtemp1);
	_amem8_f2(&output[15])	=	dtemp1;
	//_amem8_f2(&output[18])	=	dtemp1;

	//results			=	_cmpysp(_amem8_f2(&B[0]), _amem8_f2(&C[3]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&B[0]), _amem8_f2(&C[3]));

	//results			=	_cmpysp(_amem8_f2(&B[4]), _amem8_f2(&C[7]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[4]), _amem8_f2(&C[7])));

	//results			=	_cmpysp(_amem8_f2(&B[8]), _amem8_f2(&C[11]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[8]), _amem8_f2(&C[11])));

	//results			=	_cmpysp(_amem8_f2(&B[12]), _amem8_f2(&C[15]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[12]), _amem8_f2(&C[15])));

	dtemp1			=	_daddsp(_amem8_f2(&invA[3]), dtemp1);
	_amem8_f2(&output[3])	=	dtemp1;
	//_amem8_f2(&output[24])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
	
	//results			=	_cmpysp(_amem8_f2(&B[1]), _amem8_f2(&C[3]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&B[1]), _amem8_f2(&C[3]));

	//results			=	_cmpysp(_amem8_f2(&B[5]), _amem8_f2(&C[7]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[5]), _amem8_f2(&C[7])));

	//results			=	_cmpysp(_amem8_f2(&B[9]), _amem8_f2(&C[11]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[9]), _amem8_f2(&C[11])));

	//results			=	_cmpysp(_amem8_f2(&B[13]), _amem8_f2(&C[15]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[13]), _amem8_f2(&C[15])));

	dtemp1			=	_daddsp(_amem8_f2(&invA[7]), dtemp1);
	_amem8_f2(&output[10])	=	dtemp1;
	//_amem8_f2(&output[11])	=	dtemp1;
	//_amem8_f2(&output[25])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
	
	//results			=	_cmpysp(_amem8_f2(&B[2]), _amem8_f2(&C[3]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&B[2]), _amem8_f2(&C[3]));

	//results			=	_cmpysp(_amem8_f2(&B[6]), _amem8_f2(&C[7]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[6]), _amem8_f2(&C[7])));

//	results			=	_cmpysp(_amem8_f2(&B[10]), _amem8_f2(&C[11]));
//	dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[10]), _amem8_f2(&C[11])));

//	results			=	_cmpysp(_amem8_f2(&B[14]), _amem8_f2(&C[15]));
//	dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[14]), _amem8_f2(&C[15])));

	dtemp1			=	_daddsp(_amem8_f2(&invA[11]), dtemp1);
	_amem8_f2(&output[16])	=	dtemp1;
	//_amem8_f2(&output[19])	=	dtemp1;
	//_amem8_f2(&output[26])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
	
	//results			=	_cmpysp(_amem8_f2(&B[3]), _amem8_f2(&C[3]));
	//dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp1			=	_complex_conjugate_mpysp(_amem8_f2(&B[3]), _amem8_f2(&C[3]));

	//results			=	_cmpysp(_amem8_f2(&B[7]), _amem8_f2(&C[7]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[7]), _amem8_f2(&C[7])));

	//results			=	_cmpysp(_amem8_f2(&B[11]), _amem8_f2(&C[11]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[11]), _amem8_f2(&C[11])));

	//results			=	_cmpysp(_amem8_f2(&B[15]), _amem8_f2(&C[15]));
	//dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	dtemp1			=	_daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[15]), _amem8_f2(&C[15])));

	dtemp1			=	_daddsp(_amem8_f2(&invA[15]), dtemp1);
	_amem8_f2(&output[21])	=	dtemp1;
	//_amem8_f2(&output[27])	=	dtemp1;

	
/*	MATRIX_Mult4x4fp(B, tempC, C);
	for ( jj = 0; jj < 16; jj++ )
	{
		_amem8_f2(&A[jj])		=	_daddsp(_amem8_f2(&invA[jj]), _amem8_f2(&C[jj]));
	}

	for ( jj = 0; jj < 4; jj++ )
	{
		_amem8_f2(&output[jj * 8 + 0])	=	_amem8_f2(&A[jj * 4 + 0]);
		_amem8_f2(&output[jj * 8 + 1])	=	_amem8_f2(&A[jj * 4 + 1]);
		_amem8_f2(&output[jj * 8 + 2])	=	_amem8_f2(&A[jj * 4 + 2]);
		_amem8_f2(&output[jj * 8 + 3])	=	_amem8_f2(&A[jj * 4 + 3]);
	}*/

#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;

		/* A matrix */ 
		fid			=	fopen("invRn.dat", "w");
		for (jj = 0; jj < 64; jj++)
			fprintf(fid, "\t%e\t\t%e\n", output[jj].real, output[jj].imag);
		fclose(fid);

	}
#endif

}







/** 
 *  \fn     void MATRIX_4x4_BWInversionfp(
 *                            IN      cplxf_t  * RESTRICT Input,
 *                            OUT     cplxf_t  * RESTRICT output);
 *
 *  \brief   4x4 matrix inversion using block wise method.
 * 
 *  \param[in]    Input
 *              Input 4x4 matrix that needs to be inversed, stored sequentially 
 *              in format (1,1), (1,2).... (1,4), (2,1), (2,2)...(4,4).
 *
 *  \param[out]   output
 *              Output 4x4 matrix. Stored sequentially as the input.
 *
 *  \pre  
 *
 *  \post  
 * 
 *  \sa   
 * 
 */

void MATRIX_4x4_BWInversionfp (  
                             IN      cplxf_t  * RESTRICT Input,
                             OUT     cplxf_t  * output)
{
	
	float  A0, A3, D0, D3, F0, F3;
	float  detA, oneOverdetA;
	float  ftemp1;
	__float2_t A1, D1, F1, C0, C1, C2, C3;
	__float2_t B0, B1, B2, B3;
	__float2_t dtemp, dtemp1, dtemp2;

	/* load A */
	dtemp			=	_amem8_f2(&Input[0]);
	A0				=	_hif2(dtemp);
	dtemp			=	_amem8_f2(&Input[5]);
	A3				=	_hif2(dtemp);
	A1				=	_amem8_f2(&Input[1]);

	/* Calculate D = inv(D) */
	dtemp			=	 _amem8_f2(&Input[10]);
	D0				=	_hif2(dtemp);
	dtemp			=	 _amem8_f2(&Input[15]);
	D3				=	_hif2(dtemp);
	D1				=	 _amem8_f2(&Input[11]);
	dtemp			=	_dmpysp(D1, D1);
	detA			=	D0 * D3 - _hif2(dtemp) - _lof2(dtemp);
	oneOverdetA		=	_rcpsp( detA );
	oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
	oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);

	ftemp1			=	D0 * oneOverdetA;
	D0				=	D3 * oneOverdetA;
	D3				=	ftemp1;
	D1				=	_dmpysp(D1, _ftof2(-oneOverdetA, -oneOverdetA));

	/* load B */
	B0				=	_amem8_f2(&Input[2]);
	B1				=	_amem8_f2(&Input[3]);
	B2				=	_amem8_f2(&Input[6]);
	B3				=	_amem8_f2(&Input[7]);

	/* calculate C = B*inv(D) */
	//results			=	_cmpysp(D1, B1);
	//C0				=	_dsubsp(_hif2_128(results), _lof2_128(results));
	C0				=	_complex_conjugate_mpysp(D1, B1);
	C0				=	_daddsp(C0, _dmpysp(B0, _ftof2(D0, D0)));
	//results			=	_cmpysp(D1, B0);
	//C1				=	_daddsp(_hif2_128(results), _lof2_128(results));
	C1				=	_complex_mpysp(D1, B0);
	dtemp1			=	C1;
	C1				=	_daddsp(C1, _dmpysp(B1, _ftof2(D3, D3)));
	//results			=	_cmpysp(D1, B3);
	//C2				=	_dsubsp(_hif2_128(results), _lof2_128(results));
	C2				=	_complex_conjugate_mpysp(D1, B3);
	C2				=	_daddsp(C2, _dmpysp(B2, _ftof2(D0, D0)));
	//results			=	_cmpysp(D1, B2);
	//C3				=	_daddsp(_hif2_128(results), _lof2_128(results));
	C3				=	_complex_mpysp(D1, B2);
	dtemp2			=	C3;
	C3				=	_daddsp(C3, _dmpysp(B3, _ftof2(D3, D3)));

	/* calculate F = A - B *inv(D) * conj(B) -- Hermitian */
	dtemp			=	_dmpysp(B0, B0);
	F0				=	A0 - D0 * (_hif2(dtemp) + _lof2(dtemp));
	dtemp			=	_dmpysp(B1, B1);
	F0				-=	D3 * (_hif2(dtemp) + _lof2(dtemp));
	//results			=	_cmpysp(B1, dtemp1);
	//dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp			=	_complex_conjugate_mpysp(B1, dtemp1);
	F0				-=	2.f * _hif2(dtemp);

	dtemp			=	_dmpysp(B2, B2);
	F3				=	A3 - D0 * (_hif2(dtemp) + _lof2(dtemp));
	dtemp			=	_dmpysp(B3, B3);
	F3				-=	D3 * (_hif2(dtemp) + _lof2(dtemp));
	//results			=	_cmpysp(B3, dtemp2);
	//dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp			=	_complex_conjugate_mpysp(B3, dtemp2);
	F3				-=	2.f * _hif2(dtemp);

	//results			=	_cmpysp(B2, C0);
	//F1				=	_dsubsp(A1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	F1				=	_dsubsp(A1, _complex_conjugate_mpysp(B2, C0));
	//results			=	_cmpysp(B3, C1);
	//F1				=	_dsubsp(F1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	F1				=	_dsubsp(F1, _complex_conjugate_mpysp(B3, C1));

	/* Calculate F = inv(F) */
	dtemp			=	_dmpysp(F1, F1);
	detA			=	F0 * F3 - _hif2(dtemp) - _lof2(dtemp);
	oneOverdetA		=	_rcpsp( detA );
	oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
	oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
	
	ftemp1			=	F0 * oneOverdetA;
	F0				=	F3 * oneOverdetA;
	F3				=	ftemp1;
	F1				=	_dmpysp(F1, _ftof2(-oneOverdetA, -oneOverdetA));

	/* NW output */
	_amem8_f2(&output[0])	=	_ftof2(F0, 0.f);
	_amem8_f2(&output[1])	=	F1;
	_amem8_f2(&output[5])	=	_ftof2(F3, 0.f);
	//_amem8_f2(&output[4])	=	_lltof2(_f2toll(F1) ^ 0x0000000080000000);
	_amem8_f2(&output[4])	=	_ftof2(_hif2(F1), -_lof2(F1));

	/* NE output = - F * C, SW = conj(NW)*/
	//results			=	_cmpysp(F1, C2);
	//dtemp			=	_daddsp(_hif2_128(results), _lof2_128(results));
	dtemp			=	_complex_mpysp(F1, C2);
	dtemp1			=	dtemp;
	dtemp			=	_daddsp(dtemp, _dmpysp(C0, _ftof2(F0, F0)));
	_amem8_f2(&output[2])		=	_ftof2(-_hif2(dtemp), -_lof2(dtemp));
	_amem8_f2(&output[8])		=	_ftof2(-_hif2(dtemp), _lof2(dtemp));
	//_amem8_f2(&output[2])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
	//_amem8_f2(&output[8])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);

	//results			=	_cmpysp(F1, C3);
	//dtemp			=	_daddsp(_hif2_128(results), _lof2_128(results));
	dtemp			=	_complex_mpysp(F1, C3);
	dtemp2			=	dtemp;
	dtemp			=	_daddsp(dtemp, _dmpysp(C1, _ftof2(F0, F0)));
	B1				=	dtemp;
	_amem8_f2(&output[3])		=	_ftof2(-_hif2(dtemp), -_lof2(dtemp));
	_amem8_f2(&output[12])		=	_ftof2(-_hif2(dtemp), _lof2(dtemp));
	//_amem8_f2(&output[3])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
	//_amem8_f2(&output[12])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);

	//results			=	_cmpysp(F1, C0);
	//dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp			=	_complex_conjugate_mpysp(F1, C0);
	dtemp			=	_daddsp(dtemp, _dmpysp(C2, _ftof2(F3, F3)));
	_amem8_f2(&output[6])		=	_ftof2(-_hif2(dtemp), -_lof2(dtemp));
	_amem8_f2(&output[9])		=	_ftof2(-_hif2(dtemp), _lof2(dtemp));
	//_amem8_f2(&output[6])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
	//_amem8_f2(&output[9])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);

	//results			=	_cmpysp(F1, C1);
	//dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp			=	_complex_conjugate_mpysp(F1, C1);
	dtemp			=	_daddsp(dtemp, _dmpysp(C3, _ftof2(F3, F3)));
	B3				=	dtemp;
	_amem8_f2(&output[7])		=	_ftof2(-_hif2(dtemp), -_lof2(dtemp));
	_amem8_f2(&output[13])		=	_ftof2(-_hif2(dtemp), _lof2(dtemp));
	//_amem8_f2(&output[7])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
	//_amem8_f2(&output[13])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);
	

	
	/* SE output */
	/* inv(D) - conj(C) * inv(F) * C, whrer C = B * inv(D) */
	dtemp			=	_dmpysp(C0, C0);
	A0				=	D0 + F0 * (_hif2(dtemp) + _lof2(dtemp));
	dtemp			=	_dmpysp(C2, C2);
	A0				+=	F3 * (_hif2(dtemp) + _lof2(dtemp));
	//results			=	_cmpysp(C0, dtemp1);
	//dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp			=	_complex_conjugate_mpysp(C0, dtemp1);
	A0				+=	2.f * _hif2(dtemp);

	dtemp			=	_dmpysp(C1, C1);
	A3				=	D3 + F0 * (_hif2(dtemp) + _lof2(dtemp));
	dtemp			=	_dmpysp(C3, C3);
	A3				+=	F3 * (_hif2(dtemp) + _lof2(dtemp));
	//results			=	_cmpysp(C1, dtemp2);
	//dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp			=	_complex_conjugate_mpysp(C1, dtemp2);
	A3				+=	2.f * _hif2(dtemp);
	_amem8_f2(&output[10])=	_ftof2(A0, 0.f);
	_amem8_f2(&output[15])=	_ftof2(A3, 0.f);

	//results			=	_cmpysp(C0, B1);
	//dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
	dtemp			=	_complex_conjugate_mpysp(C0, B1);
	A1				=	_daddsp(D1, dtemp);
	//results			=	_cmpysp(C2, B3);
	//A1				=	_daddsp(A1, _dsubsp(_hif2_128(results), _lof2_128(results)));
	A1				=	_daddsp(A1, _complex_conjugate_mpysp(C2, B3));

	_amem8_f2(&output[11])=	A1;
	_amem8_f2(&output[14])=	_ftof2(_hif2(A1), -_lof2(A1));
	//_amem8_f2(&output[14])=	_lltof2(_f2toll(A1) ^ 0x0000000080000000);

}




void MATRIX_Mult4x4fp (  
                             IN      cplxf_t * RESTRICT A,
                             IN      cplxf_t * RESTRICT B,
                             OUT     cplxf_t * RESTRICT C)

{

#if GENERICCODE
	int32_t	jj, kk, mm;
	float   tempre, tempim;

	for ( jj = 0; jj < 4; jj++ )
	{
		for ( kk = 0; kk < 4; kk++ )
		{
			tempre		=	0.f;
			tempim		=	0.f;
			for (mm = 0; mm < 4; mm++)
			{
				tempre		+=	A[4 * jj + mm].real * B[4 * mm + kk].real - A[4 * jj + mm].imag * B[4 * mm + kk].imag;
				tempim		+=	A[4 * jj + mm].imag * B[4 * mm + kk].real + A[4 * jj + mm].real * B[4 * mm + kk].imag;
			}
			C[4 * jj + kk].real		=	tempre;
			C[4 * jj + kk].imag		=	tempim;
		}
	}
#else
	int32_t	kk, mm;
	__float2_t  * RESTRICT A1, * RESTRICT A2, * RESTRICT A3, * RESTRICT A4;
	__float2_t  * RESTRICT C1, * RESTRICT C2, * RESTRICT C3, * RESTRICT C4;
	__float2_t  dtemp, dtemp1, dtemp2, dtemp3;

	A1 				=	(__float2_t *) &A[0];
	A2 				=	(__float2_t *) &A[4];
	A3 				=	(__float2_t *) &A[8];
	A4 				=	(__float2_t *) &A[12];
	C1 				=	(__float2_t *) &C[0];
	C2 				=	(__float2_t *) &C[4];
	C3 				=	(__float2_t *) &C[8];
	C4 				=	(__float2_t *) &C[12];
	for ( kk = 0; kk < 4; kk++ )
	{
		dtemp		=	_ftof2(0.f, 0.f);
		dtemp1		=	dtemp;
		dtemp2		=	dtemp;
		dtemp3		=	dtemp;
		#ifdef _TMS320C6x
		#pragma UNROLL(4);
		#endif
		for (mm = 0; mm < 4; mm++)
		{
			dtemp		=	_daddsp(dtemp, _complex_mpysp(_amem8_f2(&A1[mm]), _amem8_f2(&B[4 * mm + kk])));
			dtemp1		=	_daddsp(dtemp1, _complex_mpysp(_amem8_f2(&A2[mm]), _amem8_f2(&B[4 * mm + kk])));
			dtemp2		=	_daddsp(dtemp2, _complex_mpysp(_amem8_f2(&A3[mm]), _amem8_f2(&B[4 * mm + kk])));
			dtemp3		=	_daddsp(dtemp3, _complex_mpysp(_amem8_f2(&A4[mm]), _amem8_f2(&B[4 * mm + kk])));
		}
		_amem8_f2(&C1[kk])	=	dtemp;
		_amem8_f2(&C2[kk])	=	dtemp1;
		_amem8_f2(&C3[kk])	=	dtemp2;
		_amem8_f2(&C4[kk])	=	dtemp3;
	}
#endif
}

