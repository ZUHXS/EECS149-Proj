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

/*!
 *   \fn     RADARDEMO_aoaEstCaponBF_dopperEstInput
 *
 *   \brief   Calculate the beam forming output for doppler estimation, for the current range bin and azimuth bin.
 *
 *   \param[in]    bfFlag
 *               Flag to indicate which covariance matrix based beamforming will be performed.
 *               If set to 1, Capon BF.
 *               If set to 0, conventional BF.
 *
 *   \param[in]    nRxAnt
 *               number of antenna
 *
 *   \param[in]    nChirps
 *               number of chirps
 *
 *   \param[in]    inputAntSamples
 *              Input 1D FFT results for the current range bin.
 *              Must be aligned to 8-byte boundary.
 *
 *   \param[in]    steeringVec
 *              steering vector for beamforming for the current azimuth bin.
 *
 *   \param[in]    invRnMatrices
 *               Output inverse of covariance matrices for the current range bin, in order of numInputRangeBins by upper triangle of nRxAnt x nRxAnt Hermitian matrix.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[in]    scratch
 *               Output inverse of covariance matrices for the current range bin, in order of numInputRangeBins by upper triangle of nRxAnt x nRxAnt Hermitian matrix.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[in]    rangeAzimuthHeatMap
 *               input range azimuth heatmap for current range bin and azimuth bin
 *
 *   \param[out]    bfOutput
 *               Beamforming output for the current range bin and azimuth bin.
 *               Must be in the order of real0, imag0, real1, imag1... as required by DSP LIB single precision floating-point FFT.
 *               Must be aligned to 8-byte boundary.
 *
 *   \ret       none
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
void		RADARDEMO_aoaEstCaponBF_dopperEstInput(
				IN uint8_t bfFlag,
				IN int32_t nRxAnt,
				IN int32_t nChirps,
				IN cplx16_t * inputAntSamples,
				IN cplxf_t * steeringVec,
				IN cplxf_t  * invRnMatrices,
				IN int32_t * scratch,
				IN float rangeAzimuthHeatMap,
				OUT float * RESTRICT bfOutput
			)
{
	int32_t chirpIdx, i;
	__float2_t * RESTRICT bweights;
	__float2_t * RESTRICT rnPtr;
	__float2_t * RESTRICT steerVecPtr;
	__float2_t f2temp1, acc2, scale2;


	/* beamforming weights = A*Rn, beamforming output = input(1x8)*A*Rn */
	scale2		=	_ftof2(rangeAzimuthHeatMap, rangeAzimuthHeatMap);
	bweights	=	(__float2_t *) &scratch[0];
	rnPtr		=	(__float2_t *) &invRnMatrices[0];
	steerVecPtr	=	(__float2_t *) &steeringVec[0];

	if (nRxAnt == 8)
	{
		int32_t * RESTRICT inPtr1;
		int32_t * RESTRICT inPtr2;
		int32_t * RESTRICT inPtr3;
		int32_t * RESTRICT inPtr4;
		int32_t * RESTRICT inPtr5;
		int32_t * RESTRICT inPtr6;
		int32_t * RESTRICT inPtr7;
		int32_t * RESTRICT inPtr8;

		inPtr1		=	(int32_t *) &inputAntSamples[0 * nChirps];
		inPtr2		=	(int32_t *) &inputAntSamples[1 * nChirps];
		inPtr3		=	(int32_t *) &inputAntSamples[2 * nChirps];
		inPtr4		=	(int32_t *) &inputAntSamples[3 * nChirps];
		inPtr5		=	(int32_t *) &inputAntSamples[4 * nChirps];
		inPtr6		=	(int32_t *) &inputAntSamples[5 * nChirps];
		inPtr7		=	(int32_t *) &inputAntSamples[6 * nChirps];
		inPtr8		=	(int32_t *) &inputAntSamples[7 * nChirps];

		if (bfFlag)
		{
			//bweights[0]
			acc2		=	_amem8_f2(&rnPtr[0]);
			for (i = 1; i < nRxAnt; i++)
			{
				f2temp1	=	_complex_conjugate_mpysp(_amem8_f2(&rnPtr[i]), _amem8_f2(&steerVecPtr[i - 1]));
				acc2	=	_daddsp(acc2, f2temp1);
			}
			_amem8_f2(&bweights[0])		=	_dmpysp(acc2, scale2);

			//bweights[1]
			acc2		=	_amem8_f2(&rnPtr[1]);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[8]), _amem8_f2(&steerVecPtr[0]));
			acc2		=	_daddsp(acc2, f2temp1);
			for (i = 2; i < nRxAnt; i++)
			{
				f2temp1	=	_complex_conjugate_mpysp(_amem8_f2(&rnPtr[i + 7]), _amem8_f2(&steerVecPtr[i - 1]));
				acc2	=	_daddsp(acc2, f2temp1);
			}
			_amem8_f2(&bweights[1])		=	_dmpysp(acc2, scale2);

			//bweights[2]
			acc2		=	_amem8_f2(&rnPtr[2]);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[9]), _amem8_f2(&steerVecPtr[0]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[15]), _amem8_f2(&steerVecPtr[1]));
			acc2		=	_daddsp(acc2, f2temp1);
			for (i = 3; i < nRxAnt; i++)
			{
				f2temp1	=	_complex_conjugate_mpysp(_amem8_f2(&rnPtr[i + 13]), _amem8_f2(&steerVecPtr[i - 1]));
				acc2	=	_daddsp(acc2, f2temp1);
			}
			_amem8_f2(&bweights[2])		=	_dmpysp(acc2, scale2);

			//bweights[3]
			acc2		=	_amem8_f2(&rnPtr[3]);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[10]), _amem8_f2(&steerVecPtr[0]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[16]), _amem8_f2(&steerVecPtr[1]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[21]), _amem8_f2(&steerVecPtr[2]));
			acc2		=	_daddsp(acc2, f2temp1);
			for (i = 4; i < nRxAnt; i++)
			{
				f2temp1	=	_complex_conjugate_mpysp(_amem8_f2(&rnPtr[i + 18]), _amem8_f2(&steerVecPtr[i - 1]));
				acc2	=	_daddsp(acc2, f2temp1);
			}
			_amem8_f2(&bweights[3])		=	_dmpysp(acc2, scale2);

			//bweights[4]
			acc2		=	_amem8_f2(&rnPtr[4]);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[11]), _amem8_f2(&steerVecPtr[0]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[17]), _amem8_f2(&steerVecPtr[1]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[22]), _amem8_f2(&steerVecPtr[2]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[26]), _amem8_f2(&steerVecPtr[3]));
			acc2		=	_daddsp(acc2, f2temp1);
			for (i = 5; i < nRxAnt; i++)
			{
				f2temp1	=	_complex_conjugate_mpysp(_amem8_f2(&rnPtr[i + 22]), _amem8_f2(&steerVecPtr[i - 1]));
				acc2	=	_daddsp(acc2, f2temp1);
			}
			_amem8_f2(&bweights[4])		=	_dmpysp(acc2, scale2);

			//bweights[5]
			acc2		=	_amem8_f2(&rnPtr[5]);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[12]), _amem8_f2(&steerVecPtr[0]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[18]), _amem8_f2(&steerVecPtr[1]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[23]), _amem8_f2(&steerVecPtr[2]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[27]), _amem8_f2(&steerVecPtr[3]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[30]), _amem8_f2(&steerVecPtr[4]));
			acc2		=	_daddsp(acc2, f2temp1);
			for (i = 6; i < nRxAnt; i++)
			{
				f2temp1	=	_complex_conjugate_mpysp(_amem8_f2(&rnPtr[i + 25]), _amem8_f2(&steerVecPtr[i - 1]));
				acc2	=	_daddsp(acc2, f2temp1);
			}
			_amem8_f2(&bweights[5])		=	_dmpysp(acc2, scale2);

			//bweights[6]
			acc2		=	_amem8_f2(&rnPtr[6]);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[13]), _amem8_f2(&steerVecPtr[0]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[19]), _amem8_f2(&steerVecPtr[1]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[24]), _amem8_f2(&steerVecPtr[2]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[28]), _amem8_f2(&steerVecPtr[3]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[31]), _amem8_f2(&steerVecPtr[4]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[33]), _amem8_f2(&steerVecPtr[5]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_conjugate_mpysp(_amem8_f2(&rnPtr[34]), _amem8_f2(&steerVecPtr[6]));
			acc2		=	_daddsp(acc2, f2temp1);
			_amem8_f2(&bweights[6])		=	_dmpysp(acc2, scale2);

			//bweights[7]
			acc2		=	_amem8_f2(&rnPtr[7]);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[14]), _amem8_f2(&steerVecPtr[0]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[20]), _amem8_f2(&steerVecPtr[1]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[25]), _amem8_f2(&steerVecPtr[2]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[29]), _amem8_f2(&steerVecPtr[3]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[32]), _amem8_f2(&steerVecPtr[4]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[34]), _amem8_f2(&steerVecPtr[5]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[35]), _amem8_f2(&steerVecPtr[6]));
			acc2		=	_daddsp(acc2, f2temp1);
			_amem8_f2(&bweights[7])		=	_dmpysp(acc2, scale2);
		}
		else
		{	
			_amem8_f2(&bweights[0])	=	_ftof2(1.f, 0.f);
			for (i = 0; i < nRxAnt - 1; i++)
			{
				_amem8_f2(&bweights[i + 1])	=	_amem8_f2(&steerVecPtr[i]);
			}
		}

		for (chirpIdx = 0; chirpIdx < nChirps; chirpIdx++)
		{
			acc2	=	_complex_mpysp(_dinthsp(_amem4(&inPtr1[chirpIdx])), _amem8_f2(&bweights[0]));
			acc2	=	_daddsp(acc2, _complex_mpysp(_dinthsp(_amem4(&inPtr2[chirpIdx])), _amem8_f2(&bweights[1])));
			acc2	=	_daddsp(acc2, _complex_mpysp(_dinthsp(_amem4(&inPtr3[chirpIdx])), _amem8_f2(&bweights[2])));
			acc2	=	_daddsp(acc2, _complex_mpysp(_dinthsp(_amem4(&inPtr4[chirpIdx])), _amem8_f2(&bweights[3])));
			acc2	=	_daddsp(acc2, _complex_mpysp(_dinthsp(_amem4(&inPtr5[chirpIdx])), _amem8_f2(&bweights[4])));
			acc2	=	_daddsp(acc2, _complex_mpysp(_dinthsp(_amem4(&inPtr6[chirpIdx])), _amem8_f2(&bweights[5])));
			acc2	=	_daddsp(acc2, _complex_mpysp(_dinthsp(_amem4(&inPtr7[chirpIdx])), _amem8_f2(&bweights[6])));
			acc2	=	_daddsp(acc2, _complex_mpysp(_dinthsp(_amem4(&inPtr8[chirpIdx])), _amem8_f2(&bweights[7])));
			_amem8_f2(&bfOutput[2*chirpIdx])		=	_ftof2(_lof2(acc2), _hif2(acc2));
		}
	}
	else if (nRxAnt == 4)
	{
		int32_t * RESTRICT inPtr1;
		int32_t * RESTRICT inPtr2;
		int32_t * RESTRICT inPtr3;
		int32_t * RESTRICT inPtr4;

		inPtr1		=	(int32_t *) &inputAntSamples[0 * nChirps];
		inPtr2		=	(int32_t *) &inputAntSamples[1 * nChirps];
		inPtr3		=	(int32_t *) &inputAntSamples[2 * nChirps];
		inPtr4		=	(int32_t *) &inputAntSamples[3 * nChirps];
		if (bfFlag)
		{
			//bweights[0]
			acc2		=	_amem8_f2(&rnPtr[0]);
			for (i = 1; i < nRxAnt; i++)
			{
				f2temp1	=	_complex_conjugate_mpysp(_amem8_f2(&rnPtr[i]), _amem8_f2(&steerVecPtr[i - 1]));
				acc2	=	_daddsp(acc2, f2temp1);
			}
			_amem8_f2(&bweights[0])		=	_dmpysp(acc2, scale2);

			//bweights[1]
			acc2		=	_amem8_f2(&rnPtr[1]);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[4]), _amem8_f2(&steerVecPtr[0]));
			acc2		=	_daddsp(acc2, f2temp1);
			for (i = 2; i < nRxAnt; i++)
			{
				f2temp1	=	_complex_conjugate_mpysp(_amem8_f2(&rnPtr[i + 3]), _amem8_f2(&steerVecPtr[i - 1]));
				acc2	=	_daddsp(acc2, f2temp1);
			}
			_amem8_f2(&bweights[1])		=	_dmpysp(acc2, scale2);

			//bweights[2]
			acc2		=	_amem8_f2(&rnPtr[2]);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[5]), _amem8_f2(&steerVecPtr[0]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[7]), _amem8_f2(&steerVecPtr[1]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_conjugate_mpysp(_amem8_f2(&rnPtr[8]), _amem8_f2(&steerVecPtr[2]));
			acc2		=	_daddsp(acc2, f2temp1);
			_amem8_f2(&bweights[2])		=	_dmpysp(acc2, scale2);

			//bweights[3]
			acc2		=	_amem8_f2(&rnPtr[3]);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[6]), _amem8_f2(&steerVecPtr[0]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[8]), _amem8_f2(&steerVecPtr[1]));
			acc2		=	_daddsp(acc2, f2temp1);
			f2temp1		=	_complex_mpysp(_amem8_f2(&rnPtr[9]), _amem8_f2(&steerVecPtr[2]));
			acc2		=	_daddsp(acc2, f2temp1);
			_amem8_f2(&bweights[3])		=	_dmpysp(acc2, scale2);
		}
		else
		{	
			_amem8_f2(&bweights[0])	=	_ftof2(1.f, 0.f);
			for (i = 0; i < nRxAnt - 1; i++)
			{
				_amem8_f2(&bweights[i + 1])	=	_amem8_f2(&steerVecPtr[i]);
			}
		}


		for (chirpIdx = 0; chirpIdx < nChirps; chirpIdx++)
		{
			acc2	=	_complex_mpysp(_dinthsp(_amem4(&inPtr1[chirpIdx])), _amem8_f2(&bweights[0]));
			acc2	=	_daddsp(acc2, _complex_mpysp(_dinthsp(_amem4(&inPtr2[chirpIdx])), _amem8_f2(&bweights[1])));
			acc2	=	_daddsp(acc2, _complex_mpysp(_dinthsp(_amem4(&inPtr3[chirpIdx])), _amem8_f2(&bweights[2])));
			acc2	=	_daddsp(acc2, _complex_mpysp(_dinthsp(_amem4(&inPtr4[chirpIdx])), _amem8_f2(&bweights[3])));
			_amem8_f2(&bfOutput[chirpIdx])		=	_ftof2(_lof2(acc2), _hif2(acc2));
		}

	}
}

