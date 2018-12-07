/*! 
 *  \file   RADARDEMO_highAccuRangeProc_priv.c
 *
 *  \brief   Windowing functions for range processing.
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

#include "RADARDEMO_highAccuRangeProc_priv.h"

#ifdef _TMS320C6X
#include "c6x.h"
#endif

#ifdef ARMVERSION
extern void dft(int size, float * input, float * output);
#endif

#ifndef ARMVERSION

/*!
   \fn     RADARDEMO_highAccuRangeProc_accumulateInput

   \brief   Accumulate signals from all chirps and convert to floating-point format with 1D windowing.

   \param[in]    nSamplesPerChirp
               Number of samples per chirp.

   \param[in]    fftSize1D
               1D FFT size.

   \param[in]    nChirpPerFrame
               Number of chirps per frame.

   \param[in]    fftWin1D
               Input pointer to 1D window function.
			   
   \param[in]    fftWin1DSize
               Number of ADC bits.

   \param[in]    inputPtr
               Pointer to input signal.

   \param[in]    chirpInd
               Flag to enable DC adjustment for ADC samples when set to 1. Otherwise, no preprocessing for ADC samples.

   \param[out]    outputPtr
               Pointer to the output signal.

   \pre       none

   \post      none


 */

void	RADARDEMO_highAccuRangeProc_accumulateInput(
                            IN  uint32_t nSamplesPerChirp,
                            IN  uint32_t fftSize1D,
                            IN  uint32_t nChirpPerFrame,
                            IN  float   * RESTRICT fftWin1D,
							IN 	int16_t  fftWin1DSize,
							IN  cplx16_t * RESTRICT inputPtr,
							IN  int32_t  chirpInd,
							OUT float * outputPtr)
{
	int32_t i, itemp;
	int64_t input;

	if (chirpInd == 0)
	{
		for( i = 0; i < ((int32_t)nSamplesPerChirp>> 1); i++)
		{
			input							=	_amem8(&inputPtr[2 * i]);
			_amem8_f2(&outputPtr[4*i])		=	_ftof2((float)_ext(_loll(input), 16, 16), (float)_ext(_loll(input), 0, 16));
			_amem8_f2(&outputPtr[4*i + 2])	=	_ftof2((float)_ext(_hill(input), 16, 16), (float)_ext(_hill(input), 0, 16));
		}
		if((int32_t)nSamplesPerChirp & 1)
		{
			itemp							=	_amem4(&inputPtr[nSamplesPerChirp - 1]);
			_amem8_f2(&outputPtr[2*(nSamplesPerChirp - 1)])	=	_ftof2((float)_ext(itemp, 16, 16), (float)_ext(itemp, 0, 16));
		}
	}
	else
	{
		for( i = 0; i < ((int32_t)nSamplesPerChirp>> 1); i++)
		{
			input							=	_amem8(&inputPtr[2 * i]);
			_amem8_f2(&outputPtr[4*i])		=	_daddsp(_amem8_f2(&outputPtr[4*i]), _ftof2((float)_ext(_loll(input), 16, 16), (float)_ext(_loll(input), 0, 16)));
			_amem8_f2(&outputPtr[4*i + 2])	=	_daddsp(_amem8_f2(&outputPtr[4*i + 2]), _ftof2((float)_ext(_hill(input), 16, 16), (float)_ext(_hill(input), 0, 16)));
		}
		if((int32_t)nSamplesPerChirp & 1)
		{
			itemp							=	_amem4(&inputPtr[nSamplesPerChirp - 1]);
			_amem8_f2(&outputPtr[2*(nSamplesPerChirp - 1)]) = _daddsp(_amem8_f2(&outputPtr[2*(nSamplesPerChirp - 1)]), _ftof2((float)_ext(itemp, 16, 16), (float)_ext(itemp, 0, 16)));
		}

		if (chirpInd == ((int32_t)nChirpPerFrame - 1))
		{
			__float2_t f2win1D;

			for( i = 0; i < fftWin1DSize; i++)
			{
				f2win1D						=	_ftof2(fftWin1D[i], fftWin1D[i]);
				_amem8_f2(&outputPtr[2*i])	=	_dmpysp(_amem8_f2(&outputPtr[2*i]), f2win1D);
				_amem8_f2(&outputPtr[2*(nSamplesPerChirp - i - 1)])	=	_dmpysp(_amem8_f2(&outputPtr[2*(nSamplesPerChirp - i - 1)]), f2win1D);
			}
			if (((int32_t)fftSize1D - (int32_t)nSamplesPerChirp) > 0)
			{
				for( i = 0; i < ((int32_t)fftSize1D - (int32_t)nSamplesPerChirp); i++);
				{
					_amem8_f2(&outputPtr[2*nSamplesPerChirp + 2*i])	=	_ftof2(0.f, 0.f);
				}
			}
		}
	}
}


/*!
   \fn     RADARDEMO_highAccuRangeProc_rangeEst

   \brief   Windowing function for 2D FFT and prepare for the input.

   \param[in]    highAccuRangeHandle
               Handle to the module.

   \param[out]    estRange
               Estimated range.

   \param[out]    deltaPhaseEst
               Estimated delta phase.

   \param[out]    estLinearSNR
               Estimated SNR.

   \pre       none

   \post      none


 */

void	RADARDEMO_highAccuRangeProc_rangeEst(
                            IN  RADARDEMO_highAccuRangeProc_handle *highAccuRangeHandle,
							OUT  float * estRange,
							OUT  float * deltaPhaseEst,
							OUT float * estLinearSNR)
{
	int32_t			i, j, k, rad1D, coarseRangeInd;
	unsigned char	* brev = NULL;
	float			totalPower, max, ftemp, sigPower, * RESTRICT powerPtr;
	__float2_t		f2input, * RESTRICT inputPtr, * RESTRICT inputPtr1;

	int32_t			zoomStartInd, zoomEndInd, itemp, tempIndFine1, tempIndFine2, tempIndCoarse, indMask, shift;
	int32_t			fineRangeInd, tempFineSearchIdx, tempCoarseSearchIdx;
	__float2_t		*RESTRICT wncPtr, wncoarse, *RESTRICT wnfPtr, wnfine1, sigAcc, f2temp;
	double			freqFineEst, fdelta, interpIndx;
	float           currP, prevP, maxPrevP, maxNextP;

	j  = 30 - _norm(highAccuRangeHandle->fft1DSize);
	if ((j & 1) == 0)
		rad1D = 4;
	else
		rad1D = 2;

	/* copy to scratch is needed because FFT function will corrupt the input. We need to preserve if for zoom-in FFT */
	inputPtr	=	(__float2_t *) highAccuRangeHandle->inputSig;
	inputPtr1	=	(__float2_t *) &highAccuRangeHandle->scratchPad[2 * highAccuRangeHandle->fft1DSize];
	for (i = 0; i < (int32_t)highAccuRangeHandle->fft1DSize; i++ )
	{
		f2input					=	_amem8_f2(inputPtr);
		_amem8_f2(inputPtr1++)	=	f2input;
		_amem8_f2(inputPtr)   =	_ftof2(_lof2(f2input), _hif2(f2input));
		inputPtr++;
	}

	inputPtr1	=	(__float2_t *) &highAccuRangeHandle->scratchPad[2 * highAccuRangeHandle->fft1DSize];
	DSPF_sp_fftSPxSP (
			highAccuRangeHandle->fft1DSize,
			(float*) inputPtr1,
			(float *)highAccuRangeHandle->twiddle, 
			highAccuRangeHandle->fft1DOutSig,
			brev,
			rad1D,
			0,
			highAccuRangeHandle->fft1DSize);

	for( i = 0; i < (int32_t)highAccuRangeHandle->skipLeft;  i++ )
	{
		_amem8_f2(&highAccuRangeHandle->fft1DOutSig[2*i]) = _ftof2(0.f, 0.f);
	}
	for( i = (int32_t)highAccuRangeHandle->fft1DSize- (int32_t)highAccuRangeHandle->skipRight; i < (int32_t)highAccuRangeHandle->fft1DSize; i++ )
	{
		_amem8_f2(&highAccuRangeHandle->fft1DOutSig[2*i]) = _ftof2(0.f, 0.f);
	}
	max			=	0.f;
	totalPower	=	0.f;
	coarseRangeInd	=	0;
	inputPtr	=	(__float2_t *)highAccuRangeHandle->fft1DOutSig;
	powerPtr	=	(float *)highAccuRangeHandle->scratchPad;
	for( i = 0; i < (int32_t)highAccuRangeHandle->fft1DSize; i++ )
	{
		f2input		=	_amem8_f2(inputPtr++);
		f2input		=	_dmpysp(f2input, f2input);
		ftemp		=	_hif2(f2input) + _lof2(f2input);
		powerPtr[i] =	ftemp;
		totalPower	+=	ftemp;
		if( max < ftemp )
		{
			max			=	ftemp;
			coarseRangeInd	=	i;
		}
	}
	
	i			=	coarseRangeInd;
	sigPower	=	powerPtr[i-2] + powerPtr[i-1] + powerPtr[i] + powerPtr[i+1] + powerPtr[i+2];

	*estLinearSNR	=	divsp((float)((int32_t)highAccuRangeHandle->fft1DSize  - (int32_t)highAccuRangeHandle->skipLeft - (int32_t)highAccuRangeHandle->skipRight- 5) * sigPower, (totalPower - sigPower));

	/* zoom in FFT: assuming always size of fft1DSize x fft1DSize */
	zoomStartInd	=	coarseRangeInd - highAccuRangeHandle->numRangeBinZoomIn;
	zoomEndInd		=	coarseRangeInd + highAccuRangeHandle->numRangeBinZoomIn;
	indMask			=	highAccuRangeHandle->fft1DSize - 1;
	shift			=	30 - _norm(highAccuRangeHandle->fft1DSize);
	inputPtr		=	(__float2_t *)highAccuRangeHandle->inputSig;
	wncPtr			=	(__float2_t *)highAccuRangeHandle->wnCoarse;
	wnfPtr			=	(__float2_t *)highAccuRangeHandle->wnFine;
	max				=	0.f;
	itemp			=	0;
	currP			=	0.f;
	prevP			=	0.f;
	maxNextP		=	0.f;
	maxPrevP		=	0.f;
	for( i = zoomStartInd; i < zoomEndInd; i++)
	{
		for( j = 0; j < (int32_t)highAccuRangeHandle->fft1DSize; j ++)
		{
			tempFineSearchIdx	=	j;
			sigAcc  =	_ftof2(0.f, 0.f); 
			tempCoarseSearchIdx	=	0;
#ifdef _TMS320C6X
#pragma UNROLL(2);
#endif
			for( k = 0; k < (int32_t)highAccuRangeHandle->nSamplesPerChirp; k ++)
			{
#if 0
				tempIndFine1	=	tempFineSearchIdx & indMask;
				tempIndFine2	=	tempFineSearchIdx >> shift;
				tempFineSearchIdx		+=	j;
				tempIndCoarse	=	tempCoarseSearchIdx & indMask;
				tempCoarseSearchIdx		+=	i;

				f2input			=	_amem8_f2(&inputPtr[k]);
				wncoarse		=	_amem8_f2(&wncPtr[tempIndCoarse]);
				wnfine1			=	_amem8_f2(&wnfPtr[tempIndFine1]);
				wnfine2			=	_amem8_f2(&wncPtr[tempIndFine2]);

				f2temp			=	_complex_mpysp(f2input, wnfine1);
				f2temp			=	_complex_mpysp(f2temp, wnfine2);
				f2temp			=	_complex_mpysp(f2temp, wncoarse);
				sigAcc			=	_daddsp(sigAcc, f2temp);
#else
				tempIndFine1	=	tempFineSearchIdx & indMask;
				tempIndFine2	=	tempFineSearchIdx >> shift;
				tempFineSearchIdx		+=	j;
				tempIndCoarse	=	(tempCoarseSearchIdx + tempIndFine2) & indMask;
				tempCoarseSearchIdx		+=	i;

				f2input			=	_amem8_f2(&inputPtr[k]);
				wncoarse		=	_amem8_f2(&wncPtr[tempIndCoarse]);
				wnfine1			=	_amem8_f2(&wnfPtr[tempIndFine1]);

				f2temp			=	_complex_mpysp(f2input, wnfine1);
				f2temp			=	_complex_mpysp(f2temp, wncoarse);
				sigAcc			=	_daddsp(sigAcc, f2temp);
#endif
			}
			prevP				=	currP;
			f2temp				=	_dmpysp(sigAcc,sigAcc);
			ftemp				=	_hif2(f2temp) + _lof2(f2temp);
			currP				=	ftemp;
			if( max < ftemp)
			{
				max				=	ftemp;
				fineRangeInd	=	itemp;
				maxPrevP		=	prevP;
			}
			if(itemp == fineRangeInd + 1)
				maxNextP		=	currP;
			itemp++;
		}
	}

	interpIndx  =   0.5 * divdp((double)maxPrevP - (double)maxNextP, (double)maxPrevP + (double)maxNextP -2.0 * (double) max);

	fdelta			=	divdp((double)highAccuRangeHandle->maxBeatFreq, (double)highAccuRangeHandle->fft1DSize * (double)highAccuRangeHandle->fft1DSize);
	freqFineEst		=	fdelta * ((double)(zoomStartInd * highAccuRangeHandle->fft1DSize + fineRangeInd) + interpIndx); 

	*estRange		=	(float)divdp(freqFineEst * 3.0e8 * (double)highAccuRangeHandle->chirpRampTime, (2.0 * (double)highAccuRangeHandle->chirpBandwidth));

	if (highAccuRangeHandle->enablePhaseEst)
	{
		float phaseCoarseEst1, phaseCoarseEst2, phaseInitial;
		double PI = 3.14159265358979323846f;
		double real, imag, denom, initReal, initImag, dtemp1, dtemp2, dtemp;
		float phaseEst, totalPhase = 0.f, phaseCorrection, rangePhaseCorrection;
		__float2_t demodSig, corrSig;

		inputPtr			=	(__float2_t *)highAccuRangeHandle->inputSig;
		phaseCoarseEst1		=	2 * (float)PI * highAccuRangeHandle->fc * (divsp(2 * (*estRange), (float)3e8) + highAccuRangeHandle->adcStartTimeConst);
		phaseCoarseEst2		=	(float)PI * highAccuRangeHandle->chirpSlope * (divsp(2 * (*estRange), (float)3e8)  + highAccuRangeHandle->adcStartTimeConst) * (divsp(2 * (*estRange), (float)3e8)  + highAccuRangeHandle->adcStartTimeConst);
		phaseInitial		=	phaseCoarseEst1 - phaseCoarseEst2;

		denom				=	divdp(1.0, (double)highAccuRangeHandle->fft1DSize);
#if 0
		dtemp1				=	cos((double)phaseInitial);
		dtemp2				=	sin(-(double)phaseInitial);
		initReal			=	cos(2.0 * PI * highAccuRangeHandle->chirpRampTime * (double) freqFineEst * denom) ;
		initImag			=	sin(-2.0 * PI * highAccuRangeHandle->chirpRampTime * (double) freqFineEst * denom);
#else
		dtemp1				=	cosdp_i((double)phaseInitial);
		dtemp2				=	sindp_i(-(double)phaseInitial);
		initReal			=	cosdp_i(2.0 * PI * highAccuRangeHandle->chirpRampTime * (double) freqFineEst * denom) ;
		initImag			=	sindp_i(-2.0 * PI * highAccuRangeHandle->chirpRampTime * (double) freqFineEst * denom);
#endif
		
		//sample @ t = 0;
		corrSig				=	_ftof2((float)dtemp1, (float)dtemp2);
		demodSig			=	_complex_mpysp(_amem8_f2(inputPtr++), corrSig);
		RADARDEMO_atan((cplxf_t *)&demodSig, &phaseEst);
		totalPhase			+=	phaseEst;

		if ((highAccuRangeHandle->enableFilter == 0) && (highAccuRangeHandle->enableLinearFit == 0))
		{
			//sample @ t = 1;
			dtemp			=	dtemp1 * initReal - dtemp2 * initImag;
			imag			=	dtemp1 * initImag + dtemp2 * initReal;
			real			=	dtemp;
			corrSig				=	_ftof2((float)real, (float)imag);
			demodSig			=	_complex_mpysp(_amem8_f2(inputPtr++), corrSig);
			RADARDEMO_atan((cplxf_t *)&demodSig, &phaseEst);
			totalPhase			+=	phaseEst;

			for( j = 2; j < (int32_t)highAccuRangeHandle->fft1DSize; j ++)
			{
				dtemp			=	real * initReal - imag * initImag;
				imag			=	real * initImag + imag * initReal;
				real			=	dtemp;
				corrSig			=	_ftof2((float)real, (float)imag);
				demodSig		=	_complex_mpysp(_amem8_f2(inputPtr++), corrSig);
				RADARDEMO_atan((cplxf_t *)&demodSig, &phaseEst);
				totalPhase		+=	phaseEst;
			}
			phaseCorrection		=	totalPhase * (float)denom;
			rangePhaseCorrection= divsp((phaseCorrection*(float)3e8), (4.f * (float) PI * highAccuRangeHandle->fc));
			*estRange			+=	rangePhaseCorrection;
		}
		else
		{
			//Not implemented yet
		}
	}
	return;
}

#endif



/*!
   \fn     RADARDEMO_highAccuRangeProc_accumulateInputGeneric

   \brief   Accumulate signals from all chirps and convert to floating-point format with 1D windowing.

   \param[in]    nSamplesPerChirp
               Number of samples per chirp.

   \param[in]    fftSize1D
               1D FFT size.

   \param[in]    nChirpPerFrame
               Number of chirps per frame.

   \param[in]    fftWin1D
               Input pointer to 1D window function.
			   
   \param[in]    fftWin1DSize
               Number of ADC bits.

   \param[in]    inputPtr
               Pointer to input signal.

   \param[in]    chirpInd
               Flag to enable DC adjustment for ADC samples when set to 1. Otherwise, no preprocessing for ADC samples.

   \param[out]    outputPtr
               Pointer to the output signal.

   \pre       none

   \post      none


 */

void	RADARDEMO_highAccuRangeProc_accumulateInputGeneric(
                            IN  uint32_t nSamplesPerChirp,
                            IN  uint32_t fftSize1D,
                            IN  uint32_t nChirpPerFrame,
                            IN  float   * RESTRICT fftWin1D,
							IN 	int16_t  fftWin1DSize,
							IN  cplx16_t * RESTRICT inputPtr,
							IN  int32_t  chirpInd,
							OUT float * outputPtr)
{
	int32_t i;
	float * tempOutputPtr;

	if (chirpInd == 0)
	{
		for( i = (int32_t)nSamplesPerChirp - 1; i >= 0; i--)
		{
			*outputPtr++					=	(float)inputPtr->real;
			*outputPtr++					=	(float)inputPtr->imag;
			inputPtr++;
		}
	}
	else
	{
		tempOutputPtr		=	outputPtr;
		for( i = (int32_t)nSamplesPerChirp - 1; i >= 0; i--)
		{
			*tempOutputPtr++					+=	(float)inputPtr->real;
			*tempOutputPtr++					+=	(float)inputPtr->imag;
			inputPtr++;
		}

		if (chirpInd == ((int32_t)nChirpPerFrame - 1))
		{
			float fwin1D;
			float *tempOutputPtr;

			tempOutputPtr	=	&outputPtr[2*nSamplesPerChirp - 1];

			for( i = fftWin1DSize - 1; i >= 0; i--)
			{
				fwin1D						=	*fftWin1D++;
				*outputPtr					=	*outputPtr * fwin1D;
				outputPtr++;
				*outputPtr					=	*outputPtr * fwin1D;
				outputPtr++;
				*tempOutputPtr				=	*tempOutputPtr * fwin1D;
				tempOutputPtr--;
				*tempOutputPtr				=	*tempOutputPtr * fwin1D;
				tempOutputPtr--;
			}
			if (((int32_t)fftSize1D - (int32_t)nSamplesPerChirp) > 0)
			{
				tempOutputPtr	=	&outputPtr[2*nSamplesPerChirp];
				for( i = 0; i < ((int32_t)fftSize1D - (int32_t)nSamplesPerChirp); i++);
				{
					*tempOutputPtr++	=	0.f;
					*tempOutputPtr++	=	0.f;
				}
			}
		}
	}
}


/*!
   \fn     RADARDEMO_highAccuRangeProc_rangeEstGeneric

   \brief   Windowing function for 2D FFT and prepare for the input.

   \param[in]    procStep
               processing step, if 1, do preprocessing of the estimation, up to coarse range estimation, if 2, do the rest of estimation (fine freq and phase).

   \param[in]    highAccuRangeHandle
               Handle to the module.

   \param[out]    estRange
               Estimated range.

   \param[out]    deltaPhaseEst
               Estimated delta phase.

   \param[out]    estLinearSNR
               Estimated SNR.

   \pre       none

   \post      none


 */

void	RADARDEMO_highAccuRangeProc_rangeEstGeneric(
							IN int8_t procStep,
                            IN  RADARDEMO_highAccuRangeProc_handle *highAccuRangeHandle,
							OUT  float * estRange,
							OUT  float * deltaPhaseEst,
							OUT float * estLinearSNR)
{
	if (procStep == 1)
	{
		int32_t			i, j, rad1D, coarseRangeInd;
		unsigned char	* brev = NULL;
		float			totalPower, max, ftemp, sigPower, * RESTRICT powerPtr;
		float		finput, * RESTRICT inputPtr, * RESTRICT inputPtr1;

		j  = highAccuRangeHandle->log2fft1DSize;
		if ((j & 1) == 0)
			rad1D = 4;
		else
			rad1D = 2;

		/* copy to scratch is needed because FFT function will corrupt the input. We need to preserve if for zoom-in FFT */
		inputPtr	=	(float *) highAccuRangeHandle->inputSig;
		inputPtr1	=	(float *) &highAccuRangeHandle->scratchPad[2 * highAccuRangeHandle->fft1DSize];
		for (i = 0; i < (int32_t)highAccuRangeHandle->fft1DSize; i++ )
		{
			*inputPtr1++		=	*inputPtr++;
			*inputPtr1++		=	*inputPtr++;
		}

		inputPtr1	=	(float *) &highAccuRangeHandle->scratchPad[2 * highAccuRangeHandle->fft1DSize];
#if 0
		DSPF_sp_fftSPxSP_cn (
				highAccuRangeHandle->fft1DSize,
				(float*) inputPtr1,
				(float *)highAccuRangeHandle->twiddle, 
				highAccuRangeHandle->fft1DOutSig,
				brev,
				rad1D,
				0,
				highAccuRangeHandle->fft1DSize);
#endif

		dft(highAccuRangeHandle->fft1DSize, (float*) inputPtr1, highAccuRangeHandle->fft1DOutSig);

		inputPtr1	=	(float *) &highAccuRangeHandle->fft1DOutSig[0];
		for( i = 0; i < (int32_t)highAccuRangeHandle->skipLeft;  i++ )
		{
			*inputPtr1++	=	0.f;
			*inputPtr1++	=	0.f;
		}
		inputPtr1	=	(float *) &highAccuRangeHandle->fft1DOutSig[2*((int32_t)highAccuRangeHandle->fft1DSize- (int32_t)highAccuRangeHandle->skipRight)];
		for( i = (int32_t)highAccuRangeHandle->fft1DSize- (int32_t)highAccuRangeHandle->skipRight; i < (int32_t)highAccuRangeHandle->fft1DSize; i++ )
		{
			*inputPtr1++	=	0.f;
			*inputPtr1++	=	0.f;
		}
		max			=	0.f;
		totalPower	=	0.f;
		coarseRangeInd	=	0;
		inputPtr	=	(float *)highAccuRangeHandle->fft1DOutSig;
		powerPtr	=	(float *)highAccuRangeHandle->scratchPad;
		for( i = 0; i < (int32_t)highAccuRangeHandle->fft1DSize; i++ )
		{
			finput		=	(*inputPtr) * (*inputPtr);
			inputPtr++;
			finput		+=	(*inputPtr) * (*inputPtr);
			inputPtr++;
			ftemp		=	finput;
			powerPtr[i] =	ftemp;
			totalPower	+=	ftemp;
			if( max < ftemp )
			{
				max			=	ftemp;
				coarseRangeInd	=	i;
			}
		}
	
		highAccuRangeHandle->coarseRangeInd = coarseRangeInd;
		i			=	coarseRangeInd;
		sigPower	=	powerPtr[i-2] + powerPtr[i-1] + powerPtr[i] + powerPtr[i+1] + powerPtr[i+2];

		*estLinearSNR	=	((float)((int32_t)highAccuRangeHandle->fft1DSize  - (int32_t)highAccuRangeHandle->skipLeft - (int32_t)highAccuRangeHandle->skipRight- 5) * sigPower) / (totalPower - sigPower);
	}
	else if (procStep == 2)
	{
		int32_t			i, j, k, coarseRangeInd = highAccuRangeHandle->coarseRangeInd;
		float			max, ftemp;

		int32_t			zoomStartInd, zoomEndInd, itemp, tempIndFine1, tempIndFine2, tempIndCoarse, indMask, shift;
		int32_t			fineRangeInd, tempFineSearchIdx, tempCoarseSearchIdx;
		float		*RESTRICT inputPtr, *RESTRICT wncPtr, *RESTRICT wnfPtr, sigAccReal, sigAccImag;
		float			freqFineEst, fdelta, finputReal, finputImag, tempReal, tempImag;

		/* zoom in FFT: assuming always size of fft1DSize x fft1DSize */
		zoomStartInd	=	coarseRangeInd - highAccuRangeHandle->numRangeBinZoomIn;
		zoomEndInd		=	coarseRangeInd + highAccuRangeHandle->numRangeBinZoomIn;
		indMask			=	highAccuRangeHandle->fft1DSize - 1;
		shift			=	highAccuRangeHandle->log2fft1DSize;
		inputPtr		=	(float *)highAccuRangeHandle->inputSig;
		wncPtr			=	(float *)highAccuRangeHandle->wnCoarse;
		wnfPtr			=	(float *)highAccuRangeHandle->wnFine;
		max				=	0.0;
		itemp			=	0;
		for( i = zoomStartInd; i < zoomEndInd; i++)
		{
			for( j = 0; j < (int32_t)highAccuRangeHandle->fft1DSize; j ++)
			{
				tempFineSearchIdx	=	j;
				sigAccReal  =	0.f; 
				sigAccImag  =	0.f; 
				tempCoarseSearchIdx	=	0;
				for( k = 0; k < (int32_t)highAccuRangeHandle->nSamplesPerChirp; k ++)
				{
					tempIndFine1	=	tempFineSearchIdx & indMask;
					tempIndFine2	=	tempFineSearchIdx >> shift;
					tempFineSearchIdx		+=	j;
					tempIndCoarse	=	(tempCoarseSearchIdx + tempIndFine2) & indMask;
					tempCoarseSearchIdx		+=	i;

					finputReal		=	inputPtr[2 * k];
					finputImag		=	inputPtr[2 * k + 1];
					tempReal		=	wncPtr[2 * tempIndCoarse + 1] * finputReal - wncPtr[2 * tempIndCoarse] * finputImag;
					tempImag		=	wncPtr[2 * tempIndCoarse] * finputReal + wncPtr[2 * tempIndCoarse + 1] * finputImag;

					sigAccReal		+=	wnfPtr[2 * tempIndFine1 + 1] * tempReal - wnfPtr[2 * tempIndFine1] * tempImag;
					sigAccImag		+=	wnfPtr[2 * tempIndFine1] * tempReal + wnfPtr[2 * tempIndFine1 + 1] * tempImag;
				}
				ftemp				=	sigAccReal * sigAccReal + sigAccImag * sigAccImag;
				if( max < ftemp)
				{
					max				=	ftemp;
					fineRangeInd	=	itemp;
				}
				itemp++;
			}
		}

		fdelta			=	(highAccuRangeHandle->maxBeatFreq) / ((float)highAccuRangeHandle->fft1DSize * (float)highAccuRangeHandle->fft1DSize);
		freqFineEst		=	fdelta * (float)(zoomStartInd * highAccuRangeHandle->fft1DSize + fineRangeInd); 

		*estRange		=	(freqFineEst * (float)3e8 * highAccuRangeHandle->chirpRampTime) / (2.f * highAccuRangeHandle->chirpBandwidth);
#if 0
		if (highAccuRangeHandle->enablePhaseEst)
		{
			float phaseCoarseEst1, phaseCoarseEst2, phaseInitial;
			double PI = 3.14159265358979323846f;
			double real, imag, denom, initReal, initImag, dtemp1, dtemp2, dtemp;
			float phaseEst, totalPhase = 0.f, phaseCorrection, rangePhaseCorrection;
			__float2_t demodSig, corrSig;

			inputPtr			=	(__float2_t *)highAccuRangeHandle->inputSig;
			phaseCoarseEst1		=	2 * (float)PI * highAccuRangeHandle->fc * (divsp(2 * (*estRange), (float)3e8) + highAccuRangeHandle->adcStartTimeConst);
			phaseCoarseEst2		=	(float)PI * highAccuRangeHandle->chirpSlope * (divsp(2 * (*estRange), (float)3e8)  + highAccuRangeHandle->adcStartTimeConst) * (divsp(2 * (*estRange), (float)3e8)  + highAccuRangeHandle->adcStartTimeConst);
			phaseInitial		=	phaseCoarseEst1 - phaseCoarseEst2;

			denom				=	divdp(1.0, (double)highAccuRangeHandle->fft1DSize);
	#if 0
			dtemp1				=	cos((double)phaseInitial);
			dtemp2				=	sin(-(double)phaseInitial);
			initReal			=	cos(2.0 * PI * highAccuRangeHandle->chirpRampTime * (double) freqFineEst * denom) ;
			initImag			=	sin(-2.0 * PI * highAccuRangeHandle->chirpRampTime * (double) freqFineEst * denom);
	#else
			dtemp1				=	cosdp_i((double)phaseInitial);
			dtemp2				=	sindp_i(-(double)phaseInitial);
			initReal			=	cosdp_i(2.0 * PI * highAccuRangeHandle->chirpRampTime * (double) freqFineEst * denom) ;
			initImag			=	sindp_i(-2.0 * PI * highAccuRangeHandle->chirpRampTime * (double) freqFineEst * denom);
	#endif
		
			//sample @ t = 0;
			corrSig				=	_ftof2((float)dtemp1, (float)dtemp2);
			demodSig			=	_complex_mpysp(_amem8_f2(inputPtr++), corrSig);
			RADARDEMO_atan((cplxf_t *)&demodSig, &phaseEst);
			totalPhase			+=	phaseEst;

			if ((highAccuRangeHandle->enableFilter == 0) && (highAccuRangeHandle->enableLinearFit == 0))
			{
				//sample @ t = 1;
				dtemp			=	dtemp1 * initReal - dtemp2 * initImag;
				imag			=	dtemp1 * initImag + dtemp2 * initReal;
				real			=	dtemp;
				corrSig				=	_ftof2((float)real, (float)imag);
				demodSig			=	_complex_mpysp(_amem8_f2(inputPtr++), corrSig);
				RADARDEMO_atan((cplxf_t *)&demodSig, &phaseEst);
				totalPhase			+=	phaseEst;

				for( j = 2; j < (int32_t)highAccuRangeHandle->fft1DSize; j ++)
				{
					dtemp			=	real * initReal - imag * initImag;
					imag			=	real * initImag + imag * initReal;
					real			=	dtemp;
					corrSig			=	_ftof2((float)real, (float)imag);
					demodSig		=	_complex_mpysp(_amem8_f2(inputPtr++), corrSig);
					RADARDEMO_atan((cplxf_t *)&demodSig, &phaseEst);
					totalPhase		+=	phaseEst;
				}
				phaseCorrection		=	totalPhase * (float)denom;
				rangePhaseCorrection= divsp((phaseCorrection*(float)3e8), (4.f * (float) PI * highAccuRangeHandle->fc));
				*estRange			+=	rangePhaseCorrection;
			}
			else
			{
				//Not implemented yet
			}
		}
#endif
	}
	return;
}
