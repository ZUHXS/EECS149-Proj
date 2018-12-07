/*! 
 *  \file   RADARDEMO_highAccuRangeProc.c
 *
 *  \brief   Range processing. 
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

#include <modules/rangeProc/highAccuRangeProc/api/RADARDEMO_highAccuRangeProc.h>
#include "RADARDEMO_highAccuRangeProc_priv.h"

#include <stdio.h>

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif




/*! 
   \fn     RADARDEMO_highAccuRangeProc_create
 
   \brief   Create and initialize RADARDEMO_highAccuRangeProc module. 
  
   \param[in]    moduleConfig
               Pointer to input configurations structure for RADARDEMO_highAccuRangeProc module.
			   
   \param[in]    errorCode
               Pointer to error code.
			   
   \ret     void pointer to the module handle. Return value of NULL indicates failed module creation.
			   
   \pre       none
 
   \post      none
  
 
 */

void	* RADARDEMO_highAccuRangeProc_create(
                            IN  RADARDEMO_highAccuRangeProc_config * moduleConfig, 
							OUT RADARDEMO_highAccuRangeProc_errorCode * errorCode)
							
{
	int32_t     i, itemp;
	double		real, imag;
	double PI = 3.14159265358979323846;
	double		denom;
	RADARDEMO_highAccuRangeProc_handle * handle;
	
	*errorCode		=	RADARDEMO_HIGHACCURANGEPROC_NO_ERROR;

	handle						=	(RADARDEMO_highAccuRangeProc_handle *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_highAccuRangeProc_handle), 1);
	if (handle == NULL)
	{
		*errorCode	=	RADARDEMO_HIGHACCURANGEPROC_FAIL_ALLOCATE_HANDLE;
		return (handle);
	}
	
	handle->nSamplesPerChirp	=	moduleConfig->nSamplesPerChirp;
	handle->fft1DSize			=	moduleConfig->fft1DSize;
	handle->numChirpsPerFrame	=	moduleConfig->numChirpsPerFrame;
	handle->win1DLength			=	moduleConfig->win1DLength;
	handle->maxBeatFreq			=	moduleConfig->maxBeatFreq;
	handle->chirpBandwidth		=	moduleConfig->chirpBandwidth;
	handle->chirpRampTime		=	moduleConfig->chirpRampTime;
	handle->fc					=	moduleConfig->fc;
	handle->chirpSlope			=	moduleConfig->chirpSlope;
	handle->adcStartTimeConst	=	moduleConfig->adcStartTimeConst;
	handle->adcSampleRate		=	moduleConfig->adcSampleRate;
	handle->numRangeBinZoomIn	=	moduleConfig->numRangeBinZoomIn;
	handle->enablePhaseEst		=	moduleConfig->enablePhaseEst;
	handle->enableLinearFit		=	moduleConfig->enableLinearFit;
	handle->enableFilter		=	moduleConfig->enableFilter;
	handle->skipLeft			=	moduleConfig->skipLeft;
	handle->skipRight			=	moduleConfig->skipRight;

	itemp						=	handle->fft1DSize;
	i							=	0;
	while(1)
    {
        if(itemp == 1) break;
        itemp = itemp >> 1;
        i++;
    }
	handle->log2fft1DSize		=	i;

	handle->win1D				=	(float *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, moduleConfig->win1DLength * sizeof(float), 8);
	if (handle->win1D == NULL)
	{
		*errorCode	=	RADARDEMO_HIGHACCURANGEPROC_FAIL_ALLOCATE_LOCALINSTMEM;
		return (handle);
	}

	for (i = 0; i < (int32_t)moduleConfig->win1DLength; i++ )
	{
		handle->win1D[i]		=	moduleConfig->win1D[i];
	}

	handle->twiddle			=	(float *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, 2 * handle->fft1DSize * sizeof(float), 8);
	if (handle->twiddle == NULL)
	{
		*errorCode	=	RADARDEMO_HIGHACCURANGEPROC_FAIL_ALLOCATE_LOCALINSTMEM;
		return (handle);
	}

	tw_gen_float(handle->twiddle, handle->fft1DSize);

	handle->inputSig		=	(float *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, 2 * handle->fft1DSize * sizeof(float), 8);
	if (handle->inputSig == NULL)
	{
		*errorCode	=	RADARDEMO_HIGHACCURANGEPROC_FAIL_ALLOCATE_LOCALINSTMEM;
		return (handle);
	}

	handle->scratchPad		=	(float *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 1, 2 * 2 * handle->fft1DSize * sizeof(float), 8);
	if (handle->scratchPad == NULL)
	{
		*errorCode	=	RADARDEMO_HIGHACCURANGEPROC_FAIL_ALLOCATE_LOCALINSTMEM;
		return (handle);
	}
	handle->fft1DOutSig		=	handle->scratchPad;
	handle->demodSig		=	handle->scratchPad;
	
	moduleConfig->fft1DIn 	= handle->inputSig;

	handle->wnCoarse		=	(float *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, 2 * handle->fft1DSize * sizeof(float), 8);
	if (handle->wnCoarse == NULL)
	{
		*errorCode	=	RADARDEMO_HIGHACCURANGEPROC_FAIL_ALLOCATE_LOCALINSTMEM;
		return (handle);
	}
	denom = 1.0/((double)handle->fft1DSize);
	for (i = 0; i < (int32_t)handle->fft1DSize; i++)
	{
		real		=	cos(-2 * PI * i *denom);
		imag		=	sin(-2 * PI * i *denom);
		handle->wnCoarse[2*i]		=	imag;
		handle->wnCoarse[2*i+ 1 ]	=	real;
	}

	handle->wnFine		=	(float *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, 2 * handle->fft1DSize * sizeof(float), 8);
	if (handle->wnFine == NULL)
	{
		*errorCode	=	RADARDEMO_HIGHACCURANGEPROC_FAIL_ALLOCATE_LOCALINSTMEM;
		return (handle);
	}
	denom = 1.0/ ((double)(handle->fft1DSize * handle->fft1DSize));
	for (i = 0; i < (int32_t)handle->fft1DSize; i++)
	{
		real		=	cos(-2 * PI * i *denom);
		imag		=	sin(-2 * PI * i *denom);
		handle->wnFine[2*i]		=	imag;
		handle->wnFine[2*i+ 1 ]	=	real;
	}

	return((void *)handle);
}

/*! 
   \fn     RADARDEMO_highAccuRangeProc_delete
 
   \brief   Delete RADARDEMO_highAccuRangeProc module. 
  
   \param[in]    handle
               Module handle.
			   
   \pre       none
 
   \post      none
  
 
 */

void	RADARDEMO_highAccuRangeProc_delete(
                            IN  void * handle)
{
	RADARDEMO_highAccuRangeProc_handle *rangeProcInst;
	
	rangeProcInst	=	(RADARDEMO_highAccuRangeProc_handle *) handle;

	radarOsal_memFree(rangeProcInst->inputSig, rangeProcInst->fft1DSize * 2 * sizeof(float));
	radarOsal_memFree(rangeProcInst->twiddle, rangeProcInst->fft1DSize * 2 * sizeof(float));
	radarOsal_memFree(rangeProcInst->wnFine, rangeProcInst->fft1DSize * 2 * sizeof(float));
	radarOsal_memFree(rangeProcInst->wnCoarse, rangeProcInst->fft1DSize * 2 * sizeof(float));
	radarOsal_memFree(rangeProcInst->win1D, rangeProcInst->win1DLength * 2 * sizeof(float));
	radarOsal_memFree(rangeProcInst->demodSig, rangeProcInst->nSamplesPerChirp * 2 * sizeof(float));
	
	radarOsal_memFree(handle, sizeof(RADARDEMO_highAccuRangeProc_handle));
}



/*! 
   \fn     RADARDEMO_highAccuRangeProc_run
 
   \brief   Range processing, always called per chirp per antenna.
  
   \param[in]    handle
               Module handle.
 
   \param[in]    rangeProcInput
               Input signal from ADC and corresponding chirp number.
 
   \param[out]    rangeProcOutput
               Outputs from range processing. 
 
   \return    errorCode
               Error code.
			   
	\pre       none
 
   \post      none
  
 
 */

RADARDEMO_highAccuRangeProc_errorCode	RADARDEMO_highAccuRangeProc_run(
                            IN  void * handle,
							IN  RADARDEMO_highAccuRangeProc_input * rangeProcInput, 
							OUT RADARDEMO_highAccuRangeProc_output * rangeProcOutput)

{
	int32_t		chirpNum;
	RADARDEMO_highAccuRangeProc_handle *rangeProcInst;
	cplx16_t    * inputSignal;
	RADARDEMO_highAccuRangeProc_errorCode errorCode = RADARDEMO_HIGHACCURANGEPROC_NO_ERROR;

	inputSignal		=	rangeProcInput->inputSignal;
	rangeProcInst	=	(RADARDEMO_highAccuRangeProc_handle *) handle;

	if (inputSignal == NULL)
		errorCode	=	RADARDEMO_HIGHACCURANGEPROC_INOUTPTR_NOTCORRECT;

	if (rangeProcInst->win1D == NULL)
		errorCode	=	RADARDEMO_HIGHACCURANGEPROC_INOUTPTR_NOTCORRECT;

	if (errorCode > RADARDEMO_HIGHACCURANGEPROC_NO_ERROR)
		return(errorCode);
	
	chirpNum		=	(int32_t) rangeProcInput->chirpNumber;
	if (chirpNum >= 0)
	{ /*accumulating all the chirps and convert to float*/
#ifndef ARMVERSION
		RADARDEMO_highAccuRangeProc_accumulateInput(
                            rangeProcInst->nSamplesPerChirp,
                            rangeProcInst->fft1DSize,
                            rangeProcInst->numChirpsPerFrame,
                            rangeProcInst->win1D,
							rangeProcInst->win1DLength,
							rangeProcInput->inputSignal,
							chirpNum,
							rangeProcInst->inputSig);
#else
		RADARDEMO_highAccuRangeProc_accumulateInputGeneric(
                            rangeProcInst->nSamplesPerChirp,
                            rangeProcInst->fft1DSize,
                            rangeProcInst->numChirpsPerFrame,
                            rangeProcInst->win1D,
							rangeProcInst->win1DLength,
							rangeProcInput->inputSignal,
							chirpNum,
							rangeProcInst->inputSig);
#endif
	}
	else
	{ 
		/* range measurements*/
#ifndef ARMVERSION
		RADARDEMO_highAccuRangeProc_rangeEst(
                            rangeProcInst, 
							&(rangeProcOutput->rangeEst),
							&(rangeProcOutput->deltaPhaseEst),
							&(rangeProcOutput->linearSNREst));
#else
		if (chirpNum == -1)
		{
			RADARDEMO_highAccuRangeProc_rangeEstGeneric(
							1,
                            rangeProcInst, 
							&(rangeProcOutput->rangeEst),
							&(rangeProcOutput->deltaPhaseEst),
							&(rangeProcOutput->linearSNREst));
		}
		else
		{
			RADARDEMO_highAccuRangeProc_rangeEstGeneric(
							2,
                            rangeProcInst, 
							&(rangeProcOutput->rangeEst),
							&(rangeProcOutput->deltaPhaseEst),
							&(rangeProcOutput->linearSNREst));
		}
#endif
	}
	return(errorCode);
}

