/*! 
 *  \file   RADARDEMO_highAccuRangeProc_priv.h
 *
 *  \brief   Header file for RADARDEMO_highAccuRangeProc module's internal functions
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

#ifndef RADARDEMO_RANGEPROC_RPIV_H
#define RADARDEMO_RANGEPROC_RPIV_H

#include <swpform.h>
#ifndef ARMVERSION
#include <ti\mathlib\src\atan2sp\c674\atan2sp_i.h>
#include <ti\mathlib\src\common\c674\common.h>
#include <ti\mathlib\src\cosdp\c674\cosdp_i.h>
#include <ti\mathlib\src\sindp\c674\sindp_i.h>

#ifndef _WIN32
#ifndef _TMS320C6600

#ifdef CCS
#include <src/DSPF_sp_fftSPxSP/DSPF_sp_fftSPxSP.h>
#else
#include <DSPF_sp_fftSPxSP.h>
#endif
#else

#include <ti/dsplib/src/DSPF_sp_fftSPxSP/c66/DSPF_sp_fftSPxSP.h>
#endif
#endif


#ifndef _TMS320C6600
#include <modules/utilities/radar_c674x.h>
#endif
#endif

#define RADARDEMO_HIGHACCURANGEPROC_MAXSIGWIN (10)

typedef struct _RADARDEMO_highAccuRangeProc_handle_
{
	float *        twiddle;                  	/**< Twiddle factors for 1DFFT.*/
	float *        wnCoarse;                  	/**< coarse level wn.*/
	float *        wnFine;                  	/**< fine level wn.*/

	float *        inputSig;                  	/**< buffer for input signal, accumulated over all chirps within the frame*/
	float *        demodSig;                  	/**< pointer to phase demodulation signal*/

	float *        scratchPad;					/**< Scratch memory for the module*/
	float *        fft1DOutSig;                 /**< pointer to 1D FFT output signal*/

	float        maxBeatFreq;  	    			/**< maximum beat frequency*/
	float      chirpBandwidth;  	    		/**< chirp bandwidth.*/
	float      chirpRampTime;  	    			/**< chirp ramp duration.*/
	float      fc;  	    					/**< chirp start frequency.*/
	float      chirpSlope;  	    			/**< chirp slope.*/
	float      adcStartTimeConst;  	    		/**< ADC start constant.*/
	float      adcSampleRate;  	    			/**< ADC sampling rate.*/
	int32_t		coarseRangeInd;					/**< Index of coarse frequency estimation.*/

	uint32_t     fft1DSize;  					/**< 1D FFT size*/
	uint32_t     log2fft1DSize;  				/**< log2 of 1D FFT size*/
	uint32_t     nSamplesPerChirp;				/**< number of samples per chirp*/
	uint32_t     numChirpsPerFrame;				/**< number of chirp per frame*/
	float      * win1D;  	    	            /**< pointer to 1D windowing function.*/
	int16_t      win1DLength;  	    	        /**< half length of the 1D windowing function.*/

	uint8_t      numRangeBinZoomIn;  	        /**< number of bins to zoom in for frequenc estimation.*/
	uint8_t      enablePhaseEst;  				/**< enable estimation using phase correction.*/
	uint8_t      enableLinearFit;  				/**< enable linear fit in phase estimation.*/
	uint8_t      enableFilter;  				/**< enable filtering in phase estimation.*/
	uint8_t      skipLeft;						/**< number of samples to skip from the left.*/
	uint8_t      skipRight;						/**< number of samples to skip from the right.*/

} RADARDEMO_highAccuRangeProc_handle;

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
                            IN  float  * fftWin1D,
							IN 	int16_t  fftWin1DSize,
							IN  cplx16_t * inputPtr,
							IN  int32_t  chirpInd,
							OUT float * outputPtr);
							

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
							OUT float * estLinearSNR);


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

extern void	RADARDEMO_highAccuRangeProc_rangeEstGeneric(
							IN int8_t procStep,
                            IN  RADARDEMO_highAccuRangeProc_handle *highAccuRangeHandle,
							OUT  float * estRange,
							OUT  float * deltaPhaseEst,
							OUT float * estLinearSNR);

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

extern void	RADARDEMO_highAccuRangeProc_accumulateInputGeneric(
                            IN  uint32_t nSamplesPerChirp,
                            IN  uint32_t fftSize1D,
                            IN  uint32_t nChirpPerFrame,
                            IN  float   * RESTRICT fftWin1D,
							IN 	int16_t  fftWin1DSize,
							IN  cplx16_t * RESTRICT inputPtr,
							IN  int32_t  chirpInd,
							OUT float * outputPtr);
#ifndef ARMVERSION
INLINE float divsp (float a, float b) {
  cmn_DIVSP (a,b);
}

INLINE double divdp (double a, double b) {
  cmn_DIVDP (a,b);
}

INLINE void RADARDEMO_atan (
				  IN cplxf_t * point,
				  OUT float *  angle)
{
	angle[0] = atan2sp_i(point[0].imag, point[0].real);
}
#endif

#ifdef _WIN32
extern void DSPF_sp_fftSPxSP (int N, float *ptr_x, float *ptr_w, float *ptr_y,
    unsigned char *brev, int n_min, int offset, int n_max);
#endif
#endif //RADARDEMO_RANGEPROC_RPIV_H

