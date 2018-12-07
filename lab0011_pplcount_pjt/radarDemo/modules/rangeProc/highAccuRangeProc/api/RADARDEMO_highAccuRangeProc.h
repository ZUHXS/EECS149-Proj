/*! 
 *  \file   RADARDEMO_highAccuRangeProc.h
 *
 *  \brief   Header file for RADARDEMO_highAccuRangeProc module
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

#ifndef RADARDEMO_HIGHACCURANGEPROC_H
#define RADARDEMO_HIGHACCURANGEPROC_H

#include <swpform.h>
#include <modules/utilities/radarOsal_malloc.h>
#include <math.h>

typedef enum
{
	RADARDEMO_HIGHACCURANGEPROC_NO_ERROR = 0,				/**< no error */
	RADARDEMO_HIGHACCURANGEPROC_FAIL_ALLOCATE_HANDLE,		/**< RADARDEMO_highAccuRangeProc_create failed to allocate handle */ 
	RADARDEMO_HIGHACCURANGEPROC_FAIL_ALLOCATE_LOCALINSTMEM,	/**< RADARDEMO_highAccuRangeProc_create failed to allocate memory for buffers in local instance  */ 
	RADARDEMO_HIGHACCURANGEPROC_INOUTPTR_NOTCORRECT	    /**< input and/or output buffer for RADARDEMO_highAccuRangeProc_run are either NULL, or not aligned properly  */
} RADARDEMO_highAccuRangeProc_errorCode;

/**
 *  \struct   _RADARDEMO_highAccuRangeProc_input_
 *   {
 *   	cplx16_t     *inputSignal;  		
 *		uint32_t     chirpNumber;	
 *   }   RADARDEMO_highAccuRangeProc_input;
 *
 *  \brief   Structure for input to RADARDEMO_highAccuRangeProc module.
 *
 *
 */

typedef struct _RADARDEMO_highAccuRangeProc_input_
{
	cplx16_t     *inputSignal;  				/**< Input signal from ADC*/
	int32_t     chirpNumber;					/**< chirp number: 0 to number of chirps per frame. If set to negative number, then input is accumulated signal over all chirps within the frame.*/
} RADARDEMO_highAccuRangeProc_input;

typedef struct _RADARDEMO_highAccuRangeProc_output_
{
	float     rangeEst;  				/**< Range estimation from fine frequency estimation
										 if config.enablePhaseEst = 0, this is from zoomed in FFT only
										 if config.enablePhaseEst = 1, this is from zoomed in FFT plus delta phase correction.*/
	float     deltaPhaseEst;			/**< delta phase estimation, only valid when enablePhaseEst is set in config.*/
	float     linearSNREst;				/**< Estimated linear SNR.*/
} RADARDEMO_highAccuRangeProc_output;


typedef struct _RADARDEMO_highAccuRangeProc_config_
{
	uint32_t     fft1DSize;  					/**< 1D FFT size*/
	uint32_t     nSamplesPerChirp;				/**< number of samples per chirp*/
	uint32_t     numChirpsPerFrame;				/**< number of chirp per frame*/
	float        maxBeatFreq;  	    			/**< maximum beat frequency*/
	float      chirpBandwidth;  	    		/**< chirp bandwidth.*/
	float      chirpRampTime;  	    			/**< chirp ramp duration.*/
	float      fc;  	    					/**< chirp start frequency.*/
	float      chirpSlope;  	    			/**< chirp slope.*/
	float      adcStartTimeConst;  	    		/**< ADC start constant.*/
	float      adcSampleRate;  	    			/**< ADC sampling rate.*/
	float      * win1D;  	    	            /**< pointer to 1D windowing function.*/
	int16_t      win1DLength;  	    	        /**< half length of the 1D windowing function.*/
	uint8_t      numRangeBinZoomIn;  	        /**< number of bins to zoom in for frequenc estimation.*/
	uint8_t      enablePhaseEst;  				/**< enable estimation using phase correction.*/
	uint8_t      enableLinearFit;  				/**< enable linear fit in phase estimation.*/
	uint8_t      enableFilter;  				/**< enable filtering in phase estimation.*/
	uint8_t      skipLeft;						/**< number of samples to skip from the left.*/
	uint8_t      skipRight;						/**< number of samples to skip from the right.*/
	float      * fft1DIn;  	    	        /**< pointer to 1D FFT input.*/
} RADARDEMO_highAccuRangeProc_config;


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

extern void	* RADARDEMO_highAccuRangeProc_create(
                            IN  RADARDEMO_highAccuRangeProc_config * moduleConfig, 
							OUT RADARDEMO_highAccuRangeProc_errorCode * errorCode);

/*! 
   \fn     RADARDEMO_highAccuRangeProc_delete
 
   \brief   Delete RADARDEMO_highAccuRangeProc module. 
  
   \param[in]    handle
               Module handle.
			   
   \pre       none
 
   \post      none
  
 
 */

extern void	RADARDEMO_highAccuRangeProc_delete(
                            IN  void * handle);


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
							OUT RADARDEMO_highAccuRangeProc_output * rangeProcOutput);

extern void tw_gen_float(float *w, int n);

#endif //RADARDEMO_RANGEPROC_H

