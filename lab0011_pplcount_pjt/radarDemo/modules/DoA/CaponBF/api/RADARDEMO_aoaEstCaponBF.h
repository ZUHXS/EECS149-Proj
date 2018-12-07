/*!
 *  \file   RADARDEMO_aoaEstCaponBF.h
 *
 *  \brief   Header file for RADARDEMO_aoaEstCaponBF module
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

#ifndef RADARDEMO_AOACAPONESTBF_H
#define RADARDEMO_AOACAPONESTBF_H


//! \brief include file <swpform.h>
//!
#include "swpform.h"
//! \brief include file <radarOsal_malloc.h>
//!
#include <modules/utilities/radarOsal_malloc.h>


//!  \brief   Error code for BF AoA estimation module.
//!
typedef enum
{
    RADARDEMO_AOACAPONBF_NO_ERROR = 0,                   /**< no error */
    RADARDEMO_AOACAPONBF_ANTSPACE_NOTSUPPORTED,          /**< normalized antenna spacing non supported */
    RADARDEMO_AOACAPONBF_NUMANT_NOTSUPPORTED,            /**< number of antennas not supported */
    RADARDEMO_AOACAPONBF_FAIL_ALLOCATE_HANDLE,           /**< RADARDEMO_aoAEstBF_create failed to allocate handle */
    RADARDEMO_AOACAPONBF_FAIL_ALLOCATE_LOCALINSTMEM, /**< RADARDEMO_aoAEstBF_create failed to allocate memory for buffers in local instance  */
    RADARDEMO_AOACAPONBF_INOUTPTR_NOTCORRECT,         /**< input and/or output buffer for RADARDEMO_aoAEstBF_run are either NULL, or not aligned properly  */
    RADARDEMO_AOACAPONBF_NUM_PEAK_EXCEED_MAX              /**< the number of detected angle exceed the max value */
} RADARDEMO_aoaEstCaponBF_errorCode;


//!  \brief   Structure element of the list of descriptors for RADARDEMO_aoaEstCaponBF configuration.
//!
typedef struct _RADARDEMO_aoaEstCaponBF_config_
{
    uint8_t       * antSpacing;             /**< antenna spacing in unit of lambda/2, size of nRxAnt, only support uniform linear array withpout holes in between antennas.*/
    uint32_t      nRxAnt;                   /**< number of receive antennas, only support 4 and 8 antennas now!!!.*/
    uint32_t      dopplerFFTSize;           /**< Size of Doppler FFT.*/
    uint32_t      numInputRangeBins;        /**< number of input range bins to be processed.*/
    float        estAngleResolution;             /**< Estimation resolution in degree.*/
    float        estAngleRange;                  /**< Range of the estimation, from -estRange to +estRange degrees*/
    float        maxOutputVar;              /**< Maximum variance range (in degree) for the output estimation. Estimates with bigger confidence range will not be reported.*/
    float         gamma;                    //!< Diagnol loading scaling factor
} RADARDEMO_aoaEstCaponBF_config;


/**
 *  \struct   _RADARDEMO_aoAEstCaponBF_input_
 *
 *  \brief   Structure element of the list of descriptors for RADARDEMO_aoAEstCaponBF module input.
 *
 *
 */

typedef struct _RADARDEMO_aoAEstCaponBF_input_
{
	uint32_t      rangeIndx;	            /**< Index to the current range bin to be processed. */
    uint32_t      azimuthIndx;              /**< Index to the azimuth bin, this is only used for doppler estimation.*/
	uint8_t		  fallBackToConvBFFlag;     /**< Flag to indicate falling back to conventional BF using covariance matrix if set to 1. */
	uint8_t		  processingStepSelector;   /**< Flag to select which processing to be done, if set to 0, construct rangeAzimuthHeatMap, if set to 1, estimate doppler for detected points. */
	uint8_t       clutterRemovalFlag;       /**< flag to indicate clutter removal needed. */
    uint32_t      nChirps;                  /**< number of chirps to be used for covariance matrix estimation.*/
    float          bwDemon;				    /**< denominator of the beamweight.*/
    cplx16_t      * inputAntSamples;        /**< input samples after range processing for rangeBin rangeIndx, array in format of (nRxAnt * nChirps).*/
} RADARDEMO_aoAEstCaponBF_input;


/**
 *  \struct   _RADARDEMO_aoAEstCaponBF_output_
 *
 *  \brief   Structure element of the list of descriptors for RADARDEMO_aoAEstCaponBF module output.
 *
 *
 */

typedef struct _RADARDEMO_aoAEstCaponBF_output_
{
    float      * rangeAzimuthHeatMap;       /**< output range azimuth heatmap, array in format of (numInputRangeBins * nAzimuthBins).*/
    uint32_t   dopplerIdx;                /**< Estimated Doppler index.*/
    float	   angleEst;                  /**< Angle estimation.*/
    float	   angleVarEst;               /**< Angle estimation variance.*/
} RADARDEMO_aoAEstCaponBF_output;


/*!
 *   \fn     RADARDEMO_aoaEstimationBF_create
 *
 *   \brief   Create and initialize RADARDEMO_aoaEstimationBF module.
 *
 *   \param[in]    moduleConfig
 *
 *   \param[out]    errorCode
 *
 *   \ret     void pointer to the module handle
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern void * RADARDEMO_aoaEstCaponBF_create(
                            IN  RADARDEMO_aoaEstCaponBF_config * moduleConfig,
                            OUT RADARDEMO_aoaEstCaponBF_errorCode * errorCode);

/*!
 *   \fn     RADARDEMO_aoaEstimationBF_delete
 *
 *   \brief   Delete RADARDEMO_aoaEstimationBF module.
 *
 *   \param[in]    handle
 *               Module handle.
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern void RADARDEMO_aoaEstCaponBF_delete(
                            IN  void * handle);


/*!
 *   \fn     RADARDEMO_aoaEstCaponBF_run
 *
 *   \brief   Estimate the angle of arrival of each detected object using BF.
 *
 *   \param[in]    handle
 *               Module handle.
 *
 *   \param[in]    input
 *               Input antenna signal and noise power for the detected object.
 *
 *   \param[out]    estOutput
 *               Pointer to the estimation output.
 *   \ret  error code
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern RADARDEMO_aoaEstCaponBF_errorCode RADARDEMO_aoaEstCaponBF_run(
                            IN  void * handle,
                            IN  RADARDEMO_aoAEstCaponBF_input * input,
                            OUT RADARDEMO_aoAEstCaponBF_output   * estOutput);

#endif //RADARDEMO_AOACAPONESTBF_H

