/*!
 *  \file   RADARDEMO_aoaEstCaponBF_priv.h
 *
 *  \brief   Header file for RADARDEMO_aoaEstCaponBF_priv.c
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
#ifndef RADARDEMO_AOAESTCAPONBF_PRIV_H
#define RADARDEMO_AOAESTCAPONBF_PRIV_H

//! \brief include file <swpform.h>
//!
#include <swpform.h>
#ifndef _TMS320C6600
#include <modules/utilities/radar_c674x.h>
#endif

#if 0
#ifndef _WIN32
#ifndef _TMS320C6600
#ifdef CCS
//#include "src/DSPF_sp_fftSPxSP/DSPF_sp_fftSPxSP.h"
#else
#include <DSPF_sp_fftSPxSP.h>
#endif
#else
#include <ti/dsplib/src/DSPF_sp_fftSPxSP/c66/DSPF_sp_fftSPxSP.h>
#endif
#endif
#endif

extern void DSPF_sp_fftSPxSP(int N, float *ptr_x, float *ptr_w, float *ptr_y, unsigned char *brev, int n_min, int offset, int n_max);

#define RADARDEMO_AOAESTBF_PIOVER180 (3.141592653589793/180.0)      //!< define the pi/180
#define RADARDEMO_AOAESTBF_PI        (3.141592653589793f)           //!< define pi


//!  \brief   Structure element of the list of descriptors for UL allocations.
//!
typedef struct _RADARDEMO_aoaEstCaponBF_handle
{
    uint32_t      nRxAnt;                   /**< number of receive antennas, only support 4 and 8 antennas now!!!.*/
    uint32_t      numInputRangeBins;        /**< number of input range bins to be processed.*/
    uint32_t      dopplerFFTSize;           /**< Size of Doppler FFT.*/
	float		*twiddle;					/**< Doppler FFT twiddle factor.*/
    float        estAngleResolution;             /**< Estimation resolution in degree.*/
    float        estAngleRange;                  /**< Range of the estimation, from -estRange to +estRange degrees*/
    float        maxOutputVar;              /**< Maximum variance range (in degree) for the output estimation. Estimates with bigger confidence range will not be reported.*/
    uint32_t      steeringVecSize;          //!< size of -estRange:estResolution:estRange
    cplxf_t       * steeringVec;            //!< steering vector for angle: -estRange:estResolution:estRange, for nRxAnt antennas, must be aligned to 8-byte boundary
                                            //!< The steeringVec is arranged in ... degree0ant1 degree0ant2 degree0ant3 degree1ant1 degree1ant2 degree1ant3 ... fashion
                                            //!< Ant0's steeringVec is 1 for all angle possiblities, so we don't save them
    uint32_t      * scratchPad;             //!< Pointer to the scratch memory used in this function, must of size:
											//!<8-antenna:max{2*DopplerFFTsize*8bytes, 360*4bytes}
											//!<4-antenna:max{2*DopplerFFTsize*8bytes, 82*4bytes}
    float         gamma;                    //!< Diagnol loading scaling factor
	cplxf_t		*invRnMatrices;				/**< Pointer to vovariance matrices memory, in the order number of range bins, and upper triangle of nRxAnt x nRxAnt Hermitian matrix.*/
} RADARDEMO_aoaEstCaponBF_handle;

/*!
 *   \fn     RADARDEMO_aoaEstCaponBF_covInv
 *
 *   \brief   Per range bin, estimate the covariance matrices from input 1D FFT results, and calculate the inverse of these matrices.
 *
 *   \param[in]    invFlag
 *               Flag to indicate matrix inversion will be performed. 
 *               If set to 1, output invRnMatrices will contain inversion of covariance matrices.
 *               If set to 0, output invRnMatrices will contain covariance matrices without inversion.
 *
 *   \param[in]    clutterRmFlag
 *               Flag to indicate clutter removal will be performed if set to 1. Otherwise, disabled.
 *
 *   \param[in]    gamma
 *               Scaling factor for diagnal loading.
 *
 *   \param[in]    nRxAnt
 *               number of antenna
 *
 *   \param[in]    nChirps
 *               number of input chirps
 *
 *   \param[in]    scratch
 *               scratch memory, must be of size of 360 32-bit words for 8 antennas, and 82 32-bit words for 4 antennas.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[in]    inputAntSamples
 *               input samples from radar cube (1D FFT output) for the current (one) range bin to be processed
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    invRnMatrices
 *               Output inverse of covariance matrices for the current range bin, in order of upper triangle of nRxAnt x nRxAnt Hermitian matrix.
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
extern void		RADARDEMO_aoaEstCaponBF_covInv(
				IN uint8_t invFlag,
				IN uint8_t clutterRmFlag,
				IN float gamma,
				IN int32_t nRxAnt,
				IN int32_t nChirps,
				IN int32_t * scratch,
				IN cplx16_t * inputAntSamples,
				OUT cplxf_t  * invRnMatrices);


/*!
 *   \fn     RADARDEMO_aoaEstCaponBF_heatmap
 *
 *   \brief   Use Capon beamforming to generate range azimuth heatmap per range bin.
 *
 *   \param[in]    bfFlag
 *               Flag to indicate which covariance matrix based beamforming will be performed. 
 *               If set to 1, Capon BF.
 *               If set to 0, conventional BF.
 *
 *   \param[in]    nRxAnt
 *               number of antenna
 *
 *   \param[in]    numAzimuthBins
 *               number of Azimuth bins
 *
 *   \param[in]    steeringVec
 *              steering vector for beamforming.
 *
 *   \param[in]    scratch
 *               scratch memory, must be of size TBD!!!
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[in]    invRnMatrices
 *               Output inverse of covariance matrices or the current range bin, in order of upper triangle of nRxAnt x nRxAnt Hermitian matrix.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    rangeAzimuthHeatMap
 *               Output range azimuth heatmap, in the format of numInputRangeBins by numAzimuthBins
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
extern void RADARDEMO_aoaEstCaponBF_heatmap(
				IN uint8_t bfFlag,
				IN int32_t  nRxAnt,
				IN int32_t  numAzimuthBins,
				IN cplxf_t * steeringVec,
				IN cplxf_t * invRnMatrices,
				OUT float  * rangeAzimuthHeatMap);



/*!
 *   \fn     RADARDEMO_aoaEstCaponBF_dopperEstInput
 *
 *   \brief   Use Capon beamforming to generate range azimuth heatmap.
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
 *               Input scratch memory. Must be size of nRxAnt * 2 32-bit word.
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
extern void		RADARDEMO_aoaEstCaponBF_dopperEstInput(
				IN uint8_t bfFlag, 
				IN int32_t nRxAnt,
				IN int32_t nChirps,
				IN cplx16_t * inputAntSamples,
				IN cplxf_t * steeringVec,
				IN cplxf_t  * invRnMatrices,
				IN int32_t * scratch,
				IN float rangeAzimuthHeatMap,
				OUT float * RESTRICT bfOutput
			);

extern void MATRIX_4x4_BWInversionfp (  
                             IN      cplxf_t  * RESTRICT Input,
                             OUT     cplxf_t  * output);


extern void MATRIX_Mult4x4fp (  
                             IN      cplxf_t * RESTRICT A,
                             IN      cplxf_t * RESTRICT B,
                             OUT     cplxf_t * RESTRICT C);

extern void MATRIX_single8x8MatInv (  
                             IN      cplxf_t * RESTRICT Input,
                             IN      int32_t * RESTRICT scratch,
                             OUT     cplxf_t  * output);

extern void tw_gen_float (float *w, int n);

							 
#endif //RADARDEMO_AOAESTCAPONBF_PRIV_H
