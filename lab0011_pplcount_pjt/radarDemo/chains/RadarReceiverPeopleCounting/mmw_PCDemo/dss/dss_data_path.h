/**
 *   @file  dss_data_path.h
 *
 *   @brief
 *      This is the data path processing header.
 *
 *  Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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
#ifndef DSS_DATA_PATH_H
#define DSS_DATA_PATH_H

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/common/mmwave_error.h>
#include <chains/RadarReceiverPeopleCounting/mmw_PCDemo/common/mmw_config.h>
#include <ti/drivers/adcbuf/ADCBuf.h>
#include <ti/drivers/edma/edma.h>
#include <chains/RadarReceiverPeopleCounting/radarProcess.h>
#include <modules/utilities/radarOsal_malloc.h>
#include <modules/utilities/cycle_measure.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EDMA_INSTANCE_A 0
#define EDMA_INSTANCE_B 1

#define DSP_CLOCK_SPEED_IN_MHZ (600)
#define DSP_CLOCK_USEC_PER_CYCLE (0.0016666666f)

#define MMW_TM_DEMO_EDMASCATCHBUF_SIZE (0x4000)


extern radarOsal_heapObj gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS];

/*! @brief DSP cycle profiling structure to accumulate different
    processing times in chirp and frame processing periods */
typedef struct cycleLog_t_ {
    float chirpProcMarginCurrInusec; /*!< @brief average margin for chirp processing */
    float chirpProcMarginMaxInusec; /*!< @brief best margin for chirp processing */
    float chirpProcMarginMinInusec; /*!< @brief worst margin for chirp processing */
    float frameProcMarginCurrInusec; /*!< @brief margin for current frame processing */
    float frameProcMarginMaxInusec; /*!< @brief best margin for frame processing */
    float frameProcMarginMinInusec; /*!< @brief worst margin for frame processing */
  	uint32_t rangeAzimuthProcCycles; /*! @brief cycle cost for Doppler processing. */
  	uint32_t cfarProcCycles; /*! @brief cycle cost for detection CFAR. */
  	uint32_t dopplerEstCycles; /*! @brief cycle cost for DoA. */
} cycleLog_t;

typedef struct outputToARM_t_ {
	MmwDemo_detOutputHdr outputHeader;
	radarProcessOutputToTracker outputToTracker;
}outputToARM_t;



/**
 * @brief
 *  Millimeter Wave Demo Data Path Information.
 *
 * @details
 *  The structure is used to hold all the relevant information for
 *  the data path.
 */
typedef struct MmwDemo_DSS_DataPathObj_t
{
    /*! @brief   configuration struction for signal processing chain */
    radarProcessConfig_t   radarProcConfig;

    /*! @brief   Data struction for output to ARM */
    outputToARM_t *outputDataToArm;

    /*! @brief   handle to signal processing chain */
	void * radarProcessHandle;

    /*! @brief valid Profile index */
    uint32_t validProfileIdx;

    /*! @brief   Number of receive channels */
    uint32_t numPhyRxAntennas;

    /*! @brief   Number of virtual receive antennas */
    uint32_t numVirtRxAntennas;

    /*! @brief number of ADC samples */
    uint32_t numAdcSamples;

    /*! @brief number of chirps per frame*/
    uint32_t numChirpsPerFrame;

    /*! @brief number of range bins */
    uint32_t numRangeBins;

    /*! @brief number of doppler bins */
    uint32_t numAzimuthBins;

    /*! @brief number of transmit antennas */
    uint32_t numTxAntennas;

    /*! @brief   Flag to indicate using EDMA for doppler processing. If set to zero, then direct core access to radar cube and heatmap in L3 */
    uint8_t dopplerProcUsingEDMAFlag;

    /*! @brief   Chirp Threshold configuration used for ADCBUF driver */
    uint8_t chirpThreshold;

    /*! @brief   Chirp Available Semaphore: */
    Semaphore_Handle  chirpSemHandle;

    /*! @brief   Counter which tracks the number of chirps detected */
    uint32_t chirpInterruptCounter;

    /*! @brief   ADCBUF handle. */
    ADCBuf_Handle adcbufHandle;

    /*! @brief   Handle of the EDMA driver. */
    EDMA_Handle edmaHandle[2];

    /*! @brief   EDMA error Information when there are errors like missing events */
    EDMA_errorInfo_t  EDMA_errorInfo;

    /*! @brief EDMA transfer controller error information. */
    EDMA_transferControllerErrorInfo_t EDMA_transferControllerErrorInfo;

    /*! @brief ADCBUF Configuration */
    MmwDemo_ADCBufCfg       adcBufCfg;

    /*! @brief Semaphore handle for 1D EDMA completion. */
    Semaphore_Handle EDMA_1D_InputDone_semHandle[2];
    /*! @brief Semaphore handle for 1D EDMA completion. */
    Semaphore_Handle EDMA_1D_OutputDone_semHandle[2];
    /*! @brief Semaphore handle for 2D EDMA completion. */
    Semaphore_Handle EDMA_2D_InputDone_semHandle[2];
    /*! @brief Semaphore handle for Detection Matrix completion. */
    Semaphore_Handle EDMA_DetMatrixOutputDone_semHandle[2];

    /*! @brief pointer to ADC buffer */
    cmplx16ImRe_t *ADCdataBuf;

    /*! @brief ADCBUF input samples in L2 scratch memory, ping and pong */
    cmplx16ImRe_t *adcDataL2[2];

    /*! @brief range proc output pointer, ping and pong */
    int32_t  *rangeProcOut[2];

    /*! @brief Doppler proc input pointer, ping and pong */
    int32_t  *rangeAsimuthProcin[2];

    /*! @brief Doppler proc output pointer, ping and pong */
    float  *rangeAsimuthProcOut[2];

    /*! @brief Flag to indicate using Ping-pong buffer scheme for Doppler proc*/
    int8_t dopplerUsePingPong;

    /*! @brief ADCBUF input samples in L2 scratch memory */
    int32_t  *scratchBuf;

    /*! @brief 1D FFT output */
    cmplx16ImRe_t *fftOut1D;

     /*! @brief Time stamp for the end of the frame processing. */
    uint32_t frameProcDoneTimeStamp;

    /*! @brief Time stamp for the beginning of the frame. */
    uint32_t frameStartTimeStamp;

    /*! @brief Time stamp for the beginning of the chirp. */
    uint32_t chirpStartTimeStamp;

    /*! @brief Time stamp for the beginning of the chirp. */
    uint32_t chirpEndTimeStamp;

  	/*! @brief frame counter modulo number of chirps per frame */
    uint32_t frameCount;

  	/*! @brief chirp counter modulo number of chirps per frame */
    uint32_t chirpCount;

  	/*! @brief chirp counter modulo number of chirps per frame */
    uint32_t rxAntennaCount;

    /*! @brief chirp counter modulo number of tx antennas */
    uint32_t txAntennaCount;

    /*! @brief chirp counter modulo number of Doppler bins */
    uint32_t dopplerBinCount;

    /*! @brief Size of point cloud info */
    uint32_t sizePointCloudInfo;

    /*! @brief Size of zero doppler info block */
    uint32_t sizeZeroDopplerInfo;

    /*! @brief Size of point cluster info*/
    uint32_t sizeClusterInfo;

    /*! @brief Size of point tracking info*/
    uint32_t sizeTrackingInfo;

    /*! @brief  DSP cycles for chirp and interframe processing and pending
     *          on EDMA data transferes*/
    cycleLog_t cycleLog;

    /*! @brief  Used for checking that chirp processing finshed on time */
    int32_t chirpProcToken;

    /*! @brief  Used for checking that inter frame processing finshed on time */
    int32_t interFrameProcToken;
} MmwDemo_DSS_DataPathObj;

/**
 *  @b Description
 *  @n
 *   Initializes data path state variables for 1D processing.
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathInit1Dstate(MmwDemo_DSS_DataPathObj *obj);

/**
 *  @b Description
 *  @n
 *   Initializes EDMA driver.
 *  @retval
 *      Not Applicable.
 */
int32_t MmwDemo_dataPathInitEdma(MmwDemo_DSS_DataPathObj *obj);

/**
 *  @b Description
 *  @n
 *   Configures EDMA driver for all of the data path processing.
 *  @retval
 *      Not Applicable.
 */
int32_t MmwDemo_dataPathConfigEdma(MmwDemo_DSS_DataPathObj *obj);

/**
 *  @b Description
 *  @n
 *   Creates heap in L2 and L3 and allocates data path buffers,
 *   The heap is destroyed at the end of the function.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathConfigBuffers(uint32_t adcBufAddress, MmwDemo_Cfg  *demoCfg, MmwDemo_DSS_DataPathObj *dataPathObj);

/**
 *  @b Description
 *  @n
 *   Configures azimuth heat map related processing.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathConfigAzimuthHeatMap(MmwDemo_DSS_DataPathObj *obj);

/**
 *  @b Description
 *  @n
 *  Wait for transfer of data corresponding to the last 2 chirps (ping/pong)
 *  to the radarCube matrix before starting interframe processing.
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_waitEndOfChirps(MmwDemo_DSS_DataPathObj *obj);

/**
  *  @b Description
  *  @n
  *  Wait for transfer of data corresponding to the last 2 range bin to
  *  the heatmap memory before starting frame processing.
  *  @retval
  *      Not Applicable.
  */
void MmwDemo_waitEndOfDopplerProc(MmwDemo_DSS_DataPathObj *obj);

/**
 *  @b Description
 *  @n
 *    Chirp processing. It is called from MmwDemo_dssDataPathProcessEvents. It
 *    is executed per chirp
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_processChirp(MmwDemo_DSS_DataPathObj *obj);

/**
 *  @b Description
 *  @n
 *    Interframe processing. It is called from MmwDemo_dssDataPathProcessEvents
 *    after all chirps of the frame have been received and 1D FFT processing on them
 *    has been completed.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_interFrameProcessing(MmwDemo_DSS_DataPathObj *obj);

/**
 *  @b Description
 *  @n
 *      Power of 2 round up function.
 */

/*!*****************************************************************************************************************
 * \brief
 * Function Name       :    mmwavelib_dftSingleBin_1Ant
 *
 * \par
 * <b>Description</b>  :    Calculates single bin DFT. Profile: N/4*5 cycles
 *
 * \param[in]          :    inputBuf    Input with int16 complex samples, real in even, imaginary in odd location
 *
 * \param[in]          :    sincos      Table with sine cosine values, exp(-1j*2*pi*k/N),
 *                                      valuse are int16 in Q15 format, imaginary in even, real in odd location
 *
 * \param[out]         :    outputBuf   Single point DFT value, int32 complex value, real in even, imaginary in odd location
 *
 * \param[in]          :    length      Length of input buffer (size of DFT) must be power of 2
 *
 * \param[in]          :    doppInd     Index value at wich the DFT is calculated
 *
 * \return                  void
 *
 *******************************************************************************************************************
 */

#ifdef __cplusplus
}
#endif

#endif /* DSS_DATA_PATH_H */

