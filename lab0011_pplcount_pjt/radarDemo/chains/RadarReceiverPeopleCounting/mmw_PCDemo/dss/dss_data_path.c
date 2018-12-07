/**
 *   @file  dss_data_path.c
 *
 *   @brief
 *      Implements Data path processing functionality.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#if defined (SUBSYS_DSS)
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>
#endif
#define DebugP_ASSERT_ENABLED 1
#include <ti/drivers/osal/DebugP.h>
#include <assert.h>
#include <ti/common/sys_common.h>
#include <ti/drivers/osal/SemaphoreP.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/soc/soc.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

#include <ti/alg/mmwavelib/mmwavelib.h>

#include "dss_mmw.h"
#include "dss_data_path.h"
#include "dss_config_edma_util.h"

#define SOC_XWR16XX_DSS_MAXNUMHEAPS (RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS)
#define SOC_XWR16XX_MSS_ADCBUF_SIZE               0x4000U
#define SOC_XWR16XX_DSS_L2_SCRATCH_SIZE           0x1000U
/*! @brief L1 heap used for allocating buffers in L1D SRAM,
    mostly scratch buffers */
#define MMW_L1_SCRATCH_SIZE    0x4000U
#define SOC_XWR16XX_DSS_L2_BUFF_SIZE              0xB000U
#define SOC_XWR16XX_DSS_HSRAM_BUFF_SIZE           0x2000U
#define SOC_XWR16XX_DSS_L3RAM_BUFF_SIZE           0xA0000U

/*! L3 RAM buffer */
#pragma DATA_SECTION(gMmwL3, ".l3data");
#pragma DATA_ALIGN(gMmwL3, 8);
uint8_t gMmwL3[SOC_XWR16XX_DSS_L3RAM_BUFF_SIZE];


/*! HSRAM buffer */
/*meta information for detected Objects shared with MSS */
#pragma DATA_SECTION(detOutputHdr, ".hsramdata");
#pragma DATA_ALIGN(detOutputHdr, 8);
MmwDemo_detOutputHdr detOutputHdr;

#pragma DATA_SECTION(gMmwHSRAM, ".hsramdata");
#pragma DATA_ALIGN(gMmwHSRAM, 8);
uint8_t gMmwHSRAM[SOC_XWR16XX_DSS_HSRAM_BUFF_SIZE];

/*! L2 RAM buffer */
#pragma DATA_SECTION(gMmwL2, ".l2data");
#pragma DATA_ALIGN(gMmwL2, 8);
uint8_t gMmwL2[SOC_XWR16XX_DSS_L2_BUFF_SIZE];

/*! L2 RAM scratch */
#pragma DATA_SECTION(gMmwL2, ".l2data");
#pragma DATA_ALIGN(gMmwL2, 8);
uint8_t gMmwL2Scratch[SOC_XWR16XX_DSS_L2_SCRATCH_SIZE];
/*! L1 Heap */
#pragma DATA_SECTION(gMmwL1, ".l1data");
#pragma DATA_ALIGN(gMmwL1, 8);
uint8_t gMmwL1[MMW_L1_SCRATCH_SIZE];

#define BYTES_PER_SAMP_1D          sizeof(cmplx16ImRe_t)
#define BYTES_PER_SAMP_DET         sizeof(float)

#define PING          0
#define PONG          1


#define MMW_EDMA_CH_1D_IN_PING      EDMA_TPCC0_REQ_FREE_0
#define MMW_EDMA_CH_1D_IN_PONG      EDMA_TPCC0_REQ_FREE_1
#define MMW_EDMA_CH_1D_OUT_PING     EDMA_TPCC0_REQ_FREE_2
#define MMW_EDMA_CH_1D_OUT_PONG     EDMA_TPCC0_REQ_FREE_3
#define MMW_EDMA_CH_2D_IN_PING      EDMA_TPCC0_REQ_FREE_4
#define MMW_EDMA_CH_2D_IN_PONG      EDMA_TPCC0_REQ_FREE_5
#define MMW_EDMA_CH_POW_OUT_PING    EDMA_TPCC0_REQ_FREE_6
#define MMW_EDMA_CH_POW_OUT_PONG    EDMA_TPCC0_REQ_FREE_7
#define MMW_EDMA_CH_3D_IN_PING      EDMA_TPCC0_REQ_FREE_8
#define MMW_EDMA_CH_3D_IN_PONG      EDMA_TPCC0_REQ_FREE_9

#define MMW_EDMA_TRIGGER_ENABLE  1
#define MMW_EDMA_TRIGGER_DISABLE 0

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
extern MmwDemo_DSS_MCB    gMmwDssMCB;


/* If the the following EDMA defines are commented out, the EDMA transfer completion is
   is implemented using polling apporach, Otherwise, if these defines are defined, the EDMA transfers
   are implemented using blocking approach, (data path task pending on semaphore, waiting for the transfer completion
   event, posted by EDMA transfer completion intedrupt. In these cases since the EDMA transfers are faster than
   DSP processing, polling approach is more appropriate. */
//#define EDMA_1D_INPUT_BLOCKING
//#define EDMA_1D_OUTPUT_BLOCKING
//#define EDMA_2D_INPUT_BLOCKING
//#define EDMA_2D_OUTPUT_BLOCKING
//#define EDMA_POW_OUT_BLOCKING


/**
 *  @b Description
 *  @n
 *      Waits for 1D FFT data to be transferrd to input buffer.
 *      This is a blocking function.
 *
 *  @param[in] obj  Pointer to data path object
 *  @param[in] pingPongId ping-pong id (ping is 0 and pong is 1)
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathWait1DInputData(MmwDemo_DSS_DataPathObj *obj, uint32_t pingPongId)
{
#ifdef EDMA_1D_INPUT_BLOCKING
    Bool       status;

    status = Semaphore_pend(obj->EDMA_1D_InputDone_semHandle[pingPongId], BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        System_printf("Error: Semaphore_pend returned %d\n",status);
    }
#else
    /* wait until transfer done */
    volatile bool isTransferDone;
    uint8_t chId;
    if(pingPongId == 0)
    {
        chId = MMW_EDMA_CH_1D_IN_PING;
    }
    else
    {
        chId = MMW_EDMA_CH_1D_IN_PONG;
    }
    do {
        if (EDMA_isTransferComplete(obj->edmaHandle[EDMA_INSTANCE_A],
                                    chId,
                                    (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            DebugP_assert(0);
        }
    } while (isTransferDone == false);
#endif
}

/**
 *  @b Description
 *  @n
 *      Waits for 1D FFT data to be transferred to output buffer.
 *      This is a blocking function.
 *
 *  @param[in] obj  Pointer to data path object
 *  @param[in] pingPongId ping-pong id (ping is 0 and pong is 1)
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathWait1DOutputData(MmwDemo_DSS_DataPathObj *obj, uint32_t pingPongId)
{
#ifdef EDMA_1D_OUTPUT_BLOCKING
    Bool       status;

    status = Semaphore_pend(obj->EDMA_1D_OutputDone_semHandle[pingPongId], BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        System_printf("Error: Semaphore_pend returned %d\n",status);
    }
#else
    /* wait until transfer done */
    volatile bool isTransferDone;
    uint8_t chId;
    if(pingPongId == 0)
    {
        chId = MMW_EDMA_CH_1D_OUT_PING;
    }
    else
    {
        chId = MMW_EDMA_CH_1D_OUT_PONG;
    }
    do {
        if (EDMA_isTransferComplete(obj->edmaHandle[EDMA_INSTANCE_A],
                                    chId,
                                    (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            DebugP_assert(0);
        }
    } while (isTransferDone == false);
#endif
}
/**
 *  @b Description
 *  @n
 *      Waits for 1D FFT data to be transferred to input buffer for 2D-FFT caclulation.
 *      This is a blocking function.
 *
 *  @param[in] obj  Pointer to data path object
 *  @param[in] pingPongId ping-pong id (ping is 0 and pong is 1)
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathWait2DInputData(MmwDemo_DSS_DataPathObj *obj, uint32_t pingPongId)
{
#ifdef EDMA_2D_INPUT_BLOCKING
    Bool       status;

    status = Semaphore_pend(obj->EDMA_2D_InputDone_semHandle[pingPongId], BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        System_printf("Error: Semaphore_pend returned %d\n",status);
    }
#else
    /* wait until transfer done */
    volatile bool isTransferDone;
    uint8_t chId;
    if(pingPongId == 0)
    {
        chId = MMW_EDMA_CH_2D_IN_PING;
    }
    else
    {
        chId = MMW_EDMA_CH_2D_IN_PONG;
    }
    do {
        if (EDMA_isTransferComplete(obj->edmaHandle[EDMA_INSTANCE_A],
                                    chId,
                                    (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            DebugP_assert(0);
        }
    } while (isTransferDone == false);
#endif
}
/**
 *  @b Description
 *  @n
 *  Wait for transfer of data corresponding to one range bin
 *  to the heatmap memory.
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathWait2DOutputData(MmwDemo_DSS_DataPathObj *obj, uint32_t pingPongId)
{
#ifdef EDMA_POW_OUT_BLOCKING
    Bool       status;

    status = Semaphore_pend(obj->EDMA_DetMatrixOutputDone_semHandle[pingPongId], BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        System_printf("Error: Semaphore_pend returned %d\n",status);
    }
#else
    /* wait until transfer done */
    volatile bool isTransferDone;
    uint8_t chId;
    if(pingPongId == 0)
    {
        chId = MMW_EDMA_CH_POW_OUT_PING;
    }
    else
    {
        chId = MMW_EDMA_CH_POW_OUT_PONG;
    }
    do {
        if (EDMA_isTransferComplete(obj->edmaHandle[EDMA_INSTANCE_A],
                                    chId,
                                    (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            DebugP_assert(0);
        }
    } while (isTransferDone == false);
#endif
}

/**
 *  @b Description
 *  @n
 *      EDMA transfer completion call back function as per EDMA API.
 *      Depending on the programmed transfer completion codes,
 *      posts the corresponding done/completion semaphore.
 *      Per current design, a single semaphore could have been used as the
 *      1D, 2D and CFAR stages are sequential, this code gives some flexibility
 *      if some design change in future.
 */
void MmwDemo_EDMA_transferCompletionCallbackFxn(uintptr_t arg,
    uint32_t transferCompletionCode)
{
    MmwDemo_DSS_DataPathObj *obj = (MmwDemo_DSS_DataPathObj *)arg;

    switch (transferCompletionCode)
    {
        case MMW_EDMA_CH_1D_IN_PING:
            Semaphore_post(obj->EDMA_1D_InputDone_semHandle[0]);
        break;

        case MMW_EDMA_CH_1D_IN_PONG:
            Semaphore_post(obj->EDMA_1D_InputDone_semHandle[1]);
        break;

        case MMW_EDMA_CH_1D_OUT_PING:
            Semaphore_post(obj->EDMA_1D_OutputDone_semHandle[0]);
        break;

        case MMW_EDMA_CH_1D_OUT_PONG:
            Semaphore_post(obj->EDMA_1D_OutputDone_semHandle[1]);
        break;

        case MMW_EDMA_CH_2D_IN_PING:
            Semaphore_post(obj->EDMA_2D_InputDone_semHandle[0]);
        break;

        case MMW_EDMA_CH_2D_IN_PONG:
            Semaphore_post(obj->EDMA_2D_InputDone_semHandle[1]);
        break;

        case MMW_EDMA_CH_POW_OUT_PING:
            Semaphore_post(obj->EDMA_DetMatrixOutputDone_semHandle[0]);
        break;

        case MMW_EDMA_CH_POW_OUT_PONG:
            Semaphore_post(obj->EDMA_DetMatrixOutputDone_semHandle[1]);
        break;

        default:
            DebugP_assert(0);
        break;
    }
}

/**
 *  @b Description
 *  @n
 *      Configures all EDMA channels and param sets used in data path processing
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      -1 if error, 0 for no error
 */
int32_t MmwDemo_dataPathConfigEdma(MmwDemo_DSS_DataPathObj *obj)
{
    uint32_t eventQueue;
    uint16_t shadowParam = EDMA_NUM_DMA_CHANNELS;
    int32_t retVal = 0;
    uint16_t dsbindx;

    /*****************************************************
     * EDMA configuration for getting ADC data from ADC buffer
     * to L2 (prior to 1D FFT)
     * For ADC Buffer to L2 use EDMA-A TPTC =1
     *****************************************************/
    eventQueue = 0U;
	 /* Ping - copies chirp samples from even antenna numbers (e.g. RxAnt0 and RxAnt2) */
	retVal =
	EDMAutil_configType1(obj->edmaHandle[EDMA_INSTANCE_A],
		(uint8_t *)(&obj->ADCdataBuf[0]),
		(uint8_t *)(SOC_translateAddress((uint32_t)obj->adcDataL2[PING],SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
		MMW_EDMA_CH_1D_IN_PING,
		false,
		EDMA3_SYNC_A,
		shadowParam++,
		obj->numAdcSamples * BYTES_PER_SAMP_1D,
		obj->numPhyRxAntennas / 2,
		obj->numAdcSamples * BYTES_PER_SAMP_1D * 2,
		0,
		eventQueue,
#ifdef EDMA_1D_INPUT_BLOCKING
		MmwDemo_EDMA_transferCompletionCallbackFxn,
#else
		NULL,
#endif
		(uintptr_t) obj);
	if (retVal < 0)
	{
		return -1;
	}

	/* Pong - copies chirp samples from odd antenna numbers (e.g. RxAnt1 and RxAnt3) */
	retVal =
	EDMAutil_configType1(obj->edmaHandle[EDMA_INSTANCE_A],
		(uint8_t *)(&obj->ADCdataBuf[obj->numAdcSamples]),
		(uint8_t *)(SOC_translateAddress((uint32_t)(obj->adcDataL2[PONG]),SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
		MMW_EDMA_CH_1D_IN_PONG,
		false,
		EDMA3_SYNC_A,
		shadowParam++,
		obj->numAdcSamples * BYTES_PER_SAMP_1D,
		obj->numPhyRxAntennas / 2,
		obj->numAdcSamples * BYTES_PER_SAMP_1D * 2,
		0,
		eventQueue,
#ifdef EDMA_1D_INPUT_BLOCKING
		MmwDemo_EDMA_transferCompletionCallbackFxn,
#else
		NULL,
#endif
		(uintptr_t) obj);
	if (retVal < 0)
	{
		return -1;
	}
    /* using different event queue between input and output to parallelize better */
    eventQueue = 1U;
    /*****************************************************
     * EDMA configuration for storing 1d fft output in transposed manner to L3.
     * It copies one Rx antenna of the chirp per trigger event.
     *****************************************************/
    /* Ping - Copies from ping FFT output (even chirp indices)  to L3 */
    retVal =
    EDMAutil_configType2a(obj->edmaHandle[EDMA_INSTANCE_A],
        (uint8_t *)(SOC_translateAddress((uint32_t)(obj->rangeProcOut[PING]),SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
        (uint8_t *)(&obj->fftOut1D[0]),
        MMW_EDMA_CH_1D_OUT_PING,
        false,
        shadowParam++,
        BYTES_PER_SAMP_1D,
        obj->numRangeBins,
        obj->numTxAntennas,
        obj->numPhyRxAntennas,
		eventQueue,
        obj->numChirpsPerFrame,
#ifdef EDMA_1D_OUTPUT_BLOCKING
        MmwDemo_EDMA_transferCompletionCallbackFxn,
#else
        NULL,
#endif
        (uintptr_t) obj);
    if (retVal < 0)
    {
        return -1;
    }

    /* Pong - Copies from pong FFT output (odd chirp indices)  to L3 */
    retVal =
    EDMAutil_configType2a(obj->edmaHandle[EDMA_INSTANCE_A],
        (uint8_t *)(SOC_translateAddress((uint32_t)(obj->rangeProcOut[PONG]),
                                         SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
        (uint8_t *)(&obj->fftOut1D[0]),
        MMW_EDMA_CH_1D_OUT_PONG,
        false,
        shadowParam++,
        BYTES_PER_SAMP_1D,
        obj->numRangeBins,
        obj->numTxAntennas,
        obj->numPhyRxAntennas,
		eventQueue,
        obj->numChirpsPerFrame,
#ifdef EDMA_1D_OUTPUT_BLOCKING
        MmwDemo_EDMA_transferCompletionCallbackFxn,
#else
        NULL,
#endif
        (uintptr_t) obj);
    if (retVal < 0)
    {
        return -1;
    }

    /*****************************************
     * Interframe processing related EDMA configuration
     *****************************************/
    eventQueue = 0U;

#if 0
    /* Ping: This DMA channel is programmed to fetch the 1D FFT data from radarCube
     * matrix in L3 mem of even antenna rows into the Ping Buffer in L2 mem*/
    retVal =
    EDMAutil_configType1(obj->edmaHandle[EDMA_INSTANCE_A],
        (uint8_t *)(&obj->fftOut1D[0]),
        (uint8_t *)(SOC_translateAddress((uint32_t)(obj->dopplerProcin[PING]),SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
        MMW_EDMA_CH_2D_IN_PING,
        false,
		EDMA3_SYNC_A,
        shadowParam++,
        obj->numChirpsPerFrame * BYTES_PER_SAMP_1D * obj->numVirtRxAntennas,
        (obj->numRangeBins) >> 1,
        obj->numChirpsPerFrame * BYTES_PER_SAMP_1D  * obj->numVirtRxAntennas * 2,
        0,
        eventQueue,
#ifdef EDMA_2D_INPUT_BLOCKING
        MmwDemo_EDMA_transferCompletionCallbackFxn,
#else
        NULL,
#endif
        (uintptr_t) obj);
    if (retVal < 0)
    {
        return -1;
    }

    /* Pong: This DMA channel is programmed to fetch the 1D FFT data from radarCube
     * matrix in L3 mem of odd antenna rows into thePong Buffer in L2 mem*/
    retVal =
    EDMAutil_configType1(obj->edmaHandle[EDMA_INSTANCE_A],
        (uint8_t *)(&obj->fftOut1D[obj->numChirpsPerFrame * obj->numVirtRxAntennas]),
        (uint8_t *)(SOC_translateAddress((uint32_t)(obj->dopplerProcin[PONG]),
                                          SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
        MMW_EDMA_CH_2D_IN_PONG,
        false,
		EDMA3_SYNC_A,
        shadowParam++,
        obj->numChirpsPerFrame * BYTES_PER_SAMP_1D * obj->numVirtRxAntennas,
        (obj->numRangeBins) >> 1,
        obj->numChirpsPerFrame * BYTES_PER_SAMP_1D  * obj->numVirtRxAntennas * 2,
        0,
        eventQueue,
#ifdef EDMA_2D_INPUT_BLOCKING
        MmwDemo_EDMA_transferCompletionCallbackFxn,
#else
        NULL,
#endif
        (uintptr_t) obj);
    if (retVal < 0)
    {
        return -1;
    }
#else
    /* Ping: This DMA channel is programmed to fetch the 1D FFT data from radarCube
     * matrix in L3 mem of even antenna rows into the Ping Buffer in L2 mem*/
	 if (obj->numAzimuthBins - obj->numChirpsPerFrame > 0)
		dsbindx =  (uint16_t)(obj->numAzimuthBins - obj->numChirpsPerFrame);
	else
		dsbindx =  1;

    if(obj->dopplerUsePingPong     == 0)
    {
        retVal =
        EDMAutil_configType1a(obj->edmaHandle[EDMA_INSTANCE_A],
            (uint8_t *)(&obj->fftOut1D[0]),
            (uint8_t *)(SOC_translateAddress((uint32_t)(obj->rangeAsimuthProcin[PING]),SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
            MMW_EDMA_CH_2D_IN_PING,
            false,
            EDMA3_SYNC_AB,
            shadowParam++,
            obj->numChirpsPerFrame * BYTES_PER_SAMP_1D,
            obj->numVirtRxAntennas,
            obj->numRangeBins,
            BYTES_PER_SAMP_1D,
			dsbindx * BYTES_PER_SAMP_1D,
            obj->numChirpsPerFrame * BYTES_PER_SAMP_1D  * obj->numVirtRxAntennas,
            0,
            eventQueue,
    #ifdef EDMA_2D_INPUT_BLOCKING
            MmwDemo_EDMA_transferCompletionCallbackFxn,
    #else
            NULL,
    #endif
            (uintptr_t) obj);
        if (retVal < 0)
        {
            return -1;
        }

    }
    else
    {
        retVal =
        EDMAutil_configType1a(obj->edmaHandle[EDMA_INSTANCE_A],
            (uint8_t *)(&obj->fftOut1D[0]),
            (uint8_t *)(SOC_translateAddress((uint32_t)(obj->rangeAsimuthProcin[PING]),SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
            MMW_EDMA_CH_2D_IN_PING,
            false,
            EDMA3_SYNC_AB,
            shadowParam++,
            obj->numChirpsPerFrame * BYTES_PER_SAMP_1D,
            obj->numVirtRxAntennas,
            (obj->numRangeBins) >> 1,
            BYTES_PER_SAMP_1D,
			dsbindx * BYTES_PER_SAMP_1D,
            obj->numChirpsPerFrame * BYTES_PER_SAMP_1D  * obj->numVirtRxAntennas * 2,
            0,
            eventQueue,
    #ifdef EDMA_2D_INPUT_BLOCKING
            MmwDemo_EDMA_transferCompletionCallbackFxn,
    #else
            NULL,
    #endif
            (uintptr_t) obj);
    /*
        EDMAutil_configType1a(EDMA_Handle handle,
            uint8_t *srcBuff,
            uint8_t *dstBuff,
            uint8_t chId,
            bool isEventTriggered,
            uint8_t synType,
            uint16_t shadowParamId,
            uint16_t aCount,
            uint16_t bCount,
            uint16_t cCount,
            uint16_t srcBIdx,
            uint16_t dstBIdx,
            uint16_t srcCIdx,
            uint16_t dstCIdx,
            uint8_t eventQueueId,
            EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
            uintptr_t transferCompletionCallbackFxnArg)
    */
        if (retVal < 0)
        {
            return -1;
        }

        /* Pong: This DMA channel is programmed to fetch the 1D FFT data from radarCube
         * matrix in L3 mem of odd antenna rows into thePong Buffer in L2 mem*/
        retVal =
        EDMAutil_configType1a(obj->edmaHandle[EDMA_INSTANCE_A],
            (uint8_t *)(&obj->fftOut1D[obj->numChirpsPerFrame * obj->numVirtRxAntennas]),
            (uint8_t *)(SOC_translateAddress((uint32_t)(obj->rangeAsimuthProcin[PONG]),
                                              SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
            MMW_EDMA_CH_2D_IN_PONG,
            false,
            EDMA3_SYNC_AB,
            shadowParam++,
            obj->numChirpsPerFrame * BYTES_PER_SAMP_1D,
            obj->numVirtRxAntennas,
            (obj->numRangeBins) >> 1,
            BYTES_PER_SAMP_1D,
			dsbindx * BYTES_PER_SAMP_1D,
            obj->numChirpsPerFrame * BYTES_PER_SAMP_1D  * obj->numVirtRxAntennas * 2,
            0,
            eventQueue,
    #ifdef EDMA_2D_INPUT_BLOCKING
            MmwDemo_EDMA_transferCompletionCallbackFxn,
    #else
            NULL,
    #endif
            (uintptr_t) obj);
        if (retVal < 0)
        {
            return -1;
        }
    }
#endif

    eventQueue = 1U;
    /* This EDMA channel copies (with transpose) the range-azimuth heatmap signal from range-azimuth processing from L2 mem to detection
     * matrix in L3 mem */
    retVal =
    EDMAutil_configType2a(obj->edmaHandle[EDMA_INSTANCE_A],
        (uint8_t *)(SOC_translateAddress((uint32_t)(obj->rangeAsimuthProcOut[PING]),SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
        (uint8_t *)(&obj->radarProcConfig.heatMapMem[0]),
		MMW_EDMA_CH_POW_OUT_PING,
        false,
        shadowParam++,
        BYTES_PER_SAMP_DET,
        obj->numAzimuthBins,
        1,
		1,
		eventQueue,
        obj->numRangeBins,
#ifdef EDMA_POW_OUT_BLOCKING
		MmwDemo_EDMA_transferCompletionCallbackFxn,
#else
		NULL,
#endif
        (uintptr_t) obj);
    if (retVal < 0)
    {
        return -1;
    }

    /* This EDMA Channel brings selected range bins  from detection matrix in
     * L3 mem (reading in transposed manner) into L2 mem for CFAR detection (in
     * range direction) */
    retVal =
    	    EDMAutil_configType2a(obj->edmaHandle[EDMA_INSTANCE_A],
    	        (uint8_t *)(SOC_translateAddress((uint32_t)(obj->rangeAsimuthProcOut[PONG]),SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
    	        (uint8_t *)(&obj->radarProcConfig.heatMapMem[0]),
    			MMW_EDMA_CH_POW_OUT_PONG,
    	        false,
    	        shadowParam++,
    	        BYTES_PER_SAMP_DET,
    	        obj->numAzimuthBins,
    	        1,
    			1,
				eventQueue,
    	        obj->numRangeBins,
#ifdef EDMA_POW_OUT_BLOCKING
				MmwDemo_EDMA_transferCompletionCallbackFxn,
#else
				NULL,
#endif
    	        (uintptr_t) obj);
    if (retVal < 0)
    {
        return -1;
    }
    return(0);
}

#define pingPongId(x) ((x) & 0x1U)
#define isPong(x) (pingPongId(x))


static uint32_t copyTranspose(uint32_t * src, uint32_t * dest, uint32_t size, int32_t offset, uint32_t stride, uint32_t pairs)
{
	int32_t i, j, k;
	j = 0;
	if (pairs == 1)
	{
		for(i = 0; i < (int32_t)size; i++)
		{
			dest[j+i*offset] = src[i];
			j += (int32_t)stride;
		}

	}
	else
	{
		for(i = 0; i < (int32_t)size; i++)
		{
			for (k = 0; k < (int32_t)pairs; k++)
			{
				dest[j+k+i*offset] = src[pairs * i + k];
			}
			j += (int32_t)stride;
		}
	}
	return(1);
}


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
//#define DEBUG_FRAMEPROC
#ifdef DEBUG_FRAMEPROC
#define DEBUGBUFLEN (200)
uint32_t debugCnt = 0;
uint32_t debugBuf[DEBUGBUFLEN];
#endif

void MmwDemo_interFrameProcessing(MmwDemo_DSS_DataPathObj *obj)
{
    uint32_t rangeIdx;
    volatile uint32_t startTime;
    volatile uint32_t startTime1;
    volatile uint32_t startTime2;
    volatile uint32_t startTimePoll, startTimePoll2;
    //uint8_t chId;

    startTime = Cycleprofiler_getTimeStamp();

#if 0 //clean the code later, but for now use direct memory access

    if(obj->dopplerProcUsingEDMAFlag == 1)
    {
		/* trigger first DMA */
		//EDMA_startDmaTransfer(obj->edmaHandle[EDMA_INSTANCE_A], MMW_EDMA_CH_2D_IN_PING);
    }

    /* initialize the  variable that keeps track of the number of objects detected */
    for (rangeIdx = 0; rangeIdx < obj->numRangeBins; rangeIdx++)
    {
        if(obj->dopplerUsePingPong     == 0)
        {
#ifdef DEBUG_FRAMEPROC
            debugBuf[debugCnt++] = rangeIdx;
            if(debugCnt >= DEBUGBUFLEN) debugCnt = 0;
            debugBuf[debugCnt++] = Cycleprofiler_getTimeStamp();
            if(debugCnt >= DEBUGBUFLEN) debugCnt = 0;
#endif

#if 0
            if (rangeIdx == 0)
            {
            	memset(obj->rangeAsimuthProcOut[PING], 0, sizeof(float)*obj->numAzimuthBins);
            }
            else
            {
				MmwDemo_dataPathWait2DInputData (obj, PING);
				radarRangeAzimuthProcessRun(obj->radarProcessHandle, rangeIdx, (cplx16_t *) obj->rangeAsimuthProcin[PING], obj->rangeAsimuthProcOut[PING];
            }
            if (rangeIdx < obj->numRangeBins - 1)
                EDMA_startDmaTransfer(obj->edmaHandle[EDMA_INSTANCE_A], MMW_EDMA_CH_2D_IN_PING);
#else
            if (rangeIdx == 0)
            {
            	memset(obj->rangeAsimuthProcOut[PING], 0, sizeof(float)*obj->numAzimuthBins);
            }
            else
            {
				//radarDopplerProcessRun(obj->radarProcessHandle, (cplx16_t *) &(obj->fftOut1D[rangeIdx * obj->numVirtRxAntennas * obj->numChirpsPerFrame]), (float *) obj->rangeAsimuthProcOut[PING]);
				radarRangeAzimuthProcessRun(obj->radarProcessHandle, rangeIdx, (cplx16_t *) &(obj->fftOut1D[rangeIdx * obj->numVirtRxAntennas * obj->numChirpsPerFrame]), obj->rangeAsimuthProcOut[PING]);
            }
#endif
#ifdef DEBUG_FRAMEPROC
        debugBuf[debugCnt++] = Cycleprofiler_getTimeStamp();
        if(debugCnt >= DEBUGBUFLEN) debugCnt = 0;
#endif

#if 0
            if(rangeIdx > 1)
                MmwDemo_dataPathWait2DOutputData(obj, PING);

            EDMAutil_triggerType3 (
                    obj->edmaHandle[EDMA_INSTANCE_A],
                    (uint8_t *) NULL,
                    (uint8_t *)(&obj->radarProcConfig.heatMapMem[rangeIdx]),
                    (uint8_t) MMW_EDMA_CH_POW_OUT_PING,
                    (uint8_t) MMW_EDMA_TRIGGER_ENABLE);
#else
            copyTranspose((uint32_t *)obj->rangeAsimuthProcOut[PING], (uint32_t *)&(obj->radarProcConfig.heatMapMem[rangeIdx]), obj->numAzimuthBins, 0, obj->numRangeBins, 1);
#endif
#ifdef DEBUG_CHIRP
        debugBuf[glbDebugBufCnt++] = Cycleprofiler_getTimeStamp();
        if(debugCnt >= DEBUGBUFLEN) debugCnt = 0;
#endif
        }
        else
        {
            if(obj->dopplerProcUsingEDMAFlag == 1)
            {
                MmwDemo_dataPathWait2DInputData (obj, pingPongId(rangeIdx));
                if (rangeIdx < obj->numRangeBins - 1)
                {
                    if (isPong(rangeIdx))
                    {
                        EDMA_startDmaTransfer(obj->edmaHandle[EDMA_INSTANCE_A], MMW_EDMA_CH_2D_IN_PING);
                    }
                    else
                    {
                        EDMA_startDmaTransfer(obj->edmaHandle[EDMA_INSTANCE_A], MMW_EDMA_CH_2D_IN_PONG);
                    }
                }
                if (rangeIdx == 0)
                {
                	memset(obj->rangeAsimuthProcOut[PING], 0, sizeof(float)*obj->numAzimuthBins);
                }
                else
                {
                	radarRangeAzimuthProcessRun(obj->radarProcessHandle, rangeIdx, (cplx16_t *) obj->rangeAsimuthProcin[pingPongId(rangeIdx)], obj->rangeAsimuthProcOut[pingPongId(rangeIdx)]);
                	//radarDopplerProcessRun(obj->radarProcessHandle, (cplx16_t *) obj->rangeAsimuthProcin[pingPongId(rangeIdx)], (float *) obj->rangeAsimuthProcOut[pingPongId(rangeIdx)]);
                }
            }
            else
            {
                if (rangeIdx == 0)
                {
                	memset(obj->rangeAsimuthProcOut[PING], 0, sizeof(float)*obj->numAzimuthBins);
                }
                else
                {
                	radarRangeAzimuthProcessRun(obj->radarProcessHandle, rangeIdx, (cplx16_t *) &(obj->fftOut1D[rangeIdx * obj->numVirtRxAntennas * obj->numChirpsPerFrame]), (float *) obj->rangeAsimuthProcOut[pingPongId(rangeIdx)]);
                }
            }

            if(obj->dopplerProcUsingEDMAFlag == 1)
            {
                if(rangeIdx > 1)
                    MmwDemo_dataPathWait2DOutputData(obj, pingPongId(rangeIdx));
                if(isPong(rangeIdx) == 0)
                {
                    chId = MMW_EDMA_CH_POW_OUT_PING;
                }
                else
                {
                    chId = MMW_EDMA_CH_POW_OUT_PONG;
                }
                EDMAutil_triggerType3 (
                        obj->edmaHandle[EDMA_INSTANCE_A],
                        (uint8_t *) NULL,
                        (uint8_t *)(&obj->radarProcConfig.heatMapMem[rangeIdx]),
                        (uint8_t) chId,
                        (uint8_t) MMW_EDMA_TRIGGER_ENABLE);
            }
            else
            {
                copyTranspose((uint32_t *)obj->rangeAsimuthProcOut[pingPongId(rangeIdx)], (uint32_t *)&(obj->radarProcConfig.heatMapMem[rangeIdx]), obj->numAzimuthBins, 0, obj->numRangeBins, 1);
            }
        }
    }


    if(obj->dopplerProcUsingEDMAFlag == 1)
    {
        if(obj->dopplerUsePingPong     == 0)
        {
            //MmwDemo_dataPathWait2DOutputData(obj, 0);
        }
        else
        {
            //MmwDemo_waitEndOfDopplerProc(obj);
        }
    }


#ifdef DEBUG_CHIRP
    debugBuf[glbDebugBufCnt++] = 8888;
    if(debugCnt >= DEBUGBUFLEN) debugCnt = 0;
    debugBuf[glbDebugBufCnt++] = Cycleprofiler_getTimeStamp();
    if(debugCnt >= DEBUGBUFLEN) debugCnt = 0;
#endif

    radarFrameProcessRun(obj->radarProcessHandle, (void *) &(obj->outputDataToArm->outputToGtracker));

#ifdef DEBUG_CHIRP
    debugBuf[glbDebugBufCnt++] = 9999;
    if(debugCnt >= DEBUGBUFLEN) debugCnt = 0;
    debugBuf[glbDebugBufCnt++] = Cycleprofiler_getTimeStamp();
    if(debugCnt >= DEBUGBUFLEN) debugCnt = 0;
#endif

#endif

    for (rangeIdx = 1; rangeIdx < obj->numRangeBins; rangeIdx++)
    {
    	radarRangeAzimuthProcessRun(obj->radarProcessHandle, obj->frameCount, rangeIdx, (cplx16_t *) &(obj->fftOut1D[rangeIdx * obj->numVirtRxAntennas * obj->numChirpsPerFrame]), obj->rangeAsimuthProcOut[PING]);
    	copyTranspose((uint32_t *)obj->rangeAsimuthProcOut[PING], (uint32_t *)&(obj->radarProcConfig.heatMapMem[rangeIdx]), obj->numAzimuthBins, 0, obj->numRangeBins, 1);
    }
    obj->cycleLog.rangeAzimuthProcCycles = Cycleprofiler_getTimeStamp() - startTime;

    radarFrameProcessRun(obj->radarProcessHandle, (void *) &(obj->outputDataToArm->outputToTracker));

    obj->cycleLog.cfarProcCycles = obj->radarProcConfig.benchmarkPtr->buffer[obj->radarProcConfig.benchmarkPtr->bufferIdx].cfarDetectionCycles;
    obj->cycleLog.dopplerEstCycles = obj->radarProcConfig.benchmarkPtr->buffer[obj->radarProcConfig.benchmarkPtr->bufferIdx].dopplerEstCycles;

}


/**
 *  @b Description
 *  @n
 *    Chirp processing. It is called from MmwDemo_dssDataPathProcessEvents. It
 *    is executed per chirp
 *
 *  @retval
 *      Not Applicable.
 */
 void MmwDemo_processChirp(MmwDemo_DSS_DataPathObj *obj)
{
    uint32_t antIndx, baseIdx;
    volatile uint32_t startTime;

    if(obj->chirpCount > 1) //verify if ping(or pong) buffer is free for odd(or even) chirps
    {
        MmwDemo_dataPathWait1DOutputData (obj, pingPongId(obj->chirpCount));
    }


	/* Kick off DMA to fetch data from ADC buffer for first channel */
	EDMA_startDmaTransfer(obj->edmaHandle[EDMA_INSTANCE_A],
					   MMW_EDMA_CH_1D_IN_PING);

    obj->rxAntennaCount = 0;
   	baseIdx		=	obj->txAntennaCount * obj->numPhyRxAntennas * obj->numChirpsPerFrame;

  /* 1d fft for first antenna, followed by kicking off the DMA of fft output */
	for (antIndx = 0; antIndx < obj->numPhyRxAntennas; antIndx++)
	{
		/* kick off DMA to fetch data for next antenna */
		if (antIndx < (obj->numPhyRxAntennas - 1))
		{
			if (isPong(antIndx))
			{
				EDMA_startDmaTransfer(obj->edmaHandle[EDMA_INSTANCE_A],
						MMW_EDMA_CH_1D_IN_PING);
			}
			else
			{
				EDMA_startDmaTransfer(obj->edmaHandle[EDMA_INSTANCE_A],
						MMW_EDMA_CH_1D_IN_PONG);
			}
		}

		MmwDemo_dataPathWait1DInputData (obj, pingPongId(antIndx));
		if (1)
		radarRangeProcessRun(obj->radarProcessHandle, (cplx16_t *) obj->adcDataL2[pingPongId(antIndx)], (cplx16_t *) obj->rangeProcOut[pingPongId(antIndx)]);
		else
		{
			int32_t i;
			int32_t * RESTRICT tempPtr;

			tempPtr 	=	(int32_t *)obj->rangeProcOut[pingPongId(antIndx)];
			for (i = 0; i < obj->numRangeBins; i++)
			{
				_amem4(tempPtr++) 	=	((i + 1) << 16) + ((antIndx + obj->txAntennaCount * obj->numPhyRxAntennas) << 8) + obj->dopplerBinCount;
			}
		}

		if(isPong(antIndx))
		{
			EDMAutil_triggerType3 (
				obj->edmaHandle[EDMA_INSTANCE_A],
				(uint8_t *) NULL,
				(uint8_t *)(&obj->fftOut1D[baseIdx + antIndx * obj->numChirpsPerFrame + obj->dopplerBinCount]),
				(uint8_t) MMW_EDMA_CH_1D_OUT_PONG,
				(uint8_t) MMW_EDMA_TRIGGER_ENABLE);
		}
		else
		{
			EDMAutil_triggerType3 (
				obj->edmaHandle[EDMA_INSTANCE_A],
				(uint8_t *) NULL,
				(uint8_t *)(&obj->fftOut1D[baseIdx + antIndx * obj->numChirpsPerFrame + obj->dopplerBinCount]),
				(uint8_t) MMW_EDMA_CH_1D_OUT_PING,
				(uint8_t) MMW_EDMA_TRIGGER_ENABLE);
		}
		obj->rxAntennaCount++;
	}

	obj->chirpCount++;
    obj->txAntennaCount++;
    if(obj->txAntennaCount == obj->numTxAntennas)
    {
        obj->txAntennaCount = 0;
        obj->dopplerBinCount++;
        if (obj->dopplerBinCount == obj->numChirpsPerFrame)
        {
            obj->dopplerBinCount = 0;
            obj->chirpCount = 0;
        }
    }
}

 /**
  *  @b Description
  *  @n
  *  Wait for transfer of data corresponding to the last 2 range bin to
  *  the heatmap memory before starting frame processing.
  *  @retval
  *      Not Applicable.
  */
void MmwDemo_waitEndOfDopplerProc(MmwDemo_DSS_DataPathObj *obj)
{
	MmwDemo_dataPathWait2DOutputData(obj, 0);
	MmwDemo_dataPathWait2DOutputData(obj, 1);
}

/**
  *  @b Description
  *  @n
  *  Wait for transfer of data corresponding to the last 2 chirps (ping/pong)
  *  to the radarCube matrix before starting interframe processing.
  *  @retval
  *      Not Applicable.
  */
void MmwDemo_waitEndOfChirps(MmwDemo_DSS_DataPathObj *obj)
{

    /* Wait for transfer of data corresponding to last 2 chirps (ping/pong) */
    MmwDemo_dataPathWait1DOutputData (obj, 0);
    MmwDemo_dataPathWait1DOutputData (obj, 1);
}


void MmwDemo_edmaErrorCallbackFxn(EDMA_Handle handle, EDMA_errorInfo_t *errorInfo)
{
	//System_printf("Error:handle = 0x%x, error = %d\n",handle, *errorInfo);
    DebugP_assert(0);
}

void MmwDemo_edmaTransferControllerErrorCallbackFxn(EDMA_Handle handle,
                EDMA_transferControllerErrorInfo_t *errorInfo)
{
    DebugP_assert(0);
}

void MmwDemo_dataPathInit1Dstate(MmwDemo_DSS_DataPathObj *obj)
{
    obj->chirpCount = 0;
    obj->dopplerBinCount = 0;
    obj->txAntennaCount = 0;

}

int32_t MmwDemo_dataPathInitEdma(MmwDemo_DSS_DataPathObj *obj)
{
    Semaphore_Params       semParams;
    uint8_t numInstances;
    int32_t errorCode;
    EDMA_Handle handle;
    EDMA_errorConfig_t errorConfig;
    uint32_t instanceId;
    EDMA_instanceInfo_t instanceInfo;

    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    obj->EDMA_1D_InputDone_semHandle[0] = Semaphore_create(0, &semParams, NULL);
    obj->EDMA_1D_InputDone_semHandle[1] = Semaphore_create(0, &semParams, NULL);
    obj->EDMA_1D_OutputDone_semHandle[0] = Semaphore_create(0, &semParams, NULL);
    obj->EDMA_1D_OutputDone_semHandle[1] = Semaphore_create(0, &semParams, NULL);
    obj->EDMA_2D_InputDone_semHandle[0] = Semaphore_create(0, &semParams, NULL);
    obj->EDMA_2D_InputDone_semHandle[1] = Semaphore_create(0, &semParams, NULL);
    obj->EDMA_DetMatrixOutputDone_semHandle[0] = Semaphore_create(0, &semParams, NULL);
    obj->EDMA_DetMatrixOutputDone_semHandle[1] = Semaphore_create(0, &semParams, NULL);

    numInstances = EDMA_getNumInstances();

    /* Initialize the edma instance to be tested */
    for(instanceId = 0; instanceId < numInstances; instanceId++) {
        EDMA_init(instanceId);

        handle = EDMA_open(instanceId, &errorCode, &instanceInfo);
        if (handle == NULL)
        {
            System_printf("Error: Unable to open the edma Instance, erorCode = %d\n", errorCode);
            return -1;
        }
        obj->edmaHandle[instanceId] = handle;

        errorConfig.isConfigAllEventQueues = true;
        errorConfig.isConfigAllTransferControllers = true;
        errorConfig.isEventQueueThresholdingEnabled = true;
        errorConfig.eventQueueThreshold = EDMA_EVENT_QUEUE_THRESHOLD_MAX;
        errorConfig.isEnableAllTransferControllerErrors = true;
        errorConfig.callbackFxn = MmwDemo_edmaErrorCallbackFxn;
        errorConfig.transferControllerCallbackFxn = MmwDemo_edmaTransferControllerErrorCallbackFxn;
        if ((errorCode = EDMA_configErrorMonitoring(handle, &errorConfig)) != EDMA_NO_ERROR)
        {
            System_printf("Debug: EDMA_configErrorMonitoring() failed with errorCode = %d\n", errorCode);
            return -1;
        }
    }
    return 0;
}

void MmwDemo_printHeapStats()
{
    System_printf("DDR Heap : size %d (0x%x), used %d (0x%x)\n",
    		gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapSize, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapSize,
			gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAllocOffset, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAllocOffset);
    System_printf("LL2 Heap : size %d (0x%x), used %d (0x%x)\n",
    		gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapSize, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapSize,
			gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapAllocOffset, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapAllocOffset);
    System_printf("LL2 Scratch : size %d (0x%x), used %d (0x%x)\n",
    		gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].scratchSize, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].scratchSize,
			gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].maxScratchSizeUsed, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].maxScratchSizeUsed);
    System_printf("LL1 Scratch : size %d (0x%x), used %d (0x%x)\n",
    		gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL1].scratchSize, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL1].scratchSize,
			gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL1].maxScratchSizeUsed, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL1].maxScratchSizeUsed);
    System_printf("HSRAM Heap : size %d (0x%x), used %d (0x%x)\n",
    		gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_HSRAM].heapSize, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_HSRAM].heapSize,
			gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_HSRAM].heapAllocOffset, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_HSRAM].heapAllocOffset);


}
#if 0
uint32_t checkUniqueChirpProfile(MmwDemo_Cfg *demoCfg, uint32_t totalProfile)
{
	uint32_t nUniqueProfile = 1;
	int32_t i, j;

    for (i = 1; i < (int32_t)totalProfile; i++)
    {
        for (j = i + 1; j < (int32_t)totalProfile; j++)
        {
        	if ( (demoCfg->ctrlCfg.u.chirpModeCfg.chirpCfg[i].cfg.adcStartTimeVar != demoCfg->ctrlCfg.u.chirpModeCfg.chirpCfg[j].cfg.adcStartTimeVar)
        	   ||(demoCfg->ctrlCfg.u.chirpModeCfg.chirpCfg[i].cfg.startFreqVar != demoCfg->ctrlCfg.u.chirpModeCfg.chirpCfg[j].cfg.startFreqVar)
        	   ||(demoCfg->ctrlCfg.u.chirpModeCfg.chirpCfg[i].cfg.freqSlopeVar != demoCfg->ctrlCfg.u.chirpModeCfg.chirpCfg[j].cfg.freqSlopeVar)
        	   ||(demoCfg->ctrlCfg.u.chirpModeCfg.chirpCfg[i].cfg.idleTimeVar != demoCfg->ctrlCfg.u.chirpModeCfg.chirpCfg[j].cfg.idleTimeVar))
        	{
        		nUniqueProfile++;
        		/* todos: need to add code to store chirp config and create a map for multichirp design*/
        	}
        }
    }
    return(nUniqueProfile);
}
#endif
void 	MmwDemo_initConfigStruct(MmwDemo_Cfg  *demoCfg, radarProcessConfig_t    *radarProcConfig)
{
	uint32_t 		uitemp;
	float frequencySlopeMHzMicoSec, adcSamplePeriodMicoSec, bandWidth, centerFrequency, chirpInterval;
    uint16_t    frameChirpStartIdx;
    uint16_t    frameChirpEndIdx;
    int16_t     frameTotalChirps;
    int32_t     errCode;
    uint32_t    profileLoopIdx, chirpLoopIdx;
    bool        foundValidProfile = false;
    MMWave_CtrlCfg* ptrCtrlCfg = &(demoCfg->ctrlCfg);
    MMWave_OpenCfg* ptrOpenCfg = &(demoCfg->openCfg);
    uint16_t    channelTxEn = ptrOpenCfg->chCfg.txChannelEn;

	radarProcConfig->mimoModeFlag	=	0;
	radarProcConfig->numTxAntenna = (uint16_t)_bitc4(demoCfg->openCfg.chCfg.txChannelEn);
	if (radarProcConfig->numTxAntenna > 1)
		radarProcConfig->mimoModeFlag	=	1;
	radarProcConfig->numPhyRxAntenna = (uint16_t)_bitc4(demoCfg->openCfg.chCfg.rxChannelEn);
	radarProcConfig->numAntenna = radarProcConfig->numPhyRxAntenna * radarProcConfig->numTxAntenna;
	radarProcConfig->numTotalChirpProfile = 0;




    /* read frameCfg chirp start/stop*/
    frameChirpStartIdx = ptrCtrlCfg->u.frameCfg.frameCfg.chirpStartIdx;
    frameChirpEndIdx = ptrCtrlCfg->u.frameCfg.frameCfg.chirpEndIdx;
    frameTotalChirps = frameChirpEndIdx - frameChirpStartIdx + 1;

    /* since validChirpTxEnBits is static array of 32 */
    DebugP_assert(frameTotalChirps<=32);

    /* loop for profiles and find if it has valid chirps */
    /* we support only one profile in this processing chain */
    for (profileLoopIdx=0;
        ((profileLoopIdx<MMWAVE_MAX_PROFILE)&&(foundValidProfile==false));
        profileLoopIdx++)
    {
        uint32_t    mmWaveNumChirps = 0;
        uint16_t    validChirpTxEnBits[32]={0};
        MMWave_ProfileHandle profileHandle;

#ifdef PANKAJ
        profileHandle = ptrCtrlCfg->u.fullControlCfg.u.chirpModeCfg.profileHandle[profileLoopIdx];
        if (profileHandle == NULL)
            continue; /* skip this profile */
#else
        /* Get the profile handle: */
        if (MMWave_getProfileHandle(gMmwDssMCB.ctrlHandle, profileLoopIdx, &profileHandle, &errCode) < 0)
        {
            /* Error: Unable to get the profile handle; this should never fail */
            DebugP_assert (0);
        }
        if (profileHandle == NULL)
            continue; /* skip this profile */
#endif

        /* get numChirps for this profile; skip error checking */
        MMWave_getNumChirps(profileHandle, &mmWaveNumChirps,&errCode);
        /* loop for chirps and find if it has valid chirps for the frame
           looping around for all chirps in a profile, in case
           there are duplicate chirps
         */
        for (chirpLoopIdx=1;chirpLoopIdx<=mmWaveNumChirps;chirpLoopIdx++)
        {
            MMWave_ChirpHandle chirpHandle;
            /* get handle and read ChirpCfg */
            if (MMWave_getChirpHandle(profileHandle,chirpLoopIdx,&chirpHandle,&errCode)==0)
            {
                rlChirpCfg_t chirpCfg;
                if (MMWave_getChirpCfg(chirpHandle,&chirpCfg,&errCode)==0)
                {
                    uint16_t chirpTxEn = chirpCfg.txEnable;
                    /* do chirps fall in range and has valid antenna enabled */
                    if ((chirpCfg.chirpStartIdx >= frameChirpStartIdx) &&
                        (chirpCfg.chirpEndIdx <= frameChirpEndIdx) &&
                        ((chirpTxEn & channelTxEn) > 0))
                    {
                        uint16_t idx = 0;
                        for (idx=(chirpCfg.chirpStartIdx-frameChirpStartIdx);idx<=(chirpCfg.chirpEndIdx-frameChirpStartIdx);idx++)
                        {
                            validChirpTxEnBits[idx] = chirpTxEn;
                            foundValidProfile = true;
                            radarProcConfig->numTotalChirpProfile++; //no check on multi-chirp with different chirp config!!!
                        }

                    }
                }
            }
        }
        /* now loop through unique chirps and check if we found all of the ones
           needed for the frame and then determine the azimuth antenna
           configuration
         */
        if (foundValidProfile) {
            for (chirpLoopIdx=0;chirpLoopIdx<frameTotalChirps;chirpLoopIdx++)
            {
                uint16_t chirpTxEn = validChirpTxEnBits[chirpLoopIdx];

            	uitemp	=	(uint32_t)_bitc4(chirpTxEn);
            	if (uitemp > 1)
            	{
            		System_printf("channelCfg Error: Current code only support SIMO and TDM MIMO !!!");
            		radarProcConfig->mimoModeFlag	=	2;
            		DebugP_assert(0);
            	}
            }
        }

        /* found valid chirps for the frame; mark this profile valid */
        if (foundValidProfile==true) {
            rlProfileCfg_t  profileCfg;

            //dataPathObj->validProfileIdx = profileLoopIdx;

            /* Get the profile configuration: */
            if (MMWave_getProfileCfg (profileHandle,&profileCfg, &errCode) < 0)
            {
                System_printf ("Error: Unable to get the profile configuration [Error code %d]\n", errCode);
                DebugP_assert (0);
            }

        	radarProcConfig->numAdcSamplePerChirp	=	profileCfg.numAdcSamples;

        	frequencySlopeMHzMicoSec		=	(float)profileCfg.freqSlopeConst * 3600.f* 900.f/((float)(1<<26));
        	adcSamplePeriodMicoSec			=	1000.0f / (float) profileCfg.digOutSampleRate;
        	adcSamplePeriodMicoSec			*=	(float)radarProcConfig->numAdcSamplePerChirp;
        	bandWidth						=	frequencySlopeMHzMicoSec * adcSamplePeriodMicoSec * 1.0e6;
        	radarProcConfig->cfarConfig.rangeRes				=	(3.0e8 / (2.f * bandWidth ));
        	centerFrequency					=	((float)profileCfg.startFreqConst * 3.6f/((float)(1<<26))) * 1.0e9 + bandWidth * 0.5;
        	chirpInterval					=	(float)((profileCfg.rampEndTime) + (float)(profileCfg.idleTimeConst)) *1.0e-8;
        	radarProcConfig->chirpInterval  =   chirpInterval;

        	radarProcConfig->numChirpPerFrame	=	demoCfg->ctrlCfg.u.frameCfg.frameCfg.numLoops;
        	radarProcConfig->framePeriod		=	(demoCfg->ctrlCfg.u.frameCfg.frameCfg.framePeriodicity) * 5e-6;
        	radarProcConfig->cfarConfig.velocityRes		=	(3.0e8 / (2.0 *  (float) radarProcConfig->numChirpPerFrame * radarProcConfig->numTxAntenna * centerFrequency * chirpInterval));
        }
    }




#if 0
	/*!!!! check TDM mimo or FDM mimo !!!!*/
    for (index = 0; index < MMWAVE_MAX_CHIRPCFG_ENTRIES; index++)
    {
        /* Is the chirp configuration entry free? */
        if (demoCfg->ctrlCfg.u.chirpModeCfg.chirpCfg[index].isValid == true)
        {
        	radarProcConfig->numTotalChirpProfile++;
        	uitemp	=	(uint32_t)_bitc4(demoCfg->ctrlCfg.u.chirpModeCfg.chirpCfg[index].cfg.txEnable);
        	if (uitemp > 1)
        	{
        		System_printf("channelCfg Error: Current code does not support FDM MIMO !!!");
        		radarProcConfig->mimoModeFlag	=	2;
        		DebugP_assert(0);
        	}
        }
    }
    if (radarProcConfig->numTotalChirpProfile > 1)
    	radarProcConfig->numUniqueChirpProfile = checkUniqueChirpProfile(demoCfg, radarProcConfig->numTotalChirpProfile);
    else
    	radarProcConfig->numUniqueChirpProfile = 1;

    if (radarProcConfig->numUniqueChirpProfile > 1)
	{
		System_printf("channelCfg Error: Current code does not support multi-chirp design !!!");
		DebugP_assert(0);
	}
#endif
    radarProcConfig->numUniqueChirpProfile = 1;
	uitemp 			=	demoCfg->openCfg.adcOutCfg.fmt.b2AdcBits;
	if (uitemp == 2)
		radarProcConfig->numAdcBitsPerSample = 16;
	else if(uitemp == 1)
		radarProcConfig->numAdcBitsPerSample = 14;
	else if(uitemp == 0)
		radarProcConfig->numAdcBitsPerSample = 12;


    /* Range window parameters. */
	radarProcConfig->rangeWinSize     = 16;
	if(0)
	{
		int32_t i;
		for (i = 0; i < radarProcConfig->rangeWinSize; i++)
		{
			radarProcConfig->rangeWindow[i]  = 1.f;
		}
	}
	else
	{
		radarProcConfig->rangeWindow[0]  = 0.0800f;
		radarProcConfig->rangeWindow[1]  = 0.0894f;
		radarProcConfig->rangeWindow[2]  = 0.1173f;
		radarProcConfig->rangeWindow[3]  = 0.1624f;
		radarProcConfig->rangeWindow[4]  = 0.2231f;
		radarProcConfig->rangeWindow[5]  = 0.2967f;
		radarProcConfig->rangeWindow[6]  = 0.3802f;
		radarProcConfig->rangeWindow[7]  = 0.4703f;
		radarProcConfig->rangeWindow[8]  = 0.5633f;
		radarProcConfig->rangeWindow[9]  = 0.6553f;
		radarProcConfig->rangeWindow[10] = 0.7426f;
		radarProcConfig->rangeWindow[11] = 0.8216f;
		radarProcConfig->rangeWindow[12] = 0.8890f;
		radarProcConfig->rangeWindow[13] = 0.9422f;
		radarProcConfig->rangeWindow[14] = 0.9789f;
		radarProcConfig->rangeWindow[15] = 0.9976f;
	}
    radarProcConfig->maxNumDetObj			= (uint8_t)MAX_RESOLVED_OBJECTS_PER_FRAME;
}


void MmwDemo_dataPathConfigBuffers(uint32_t adcBufAddress, MmwDemo_Cfg  *demoCfg, MmwDemo_DSS_DataPathObj *dataPathObj)
{
    //MmwDemo_DSS_DataPathObj *dataPathObj;
    radarOsal_heapConfig heapconfig[SOC_XWR16XX_DSS_MAXNUMHEAPS];
	ProcessErrorCodes procErrorCode;

    /* Get data path object and control configuration */
    //dataPathObj  = &gMmwDssMCB.dataPathObj;
	int32_t tempNumSamples;

    dataPathObj->ADCdataBuf = (cmplx16ImRe_t *)adcBufAddress;

    // heap init
	memset(heapconfig, 0, sizeof(heapconfig));
	heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapType 	= 	RADARMEMOSAL_HEAPTYPE_DDR_CACHED;
	heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAddr   = 	(int8_t *) &gMmwL3[0];
	heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapSize   = 	SOC_XWR16XX_DSS_L3RAM_BUFF_SIZE;
	heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchAddr= 	NULL; 	/* not DDR scratch for TM demo  */
	heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchSize= 	0; 	/* not DDR scratch for TM demo  */

	heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapType 			= 	RADARMEMOSAL_HEAPTYPE_LL2;
	heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapAddr   		= 	(int8_t *) &gMmwL2[0];
	heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapSize   		= 	SOC_XWR16XX_DSS_L2_BUFF_SIZE;
	heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].scratchAddr   	= 	(int8_t *)&gMmwL2Scratch[0];;
	heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].scratchSize   	= 	SOC_XWR16XX_DSS_L2_SCRATCH_SIZE;

	heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].heapType 			= 	RADARMEMOSAL_HEAPTYPE_LL1;
	heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].heapAddr   		= 	NULL; /* not used as L1 heap in TM demo  */
	heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].heapSize   		= 	0;    /* not used as L1 heap in TM demo  */
 	heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].scratchAddr   	= 	(int8_t *) &gMmwL1[0];
	heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].scratchSize   	= 	MMW_L1_SCRATCH_SIZE;

	heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].heapType 		= 	RADARMEMOSAL_HEAPTYPE_HSRAM;
	heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].heapAddr   		= 	(int8_t *) &gMmwHSRAM[0];
	heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].heapSize   		= 	SOC_XWR16XX_DSS_HSRAM_BUFF_SIZE;
 	heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].scratchAddr   	= 	NULL; 	/* not HSRAM scratch for TM demo  */
	heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].scratchSize   	= 	0;    /* not HSRAM scratch for TM demo  */

	if(radarOsal_memInit(&heapconfig[0], SOC_XWR16XX_DSS_MAXNUMHEAPS) == RADARMEMOSAL_FAIL)
	{
		System_printf("Error: radarOsal_memInit fail\n");
	}

	/*initializing configuration structure for signal processing chain */
	MmwDemo_initConfigStruct(demoCfg, &dataPathObj->radarProcConfig);

	dataPathObj->radarProcessHandle = (void *) radarProcessCreate(&dataPathObj->radarProcConfig, &procErrorCode);
    if (procErrorCode > PROCESS_OK)
    {
    	System_printf("radar process creat error = %d! Exit!\n", (uint8_t)procErrorCode);
    	DebugP_assert(0);
    }
    dataPathObj->outputDataToArm = (outputToARM_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, sizeof(outputToARM_t), 128);
    memset((void *)dataPathObj->outputDataToArm, 0, sizeof(outputToARM_t));
	dataPathObj->scratchBuf 			= 	(int32_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 1, MMW_TM_DEMO_EDMASCATCHBUF_SIZE, 8); //fixed to 16k byte

	//if (MMW_TM_DEMO_EDMASCATCHBUF_SIZE >= dataPathObj->radarProcConfig.fftSize1D * 2 * sizeof(cmplx16ImRe_t) * dataPathObj->radarProcConfig.numPhyRxAntenna)
	if (0)
	{
		/* no PING PONG for input, only one shot for all antennas of the chirp */
		dataPathObj->adcDataL2[PING] 		= 	(cmplx16ImRe_t *)&dataPathObj->scratchBuf[0];
		dataPathObj->adcDataL2[PONG] 		= 	NULL;
		dataPathObj->rangeProcOut[PING] 	= 	(int32_t *)&dataPathObj->scratchBuf[dataPathObj->radarProcConfig.fftSize1D * dataPathObj->radarProcConfig.numPhyRxAntenna];
		dataPathObj->rangeProcOut[PONG] 	= 	(int32_t *)&dataPathObj->scratchBuf[dataPathObj->radarProcConfig.fftSize1D * dataPathObj->radarProcConfig.numPhyRxAntenna + dataPathObj->radarProcConfig.fftSize1D];
	}
	else
	{
		/* PING PONG IN OUT */
		dataPathObj->adcDataL2[PING] 		= 	(cmplx16ImRe_t *)&dataPathObj->scratchBuf[0];
		dataPathObj->adcDataL2[PONG] 		= 	(cmplx16ImRe_t *)&dataPathObj->scratchBuf[dataPathObj->radarProcConfig.fftSize1D];
		dataPathObj->rangeProcOut[PING] 	= 	(int32_t *)&dataPathObj->scratchBuf[2 * dataPathObj->radarProcConfig.fftSize1D];
		dataPathObj->rangeProcOut[PONG] 	= 	(int32_t *)&dataPathObj->scratchBuf[3 * dataPathObj->radarProcConfig.fftSize1D];
	}
	
	//if ((dataPathObj->radarProcConfig.numAntenna * dataPathObj->radarProcConfig.fftSize2D * sizeof(int32_t)) >= (MMW_L1_SCRATCH_SIZE >> 2))
	if(1)
	{
        dataPathObj->rangeAsimuthProcin[PING]  	= 	(int32_t *)&dataPathObj->scratchBuf[0 * dataPathObj->radarProcConfig.numAntenna * dataPathObj->radarProcConfig.fftSize2D];
        //dataPathObj->rangeAsimuthProcin[PONG]    =   (int32_t *)&dataPathObj->scratchBuf[1 * dataPathObj->radarProcConfig.numAntenna * dataPathObj->radarProcConfig.fftSize2D];
        dataPathObj->rangeAsimuthProcOut[PING] 	= 	(float *)&dataPathObj->scratchBuf[2 * dataPathObj->radarProcConfig.numAntenna * dataPathObj->radarProcConfig.fftSize2D];
        //dataPathObj->rangeAsimuthProcOut[PONG]   =   (float *)&dataPathObj->scratchBuf[3 * dataPathObj->radarProcConfig.numAntenna * dataPathObj->radarProcConfig.fftSize2D];
        dataPathObj->dopplerUsePingPong     =   0;
	}
	else
	{
        dataPathObj->rangeAsimuthProcin[PING]    =   (int32_t *)&dataPathObj->scratchBuf[0 * dataPathObj->radarProcConfig.numAntenna * dataPathObj->radarProcConfig.fftSize2D];
        dataPathObj->rangeAsimuthProcin[PONG]    =   (int32_t *)&dataPathObj->scratchBuf[1 * dataPathObj->radarProcConfig.numAntenna * dataPathObj->radarProcConfig.fftSize2D];
        dataPathObj->rangeAsimuthProcOut[PING]   =   (float *)&dataPathObj->scratchBuf[2 * dataPathObj->radarProcConfig.numAntenna * dataPathObj->radarProcConfig.fftSize2D];
        dataPathObj->rangeAsimuthProcOut[PONG]   =   (float *)&dataPathObj->scratchBuf[3 * dataPathObj->radarProcConfig.numAntenna * dataPathObj->radarProcConfig.fftSize2D];
        dataPathObj->dopplerUsePingPong     =   1;
	}
	dataPathObj->fftOut1D  				= 	(cmplx16ImRe_t *)dataPathObj->radarProcConfig.pFFT1DBuffer;
	dataPathObj->numPhyRxAntennas  		= 	(uint32_t)dataPathObj->radarProcConfig.numPhyRxAntenna;
	dataPathObj->numVirtRxAntennas  	= 	(uint32_t)dataPathObj->radarProcConfig.numAntenna;
	dataPathObj->numTxAntennas  		= 	(uint32_t)dataPathObj->radarProcConfig.numTxAntenna;
	tempNumSamples 						= 	(((uint32_t)dataPathObj->radarProcConfig.numAdcSamplePerChirp + 16/2) >> 4) << 4; // round up to multiple of 16!!!
	dataPathObj->numAdcSamples  		= 	tempNumSamples;
	dataPathObj->numChirpsPerFrame  	= 	(uint32_t)dataPathObj->radarProcConfig.numChirpPerFrame;
	dataPathObj->numRangeBins  			= 	(uint32_t)dataPathObj->radarProcConfig.fftSize1D;
	dataPathObj->numAzimuthBins  		= 	(uint32_t)dataPathObj->radarProcConfig.numAzimuthBin;
	dataPathObj->chirpThreshold  		= 	1;
	dataPathObj->sizePointCloudInfo 	=   sizeof(radarProcessOutputToTracker);
	dataPathObj->frameCount 			=	0;

	//if (dataPathObj->numDopplerBins * dataPathObj->numVirtRxAntennas < MMW_TM_DEMO_EDMASCATCHBUF_SIZE/8) dataPathObj->dopplerProcUsingEDMAFlag = 0;
	//else dataPathObj->dopplerProcUsingEDMAFlag = 1;

	dataPathObj->dopplerProcUsingEDMAFlag = 1;

	MmwDemo_printHeapStats();
    radarOsal_memDeInit();
}
