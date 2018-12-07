/*
 *   @file  dss_main.c
 *
 *   @brief
 *      Millimeter Wave Demo running on DSS
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
#include <ti/sysbios/family/c64p/Cache.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>

/* MMWSDK Include Files. */
#include <ti/drivers/soc/soc.h>
#include <ti/common/sys_common.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

/* MMWAVE Demo Include Files */
#include <chains/RadarReceiverPeopleCounting/mmw_PCDemo/dss/dss_mmw.h>
#include <chains/RadarReceiverPeopleCounting/mmw_PCDemo/dss/dss_data_path.h>
#include <chains/RadarReceiverPeopleCounting/mmw_PCDemo/common/mmw_messages.h>

/* Related to linker copy table for copying from L3 to L1PSRAM for example */
#include <cpy_tbl.h>

/**************************************************************************
 *************************** MmwDemo External DSS Functions ******************
 **************************************************************************/


/**************************************************************************
 *************************** Global Definitions ********************************
 **************************************************************************/


/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
MmwDemo_DSS_MCB    gMmwDssMCB;

/* copy table related */
extern far COPY_TABLE _MmwDemo_fastCode_L1PSRAM_copy_table;

extern MmwDemo_detOutputHdr detOutputHdr;

/**************************************************************************
 ************************* MmwDemo Functions Prototype  **********************
 **************************************************************************/

/* Copy table related */
static void MmwDemo_edmaBlockCopy(EDMA_Handle handle, uint32_t loadAddr, uint32_t runAddr, uint16_t size);
static void MmwDemo_copyTable(EDMA_Handle handle, COPY_TABLE *tp);

/* Internal DataPath Functions */
int32_t MmwDemo_dssDataPathInit(void);
static int32_t MmwDemo_dssDataPathConfig(void);
static int32_t MmwDemo_dssDataPathStart(bool doRFStart);
static int32_t MmwDemo_dssDataPathStop(void);
static int32_t MmwDemo_dssDataPathProcessEvents(UInt event);

/* Internal MMWave Call back Functions */
static int32_t MmwDemo_dssMmwaveEventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload);
static void MmwDemo_dssMmwaveConfigCallbackFxn(MMWave_CtrlCfg* ptrCtrlCfg);
static void MmwDemo_dssMmwaveStartCallbackFxn(MMWave_CalibrationCfg* ptrCalibrationCfg);
static void MmwDemo_dssMmwaveStopCallbackFxn(void);


/* Internal Interrupt handler */
static void MmwDemo_dssChirpIntHandler(uintptr_t arg);
static void MmwDemo_dssFrameStartIntHandler(uintptr_t arg);

/* Internal mmwDemo Tasks running on DSS */
static void MmwDemo_dssInitTask(UArg arg0, UArg arg1);
static void MmwDemo_dssDataPathTask(UArg arg0, UArg arg1);
static void MmwDemo_dssMMWaveCtrlTask(UArg arg0, UArg arg1);

/* Internal Functions for sending detected objects */
static void MmwDemo_dssSendOutputToUART(UART_Handle uartHandle,
                                               MmwDemo_DSS_DataPathObj *obj);
static int32_t MmwDemo_dssSendProcessOutputToMSS
(
    MmwDemo_DSS_DataPathObj   *obj
);
void MmwDemo_dssDataPathOutputLogging(    MmwDemo_DSS_DataPathObj   * dataPathObj);

/**************************************************************************
 *************************** MmwDemo DSS Functions **************************
 **************************************************************************/
#define DEBUG
//#define DEBUG_CHIRP

#if (defined DEBUG) || (defined DEBUG_CHIRP)
#define GLBDEBUGBUFLEN (200)
uint32_t glbDebugBufCnt = 0;
uint32_t glbDebugBuf[GLBDEBUGBUFLEN];
#endif
/**
 *  @b Description
 *  @n
 *      Interrupt handler callback for chirp available. It runs in the ISR context.
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_dssChirpIntHandler(uintptr_t arg)
{
    /* Increment interrupt counter for debugging purpose */
    gMmwDssMCB.stats.chirpIntCounter++;

    gMmwDssMCB.dataPathObj.chirpStartTimeStamp = Cycleprofiler_getTimeStamp();
#ifdef DEBUG_CHIRP
        glbDebugBuf[glbDebugBufCnt++] = gMmwDssMCB.dataPathObj.chirpStartTimeStamp;
        if(glbDebugBufCnt >= GLBDEBUGBUFLEN) glbDebugBufCnt = 0;
#endif

    if (gMmwDssMCB.dataPathObj.chirpCount == 0)
    {
    	gMmwDssMCB.dataPathObj.frameStartTimeStamp = gMmwDssMCB.dataPathObj.chirpStartTimeStamp;
#ifdef DEBUG
    	glbDebugBuf[glbDebugBufCnt++] = gMmwDssMCB.stats.frameStartIntCounter;
    	if(glbDebugBufCnt >= GLBDEBUGBUFLEN) glbDebugBufCnt = 0;
    	glbDebugBuf[glbDebugBufCnt++] = gMmwDssMCB.dataPathObj.chirpStartTimeStamp;
    	if(glbDebugBufCnt >= GLBDEBUGBUFLEN) glbDebugBufCnt = 0;
#endif
    }
    /* Check if previous chirp processing has completed*/
    //MmwDemo_processChirp(&(gMmwDssMCB.dataPathObj));
    DebugP_assert(gMmwDssMCB.dataPathObj.chirpProcToken == 0);
    gMmwDssMCB.dataPathObj.chirpProcToken++;

    /* Post event to notify chirp available interrupt */
    Event_post(gMmwDssMCB.eventHandle, MMWDEMO_CHIRP_EVT);
}

/**
 *  @b Description
 *  @n
 *      Interrupt handler callback for frame start ISR.
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_dssFrameStartIntHandler(uintptr_t arg)
{
    /* Increment interrupt counter for debugging purpose */
    gMmwDssMCB.stats.frameStartIntCounter++;

#ifdef DEBUG
    glbDebugBuf[glbDebugBufCnt++] = 0xaaaa;
    if(glbDebugBufCnt >= GLBDEBUGBUFLEN) glbDebugBufCnt = 0;
	glbDebugBuf[glbDebugBufCnt++] = Cycleprofiler_getTimeStamp();
	if(glbDebugBufCnt >= GLBDEBUGBUFLEN) glbDebugBufCnt = 0;
#endif

    /* Check if previous chirp processing has completed*/
    DebugP_assert(gMmwDssMCB.dataPathObj.interFrameProcToken == 0);
    gMmwDssMCB.dataPathObj.interFrameProcToken++;

    /* Post event to notify frame start interrupt */
    Event_post(gMmwDssMCB.eventHandle, MMWDEMO_FRAMESTART_EVT);
}

/**
 *  @b Description
 *  @n
 *      Registered event callback function on DSS which is invoked by MMWAVE library when an event from the
 *      BSS is received.
 *
 *  @param[in]  msgId
 *      Message Identifier
 *  @param[in]  sbId
 *      Subblock identifier
 *  @param[in]  sbLen
 *      Length of the subblock
 *  @param[in]  payload
 *      Pointer to the payload buffer
 *
 *  @retval
 *      Always return 0. [Pass the event to the remote domain]
 */
static int32_t MmwDemo_dssMmwaveEventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

    /* Debug Message: */
    /*System_printf ("Debug: MMWDemoDSS received BSS Event MsgId: %d [Sub Block Id: %d Sub Block Length: %d]\n",
                    msgId, sbId, sbLen); */

    /* Process the received message: */
    switch (msgId)
    {
        case RL_RF_ASYNC_EVENT_MSG:
        {
            /* Received Asychronous Message: */
            switch (asyncSB)
            {
                case RL_RF_AE_CPUFAULT_SB:
                {
                    /* BSS fault */
                    DebugP_assert(0);
                    break;
                }
                case RL_RF_AE_ESMFAULT_SB:
                {
                    /* BSS fault */
                    DebugP_assert(0);
                    break;
                }
                case RL_RF_AE_INITCALIBSTATUS_SB:
                {
                    /* This event should be handled by mmwave internally, ignore the event here */
                    break;
                }

                case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
                {
                    /* Post event to datapath task notify BSS events */
                    Event_post(gMmwDssMCB.eventHandle, MMWDEMO_BSS_FRAME_TRIGGER_READY_EVT);

                    break;
                }
                case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
                {
                    /* Increment the statistics to reports that the calibration failed */
                    gMmwDssMCB.stats.numFailedTimingReports++;
                    break;
                }
                case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
                {
                    /* Increment the statistics to indicate that a calibration report was received */
                    gMmwDssMCB.stats.numCalibrationReports++;
                    break;
                }
                default:
                {
                    System_printf ("Error: Asynchronous Event SB Id %d not handled\n", asyncSB);
                    break;
                }
            }
            break;
        }
        default:
        {
            System_printf ("Error: Asynchronous message %d is NOT handled\n", msgId);
            break;
        }
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Registered config callback function on DSS which is invoked by MMWAVE library when the remote side
 *  has finished configure mmWaveLink and BSS. The configuration need to be saved on DSS and used for DataPath.
 *
 *  @param[in]  ptrCtrlCfg
 *      Pointer to the control configuration
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_dssMmwaveConfigCallbackFxn(MMWave_CtrlCfg* ptrCtrlCfg)
{
    /* Save the configuration */
    memcpy((void *)(&gMmwDssMCB.cfg.ctrlCfg), (void *)ptrCtrlCfg, sizeof(MMWave_CtrlCfg));

    gMmwDssMCB.stats.configEvt++;

    /* Post event to notify configuration is done */
    Event_post(gMmwDssMCB.eventHandle, MMWDEMO_CONFIG_EVT);

    return;
}

/**
 *  @b Description
 *  @n
 *      Registered open callback function which is invoked when the mmWave module
 *      has been opened on the MSS
 *
 *  @param[in]  ptrOpenCfg
 *      Pointer to the open configuration
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_dssMmwaveOpenCallbackFxn(MMWave_OpenCfg* ptrOpenCfg)
{
    /* Save the configuration */
    memcpy((void *)(&gMmwDssMCB.cfg.openCfg), (void *)ptrOpenCfg, sizeof(MMWave_OpenCfg));
    gMmwDssMCB.stats.openEvt++;
    return;
}

/**
 *  @b Description
 *  @n
 *      Registered close callback function which is invoked when the mmWave module
 *      has been close on the MSS
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_dssMmwaveCloseCallbackFxn(void)
{
    gMmwDssMCB.stats.closeEvt++;
    return;
}

/**
 *  @b Description
 *  @n
 *      Registered Start callback function on DSS which is invoked by MMWAVE library
 *    when the remote side has started mmWaveLink and BSS. This Callback function passes
 *    the event to DataPath task.
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_dssMmwaveStartCallbackFxn(MMWave_CalibrationCfg* ptrCalibrationCfg)
{
    gMmwDssMCB.stats.startEvt++;

    /* Post event to start is done */
    Event_post(gMmwDssMCB.eventHandle, MMWDEMO_START_EVT);
}

/**
 *  @b Description
 *  @n
 *      Registered Start callback function on DSS which is invoked by MMWAVE library
 *    when the remote side has stop mmWaveLink and BSS. This Callback function passes
 *    the event to DataPath task.
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_dssMmwaveStopCallbackFxn(void)
{
    gMmwDssMCB.stats.stopEvt++;

    /* Post event to stop is done */
    Event_post(gMmwDssMCB.eventHandle, MMWDEMO_STOP_EVT);
}

/**
 *  @b Description
 *  @n
 *      Function to send a message to peer through Mailbox virtural channel 
 *
 *  @param[in]  message
 *      Pointer to the Captuere demo message.  
 *
 *  @retval
 *      Success    - 0
 *      Fail       < -1 
 */
static int32_t MmwDemo_mboxWrite(MmwDemo_message    *message)
{
    int32_t                  retVal = -1;
    
    retVal = Mailbox_write (gMmwDssMCB.peerMailbox, (uint8_t*)message, sizeof(MmwDemo_message));
    if (retVal == sizeof(MmwDemo_message))
    {
        retVal = 0;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The Task is used to handle  the mmw demo messages received from Mailbox virtual channel.
 *
 *  @param[in]  arg0
 *      arg0 of the Task. Not used
 *  @param[in]  arg1
 *      arg1 of the Task. Not used
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_mboxReadTask(UArg arg0, UArg arg1)
{
    MmwDemo_message      message;
    int32_t              retVal = 0;

    /* wait for new message and process all the messsages received from the peer */
    while(1)
    {
        Semaphore_pend(gMmwDssMCB.mboxSemHandle, BIOS_WAIT_FOREVER);
        
        /* Read the message from the peer mailbox: We are not trying to protect the read
         * from the peer mailbox because this is only being invoked from a single thread */
        retVal = Mailbox_read(gMmwDssMCB.peerMailbox, (uint8_t*)&message, sizeof(MmwDemo_message));
        if (retVal < 0)
        {
            /* Error: Unable to read the message. Setup the error code and return values */
            System_printf ("Error: Mailbox read failed [Error code %d]\n", retVal);
        }
        else if (retVal == 0)
        {
            /* We are done: There are no messages available from the peer execution domain. */
            continue;
        }
        else
        {
            /* Flush out the contents of the mailbox to indicate that we are done with the message. This will
             * allow us to receive another message in the mailbox while we process the received message. */
            Mailbox_readFlush (gMmwDssMCB.peerMailbox);

            /* Process the received message: */
            switch (message.type)
            {
                case MMWDEMO_MSS2DSS_GUIMON_CFG:
                {
                    /* Save guimon configuration */
                    memcpy((void *)&gMmwDssMCB.cfg.guiMonSel, (void *)&message.body.guiMonSel, sizeof(MmwDemo_GuiMonSel));
                    break;
                }
                case MMWDEMO_MSS2DSS_CFAR_CFG:
                {
                    /* Save cfarRange configuration */
                    memcpy((void *)&gMmwDssMCB.dataPathObj.radarProcConfig.cfarConfig,
                           (void *)&message.body.cfar, sizeof(radarModuleCfarConfig));
                    break;
                }
                case MMWDEMO_MSS2DSS_DOA_CFG:
                {
                    /* Save cfarDoppler configuration */
                    memcpy((void *)&gMmwDssMCB.dataPathObj.radarProcConfig.doaConfig,
                           (void *)&message.body.doa, sizeof(radarModuleDoaConfig));
                    break;
                }

                case MMWDEMO_MSS2DSS_TRACKING_CFG:
                {
                    break;
                }

                case MMWDEMO_MSS2DSS_ADCBUFCFG:
                {
                    /* Save ADCBUF configuration */
                    memcpy((void *)&gMmwDssMCB.dataPathObj.adcBufCfg,
                           (void *)&message.body.adcBufCfg, sizeof(MmwDemo_ADCBufCfg));
                    break;
                }
                case MMWDEMO_MSS2DSS_DETOBJ_SHIPPED:
                {
                    gMmwDssMCB.loggingBufferAvailable = 1;
                    break;
                }
                case MMWDEMO_MSS2DSS_SET_DATALOGGER:
                {
                    gMmwDssMCB.cfg.dataLogger = message.body.dataLogger;
                    break;
                }
                default:
                {
                    /* Message not support */
                    System_printf ("Error: unsupport Mailbox message id=%d\n", message.type);
                    DebugP_assert(0);
                    break;
                }
            }
        }
    }
}

/**
 *  @b Description
 *  @n
 *      This function is a callback funciton that invoked when a message is received from the peer.
 *
 *  @param[in]  handle
 *      Handle to the Mailbox on which data was received
 *  @param[in]  peer
 *      Peer from which data was received
 
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mboxCallback
(
   // Mailbox_Handle  handle,
        Mbox_Handle  handle,
        Mailbox_Type    peer
)
{
    /* Message has been received from the peer endpoint. Wakeup the mmWave thread to process
     * the received message. */
    Semaphore_post (gMmwDssMCB.mboxSemHandle);
}

/**
 *  @b Description
 *  @n
 *      Function to send detected object to uart logger.
 *
 *  @param[in]  uartHandle
 *      Handle of the UART logger
 *  @param[in]  obj
 *      Handle of data path object
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dssSendOutputToUART(UART_Handle uartHandle,
                                        MmwDemo_DSS_DataPathObj *obj)
{
    //uint32_t i;
    static uint16_t magic_word[4] = {0x0102, 0x0304, 0x0506, 0x0708};
    uint32_t noiseEnergy = 0;
    uint32_t cycles = 0;
    radarProcessOutputToTracker * outputObjBuff;
    uint32_t numObj;
    MmwDemo_GuiMonSel *pGuiMonSel = (MmwDemo_GuiMonSel *) &gMmwDssMCB.cfg.guiMonSel;

    outputObjBuff = (radarProcessOutputToTracker *)obj->outputDataToArm;
    numObj = outputObjBuff->object_count;

    /* Sending message header
       Magic word: 8 bytes
       Inter frame processing cycles: 4 bytes
       Noise energy: 4 bytes
       Number of Objects: 4 byte
     */
    UART_writePolling (uartHandle,
            (uint8_t*)magic_word,
            sizeof(magic_word));
    UART_writePolling (uartHandle,
            (uint8_t*) &cycles,
            sizeof(uint32_t));
    UART_writePolling (uartHandle,
            (uint8_t*)&noiseEnergy,
            sizeof(uint32_t));
    UART_writePolling (uartHandle,
            (uint8_t*)&numObj,
            sizeof(uint32_t));

    /* Sending detected Objects */
    if (pGuiMonSel->detectedObjects == 1)
    {
        UART_writePolling (uartHandle,
                (uint8_t*)outputObjBuff,
                sizeof(radarProcessOutputToTracker));
    }


#if 0
    /* Sending range Doppler log Magnitude matrix  */
    if (pGuiMonSel->logMagRange == 1)
    {
        for(i = 0; i < obj->numRangeBins; i++)
        {
            UART_writePolling (uartHandle,
                    (uint8_t*)mmwRangeDopplerLogMagMatrix,
                    sizeof(uint16_t));
            mmwRangeDopplerLogMagMatrix += obj->numDopplerBins;
        }
    }
    /* Sending range Azimuth Heat Map */
    if (pGuiMonSel->rangeAzimuthHeatMap == 1)
    {
        UART_writePolling (uartHandle,
                (uint8_t *) obj->azimuthStaticHeatMap,
                obj->numRangeBins * obj->numVirtualAntAzim * sizeof(cmplx16ImRe_t));
    }

    /* Sending range Doppler Heat Map  */
    if (pGuiMonSel->rangeDopplerHeatMap == 1)
    {
        UART_writePolling (uartHandle,
                (uint8_t*)obj->detMatrix,
            obj->numRangeBins * obj->numDopplerBins * sizeof(uint16_t));
    }
#endif
}

/**
 *  @b Description
 *  @n
 *      Function to send detected objects to MSS logger.
 *
 *  @param[in]  ptrOutputBuffer
 *      Pointer to the output buffer
 *  @param[in]  outputBufSize
 *      Size of the output buffer
 *  @param[in]  obj
 *      Handle to the Data Path Object
 *
 *  @retval
 *      =0    Success
 *      <0    Failed
 */
int32_t MmwDemo_dssSendProcessOutputToMSS
(
    MmwDemo_DSS_DataPathObj   *obj
)
{
    //uint32_t            i;
    //uint8_t             *ptrCurrDetOutput;
    uint32_t            totalSize, infoSize;
    int32_t             retVal = 0;
	MmwDemo_detOutputHdr           *ptrDetOutputHdr = (MmwDemo_detOutputHdr *)&(obj->outputDataToArm->outputHeader);
    MmwDemo_message     message;
    radarProcessOutputToTracker *outputObjBuff = (radarProcessOutputToTracker *)&(obj->outputDataToArm->outputToTracker);
    //MmwDemo_GuiMonSel   *pGuiMonSel;

    /* Validate input params */
    if(ptrDetOutputHdr == NULL)
    {
        retVal = -1;
        /*work around MING MING MING */
        cache_wbInvAllL2Wait();
        goto Exit;
    }
    ptrDetOutputHdr->magicWord[0] = 0x0102;
    ptrDetOutputHdr->magicWord[1] = 0x0304;
    ptrDetOutputHdr->magicWord[2] = 0x0506;
    ptrDetOutputHdr->magicWord[3] = 0x0708;

    ptrDetOutputHdr->chirpProcessingMarginInUsec = (uint32_t)obj->cycleLog.chirpProcMarginCurrInusec;
    ptrDetOutputHdr->frameProcessingMarginInUsec = (uint32_t)obj->cycleLog.frameProcMarginCurrInusec;
    

        ptrDetOutputHdr->numDetectedObj = outputObjBuff->object_count;
        ptrDetOutputHdr->frameNumber = obj->frameCount;
        infoSize    =   obj->sizePointCloudInfo;
        totalSize   =   sizeof(MmwDemo_detOutputHdr) + infoSize;

    if( retVal == 0)
    {
        /* Send a message to MSS to log the output data */
        memset((void *)&message, 0, sizeof(MmwDemo_message));

        message.type = MMWDEMO_DSS2MSS_DETOBJ_READY;
        message.body.detObj.detObjOutAddress = (uint32_t)ptrDetOutputHdr;
        message.body.detObj.detObjOutsize= totalSize;
        /*work around MING MING MING */
        cache_wbInvAllL2Wait();

        if (MmwDemo_mboxWrite(&message) != 0)
        {
            retVal = -1;
        }
    }
Exit:
#ifdef DEBUG
    glbDebugBuf[glbDebugBufCnt++] = 0xbbbb;
    if(glbDebugBufCnt >= GLBDEBUGBUFLEN) glbDebugBufCnt = 0;
	glbDebugBuf[glbDebugBufCnt++] = Cycleprofiler_getTimeStamp();
	if(glbDebugBufCnt >= GLBDEBUGBUFLEN) glbDebugBufCnt = 0;
#endif
    /*work around MING MING MING */
    cache_wbInvAllL2Wait();
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function to send data path detection output.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dssDataPathOutputLogging(MmwDemo_DSS_DataPathObj   * dataPathObj)
{
    if(gMmwDssMCB.cfg.dataLogger == 1)
    {
        /* Send to on board UART */
        MmwDemo_dssSendOutputToUART(gMmwDssMCB.loggingUartHandle,  dataPathObj);
    }
    else 
    {
        /* Sending detected objects to logging buffer and shipped out from MSS UART */
        if (gMmwDssMCB.loggingBufferAvailable == 1)
        {
            /* Set the logging buffer available flag to be 0 */
            gMmwDssMCB.loggingBufferAvailable = 0;

            /* Save output in logging buffer - HSRAM memory and a message is sent to MSS to notify
               logging buffer is ready */
            if (MmwDemo_dssSendProcessOutputToMSS(dataPathObj) < 0)
            {
                /* Increment logging error */
                gMmwDssMCB.stats.detObjLoggingErr++;
            }
        }
        else
        {
            /* Logging buffer is not available, skip saving detected objectes to logging buffer */
            gMmwDssMCB.stats.detObjLoggingBufNotRdyCnt++;
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Initialization on DSS.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t MmwDemo_dataPathAdcBufInit(MmwDemo_DSS_DataPathObj *obj)
{
    ADCBuf_Params       ADCBufparams;

    /*****************************************************************************
     * Initialize ADCBUF driver
     *****************************************************************************/
    ADCBuf_init();

    /* ADCBUF Params initialize */
    ADCBuf_Params_init(&ADCBufparams);
    ADCBufparams.chirpThresholdPing = 1;
    ADCBufparams.chirpThresholdPong = 1;
    ADCBufparams.continousMode  = 0;

    /* Open ADCBUF driver */
    obj->adcbufHandle = ADCBuf_open(0, &ADCBufparams);
    if (obj->adcbufHandle == NULL)
    {
        System_printf("Error: MMWDemoDSS Unable to open the ADCBUF driver\n");
        return -1;
    }
    System_printf("Debug: MMWDemoDSS ADCBUF Instance(0) %p has been opened successfully\n", obj->adcbufHandle);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Performs linker generated copy table copy using EDMA. Currently this is
 *      used to page in fast code from L3 to L1PSRAM.
 *  @param[in]  handle EDMA handle
 *  @param[in]  tp Pointer to copy table
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_copyTable(EDMA_Handle handle, COPY_TABLE *tp)
{
    uint16_t i;
    COPY_RECORD crp;
    uint32_t loadAddr;
    uint32_t runAddr;

    for (i = 0; i < tp->num_recs; i++)
    {
        crp = tp->recs[i];
        loadAddr = (uint32_t)crp.load_addr;
        runAddr = (uint32_t)crp.run_addr;

        /* currently we use only one count of EDMA which is 16-bit so we cannot
           handle tables bigger than 64 KB */
        DebugP_assert(crp.size <= 65536U);

        if (crp.size)
        {
            MmwDemo_edmaBlockCopy(handle, loadAddr, runAddr, crp.size);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Performs simple block copy using EDMA. Used for the purpose of copying
 *      linker table for L3 to L1PSRAM copy. memcpy cannot be used because there is
 *      no data bus access to L1PSRAM.
 *
 *  @param[in]  handle EDMA handle
 *  @param[in]  loadAddr load address
 *  @param[in]  runAddr run address
 *  @param[in]  size size in bytes
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_edmaBlockCopy(EDMA_Handle handle, uint32_t loadAddr, uint32_t runAddr, uint16_t size)
{
    EDMA_channelConfig_t config;
    volatile bool isTransferDone;

    config.channelId = EDMA_TPCC0_REQ_FREE_0;
    config.channelType = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    config.paramId = (uint16_t)EDMA_TPCC0_REQ_FREE_0;
    config.eventQueueId = 0;

    config.paramSetConfig.sourceAddress = (uint32_t) SOC_translateAddress((uint32_t)loadAddr,
        SOC_TranslateAddr_Dir_TO_EDMA, NULL);
    config.paramSetConfig.destinationAddress = (uint32_t) SOC_translateAddress((uint32_t)runAddr,
        SOC_TranslateAddr_Dir_TO_EDMA, NULL);

    config.paramSetConfig.aCount = size;
    config.paramSetConfig.bCount = 1U;
    config.paramSetConfig.cCount = 1U;
    config.paramSetConfig.bCountReload = 0U;

    config.paramSetConfig.sourceBindex = 0U;
    config.paramSetConfig.destinationBindex = 0U;

    config.paramSetConfig.sourceCindex = 0U;
    config.paramSetConfig.destinationCindex = 0U;

    config.paramSetConfig.linkAddress = EDMA_NULL_LINK_ADDRESS;
    config.paramSetConfig.transferType = (uint8_t)EDMA3_SYNC_A;
    config.paramSetConfig.transferCompletionCode = (uint8_t) EDMA_TPCC0_REQ_FREE_0;
    config.paramSetConfig.sourceAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;
    config.paramSetConfig.destinationAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;

    /* don't care because of linear addressing modes above */
    config.paramSetConfig.fifoWidth = (uint8_t) EDMA3_FIFO_WIDTH_8BIT;

    config.paramSetConfig.isStaticSet = false;
    config.paramSetConfig.isEarlyCompletion = false;
    config.paramSetConfig.isFinalTransferInterruptEnabled = true;
    config.paramSetConfig.isIntermediateTransferInterruptEnabled = false;
    config.paramSetConfig.isFinalChainingEnabled = false;
    config.paramSetConfig.isIntermediateChainingEnabled = false;
    config.transferCompletionCallbackFxn = NULL;
    config.transferCompletionCallbackFxnArg = NULL;

    if (EDMA_configChannel(handle, &config, false) != EDMA_NO_ERROR)
    {
        DebugP_assert(0);
    }

    if (EDMA_startDmaTransfer(handle, config.channelId) != EDMA_NO_ERROR)
    {
        DebugP_assert(0);
    }
    
    /* wait until transfer done */
    do {
        if (EDMA_isTransferComplete(handle,
                config.paramSetConfig.transferCompletionCode,
                (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            DebugP_assert(0);
        }
    } while (isTransferDone == false);

    /* make sure to disable channel so it is usable later */
    EDMA_disableChannel(handle, config.channelId, config.channelType);
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Initialization on DSS.
 *
 *  @retval
 *      Not Applicable.
 */
int32_t MmwDemo_dssDataPathInit(void)
{
    MmwDemo_DSS_DataPathObj *obj;
    int32_t retVal;
    SOC_SysIntListenerCfg  socIntCfg;
    int32_t errCode;

    /* Get DataPath Object handle */
    obj = &gMmwDssMCB.dataPathObj;
    
    /* Initialize entire data path object to a known state */
    memset((void *)obj, 0, sizeof(MmwDemo_DSS_DataPathObj));

    MmwDemo_dataPathInit1Dstate(obj);    
    retVal = MmwDemo_dataPathInitEdma(obj);
    if (retVal < 0)
    {
        return -1;
    }
    
    /* Copy code from L3 to L1PSRAM, this code related to data path processing */
    MmwDemo_copyTable(obj->edmaHandle[0], &_MmwDemo_fastCode_L1PSRAM_copy_table);

    retVal = MmwDemo_dataPathAdcBufInit(obj);
    if (retVal < 0)
    {
        return -1;
    }

    /* Register chirp interrupt listener */
    socIntCfg.systemInterrupt  = SOC_XWR16XX_DSS_INTC_EVENT_CHIRP_AVAIL;
    socIntCfg.listenerFxn      = MmwDemo_dssChirpIntHandler;
    socIntCfg.arg              = (uintptr_t)NULL;
    if (SOC_registerSysIntListener(gMmwDssMCB.socHandle, &socIntCfg, &errCode) == NULL)
    {
        System_printf("Error: Unable to register chirp interrupt listener , error = %d\n", errCode);
        return -1;
    }

    /* Register frame start interrupt listener */
    socIntCfg.systemInterrupt  = SOC_XWR16XX_DSS_INTC_EVENT_FRAME_START;
    socIntCfg.listenerFxn      = MmwDemo_dssFrameStartIntHandler;
    socIntCfg.arg              = (uintptr_t)NULL;
    if (SOC_registerSysIntListener(gMmwDssMCB.socHandle, &socIntCfg, &errCode) == NULL)
    {
        System_printf("Error: Unable to register frame start interrupt listener , error = %d\n", errCode);
        return -1;
    }

    /* Initialize detected objects logging */
    gMmwDssMCB.loggingBufferAvailable = 1;
    return 0;

}

/**
 *  @b Description
 *  @n
 *      Function to configure ADCBUF driver based on CLI inputs.
 *  @param[out] numRxChannels Number of receive channels.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t MmwDemo_dssDataPathConfigAdcBuf(uint8_t *numRxChannels)
{
    int64_t                     dummy;
    uint32_t                    chirpThreshold;
    ADCBuf_dataFormat           dataFormat;
    ADCBuf_RxChanConf           rxChanConf;
    int32_t                     retVal;
    uint8_t                     channel;
    uint8_t                     numBytePerSample = 0;
    MMWave_OpenCfg*             ptrOpenCfg;
    MmwDemo_DSS_DataPathObj*    ptrDataPathObj;
    uint32_t                    rxChanMask = 0xF;

    /* Get data path object and control configuration */
    ptrOpenCfg     = &gMmwDssMCB.cfg.openCfg;
    ptrDataPathObj = &gMmwDssMCB.dataPathObj;
    *numRxChannels = 0;

    /*****************************************************************************
     * Data path :: ADCBUF driver Configuration
     *****************************************************************************/
    /* Validate the adcFmt */
    if(ptrDataPathObj->adcBufCfg.adcFmt == 1)
    {
        /* Real dataFormat has 2 bytes */
        numBytePerSample =  2;
    }
    else if(ptrDataPathObj->adcBufCfg.adcFmt == 0)
    {
        /* Complex dataFormat has 4 bytes */
        numBytePerSample =  4;
    }
    else
    {
        DebugP_assert(0); /* Data format not supported */
    }

    /* On XWR16xx only channel non-interleaved mode is supported */
    if(ptrDataPathObj->adcBufCfg.chInterleave != 1)
    {
        DebugP_assert(0); /* Not supported */
    }

    /* for TM demo only Q/I format is supported */
    if(ptrDataPathObj->adcBufCfg.iqSwapSel != 1)
    {
        DebugP_assert(0); /* Not supported */
    }

    /* Disable all ADCBuf channels */
    if ((retVal = ADCBuf_control(ptrDataPathObj->adcbufHandle, ADCBufMMWave_CMD_CHANNEL_DISABLE, (void *)&rxChanMask)) < 0)
    {
       System_printf("Error: Disable ADCBuf channels failed with [Error=%d]\n", retVal);
       return retVal;
    }

    /* Populate data format from configuration */
    dataFormat.adcOutFormat       = ptrDataPathObj->adcBufCfg.adcFmt;
    dataFormat.channelInterleave  = ptrDataPathObj->adcBufCfg.chInterleave;
    dataFormat.sampleInterleave   = ptrDataPathObj->adcBufCfg.iqSwapSel;

    retVal = ADCBuf_control(ptrDataPathObj->adcbufHandle, ADCBufMMWave_CMD_CONF_DATA_FORMAT, (void *)&dataFormat);
    if (retVal < 0)
    {
        System_printf ("Error: MMWDemoDSS Unable to configure the data formats\n");
        return -1;
    }

    /* Enable Rx Channels */
    for (channel = 0; channel < 4; channel++)
    {
        if(ptrOpenCfg->chCfg.rxChannelEn & (0x1U << channel))
        {
        	int32_t tempNumSamples;

        	tempNumSamples = ptrDataPathObj->numAdcSamples;

            /* Populate the receive channel configuration: */
            rxChanConf.channel = channel;
            rxChanConf.offset  =  tempNumSamples * numBytePerSample * (*numRxChannels);
            retVal = ADCBuf_control(ptrDataPathObj->adcbufHandle, ADCBufMMWave_CMD_CHANNEL_ENABLE, (void *)&rxChanConf);
            if (retVal < 0)
            {
                System_printf("Error: MMWDemoDSS ADCBuf Control for Channel %d Failed\n");
                return -1;
            }

            /* Track the number of receive channels: */
            *numRxChannels += 1;
        }
    }

    /* Set ping/pong chirp threshold: */
    /* ADCBuf control function requires argument alignment at 4 bytes boundary */
    chirpThreshold = ptrDataPathObj->adcBufCfg.chirpThreshold;

    retVal = ADCBuf_control(ptrDataPathObj->adcbufHandle, ADCBufMMWave_CMD_SET_PING_CHIRP_THRESHHOLD,
                            (void *)&chirpThreshold);
    if(retVal < 0)
    {
        System_printf("Error: ADCbuf Ping Chirp Threshold Failed with Error[%d]\n", retVal);
        dummy = -1;
        return (int32_t)dummy;
    }
    retVal = ADCBuf_control(ptrDataPathObj->adcbufHandle, ADCBufMMWave_CMD_SET_PONG_CHIRP_THRESHHOLD,
                            (void *)&chirpThreshold);
    if(retVal < 0)
    {
        System_printf("Error: ADCbuf Pong Chirp Threshold Failed with Error[%d]\n", retVal);
        return -1;
    }
    
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Configuration on DSS.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t MmwDemo_dssDataPathConfig(void)
{
    int32_t             retVal;
    uint8_t             numRxChannels = 0;
    //MMWave_CtrlCfg      *ptrCtrlCfg;
    MmwDemo_DSS_DataPathObj *dataPathObj;
    //uint32_t numTxAntAzim = 0;

    /* Get data path object and control configuration */
    //ptrCtrlCfg   = &gMmwDssMCB.cfg.ctrlCfg;
    dataPathObj  = &gMmwDssMCB.dataPathObj;


    /*****************************************************************************
     * Data path :: Algorithm Configuration
     *****************************************************************************/
    MmwDemo_dataPathConfigBuffers(SOC_XWR16XX_DSS_ADCBUF_BASE_ADDRESS, &gMmwDssMCB.cfg, dataPathObj);


    retVal = MmwDemo_dssDataPathConfigAdcBuf(&numRxChannels);
    if (retVal < 0)
    {
        return -1;
    }


    MmwDemo_dataPathConfigEdma(dataPathObj);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to start Data Path on DSS.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t MmwDemo_dssDataPathStart(bool doRFStart)
{
    int32_t    errCode;
    MMWave_CalibrationCfg   calibrationCfg;

    gMmwDssMCB.dataPathObj.chirpProcToken = 0;
    gMmwDssMCB.dataPathObj.interFrameProcToken = 0;
    gMmwDssMCB.dataPathObj.cycleLog.chirpProcMarginMaxInusec = 0.f;
    gMmwDssMCB.dataPathObj.cycleLog.chirpProcMarginMinInusec = 1.0e15;
    gMmwDssMCB.dataPathObj.cycleLog.frameProcMarginMaxInusec = 0.f;
    gMmwDssMCB.dataPathObj.cycleLog.frameProcMarginMinInusec = 1.0e15;

    if (doRFStart)
    {
        /* Initialize the calibration configuration: */
        memset ((void*)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));

        /* Populate the calibration configuration: */
        calibrationCfg.dfeDataOutputMode                          = MMWave_DFEDataOutputMode_FRAME;
        calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = true;
        calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = true;
        calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

        /* Start the mmWave module: The configuration has been applied successfully. */
        if (MMWave_start (gMmwDssMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
        {
            /* Error: Unable to start the mmWave control */
            System_printf ("Error: MMWDemoDSS mmWave Start failed [Error code %d]\n", errCode);
        return -1;
    }
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to process Data Path events at runtime.
 *
 *  @param[in]  event
 *      Data Path Event
 *
 *  @retval
 *      Not Applicable.
 */
//uint32_t negMinChirpNumber = 0;

static int32_t MmwDemo_dssDataPathProcessEvents(UInt event)
{
    MmwDemo_DSS_DataPathObj *dataPathObj;
    volatile uint32_t startTime;

    dataPathObj = &gMmwDssMCB.dataPathObj;

    /* Handle dataPath events */
    switch(event)
    {
        case MMWDEMO_CHIRP_EVT:
            /* Increment event stats */
            gMmwDssMCB.stats.chirpEvt++;

            MmwDemo_processChirp(dataPathObj);
            dataPathObj->chirpProcToken--;
            dataPathObj->chirpEndTimeStamp = Cycleprofiler_getTimeStamp();
            dataPathObj->cycleLog.chirpProcMarginCurrInusec = 1.0e6 * dataPathObj->radarProcConfig.chirpInterval
            		- DSP_CLOCK_USEC_PER_CYCLE * (float)(dataPathObj->chirpEndTimeStamp - dataPathObj->chirpStartTimeStamp);

            if (gMmwDssMCB.stats.chirpIntCounter > 4)
            {
				if (dataPathObj->cycleLog.chirpProcMarginMaxInusec < dataPathObj->cycleLog.chirpProcMarginCurrInusec)
					dataPathObj->cycleLog.chirpProcMarginMaxInusec = dataPathObj->cycleLog.chirpProcMarginCurrInusec;
				if (dataPathObj->cycleLog.chirpProcMarginMinInusec > dataPathObj->cycleLog.chirpProcMarginCurrInusec)
					dataPathObj->cycleLog.chirpProcMarginMinInusec = dataPathObj->cycleLog.chirpProcMarginCurrInusec;
            }

            if (dataPathObj->chirpCount == 0)
            {
                MmwDemo_waitEndOfChirps(dataPathObj);

#ifdef DEBUG
                glbDebugBuf[glbDebugBufCnt++] = 0xcccc;
                if(glbDebugBufCnt >= GLBDEBUGBUFLEN) glbDebugBufCnt = 0;
                glbDebugBuf[glbDebugBufCnt++] = Cycleprofiler_getTimeStamp();
                if(glbDebugBufCnt >= GLBDEBUGBUFLEN) glbDebugBufCnt = 0;
#endif
                MmwDemo_interFrameProcessing(dataPathObj);
#ifdef DEBUG
                glbDebugBuf[glbDebugBufCnt++] = 0xdddd;
                if(glbDebugBufCnt >= GLBDEBUGBUFLEN) glbDebugBufCnt = 0;
				glbDebugBuf[glbDebugBufCnt++] = Cycleprofiler_getTimeStamp();
				if(glbDebugBufCnt >= GLBDEBUGBUFLEN) glbDebugBufCnt = 0;
#endif
                /* Sending detected objects to logging buffer */
                MmwDemo_dssDataPathOutputLogging (dataPathObj); 
                dataPathObj->frameProcDoneTimeStamp = Cycleprofiler_getTimeStamp();

                dataPathObj->cycleLog.frameProcMarginCurrInusec = (float)(dataPathObj->radarProcConfig.framePeriod) * 1000.f - DSP_CLOCK_USEC_PER_CYCLE * (float)((int32_t)(dataPathObj->frameProcDoneTimeStamp) - (int32_t)(dataPathObj->frameStartTimeStamp));
                if(dataPathObj->cycleLog.frameProcMarginCurrInusec > 0.f)
                {
					if (dataPathObj->cycleLog.frameProcMarginMaxInusec < dataPathObj->cycleLog.frameProcMarginCurrInusec)
						dataPathObj->cycleLog.frameProcMarginMaxInusec = dataPathObj->cycleLog.frameProcMarginCurrInusec;
					if (dataPathObj->cycleLog.frameProcMarginMinInusec > dataPathObj->cycleLog.frameProcMarginCurrInusec)
						dataPathObj->cycleLog.frameProcMarginMinInusec = dataPathObj->cycleLog.frameProcMarginCurrInusec;
                }
                dataPathObj->interFrameProcToken--;
                dataPathObj->frameCount++;
            }
            break;

        case MMWDEMO_FRAMESTART_EVT:
            /* Increment event stats */
            gMmwDssMCB.stats.frameStartEvt++;
            DebugP_assert(dataPathObj->chirpCount == 0);
            break;

        case MMWDEMO_BSS_FRAME_TRIGGER_READY_EVT:
            /* Increment event stats */
            gMmwDssMCB.stats.frameTrigEvt++;

            break;

        default:
            break;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to stop Data Path on DSS. Assume BSS has been stopped by mmWave already.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t MmwDemo_dssDataPathStop(void)
{
    return 0;
}


/**
 *  @b Description
 *  @n
 *      The task is used to provide an execution context for the mmWave
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_dssMMWaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    while (1)
    {
        /* Execute the mmWave control module: */
        if (MMWave_execute (gMmwDssMCB.ctrlHandle, &errCode) < 0)
            System_printf ("Error: MMWDemoDSS mmWave control execution failed [Error code %d]\n", errCode);
    }
}

/**
 *  @b Description
 *  @n
 *      Data Path main task that handles events from remote and do dataPath processing.
 *  This task is created when MSS is responsible for the mmwave Link and DSS is responsible
 *  for data path processing.
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_dssDataPathTask(UArg arg0, UArg arg1)
{
    int32_t       retVal = 0;
    UInt          event;

    /************************************************************************
     * Data Path :: Config
     ************************************************************************/

    /* Waiting for Config event from Remote - MSS */
    Event_pend(gMmwDssMCB.eventHandle, MMWDEMO_CONFIG_EVT, Event_Id_NONE, BIOS_WAIT_FOREVER);
    if ((retVal = MmwDemo_dssDataPathConfig()) < 0 )
    {
        System_printf ("Debug: MMWDemoDSS Data Path config failed with Error[%d]\n",retVal);
        goto exit;
    }
	else
	{
		// send config ready message to MSS along with some static info
        /* Send a message to MSS to log the output data */
		MmwDemo_message     message;
        memset((void *)&message, 0, sizeof(MmwDemo_message));

        message.type = MMWDEMO_DSS2MSS_CONFIGDONE;
        message.body.dssStaticInfo.heatmapAddress = (uint32_t)(gMmwDssMCB.dataPathObj.radarProcConfig.heatMapMem);
        message.body.dssStaticInfo.heatmapRowLen = (uint32_t) (gMmwDssMCB.dataPathObj.numRangeBins);
        message.body.dssStaticInfo.heatmapNumRows = (uint32_t) (gMmwDssMCB.dataPathObj.numAzimuthBins);

        if (MmwDemo_mboxWrite(&message) != 0)
        {
            retVal = -1;
        }
		
	}
   
    /************************************************************************
     * Data Path :: Start, mmwaveLink start will be triggered from DSS!
     ************************************************************************/
    if ((retVal = MmwDemo_dssDataPathStart(true)) < 0 )
    {
        System_printf ("Debug: MMWDemoDSS Data Path start failed with Error[%d]\n",retVal);
        goto exit;
    }

    gMmwDssMCB.state = MmwDemo_DSS_STATE_STARTED;

    /************************************************************************
     * Data Path :: Main loop
     ************************************************************************/
    while (1)
    {
        event = Event_pend(gMmwDssMCB.eventHandle,
                          Event_Id_NONE,
                          MMWDEMO_FRAMESTART_EVT | MMWDEMO_CHIRP_EVT |
                          MMWDEMO_STOP_EVT | MMWDEMO_CONFIG_EVT,
                          BIOS_WAIT_FOREVER);

        if(event & MMWDEMO_STOP_EVT)
        {
            System_printf ("Debug: MMWDemoDSS Received STOP Event\n");

            if(gMmwDssMCB.state == MmwDemo_DSS_STATE_STARTED)
            {
                /************************************************************************
                 * Local Data Path Stop - TODO
                 ************************************************************************/
                if ((retVal = MmwDemo_dssDataPathStop()) < 0 )
                {
                    System_printf ("Debug: MMWDemoDSS Data Path stop failed with Error[%d]\n",retVal);
                }
                System_printf ("Debug: MMWDemoDSS Data Path stop succeeded\n");

                /* Change state to "STOPPED" state */
                gMmwDssMCB.state = MmwDemo_DSS_STATE_STOPPED;

                /* Update stats */
                gMmwDssMCB.stats.chirpEvt = 0;
                gMmwDssMCB.stats.frameStartEvt = 0;
                gMmwDssMCB.stats.frameTrigEvt = 0;
            }
            else
            {
                // Ignore the stop event
            }
        }

        /************************************************************************
         * Data Path process frame start event
         ************************************************************************/
        if(event & MMWDEMO_FRAMESTART_EVT)
        {
            if(gMmwDssMCB.state == MmwDemo_DSS_STATE_STARTED)
            {
                if ((retVal = MmwDemo_dssDataPathProcessEvents(MMWDEMO_FRAMESTART_EVT)) < 0 )
                {
                    System_printf ("Error: MMWDemoDSS Data Path process frame start event failed with Error[%d]\n",
                                  retVal);
                }
            }
        }

        /************************************************************************
         * Data Path process chirp event
         ************************************************************************/
        if(event & MMWDEMO_CHIRP_EVT)
        {
            if(gMmwDssMCB.state == MmwDemo_DSS_STATE_STARTED)
            {
                if ((retVal = MmwDemo_dssDataPathProcessEvents(MMWDEMO_CHIRP_EVT)) < 0 )
                {
                    System_printf ("Error: MMWDemoDSS Data Path process chirp event failed with Error[%d]\n",
                                  retVal);
                }
            }
        }

        /************************************************************************
         * Data Path re-config, only supported reconfiguration in stop state
         ************************************************************************/
        if(event & MMWDEMO_CONFIG_EVT)
        {
            if(gMmwDssMCB.state == MmwDemo_DSS_STATE_STOPPED)
            {
                if ((retVal = MmwDemo_dssDataPathConfig()) < 0 )
                {
                    System_printf ("Debug: MMWDemoDSS Data Path config failed with Error[%d]\n",retVal);
                    goto exit;
                }

                /************************************************************************
                 * Data Path :: Start, mmwaveLink start will be triggered from DSS!
                 ************************************************************************/
                if ((retVal = MmwDemo_dssDataPathStart(true)) < 0 )
                {
                    System_printf ("Error: MMWDemoDSS Data Path start failed with Error[%d]\n",retVal);
                    goto exit;
                }
                gMmwDssMCB.state = MmwDemo_DSS_STATE_STARTED;
            }
            else
            {
                System_printf ("Error: MMWDemoDSS Data Path config event in wrong state[%d]\n", gMmwDssMCB.state);
                goto exit;
            }
        }
    }
exit:
    System_printf("Debug: MMWDemoDSS Data path exit\n");

}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_dssInitTask(UArg arg0, UArg arg1)
{
    int32_t             errCode;
    MMWave_InitCfg      initCfg;
    Task_Params         taskParams;
    UART_Params         uartParams;
    Semaphore_Params    semParams;
    Mailbox_Config      mboxCfg;
    Error_Block         eb;

    /* Initialize the UART */
    UART_init();

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_DSS);

    /*****************************************************************************
     * Open & configure the drivers:
     *****************************************************************************/

    /* Setup the default logging UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.clockFrequency  = DSS_SYS_VCLK; 
    uartParams.baudRate        = gMmwDssMCB.cfg.loggingBaudRate;
    uartParams.isPinMuxDone    = 1;

    /* Open the Logging UART Instance: */
    gMmwDssMCB.loggingUartHandle = UART_open(0, &uartParams);
    if (gMmwDssMCB.loggingUartHandle == NULL)
    {
        System_printf("Error: Unable to open the Logging UART Instance\n");
        return;
    }
    System_printf("Debug: Logging UART Instance %p has been opened successfully\n", gMmwDssMCB.loggingUartHandle);

    /*****************************************************************************
     * Create mailbox Semaphore:
     *****************************************************************************/
    /* Create a binary semaphore which is used to handle mailbox interrupt. */
    Semaphore_Params_init(&semParams);
    semParams.mode             = Semaphore_Mode_BINARY;
    gMmwDssMCB.mboxSemHandle = Semaphore_create(0, &semParams, NULL);

    /* Setup the default mailbox configuration */
    Mailbox_Config_init(&mboxCfg);

    /* Setup the configuration: */
    mboxCfg.chType       = MAILBOX_CHTYPE_MULTI;
    mboxCfg.chId         = MAILBOX_CH_ID_0;
    mboxCfg.writeMode    = MAILBOX_MODE_BLOCKING;
    mboxCfg.readMode     = MAILBOX_MODE_CALLBACK;
    mboxCfg.readCallback = &MmwDemo_mboxCallback;

    gMmwDssMCB.peerMailbox = Mailbox_open(MAILBOX_TYPE_MSS, &mboxCfg, &errCode);
    if (gMmwDssMCB.peerMailbox == NULL)
    {
        /* Error: Unable to open the mailbox */
        System_printf("Error: Unable to open the Mailbox to the DSS [Error code %d]\n", errCode);
        return;
    }
    else
    {
        /* Debug Message: */
        System_printf("Debug: DSS Mailbox Handle %p\n", gMmwDssMCB.peerMailbox);
        
        /* Create task to handle mailbox messges */
        Task_Params_init(&taskParams);
        taskParams.stackSize = 800;
        Task_create(MmwDemo_mboxReadTask, &taskParams, NULL);
    }

    /*****************************************************************************
     * Create Event to handle mmwave callback and system datapath events
     *****************************************************************************/
    /* Default instance configuration params */
    Error_init(&eb);
    gMmwDssMCB.eventHandle = Event_create(NULL, &eb);
    if (gMmwDssMCB.eventHandle == NULL)
    {
        /* FATAL_TBA */
        System_printf("Error: MMWDemoDSS Unable to create an event handle\n");
        return ;
    }
    System_printf("Debug: MMWDemoDSS create event handle succeeded\n");

    /************************************************************************
     * mmwave library initialization
     ************************************************************************/

    /* Populate the init configuration for mmwave library: */
    initCfg.domain                      = MMWave_Domain_DSS;
    initCfg.socHandle                   = gMmwDssMCB.socHandle;
    initCfg.eventFxn                    = MmwDemo_dssMmwaveEventCallbackFxn;
    initCfg.linkCRCCfg.useCRCDriver     = 1U;
    initCfg.linkCRCCfg.crcChannel       = CRC_Channel_CH1;
    initCfg.cfgMode                     = MMWave_ConfigurationMode_FULL;
    initCfg.executionMode               = MMWave_ExecutionMode_COOPERATIVE;
    initCfg.cooperativeModeCfg.cfgFxn   = MmwDemo_dssMmwaveConfigCallbackFxn;
    initCfg.cooperativeModeCfg.startFxn = MmwDemo_dssMmwaveStartCallbackFxn;
    initCfg.cooperativeModeCfg.stopFxn  = MmwDemo_dssMmwaveStopCallbackFxn;
    initCfg.cooperativeModeCfg.openFxn  = MmwDemo_dssMmwaveOpenCallbackFxn;
    initCfg.cooperativeModeCfg.closeFxn = MmwDemo_dssMmwaveCloseCallbackFxn;

    /* Initialize and setup the mmWave Control module */
    gMmwDssMCB.ctrlHandle = MMWave_init (&initCfg, &errCode);
    if (gMmwDssMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        System_printf ("Error: MMWDemoDSS mmWave Control Initialization failed [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: MMWDemoDSS mmWave Control Initialization succeeded\n");

    /******************************************************************************
     * TEST: Synchronization
     * - The synchronization API always needs to be invoked.
     ******************************************************************************/
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = MMWave_sync (gMmwDssMCB.ctrlHandle , &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the mmWave control module */
            System_printf ("Error: MMWDemoDSS mmWave Control Synchronization failed [Error code %d]\n", errCode);
            return;
        }
        if (syncStatus == 1)
        {
            /* Synchronization acheived: */
            break;
        }
        /* Sleep and poll again: */
        Task_sleep(1);
    }

    /*****************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priroity than any other task which uses the
     *   mmWave control API
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority = 6;
    taskParams.stackSize = 3300;
    Task_create(MmwDemo_dssMMWaveCtrlTask, &taskParams, NULL);

    /*****************************************************************************
     * Data path Startup
     *****************************************************************************/
    if ((errCode = MmwDemo_dssDataPathInit()) < 0 )
    {
        System_printf ("Error: MMWDemoDSS Data Path init failed with Error[%d]\n",errCode);
        return;
    }
    System_printf ("Debug: MMWDemoDSS Data Path init succeeded\n");
    gMmwDssMCB.state = MmwDemo_DSS_STATE_INIT;

    /* Start data path task */
    Task_Params_init(&taskParams);
    taskParams.priority = 5;
    taskParams.stackSize = 3*1024;
    Task_create(MmwDemo_dssDataPathTask, &taskParams, NULL);

    System_printf("Debug: MMWDemoDSS initTask exit\n");
    return;
}

/**
 *  @b Description
 *  @n
 *      Entry point into the test code.
 *
 *  @retval
 *      Not Applicable.
 */
int main (void)
{
    Task_Params    taskParams;
    SOC_Cfg        socCfg;
    int32_t        errCode;

    /* Initialize and populate the demo MCB */
    memset ((void*)&gMmwDssMCB, 0, sizeof(MmwDemo_DSS_MCB));

    /* Initialize the SOC confiugration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_BYPASS_INIT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    gMmwDssMCB.socHandle = SOC_init (&socCfg, &errCode);
    if (gMmwDssMCB.socHandle == NULL)
    {
        System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the DEMO configuration: */
    gMmwDssMCB.cfg.sysClockFrequency = DSS_SYS_VCLK;
    gMmwDssMCB.cfg.loggingBaudRate   = 921600;

    Cycleprofiler_init();

    /*current workaround MING MING MING*/
	{
		cache_setL2Size(CACHE_0KCACHE);
		cache_setMar((unsigned int *)0x20000000, 0xa0000, Cache_PC | Cache_PFX);
		cache_setMar((unsigned int *)0x21080000, 0x8000, Cache_PC | Cache_PFX);
		startClock();
	}


    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 2*1024;
    Task_create(MmwDemo_dssInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}

