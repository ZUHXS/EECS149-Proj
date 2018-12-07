/**
 *   @file  task_mbox.c
 *
 *   @brief
 *     MSS main implementation of the millimeter wave Demo
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
#include <ti/sysbios/family/arm/v7a/Pmu.h>
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/utils/cli/cli.h>
#include <ti/drivers/osal/MemoryP.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/osal/HwiP.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

#include <gtrack.h>

/* Demo Include Files */
#include "mss_mmw.h"
#include <chains/RadarReceiverPeopleCounting/mmw_PCDemo/common/mmw_messages.h>
#include <chains/RadarReceiverPeopleCounting/mmw_PCDemo/common/mmw_output.h>

extern int32_t MmwDemo_mboxWrite(MmwDemo_message *message);
extern void MmwDemo_printHeapStats(void);

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

typedef enum {
    TRACKING_DEFAULT_PARAM_SET = 0,
    TRACKING_TRAFFIC_MONITORING_PARAM_SET,
    TRACKING_PEOPLE_COUNTING_PARAM_SET
} TRACKING_ADVANCED_PARAM_SET;

typedef enum {
    TRACKING_PARAM_SET_TM = 0,
    TRACKING_PARAM_SET_PC
} TRACKING_ADVANCED_PARAM_SET_TABLE;

//// This test application (traffic monitoring), wants to modify default parameters
//SceneryParams appSceneryParamTable[2] = {
//     {-100, 100, 0, 100},   // TM: No walls, Bottom exit Zone (10m), no Upper Exit zone
////     {-2.9, 1.9, 1, 4.5}      // PC HALL Left Wall (-1.5), Right Wall (1.5), Bottom Exit Zone (1m), Upper Exit Zone (5m)
////     {-0.25, 3.65, 1, 4.3}      // PC OFFICE Left Wall (-1.5), Right Wall (1.5), Bottom Exit Zone (1m), Upper Exit Zone (4.5m)
//     {-6, 6, 0.05, 4}      // PC OFFICE Left Wall (-1.5), Right Wall (1.5), Bottom Exit Zone (1m), Upper Exit Zone (4.5m)
//};
//GatingParams appGatingParamTable[2] = {
//     {16, {12, 8, 0}},    // TM: 16 gating volume, Limits are set to 8m in length, 2m in width, 0 no limit in doppler
//     {4,  {3, 2, 0}}     // PC: 4 gating volume, Limits are set to 3m in length, 2m in width, 0 no limit in doppler
//};
//StateParams appStateParamTable[2] = {
//     {3, 3, 5, 5, 5},              // TM: 3 frames to activate, 3 to forget, 5 to delete
//     {10, 5, 10, 100, 5}          // PC: 10 frames to activate, 5 to forget, 10 active to free, 1000 static to free, 5 exit to free
//};
//AllocationParams appAllocationParamTable[2] = {
//     {100, 0.5, 3, 4, 2},          // TM: 3 points, 4m in distance, 2m/c in velocity
//     {250, 0.01, 5, 1, 2}           // PC: 250 SNR, 0.1 minimal velocity, 5 points, 1m in distance, 2m/c in velocity
//};
//// Using standard deviation of uniformly distributed variable in the range [a b]: 1/sqrt(12)*(b-a)
//VarParams appVariationParamTable[2] = {
//     // Standard deviation of uniformly distributed number in range [a b]: sqrt(1/12)*(b-a)
//     {1.f/3.46, 1.f/3.46, 1.f},     // TM: 1m height, 1m in width, 2 m/s for doppler
//     {1.f/3.46, 1.f/3.46, 1.f}      // PC: 1m height, 1m in width, 1 m/s for doppler
//};

SceneryParams appSceneryParamTable[2] = {
     {-100, 100, 0, 100},   // TM: No walls, Bottom exit Zone (10m), no Upper Exit zone
//     {-2.9, 1.9, 1, 4.5}      // PC HALL Left Wall (-1.5), Right Wall (1.5), Bottom Exit Zone (1m), Upper Exit Zone (5m)
//     {-0.25, 3.65, 1, 4.3}      // PC OFFICE Left Wall (-1.5), Right Wall (1.5), Bottom Exit Zone (1m), Upper Exit Zone (4.5m)
     {-15, 15, 0.05, 20}      // PC OFFICE Left Wall (-1.5), Right Wall (1.5), Bottom Exit Zone (1m), Upper Exit Zone (4.5m)
};
GatingParams appGatingParamTable[2] = {
     {16, {12, 8, 0}},    // TM: 16 gating volume, Limits are set to 8m in length, 2m in width, 0 no limit in doppler
     {4,  {3, 2, 0}}     // PC: 4 gating volume, Limits are set to 3m in length, 2m in width, 0 no limit in doppler
};
StateParams appStateParamTable[2] = {
     {3, 3, 5, 5, 5},              // TM: 3 frames to activate, 3 to forget, 5 to delete
     {5, 5, 10, 100, 5}          // PC: 10 frames to activate, 5 to forget, 10 active to free, 1000 static to free, 5 exit to free
};
AllocationParams appAllocationParamTable[2] = {
     {100, 0.5, 3, 4, 2},          // TM: 3 points, 4m in distance, 2m/c in velocity
     {50, 0.01, 5, 0.5, 2}           // PC: 250 SNR, 0.1 minimal velocity, 5 points, 1m in distance, 2m/s in velocity
};
// Using standard deviation of uniformly distributed variable in the range [a b]: 1/sqrt(12)*(b-a)
VarParams appVariationParamTable[2] = {
     // Standard deviation of uniformly distributed number in range [a b]: sqrt(1/12)*(b-a)
     {1.f/3.46, 1.f/3.46, 1.f},     // TM: 1m height, 1m in width, 2 m/s for doppler
     {1.f/3.46, 1.f/3.46, 1.f}      // PC: 1m height, 1m in width, 1 m/s for doppler
};



/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
extern MmwDemo_MCB    gMmwMssMCB;


/**
 *  @b Description
 *  @n
 *      The Task is used to handle  the mmw demo messages received from
 *      Mailbox virtual channel.
 *
 *  @param[in]  arg0
 *      arg0 of the Task. Not used
 *  @param[in]  arg1
 *      arg1 of the Task. Not used
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_appTask(UArg arg0, UArg arg1)
{
    MmwDemo_ApplicationCfg appConfig;
    MmwDemo_output_message_targetList *targetList;
    MmwDemo_output_message_targetIndex *targetIndex;
    GTRACK_targetDesc targetDescr[20];


	GTRACK_measurementPoint *points;
    GTRACK_measurementVariance *variances;
    uint32_t timeStart;
    uint32_t    *benchmarks;
    uint16_t    mNum;
    uint16_t    tNum;
    uint16_t    n;
    _Bool currentDescr;

    memset ((void *)&appConfig, 0, sizeof(MmwDemo_ApplicationCfg));
    appConfig.leftWallPos = -3.1f; // Hard code for now
    appConfig.rightWallPos = 1.7f;
    appConfig.sensorAzimuthTilt = 0;
    /* Save Configuration to use later */
    memcpy((void *)&gMmwMssMCB.cfg.applicationCfg, (void *)&appConfig, sizeof(MmwDemo_ApplicationCfg));

	benchmarks = gMmwMssMCB.mssDataPathObj.cycleLog.benchmarks;
    /* wait for new message and process all the messages received from the peer */
    while(1)
    {
        Semaphore_pend(gMmwMssMCB.appSemHandle, BIOS_WAIT_FOREVER);

        timeStart = Cycleprofiler_getTimeStamp();

        if(gMmwMssMCB.pointCloud->header.length > sizeof(MmwDemo_output_message_tl))
            mNum = (gMmwMssMCB.pointCloud->header.length-sizeof(MmwDemo_output_message_tl))/sizeof(MmwDemo_output_message_point);
        else
            mNum = 0;

        currentDescr = gMmwMssMCB.targetDescrHandle->currentDescr;
        targetList = gMmwMssMCB.targetDescrHandle->tList[currentDescr];
        targetIndex = gMmwMssMCB.targetDescrHandle->tIndex[currentDescr];
        points = (GTRACK_measurementPoint *)gMmwMssMCB.pointCloud->point;
        variances = NULL;

        // Execute tracking
        gtrack_moduleStep(gMmwMssMCB.gtrackHandle, points, variances, mNum, targetDescr, &tNum, targetIndex->index, benchmarks);

        for(n=0; n<tNum; n++) {
            targetList->target[n].tid  = (uint32_t)targetDescr[n].tid;

            targetList->target[n].posX = targetDescr[n].S[0];
            targetList->target[n].posY = targetDescr[n].S[1];
            targetList->target[n].velX = targetDescr[n].S[2];
            targetList->target[n].velY = targetDescr[n].S[3];
            targetList->target[n].accX = targetDescr[n].S[4];
            targetList->target[n].accY = targetDescr[n].S[5];

            memcpy(targetList->target[n].ec, targetDescr[n].EC, sizeof(targetDescr[n].EC));

            targetList->target[n].g = targetDescr[n].G;
        }

        if(tNum > 0)
            targetList->header.length = sizeof(MmwDemo_output_message_tl) + tNum*sizeof(MmwDemo_output_message_target);
        else
            targetList->header.length = 0;

        if((mNum > 0) && (tNum > 0))
            /* Target Indices exist only when we have both points AND targets */
            targetIndex->header.length = sizeof(MmwDemo_output_message_tl) + mNum*sizeof(uint8_t);
        else
            targetIndex->header.length = 0;

        gMmwMssMCB.mssDataPathObj.cycleLog.trackingTimeCurrInusec = ((float)(Cycleprofiler_getTimeStamp() - timeStart))/(float)R4F_CLOCK_MHZ;
        if ((gMmwMssMCB.mssDataPathObj.cycleLog.trackingTimeCurrInusec > 0) && (gMmwMssMCB.mssDataPathObj.cycleLog.trackingTimeCurrInusec > gMmwMssMCB.mssDataPathObj.cycleLog.trackingTimeMaxInusec))
            gMmwMssMCB.mssDataPathObj.cycleLog.trackingTimeMaxInusec = gMmwMssMCB.mssDataPathObj.cycleLog.trackingTimeCurrInusec;

	}
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for tracking configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_CLITrackingCfg (int32_t argc, char* argv[])
{
    GTRACK_moduleConfig         config;
    GTRACK_advancedParameters   advParams;

    TRACKING_ADVANCED_PARAM_SET trackingParamSet;

    MmwDemo_message         message;
    uint32_t                pointCloudSize;
    uint32_t                targetListSize;
    uint32_t                targetIndexSize;

    //Memory_Stats            startMemoryStats;
    //Memory_Stats            endMemoryStats;

    if (argc >= 1) {
        gMmwMssMCB.mssDataPathObj.groupTrackerEnabled = (uint16_t) atoi (argv[1]);
    }

    if(gMmwMssMCB.mssDataPathObj.groupTrackerEnabled != 1) {
        return 0;
    }

    /* Sanity Check: Minimum argument check */
    if (argc != 8)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    System_printf("Debug: Heap before creating a tracker\n");
    MmwDemo_printHeapStats();

    /* Initialize CLI configuration: */
    memset ((void *)&config, 0, sizeof(GTRACK_moduleConfig));

    trackingParamSet            = (TRACKING_ADVANCED_PARAM_SET) atoi (argv[2]);
    switch(trackingParamSet)
    {
        case TRACKING_DEFAULT_PARAM_SET:
            // Do not configure advanced parameters, use library default parameters
            config.advParams = 0;
            break;

        case TRACKING_TRAFFIC_MONITORING_PARAM_SET:
            /* Initialize CLI configuration: */
            memset ((void *)&advParams, 0, sizeof(GTRACK_advancedParameters));
            advParams.sceneryParams = &appSceneryParamTable[TRACKING_PARAM_SET_TM];
            advParams.allocationParams = &appAllocationParamTable[TRACKING_PARAM_SET_TM];
            advParams.gatingParams = &appGatingParamTable[TRACKING_PARAM_SET_TM];
            advParams.stateParams = &appStateParamTable[TRACKING_PARAM_SET_TM];
            advParams.variationParams = &appVariationParamTable[TRACKING_PARAM_SET_TM];

            config.advParams = &advParams;
            config.initialRadialVelocity = -20; // for TM, detected targets are approaching
            config.maxAcceleration = 20; // for TM, maximum acceleration is set to 20m/s2
            break;

        case TRACKING_PEOPLE_COUNTING_PARAM_SET:

            /* Initialize CLI configuration: */
            memset ((void *)&advParams, 0, sizeof(GTRACK_advancedParameters));
            advParams.sceneryParams = &appSceneryParamTable[TRACKING_PARAM_SET_PC];
            advParams.allocationParams = &appAllocationParamTable[TRACKING_PARAM_SET_PC];
            advParams.gatingParams = &appGatingParamTable[TRACKING_PARAM_SET_PC];
            advParams.stateParams = &appStateParamTable[TRACKING_PARAM_SET_PC];
            advParams.variationParams = &appVariationParamTable[TRACKING_PARAM_SET_PC];

            config.advParams = &advParams;
            config.initialRadialVelocity = 0; //For PC, detected target velocity is unknown
            config.maxAcceleration = 5; // for PC, maximum acceleration is set to 5m/s2
            break;

        default:
            CLI_write ("Error: Invalid usage of the CLI command\n");
            return -1;
    }

    config.stateVectorType      = GTRACK_STATE_VECTORS_2DA; // Track two dimensions with acceleration
    config.verbose              = GTRACK_VERBOSE_NONE;
    config.maxNumPoints         = (uint16_t) atoi(argv[3]);
    config.maxNumTracks         = (uint16_t) atoi(argv[4]);
    config.maxURadialVelocity   = (float) atoi(argv[5]) *0.1;
    config.deltaT               = (float) atoi(argv[6]) *0.001f;

    gMmwMssMCB.cfg.applicationCfg.sensorAzimuthTilt = (90-atoi(argv[7]))*3.14f/180;

    if(gMmwMssMCB.cfg.trackingCfg.config.maxNumPoints != 0) {
        pointCloudSize = sizeof(MmwDemo_output_message_tl) + gMmwMssMCB.cfg.trackingCfg.config.maxNumPoints*sizeof(GTRACK_measurementPoint);
        if(gMmwMssMCB.pointCloud != NULL) {
            MemoryP_ctrlFree(gMmwMssMCB.pointCloud, pointCloudSize);
        }
    }
    if(gMmwMssMCB.cfg.trackingCfg.config.maxNumTracks != 0) {
        targetListSize = sizeof(MmwDemo_output_message_tl) + gMmwMssMCB.cfg.trackingCfg.config.maxNumTracks*sizeof(GTRACK_targetDesc);
        targetIndexSize = sizeof(MmwDemo_output_message_tl) + gMmwMssMCB.cfg.trackingCfg.config.maxNumPoints*sizeof(uint8_t);
        if(gMmwMssMCB.targetDescrHandle != NULL) {
            /* Free Target List Arrays */
            if(gMmwMssMCB.targetDescrHandle->tList[0] != NULL)
                MemoryP_ctrlFree(gMmwMssMCB.targetDescrHandle->tList[0], targetListSize);
            if(gMmwMssMCB.targetDescrHandle->tList[1] != NULL)
                MemoryP_ctrlFree(gMmwMssMCB.targetDescrHandle->tList[1], targetListSize);

            /* Free Target Index Arrays */
            if(gMmwMssMCB.targetDescrHandle->tIndex[0] != NULL)
                MemoryP_ctrlFree(gMmwMssMCB.targetDescrHandle->tIndex[0], targetIndexSize);
            if(gMmwMssMCB.targetDescrHandle->tIndex[1] != NULL)
                MemoryP_ctrlFree(gMmwMssMCB.targetDescrHandle->tIndex[1], targetIndexSize);
        }
    }
    if(gMmwMssMCB.targetDescrHandle != NULL) {
        MemoryP_ctrlFree(gMmwMssMCB.targetDescrHandle, sizeof(MmwDemo_targetDescrHandle));
    }

    /* Save Configuration to use later */
    memcpy((void *)&gMmwMssMCB.cfg.trackingCfg.config, (void *)&config, sizeof(GTRACK_moduleConfig));
    memcpy((void *)&gMmwMssMCB.cfg.trackingCfg.params, (void *)&advParams, sizeof(GTRACK_advancedParameters));

    //HeapMem_getStats (heap0, &startMemoryStats);
    //MmwDemo_printHeapStats();

    /* Allocate memory for Point Cloud TLV */
    pointCloudSize = sizeof(MmwDemo_output_message_tl) + config.maxNumPoints*sizeof(GTRACK_measurementPoint);
    gMmwMssMCB.pointCloud = (MmwDemo_output_message_pointCloud *)MemoryP_ctrlAlloc(pointCloudSize, sizeof(float));
    gMmwMssMCB.pointCloud->header.type = MMWDEMO_OUTPUT_MSG_POINT_CLOUD;

    if(gMmwMssMCB.pointCloud == NULL) {
        System_printf("Error: Unable to allocate %d bytes for pointCloud\n", config.maxNumPoints*sizeof(GTRACK_measurementPoint));
        DebugP_assert(0);
    }

    /* Allocate memory for Target Descriptor handle */
    gMmwMssMCB.targetDescrHandle = (MmwDemo_targetDescrHandle *)MemoryP_ctrlAlloc(sizeof(MmwDemo_targetDescrHandle), sizeof(float));
    if(gMmwMssMCB.targetDescrHandle == NULL) {
        System_printf("Error: Unable to allocate %d bytes for targetDescr handle\n", sizeof(MmwDemo_targetDescrHandle));
        DebugP_assert(0);
    }
    memset ((void *)gMmwMssMCB.targetDescrHandle, 0, sizeof(MmwDemo_targetDescrHandle));

    /* Allocate memory for ping/pong target lists */
    targetListSize = sizeof(MmwDemo_output_message_tl) + config.maxNumTracks*sizeof(GTRACK_targetDesc);
    gMmwMssMCB.targetDescrHandle->tList[0] = (MmwDemo_output_message_targetList *)MemoryP_ctrlAlloc(targetListSize, sizeof(float));
    gMmwMssMCB.targetDescrHandle->tList[1] = (MmwDemo_output_message_targetList *)MemoryP_ctrlAlloc(targetListSize, sizeof(float));

    if((gMmwMssMCB.targetDescrHandle->tList[0] == NULL) || (gMmwMssMCB.targetDescrHandle->tList[1] == NULL)){
        System_printf("Error: Unable to allocate %d bytes for targetLists\n", targetListSize*2);
        DebugP_assert(0);
    }

    gMmwMssMCB.targetDescrHandle->tList[0]->header.type = MMWDEMO_OUTPUT_MSG_TARGET_LIST;
    gMmwMssMCB.targetDescrHandle->tList[0]->header.length = 0;
    gMmwMssMCB.targetDescrHandle->tList[1]->header.type = MMWDEMO_OUTPUT_MSG_TARGET_LIST;
    gMmwMssMCB.targetDescrHandle->tList[1]->header.length = 0;

    /* Allocate memory for ping/pong target indices */
    targetIndexSize = sizeof(MmwDemo_output_message_tl) + config.maxNumPoints*sizeof(uint8_t);
    gMmwMssMCB.targetDescrHandle->tIndex[0] = (MmwDemo_output_message_targetIndex *)MemoryP_ctrlAlloc(targetIndexSize, sizeof(float));
    gMmwMssMCB.targetDescrHandle->tIndex[1] = (MmwDemo_output_message_targetIndex *)MemoryP_ctrlAlloc(targetIndexSize, sizeof(float));

    if((gMmwMssMCB.targetDescrHandle->tIndex[0] == NULL) || (gMmwMssMCB.targetDescrHandle->tIndex[1] == NULL)){
        System_printf("Error: Unable to allocate %d bytes for targetIndices\n", targetIndexSize*2);
        DebugP_assert(0);
    }

    gMmwMssMCB.targetDescrHandle->tIndex[0]->header.type = MMWDEMO_OUTPUT_MSG_TARGET_INDEX;
    gMmwMssMCB.targetDescrHandle->tIndex[0]->header.length = 0;
    gMmwMssMCB.targetDescrHandle->tIndex[1]->header.type = MMWDEMO_OUTPUT_MSG_TARGET_INDEX;
    gMmwMssMCB.targetDescrHandle->tIndex[1]->header.length = 0;

    //MmwDemo_printHeapStats();

    /* Create a Tracker */
    if(gMmwMssMCB.gtrackHandle != NULL)
        gtrack_moduleDelete(gMmwMssMCB.gtrackHandle);

    gMmwMssMCB.gtrackHandle = gtrack_moduleCreate(&config);
    if(gMmwMssMCB.gtrackHandle == NULL) {
        System_printf("Error: Unable to allocate memory for Tracker\n");
        DebugP_assert(0);
    }
    System_printf("Debug: (GtrackModuleInstance *)0x%x\n", (uint32_t)gMmwMssMCB.gtrackHandle);
    MmwDemo_printHeapStats();

    /* Get the heap statistics at the beginning of the tests */
    //HeapMem_getStats (heap0, &endMemoryStats);
    // System_printf ("Debug: Tracker %d used bytes from System Heap\n", startMemoryStats.totalFreeSize - endMemoryStats.totalFreeSize);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_TRACKING_CFG;
    memcpy((void *)&message.body.tracking, (void *)&config, sizeof(GTRACK_moduleConfig));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for SceneryParam configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_CLISceneryParamCfg (int32_t argc, char* argv[])
{


    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
	
	/* Initialize the ADC Output configuration: */
	memset ((void *)&appSceneryParamTable, 0, sizeof(appSceneryParamTable));

	
	/* Populate configuration: */
	appSceneryParamTable[1].leftWall = (float) atof (argv[1]);
	appSceneryParamTable[1].rightWall = (float) atof (argv[2]);
	appSceneryParamTable[1].lowerEntrance = (float) atof (argv[3]);
	appSceneryParamTable[1].upperEntrance = (float) atof (argv[4]);

	appSceneryParamTable[0].leftWall = (float) atof (argv[1]);
	appSceneryParamTable[0].rightWall = (float) atof (argv[2]);
	appSceneryParamTable[0].lowerEntrance = (float) atof (argv[3]);
	appSceneryParamTable[0].upperEntrance = (float) atof (argv[4]);

//for next version algorithm	
//	appSceneryParamTable[1].numBoundaryBoxes= (uint16_t) atoi (argv[1]);
//	appSceneryParamTable[1].boundaryBox[1].left = (uint16_t) atoi (argv[3]);
//	appSceneryParamTable[1].boundaryBox[1].right = (uint16_t) atoi (argv[4]);
//	appSceneryParamTable[1].boundaryBox[1].bottom = (uint16_t) atoi (argv[5]);
//	appSceneryParamTable[1].boundaryBox[1].top= (uint16_t) atoi (argv[6]);
//	appSceneryParamTable[1].boundaryBox[2].left = (uint16_t) atoi (argv[8]);
//	appSceneryParamTable[1].boundaryBox[2].right = (uint16_t) atoi (argv[9]);
//	appSceneryParamTable[1].boundaryBox[2].bottom = (uint16_t) atoi (argv[10]);
//	appSceneryParamTable[1].boundaryBox[2].top= (uint16_t) atoi (argv[11]);
//
//	appSceneryParamTable[1].numStaticBoxes= (uint16_t) atoi (argv[13]);
//	appSceneryParamTable[1].staticBox[1].left= (uint16_t) atoi (argv[15]);
//	appSceneryParamTable[1].staticBox[1].right = (uint16_t) atoi (argv[16]);
//	appSceneryParamTable[1].staticBox[1].bottom = (uint16_t) atoi (argv[17]);
//	appSceneryParamTable[1].staticBox[1].top= (uint16_t) atoi (argv[18]);
//	appSceneryParamTable[1].staticBox[2].left = (uint16_t) atoi (argv[20]);
//	appSceneryParamTable[1].staticBox[2].right = (uint16_t) atoi (argv[21]);
//	appSceneryParamTable[1].staticBox[2].bottom = (uint16_t) atoi (argv[22]);
//	appSceneryParamTable[1].staticBox[2].top= (uint16_t) atoi (argv[23]);
//
//	appSceneryParamTable[0].numBoundaryBoxes= (uint16_t) atoi (argv[1]);
//	appSceneryParamTable[0].boundaryBox[1].left = (uint16_t) atoi (argv[3]);
//	appSceneryParamTable[0].boundaryBox[1].right = (uint16_t) atoi (argv[4]);
//	appSceneryParamTable[0].boundaryBox[1].bottom = (uint16_t) atoi (argv[5]);
//	appSceneryParamTable[0].boundaryBox[1].top= (uint16_t) atoi (argv[6]);
//	appSceneryParamTable[0].boundaryBox[2].left = (uint16_t) atoi (argv[8]);
//	appSceneryParamTable[0].boundaryBox[2].right = (uint16_t) atoi (argv[9]);
//	appSceneryParamTable[0].boundaryBox[2].bottom = (uint16_t) atoi (argv[10]);
//	appSceneryParamTable[0].boundaryBox[2].top= (uint16_t) atoi (argv[11]);
//
//	appSceneryParamTable[0].numStaticBoxes= (uint16_t) atoi (argv[13]);
//	appSceneryParamTable[0].staticBox[1].left= (uint16_t) atoi (argv[15]);
//	appSceneryParamTable[0].staticBox[1].right = (uint16_t) atoi (argv[16]);
//	appSceneryParamTable[0].staticBox[1].bottom = (uint16_t) atoi (argv[17]);
//	appSceneryParamTable[0].staticBox[1].top= (uint16_t) atoi (argv[18]);
//	appSceneryParamTable[0].staticBox[2].left = (uint16_t) atoi (argv[20]);
//	appSceneryParamTable[0].staticBox[2].right = (uint16_t) atoi (argv[21]);
//	appSceneryParamTable[0].staticBox[2].bottom = (uint16_t) atoi (argv[22]);
//	appSceneryParamTable[0].staticBox[2].top= (uint16_t) atoi (argv[23]);


	return 0;
    
}



/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for GatingParam configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_CLIGatingParamCfg (int32_t argc, char* argv[])
{


    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
	/* Initialize the ADC Output configuration: */
	memset ((void *)&appGatingParamTable, 0, sizeof(appGatingParamTable));

	appGatingParamTable[0].volume = (float) atof (argv[1]);
	appGatingParamTable[0].limits.length = (float) atof (argv[2]);
	appGatingParamTable[0].limits.width = (float) atof (argv[3]);
	appGatingParamTable[0].limits.vel = (float) atof (argv[4]);
	
	appGatingParamTable[1].volume = (float) atof (argv[1]);
	appGatingParamTable[1].limits.length = (float) atof (argv[2]);
	appGatingParamTable[1].limits.width = (float) atof (argv[3]);
	appGatingParamTable[1].limits.vel = (float) atof (argv[4]);
	


	return 0;
    
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for StateParam configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_CLIStateParamCfg (int32_t argc, char* argv[])
{


    /* Sanity Check: Minimum argument check */
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }


    //System_printf("StateParam config\n");

	//"<det2act> <det2free> <act2free> <stat2free> <exit2free>";//det2act, det2free, act2free, stat2free, exit2free
   
    /* Populate configuration: */
	//"<det2act> <det2free> <act2free> <stat2free> <exit2free>";//det2act, det2free, act2free, stat2free, exit2free

//    CLI_write ("\r\n111\r\n");
//	System_printf("StateParamTable config 0: %d, %d, %d, %d, %d\n", appStateParamTable[0].det2actThre, appStateParamTable[0].det2freeThre, appStateParamTable[0].active2freeThre, appStateParamTable[0].static2freeThre, appStateParamTable[0].exit2freeThre);
//	System_printf("StateParamTable config 1: %d, %d, %d, %d, %d\n", appStateParamTable[1].det2actThre, appStateParamTable[1].det2freeThre, appStateParamTable[1].active2freeThre, appStateParamTable[1].static2freeThre, appStateParamTable[1].exit2freeThre);

	/* Initialize the ADC Output configuration: */
	memset ((void *)&appStateParamTable, 0, sizeof(appStateParamTable));

	
    /* Populate configuration: */
	appStateParamTable[1].det2actThre = (uint16_t) atoi (argv[1]);
    appStateParamTable[1].det2freeThre= (uint16_t) atoi (argv[2]);
    appStateParamTable[1].active2freeThre= (uint16_t) atoi (argv[3]);
    appStateParamTable[1].static2freeThre= (uint16_t) atoi (argv[4]);
    appStateParamTable[1].exit2freeThre= (uint16_t) atoi (argv[5]);

	appStateParamTable[0].det2actThre = (uint16_t) atoi (argv[1]);
    appStateParamTable[0].det2freeThre= (uint16_t) atoi (argv[2]);
    appStateParamTable[0].active2freeThre= (uint16_t) atoi (argv[3]);
    appStateParamTable[0].static2freeThre= (uint16_t) atoi (argv[4]);
    appStateParamTable[0].exit2freeThre= (uint16_t) atoi (argv[5]);    

//	System_printf("After\n");
//
//	System_printf("StateParamTable config 0: %d, %d, %d, %d, %d\n", appStateParamTable[0].det2actThre, appStateParamTable[0].det2freeThre, appStateParamTable[0].active2freeThre, appStateParamTable[0].static2freeThre, appStateParamTable[0].exit2freeThre);
//	System_printf("StateParamTable config 1: %d, %d, %d, %d, %d\n", appStateParamTable[1].det2actThre, appStateParamTable[1].det2freeThre, appStateParamTable[1].active2freeThre, appStateParamTable[1].static2freeThre, appStateParamTable[1].exit2freeThre);

//	CLI_write ("Done\r\n");

	
    //System_printf("CFAR config:method = %d\n", cfarCfg.cfarMethod);
    return 0;
    
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for AllocationParam configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_CLIAllocationParamCfg (int32_t argc, char* argv[])
{


    /* Sanity Check: Minimum argument check */
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

	/* Initialize the ADC Output configuration: */
	memset ((void *)&appAllocationParamTable, 0, sizeof(appAllocationParamTable));

	
    /* Populate configuration: */
	appAllocationParamTable[0].snrThre = (float) atof (argv[1]);
	appAllocationParamTable[0].velocityThre = (float) atof (argv[2]);
	appAllocationParamTable[0].pointsThre = (uint16_t) atoi (argv[3]);
	appAllocationParamTable[0].maxDistanceThre = (float) atof (argv[4]);
	appAllocationParamTable[0].maxVelThre = (float) atof (argv[5]);
	
	appAllocationParamTable[1].snrThre = (float) atof (argv[1]);
	appAllocationParamTable[1].velocityThre = (float) atof (argv[2]);
	appAllocationParamTable[1].pointsThre = (uint16_t) atoi (argv[3]);
	appAllocationParamTable[1].maxDistanceThre = (float) atof (argv[4]);
	appAllocationParamTable[1].maxVelThre = (float) atof (argv[5]);


	return 0;
    
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for VariationParam configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_CLIVariationParamCfg (int32_t argc, char* argv[])
{


    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

	/* Initialize the ADC Output configuration: */
	memset ((void *)&appVariationParamTable, 0, sizeof(appVariationParamTable));

	
    /* Populate configuration: */
	appVariationParamTable[0].lengthStd = (float) atof (argv[1]);
	appVariationParamTable[0].widthStd = (float) atof (argv[2]);
	appVariationParamTable[0].dopplerStd = (float) atof (argv[3]);

	appVariationParamTable[1].lengthStd = (float) atof (argv[1]);
	appVariationParamTable[1].widthStd = (float) atof (argv[2]);
	appVariationParamTable[1].dopplerStd = (float) atof (argv[3]);

	return 0;
    
}

int32_t MmwDemo_CLIPointCloudEn (int32_t argc, char* argv[]) {
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of PointCloudEn command\n");
        return -1;
    }
    gMmwMssMCB.pcEnable = (bool)atoi(argv[1]);
    return 0;
}
