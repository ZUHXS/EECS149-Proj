/**
 *   @file  gtrackModuleStep.c
 *
 *   @brief
 *      Module level step function for the GTRACK Algorithm
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2017 Texas Instruments, Inc.
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

#include <math.h>

#include <ti/alg/gtrack/gtrack.h>
#include <ti/alg/gtrack/include/gtrack_int.h>

void gtrack_modulePredict(GtrackModuleInstance *inst);
void gtrack_moduleAssociate(GtrackModuleInstance *inst, GTRACK_measurementPoint *point, uint16_t num);
void gtrack_moduleAllocate(GtrackModuleInstance *inst, GTRACK_measurementPoint *point, uint16_t num);
void gtrack_moduleUpdate(GtrackModuleInstance *inst, GTRACK_measurementPoint *point, GTRACK_measurementVariance *var, uint16_t num);
void gtrack_moduleReport(GtrackModuleInstance *inst, GTRACK_targetDesc *t, uint16_t *tNum);

/**
*  @b Description
*  @n
*      This is a MODULE level step function. Application shall call this function to process one frame of measurements
*
*  @param[in]  handle
*      Handle to GTRACK module
*  @param[in]  point
*      Pointer to an array of input measurments. Each measurement has range/angle/radial velocity information
*  @param[in]  vid
*      Pointer to an array of target identifies. Used for testing purposes only to correlate a measurment point with ground truth
*  @param[in]  mNum
*      Number of input measurements
*  @param[out]  t
*      Pointer to an array of target descriptors. This function populates the descritions for each of the tracked target 
*  @param[out]  tNum
*      Number of populated target descriptos 
*
*  \ingroup GTRACK_ALG_EXTERNAL_FUNCTION
*
*  @retval
*      None
*/


void gtrack_moduleStep(void *handle, GTRACK_measurementPoint *point, GTRACK_measurementVariance *var, uint16_t mNum, GTRACK_targetDesc *t, uint16_t *tNum, uint8_t *mIndex, uint32_t *bench)
{
    GtrackModuleInstance *inst;
	uint16_t n;
	float xPos;

    inst = (GtrackModuleInstance *)handle;
	
	inst->heartBeat++;
	if(inst->verbose & VERBOSE_WARNING_INFO)
		gtrack_log(GTRACK_VERBOSE_DEBUG, "Frame #%llu, %u Target(s), %hu Measurements\n", inst->heartBeat, gtrack_listGetCount(&inst->activeList), mNum);

	if(mNum > inst->maxNumPoints)
		mNum = inst->maxNumPoints;

	for(n=0; n< mNum; n++) {
        inst->bestScore[n] = 100;

	    xPos =  point[n].range*sinf(point[n].angle);
        if((xPos > inst->params.sceneryParams.leftWall) && (xPos < inst->params.sceneryParams.rightWall)) {
            inst->bestIndex[n] = POINT_NOT_ASSOCIATED;
        }
        else {
            // Remove Ghosts behind the walls
		    inst->bestIndex[n] = POINT_BEHIND_THE_WALL;
        }
	}

    bench[0] = gtrack_getCycleCount();
	gtrack_modulePredict(inst);
	bench[1] = gtrack_getCycleCount();
	gtrack_moduleAssociate(inst, point, mNum);
    bench[2] = gtrack_getCycleCount();
	gtrack_moduleAllocate(inst, point, mNum);
    bench[3] = gtrack_getCycleCount();
	gtrack_moduleUpdate(inst, point, var, mNum);
    bench[4] = gtrack_getCycleCount();
	gtrack_moduleReport(inst, t, tNum);
    bench[5] = gtrack_getCycleCount();

	// If requested, report tids associated with measurment vector 
	if(mIndex != 0) {
		for(n=0; n< mNum; n++) {
			mIndex[n] = inst->bestIndex[n];
		}
	}
}


void gtrack_modulePredict(GtrackModuleInstance *inst)
{
	GTrack_ListElem *tElem;
	uint16_t tid;

	tElem = gtrack_listGetFirst(&inst->activeList);
	while(tElem != 0)
	{
		tid = tElem->data;
		if(tid > 99)
			while(1) {};

		gtrack_unitPredict(inst->hTrack[tid]);

		tElem = gtrack_listGetNext(tElem);
	}
}

void gtrack_moduleAssociate(GtrackModuleInstance *inst, GTRACK_measurementPoint *point, uint16_t num)
{
	GTrack_ListElem *tElem;
	uint16_t tid;

	tElem = gtrack_listGetFirst(&inst->activeList);
	while(tElem != 0)
	{
		tid = tElem->data;
		gtrack_unitScore(inst->hTrack[tid], point, inst->bestScore, inst->bestIndex, num);

		tElem = gtrack_listGetNext(tElem);
	}

}

void gtrack_moduleAllocate(GtrackModuleInstance *inst, GTRACK_measurementPoint *point, uint16_t num)
{
	uint16_t n, k;

	float un[3], uk[3];
	float unSum[3];
	uint16_t allocNum;
	float dist;
	float allocSNR;
	GTrack_ListElem *tElem;


	for(n=0; n<num; n++) {
		if(inst->bestIndex[n] == POINT_NOT_ASSOCIATED) {
			
			tElem = gtrack_listGetFirst(&inst->freeList);
			if(tElem == 0) {
			    if(inst->verbose & VERBOSE_WARNING_INFO)
			        gtrack_log(GTRACK_VERBOSE_WARNING, "Maximum number of tracks reached!");
			    return;
			}

			inst->allocIndex[0] = n;
			allocNum = 1;
			allocSNR = point[n].snr;

			un[0] = unSum[0] = point[n].range;
			un[1] = unSum[1] = point[n].angle; 
			un[2] = unSum[2] = point[n].doppler;

			for(k=n+1; k<num; k++) {
				if(inst->bestIndex[k] == POINT_NOT_ASSOCIATED) {

					uk[0] = point[k].range;
					uk[1] = point[k].angle;
					uk[2] = gtrack_unrollRadialVelocity(inst->params.maxURadialVelocity, un[2], point[k].doppler);

					if(fabs(uk[2] - un[2]) < inst->params.allocationParams.maxVelThre) {
						dist = un[0]*un[0] + uk[0]*uk[0] - 2*un[0]*uk[0]*cosf(un[1]-uk[1]);
						if(dist < inst->params.allocationParams.maxDistanceThre) {
								
							inst->allocIndex[allocNum] = k;

							unSum[0] += uk[0];
							unSum[1] += uk[1];
							unSum[2] += uk[2];

							allocNum++;
							allocSNR +=point[k].snr;

							un[0] = unSum[0]/allocNum;
							un[1] = unSum[1]/allocNum;
							un[2] = unSum[2]/allocNum;
						}
					}
				}
			}
			if((allocNum > inst->params.allocationParams.pointsThre) &&
			        (allocSNR > inst->params.allocationParams.snrThre) &&
			        (fabs(un[2]) > inst->params.allocationParams.velocityThre) )
			{
				// Associate points with new TID 
				for(k=0; k<allocNum; k++)
					inst->bestIndex[inst->allocIndex[k]] = (uint8_t)tElem->data;

				// Allocate new tracker
				tElem = gtrack_listDequeue(&inst->freeList);
				gtrack_unitStart(inst->hTrack[tElem->data], inst->heartBeat, un);
				gtrack_listEnqueue(&inst->activeList, tElem);
			}
		}
	}
}

void gtrack_moduleUpdate(GtrackModuleInstance *inst, GTRACK_measurementPoint *point, GTRACK_measurementVariance *var, uint16_t num)
{
	GTrack_ListElem *tElem;
	GTrack_ListElem *tElemToRemove;
	uint16_t tid;
	TrackState state;

	tElem = gtrack_listGetFirst(&inst->activeList);
	while(tElem != 0)
	{
		tid = tElem->data;
		state = gtrack_unitUpdate(inst->hTrack[tid], point, var, inst->bestIndex, num);
		if(state == TRACK_STATE_FREE) {
			tElemToRemove = tElem;
			tElem = gtrack_listGetNext(tElem);
			gtrack_listRemoveElement(&inst->activeList, tElemToRemove);

			gtrack_unitStop(inst->hTrack[tElemToRemove->data]);

			gtrack_listEnqueue(&inst->freeList, tElemToRemove);
		}
		else
			tElem = gtrack_listGetNext(tElem);
	}
}

void gtrack_moduleReport(GtrackModuleInstance *inst, GTRACK_targetDesc *t, uint16_t *tNum)
{
	GTrack_ListElem *tElem;
	uint16_t tid;
	uint16_t num = 0;


	tElem = gtrack_listGetFirst(&inst->activeList);
	while(tElem != 0)
	{
		tid = tElem->data;
		gtrack_unitReport(inst->hTrack[tid], &t[num++]);
		tElem = gtrack_listGetNext(tElem);
	}
	*tNum = num;
}
