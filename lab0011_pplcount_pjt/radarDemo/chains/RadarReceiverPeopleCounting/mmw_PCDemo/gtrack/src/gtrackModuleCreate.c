/**
 *   @file  gtrackModuleCreate.c
 *
 *   @brief
 *      Module level create function for the GTRACK Algorithm
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
#include <string.h>
#include <math.h>

#include <ti/alg/gtrack/gtrack.h>
#include <ti/alg/gtrack/include/gtrack_int.h>

extern const SceneryParams defaultSceneryParams;
extern const GatingParams defaultGatingParams;
extern const ScoringParams defaultScoringParams;
extern const StateParams defaultStateParams;
extern const AllocationParams defaultAllocationParams;
extern const UnrollingParams defaultUnrollingParams;
extern const VarParams defaultVariationParams;

/**
*  @b Description
*  @n
*		Application calls this function to instantiate GTRACK module desired configuration parameters. 
*		Function returns a handle, which shall be used to execute a single frame
*
*  @param[in]  config
*		This is a pointer to the configuration structure. The structure contains all parameters that are exposed by GTRACK alrorithm.
*		The configuration does not need to persist.
*
*  \ingroup GTRACK_ALG_EXTERNAL_FUNCTION
*
*  @retval
*      Handle to GTRACK module
*/

void *gtrack_moduleCreate(GTRACK_moduleConfig *config)
{
	GtrackModuleInstance *inst;
	uint16_t n;

	float dt, dt2, dt3, dt4;

    inst = (GtrackModuleInstance *)gtrack_alloc(1, sizeof(GtrackModuleInstance));

//	gtrack_log(GTRACK_VERBOSE_DEBUG, "Module Instance address is #%llx\n", inst);

	if(config->maxNumPoints > NUM_POINTS_MAX)
		return 0;

	if(config->maxNumTracks > NUM_TRACKS_MAX)
		return 0;

	inst->maxNumPoints = config->maxNumPoints;
	inst->maxNumTracks = config->maxNumTracks;

	inst->heartBeat = 0;

    // default parameters
	inst->params.gatingParams = defaultGatingParams;
    inst->params.scoringParams = defaultScoringParams;
    inst->params.stateParams = defaultStateParams;
    inst->params.unrollingParams = defaultUnrollingParams;
    inst->params.allocationParams = defaultAllocationParams;
	inst->params.variationParams = defaultVariationParams;
    inst->params.sceneryParams = defaultSceneryParams;

	if(config->advParams != NULL) {
		// user overwrites default parameters
	    if(config->advParams->gatingParams)
			memcpy(&inst->params.gatingParams, config->advParams->gatingParams, sizeof(GatingParams));
	    if(config->advParams->scoringParams)
			memcpy(&inst->params.scoringParams, config->advParams->scoringParams, sizeof(ScoringParams));
		if(config->advParams->stateParams)
			memcpy(&inst->params.stateParams, config->advParams->stateParams, sizeof(StateParams));
		if(config->advParams->unrollingParams)
			memcpy(&inst->params.unrollingParams, config->advParams->unrollingParams, sizeof(UnrollingParams));
		if(config->advParams->allocationParams)
			memcpy(&inst->params.allocationParams, config->advParams->allocationParams, sizeof(AllocationParams));
		if(config->advParams->variationParams)
			memcpy(&inst->params.variationParams, config->advParams->variationParams, sizeof(VarParams));
        if(config->advParams->sceneryParams)
            memcpy(&inst->params.sceneryParams, config->advParams->sceneryParams, sizeof(SceneryParams));
	}

	// configured parameters
	inst->params.stateVectorType = config->stateVectorType;
	inst->params.deltaT = config->deltaT;
	inst->params.maxAcceleration = config->maxAcceleration;
	inst->params.maxURadialVelocity = config->maxURadialVelocity;
	inst->params.initialRadialVelocity = config->initialRadialVelocity;

	switch(config->verbose) {
		case GTRACK_VERBOSE_NONE:
			inst->params.verbose = 0;
			break;
		case GTRACK_VERBOSE_ERROR:
			inst->params.verbose = VERBOSE_ERROR_INFO;
			break;
		case GTRACK_VERBOSE_WARNING:
			inst->params.verbose = VERBOSE_ERROR_INFO | VERBOSE_WARNING_INFO;
			break;
		default:
		case GTRACK_VERBOSE_DEBUG:
			inst->params.verbose = VERBOSE_ERROR_INFO | VERBOSE_WARNING_INFO | VERBOSE_DEBUG_INFO | VERBOSE_UNROLL_INFO | VERBOSE_STATE_INFO;
			break;
		case GTRACK_VERBOSE_MATRIX:			
			inst->params.verbose = VERBOSE_ERROR_INFO | VERBOSE_WARNING_INFO | VERBOSE_DEBUG_INFO | VERBOSE_MATRIX_INFO;
			break;
		case GTRACK_VERBOSE_MAXIMUM:
			inst->params.verbose = VERBOSE_ERROR_INFO | VERBOSE_WARNING_INFO | VERBOSE_DEBUG_INFO | VERBOSE_MATRIX_INFO | VERBOSE_UNROLL_INFO | VERBOSE_STATE_INFO | VERBOSE_ASSOSIATION_INFO;
			break;
	};

	// computed parameters
	dt = config->deltaT;
	dt2 = powf(dt,2);
	dt3 = powf(dt,3);
	dt4 = powf(dt,4);

	{
		float F4[16] = {				
		1,  0,  dt,	0,
		0,  1,  0,	dt,
		0,  0,  1,	0,
		0,  0,  0,	1};

		float Q4[16] = {
		dt4/4,	0,		dt3/2,	0,
		0,		dt4/4,	0,		dt3/2,
		dt3/2,	0,		dt2,	0,
		0,		dt3/2,	0,		dt2};

		float F6[36] = {				
		1,  0,  dt,	0,	dt2/2, 0, 
		0,  1,  0,	dt,	0,		dt2/2,
		0,  0,  1,	0,	dt,		0,
		0,  0,  0,	1,	0,		dt,
		0,	0,	0,	0,	1,		0,
		0,	0,	0,	0,	0,		1};

		float Q6[36] = {
		dt4/4,	0,		dt3/2,	0,		dt2/2,	0,
		0,		dt4/4,	0,		dt3/2,	0,		dt2/2,
		dt3/2,	0,		dt2,	0,		dt,		0,
		0,		dt3/2,	0,		dt2,	0,		dt,
		dt2/2,	0,		dt,		0,		1,		0,
		0,		dt2/2,	0,		dt,		0,		1};
	
		memcpy(inst->params.F4, F4, sizeof(F4));
		memcpy(inst->params.Q4, Q4, sizeof(Q4));
		memcpy(inst->params.F6, F6, sizeof(F6));
		memcpy(inst->params.Q6, Q6, sizeof(Q6));
	}

	// hTrack is an array of void pointers
	inst->hTrack = (void **) gtrack_alloc(inst->maxNumTracks, sizeof(GtrackUnitInstance *));

	// scoreSheet is an array of best scores
	inst->bestScore = (float *) gtrack_alloc(inst->maxNumPoints, sizeof(float));
	// association array holds the ids of the best scorers
	inst->bestIndex = (uint8_t *) gtrack_alloc(inst->maxNumPoints, sizeof(uint8_t));
	// allocation array holds the measurement indices of allocation set
	inst->allocIndex = (uint16_t *) gtrack_alloc(inst->maxNumPoints, sizeof(uint16_t));
	// array of tracking IDs
	inst->tidElem = (GTrack_ListElem *) gtrack_alloc(inst->maxNumTracks, sizeof(GTrack_ListElem));

	inst->targetNum = 0;
	
	gtrack_listInit(&inst->freeList);
	gtrack_listInit(&inst->activeList);

	// Create unit trackers
	for(n=0; n < inst->maxNumTracks; n++) {
		inst->tidElem[n].data = n;
		gtrack_listEnqueue(&inst->freeList, &inst->tidElem[n]);
		inst->params.tid = n;
		inst->hTrack[n] = gtrack_unitCreate(&inst->params);
	}

	return (void *)inst;
}

