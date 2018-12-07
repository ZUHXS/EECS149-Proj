/*! 
 *  \file   detectionRagDop.c
 *
 *  \brief   CFAR functions.
 *
 *  Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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


/**
 *  @file   detection.h
 *  @brief  detection API implementation
 *
 */

#include <modules/detection/api/detection.h>
#include <modules/utilities/radarOsal_malloc.h>
#include <modules/edma/api/edma.h>
#include <stdio.h>


typedef struct _detectionInstance_
{

	uint16_t startRangeCell; /**< start index where range bins are kept, detected range index should be detectValue + startRangeCell. */
	uint16_t  rangeNumBins; /**< number of bins in range direction. */
	uint16_t  DopplerNumBins; /**< number of bins in Doppler direction. */
	uint16_t  refWinSize[2]; /**< reference window size in each side in two directions for clutter variance estimation. */
	uint16_t  guardWinSize[2]; /**< guard window size in each side in two directions. */
	float  thre; /**< threshold used for compare. */
	uint16_t  threWinSize; /**< window size for the second step threshold detection within neighborhood of the first detected point. */
	float  threRatio; /**< ration for neighborhood detection, thre = threRatio*peak. */
	float rangeRes; /**< range resolution in meters. */
	float velocityRes; /**< velocity resolution in meters/sec. */
	uint16_t detMethodID; /**< detection method ID, for future use for different type of detection method. */
	uint16_t  numAntenna; /**< number of antennas. */
	uint16_t *pDetectedTargetsRange; /**< memory used to store range line detection index. */
	float *pDetectedTargetsRangeNoise; /**< memory used to store range line detection noisePower. */
	//uint32_t *pDetTargetRangeNoiseVar; /**< memory used to store range line detection noise variance. */
	float *pDataIntegrate; /**< memory used to store integration results. */
	//char *pDetectionResults; /**< memory used to store final detection results. */
	uint16_t numObjDetFirstDim; /**< number of objects detected along the first direction. */
	uint16_t numObj; /**< number of detected objects. */

	char *scrachMemL2;
	uint32_t stachMemSizeL2;
	char *scrachMemDDR;
	uint32_t stachMemSizeDDR;


}detectionInstance_t;


void integrateNonCoherent(cplxf_t *restrict pDataIn,detectionInstance_t *restrict pDetectionParams, float *restrict pDataIntegrate);

uint16_t detectTargetsRange_CASO(float *restrict pDataIn, detectionInstance_t *restrict pDetectionParams);


void *detectTargetsCreate(detectionParaConfig_t *detectionParams,
		                  void                  *pDataIntegrate)
{
	detectionInstance_t *inst;
	unsigned int memoryUsed = 0;

	memoryUsed += sizeof(detectionInstance_t);
	inst = (detectionInstance_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(detectionInstance_t), 8);

	//carry all parameters from configuration parameters
	inst->startRangeCell = detectionParams->startRangeCell;
	inst->rangeNumBins = detectionParams->rangeNumBins;
	inst->DopplerNumBins = detectionParams->DopplerNumBins;
	inst->refWinSize[0]  = detectionParams->refWinSize[0];
	inst->refWinSize[1]  = detectionParams->refWinSize[1];
	inst->guardWinSize[0]  = detectionParams->guardWinSize[0];
	inst->guardWinSize[1]  = detectionParams->guardWinSize[1];
	inst->thre = detectionParams->thre;
	inst->threWinSize = detectionParams->threWinSize;
	inst->threRatio = detectionParams->threRatio;
	inst->rangeRes = detectionParams->rangeRes;
	inst->velocityRes = detectionParams->velocityRes;
	inst->detMethodID = detectionParams->detMethodID;
	inst->numAntenna = detectionParams->numAntenna;

	//initialize memory from L2 for first step detection along range direction, used as scratch pad, to store range and Doppler index with type of uint16_t
	inst->stachMemSizeL2 = (2 * sizeof(uint16_t)+sizeof(float))*DETECTION_MAX_NUM_OBJ_RANGE;
	inst->pDetectedTargetsRange = (uint16_t *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 1, inst->stachMemSizeL2, 8);
	inst->scrachMemL2 = (char *)inst->pDetectedTargetsRange;
	inst->pDetectedTargetsRangeNoise = (float *) (inst->pDetectedTargetsRange + 2 * sizeof(uint16_t)*DETECTION_MAX_NUM_OBJ_RANGE);
	memoryUsed +=  inst->stachMemSizeL2;

	//initialize memory for integration, get sratch pad memory from DDR, not from L2
	inst->pDataIntegrate = (float*)pDataIntegrate;
    inst->scrachMemDDR = (char *)inst->pDataIntegrate;
    inst->stachMemSizeDDR = inst->rangeNumBins*inst->DopplerNumBins*sizeof(uint32_t);

	return (void *) inst;
}


int32_t detectTargetsRun(void *handle, cplxf_t *restrict pDataIn, detectedResults_t *restrict pDataOut)
{
	int32_t DetectionStatus;

	//DetectionStatus = detectTargetsRangeDopplerThre(handle, pDataIn, pDataOut);
	DetectionStatus = detectTargetsRangeDopplerCross(handle, pDataIn, pDataOut);


	return DetectionStatus;
}

int32_t detectTargetsRangeDopplerCross(void *handle, cplxf_t *restrict pDataIn, detectedResults_t *restrict pDetectedTargets)
{
	uint16_t numObjDetRange, numObjDet, numObjDetValid, i_objDet, i_search, i_Dop, i_refWin;
	float *restrict pCUT, *restrict pRefWinL, *restrict pRefWinLStart, *restrict pRefWinR,*restrict pRefWinRStart;
	float noiseEst[MAX_NUM_DOPPLER_LEN], noise;
	uint32_t offset;
	float *restrict pDataIntegrate;
	uint16_t *restrict pDetectedTargetsRange, *restrict pDetectedTargetsRangeSt, DopplerNumBins, rangeCell;
	float *restrict pDetectedTargetsRangeNoise;
	uint8_t flag[MAX_NUM_RANGE_LEN], refWinSize, guardWinSize, refWinGuardSize;
	float averageRatio, thre;
	cplxf_t *restrict pDataInStart, *restrict pDataInVal;
	float rangeVar, velVar, SInverse, noiseAveFactor;
	activeBin_t *restrict pDetectedTargetsStart, *pDetectedTargetsLoc;
	detectionInstance_t *pDetectionParams = (detectionInstance_t *)handle;

	pDetectedTargetsLoc = pDetectedTargets->activebins;


#ifdef PROFILE_CYCLES_IN
	double overhead=0, cpuCycles = 0;
	Types_Timestamp64 timeIn, timeOut;
#endif



	pDataIntegrate = pDetectionParams->pDataIntegrate; /*pointer to the integration memory*/
	DopplerNumBins = pDetectionParams->DopplerNumBins;
	//DopplerNumBins2 = DopplerNumBins >> 1;
	refWinSize = pDetectionParams->refWinSize[1]; /*refernce win size on each side*/
	guardWinSize = pDetectionParams->guardWinSize[1]; /*guard win size on each side*/
	refWinGuardSize = refWinSize + guardWinSize;
	averageRatio = (float) 1/(refWinSize<<1);
	thre = pDetectionParams->thre * averageRatio; /*lump the average ratio to the threshold*/
	pDetectedTargetsStart = pDetectedTargetsLoc;


	//error due to quantization (deltaR^2/12)
	rangeVar = pDetectionParams->rangeRes * pDetectionParams->rangeRes * 0.08333;
	//error due to quantization (deltaV^2/12)
	velVar = pDetectionParams->velocityRes * pDetectionParams->velocityRes * 0.08333;

	numObjDet = 0;
	numObjDetRange = 0;

#ifdef PROFILE_CYCLES_IN
Timestamp_get64(&timeIn);
Timestamp_get64(&timeOut);
overhead = getTimeStampDif(timeIn, timeOut);
Timestamp_get64(&timeIn);
#endif
	/*peform integration*/
	integrateNonCoherent(pDataIn, pDetectionParams, pDataIntegrate);

#ifdef PROFILE_CYCLES_IN
Timestamp_get64(&timeOut);
cpuCycles = getTimeStampDif(timeIn, timeOut);
cpuCycles -= overhead;
System_printf("Total cycle count for integration is %f\n", cpuCycles);
Timestamp_get64(&timeIn);
#endif

	/*first range line detection*/
	numObjDetRange = detectTargetsRange_CASO(pDataIntegrate, pDetectionParams);

	pDetectionParams->numObjDetFirstDim = numObjDetRange;

#ifdef PROFILE_CYCLES_IN
Timestamp_get64(&timeOut);
cpuCycles = getTimeStampDif(timeIn, timeOut);
cpuCycles -= overhead;
System_printf("Total cycle count for detection in range line is %f\n", cpuCycles);
Timestamp_get64(&timeIn);
#endif

	/*second Doppler line detection*/
	/*pDetectedTargetsRange has been updated after calling detectTargetsRange_CA*/
	pDetectedTargetsRange = pDetectionParams->pDetectedTargetsRange;
	

	memset(&(flag[0]), 0, MAX_NUM_RANGE_LEN);

	for (i_objDet = 0; i_objDet < numObjDetRange; i_objDet ++)
	{
		rangeCell = *pDetectedTargetsRange;

		if (flag[rangeCell] == 0)
		{
			//printf("%d \n", rangeCell);
			/*set up the flag so that the same Doppler line won't be processed twice*/
			flag[rangeCell] = 1;
			/*point to the detected range cell for that Doppler line*/
			pCUT = pDataIntegrate + rangeCell * DopplerNumBins;
			pRefWinR = pCUT + guardWinSize + 1;
			pRefWinRStart = pRefWinR;
			pRefWinL = pCUT + (DopplerNumBins-refWinGuardSize);
			pRefWinLStart = pRefWinL;


			/*when processing the element at the left and right boundary, wrap around*/
			////memset(&(noiseEst[0]), 0, sizeof(uint32_t)*MAX_NUM_DOPPLER_LEN);
			noiseEst[0] = 0;
			for(i_refWin = 0; i_refWin < refWinSize; i_refWin ++)
			{
				noiseEst[0] += *pRefWinL;
				pRefWinL ++;
				noiseEst[0] += *pRefWinR;
				pRefWinR ++;
			}
			/*noise estimation for [1 guardWinSize]*/
			for(i_Dop = 1; i_Dop <= guardWinSize; i_Dop ++)
			{
				noiseEst[i_Dop] = noiseEst[i_Dop-1]- *pRefWinRStart;
				noiseEst[i_Dop] += *pRefWinR;
				noiseEst[i_Dop] = noiseEst[i_Dop-1]- *pRefWinLStart;
				noiseEst[i_Dop] += *pRefWinL;
				pRefWinR ++;
				pRefWinRStart ++;
				pRefWinLStart ++;
				pRefWinL ++;

			}

			/*noise estimate for next refWinGuardSize-1 Doppler cells*/
			pRefWinL = pCUT;
			for(i_Dop = guardWinSize+1; i_Dop <= refWinGuardSize; i_Dop ++)
			{
				noiseEst[i_Dop] = noiseEst[i_Dop-1]- *pRefWinLStart; /*pRefWinLStart stays at PCUT*/
				noiseEst[i_Dop] += *pRefWinL;
				noiseEst[i_Dop] -= *pRefWinRStart;
				noiseEst[i_Dop] += *pRefWinR;

				pRefWinRStart ++;
				pRefWinLStart ++;
				pRefWinL ++;
				pRefWinR ++;

			}

			/*noise estimate for middle [refGuard DopplerSize-refGuard] Doppler cells*/
			pRefWinLStart = pCUT;
			for(i_Dop = refWinGuardSize+1; i_Dop < DopplerNumBins-refWinGuardSize; i_Dop ++)
			{
				noiseEst[i_Dop] = noiseEst[i_Dop-1]- *pRefWinLStart;
				noiseEst[i_Dop] -= *pRefWinRStart;
				noiseEst[i_Dop] += *pRefWinL;
				noiseEst[i_Dop] += *pRefWinR;

				pRefWinLStart ++;
				pRefWinRStart ++;
				pRefWinL ++;
				pRefWinR ++;

			}


			/*noise estimate for last refGuard Doppler cells*/
			pRefWinR = pCUT;
			for(i_Dop = DopplerNumBins-refWinGuardSize; i_Dop < DopplerNumBins-guardWinSize; i_Dop ++)
			{
				noiseEst[i_Dop] = noiseEst[i_Dop-1]- *pRefWinLStart;
				noiseEst[i_Dop] -= *pRefWinRStart;
				noiseEst[i_Dop] += *pRefWinL;
				noiseEst[i_Dop] += *pRefWinR; /*pRefWinR stays at the right most value*/

				pRefWinLStart ++;
				pRefWinRStart ++;
				pRefWinL ++;
				pRefWinR ++;

			}

			pRefWinRStart = pCUT;
			for(i_Dop = DopplerNumBins-guardWinSize; i_Dop < DopplerNumBins; i_Dop ++)
			{
				noiseEst[i_Dop] = noiseEst[i_Dop-1]- *pRefWinLStart;
				noiseEst[i_Dop] -= *pRefWinRStart;
				noiseEst[i_Dop] += *pRefWinL;
				noiseEst[i_Dop] += *pRefWinR; /*pRefWinR stays at the right most value*/

				pRefWinLStart ++;
				pRefWinRStart ++;
				pRefWinL ++;
				pRefWinR ++;

			}

			/*detect at Doppler cell based on the noise estimation*/
			for(i_Dop = 0; i_Dop < DopplerNumBins; i_Dop ++)
			{
				if ((numObjDet < DETECTION_MAX_NUM_OBJ))
				{
					if(*pCUT > thre * noiseEst[i_Dop])
					{
						/*find if this detection overlap with that in range line detecion*/
						pDetectedTargetsRangeSt = pDetectionParams->pDetectedTargetsRange;
						pDetectedTargetsRangeNoise = pDetectionParams->pDetectedTargetsRangeNoise;
						for (i_search = 0; i_search < numObjDetRange; i_search ++)
						{
							/*compare the range index*/
							if (*pDetectedTargetsRangeSt++ != rangeCell)
							{
								pDetectedTargetsRangeSt ++;								
							}
							else
							{
								/*compare the Doppler index*/
								if (*pDetectedTargetsRangeSt++ == i_Dop)
								{
									numObjDet ++;
									pDetectedTargetsLoc->rangeIndex = rangeCell;
									pDetectedTargetsLoc->DopplerIndex = i_Dop;
									pDetectedTargetsLoc->noisePower =  *pDetectedTargetsRangeNoise;
									/*compute range and velocity*/
									pDetectedTargetsLoc->range = (rangeCell +pDetectionParams->startRangeCell ) * pDetectionParams->rangeRes;
									//pDetectedTargetsLoc->velocity = (i_Dop - DopplerNumBins2)* pDetectionParams->velocityRes; //with FFT shift
									if (i_Dop >= (DopplerNumBins >> 1))
									    pDetectedTargetsLoc->velocity = (i_Dop - DopplerNumBins)* pDetectionParams->velocityRes; //without FFTshift
									else
										pDetectedTargetsLoc->velocity = (i_Dop)* pDetectionParams->velocityRes;
									/*compute the error due to quantization*/
									pDetectedTargetsLoc->rangeVar = rangeVar;
									pDetectedTargetsLoc->velocityVar = velVar;

									pDetectedTargetsLoc ++;


									break; // break search for (i_search = 0; i_search < numObjDetDoppler; i_search ++)

								}
							}
							pDetectedTargetsRangeNoise ++;

						}
					}

				}
				else
					break;

				pCUT ++;
			}


		}
		pDetectedTargetsRange +=2;
		

	}

#ifdef PROFILE_CYCLES_IN
Timestamp_get64(&timeOut);
cpuCycles = getTimeStampDif(timeIn, timeOut);
cpuCycles -= overhead;
System_printf("Total cycle count for detection in Doppler line is %f\n", cpuCycles);
Timestamp_get64(&timeIn);
#endif

	/*set the reference cell size for range direction*/
	refWinSize = pDetectionParams->refWinSize[0]; /*refernce win size on each side*/
	guardWinSize = pDetectionParams->guardWinSize[0]; /*guard win size on each side*/
	refWinGuardSize = refWinSize + guardWinSize;
	/*factor used to compute the average of noise from reference cells*/
	noiseAveFactor = (float)1/refWinSize;

	/*compute noiseVar for each antenna at the detected location along the range direction*/
	pDetectedTargetsLoc = pDetectedTargetsStart;
	offset = DopplerNumBins * pDetectionParams->numAntenna;

	//get the square value first. rangeVar and velVar are reused here!
	rangeVar = pDetectionParams->rangeRes * pDetectionParams->rangeRes;
	velVar = pDetectionParams->velocityRes * pDetectionParams->velocityRes;

	numObjDetValid = 0; //count the final valid object
	for(i_Dop = 0; i_Dop < numObjDet; i_Dop ++)
	{
		/*starting address of each detected range cell*/
		pDataInStart = pDataIn + offset * pDetectedTargetsLoc->rangeIndex;
		pDataInVal = pDataInStart + pDetectedTargetsLoc->DopplerIndex;


		pDetectedTargetsLoc->binVal[0] = *pDataInVal;
		pDataInVal += DopplerNumBins;

		pDetectedTargetsLoc->binVal[1] = *pDataInVal;
		pDataInVal += DopplerNumBins;

		pDetectedTargetsLoc->binVal[2] = *pDataInVal;
		pDataInVal += DopplerNumBins;

		pDetectedTargetsLoc->binVal[3] = *pDataInVal;

		/*copy information if the current target is valid*/
		pDetectedTargetsLoc->noisePower = (pDetectedTargetsLoc->noisePower * noiseAveFactor);

		/*compute the error due to quantization*/
		/*add the noise due to SInverse, deltaR^2/(2SNR)*/
		pCUT = pDataIntegrate + pDetectedTargetsLoc->rangeIndex * DopplerNumBins;
		pCUT += pDetectedTargetsLoc->DopplerIndex;
		noise = pDetectedTargetsLoc->noisePower;
		SInverse = 1 / (float)*pCUT ;
		SInverse *= 0.5;
		pDetectedTargetsLoc->rangeVar += rangeVar * noise * SInverse;
		pDetectedTargetsLoc->velocityVar += velVar * noise * SInverse;

		/*add startRangeCell to consider the range bin offset, only [startRangeCell endRangeCell] was kept after range fft*/
		pDetectedTargetsLoc->rangeIndex += pDetectionParams->startRangeCell ;

		pDetectedTargetsLoc ++;
		numObjDetValid ++;

	}

#ifdef PROFILE_CYCLES_IN
Timestamp_get64(&timeOut);
cpuCycles = getTimeStampDif(timeIn, timeOut);
cpuCycles -= overhead;
System_printf("Total cycle count for setting final value is %f\n", cpuCycles);
#endif

	pDetectedTargets->numObjects = numObjDetValid;
	pDetectionParams->numObj = numObjDetValid;

	return DETECTION_OK;



}//detectTargetsRangeDopplerCross()



int32_t detectTargetsDelete(void *handle)
{
	//detectionInstance_t *inst = (detectionInstance_t*)handle;

	//radarOsal_memFree(inst->scrachMemL2, inst->stachMemSizeL2);
	//radarOsal_memFree(inst->scrachMemDDR, inst->stachMemSizeDDR);
	radarOsal_memFree(handle, sizeof(detectionInstance_t));
	return DETECTION_OK;
}


void integrateNonCoherent(cplxf_t *restrict pDataIn,detectionInstance_t *restrict pDetectionParams, float *restrict pDataIntegrate)
{
	cplxf_t *restrict pDataInLoc1, *restrict pDataInLoc2, *restrict pDataInLoc3, *restrict pDataInLoc4;
	uint16_t rangeNumBins, DopplerNumBins, i_range, i_Dop;



	rangeNumBins = pDetectionParams->rangeNumBins;
	DopplerNumBins = pDetectionParams->DopplerNumBins;
	pDataInLoc1 = (cplxf_t *)pDataIn;
	pDataInLoc2 = pDataInLoc1 + DopplerNumBins;
	pDataInLoc3 = pDataInLoc2 + DopplerNumBins;
	pDataInLoc4 = pDataInLoc3 + DopplerNumBins;


	for (i_range = 0; i_range < rangeNumBins; i_range ++)
	{

		for (i_Dop = 0; i_Dop < DopplerNumBins; i_Dop ++)
		{

			*pDataIntegrate = pDataInLoc1->imag * pDataInLoc1->imag + pDataInLoc1->real * pDataInLoc1->real;
			*pDataIntegrate += pDataInLoc2->imag * pDataInLoc2->imag + pDataInLoc2->real * pDataInLoc2->real;
			*pDataIntegrate += pDataInLoc3->imag * pDataInLoc3->imag + pDataInLoc3->real * pDataInLoc3->real;
			*pDataIntegrate += pDataInLoc4->imag * pDataInLoc4->imag + pDataInLoc4->real * pDataInLoc4->real;

			*pDataIntegrate ++;
			pDataInLoc1 ++;
			pDataInLoc2 ++;
			pDataInLoc3 ++;
			pDataInLoc4 ++;


		}
		/*update the pointers for 4 antennas*/
		pDataInLoc1 = pDataInLoc4;
		pDataInLoc2 = pDataInLoc1 + DopplerNumBins;
		pDataInLoc3 = pDataInLoc2 + DopplerNumBins;
		pDataInLoc4 = pDataInLoc3 + DopplerNumBins;
	}
}




uint16_t detectTargetsRange_CASO(float *restrict pDataIn, detectionInstance_t *restrict pDetectionParams)
{

	uint16_t refWinSize, guardWinSize, refWinGuardSize, CUTEnd, i_refWin, i_Dop, i_cell, i_CUT, *restrict pDetectedTargetsRange;
	float *restrict pDetectedTargetsRangeNoise;
	uint16_t DopplerNumBins; /*64 bytes = 16 number of 32 bits*/
	float *restrict pRefWinL, *restrict pRefWinLStart, *restrict pRefWinR, *restrict pRefWinRStart, *restrict pCUT;
	float averageRatio, thre, thre_half;
	float noiseEstL[MAX_NUM_DOPPLER_LEN], noiseEstR[MAX_NUM_DOPPLER_LEN], noiseEstSmall;
	//uint32_t *restrict pDetTargetRangeNoiseVar;
	uint32_t numObjDet = 0, /*offsetRefGuard, */offsetGuard;
	//long long sum2Int32;


	DopplerNumBins =  pDetectionParams->DopplerNumBins;
	refWinSize = pDetectionParams->refWinSize[0]; /*refernce win size on each side*/
	guardWinSize = pDetectionParams->guardWinSize[0]; /*guard win size on each side*/
	refWinGuardSize = refWinSize + guardWinSize;
	averageRatio = (float) 1/(refWinSize);
	thre = pDetectionParams->thre * averageRatio; /*lump the araverate ratio to the threshold*/
	thre_half = pDetectionParams->thre * averageRatio; /*for first guard+ref cells*/


	//offsetRefGuard = DopplerNumBins * refWinGuardSize;
	offsetGuard = DopplerNumBins * (guardWinSize+1);

	/*actual range bins to detect, assuming that the last refWinSize+guardWinSize cells don't have target*/
	CUTEnd = pDetectionParams ->rangeNumBins-refWinGuardSize;


	pCUT = pDataIn;
	pDetectedTargetsRange = pDetectionParams->pDetectedTargetsRange;
	pDetectedTargetsRangeNoise = pDetectionParams->pDetectedTargetsRangeNoise;

	pRefWinL = pCUT;
	pRefWinR = pCUT + offsetGuard;  /*pointer to right side ref win*/
	//pRefWinLStart = pRefWinL;
	pRefWinRStart = pRefWinR;


	memset(&(noiseEstL[0]), 0, sizeof(uint32_t)*DopplerNumBins);
	memset(&(noiseEstR[0]), 0, sizeof(uint32_t)*DopplerNumBins);

	/*assuming that the last refWinSize+guardWinSize cells don't have target */
	/*detection on the left boundary by applying the extending rule : 000000001234567*/
	/*detection on the first range cell*/


	/*estimate noise for first cells only use the right hand samples */
	for(i_refWin = 0; i_refWin < refWinSize; i_refWin ++)
	{
		for (i_Dop = 0; i_Dop < DopplerNumBins; i_Dop++)
		{
			noiseEstR[i_Dop] += *pRefWinR;
			pRefWinR ++;
			noiseEstL[i_Dop] += *pRefWinL;
			pRefWinL ++;
		}

	}

	/*estimate noise for first refWinGuardSize cells only use the right hand samples */
	pCUT += DopplerNumBins; /*first range cell are not used for detection, skip it*/
	for(i_cell = 1; i_cell <= refWinGuardSize; i_cell ++)
	{
		for (i_Dop = 0; i_Dop < DopplerNumBins; i_Dop++)
		{
			noiseEstR[i_Dop] -= *pRefWinRStart;
			noiseEstR[i_Dop] += *pRefWinR;
			pRefWinR ++;
			pRefWinRStart ++;
		}

		for (i_Dop = 0; i_Dop < DopplerNumBins; i_Dop++)
		{
			/*detection*/
			if (numObjDet < DETECTION_MAX_NUM_OBJ_RANGE)
			{
				if (*pCUT > thre_half * noiseEstR[i_Dop])
				{
					/*detected*/
					numObjDet += 1;
					*pDetectedTargetsRange = i_cell;
					pDetectedTargetsRange ++;
					*pDetectedTargetsRange = i_Dop;
					pDetectedTargetsRange ++;
					*pDetectedTargetsRangeNoise = noiseEstR[i_Dop];
					pDetectedTargetsRangeNoise ++;
					

				}
				pCUT ++;
			}
			else
				return numObjDet;
		}


	}

	/*Process the following range cells*/
	//pRefWinL = pDataIn;
	pRefWinLStart = pDataIn;

	i_CUT = refWinGuardSize + 1;
	for (; i_CUT < CUTEnd; i_CUT ++)
	{
		for (i_Dop = 0; i_Dop < DopplerNumBins; i_Dop++)
		{
			/*subtract the sample just moved out of the window*/
			//sum2Int32 = _itoll(*pRefWinLStart, *pRefWinL) + _itoll(*pRefWinRStart, *pRefWinR);
			/*add the sample just moved into of the window*/
			//noiseEst[i_Dop] -= _hill(sum2Int32);
			//noiseEst[i_Dop] += _loll(sum2Int32);
			noiseEstL[i_Dop] -= *pRefWinLStart;
			noiseEstL[i_Dop] += *pRefWinL;
			noiseEstR[i_Dop] -= *pRefWinRStart;
			noiseEstR[i_Dop] += *pRefWinR;

			pRefWinL ++;
			pRefWinR ++;
			pRefWinLStart ++;
			pRefWinRStart ++;
		}
		for (i_Dop = 0; i_Dop < DopplerNumBins; i_Dop++)
		{
			if (numObjDet < DETECTION_MAX_NUM_OBJ_RANGE)
			{
				noiseEstSmall = noiseEstL[i_Dop];

				if (noiseEstL[i_Dop] > noiseEstR[i_Dop])
				{
					noiseEstSmall = noiseEstR[i_Dop];
				}
				/*detection*/
				if (*pCUT > thre * noiseEstSmall)
				{
					/*detected*/
					numObjDet += 1;
					*pDetectedTargetsRange = i_CUT;
					pDetectedTargetsRange ++;
					*pDetectedTargetsRange = i_Dop;
					pDetectedTargetsRange ++;
					*pDetectedTargetsRangeNoise = noiseEstSmall;
					pDetectedTargetsRangeNoise ++;
				}
				pCUT ++;
			}
			else
				return numObjDet;

		}

	}
	return numObjDet;
}




int32_t detectTargetsDebugQuery(void *handle)
{
	detectionInstance_t *inst =  (detectionInstance_t *)handle;

	uint32_t numSample;

	numSample= sizeof(detectedResults_t)-(DETECTION_MAX_NUM_OBJ-inst->numObj)*sizeof(activeBin_t);

	return numSample;
}




