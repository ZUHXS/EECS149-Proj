/**
 *   @file  linearfit.c
 *
 *   @brief
 *      Linear fit utility function
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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


#include "linearFit.h"

void computeLinearFit (float *pDataInX, float *pDataInY, linearFitParams_t *plinearFitParamsOut, uint_least32_t sizeData)
{
    float sumX, sumY, sumXY, sumX2;
	float squareSumX;
	float yInstant, timeInstant;
	float slope, intercept;
	float tempVar;
	int_least16_t tempIndex;
	sumX = 0;
	sumY = 0;
	sumXY = 0;
	sumX2 = 0;
	squareSumX = 0;


	for (tempIndex=0; tempIndex < sizeData; tempIndex++)
	{
	timeInstant = pDataInX[tempIndex];
	yInstant    = pDataInY[tempIndex];

	sumX  = sumX  + timeInstant;
	sumY  = sumY  + yInstant;

	tempVar = timeInstant*yInstant;
	sumXY = sumXY + tempVar;

	sumX2 = sumX2 + timeInstant*timeInstant;
	}

	squareSumX = sumX*sumX;
	tempVar = (float) sizeData * sumXY - (sumX)*(sumY);
	slope = tempVar/( (float) sizeData *sumX2 - squareSumX );
	intercept = (sumY - slope*sumX)/(float) sizeData;

	plinearFitParamsOut->slope = slope;
	plinearFitParamsOut->intercept = intercept;

#ifdef	DEBUG
	System_printf("\n  The slope = %f  Intercept = %f \n",slope, intercept);
#endif

}
