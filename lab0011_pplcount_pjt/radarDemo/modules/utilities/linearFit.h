/**
 *   @file  linearfit.h
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


#ifndef RADARDEMO_LINEARFIT_H_
#define RADARDEMO_LINEARFIT_H_

/**
 *  @file   linearFit.h
 *
 *  @brief  Performs a linear fit on the data.
 *
 * *
 *  @param[in]   pDataInX                   The X-axis values of the data to be fitted
 *  @param[in]   pDataInY                   The data values (corresponding to the x-axis values) to be fitted to a straight line
 *  @param[out]  plinearFitParamsOut        The computed "slope" and "intercept" are stored in this structure
 *  @param[in]   sizeData                   Number of samples in the input data
 *
 *  @remarks
 */


#include <types/types.h>

typedef struct
{
  float slope;       // The estimated Slope
  float intercept;   // The estimated Intercept
} linearFitParams_t;

void computeLinearFit (float *pDataInX, float *pDataInY, linearFitParams_t *plinearFitParamsOut, uint_least32_t sizeData);


#endif /* RADARDEMO_LINEARFIT_H_ */
