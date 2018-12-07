/**
 *   @file  unwrap.c
 *
 *   @brief
 *      Phase unwrap utility function
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

/**
 *  @file   unwrap.c
 *  @brief  implement phase unwrap function
 *
 */

#include "unwrap.h"

float unwrap(float phase, float phasePrev)
{
	//uint16_t modFactor;
	float modFactorF;
	float diffPhase;
	float diffPhaseMod;
	float diffPhaseCorrection;
	float phaseOut;

	// incremental phase variation
	// MATLAB: dp = diff(p, 1, 1);
	diffPhase = phase - phasePrev;

	// equivalent phase variation in [-pi, pi]
	// MATLAB: dps = mod(dp+pi,2*pi) - pi;
	//modFactor = (uint16_t) divsp_i(diffPhase + M_PI, 2*M_PI);
	//modFactorF = (float) modFactor;
	if (diffPhase > M_PI)
		modFactorF = 1;
	else if (diffPhase < - M_PI)
	    modFactorF = -1;
	else
		modFactorF = 0;

	diffPhaseMod = diffPhase - modFactorF*2*M_PI;

	// preserve variation sign for +pi vs. -pi
	// MATLAB: dps(dps==pi & dp>0,:) = pi;
	if ((diffPhaseMod == -M_PI) && (diffPhase > 0))
		diffPhaseMod = M_PI;

	// incremental phase correction
	// MATLAB: dp_corr = dps - dp;
	diffPhaseCorrection = diffPhaseMod - diffPhase;

	// Ignore correction when incremental variation is smaller than cutoff
	// MATLAB: dp_corr(abs(dp)<cutoff,:) = 0;
	if (((diffPhaseCorrection < M_PI) && (diffPhaseCorrection>0)) ||
			((diffPhaseCorrection > -M_PI) && (diffPhaseCorrection<0)))
		diffPhaseCorrection = 0;

	// Find cumulative sum of deltas
	// MATLAB: cumsum = cumsum(dp_corr, 1);

	// Integrate corrections and add to P to produce smoothed phase values
	// MATLAB: p(2:m,:) = p(2:m,:) + cumsum(dp_corr,1);
	phaseOut = phase + diffPhaseCorrection;
    return phaseOut;
 }
