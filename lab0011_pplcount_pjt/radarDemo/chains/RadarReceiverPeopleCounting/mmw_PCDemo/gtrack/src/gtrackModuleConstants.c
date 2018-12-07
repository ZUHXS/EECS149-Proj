/**
 *   @file  gtrackModuleConstants.c
 *
 *   @brief
 *      This is the set of constants used by GTRACK Algorithm
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

#include <ti/alg/gtrack/gtrack.h>
#include <ti/alg/gtrack/include/gtrack_int.h>

const SceneryParams defaultSceneryParams = {-100, 100, 0, 100}; // No walls, no entrance areas
const GatingParams defaultGatingParams = {2, {3, 2, 0}};
const ScoringParams defaultScoringParams = {1};
const StateParams defaultStateParams = {3, 3, 5, 5, 5}; // 3 frames to activate, 100 frames to forget
const AllocationParams defaultAllocationParams = {100, 0.5, 5, 1, 2}; // At least 100 SNR, 0.5 m/s, 5 points: up to 1m in distance, up to 2m/c in velocity
const UnrollingParams defaultUnrollingParams = {0.5f, 0.1f};
const VarParams defaultVariationParams = {1.f/3.46f, 1.f/3.46f, 2.f}; // Based on standard deviation of uniformly distributed variable in range [a b]: 1/sqrt(12)*(b-a). For object of 1m height, 1m width, 1m/s doppler spread

const float eye4[16] = {
	1,0,0,0,
	0,1,0,0,
	0,0,1,0,
	0,0,0,1
};
const float eye6[36] = {
	1,0,0,0,0,0,
	0,1,0,0,0,0,
	0,0,1,0,0,0,
	0,0,0,1,0,0,
	0,0,0,0,1,0,
	0,0,0,0,0,1
};
