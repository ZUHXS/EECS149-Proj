/**
 *   @file  gtrack_int.h
 *
 *   @brief
 *      This is the internal header file for GTRACK Algorithm
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


#ifndef GTRACK_INT_LIB_H__
#define GTRACK_INT_LIB_H__

#include <stdint.h>
#include "gtrack_listlib.h"

#define MIN_RANGE_VARIANCE (0.4f/2)*(0.4f/2)
#define MIN_ANGLE_VARIANCE (3.14f/180/2)*(3.14f/180/2)
#define MIN_DOPPLER_VARIANCE (0.3298f/2)*(0.3298f/2)

#define NUM_POINTS_MAX			1000	
#define NUM_TRACKS_MAX			200	

// Tagging IDs definitions
#define POINT_NOT_ASSOCIATED	255
#define POINT_BEHIND_THE_WALL	254
#define POINT_TOO_WEAK			253

// Matrix can be viewed as an row-wise array OR addressed by individual element
typedef union {
	float a[9];
	struct
	{
		float e11; float e12; float e13;
		float e21; float e22; float e23;
		float e31; float e32; float e33;
	};
} MATRIX3x3; 

typedef union {
	float a[16];
	struct
	{
		float e11; float e12; float e13; float e14;
		float e21; float e22; float e23; float e24;
		float e31; float e32; float e33; float e34;
		float e41; float e42; float e43; float e44;
	};
} MATRIX4x4;

typedef union {
	float a[36];
	struct
	{
		float e11; float e12; float e13; float e14; float e15; float e16;
		float e21; float e22; float e23; float e24; float e25; float e26;
		float e31; float e32; float e33; float e34; float e35; float e36;
		float e41; float e42; float e43; float e44; float e45; float e46;
		float e51; float e52; float e53; float e54; float e55; float e56;
		float e61; float e62; float e63; float e64; float e65; float e66;
	};
} MATRIX6x6;


/**
 * @brief 
 *  GTRACK Unit Parameters structure
 *
 * @details
 *  The structure describes the parameters used to instantiate GTRACK unit instance
 */
typedef struct {
	
    /**  @brief  Track unit identifier */
	uint16_t tid;
    /**  @brief  State vector type to use */
	GTRACK_STATE_VECTOR_TYPE stateVectorType;
    /**  @brief  Verbosenes mask to use */
    uint64_t verbose;
    
    /**  @brief   Expected target radial velocity at the moment of detection, m/s */
    float initialRadialVelocity;
    /**  @brief  maximum unambigiuos radial velocity */
    float maxURadialVelocity;
    /**  @brief  maximum target acceleration */
    float maxAcceleration;
    /**  @brief  frame rate */
    float deltaT;

    /**  @brief  Gating parameters */    
     GatingParams gatingParams;
     /**  @brief  Scoring parameters */   
     ScoringParams scoringParams;
     /**  @brief  Allocation parameters */
     AllocationParams allocationParams;
     /**  @brief  Unrolling parameters */
     UnrollingParams unrollingParams;
     /**  @brief  State parameters */
     StateParams stateParams;
     /**  @brief  Variation parameters */
     VarParams   variationParams;    
     /**  @brief  Scenery parameters */
     SceneryParams   sceneryParams;
   
    /**  @brief  2D Transition Matrix */
	float F4[4*4];
    /**  @brief  2DA Transition Matrix */
	float F6[6*6];
    /**  @brief  2D Process Noise Matrix */
	float Q4[4*4];
    /**  @brief  2DA Process Noise Matrix */
	float Q6[6*6];

} TrackingParams;

/**
 * @brief 
 *  GTRACK Unit Verbosity definitions
 *
 * @details
 *  The structure describes masks used for output verbosity
 */

/**  @brief Report Errors */
#define VERBOSE_ERROR_INFO 1
/**  @brief Report Warnings */
#define VERBOSE_WARNING_INFO 2
/**  @brief Report Debuging information */
#define VERBOSE_DEBUG_INFO 4
/**  @brief Report all Matrix math */
#define VERBOSE_MATRIX_INFO 8
/**  @brief Report velocity unrolling data */
#define VERBOSE_UNROLL_INFO 16
/**  @brief Report state transitions */
#define VERBOSE_STATE_INFO 32
/**  @brief Report association data */
#define VERBOSE_ASSOSIATION_INFO 64
/**  @brief Report gating in XY space */
#define VERBOSE_GATEXY_INFO 128
/**  @brief Report gating in range/angle space */
#define VERBOSE_GATERA_INFO 256
/**  @brief Report unitary gating */
#define VERBOSE_GATEG1_INFO 512

/**
 * @brief 
 *  GTRACK Unit State
 *
 * @details
 *  The structure describes GTRACK unit states
 */
typedef enum {
    /**  @brief Free (not allocated) */
    TRACK_STATE_FREE = 0,
    /**  @brief INIT */
    TRACK_STATE_INIT,
    /**  @brief DETECTION State */
    TRACK_STATE_DETECTION,
    /**  @brief ACTIVE State */
    TRACK_STATE_ACTIVE
} TrackState;

/**
 * @brief 
 *  GTRACK Unit Velocity Handling State
 *
 * @details
 *  The structure describes GTRACK velocity handling states
 */
typedef enum {
    /**  @brief INIT */
    VELOCITY_INIT = 0,
    /**  @brief Range Rate filtering */
    VELOCITY_RATE_FILTER,
    /**  @brief Stabilizing Velocity Estimation */
    VELOCITY_TRACKING,
    /**  @brief Locked */
    VELOCITY_LOCKED
} VelocityHandlingState;

/**
 * @brief 
 *  GTRACK Unit instance structure
 *
 * @details
 *  The structure describes the individual GTRACK unit instance
 */

typedef struct
{
    /**  @brief Tracking Unit identifier */
    uint16_t	tid;
    /**  @brief Target Object identifier */
    uint32_t	vid;

    /**  @brief TimeStamp */
    uint64_t	heartBeatCount;
    /**  @brief Allocation Time */
    uint64_t	allocationTime;
    /**  @brief Allocation Range */
	float		allocationRange;
    /**  @brief Allocation Radial Velocity */
	float		allocationVelocity;
    /**  @brief Number of Associated Points */
    uint16_t	associatedPoints;

    /**  @brief Current State */
    TrackState	state;
    /**  @brief Requested State Vector type */
	GTRACK_STATE_VECTOR_TYPE	stateVectorType;
    /**  @brief Current State Vector type */
	GTRACK_STATE_VECTOR_TYPE	currentStateVectorType;
    /**  @brief Length of State Vector */
    uint16_t	stateVectorLength;
    /**  @brief Length of Measurement Vector */
    uint16_t	measurementVectorLength;
    /**  @brief veboseness Mask */
	uint64_t	verbose;
    
    /**  @brief Gating Parameters */
	GatingParams		*gatingParams;
    /**  @brief Scoring Parameters */
	ScoringParams		*scoringParams;
    /**  @brief State Changing Parameters */
	StateParams	        *stateParams;
    /**  @brief Allocation Parameters */
	AllocationParams	*allocationParams;
    /**  @brief Unrolling Parameters */
	UnrollingParams		*unrollingParams;
    /**  @brief Measurement Variation Parameters */
	VarParams           *variationParams;
    /**  @brief Scenery Parameters */
    SceneryParams       *sceneryParams;
	
    /**  @brief Current velocity handling State */
	VelocityHandlingState	velocityHandling;
	
    /**  @brief   Expected target radial velocity at the moment of detection, m/s */
    float initialRadialVelocity;	
    /**  @brief Configured maximum unambigious radial velocity */
	float		maxURadialVelocity;
    /**  @brief Current Range Rate value*/
    float		rangeRate;

    /**  @brief Detection state count to active */
    uint16_t	detect2activeCount;
    /**  @brief Detection state count to free */
    uint16_t	detect2freeCount;
    /**  @brief Active state count to free */
    uint16_t	active2freeCount;

	/**  @brief Configured target maximum acceleration */
	float		maxAcceleration;
	/**  @brief Current Process Variance */
	float		processVariance;
	/**  @brief Configured Frame rate */
    float		dt;

	/**  @brief Pointer to 2D Transition matrix */
    float		*F4;
	/**  @brief Pointer to 2DA Transition matrix */
    float		*F6;
	/**  @brief Pointer to 2D Process Noise matrix */
    float		*Q4;
	/**  @brief Pointer to 2DA Process Noise matrix */
    float		*Q6;

	/**  @brief Pointer to current Transition matrix */
    float		*F;
	/**  @brief Pointer to current Process Noise matrix */
    float		*Q;

	/**  @brief State matrix, estimated */
    float		S_hat[6];
	/**  @brief State matrix, predicted */
    float		S_apriori_hat[6];
	/**  @brief Process matrix, estimated */
    float		P_hat[36];
	/**  @brief Process matrix, predicted */
    float		P_apriori_hat[36];
	/**  @brief Expected Measurment matrix */
    float		H_s[3];
	/**  @brief Group Dispersion matrix */
    float		gD[9];
	/**  @brief Group Member Covariance matrix (between a member in measurment group and group centroid) */
    float		gC[9];
	/**  @brief Inverse of Group Covariance matrix */
    float		gC_inv[9];
	/**  @brief Gain used in association function */
	float		G;

} GtrackUnitInstance;

/**
 * @brief 
 *  GTRACK Module instance structure
 *
 * @details
 *  The structure describes the GTRACK module instance
 */
typedef struct {

	/**  @brief Maximum number of measurement points per frame */
	uint16_t maxNumPoints;
	/**  @brief Maximum number of Tracking objects */
	uint16_t maxNumTracks;
	/**  @brief Tracking Unit Parameters  */
	TrackingParams params;

	/**  @brief TimeStamp  */
	uint64_t	heartBeat;
    /**  @brief verboseness Mask */
	uint64_t	verbose;
	/**  @brief Array of best scores  */
	float		*bestScore;
	/**  @brief Array of best score authors (TIDs)  */
	uint8_t	    *bestIndex;
	/**  @brief Temporary Array of measurement indices for set-under-construction */
	uint16_t	*allocIndex;
	/**  @brief Array of Tracking Unit handles */
	void		**hTrack;
	/**  @brief List of currently active Tracking Units */
	GTrack_ListObj activeList;
	/**  @brief List of Free Tracking Units TIDs*/
	GTrack_ListObj freeList;
	/**  @brief Array of TID elements */
	GTrack_ListElem *tidElem;

	/**  @brief Array of Target descriptors */
	GTRACK_targetDesc	*targetDesc;
	/**  @brief Array of Target indices */
	uint16_t			*targetInd;
	/**  @brief Number of currently tracked Targets */
	uint16_t			targetNum;

} GtrackModuleInstance;

void *gtrack_unitCreate(TrackingParams *params);
void gtrack_unitDelete(void *handle);
void gtrack_unitStart(void *handle, uint64_t timeStamp, float *um);
void gtrack_unitStop(void *handle);
void gtrack_unitPredict(void *handle);
TrackState gtrack_unitUpdate(void *handle, GTRACK_measurementPoint *point, GTRACK_measurementVariance *var, uint8_t *pInd, uint16_t num);
void gtrack_unitScore(void *handle, GTRACK_measurementPoint *point, float *bestScore, uint8_t *bestInd, uint16_t num);
void gtrack_unitEvent(void *handle, uint16_t num);
void gtrack_unitReport(void *handle, GTRACK_targetDesc *target);

void gtrack_velocityStateHandling(void *handle, float *um);
float gtrack_unrollRadialVelocity(float rvMax, float rvExp, float rvIn);
void gtrack_spherical2cartesian(GTRACK_STATE_VECTOR_TYPE format, float *sph, float *cart);
void gtrack_cartesian2spherical(GTRACK_STATE_VECTOR_TYPE format, float *cart, float *sph);
void gtrack_computeJacobian(GTRACK_STATE_VECTOR_TYPE format, float *cart, float *jac);
float gtrack_gateCreateLim (float volume, float *EC, float range, float *gateLim);
void gtrack_computeMahalanobis3(float *d, float *S, float *chi2);

void gtrack_matrixMultiply(uint16_t m1, uint16_t m2, uint16_t m3, float *A, float *B, float *C);
void gtrack_matrixComputeCov(float *J, float *P, float *R, float *PJ, float *C);
void gtrack_matrixMultiply66(float *A, float *B, float *C);
void gtrack_matrixMultiply66M(float *A, float *B, float *C);
void gtrack_matrixMultiply66T(float *A, float *B, float *C);
void gtrack_matrixComputePJT(float *P, float *J, float *PJ);
void gtrack_matrixTransposeMultiply(uint16_t rows, uint16_t m, uint16_t cols, float *A, float *B, float *C);
void gtrack_matrixScallerMultiply(uint16_t m1, uint16_t m2, float *A, float C, float *B);
void gtrack_matrixAdd(uint16_t m1, uint16_t m2, float *A, float *B, float *C);
void gtrack_matrixSub(uint16_t m1, uint16_t m2, float *A, float *B, float *C);
void gtrack_matrixDet3(float *A, float *det);
void gtrack_matrixInv3(float *A, float *Ainv);
void gtrack_matrixMakeSymmetrical(uint16_t m, float *A, float *B);

void gtrack_matrixCholesky3(float *A, float *G);

void gtrack_matrixPrint(uint16_t m1, uint16_t m2, float *A);
void gtrack_matrixPrint2(uint16_t rows, uint16_t cols, float *A, float *B);

#endif /* GTRACK_INT_LIB_H__ */
