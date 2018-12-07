/**
 *   @file  gtrack.h
 *
 *   @brief
 *      This is the header file for the GTRACK Algorithm
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

/** @mainpage gtrack
 *
 *  ## GTRACK Algorithm
 
 *	This code is an implementation of Group TRACKing algorithm
 *
 *	The algorithm is designed to track multiple targets, where each target is represented by a set of measurement points. 
 *	Each measurement point carries detection informations, for example, range, angle, and radial velocity. 
 *	Instead of tracking individual reflections, the algorithm predicts and updates the location and dispersion properties of the group. 
 *	The group is defined as the set of measurements (typically, few tens; sometimes few hundreds) associated with a real life target.
 *
 *	- Algorithm inputs the Point Cloud. For example, thousands of individual measurements (reflection points).
 *	- Algorithm outputs a Target List. For example, an array of hundreds of targets.
 *
 *	- Algorithm uses extended Kalman Filter to model target motion in Cartesian coordinates.
 *	- Algorithm supports constant velocity and constant acceleartion models.
 *	- Algorithm uses 3D Mahalanobis distances as gating function and Max-likelihood criterias as scoring function to associate points with an existing track.
 *
 *  The GTRACK header file should be included in an application as follows:
 *  @code
    #include <ti/alg/gtrack.h>
 *  @endcode
 *
 *  ## Architecture of the GTRACK Algorithm

 *	Algorithm is implemented with two software sublayers: MODULE and UNIT(s).
 *
 *	- Application interfaces with MODULE level only. It can Create, Run, and Delete the MODULE.
 *	- MODULE API is the external GTRACK API.
 *	- MODULE creates and manages multiple UNITs using UNIT level APIs. Each UNIT represents a single tracking object.
 *	- UNIT API is the internal API, used by MODULE.
 *	- All resources are allocated at MODULE creation time.
 */
#ifndef GTRACK_H
#define GTRACK_H

/** @defgroup GTRACK_ALG      GTRACK Algorithm
 */
#include <stdint.h>

/**
@defgroup GTRACK_ALG_EXTERNAL_FUNCTION            GTRACK External Functions
@ingroup GTRACK_ALG
@brief
*	Exported API which the Applications need to invoke in order to use the GTRACK Algorithm.
*	The Application interfaces the algorithm at Module layer. 
*/
/**
@defgroup GTRACK_ALG_EXTERNAL_DATA_STRUCTURE      GTRACK External Data Structures
@ingroup GTRACK_ALG
@brief
*   Data structures which are exposed to the Application
*/
/**
@defgroup GTRACK_ALG_UNIT_FUNCTION            GTRACK Unit Level Functions
@ingroup GTRACK_ALG
@brief
*   Internal unit level API
*/
/**
@defgroup GTRACK_ALG_UTILITY_FUNCTION			GTRACK Utility Functions
@ingroup GTRACK_ALG
@brief
*   Utility functions which are used internally by the GTRACK module.
*/

/**
@defgroup GTRACK_ALG_MATH_FUNCTION			GTRACK Math Functions
@ingroup GTRACK_ALG
@brief
*   Matrix math functions which are used internally by the GTRACK module.
*/
/**
@defgroup GTRACK_ALG_INTERNAL_DATA_STRUCTURE      GTRACK Internal Data Structures
@ingroup GTRACK_ALG
@brief
*   Internal data structures used by the GTRACK module.
*/

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup GTRACK_ALG_EXTERNAL_DATA_STRUCTURE
 @{ */

/**
 * @brief 
 *  GTRACK State Vector
 *
 * @details
 *  Defines State vector options 
 *		2D, 2DA - Supported
 *		3D, 3DA - Not supported
 */
typedef enum
{
    /**  @brief   2D => (X,Y, Vx,Vy) */
	GTRACK_STATE_VECTORS_2D = 0,
    /**  @brief   2DA => (X,Y, Vx,Vy, Ax,Ay) */
	GTRACK_STATE_VECTORS_2DA,
    /**  @brief   3D => (X,Y,Z, Vx,Vy,Vz) */
	GTRACK_STATE_VECTORS_3D,
    /**  @brief   3DA => (X,Y,Z, Vx,Vy,Vz, Ax,Ay, Az) */
	GTRACK_STATE_VECTORS_3DA
} GTRACK_STATE_VECTOR_TYPE;

/**
 * @brief 
 *  GTRACK Verbose Level
 *
 * @details
 *  Defines Algorithm verboseness level 
 */
typedef enum
{
    /**  @brief   NONE */
	GTRACK_VERBOSE_NONE = 0,
    /**  @brief   ERROR */
	GTRACK_VERBOSE_ERROR,
    /**  @brief   WARNING */
	GTRACK_VERBOSE_WARNING,
    /**  @brief   DEBUG */
	GTRACK_VERBOSE_DEBUG,
    /**  @brief   MATRIX */
	GTRACK_VERBOSE_MATRIX,
    /**  @brief   MAXIMUM */
	GTRACK_VERBOSE_MAXIMUM
} GTRACK_VERBOSE_TYPE;


/**
 * @brief 
 *  GTRACK Scene Parameters
 *
 * @details
 *  The structure describes the scene parameters that can be used by tracker to refine allocation/association and management functions
 */
typedef struct {
    /**  @brief position of the left wall, in meters, set to -100 if no wall */
    float leftWall;
    /**  @brief position of the right wall, in meters, set to +100 if no wall */
    float rightWall;
    /**  @brief entrance area lower boundary, in meters; set to 0 if not defined */
    float lowerEntrance;
    /**  @brief entrance area upper boundary, in meters; set to 100 if not defined */
    float upperEntrance;
} SceneryParams;

/**
 * @brief
 *  GTRACK Gate Limits
 *
 * @details
 *  The structure describes the limits the gating function will expand
 */
// Parameters Definitions
typedef struct {
    /**  @brief   Length limit, m */
	float length;
    /**  @brief   Width limit, m */
	float width;
    /**  @brief   Radial velocity limit, m/s */
	float vel;
} GateLimits; 

/**
 * @brief 
 *  GTRACK Gating Function Parameters
 *
 * @details
 *  The structure describes gating function parameters
 */
typedef struct {
    /**  @brief   Volume of the gating function */
	float		volume;
    /**  @brief   Gating function limits */
	GateLimits	limits;
} GatingParams;

/**
 * @brief 
 *  GTRACK Tracking Management Function Parameters
 *
 * @details
 *  The structure describes state changing thresholds
 */
typedef struct {
    /**  @brief  DETECTION => ACTIVE threshold */
	uint16_t det2actThre;
    /**  @brief  DETECTION => FREE threshold */
    uint16_t det2freeThre;

    /**  @brief  ACTIVE => FREE threshold */
    uint16_t active2freeThre;
    /**  @brief  STATIC => FREE threshold */
    uint16_t static2freeThre;
    /**  @brief  EXIT ZONE => FREE threshold */
	uint16_t exit2freeThre;
} StateParams;

/**
 * @brief 
 *  GTRACK Update Function Parameters
 *
 * @details
 *  The structure describes state default standard deviation values applied when no variance information provided in the point Cloud
 */
typedef struct {
    /**  @brief Expected standard deviation of measurements in target length dimension*/
    float lengthStd;
    /**  @brief Expected standard deviation of measurements in target width dimension*/
    float widthStd;
    /**  @brief Expected standard deviation of measurements of target radial velocity */
    float dopplerStd;
} VarParams;

/**
 * @brief
 *  GTRACK Scoring Function Parameters
 *
 * @details
 *  The structure describes the dispersion factor
 */
typedef struct {
    /**  @brief  Dispersion factor is a multiplier to group covarience matrix determinant */
	float	dispersionFactor;
} ScoringParams;

/**
 * @brief 
 *  GTRACK Allocation Function Parameters
 *
 * @details
 *  The structure describes the thresholds used in Allocation function
 */
typedef struct {
    /**  @brief  Minimum total SNR */
    float snrThre;
    /**  @brief  Minimum initial velocity, m/s */
    float velocityThre;
    /**  @brief  Minimum number of points in a set */
	uint16_t pointsThre;
    /**  @brief  Maximum squared distance between points in a set */
    float	maxDistanceThre;
    /**  @brief  Maximum velocity delta between points in a set */
    float	maxVelThre;
} AllocationParams;

/**
 * @brief 
 *  GTRACK Unrolling Parameters
 *
 * @details
 *  The structure describes the filtering parameters used to switch unrolling states
 */

typedef struct {
    /**  @brief  Range rate filtering alpha */
	float alpha;
    /**  @brief  Range rate filtering confidence */
	float confidence;
} UnrollingParams;

typedef struct {
	/**  @brief  Pointer to gating parameters */
	GatingParams *gatingParams;
    /**  @brief  Pointer to scoring parameters */
	ScoringParams *scoringParams;
    /**  @brief  Pointer to allocation parameters */
	AllocationParams *allocationParams;
    /**  @brief  Pointer to unrolling parameters */
	UnrollingParams *unrollingParams;
    /**  @brief  Pointer to tracking state parameters */
	StateParams *stateParams;
    /**  @brief  Pointer to measurements variation parameters */
	VarParams   *variationParams;
    /**  @brief  Pointer to scenery parameters */
	SceneryParams *sceneryParams;
} GTRACK_advancedParameters;

/**
 * @brief 
 *  GTRACK Configuration
 *
 * @details
 *  The structure describes the GTRACK algorithm configuration options. 
 */
typedef struct
{
    /**  @brief   State Vector Type */
    GTRACK_STATE_VECTOR_TYPE stateVectorType;
    /**  @brief   Verboseness Level */
    GTRACK_VERBOSE_TYPE verbose;
    /**  @brief   Maximum Number of Measurement Points per frame */
	uint16_t maxNumPoints;
    /**  @brief   Maximum Number of Tracking Objects */
	uint16_t maxNumTracks;

    /**  @brief   Expected target radial velocity at the moment of detection, m/s */
    float initialRadialVelocity;
    /**  @brief   Maximum unambiguous radial velocity, +/- m/s */
	float maxURadialVelocity;
    /**  @brief   Maximum expected target acceleration, m/s2 */
    float maxAcceleration;
    /**  @brief   Frame rate, ms */
    float deltaT;

    /**  @brief   Advanced parameters, set to NULL for defaults */
    GTRACK_advancedParameters *advParams;

} GTRACK_moduleConfig;

/**
 * @brief 
 *  GTRACK Measurement point
 *
 * @details
 *  The structure describes measurement point format
 */
typedef struct
{
    /**  @brief   Range, m */
	float range;
    /**  @brief   Angle, rad */
	float angle;
    /**  @brief   Radial velocity, m/s */
	float doppler;
	/**  @brief   Range detection SNR, linear */
    float snr;

} GTRACK_measurementPoint;

/**
 * @brief 
 *  GTRACK Measurement variances
 *
 * @details
 *  The structure describes measurement point variances
 */

typedef struct
{
    /**  @brief   Range measurement variance, m2 */
	float rangeVar;
    /**  @brief   Angle measurement variance, deg2 */
	float angleVar;
    /**  @brief   Radial Velocity measurement variance (m/s)2 */
	float dopplerVar;
} GTRACK_measurementVariance;

/**
 * @brief 
 *  GTRACK target descriptor
 *
 * @details
 *  The structure describes target descriptorformat
 */

typedef struct
{
	/**  @brief   Tracking Object Identifier */
	uint16_t tid;
	/**  @brief   Vehicle Object Identifier */
	uint32_t vid;
	/**  @brief   State vector 2D: (X, Y, Vx, Vy, Ax, Ay) */
	float S[6];
	/**  @brief   Group covariance matrix */
	float EC[9];
	/**  @brief   Gain factor*/
	float G;

} GTRACK_targetDesc;

extern void *gtrack_moduleCreate(GTRACK_moduleConfig *config);
extern void gtrack_moduleStep(void *handle, GTRACK_measurementPoint *point, GTRACK_measurementVariance *var, uint16_t mNum, GTRACK_targetDesc *t, uint16_t *tNum, uint8_t *mIndex, uint32_t *bench);
extern void gtrack_moduleDelete(void *handle);

// External dependencies
void *gtrack_alloc(unsigned int numElements, unsigned int sizeInBytes);
void gtrack_free(void *pFree, unsigned int sizeInBytes);

#ifdef _WIN32
#include <intrin.h>
static __inline uint32_t gtrack_getCycleCount(void){
  return (uint32_t)__rdtsc();
}
#endif

#ifdef SUBSYS_MSS
#if defined (__GNUC__) && !defined(__ti__)
static inline unsigned int gtrack_getCycleCount (void)
{
    unsigned int value;
    // Read CCNT Register
    asm volatile ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(value));
    return value;
}
#else
#define gtrack_getCycleCount() __MRC(15, 0, 9, 13, 0)
#endif
#endif

#ifdef SUBSYS_DSS
#include <c6x.h>
#define  gtrack_getCycleCount() TSCL
#endif

#ifdef _MEX_
	#define gtrack_log(level, format, ...) mexPrintf(format, ##__VA_ARGS__);
	extern int mexPrintf(const char *format, ...);
#else
	extern void gtrack_log(GTRACK_VERBOSE_TYPE level, const char *format, ...);
#endif



#ifdef __cplusplus
}
#endif

#endif /* GTRACK_H */
