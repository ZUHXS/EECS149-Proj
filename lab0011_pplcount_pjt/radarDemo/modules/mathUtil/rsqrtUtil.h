#ifndef _RSQRTUTIL_H
#define _RSQRTUTIL_H

/**
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 *  @file   recipUtil.h
 *  @brief  interface definitions for various reciprocal modules
 *          Functions in this file do not perform any any input
 *          range checking.
 *
 */

#ifdef __cplusplus
#define _extern extern "C"
#define _externfar extern "C" far
#else
#define _extern extern
#define _externfar extern far
#endif

// **************************************************************************
// the includes
#ifdef __TI_COMPILER_VERSION__
#include "c6x.h"
#else
#include "common/c6xsim/C6xSimulator.h"
#endif

// **************************************************************************
// the defines


/* ======================================================================= */
/* rsqrtsp_i.h - single precision floating point reciprocal sqrt           */
/*              optimized inlined C implementation (w/ intrinsics)         */
/* ======================================================================= */

static inline float rsqrt_sp(float a)
{
  float Half  =  0.5;
  float OneP5 =  1.5;
  float Small =  1.17549435e-38;
  float X0, X1, X2, X3;

  X0 = _rsqrsp(a);
  X1 = a * X0;
  X3 = OneP5 - (X1*X0*Half);
  X1 = X0 * X3;
  X2 = X1 * (OneP5  - (a*X1*X1*Half));

  if (a < Small) {
    X2 = _itof(0x7F800000);
  }

  return (X2);
}



#endif // _RSQRTUTIL_H

// nothing past this point
