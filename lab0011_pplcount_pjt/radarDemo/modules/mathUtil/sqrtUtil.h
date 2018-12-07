#ifndef _SQRTUTIL_H
#define _SQRTUTIL_H

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
 *  @file   sqrtUtil.h
 *  @brief  interface definitions for various square root modules
 *          Functions in this file do not perform any any input
 *          range checking. sqrt is performed by first calculating
 *          1/sqrt() and hence input value of zero may result in
 *          indeterminate value
 *
 */
// **************************************************************************
// the includes
#ifdef __TI_COMPILER_VERSION__
#include "c6x.h"
#else
#include "common/c6xsim/C6xSimulator.h"
#endif

// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief      finds square root to full single precsion (23 bits) for single float.
//!             performs two Newton-Raphson iterations. Input must be within range
//!             of single precision floating point output after reciprocal. 
//!             No checking on validity of input done here.
//! \param[in]  a          input
//! \return     square root of a
static inline float sqrtsp_1X_FP (float a)
{
  float  Half  =  0.5;
  float  OneP5 =  1.5;
  float  X;

  X = _rsqrsp(a);

  X = X*(OneP5 - (a*X*X*Half));
  X = X*(OneP5 - (a*X*X*Half));
  X = a*X;

  return (X);
}

//! \brief      finds square root to hakf single precsion (16 bits) for single float.
//!             performs one Newton-Raphson iterations. Input must be within range
//!             of single precision floating point output after reciprocal. 
//!             No checking on validity of input done here.
//! \param[in]  a          input
//! \return     square root of a
static inline float sqrtsp_1X_HP (float a)
{
  float  Half  =  0.5;
  float  OneP5 =  1.5;
  float  X;

  X = _rsqrsp(a);

  X = X*(OneP5 - (a*X*X*Half));
  X = a*X;

  return (X);
}

//! \brief      finds square root to full single precsion (23 bits) for two float values.
//!             performs two Newton-Raphson iterations. Input must be within range
//!             of single precision floating point output after reciprocal. 
//!             No checking on validity of input done here.
//! \param[in]  a          dual input
//! \return     reciprocal of a[0..1]
static inline __float2_t sqrtsp_2X_FP (__float2_t a)
{
  __float2_t  Half  =  _ftof2(0.5, 0.5);
  __float2_t  OneP5 =  _ftof2(1.5, 1.5);
  __float2_t  X, Y, Z;

  X = _ftof2(_rsqrsp(_hif2(a)),_rsqrsp(_lof2(a)));
  Z = _dmpysp(Half, a);
  Y = _dmpysp(Z, X);
  Y = _dmpysp(Y, X);
  X = _dmpysp(_dsubsp(OneP5,Y), X);
  Y = _dmpysp(Z, X);
  Y = _dmpysp(Y, X);
  X = _dmpysp(_dsubsp(OneP5,Y), X);
  X = _dmpysp(a, X);

  return (X);
}

//! \brief      finds square root to hakf single precsion (16 bits) for two float values.
//!             performs one Newton-Raphson iterations. Input must be within range
//!             of single precision floating point output after reciprocal. 
//!             No checking on validity of input done here.
//! \param[in]  a          dual input
//! \return     reciprocal of a[0..1]
static inline __float2_t sqrtsp_2X_HP (__float2_t a)
{
  __float2_t  Half  =  _ftof2(0.5, 0.5);
  __float2_t  OneP5 =  _ftof2(1.5, 1.5);
  __float2_t  X, Y, Z;

  X = _ftof2(_rsqrsp(_hif2(a)),_rsqrsp(_lof2(a)));
  Z = _dmpysp(Half, a);
  Y = _dmpysp(Z, X);
  Y = _dmpysp(Y, X);
  X = _dmpysp(_dsubsp(OneP5,Y), X);
  X = _dmpysp(a, X);

  return (X);
}

//! \brief      finds square root to full single precsion (23 bits) for four float values.
//!             performs two Newton-Raphson iterations. Input must be within range
//!             of single precision floating point output after reciprocal. 
//!             No checking on validity of input done here.
//! \param[in]  a          quad input
//! \return     reciprocal of a[0..3]
//!
//! \rem        current compiler does not appear to inline __x128_t based functions
static inline __x128_t sqrtsp_4X_FP (__x128_t a)
{
  __x128_t  Half  =  _fto128(0.5, 0.5, 0.5, 0.5);
  __x128_t  OneP5 =  _fto128(1.5, 1.5, 1.5, 1.5);
  __x128_t  X, Y, Z;

  X = _fto128(_rsqrsp(_get32f_128(a, 3)),
		      _rsqrsp(_get32f_128(a, 2)),
			  _rsqrsp(_get32f_128(a, 1)),
			  _rsqrsp(_get32f_128(a, 0)));
  Z = _qmpysp(Half, a);
  Y = _qmpysp(Z, X);
  Y = _qmpysp(Y, X);
  X = _qmpysp(_f2to128(_dsubsp(_hif2_128(OneP5),_hif2_128(Y)), _dsubsp(_lof2_128(OneP5),_lof2_128(Y))), X);
  Y = _qmpysp(Z, X);
  Y = _qmpysp(Y, X);
  X = _qmpysp(_f2to128(_dsubsp(_hif2_128(OneP5),_hif2_128(Y)), _dsubsp(_lof2_128(OneP5),_lof2_128(Y))), X);
  X = _qmpysp(a, X);

  return (X);
}

//! \brief      finds square root to hakf single precsion (16 bits) for four float values.
//!             performs one Newton-Raphson iterations. Input must be within range
//!             of single precision floating point output after reciprocal. 
//!             No checking on validity of input done here.
//! \param[in]  a          quad input
//! \return     reciprocal of a[0..3]
//!
//! \rem        current compiler does not appear to inline __x128_t based functions
static inline __x128_t sqrtsp_4X_HP (__x128_t a)
{
  __x128_t  Half  =  _fto128(0.5, 0.5, 0.5, 0.5);
  __x128_t  OneP5 =  _fto128(1.5, 1.5, 1.5, 1.5);
  __x128_t  X, Y, Z;

  X = _fto128(_rsqrsp(_get32f_128(a, 3)),
		      _rsqrsp(_get32f_128(a, 2)),
			  _rsqrsp(_get32f_128(a, 1)),
			  _rsqrsp(_get32f_128(a, 0)));
  Z = _qmpysp(Half, a);
  Y = _qmpysp(Z, X);
  Y = _qmpysp(Y, X);
  X = _qmpysp(_f2to128(_dsubsp(_hif2_128(OneP5),_hif2_128(Y)), _dsubsp(_lof2_128(OneP5),_lof2_128(Y))), X);
  X = _qmpysp(a, X);

  return (X);
}

#endif // _SQRTUTIL_H

// nothing past this point
