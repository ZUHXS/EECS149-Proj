#ifndef _RECIPUTIL_H
#define _RECIPUTIL_H

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


// **************************************************************************
// the typedefs

//! \brief      finds reciprocal to full single precsion (23 bits) for single float.
//!             performs two Newton-Raphson iterations. Input must be within range
//!             of single precision floating point output after reciprocal. 
//!             No checking on validity of input done here.
//! \param[in]  a          input
//! \return     reciprocal of a
static inline float recipsp_1X_FP (float a)
{
  float TWO   =  2.0;
  float X;

  X = _rcpsp(a);
  X = X  * (TWO - a*X);
  X = X  * (TWO - a*X); 

  return (X);
}

//! \brief      finds reciprocal to half precsion (16 bits) for single float.
//!             performs one Newton-Raphson iteration. Input must be within range
//!             of single precision floating point output after reciprocal. 
//!             No checking on validity of input done here.
//! \param[in]  a          input
//! \return     reciprocal of a
static inline float recipsp_1X_HP (float a)
{
  float TWO   =  2.0;
  float X;

  X = _rcpsp(a);
  X = X  * (TWO - a*X);

  return (X);
}

//! \brief      finds reciprocal to full single precsion (23 bits) for two float values.
//!             performs two Newton-Raphson iteration. Input must be within range
//!             of single precision floating point output after reciprocal. 
//!             No checking on validity of input done here.
//! \param[in]  a          dual input
//! \return     reciprocal of a[0..1]
static inline __float2_t recipsp_2X_FP (__float2_t a)
{
	__float2_t TWO   =  _ftof2(2.0,2.0);
	__float2_t X;

  X = _ftof2(_rcpsp(_hif2(a)),_rcpsp(_lof2(a)));
  X = _dmpysp(X, _dsubsp(TWO, _dmpysp(a, X)));
  X = _dmpysp(X, _dsubsp(TWO, _dmpysp(a, X)));

  return (X);
}

//! \brief      finds reciprocal to half precsion (16 bits) for two float values.
//!             performs one Newton-Raphson iterations. Input must be within range
//!             of single precision floating point output after reciprocal. 
//!             No checking on validity of input done here.
//! \param[in]  a          dual input
//! \return     reciprocal of a[0..1]
static inline __float2_t recipsp_2X_HP (__float2_t a)
{
	__float2_t TWO   =  _ftof2(2.0,2.0);
	__float2_t X;

  X = _ftof2(_rcpsp(_hif2(a)),_rcpsp(_lof2(a)));
  X = _dmpysp(X, _dsubsp(TWO, _dmpysp(a, X)));

  return (X);
}

//! \brief      finds reciprocal to full single precsion (23 bits) for four float values.
//!             performs two Newton-Raphson iteration. Input must be within range
//!             of single precision floating point output after reciprocal. 
//!             No checking on validity of input done here.
//! \param[in]  a          quad input
//! \return     reciprocal of a[0..3]
//!
//! \rem        current compiler does not appear to inline __x128_t based functions
static inline __x128_t recipsp_4X_FP (__x128_t a)
{
  __float2_t TWO   = _ftof2(2.0, 2.0);
  __x128_t X, Y;

  X = _fto128(_rcpsp(_get32f_128(a, 3)),
		      _rcpsp(_get32f_128(a, 2)),
			  _rcpsp(_get32f_128(a, 1)),
			  _rcpsp(_get32f_128(a, 0)));
  Y = _qmpysp(a, X);
  Y = _f2to128(_dsubsp(TWO,_hif2_128(Y)),_dsubsp(TWO,_lof2_128(Y)));
  X = _qmpysp(X, Y);
  Y = _qmpysp(a, X);
  Y = _f2to128(_dsubsp(TWO,_hif2_128(Y)),_dsubsp(TWO,_lof2_128(Y)));
  X = _qmpysp(X, Y);

  return (X); 
}

//! \brief      finds reciprocal to half precsion (16 bits) for four float values.
//!             performs one Newton-Raphson iterations. Input must be within range
//!             of single precision floating point output after reciprocal. 
//!             No checking on validity of input done here.
//! \param[in]  a          quad input
//! \return     reciprocal of a[0..3]
//!
//! \rem        current compiler does not apper to inline __x128_t based functions
static inline __x128_t recipsp_4X_HP (__x128_t a)
{
  __float2_t TWO   = _ftof2(2.0, 2.0);
  __x128_t X, Y;

  X = _fto128(_rcpsp(_get32f_128(a, 3)),
		      _rcpsp(_get32f_128(a, 2)),
			  _rcpsp(_get32f_128(a, 1)),
			  _rcpsp(_get32f_128(a, 0)));
  Y = _qmpysp(a, X);
  Y = _f2to128(_dsubsp(TWO,_hif2_128(Y)),_dsubsp(TWO,_lof2_128(Y)));
  X = _qmpysp(X, Y);

  return (X); 
}

#endif // _RECIPUTIL_H

// nothing past this point
