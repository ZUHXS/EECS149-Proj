/**
 *  @file   types.h
 *  @brief  Contains type definitions for portable types. Refer to Coding Guidelines document.
 *
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
 #ifndef _TYPES_C66_H
#define _TYPES_C66_H

/**
 *  @file   c66\types.h
 *  @brief  Contains type definitions for c66x target.
 *
 * @p 
 * (C) Copyright, 2013 Texas Instruments, Inc.
 * @p 
 */

#ifdef __cplusplus
extern "C" {
#endif

/* Include stdint.h */   
#include <stdint.h>

#ifndef false
#define false 0
#endif

#ifndef true
#define true 1
#endif

#ifndef PASS
#define PASS 0
#endif

#ifndef FAIL
#define FAIL 1
#endif

/*! signed minimum 8-bit complex number.
 *
 * Make real always MSW and imag always LSW 
 */
typedef struct _cplx_least8_t_ {
#if defined(_LITTLE_ENDIAN)
  int_least8_t imag;
  int_least8_t real;
  
#else
  int_least8_t real;
  int_least8_t imag;
#endif
} cplx_least8_t;

/*! signed minimum 16-bit complex number.
 *
 * Make real always MSW and imag always LSW 
 */
typedef struct _cplx_least16_t_ {
#if defined(_LITTLE_ENDIAN)
  int_least16_t imag;
  int_least16_t real;
  
#else
  int_least16_t real;
  int_least16_t imag;
#endif
} cplx_least16_t;

/*! signed minimum 32-bit complex number.
 *
 * Make real always MSW and imag always LSW 
 */
typedef struct _cplx_least32_t_ {
#if defined(_LITTLE_ENDIAN)
  int_least32_t imag;
  int_least32_t real;
  
#else
  int_least32_t real;
  int_least32_t imag;
#endif
} cplx_least32_t;

/*! signed minimum 40-bit complex number.
 *
 * Make real always MSW and imag always LSW 
 */
typedef struct _cplx_least40_t_ {
#if defined(_LITTLE_ENDIAN)
  int_least40_t imag;
  int_least40_t real;
  
#else
  int_least40_t real;
  int_least40_t imag;
#endif
} cplx_least40_t;

/*! signed minimum 64-bit complex number.
 *
 * Make real always MSW and imag always LSW 
 */
typedef struct _cplx_least64_t_ {
#if defined(_LITTLE_ENDIAN)
  int_least64_t imag;
  int_least64_t real;
  
#else
  int_least64_t real;
  int_least64_t imag;
#endif
} cplx_least64_t;

/*! signed exact 8-bit complex number.
 *
 * Make real always MSW and imag always LSW 
 */
typedef struct _cplx8_t_ {
#if defined(_LITTLE_ENDIAN)
  int8_t imag;
  int8_t real;
  
#else
  int8_t real;
  int8_t imag;
#endif
} cplx8_t;

/*! signed exact 16-bit complex number.
 *
 * Make real always MSW and imag always LSW 
 */
typedef struct _cplx16_t_ {
#if defined(_LITTLE_ENDIAN)
  int16_t imag;
  int16_t real;
  
#else
  int16_t real;
  int16_t imag;
#endif
} cplx16_t;

/*! signed exact 32-bit complex number.
 *
 * Make real always MSW and imag always LSW 
 */
typedef struct _cplx32_t_ {
#if defined(_LITTLE_ENDIAN)
  int32_t imag;
  int32_t real;
  
#else
  int32_t real;
  int32_t imag;
#endif
} cplx32_t;

/*! signed exact 40-bit complex number.
 *
 * Make real always MSW and imag always LSW 
 */
typedef struct _cplx40_t_ {
#if defined(_LITTLE_ENDIAN)
  int40_t imag;
  int40_t real;
 
#else
  int40_t real;
  int40_t imag;
#endif
} cplx40_t;

/*! signed exact 64-bit complex number.
 *
 * Make real always MSW and imag always LSW 
 */
typedef struct _cplx64_t_ {
#if defined(_LITTLE_ENDIAN)
  int64_t imag;
  int64_t real;

#else
  int64_t real;
  int64_t imag;
#endif
} cplx64_t;

/*! floating pont complex number.
 *
 * Make real always MSW and imag always LSW 
 */
typedef struct _cplxf_t_ {
#if defined(_LITTLE_ENDIAN)
  float imag;
  float real;
  
#else
  float real;
  float imag;
#endif
} cplxf_t;

#define Real(xy) (xy.real)
#define Imag(xy) (xy.imag)
#define mkCplx(x,y, xy) { xy.real = x; xy.imag = y; }

#define LReal(xy) (xy.real)
#define LImag(xy) (xy.imag)
#define mkLCplx(x,y, xy) { xy.real = x; xy.imag = y; }

#if defined(_LITTLE_ENDIAN)
 #ifndef BIG_ENDIAN
  #define BIG_ENDIAN
 #endif
#else
 #ifndef LITTLE_ENDIAN
  #define LITTLE_ENDIAN
 #endif
#endif

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _TYPES_C66_H */

/* nothing past this point */
