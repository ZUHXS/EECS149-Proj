
/**
 *  @file   pc\types.h
 *  @brief  Contains type definitions for pc target.
 *          Refer to Coding Guidelines document.
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
#ifndef _TYPES_PC_H
#define _TYPES_PC_H

#ifdef __cplusplus
extern "C" {
#endif

#define __STDC_LIMIT_MACROS

/* C99 7.18.1.2 Minimum-width integer types */

    typedef   signed    char  int_least8_t;
    typedef unsigned    char uint_least8_t;
    typedef            short  int_least16_t;
    typedef unsigned   short uint_least16_t;
    typedef              int  int_least32_t;
    typedef unsigned     int uint_least32_t;
    typedef          __int64  int_least40_t;
    typedef unsigned __int64 uint_least40_t;
    typedef          __int64  int_least64_t;
    typedef unsigned __int64 uint_least64_t;

#if !defined(__cplusplus) || defined(__STDC_LIMIT_MACROS)
/*
   There is a defect report filed against the C99 standard concerning how 
   the (U)INTN_C macros should be implemented.  Please refer to --
   http://wwwold.dkuug.dk/JTC1/SC22/WG14/www/docs/dr_209.htm 
   for more information.  These macros are implemented according to the
   suggestion given at this web site.
*/

/*    #define  INT8_C(value)  ((int_least8_t)(value))
    #define UINT8_C(value)  ((uint_least8_t)(value))
    #define  INT16_C(value) ((int_least16_t)(value))
    #define UINT16_C(value) ((uint_least16_t)(value))
    #define  INT32_C(value) ((int_least32_t)(value))
    #define UINT32_C(value) ((uint_least32_t)(value))

    #define  INT40_C(value) ((int_least40_t)(value))
    #define UINT40_C(value) ((uint_least40_t)(value))

    #define  INT64_C(value) ((int_least64_t)(value))
    #define UINT64_C(value) ((uint_least64_t)(value))

    #define  INT_LEAST8_MAX   INT32_C(127)
    #define  INT_LEAST8_MIN   (-INT_LEAST8_MAX-1)
    #define UINT_LEAST8_MAX   UINT32_C(255)

    #define  INT_LEAST16_MAX  INT32_C(32767)
    #define  INT_LEAST16_MIN  (-INT_LEAST16_MAX-1)
    #define UINT_LEAST16_MAX  UINT32_C(65535)

    #define  INT_LEAST32_MAX  INT32_C(2147483647)
    #define  INT_LEAST32_MIN  (-INT_LEAST32_MAX-1)
    #define UINT_LEAST32_MAX  UINT32_C(4294967295U)

    #define  INT_LEAST64_MAX  INT64_C(9223372036854775807)
    #define  INT_LEAST64_MIN  (-INT_LEAST64_MAX-1)
    #define UINT_LEAST64_MAX  UINT64_C(18446744073709551615ULL)

    #define  INT_LEAST40_MAX   INT_LEAST64_MAX 
    #define  INT_LEAST40_MIN   INT_LEAST64_MIN
    #define UINT_LEAST40_MAX  UINT_LEAST64_MAX*/
#endif

/*! Actual number of bits for the portable (*least*) types.
 */
    #define  INT_LEAST8_BITS    8
    #define UINT_LEAST8_BITS    8
    #define  INT_LEAST16_BITS  16
    #define UINT_LEAST16_BITS  16
    #define  INT_LEAST32_BITS  32
    #define UINT_LEAST32_BITS  32
    #define  INT_LEAST40_BITS  64
    #define UINT_LEAST40_BITS  64
    #define  INT_LEAST64_BITS  64
    #define UINT_LEAST64_BITS  64

/* 7.18.1.1 Exact-width integer types */

#ifndef DISALLOW_EXACT_BITS_TYPES /* Should be defined for portable source projects */
    typedef   signed    char  int8_t;
    typedef unsigned    char uint8_t;
    typedef            short  int16_t;
    typedef unsigned   short uint16_t;
    typedef              int  int32_t;
    typedef unsigned     int uint32_t;
    typedef          __int64  int64_t;
    typedef unsigned __int64 uint64_t;

/* Note: (u)int40_t types do not exist on host but are defined below
 * when simulation of DSP like 64x is desired on host using intrinsics library.
 */
#ifdef HOST_SIMULATION
    typedef          __int64  int40_t;
    typedef unsigned __int64 uint40_t;
#endif

/*    #define  INT8_MAX   INT32_C(127)
    #define  INT8_MIN   (-INT8_MAX-1)
    #define UINT8_MAX   UINT32_C(255)

    #define  INT16_MAX  INT32_C(32767)
    #define  INT16_MIN  (-INT16_MAX-1)
    #define UINT16_MAX  UINT32_C(65535)

    #define  INT32_MAX  INT32_C(2147483647)
    #define  INT32_MIN  (-INT32_MAX-1)
    #define UINT32_MAX  UINT32_C(4294967295U)

    #define  INT64_MAX  INT64_C(9223372036854775807)
    #define  INT64_MIN  (-INT64_MAX-1)
    #define UINT64_MAX  UINT64_C(18446744073709551615ULL)*/

#ifdef HOST_SIMULATION
    #define  INT40_MAX   INT64_MAX 
    #define  INT40_MIN   INT64_MIN
    #define UINT40_MAX  UINT64_MAX
#endif

/* Note: (U)INTX_BITS is by definition X so below
   defines are same across all platforms and in principle are not imagly
   required but may be used to improve readability of the code that uses
   exact types 
 */
    #define  INT8_BITS    8
    #define UINT8_BITS    8
    #define  INT16_BITS  16
    #define UINT16_BITS  16
    #define  INT32_BITS  32
    #define UINT32_BITS  32
    #define  INT40_BITS  40
    #define UINT40_BITS  40
    #define  INT64_BITS  64
    #define UINT64_BITS  64

#endif /* DISALLOW_... */

/* 7.18.1.3 Fastest minimum-width integer types */
	/*
    typedef   signed    char  int_fast8_t;
    typedef unsigned    char uint_fast8_t;
    typedef            short  int_fast16_t;
    typedef unsigned   short uint_fast16_t;
    typedef              int  int_fast32_t;
    typedef unsigned     int uint_fast32_t;
    typedef          __int64  int_fast40_t;
    typedef unsigned __int64 uint_fast40_t;
    typedef          __int64  int_fast64_t;
    typedef unsigned __int64 uint_fast64_t;

    #define  INT_FAST8_MAX   INT32_C(127)
    #define  INT_FAST8_MIN   (-INT_FAST8_MAX-1)
    #define UINT_FAST8_MAX   UINT32_C(255)

    #define  INT_FAST16_MAX  INT32_C(32767)
    #define  INT_FAST16_MIN  (-INT_FAST16_MAX-1)
    #define UINT_FAST16_MAX  UINT32_C(65535)

    #define  INT_FAST32_MAX  INT32_C(2147483647)
    #define  INT_FAST32_MIN  (-INT_FAST32_MAX-1)
    #define UINT_FAST32_MAX  UINT32_C(4294967295U)

    #define  INT_FAST64_MAX  INT64_C(9223372036854775807)
    #define  INT_FAST64_MIN  (-INT_FAST64_MAX-1)
    #define UINT_FAST64_MAX  UINT64_C(18446744073709551615ULL)

    #define  INT_FAST40_MAX   INT_FAST64_MAX
    #define  INT_FAST40_MIN   INT_FAST64_MIN
    #define UINT_FAST40_MAX  UINT_FAST64_MAX
	*/
/*! Actual number of bits for the portable (*least*) types.
 */
	/*
    #define  INT_FAST8_BITS    8
    #define UINT_FAST8_BITS    8
    #define  INT_FAST16_BITS  16
    #define UINT_FAST16_BITS  16
    #define  INT_FAST32_BITS  32
    #define UINT_FAST32_BITS  32
    #define  INT_FAST40_BITS  64
    #define UINT_FAST40_BITS  64
    #define  INT_FAST64_BITS  64
    #define UINT_FAST64_BITS  64
	*/
/* 7.18.1.4 Integer types capable of holding object pointers */
    typedef          int intptr_t;
    typedef unsigned int uintptr_t;

/* 7.18.1.5 Greatest-width integer types */
    typedef          __int64 intmax_t;
    typedef unsigned __int64 uintmax_t;

/* 
   According to footnotes in the 1999 C standard, "C++ implementations
   should define these macros only when __STDC_LIMIT_MACROS is defined
   before <stdint.h> is included." 
*/
#if !defined(__cplusplus) || defined(__STDC_LIMIT_MACROS)

/* 7.18.2 Limits of specified width integer types */

/*    #define INTPTR_MAX   INT32_C(2147483647)
    #define INTPTR_MIN   (-INTPTR_MAX-1)
    #define UINTPTR_MAX  UINT32_C(4294967295U)

    #define INTMAX_MAX   INT64_C(9223372036854775807)
    #define INTMAX_MIN   (-INTMAX_MAX-1)
    #define UINTMAX_MAX  UINT64_C(18446744073709551615ULL)*/

/* 7.18.3 Limits of other integer types */

//    #define PTRDIFF_MAX INT32_C(2147483647)
//    #define PTRDIFF_MIN (-PTRDIFF_MAX-1)
#endif


//#define WCHAR_MAX 0xffffu
//#define WCHAR_MIN 0

/* wint_t not defined */


/* 7.18.4.2 Macros for greatest-width integer constants */

//    #define  INTMAX_C(value) ((intmax_t)(value))
//    #define UINTMAX_C(value) ((uintmax_t)(value))

typedef char            word_t;

#ifndef bool
typedef int_least8_t bool;
#endif

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


/*! Portable signed minimum 8-bit complex number.
 *
 * Make imag always MSW and real always LSW 
 */
typedef struct cplx_least8_t_ {
  int_least8_t real;
  int_least8_t imag;
} cplx_least8_t;

/*! Portable signed minimum 16-bit complex number.
 *
 * Make imag always MSW and real always LSW 
 */
typedef struct cplx_least16_t_ {
  int_least16_t real;
  int_least16_t imag;
} cplx_least16_t;


/*! Portable signed minimum 32-bit complex number.
 *
 * Make imag always MSW and real always LSW 
 */
typedef struct cplx_least32_t_ {
  int_least32_t real;
  int_least32_t imag;
} cplx_least32_t;

/*! Portable signed minimum 40-bit complex number.
 *
 * Make imag always MSW and real always LSW 
 */
typedef struct cplx_least40_t_ {
  int_least40_t real;
  int_least40_t imag;
} cplx_least40_t;

/*! Portable signed minimum 64-bit complex number.
 *
 * Make imag always MSW and real always LSW 
 */
typedef struct cplx_least64_t_ {
  int_least64_t real;
  int_least64_t imag;
} cplx_least64_t;

#ifndef DISALLOW_EXACT_BITS_TYPES /* Should be defined for portable source projects */

/*! Non portable signed exact 8-bit complex number.
 *
 * Make imag always MSW and real always LSW 
 */
typedef struct cplx8_t_ {
  int8_t real;
  int8_t imag;
} cplx8_t;

/*! Non portable signed exact 16-bit complex number.
 *
 * Make imag always MSW and real always LSW 
 */
typedef struct cplx16_t_ {
  int16_t real;
  int16_t imag;
} cplx16_t;

/*! Non portable signed exact 32-bit complex number.
 *
 * Make imag always MSW and real always LSW 
 */
typedef struct cplx32_t_ {
  int32_t real;
  int32_t imag;
} cplx32_t;

#ifdef HOST_SIMULATION
/*! Non portable signed exact 40-bit complex number.
 *
 * Make imag always MSW and real always LSW 
 */
typedef struct cplx40_t_ {
  int40_t real;
  int40_t imag;
} cplx40_t;
#endif

/*! Non portable signed exact 64-bit complex number.
 *
 * Make imag always MSW and real always LSW 
 */
typedef struct cplx64_t_ {
  int64_t real;
  int64_t imag;
} cplx64_t;

#endif /* DISALLOW... */

#define imag(xy) (xy.imag)
#define real(xy) (xy.real)
#define mkCplx(x,y, xy) { xy.imag = x; xy.real = y; }

#define Limag(xy) (xy.imag)
#define Lreal(xy) (xy.real)
#define mkLCplx(x,y, xy) { xy.imag = x; xy.real = y; }

/*! Worst case structure alignment constant expressed as log2(.)
 */
#define TYP_STRUCT_LOG2ALIGN 3 /* structure alignment */

/*! Worst case structure alignment constant expressed as N words
 */
#define TYP_STRUCT_ALIGN     8


/*! Actual number of bits for the portable (*least*) types.
 */
#define  INT_LEAST8_BITS    8
#define UINT_LEAST8_BITS    8
#define  INT_LEAST16_BITS  16
#define UINT_LEAST16_BITS  16
#define  INT_LEAST32_BITS  32
#define UINT_LEAST32_BITS  32
#define  INT_LEAST40_BITS  64
#define UINT_LEAST40_BITS  64
#define  INT_LEAST64_BITS  64
#define UINT_LEAST64_BITS  64

/*! Number of bits in the bool type */
#define  BOOL_BITS          8

/*! Number of bits in the word_t type */
#define  WORD_BITS          8

#define LITTLE_ENDIAN

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _TYPES_PC_H */
/* nothing past this point */
