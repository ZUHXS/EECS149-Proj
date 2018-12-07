/**
 * @file swpform.h
 *
 * @desc This file defines the types to be used when programming proton
 *
 * Copyright (C) 2005-2017 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
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
 *
*/

#ifndef _SWPFORM_H_
#define _SWPFORM_H_

#if !((defined (__GNUC__)) || (defined (__LISAC__)) || (defined (_MSC_VER)))
#error compiler type not supported!
#endif

#if (defined __cplusplus)
#error C++ compilation not supported!
#endif

/****************/
/* GNU compiler */
/****************/
#if defined (__GNUC__)
/* C99 types */
#if (__STDC_VERSION__ >= 199901L)
#include <inttypes.h>
#include <stdbool.h>
#else
#error Please enable ISO C99 compilation (ISO/IEC 9899:1999 standard)
#endif
struct regpair__
{
    uint64_t lo;
    uint64_t hi;
};
typedef struct regpair__ uint128_t; /* 128-bit type only used as a container */
#define uint128_const_make(hi,lo) {(lo),(hi)}
#define uint128_gethi(val) (val).hi
#define uint128_getlo(val) (val).lo
/* useful macros */
#define IN
#define OUT
#define INOUT
#define INLINE static __inline__ 
#define RESTRICT 
/* some standard includes */
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <intrinsic.h>          /* intrinsics */
#endif /* __GNUC__ */

/**********************/
/* Microsoft compiler */
/**********************/
#if defined (_MSC_VER)
/* C99 types */
#if (__STDC_VERSION__ >= 199901L)
#include <inttypes.h>
#include <stdbool.h>
#else
typedef unsigned int uintptr_t; /* windows in mostly 32-bit */
typedef int intptr_t; /* windows is mostly 32-bit */
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef unsigned __int64 uint64_t;
typedef char int8_t;
typedef short int16_t;
typedef int int32_t;
typedef __int64 int64_t;
typedef unsigned int _Bool;
typedef _Bool bool;
#define true 1
#define false 0
#define __bool_true_false_are_defined   1
#endif
struct regpair__
{
    uint64_t lo;
    uint64_t hi;
};
typedef struct regpair__ uint128_t; /* 128-bit type only used as a container */
#define uint128_const_make(hi,lo) {(lo),(hi)}
#define uint128_gethi(val) (val).hi
#define uint128_getlo(val) (val).lo
/* useful macros */
#define IN
#define OUT
#define INOUT
#define INLINE static __inline
#define RESTRICT 
/* some standard includes */
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <intrinsic.h>          /* intrinsics */
#endif /* _MSC_VER */

/*************************************************************/
/* Proton compiler - generated from CoWare compiler designer */
/*************************************************************/
#if defined (__LISAC__)
/* C99 types */
/*#if (__STDC_VERSION__ >= 199901L)*/
#if 0
#include <inttypes.h>
#include <stdbool.h>
#else
typedef unsigned int uintptr_t; /* pointers are 32-bit on proton */
typedef int intptr_t; /* pointers are 32-bit on proton */
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long uint64_t;
typedef char int8_t;
typedef short int16_t;
typedef int int32_t;
typedef long int64_t;
typedef _Bool bool;
#define true 1
#define false 0
#define __bool_true_false_are_defined   1
#endif
typedef unsigned long long uint128_t;   /* 128-bit type only used as a container - must pass the -flong-long to the code generator i.e. -Wf,-flong-long */
#define uint128_const_make(hi,lo) (((hi##LLU)<<64)|((lo##LLU)&0xFFFFFFFFFFFFFFFFLU))
#define uint128_getlo(val) *((uint64_t*)(&##(val)))
#define uint128_gethi(val) *((uint64_t*)(&##(val))+1)
/* useful macros */
#define IN
#define OUT
#define INOUT
#define INLINE #pragma inline
#define RESTRICT restrict       /* restrict is support passing the -frestrict option to the code generator i.e. -Wf,-frestrict */
/* some standard includes */
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#endif /* __LISAC__ */

#endif /* _SWPFORM_H_ */
