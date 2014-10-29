/******************************************************************************
 * Copyright (c) 2011-2014, Texas Instruments Incorporated - http://www.ti.com/
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *       * Redistributions of source code must retain the above copyright
 *         notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *         notice, this list of conditions and the following disclaimer in the
 *         documentation and/or other materials provided with the distribution.
 *       * Neither the name of Texas Instruments Incorporated nor the
 *         names of its contributors may be used to endorse or promote products
 *         derived from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 *   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *   THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include "clc.h"

#define UNARY(function) \
_CLC_PROTECTED _CLC_INLINE float function##f(float x)  { return (float)__builtin_##function(x); } \
_CLC_PROTECTED _CLC_INLINE double function##d(double x)  { return __builtin_##function(x); } \
UNARY_VEC_DEF(float,  float,  function, function##f) \
UNARY_VEC_DEF(double, double, function, function##d) \

#define UNARY_ALT(utype, function) \
_CLC_PROTECTED _CLC_INLINE utype function##f(float x)  { return __builtin_##function(x); } \
_CLC_PROTECTED _CLC_INLINE utype function##d(double x)  { return __builtin_##function(x); } \
UNARY_VEC_DEF(float,  utype,  function, function##f) \
UNARY_VEC_DEF(double, utype, function, function##d) \

#define UNARY_NO_BUILTIN(function) \
UNARY_VEC_DEF(float,  float,  function, function) \
UNARY_VEC_DEF(double, double, function, function) \

#define BINARY(function) \
_CLC_PROTECTED _CLC_INLINE float function##f(float x, float y)  { return (float)__builtin_##function(x,y); } \
_CLC_PROTECTED _CLC_INLINE double function##d(double x, double y)  { return __builtin_##function(x,y); } \
BINARY_VEC_DEF(float,  float,  function, function) \
BINARY_VEC_DEF(double, double, function, function) \

#define BINARY_NO_BUILTIN(function) \
BINARY_VEC_DEF(float,  float,  function, function) \
BINARY_VEC_DEF(double, double, function, function) \

#define TERNARY(function) \
_CLC_PROTECTED _CLC_INLINE float function##f(float x, float y, float z)  { return (float)__builtin_##function(x,y,z); } \
_CLC_PROTECTED _CLC_INLINE double function##d(double x, double y, double z)  { return __builtin_##function(x,y,z); } \
TERNARY_VEC_DEF(float,  float,  function, function) \
TERNARY_VEC_DEF(double, double, function, function) \

#define TERNARY_NO_BUILTIN(function) \
TERNARY_VEC_DEF(float,  float,  function, function) \
TERNARY_VEC_DEF(double, double, function, function) \

/*-------------------------------------------------------------------------
* Prototypes for the math builtins 
*------------------------------------------------------------------------*/
UNARY(acos)
UNARY(acosh)
UNARY_NO_BUILTIN(acospi)
UNARY(asin)
UNARY(asinh)
UNARY_NO_BUILTIN(asinpi)
UNARY(atan)
BINARY_NO_BUILTIN(atan2pi)
UNARY(atanh)
UNARY_NO_BUILTIN(atanpi)
BINARY(atan2)
UNARY(cbrt)
UNARY(ceil)
UNARY(cos)
BINARY(copysign)
UNARY(cosh)
UNARY_NO_BUILTIN(cospi)
UNARY(erf)
UNARY(erfc)
UNARY(exp)
UNARY(exp2)
UNARY_NO_BUILTIN(exp10)
UNARY(expm1)
UNARY(fabs)
BINARY(fdim)
UNARY(floor)
TERNARY(fma)
BINARY(fmax)
BINARY(fmin)
BINARY(fmod)
BINARY(hypot)

UNARY_ALT(int, ilogb)

BINARY_VEC_DEF_ALT(float,  float,  int, ldexp, ldexpf)
BINARY_VEC_DEF_ALT(double, double, int, ldexp, ldexp)

UNARY(lgamma)
UNARY(log)
UNARY(log2)
UNARY(log10)
UNARY(log1p)
UNARY(logb)
TERNARY_NO_BUILTIN(mad)
BINARY_NO_BUILTIN(maxmag)
BINARY_NO_BUILTIN(minmag)

UNARY_VEC_DEF(uint,  float,  nan, nan)
UNARY_VEC_DEF(ulong, double, nan, nan)

BINARY(nextafter)
BINARY(pow)

BINARY_VEC_DEF_ALT(float,  float,  int, pown, powf)
BINARY_VEC_DEF_ALT(double, double, int, pown, builtin_pow)

BINARY_NO_BUILTIN(powr)
BINARY(remainder)
UNARY(rint)

BINARY_VEC_DEF_ALT(float,  float,  int, rootn, builtin_rootnf)
BINARY_VEC_DEF_ALT(double, double, int, rootn, builtin_rootn)

UNARY(round)
UNARY_NO_BUILTIN(rsqrt)
UNARY(sin)
UNARY(sinh)
UNARY_NO_BUILTIN(sinpi)
UNARY(sqrt)
UNARY(tan)
UNARY(tanh)
UNARY_NO_BUILTIN(tanpi)
UNARY(tgamma)
UNARY(trunc)

/*-------------------------------------------------------------------------
* Half functions:
*------------------------------------------------------------------------*/

BINARY_NO_BUILTIN(half_divide)
UNARY_NO_BUILTIN(half_recip)


