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

_CLC_PROTECTED float  lgammaf_r(float  x, int * ptr);
_CLC_PROTECTED double builtin_lgamma_r(double x, int * ptr);

_CLC_OVERLOAD _CLC_DEF float lgamma_r(float x, global  int * ptr) SCALAR_BODY(float, lgammaf_r, int)
_CLC_OVERLOAD _CLC_DEF float lgamma_r(float x, local   int * ptr) SCALAR_BODY(float, lgammaf_r, int)
_CLC_OVERLOAD _CLC_DEF float lgamma_r(float x, private int * ptr) SCALAR_BODY(float, lgammaf_r, int)

_CLC_OVERLOAD _CLC_DEF float2 lgamma_r(float2 x, global  int2 * ptr) VECTOR_BODY(float, 2, lgammaf_r, int)
_CLC_OVERLOAD _CLC_DEF float2 lgamma_r(float2 x, local   int2 * ptr) VECTOR_BODY(float, 2, lgammaf_r, int)
_CLC_OVERLOAD _CLC_DEF float2 lgamma_r(float2 x, private int2 * ptr) VECTOR_BODY(float, 2, lgammaf_r, int)

_CLC_OVERLOAD _CLC_DEF float3 lgamma_r(float3 x, global  int3 * ptr) VECTOR_BODY(float, 3, lgammaf_r, int)
_CLC_OVERLOAD _CLC_DEF float3 lgamma_r(float3 x, local   int3 * ptr) VECTOR_BODY(float, 3, lgammaf_r, int)
_CLC_OVERLOAD _CLC_DEF float3 lgamma_r(float3 x, private int3 * ptr) VECTOR_BODY(float, 3, lgammaf_r, int)

_CLC_OVERLOAD _CLC_DEF float4 lgamma_r(float4 x, global  int4 * ptr) VECTOR_BODY(float, 4, lgammaf_r, int)
_CLC_OVERLOAD _CLC_DEF float4 lgamma_r(float4 x, local   int4 * ptr) VECTOR_BODY(float, 4, lgammaf_r, int)
_CLC_OVERLOAD _CLC_DEF float4 lgamma_r(float4 x, private int4 * ptr) VECTOR_BODY(float, 4, lgammaf_r, int)

_CLC_OVERLOAD _CLC_DEF float8 lgamma_r(float8 x, global  int8 * ptr) VECTOR_BODY(float, 8, lgammaf_r, int)
_CLC_OVERLOAD _CLC_DEF float8 lgamma_r(float8 x, local   int8 * ptr) VECTOR_BODY(float, 8, lgammaf_r, int)
_CLC_OVERLOAD _CLC_DEF float8 lgamma_r(float8 x, private int8 * ptr) VECTOR_BODY(float, 8, lgammaf_r, int)

_CLC_OVERLOAD _CLC_DEF float16 lgamma_r(float16 x, global  int16 * ptr) VECTOR_BODY(float, 16, lgammaf_r, int)
_CLC_OVERLOAD _CLC_DEF float16 lgamma_r(float16 x, local   int16 * ptr) VECTOR_BODY(float, 16, lgammaf_r, int)
_CLC_OVERLOAD _CLC_DEF float16 lgamma_r(float16 x, private int16 * ptr) VECTOR_BODY(float, 16, lgammaf_r, int)

_CLC_OVERLOAD _CLC_DEF double lgamma_r(double x, global  int * ptr) SCALAR_BODY(double, builtin_lgamma_r, int)
_CLC_OVERLOAD _CLC_DEF double lgamma_r(double x, local   int * ptr) SCALAR_BODY(double, builtin_lgamma_r, int)
_CLC_OVERLOAD _CLC_DEF double lgamma_r(double x, private int * ptr) SCALAR_BODY(double, builtin_lgamma_r, int)

_CLC_OVERLOAD _CLC_DEF double2 lgamma_r(double2 x, global  int2 * ptr) VECTOR_BODY(double, 2, builtin_lgamma_r, int)
_CLC_OVERLOAD _CLC_DEF double2 lgamma_r(double2 x, local   int2 * ptr) VECTOR_BODY(double, 2, builtin_lgamma_r, int)
_CLC_OVERLOAD _CLC_DEF double2 lgamma_r(double2 x, private int2 * ptr) VECTOR_BODY(double, 2, builtin_lgamma_r, int)

_CLC_OVERLOAD _CLC_DEF double3 lgamma_r(double3 x, global  int3 * ptr) VECTOR_BODY(double, 3, builtin_lgamma_r, int)
_CLC_OVERLOAD _CLC_DEF double3 lgamma_r(double3 x, local   int3 * ptr) VECTOR_BODY(double, 3, builtin_lgamma_r, int)
_CLC_OVERLOAD _CLC_DEF double3 lgamma_r(double3 x, private int3 * ptr) VECTOR_BODY(double, 3, builtin_lgamma_r, int)

_CLC_OVERLOAD _CLC_DEF double4 lgamma_r(double4 x, global  int4 * ptr) VECTOR_BODY(double, 4, builtin_lgamma_r, int)
_CLC_OVERLOAD _CLC_DEF double4 lgamma_r(double4 x, local   int4 * ptr) VECTOR_BODY(double, 4, builtin_lgamma_r, int)
_CLC_OVERLOAD _CLC_DEF double4 lgamma_r(double4 x, private int4 * ptr) VECTOR_BODY(double, 4, builtin_lgamma_r, int)

_CLC_OVERLOAD _CLC_DEF double8 lgamma_r(double8 x, global  int8 * ptr) VECTOR_BODY(double, 8, builtin_lgamma_r, int)
_CLC_OVERLOAD _CLC_DEF double8 lgamma_r(double8 x, local   int8 * ptr) VECTOR_BODY(double, 8, builtin_lgamma_r, int)
_CLC_OVERLOAD _CLC_DEF double8 lgamma_r(double8 x, private int8 * ptr) VECTOR_BODY(double, 8, builtin_lgamma_r, int)

_CLC_OVERLOAD _CLC_DEF double16 lgamma_r(double16 x, global  int16 * ptr) VECTOR_BODY(double, 16, builtin_lgamma_r, int)
_CLC_OVERLOAD _CLC_DEF double16 lgamma_r(double16 x, local   int16 * ptr) VECTOR_BODY(double, 16, builtin_lgamma_r, int)
_CLC_OVERLOAD _CLC_DEF double16 lgamma_r(double16 x, private int16 * ptr) VECTOR_BODY(double, 16, builtin_lgamma_r, int)

