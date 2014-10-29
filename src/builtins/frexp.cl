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

_CLC_OVERLOAD _CLC_DEF float frexp(float x, global  int * ptr) SCALAR_BODY(float, __builtin_frexpf, int)
_CLC_OVERLOAD _CLC_DEF float frexp(float x, local   int * ptr) SCALAR_BODY(float, __builtin_frexpf, int)
_CLC_OVERLOAD _CLC_DEF float frexp(float x, private int * ptr) SCALAR_BODY(float, __builtin_frexpf, int)

_CLC_OVERLOAD _CLC_DEF float2 frexp(float2 x, global  int2 * ptr) VECTOR_BODY(float, 2, __builtin_frexpf, int)
_CLC_OVERLOAD _CLC_DEF float2 frexp(float2 x, local   int2 * ptr) VECTOR_BODY(float, 2, __builtin_frexpf, int)
_CLC_OVERLOAD _CLC_DEF float2 frexp(float2 x, private int2 * ptr) VECTOR_BODY(float, 2, __builtin_frexpf, int)

_CLC_OVERLOAD _CLC_DEF float3 frexp(float3 x, global  int3 * ptr) VECTOR_BODY(float, 3, __builtin_frexpf, int)
_CLC_OVERLOAD _CLC_DEF float3 frexp(float3 x, local   int3 * ptr) VECTOR_BODY(float, 3, __builtin_frexpf, int)
_CLC_OVERLOAD _CLC_DEF float3 frexp(float3 x, private int3 * ptr) VECTOR_BODY(float, 3, __builtin_frexpf, int)

_CLC_OVERLOAD _CLC_DEF float4 frexp(float4 x, global  int4 * ptr) VECTOR_BODY(float, 4, __builtin_frexpf, int)
_CLC_OVERLOAD _CLC_DEF float4 frexp(float4 x, local   int4 * ptr) VECTOR_BODY(float, 4, __builtin_frexpf, int)
_CLC_OVERLOAD _CLC_DEF float4 frexp(float4 x, private int4 * ptr) VECTOR_BODY(float, 4, __builtin_frexpf, int)

_CLC_OVERLOAD _CLC_DEF float8 frexp(float8 x, global  int8 * ptr) VECTOR_BODY(float, 8, __builtin_frexpf, int)
_CLC_OVERLOAD _CLC_DEF float8 frexp(float8 x, local   int8 * ptr) VECTOR_BODY(float, 8, __builtin_frexpf, int)
_CLC_OVERLOAD _CLC_DEF float8 frexp(float8 x, private int8 * ptr) VECTOR_BODY(float, 8, __builtin_frexpf, int)

_CLC_OVERLOAD _CLC_DEF float16 frexp(float16 x, global  int16 * ptr) VECTOR_BODY(float, 16, __builtin_frexpf, int)
_CLC_OVERLOAD _CLC_DEF float16 frexp(float16 x, local   int16 * ptr) VECTOR_BODY(float, 16, __builtin_frexpf, int)
_CLC_OVERLOAD _CLC_DEF float16 frexp(float16 x, private int16 * ptr) VECTOR_BODY(float, 16, __builtin_frexpf, int)

_CLC_OVERLOAD _CLC_DEF double frexp(double x, global  int * ptr) SCALAR_BODY(double, __builtin_frexp, int)
_CLC_OVERLOAD _CLC_DEF double frexp(double x, local   int * ptr) SCALAR_BODY(double, __builtin_frexp, int)
_CLC_OVERLOAD _CLC_DEF double frexp(double x, private int * ptr) SCALAR_BODY(double, __builtin_frexp, int)

_CLC_OVERLOAD _CLC_DEF double2 frexp(double2 x, global  int2 * ptr) VECTOR_BODY(double, 2, __builtin_frexp, int)
_CLC_OVERLOAD _CLC_DEF double2 frexp(double2 x, local   int2 * ptr) VECTOR_BODY(double, 2, __builtin_frexp, int)
_CLC_OVERLOAD _CLC_DEF double2 frexp(double2 x, private int2 * ptr) VECTOR_BODY(double, 2, __builtin_frexp, int)

_CLC_OVERLOAD _CLC_DEF double3 frexp(double3 x, global  int3 * ptr) VECTOR_BODY(double, 3, __builtin_frexp, int)
_CLC_OVERLOAD _CLC_DEF double3 frexp(double3 x, local   int3 * ptr) VECTOR_BODY(double, 3, __builtin_frexp, int)
_CLC_OVERLOAD _CLC_DEF double3 frexp(double3 x, private int3 * ptr) VECTOR_BODY(double, 3, __builtin_frexp, int)

_CLC_OVERLOAD _CLC_DEF double4 frexp(double4 x, global  int4 * ptr) VECTOR_BODY(double, 4, __builtin_frexp, int)
_CLC_OVERLOAD _CLC_DEF double4 frexp(double4 x, local   int4 * ptr) VECTOR_BODY(double, 4, __builtin_frexp, int)
_CLC_OVERLOAD _CLC_DEF double4 frexp(double4 x, private int4 * ptr) VECTOR_BODY(double, 4, __builtin_frexp, int)

_CLC_OVERLOAD _CLC_DEF double8 frexp(double8 x, global  int8 * ptr) VECTOR_BODY(double, 8, __builtin_frexp, int)
_CLC_OVERLOAD _CLC_DEF double8 frexp(double8 x, local   int8 * ptr) VECTOR_BODY(double, 8, __builtin_frexp, int)
_CLC_OVERLOAD _CLC_DEF double8 frexp(double8 x, private int8 * ptr) VECTOR_BODY(double, 8, __builtin_frexp, int)

_CLC_OVERLOAD _CLC_DEF double16 frexp(double16 x, global  int16 * ptr) VECTOR_BODY(double, 16, __builtin_frexp, int)
_CLC_OVERLOAD _CLC_DEF double16 frexp(double16 x, local   int16 * ptr) VECTOR_BODY(double, 16, __builtin_frexp, int)
_CLC_OVERLOAD _CLC_DEF double16 frexp(double16 x, private int16 * ptr) VECTOR_BODY(double, 16, __builtin_frexp, int)
