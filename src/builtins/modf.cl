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

_CLC_PROTECTED float  modff(float  x, float  * iptr);
_CLC_PROTECTED double  builtin_modf(double  x, double  * iptr);


_CLC_OVERLOAD _CLC_DEF float modf(float x, global  float * ptr) SCALAR_BODY(float, modff, float)
_CLC_OVERLOAD _CLC_DEF float modf(float x, local   float * ptr) SCALAR_BODY(float, modff, float)
_CLC_OVERLOAD _CLC_DEF float modf(float x, private float * ptr) SCALAR_BODY(float, modff, float)

_CLC_OVERLOAD _CLC_DEF float2 modf(float2 x, global  float2 * ptr) VECTOR_BODY(float, 2, modff, float)
_CLC_OVERLOAD _CLC_DEF float2 modf(float2 x, local   float2 * ptr) VECTOR_BODY(float, 2, modff, float)
_CLC_OVERLOAD _CLC_DEF float2 modf(float2 x, private float2 * ptr) VECTOR_BODY(float, 2, modff, float)

_CLC_OVERLOAD _CLC_DEF float3 modf(float3 x, global  float3 * ptr) VECTOR_BODY(float, 3, modff, float)
_CLC_OVERLOAD _CLC_DEF float3 modf(float3 x, local   float3 * ptr) VECTOR_BODY(float, 3, modff, float)
_CLC_OVERLOAD _CLC_DEF float3 modf(float3 x, private float3 * ptr) VECTOR_BODY(float, 3, modff, float)

_CLC_OVERLOAD _CLC_DEF float4 modf(float4 x, global  float4 * ptr) VECTOR_BODY(float, 4, modff, float)
_CLC_OVERLOAD _CLC_DEF float4 modf(float4 x, local   float4 * ptr) VECTOR_BODY(float, 4, modff, float)
_CLC_OVERLOAD _CLC_DEF float4 modf(float4 x, private float4 * ptr) VECTOR_BODY(float, 4, modff, float)

_CLC_OVERLOAD _CLC_DEF float8 modf(float8 x, global  float8 * ptr) VECTOR_BODY(float, 8, modff, float)
_CLC_OVERLOAD _CLC_DEF float8 modf(float8 x, local   float8 * ptr) VECTOR_BODY(float, 8, modff, float)
_CLC_OVERLOAD _CLC_DEF float8 modf(float8 x, private float8 * ptr) VECTOR_BODY(float, 8, modff, float)

_CLC_OVERLOAD _CLC_DEF float16 modf(float16 x, global  float16 * ptr) VECTOR_BODY(float, 16, modff, float)
_CLC_OVERLOAD _CLC_DEF float16 modf(float16 x, local   float16 * ptr) VECTOR_BODY(float, 16, modff, float)
_CLC_OVERLOAD _CLC_DEF float16 modf(float16 x, private float16 * ptr) VECTOR_BODY(float, 16, modff, float)

_CLC_OVERLOAD _CLC_DEF double modf(double x, global  double * ptr) SCALAR_BODY(double, builtin_modf, double)
_CLC_OVERLOAD _CLC_DEF double modf(double x, local   double * ptr) SCALAR_BODY(double, builtin_modf, double)
_CLC_OVERLOAD _CLC_DEF double modf(double x, private double * ptr) SCALAR_BODY(double, builtin_modf, double)

_CLC_OVERLOAD _CLC_DEF double2 modf(double2 x, global  double2 * ptr) VECTOR_BODY(double, 2, builtin_modf, double)
_CLC_OVERLOAD _CLC_DEF double2 modf(double2 x, local   double2 * ptr) VECTOR_BODY(double, 2, builtin_modf, double)
_CLC_OVERLOAD _CLC_DEF double2 modf(double2 x, private double2 * ptr) VECTOR_BODY(double, 2, builtin_modf, double)

_CLC_OVERLOAD _CLC_DEF double3 modf(double3 x, global  double3 * ptr) VECTOR_BODY(double, 3, builtin_modf, double)
_CLC_OVERLOAD _CLC_DEF double3 modf(double3 x, local   double3 * ptr) VECTOR_BODY(double, 3, builtin_modf, double)
_CLC_OVERLOAD _CLC_DEF double3 modf(double3 x, private double3 * ptr) VECTOR_BODY(double, 3, builtin_modf, double)

_CLC_OVERLOAD _CLC_DEF double4 modf(double4 x, global  double4 * ptr) VECTOR_BODY(double, 4, builtin_modf, double)
_CLC_OVERLOAD _CLC_DEF double4 modf(double4 x, local   double4 * ptr) VECTOR_BODY(double, 4, builtin_modf, double)
_CLC_OVERLOAD _CLC_DEF double4 modf(double4 x, private double4 * ptr) VECTOR_BODY(double, 4, builtin_modf, double)

_CLC_OVERLOAD _CLC_DEF double8 modf(double8 x, global  double8 * ptr) VECTOR_BODY(double, 8, builtin_modf, double)
_CLC_OVERLOAD _CLC_DEF double8 modf(double8 x, local   double8 * ptr) VECTOR_BODY(double, 8, builtin_modf, double)
_CLC_OVERLOAD _CLC_DEF double8 modf(double8 x, private double8 * ptr) VECTOR_BODY(double, 8, builtin_modf, double)

_CLC_OVERLOAD _CLC_DEF double16 modf(double16 x, global  double16 * ptr) VECTOR_BODY(double, 16, builtin_modf, double)
_CLC_OVERLOAD _CLC_DEF double16 modf(double16 x, local   double16 * ptr) VECTOR_BODY(double, 16, builtin_modf, double)
_CLC_OVERLOAD _CLC_DEF double16 modf(double16 x, private double16 * ptr) VECTOR_BODY(double, 16, builtin_modf, double)

