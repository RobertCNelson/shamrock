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

#define REMQUO_SCALAR_BODY(type, op, ptr_type) \
{ \
    ptr_type temp; \
    type result = op(x, y, &temp); \
    *ptr = temp; \
    return result; \
} \

#define REMQUO_VECTOR_BODY_2(op, ptr_type) \
    temp.s0 = op(x.s0 ,y.s0, &(((ptr_type*)&itemp)[0])); \
    temp.s1 = op(x.s1 ,y.s1, &(((ptr_type*)&itemp)[1])); \

#define REMQUO_VECTOR_BODY_3(op, ptr_type) \
    REMQUO_VECTOR_BODY_2(op, ptr_type) \
    temp.s2 = op(x.s2 ,y.s2, &(((ptr_type*)&itemp)[2])); \

#define REMQUO_VECTOR_BODY_4(op, ptr_type) \
    REMQUO_VECTOR_BODY_3(op, ptr_type) \
    temp.s3 = op(x.s3 ,y.s3, &(((ptr_type*)&itemp)[3])); \

#define REMQUO_VECTOR_BODY_8(op, ptr_type) \
    REMQUO_VECTOR_BODY_4(op, ptr_type) \
    temp.s4 = op(x.s4 ,y.s4, &(((ptr_type*)&itemp)[4])); \
    temp.s5 = op(x.s5 ,y.s5, &(((ptr_type*)&itemp)[5])); \
    temp.s6 = op(x.s6 ,y.s6, &(((ptr_type*)&itemp)[6])); \
    temp.s7 = op(x.s7 ,y.s7, &(((ptr_type*)&itemp)[7])); \

#define REMQUO_VECTOR_BODY_16(op, ptr_type) \
    REMQUO_VECTOR_BODY_8(op, ptr_type) \
    temp.s8 = op(x.s8 ,y.s8, &(((ptr_type*)&itemp)[8])); \
    temp.s9 = op(x.s9 ,y.s9, &(((ptr_type*)&itemp)[9])); \
    temp.sa = op(x.sa ,y.sa, &(((ptr_type*)&itemp)[10])); \
    temp.sb = op(x.sb ,y.sb, &(((ptr_type*)&itemp)[11])); \
    temp.sc = op(x.sc ,y.sc, &(((ptr_type*)&itemp)[12])); \
    temp.sd = op(x.sd ,y.sd, &(((ptr_type*)&itemp)[13])); \
    temp.se = op(x.se ,y.se, &(((ptr_type*)&itemp)[14])); \
    temp.sf = op(x.sf ,y.sf, &(((ptr_type*)&itemp)[15])); \

#define REMQUO_VECTOR_BODY(prim_type, num, op, ptr_type) \
{ \
    prim_type##num   temp; \
    ptr_type##num    itemp; \
    REMQUO_VECTOR_BODY_##num(op, ptr_type)\
    *ptr =  itemp; \
    return temp; \
} \


_CLC_PROTECTED float  remquof(float  x, float y, int * ptr);
_CLC_PROTECTED double builtin_remquo(double x, double y, int * ptr);

_CLC_OVERLOAD _CLC_DEF float remquo(float x, float y, global  int * ptr) REMQUO_SCALAR_BODY(float, remquof, int)
_CLC_OVERLOAD _CLC_DEF float remquo(float x, float y, local   int * ptr) REMQUO_SCALAR_BODY(float, remquof, int)
_CLC_OVERLOAD _CLC_DEF float remquo(float x, float y, private int * ptr) REMQUO_SCALAR_BODY(float, remquof, int)

_CLC_OVERLOAD _CLC_DEF float2 remquo(float2 x, float2 y, global  int2 * ptr) REMQUO_VECTOR_BODY(float, 2, remquof, int)
_CLC_OVERLOAD _CLC_DEF float2 remquo(float2 x, float2 y, local   int2 * ptr) REMQUO_VECTOR_BODY(float, 2, remquof, int)
_CLC_OVERLOAD _CLC_DEF float2 remquo(float2 x, float2 y, private int2 * ptr) REMQUO_VECTOR_BODY(float, 2, remquof, int)

_CLC_OVERLOAD _CLC_DEF float3 remquo(float3 x, float3 y, global  int3 * ptr) REMQUO_VECTOR_BODY(float, 3, remquof, int)
_CLC_OVERLOAD _CLC_DEF float3 remquo(float3 x, float3 y, local   int3 * ptr) REMQUO_VECTOR_BODY(float, 3, remquof, int)
_CLC_OVERLOAD _CLC_DEF float3 remquo(float3 x, float3 y, private int3 * ptr) REMQUO_VECTOR_BODY(float, 3, remquof, int)

_CLC_OVERLOAD _CLC_DEF float4 remquo(float4 x, float4 y, global  int4 * ptr) REMQUO_VECTOR_BODY(float, 4, remquof, int)
_CLC_OVERLOAD _CLC_DEF float4 remquo(float4 x, float4 y, local   int4 * ptr) REMQUO_VECTOR_BODY(float, 4, remquof, int)
_CLC_OVERLOAD _CLC_DEF float4 remquo(float4 x, float4 y, private int4 * ptr) REMQUO_VECTOR_BODY(float, 4, remquof, int)

_CLC_OVERLOAD _CLC_DEF float8 remquo(float8 x, float8 y, global  int8 * ptr) REMQUO_VECTOR_BODY(float, 8, remquof, int)
_CLC_OVERLOAD _CLC_DEF float8 remquo(float8 x, float8 y, local   int8 * ptr) REMQUO_VECTOR_BODY(float, 8, remquof, int)
_CLC_OVERLOAD _CLC_DEF float8 remquo(float8 x, float8 y, private int8 * ptr) REMQUO_VECTOR_BODY(float, 8, remquof, int)

_CLC_OVERLOAD _CLC_DEF float16 remquo(float16 x, float16 y, global  int16 * ptr) REMQUO_VECTOR_BODY(float, 16, remquof, int)
_CLC_OVERLOAD _CLC_DEF float16 remquo(float16 x, float16 y, local   int16 * ptr) REMQUO_VECTOR_BODY(float, 16, remquof, int)
_CLC_OVERLOAD _CLC_DEF float16 remquo(float16 x, float16 y, private int16 * ptr) REMQUO_VECTOR_BODY(float, 16, remquof, int)

_CLC_OVERLOAD _CLC_DEF double remquo(double x, double y, global  int * ptr) REMQUO_SCALAR_BODY(double, builtin_remquo, int)
_CLC_OVERLOAD _CLC_DEF double remquo(double x, double y, local   int * ptr) REMQUO_SCALAR_BODY(double, builtin_remquo, int)
_CLC_OVERLOAD _CLC_DEF double remquo(double x, double y, private int * ptr) REMQUO_SCALAR_BODY(double, builtin_remquo, int)

_CLC_OVERLOAD _CLC_DEF double2 remquo(double2 x, double2 y, global  int2 * ptr) REMQUO_VECTOR_BODY(double, 2, builtin_remquo, int)
_CLC_OVERLOAD _CLC_DEF double2 remquo(double2 x, double2 y, local   int2 * ptr) REMQUO_VECTOR_BODY(double, 2, builtin_remquo, int)
_CLC_OVERLOAD _CLC_DEF double2 remquo(double2 x, double2 y, private int2 * ptr) REMQUO_VECTOR_BODY(double, 2, builtin_remquo, int)

_CLC_OVERLOAD _CLC_DEF double3 remquo(double3 x, double3 y, global  int3 * ptr) REMQUO_VECTOR_BODY(double, 3, builtin_remquo, int)
_CLC_OVERLOAD _CLC_DEF double3 remquo(double3 x, double3 y, local   int3 * ptr) REMQUO_VECTOR_BODY(double, 3, builtin_remquo, int)
_CLC_OVERLOAD _CLC_DEF double3 remquo(double3 x, double3 y, private int3 * ptr) REMQUO_VECTOR_BODY(double, 3, builtin_remquo, int)

_CLC_OVERLOAD _CLC_DEF double4 remquo(double4 x, double4 y, global  int4 * ptr) REMQUO_VECTOR_BODY(double, 4, builtin_remquo, int)
_CLC_OVERLOAD _CLC_DEF double4 remquo(double4 x, double4 y, local   int4 * ptr) REMQUO_VECTOR_BODY(double, 4, builtin_remquo, int)
_CLC_OVERLOAD _CLC_DEF double4 remquo(double4 x, double4 y, private int4 * ptr) REMQUO_VECTOR_BODY(double, 4, builtin_remquo, int)

_CLC_OVERLOAD _CLC_DEF double8 remquo(double8 x, double8 y, global  int8 * ptr) REMQUO_VECTOR_BODY(double, 8, builtin_remquo, int)
_CLC_OVERLOAD _CLC_DEF double8 remquo(double8 x, double8 y, local   int8 * ptr) REMQUO_VECTOR_BODY(double, 8, builtin_remquo, int)
_CLC_OVERLOAD _CLC_DEF double8 remquo(double8 x, double8 y, private int8 * ptr) REMQUO_VECTOR_BODY(double, 8, builtin_remquo, int)

_CLC_OVERLOAD _CLC_DEF double16 remquo(double16 x, double16 y, global  int16 * ptr) REMQUO_VECTOR_BODY(double, 16, builtin_remquo, int)
_CLC_OVERLOAD _CLC_DEF double16 remquo(double16 x, double16 y, local   int16 * ptr) REMQUO_VECTOR_BODY(double, 16, builtin_remquo, int)
_CLC_OVERLOAD _CLC_DEF double16 remquo(double16 x, double16 y, private int16 * ptr) REMQUO_VECTOR_BODY(double, 16, builtin_remquo, int)
