/******************************************************************************
 * Copyright (c) 2013, Texas Instruments Incorporated - http://www.ti.com/
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
#include "cpu.h"
/* 32 bits atomic add */
_CLC_OVERLOAD _CLC_DEF int  atomic_add(volatile global int*  p, int  val)
{
     return __sync_fetch_and_add(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_add(volatile global uint* p, uint val)
{
    return __sync_fetch_and_add(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF int  atomic_add(volatile local  int*  p, int  val)
{
    return __sync_fetch_and_add(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_add(volatile local  uint* p, uint val)
{
    return __sync_fetch_and_add(p, val, __ATOMIC_RELAXED);
}

/* 64 bits atomic add */
_CLC_OVERLOAD _CLC_DEF long  atomic_add(volatile global long*  p, long  val)
{
     return __sync_fetch_and_add(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_add(volatile global ulong* p, ulong val)
{
    return __sync_fetch_and_add(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF long  atomic_add(volatile local  long*  p, long  val)
{
    return __sync_fetch_and_add(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_add(volatile local  ulong* p, ulong val)
{
    return __sync_fetch_and_add(p, val, __ATOMIC_RELAXED);
}

/* 32 bits atomic sub */
_CLC_OVERLOAD _CLC_DEF int  atomic_sub(volatile global int*  p, int  val)
{
    return __sync_fetch_and_sub(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_sub(volatile global uint* p, uint val)
{
    return __sync_fetch_and_sub(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF int  atomic_sub(volatile local  int*  p, int  val)
{
    return __sync_fetch_and_sub(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_sub(volatile local  uint* p, uint val)
{
    return __sync_fetch_and_sub(p, val, __ATOMIC_RELAXED);
}

/* 64 bits atomic sub */
_CLC_OVERLOAD _CLC_DEF long  atomic_sub(volatile global long*  p, long  val)
{
    return __sync_fetch_and_sub(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_sub(volatile global ulong* p, ulong val)
{
    return __sync_fetch_and_sub(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF long  atomic_sub(volatile local  long*  p, long  val)
{
    return __sync_fetch_and_sub(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_sub(volatile local  ulong* p, ulong val)
{
    return __sync_fetch_and_sub(p, val, __ATOMIC_RELAXED);
}

/* 32 bits atomic xchg */
_CLC_OVERLOAD _CLC_DEF int  atomic_xchg(volatile global int*  p, int  val)
{
    return __atomic_exchange_n(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_xchg(volatile global uint* p, uint val)
{
    return __atomic_exchange_n(p, val, __ATOMIC_RELAXED);
}
#if 0
_CLC_OVERLOAD _CLC_DEF float atomic_xchg(volatile global float* p, float val)
{
    return __atomic_exchange_n(p, val, __ATOMIC_RELAXED);
}
#endif
_CLC_OVERLOAD _CLC_DEF int  atomic_xchg(volatile local  int*  p, int  val)
{
    return __atomic_exchange_n(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_xchg(volatile local  uint* p, uint val)
{
    return __atomic_exchange_n(p, val, __ATOMIC_RELAXED);
}
#if 0
_CLC_OVERLOAD _CLC_DEF float atomic_xchg(volatile local  float* p, float val)
{
    return __atomic_exchange_n(p, val, __ATOMIC_RELAXED);
}
#endif

/* 64 bits atomic xchg */
_CLC_OVERLOAD _CLC_DEF long  atomic_xchg(volatile global long*  p, long  val)
{
    return __atomic_exchange_n(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_xchg(volatile global ulong* p, ulong val)
{
    return __atomic_exchange_n(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF long  atomic_xchg(volatile local  long*  p, long  val)
{
    return __atomic_exchange_n(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_xchg(volatile local  ulong* p, ulong val)
{
    return __atomic_exchange_n(p, val, __ATOMIC_RELAXED);
}

/* 32 bits atomic inc */
_CLC_OVERLOAD _CLC_DEF int  atomic_inc(volatile global int*  p)
{
    return atomic_add(p, 1);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_inc(volatile global uint* p)
{
    return atomic_add(p, 1);
}

_CLC_OVERLOAD _CLC_DEF int  atomic_inc(volatile local  int*  p)
{
    return atomic_add(p, 1);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_inc(volatile local  uint* p)
{
    return atomic_add(p, 1);
}

/* 64 bits atomic inc */
_CLC_OVERLOAD _CLC_DEF long  atomic_inc(volatile global long*  p)
{
    return atomic_add(p, 1);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_inc(volatile global ulong* p)
{
    return atomic_add(p, 1);
}

_CLC_OVERLOAD _CLC_DEF long  atomic_inc(volatile local  long*  p)
{
    return atomic_add(p, 1);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_inc(volatile local  ulong* p)
{
    return atomic_add(p, 1);
}

/* 32 bits atomic dec */
_CLC_OVERLOAD _CLC_DEF int  atomic_dec(volatile global int*  p)
{
    return atomic_sub(p, 1);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_dec(volatile global uint* p)
{
    return atomic_sub(p, 1);
}

_CLC_OVERLOAD _CLC_DEF int  atomic_dec(volatile local  int*  p)
{
    return atomic_sub(p, 1);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_dec(volatile local  uint* p)
{
    return atomic_sub(p, 1);
}

/* 64 bits atomic dec */
_CLC_OVERLOAD _CLC_DEF long  atomic_dec(volatile global long*  p)
{
    return atomic_sub(p, 1);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_dec(volatile global ulong* p)
{
    return atomic_sub(p, 1);
}

_CLC_OVERLOAD _CLC_DEF long  atomic_dec(volatile local  long*  p)
{
    return atomic_sub(p, 1);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_dec(volatile local  ulong* p)
{
    return atomic_sub(p, 1);
}

/* 32 bits atomic cmpxchg */
_CLC_OVERLOAD _CLC_DEF int  atomic_cmpxchg(volatile global int*  p, int  cmp, int  val)
{
    __atomic_compare_exchange_n((volatile global int*)p, &cmp, val, false,
                                            __ATOMIC_RELAXED, __ATOMIC_RELAXED);
    return cmp;
}

_CLC_OVERLOAD _CLC_DEF uint atomic_cmpxchg(volatile global uint* p, uint cmp, uint val)
{
    __atomic_compare_exchange_n((volatile global uint*)p, &cmp, val, false,
                                            __ATOMIC_RELAXED, __ATOMIC_RELAXED);
    return cmp;
}

_CLC_OVERLOAD _CLC_DEF int  atomic_cmpxchg(volatile local int*  p, int  cmp, int  val)
{
    __atomic_compare_exchange_n((volatile local int*)p, &cmp, val, false,
                                            __ATOMIC_RELAXED, __ATOMIC_RELAXED);
    return cmp;
}

_CLC_OVERLOAD _CLC_DEF uint atomic_cmpxchg(volatile local  uint* p, uint cmp, uint val)
{
    __atomic_compare_exchange_n((volatile local uint*)p, &cmp, val, false,
                                            __ATOMIC_RELAXED, __ATOMIC_RELAXED);
    return cmp;
}

/* 64 bits atomic cmpxchg */
_CLC_OVERLOAD _CLC_DEF long  atomic_cmpxchg(volatile global long*  p, long  cmp, long  val)
{
    __atomic_compare_exchange_n((volatile global long*)p, &cmp, val, false,
                                            __ATOMIC_RELAXED, __ATOMIC_RELAXED);
    return cmp;
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_cmpxchg(volatile global ulong* p, ulong cmp, ulong val)
{
    __atomic_compare_exchange_n((volatile global ulong*)p, &cmp, val, false,
                                            __ATOMIC_RELAXED, __ATOMIC_RELAXED);
    return cmp;
}

_CLC_OVERLOAD _CLC_DEF long  atomic_cmpxchg(volatile local long*  p, long  cmp, long  val)
{
    __atomic_compare_exchange_n((volatile local long*)p, &cmp, val, false,
                                            __ATOMIC_RELAXED, __ATOMIC_RELAXED);
    return cmp;
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_cmpxchg(volatile local  ulong* p, ulong cmp, ulong val)
{
    __atomic_compare_exchange_n((volatile local ulong*)p, &cmp, val, false,
                                            __ATOMIC_RELAXED, __ATOMIC_RELAXED);
    return cmp;
}

/* 32 bits atomic min */
_CLC_OVERLOAD _CLC_DEF int  atomic_min(volatile global int*  p, int  val)
{
    return min(*p, val);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_min(volatile global uint* p, uint val)
{
    return min(*p, val);
}

_CLC_OVERLOAD _CLC_DEF int  atomic_min(volatile local  int*  p, int  val)
{
    return min(*p, val);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_min(volatile local  uint* p, uint val)
{
    return min(*p, val);
}

/* 64 bits atomic min */
_CLC_OVERLOAD _CLC_DEF long  atomic_min(volatile global long*  p, long  val)
{
    return min(*p, val);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_min(volatile global ulong* p, ulong val)
{
    return min(*p, val);
}

_CLC_OVERLOAD _CLC_DEF long  atomic_min(volatile local  long*  p, long  val)
{
    return min(*p, val);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_min(volatile local  ulong* p, ulong val)
{
    return min(*p, val);
}

/* 32 bits atomic max */
_CLC_OVERLOAD _CLC_DEF int  atomic_max(volatile global int*  p, int  val)
{
    return max(*p, val);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_max(volatile global uint* p, uint val)
{
    return max(*p, val);
}

_CLC_OVERLOAD _CLC_DEF int  atomic_max(volatile local  int*  p, int  val)
{
    return max(*p, val);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_max(volatile local  uint* p, uint val)
{
    return max(*p, val);
}

/* 64 bits atomic max */
_CLC_OVERLOAD _CLC_DEF long  atomic_max(volatile global long*  p, long  val)
{
    return max(*p, val);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_max(volatile global ulong* p, ulong val)
{
    return max(*p, val);
}

_CLC_OVERLOAD _CLC_DEF long  atomic_max(volatile local  long*  p, long  val)
{
    return max(*p, val);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_max(volatile local  ulong* p, ulong val)
{
    return max(*p, val);
}

/* 32 bits atomic and */
_CLC_OVERLOAD _CLC_DEF int  atomic_and(volatile global int*  p, int  val)
{
    return __sync_fetch_and_and(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_and(volatile global uint* p, uint val)
{
    return __sync_fetch_and_and(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF int  atomic_and(volatile local  int*  p, int  val)
{
    return __sync_fetch_and_and(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_and(volatile local  uint* p, uint val)
{
    return __sync_fetch_and_and(p, val, __ATOMIC_RELAXED);
}

/* 64 bits atomic and */
_CLC_OVERLOAD _CLC_DEF long  atomic_and(volatile global long*  p, long  val)
{
    return __sync_fetch_and_and(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_and(volatile global ulong* p, ulong val)
{
    return __sync_fetch_and_and(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF long  atomic_and(volatile local  long*  p, long  val)
{
    return __sync_fetch_and_and(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_and(volatile local  ulong* p, ulong val)
{
    return __sync_fetch_and_and(p, val, __ATOMIC_RELAXED);
}

/* 32 bits atomic or */
_CLC_OVERLOAD _CLC_DEF int  atomic_or(volatile global int*  p, int  val)
{
    return __sync_fetch_and_or(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_or(volatile global uint* p, uint val)
{
    return __sync_fetch_and_or(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF int  atomic_or(volatile local  int*  p, int  val)
{
    return __sync_fetch_and_or(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_or(volatile local  uint* p, uint val)
{
    return __sync_fetch_and_or(p, val, __ATOMIC_RELAXED);
}

/* 64 bits atomic or */
_CLC_OVERLOAD _CLC_DEF long  atomic_or(volatile global long*  p, long  val)
{
    return __sync_fetch_and_or(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_or(volatile global ulong* p, ulong val)
{
    return __sync_fetch_and_or(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF long  atomic_or(volatile local  long*  p, long  val)
{
    return __sync_fetch_and_or(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_or(volatile local  ulong* p, ulong val)
{
    return __sync_fetch_and_or(p, val, __ATOMIC_RELAXED);
}

/* 32 bits atomic xor */
_CLC_OVERLOAD _CLC_DEF int  atomic_xor(volatile global int*  p, int  val)
{
    return __sync_fetch_and_xor(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_xor(volatile global uint* p, uint val)
{
    return __sync_fetch_and_xor(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF int  atomic_xor(volatile local  int*  p, int  val)
{
    return __sync_fetch_and_xor(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF uint atomic_xor(volatile local  uint* p, uint val)
{
    return __sync_fetch_and_xor(p, val, __ATOMIC_RELAXED);
}

/* 64 bits atomic xor */
_CLC_OVERLOAD _CLC_DEF long  atomic_xor(volatile global long*  p, long  val)
{
    return __sync_fetch_and_xor(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_xor(volatile global ulong* p, ulong val)
{
    return __sync_fetch_and_xor(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF long  atomic_xor(volatile local  long*  p, long  val)
{
    return __sync_fetch_and_xor(p, val, __ATOMIC_RELAXED);
}

_CLC_OVERLOAD _CLC_DEF ulong atomic_xor(volatile local  ulong* p, ulong val)
{
    return __sync_fetch_and_xor(p, val, __ATOMIC_RELAXED);
}