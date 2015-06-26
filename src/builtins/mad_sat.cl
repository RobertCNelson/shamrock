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

_CLC_OVERLOAD _CLC_INLINE char mad_sat(char a, char b, char c)
{
    short tmp = (short)a * (short)b + (short)c;

    if (tmp > (short)CHAR_MAX) return CHAR_MAX;
    if (tmp < (short)CHAR_MIN) return CHAR_MIN;
    return tmp;
}

_CLC_OVERLOAD _CLC_INLINE uchar mad_sat(uchar a, uchar b, uchar c)
{
    ushort tmp = (ushort)a * (ushort)b + (ushort)c;

    if (tmp > (ushort)UCHAR_MAX) return UCHAR_MAX;
    return tmp;
}

_CLC_OVERLOAD _CLC_INLINE short mad_sat(short a, short b, short c)
{
    int tmp = (int)a * (int)b + (int)c;

    if (tmp > (int)SHRT_MAX) return SHRT_MAX;
    if (tmp < (int)SHRT_MIN) return SHRT_MIN;
    return tmp;
}

_CLC_OVERLOAD _CLC_INLINE ushort mad_sat(ushort a, ushort b, ushort c)
{
    uint tmp = (uint)a * (uint)b + (uint)c;

    if (tmp > (uint)USHRT_MAX) return USHRT_MAX;
    return tmp;
}

_CLC_OVERLOAD _CLC_INLINE int mad_sat(int a, int b, int c)
{
    long tmp = (long)a * (long)b + (long)c;

    if (tmp > (long)INT_MAX) return INT_MAX;
    if (tmp < (long)INT_MIN) return INT_MIN;
    return tmp;
}

_CLC_OVERLOAD _CLC_INLINE uint mad_sat(uint a, uint b, uint c)
{
    ulong tmp = (ulong)a * (ulong)b + (ulong)c;

    if (tmp > (ulong)UINT_MAX) return UINT_MAX;
    return tmp;
}

_CLC_OVERLOAD _CLC_INLINE long mad_sat(long a, long b, long c)
{
    if (a > 0 && b > 0 && a > (LONG_MAX/b)) return LONG_MAX;
    if (a > 0 && b < 0 && b < (LONG_MIN/a)) return LONG_MIN;
    if (a < 0 && b > 0 && a < (LONG_MIN/b)) return LONG_MIN;
    if (a < 0 && b < 0 && b < (LONG_MAX/a))
    {
        ulong tmp = a * b + c;
        if(tmp == 0)
            return tmp;
        else
            return LONG_MAX;
    }
    return add_sat(a*b, c);
}

_CLC_OVERLOAD _CLC_INLINE ulong mad_sat(ulong a, ulong b, ulong c)
{
    if (a > (ULONG_MAX/b)) return ULONG_MAX;
    return add_sat(a*b, c);
}

TERNARY_VEC_DEF(char, char,  mad_sat, mad_sat)
TERNARY_VEC_DEF(uchar, uchar, mad_sat, mad_sat)
TERNARY_VEC_DEF(short, short, mad_sat, mad_sat)
TERNARY_VEC_DEF(ushort, ushort,mad_sat, mad_sat)
TERNARY_VEC_DEF(int, int,   mad_sat, mad_sat)
TERNARY_VEC_DEF(uint, uint,  mad_sat, mad_sat)
TERNARY_VEC_DEF(long, long,  mad_sat, mad_sat)
TERNARY_VEC_DEF(ulong, ulong, mad_sat, mad_sat)
