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

_CLC_OVERLOAD _CLC_DECL char   sub_sat(char x, char y)
{
    int bits = CHAR_BIT * sizeof(char);
    char min = (char)1 << (char)(bits-1);
    char max = min - (char)1;
    return ((x^y) >= (char)0 ?
            /* different signs: no overflow/underflow */
            x-y : x >= (char)0 ?
                    /* x and y positive: can overflow */
                    (x > max+y ? max : x-y) :
                    /* x and y negative: can underflow */
                    (x < min+y ? min : x-y));

}

_CLC_OVERLOAD _CLC_DECL short   sub_sat(short x, short y)
{
    int bits = CHAR_BIT * sizeof(short);
    short min = (short)1 << (short)(bits-1);
    short max = min - (short)1;
    return ((x^y) >= (short)0 ?
            /* different signs: no overflow/underflow */
            x-y : x >= (short)0 ?
                    /* x and y positive: can overflow */
                    (x > max+y ? max : x-y) :
                    /* x and y negative: can underflow */
                    (x < min+y ? min : x-y));

}
_CLC_OVERLOAD _CLC_DECL int   sub_sat(int x, int y)
{
    int bits = CHAR_BIT * sizeof(int);
    int min = (int)1 << (int)(bits-1);
    int max = min - (int)1;
    return ((x^y) >= (int)0 ?
            /* different signs: no overflow/underflow */
            x-y : x >= (int)0 ?
                    /* x and y positive: can overflow */
                    (x > max+y ? max : x-y) :
                    /* x and y negative: can underflow */
                    (x < min+y ? min : x-y));

}

_CLC_OVERLOAD _CLC_DECL long   sub_sat(long x, long y)
{
    int bits = CHAR_BIT * sizeof(long);
    long min = (long)1 << (long)(bits-1);
    long max = min - (long)1;
    return ((x^y) >= (long)0 ?
            /* different signs: no overflow/underflow */
            x-y : x >= (long)0 ?
                    /* x and y positive: can overflow */
                    (x > max+y ? max : x-y) :
                    /* x and y negative: can underflow */
                    (x < min+y ? min : x-y));

}

_CLC_OVERLOAD _CLC_DECL uchar  sub_sat(uchar x, uchar y)
{
    uchar min = (uchar)0;
    return (x < min+y ? min : x-y);
}

_CLC_OVERLOAD _CLC_DECL ushort  sub_sat(ushort x, ushort y)
{
    ushort min = (ushort)0;
    return (x < min+y ? min : x-y);
}

_CLC_OVERLOAD _CLC_DECL uint  sub_sat(uint x, uint y)
{
    uint min = (uint)0;
    return (x < min+y ? min : x-y);
}

_CLC_OVERLOAD _CLC_DECL ulong  sub_sat(ulong x, ulong y)
{
    ulong min = (ulong)0;
    return (x < min+y ? min : x-y);
}

BINARY_VEC_DEF(char, char,  sub_sat, sub_sat)
BINARY_VEC_DEF(uchar, uchar, sub_sat, sub_sat)
BINARY_VEC_DEF(short, short, sub_sat, sub_sat)
BINARY_VEC_DEF(ushort, ushort,sub_sat, sub_sat)
BINARY_VEC_DEF(int, int,   sub_sat, sub_sat)
BINARY_VEC_DEF(uint, uint,  sub_sat, sub_sat)
BINARY_VEC_DEF(long, long,  sub_sat, sub_sat)
BINARY_VEC_DEF(ulong, ulong, sub_sat, sub_sat)
