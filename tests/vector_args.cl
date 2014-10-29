// RUN:  
//  % clang -c -emit-llvm -x cl -O2 -nostdinc -fno-builtin -I/usr/include vector_args.cl -o vector_args.bc
//  % llvm-dis vector_args.bc; more vector_args.ll


#include <CL/clc.h>

/*
__kernel void test_kernel%s(char%s c, uchar%s uc, short%s s, ushort%s us, int%s i, uint%s ui, float%s f,
                          __global float%s *result)
{
}
*/

__kernel void test_kernel(char c, uchar uc, short s, ushort us, int i, uint ui, float f,
                          __global float *result)
{
}

__kernel void test_kernel2(char2 c, uchar2 uc, short2 s, ushort2 us, int2 i, uint2 ui, float2 f,
                          __global float2 *result)
{
}

__kernel void test_kernel4(char4 c, uchar4 uc, short4 s, ushort4 us, int4 i, uint4 ui, float4 f,
                          __global float4 *result)
{
}

__kernel void test_kernel8(char8 c, uchar8 uc, short8 s, ushort8 us, int8 i, uint8 ui, float8 f,
                          __global float8 *result)
{
}

__kernel void test_kernel16(char16 c, uchar16 uc, short16 s, ushort16 us, int16 i, uint16 ui, float16 f,
                          __global float16 *result)
{
}
