__kernel void test_kernel(int4  i, __global float4 *result)
{
  result[0] = convert_float4(i);
}
