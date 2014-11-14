__kernel void test_kernel(char4 c, __global float4 *result)
{
  result[0] = convert_float4(c);
}
