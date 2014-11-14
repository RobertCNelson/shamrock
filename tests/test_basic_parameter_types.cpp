/*
 * Copyright (c) 2011, Denis Steckelmacher <steckdenis@yahoo.fr>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include "test_basic_parameter_types.h"
#include "CL/cl.h"

#define MAX_SOURCE_SIZE (0x51200)  // 50K chars max source file

START_TEST (test_basic_parameter_types)
{
    cl_platform_id platform = 0;
    cl_device_id device;
    cl_context ctx;
    cl_command_queue queue;
    cl_program program;
    cl_int error;
    cl_kernel kernel;
    cl_mem results;
    cl_char c[4]={0,1,2,3};   // input: one char4 vector.
    float results_back[1*4] = {0.0f, 0.0f, 0.0f, 0.0f};  // output: one float4 vector.
    bool ok = true;
    float expected;

    const size_t global[3] = {1, 1, 1};
    FILE *fp;
    const char fileName[] = "basic_parameter_types.cl";  // Note: cp from ~/shamrock/tests/
    size_t source_size;
    char *source_str;

    /* Load kernel source code */
    fp = fopen(fileName, "r");
    fail_if(
        fp == NULL,
        "unable to open CL file"
    );
    source_str = (char *)malloc(MAX_SOURCE_SIZE);
    source_size = fread(source_str, 1, MAX_SOURCE_SIZE, fp);
    fclose(fp);

    error = clGetDeviceIDs(platform, CL_DEVICE_TYPE_DEFAULT, 1, &device, 0);
    fail_if(
        error != CL_SUCCESS,
        "unable to get the default device"
    );

    ctx = clCreateContext(0, 1, &device, 0, 0, &error);
    fail_if(
        error != CL_SUCCESS || ctx == 0,
        "unable to create a valid context"
    );

    queue = clCreateCommandQueue(ctx, device, 0, &error);
    fail_if(
        error != CL_SUCCESS,
        "unable to create a command queue"
    );

    program = clCreateProgramWithSource(ctx, 1, (const char **)&source_str,
               (const size_t *)&source_size, &error);
    fail_if(
        error != CL_SUCCESS,
        "cannot create a program from source with sane arguments"
    );

    error = clBuildProgram(program, 1, &device, NULL, NULL, NULL);
    fail_if(
        error != CL_SUCCESS,
        "cannot build a valid program"
    );

    kernel  = clCreateKernel(program, "test_kernel", &error);
    fail_if(
        error != CL_SUCCESS,
        "unable to create a valid kernel"
    );

    // Create the results buffer
    results = clCreateBuffer(ctx, CL_MEM_READ_WRITE, sizeof(cl_float)*1*4, NULL, &error);
    fail_if(
        error != CL_SUCCESS,
        "cannot create a valid read-write buffer"
    );

    error = clSetKernelArg(kernel, 0, sizeof(cl_char)*4, &c);
    fail_if(
        error != CL_SUCCESS,
        "cannot set kernel argument"
    );
    error = clSetKernelArg(kernel, 1, sizeof(cl_mem), &results);

    error = clEnqueueNDRangeKernel(queue, kernel, 1, NULL, global, 0, 0, 0, NULL);
    fail_if(
        error != CL_SUCCESS,
        "unable to queue a NDRange kernel with local work size guessed"
    );

    // Read back the results
    error = clEnqueueReadBuffer(queue, results, CL_TRUE, 0, sizeof(cl_float)*1*4, results_back, 0, NULL, NULL);

    // Verify the results
    for (int index = 0; index < 4; index++) {
        expected = (float)c[index];
        if (results_back[index] != expected) {
            std::cout << "Conversion from char failed: got " << results_back[index] << ",expected " << expected << std::endl;
	    ok = false;
        }
    }
    fail_if(
        ok == false,
        "the kernel hasn't done its job, the buffer is wrong"
    );

    clReleaseMemObject(results);
    clReleaseKernel(kernel);
    clReleaseProgram(program);
    clReleaseContext(ctx);
}
END_TEST

TCase *cl_bpt_tcase_create(void)
{
    TCase *tc = NULL;
    tc = tcase_create("basic_parameter_types");
    tcase_add_test(tc, test_basic_parameter_types);
    return tc;
}
