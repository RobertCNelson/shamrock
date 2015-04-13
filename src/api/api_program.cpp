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

/**
 * \file api_program.cpp
 * \brief Programs
 */

#include "CL/cl.h"
#include <core/program.h>
#include <core/context.h>

#include <cstdlib>

// Program Object APIs
cl_program
clCreateProgramWithSource(cl_context        context,
                          cl_uint           count,
                          const char **     strings,
                          const size_t *    lengths,
                          cl_int *          errcode_ret)
{
    cl_int dummy_errcode;

    if (!errcode_ret)
        errcode_ret = &dummy_errcode;

    if (!context->isA(Coal::Object::T_Context))
    {
        *errcode_ret = CL_INVALID_CONTEXT;
        return 0;
    }

    if (!count || !strings)
    {
        *errcode_ret = CL_INVALID_VALUE;
        return 0;
    }

    Coal::Program *program = new Coal::Program(context);

    *errcode_ret = CL_SUCCESS;
    *errcode_ret = program->loadSources(count, strings, lengths);

    if (*errcode_ret != CL_SUCCESS)
    {
        delete program;
        return 0;
    }

    return (cl_program)program;
}

cl_program
clCreateProgramWithBinary(cl_context            context,
                          cl_uint               num_devices,
                          const cl_device_id *  device_list,
                          const size_t *        lengths,
                          const unsigned char **binaries,
                          cl_int *              binary_status,
                          cl_int *              errcode_ret)
{
    cl_int dummy_errcode;

    if (!errcode_ret)
        errcode_ret = &dummy_errcode;

    if (!context->isA(Coal::Object::T_Context))
    {
        *errcode_ret = CL_INVALID_CONTEXT;
        return 0;
    }

    if (!num_devices || !device_list || !lengths || !binaries)
    {
        *errcode_ret = CL_INVALID_VALUE;
        return 0;
    }

    // Check the devices for compliance
    cl_uint context_num_devices = 0;
    cl_device_id *context_devices;

    *errcode_ret = context->info(CL_CONTEXT_NUM_DEVICES, sizeof(cl_uint),
                                 &context_num_devices, 0);

    if (*errcode_ret != CL_SUCCESS)
        return 0;

    context_devices =
        (cl_device_id *)std::malloc(context_num_devices * sizeof(cl_device_id));

    *errcode_ret = context->info(CL_CONTEXT_DEVICES,
                                 context_num_devices * sizeof(cl_device_id),
                                 context_devices, 0);

    if (*errcode_ret != CL_SUCCESS)
        return 0;

    for (cl_uint i=0; i<num_devices; ++i)
    {
        bool found = false;

        if (!lengths[i] || !binaries[i])
        {
            if (binary_status)
                binary_status[i] = CL_INVALID_VALUE;

            *errcode_ret = CL_INVALID_VALUE;
            return 0;
        }

        for (cl_uint j=0; j<context_num_devices; ++j)
        {
            if (device_list[i] == context_devices[j])
            {
                found = true;
                break;
            }
        }

        if (!found)
        {
            *errcode_ret = CL_INVALID_DEVICE;
            return 0;
        }
    }

    // Create a program
    Coal::Program *program = new Coal::Program(context);
    *errcode_ret = CL_SUCCESS;

    // Init program
    *errcode_ret = program->loadBinaries(binaries,
                                         lengths, binary_status, num_devices,
                                         (Coal::DeviceInterface * const*)device_list);

    if (*errcode_ret != CL_SUCCESS)
    {
        delete program;
        return 0;
    }

    return (cl_program)program;
}

cl_int
clRetainProgram(cl_program program)
{
    if (!program->isA(Coal::Object::T_Program))
        return CL_INVALID_PROGRAM;

    program->reference();

    return CL_SUCCESS;
}

cl_int
clReleaseProgram(cl_program program)
{
    if (!program->isA(Coal::Object::T_Program))
        return CL_INVALID_PROGRAM;

    if (program->dereference())
        delete program;

    return CL_SUCCESS;
}

static cl_int
validateContext(Coal::Context      * context,
                cl_uint              &num_devices,
                const cl_device_id * &device_list)
{
    cl_uint context_num_devices = 0;
    cl_device_id *context_devices;
    cl_int result;

    result = context->info(CL_CONTEXT_NUM_DEVICES, sizeof(cl_uint),
                                 &context_num_devices, 0);

    if (result != CL_SUCCESS) return result;

    context_devices =
        (cl_device_id *)std::malloc(context_num_devices * sizeof(cl_device_id));

    result = context->info(CL_CONTEXT_DEVICES,
                                 context_num_devices * sizeof(cl_device_id),
                                 context_devices, 0);

    if (result != CL_SUCCESS) return result;


    // Check the devices for compliance
    if (num_devices)
    {
        for (cl_uint i=0; i<num_devices; ++i)
        {
            bool found = false;

            for (cl_uint j=0; j<context_num_devices; ++j)
            {
                if (device_list[i] == context_devices[j])
                {
                    found = true;
                    break;
                }
            }

            if (!found)
                return CL_INVALID_DEVICE;
        }
    }
    else
    {
        num_devices = context_num_devices;
        device_list = context_devices;
    }
    return CL_SUCCESS;
}


static cl_int
validateBuildArgs(cl_program           program,
                  cl_uint              &num_devices,
                  const cl_device_id * &device_list,
                  void (*pfn_notify)(cl_program program, void * user_data),
                  void *               user_data)
{
    if (!program->isA(Coal::Object::T_Program))
        return CL_INVALID_PROGRAM;

    if (!device_list && num_devices > 0)
        return CL_INVALID_VALUE;

    if (!num_devices && device_list)
        return CL_INVALID_VALUE;

    if (!pfn_notify && user_data)
        return CL_INVALID_VALUE;

    Coal::Context *context = (Coal::Context *)program->parent();
    cl_int result = validateContext(context, num_devices, device_list);
    if (result != CL_SUCCESS)
        return result;

    // We cannot try to build a previously-failed program
    if (!(program->state() == Coal::Program::Loaded ||
          program->state() == Coal::Program::Built ||
          program->state() == Coal::Program::Compiled  ))
        return CL_INVALID_OPERATION;

    return (result);
}


cl_int
clBuildProgram(cl_program           program,
               cl_uint              num_devices,
               const cl_device_id * device_list,
               const char *         options,
               void (*pfn_notify)(cl_program program, void * user_data),
               void *               user_data)
{
    cl_int result;

    result = validateBuildArgs(program, num_devices, device_list,
                               pfn_notify, user_data);

    if (result == CL_SUCCESS)  {
        // Build program
        result =  program->build(options, pfn_notify, user_data, num_devices,
                              (Coal::DeviceInterface * const*)device_list);
    }

    if (pfn_notify)
        pfn_notify(program, user_data);

    return result;
}


cl_int
clCompileProgram(cl_program           program,
                 cl_uint              num_devices,
                 const cl_device_id * device_list,
                 const char *         options,
                 cl_uint              num_input_headers,
                 const cl_program *   input_headers,
                 const char **        header_include_names,
                 void (CL_CALLBACK *  pfn_notify)(cl_program program, void * user_data),
                 void *               user_data)
{
    cl_int result = CL_SUCCESS;

    result = validateBuildArgs(program, num_devices, device_list,
                               pfn_notify,user_data);

    if ((result == CL_SUCCESS) &&
        (((num_input_headers == 0) && (header_include_names || input_headers)) ||
         (num_input_headers && (!header_include_names || !input_headers)))) {
        result = CL_INVALID_VALUE;
    }
    else {
        result = program->compile(options, pfn_notify, user_data, num_devices,
                                  (Coal::DeviceInterface * const*)device_list,
                                  num_input_headers, input_headers, header_include_names);
    }

    if (pfn_notify)
        pfn_notify(program, user_data);

    return (result);
}

cl_program
clLinkProgram(cl_context           context,
              cl_uint              num_devices,
              const cl_device_id * device_list,
              const char *         options,
              cl_uint              num_input_programs,
              const cl_program *   input_programs,
              void (CL_CALLBACK *  pfn_notify)(cl_program program, void * user_data),
              void *               user_data,
              cl_int *             errcode_ret)
{
    cl_int retcode = CL_SUCCESS;

    if (!num_input_programs || ((num_input_programs > 0) && !input_programs))
        retcode = CL_INVALID_VALUE;
    else if (!device_list && num_devices > 0)
        retcode =  CL_INVALID_VALUE;
    else if (!num_devices && device_list)
        retcode = CL_INVALID_VALUE;
    else if (!pfn_notify && user_data)
        retcode = CL_INVALID_VALUE;
    else if (!context->isA(Coal::Object::T_Context))
        retcode = CL_INVALID_CONTEXT;

    if (retcode == CL_SUCCESS) {
        retcode = validateContext(context, num_devices, device_list);
    }

    if (retcode == CL_SUCCESS) {
        // Check that each program is either loaded with a binary, or compiled
        for (int i = 0; i < num_input_programs; i++) {
            if (!input_programs[i]->isA(Coal::Object::T_Program)) {
                retcode = CL_INVALID_PROGRAM;
                break;
            }
            if (!((input_programs[i]->state() == Coal::Program::Loaded &&
                  input_programs[i]->type() == Coal::Program::Binary) ||
                  input_programs[i]->state() == Coal::Program::Compiled)) {
                retcode =  CL_INVALID_OPERATION;
                break;
            }
        }
    }

    // Create a new program object, and link input programs into it:
    Coal::Program *program = NULL;
    if (retcode == CL_SUCCESS) {
        program = new Coal::Program(context);
        retcode = program->link(options, pfn_notify, user_data, num_devices,
                                (Coal::DeviceInterface * const*)device_list,
                                 num_input_programs, input_programs);

        // Note: Unlike clCompileProgram() and clLinkProgram(), which per the 1.2 spec,
	// must trigger the callback whether the build succeeds or not, here we must have a
	// program object to pass to the pfn_notify() callback.
        if (pfn_notify)
            pfn_notify((cl_program)program, user_data);

        if (retcode != CL_SUCCESS)
        {
            delete program;
            program = NULL;
        }
    }

    if (errcode_ret) *errcode_ret = retcode;

    return (cl_program)program;
}

cl_int
clUnloadCompiler(void)
{
    return CL_SUCCESS;
}

cl_int
clGetProgramInfo(cl_program         program,
                 cl_program_info    param_name,
                 size_t             param_value_size,
                 void *             param_value,
                 size_t *           param_value_size_ret)
{
    if (!program->isA(Coal::Object::T_Program))
        return CL_INVALID_PROGRAM;

    return program->info(param_name, param_value_size, param_value,
                         param_value_size_ret);
}

cl_int
clGetProgramBuildInfo(cl_program            program,
                      cl_device_id          device,
                      cl_program_build_info param_name,
                      size_t                param_value_size,
                      void *                param_value,
                      size_t *              param_value_size_ret)
{
    if (!program->isA(Coal::Object::T_Program))
        return CL_INVALID_PROGRAM;

	if (!device)
        return CL_INVALID_DEVICE;

    return program->buildInfo((Coal::DeviceInterface *)device, param_name,
                              param_value_size, param_value,
                              param_value_size_ret);
}
