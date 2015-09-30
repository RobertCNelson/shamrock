/*
 * Copyright (c) 2011, Denis Steckelmacher <steckdenis@yahoo.fr>
 * Copyright (c) 2012-2014, Texas Instruments Incorporated - http://www.ti.com/
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
 * \file core/program.cpp
 * \brief Program
 */

#include "program.h"
#include "context.h"
#include "compiler.h"
#include "kernel.h"
#include "propertylist.h"
#include "deviceinterface.h"

#include <string>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <algorithm>
#include <sys/types.h>
#include <sys/stat.h>

#include <llvm/ADT/StringRef.h>
#include <llvm/ADT/SmallVector.h>
#include <llvm/Support/MemoryBuffer.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/Casting.h>
#include <llvm/Support/ErrorOr.h>
#include <llvm/Bitcode/ReaderWriter.h>
#include <llvm/Transforms/IPO.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/Linker/Linker.h>
#include <llvm/PassManager.h>
#include <llvm/IR/Metadata.h>
#include <llvm/IR/Function.h>
#include <llvm/Analysis/Passes.h>
#include <llvm/Transforms/IPO.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/InstIterator.h>
#include <llvm/IR/DiagnosticPrinter.h>
#include <llvm/Transforms/Utils/Cloning.h>

#include <runtime/stdlib.c.bc.embed.h>


/*-----------------------------------------------------------------------------
* temporary for source file cacheing, remove from product releases
*----------------------------------------------------------------------------*/
//#include "dsp/source_cache.h"
//source_cache * source_cache::pInstance = 0;

using namespace Coal;
using namespace llvm;

Program::Program(Context *ctx)
  : Object(Object::T_Program, ctx), p_type(Invalid), p_state(Empty),
    p_binary_type(CL_PROGRAM_BINARY_TYPE_NONE)
{
    p_null_device_dependent.compiler = 0;
    p_null_device_dependent.device = 0;
    p_null_device_dependent.linked_module = 0;
    p_null_device_dependent.program = 0;
}

Program::~Program()
{
   resetDeviceDependent();
}

void Program::resetDeviceDependent()
{
    while (p_device_dependent.size())
    {
        DeviceDependent &dep = p_device_dependent.back();

        delete dep.compiler;
        delete dep.program;
        delete dep.linked_module;
        dep.unlinked_binary.clear();

        p_device_dependent.pop_back();
    }
}

void Program::setDevices(cl_uint num_devices, DeviceInterface * const*devices)
{
    p_device_dependent.resize(num_devices);

    for (cl_uint i=0; i<num_devices; ++i)
    {
        DeviceDependent &dep = p_device_dependent[i];

        dep.device           = devices[i];
        dep.program          = dep.device->createDeviceProgram(this);
        dep.is_native_binary = false;
        dep.linked_module    = 0;
        dep.compiler         = new Compiler(dep.device);
        dep.unlinked_binary.clear();
    }
}

static bool matchDeviceOrParent(DeviceInterface *device_dep, DeviceInterface *device)
{
    bool match = (device_dep == device);
    DeviceInterface *next_device = device->parentDevice();

    // If no match, device could be a sub-device - so go up the hierarchy checking parents:
    while (!match && next_device) {
        match = (device_dep == next_device);
        next_device = next_device->parentDevice();
    }
    return match;
}


Program::DeviceDependent &Program::deviceDependent(DeviceInterface *device)
{
    for (size_t i=0; i<p_device_dependent.size(); ++i)
    {
        DeviceDependent &rs = p_device_dependent[i];

        if (matchDeviceOrParent(rs.device, device) || (!device && p_device_dependent.size() == 1))
            return rs;
    }

    return p_null_device_dependent;
}

const Program::DeviceDependent &Program::deviceDependent(DeviceInterface *device) const
{
    for (size_t i=0; i<p_device_dependent.size(); ++i)
    {
        const DeviceDependent &rs = p_device_dependent[i];

        if (matchDeviceOrParent(rs.device, device) || (!device && p_device_dependent.size() == 1))
            return rs;
    }

    return p_null_device_dependent;
}

DeviceProgram *Program::deviceDependentProgram(DeviceInterface *device) const
{
    const DeviceDependent &dep = deviceDependent(device);

    return dep.program;
}

std::string Program::deviceDependentCompilerOptions(DeviceInterface *device) const
{
    const DeviceDependent &dep = deviceDependent(device);

    return dep.compiler->options();
}

std::vector<llvm::Function *> Program::kernelFunctions(DeviceDependent &dep)
{
    std::vector<llvm::Function *> rs;

    llvm::NamedMDNode *kernels =
               dep.linked_module->getNamedMetadata("opencl.kernels");

    if (!kernels) return rs;

    for (unsigned int i=0; i<kernels->getNumOperands(); ++i)
    {
        llvm::MDNode *node = kernels->getOperand(i);

        /*---------------------------------------------------------------------
        * Each node has only one operand : a llvm::Function
        *--------------------------------------------------------------------*/
        llvm::Value *value = dyn_cast<llvm::ValueAsMetadata>(node->getOperand(0))->getValue();

        /*---------------------------------------------------------------------
        * Bug somewhere, don't crash
        *--------------------------------------------------------------------*/
        if (!llvm::isa<llvm::Function>(value)) continue;

        llvm::Function *f = llvm::cast<llvm::Function>(value);
        rs.push_back(f);
    }

    return rs;
}

/******************************************************************************
* Kernel *Program::createKernel(const std::string &name, cl_int *errcode_ret)
******************************************************************************/
Kernel *Program::createKernel(const std::string &name, cl_int *errcode_ret)
{
    Kernel *rs = NULL;

	for (size_t i=0; i < kernelList.size(); i++)
    {
        if (kernelList[i]->p_name.compare(name) == 0)
        {
            *errcode_ret = CL_SUCCESS;
			return kernelList[i];
        }
    }
	/* Now check the previously released list */
	for (size_t i=0; i < kernelReleasedList.size(); i++)
    {
        if (kernelReleasedList[i]->p_name.compare(name) == 0)
        {
            *errcode_ret = CL_SUCCESS;
			rs = kernelReleasedList[i];
			kernelReleasedList.erase(kernelReleasedList.begin() + i);
			kernelList.push_back(rs);

			return rs;
        }
    }

    rs = new Kernel(this);

    /*-------------------------------------------------------------------------
    * Add a function definition for each device
    *------------------------------------------------------------------------*/
    for (size_t i=0; i < p_device_dependent.size(); ++i)
    {
        bool found = false;
        DeviceDependent &dep = p_device_dependent[i];
        const std::vector<llvm::Function *> &kernels = kernelFunctions(dep);

        /*---------------------------------------------------------------------
        * Find the one with the good name
        *--------------------------------------------------------------------*/
        for (size_t j=0; j < kernels.size(); ++j)
        {
            llvm::Function *func = kernels[j];

            if (func->getName().str().compare(name) == 0)
            {
                found = true;
                *errcode_ret = rs->addFunction(dep.device, func,
                                               dep.linked_module);
                if (*errcode_ret != CL_SUCCESS) return rs;
                break;
            }
        }

        /*---------------------------------------------------------------------
        * Kernel unavailable for this device
        *--------------------------------------------------------------------*/
        if (!found)
        {
            *errcode_ret = CL_INVALID_KERNEL_NAME;
            return rs;
        }
        else
        {
            kernelList.push_back(rs);
        }
    }

    return rs;
}

Kernel * Program::createKernelsAndReturnKernel(const std::string &name, cl_int *errcode_ret)
{
    Kernel *rs = NULL;
    /*-------------------------------------------------------------------------
    * We should never go here
    *------------------------------------------------------------------------*/
    if (p_device_dependent.size() == 0) return rs;


	for (size_t i=0; i < kernelList.size(); i++)
    {
        if (kernelList[i]->p_name.compare(name) == 0)
        {
            *errcode_ret = CL_SUCCESS;
			return kernelList[i];
        }
    }
	/* Now check the previously released list */
	for (size_t i=0; i < kernelReleasedList.size(); i++)
    {
        if (kernelReleasedList[i]->p_name.compare(name) == 0)
        {
            *errcode_ret = CL_SUCCESS;
			rs = kernelReleasedList[i];
			kernelReleasedList.erase(kernelReleasedList.begin() + i);
			kernelList.push_back(rs);

			return rs;
        }
	}

    /*-------------------------------------------------------------------------
    * Take the list of kernels for the first device dependent
    *------------------------------------------------------------------------*/
    DeviceDependent &dep = p_device_dependent[0];
    const std::vector<llvm::Function *> &kernels = kernelFunctions(dep);

    /*-------------------------------------------------------------------------
    * Create the kernel for each function name
    * It returns an error if the signature is not the same for every device
    * or if the kernel isn't found on all the devices.
    *------------------------------------------------------------------------*/
    *errcode_ret = CL_SUCCESS;

    for (size_t i=0; i < kernels.size(); ++i)
    {
        cl_int result  = CL_SUCCESS;
        Kernel *kernel = createKernel(kernels[i]->getName().str(), &result);

        if (result == CL_SUCCESS)
        {
        }
        else
        {
            *errcode_ret = result;
            delete kernel;
        }
        if (kernel->p_name.compare(name) == 0 && result == CL_SUCCESS)
        {
           rs = kernel;
           *errcode_ret = result;
        }
    }

	if (!rs && (*errcode_ret == CL_SUCCESS))
		*errcode_ret = CL_INVALID_KERNEL_NAME;

    return rs;
}

std::vector<Kernel *> Program::createKernels(cl_int *errcode_ret)
{
    std::vector<Kernel *> rs;
    Kernel *kern = NULL;
    size_t i = 0;

    /*-------------------------------------------------------------------------
    * We should never go here
    *------------------------------------------------------------------------*/
    if (p_device_dependent.size() == 0) return rs;

    /*
     * Resurrect any released kernels back to the kernel list.  This handles the
     * case where clCreateKernelsInProgram() is asking only for a count of kernels in
     * the currently built program.  In that case, KernelList.size() must be the actual
     * number of kernels compiled into the program (event if they were previously released).
     */
    //for (size_t i=0; i < kernelReleasedList.size(); i++)
    while(kernelReleasedList.size())
    {
        kern = kernelReleasedList[i];
        kernelReleasedList.erase(kernelReleasedList.begin() + i);
        kernelList.push_back(kern);
    }

    if (kernelList.size()) return kernelList;

    /*-------------------------------------------------------------------------
    * Take the list of kernels for the first device dependent
    *------------------------------------------------------------------------*/
    DeviceDependent &dep = p_device_dependent[0];
    const std::vector<llvm::Function *> &kernels = kernelFunctions(dep);

    /*-------------------------------------------------------------------------
    * Create the kernel for each function name
    * It returns an error if the signature is not the same for every device
    * or if the kernel isn't found on all the devices.
    *------------------------------------------------------------------------*/
    for (size_t i=0; i < kernels.size(); ++i)
    {
        cl_int result  = CL_SUCCESS;
        Kernel *kernel = createKernel(kernels[i]->getName().str(), &result);
        if(result != CL_SUCCESS)
        {
            *errcode_ret = result;
            delete kernel;
            break;
        }
    }

    return kernelList;
}

unsigned int Program::getNumKernels() const
{

    if (p_device_dependent.size() == 0) return 0;

    const DeviceDependent &dep = p_device_dependent[0];
    llvm::NamedMDNode *kernels =
               dep.linked_module->getNamedMetadata("opencl.kernels");

    if (!kernels) return 0;
    else return kernels->getNumOperands();
}

std::string Program::getKernelNames() const
{
    std::string retString = "";

    if (p_device_dependent.size() == 0) return retString;

    const DeviceDependent &dep = p_device_dependent[0];
    llvm::NamedMDNode *kernels =
               dep.linked_module->getNamedMetadata("opencl.kernels");

    if (!kernels) return retString;
    else  {
        for (unsigned int i=0; i<kernels->getNumOperands(); i++) {
            llvm::MDNode *node = kernels->getOperand(i);

            llvm::Function *kern_signature =
              llvm::cast<llvm::Function>(
                  dyn_cast<llvm::ValueAsMetadata>(
                        node->getOperand(0))->getValue());
            std::string kern_name = kern_signature->getName().str();
            if (i > 0) retString += ";";
            retString += kern_name;
        }
    }
    return retString;
}


cl_int Program::loadSources(cl_uint count, const char **strings,
                            const size_t *lengths)
{
    // Initialize
    p_source  = std::string("");

    // Merge all strings into one big one
    for (cl_uint i=0; i<count; ++i)
    {
        size_t len = 0;
        const char *data = strings[i];

        if (!data)
            return CL_INVALID_VALUE;

        // Get the length of the source
        if (lengths && lengths[i])
            len = lengths[i];
        else
            len = std::strlen(data);

        // Remove trailing \0's, it's not good for sources (it can arise when
        // the client application wrongly sets lengths
        while (len > 0 && data[len-1] == 0)
            len--;

        // Merge the string
        std::string part(data, len);
        p_source += part;
    }

    /*-------------------------------------------------------------------------
    * temporary for source file cacheing, remove from product releases
    *------------------------------------------------------------------------*/
    //source_cache::instance()->remember(p_source);

    p_type = Source;
    p_state = Loaded;

    return CL_SUCCESS;
}

cl_int Program::loadBinaries(const unsigned char **data, const size_t *lengths,
                             cl_int *binary_status, cl_uint num_devices,
                             DeviceInterface * const*device_list)
{
    // Set device infos
    setDevices(num_devices, device_list);

    // Load the data
    for (cl_uint i=0; i<num_devices; ++i)
    {
        DeviceDependent &dep = deviceDependent(device_list[i]);
        dep.unlinked_binary = std::string((const char *)data[i], lengths[i]);
        dep.is_native_binary = true;

        /*--------------------------------------------------------------------
        * Loaded binary is either native code with LLVM bitcode embedded,
        *                  or     LLVM bitcode itself
        *--------------------------------------------------------------------*/
        std::string bitcode;
        if (! dep.program->ExtractMixedBinary(&dep.unlinked_binary, &bitcode,
                                              NULL))
        {
            bitcode = dep.unlinked_binary;
            dep.is_native_binary = false;
        }

        const llvm::StringRef s_data(bitcode);
        const llvm::StringRef s_name("<binary>");

        std::unique_ptr<llvm::MemoryBuffer> buffer =
	  llvm::MemoryBuffer::getMemBuffer(s_data, s_name, false);

        if (!buffer)
            return CL_OUT_OF_HOST_MEMORY;

        // Make a module of it
        ErrorOr<Module *> ModuleOrErr = parseBitcodeFile(buffer->getMemBufferRef(),
						  llvm::getGlobalContext());
        if (ModuleOrErr) {
             dep.linked_module = ModuleOrErr.get();
        }
        else {
            dep.linked_module = NULL;
            if (binary_status) binary_status[i] = CL_INVALID_VALUE;
            return CL_INVALID_BINARY;
        }

        if (binary_status) binary_status[i] = CL_SUCCESS;
    }

    p_type = Binary;
    p_state = Loaded;
    // binary_type could also be CL_PROGRAM_BINARY_TYPE_LIBRARY or
    // CL_PROGRAM_BINARY_TYPE_EXECUTABLE, but need a foolproof way to distinguish
    // that based on the bits actually passed in.
    p_binary_type = CL_PROGRAM_BINARY_TYPE_COMPILED_OBJECT;

    return CL_SUCCESS;
}

// "Embedded Header" processing.
// Seeing no better alternative, let's place passed in header source into a /tmp directory.
// Note this will work for relative paths used in the #include directive, which
// is the typical case.
const std::string headers_dir = "tmp/.shamrock";
const std::string tmp_headers_path = "/" + headers_dir + "/";

void mkdirTree(std::string sub, std::string dir)
{
    int i;
    if (sub.length() == 0)
        return;

    for (i=0; i<sub.length(); i++){
        dir += sub[i];
        if (sub[i] == '/')
            break;
    }
    mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (i+1 < sub.length())
        mkdirTree(sub.substr(i+1), dir);
}

static int storeInputHeaders(cl_uint num_input_headers,
                     const cl_program *input_headers,
                     const char **header_include_names)
{
    int ret = 0;

    if (num_input_headers > 0) {
        mkdirTree(headers_dir, "/");
    }
    for (int i = 0; i < num_input_headers; i++) {
        std::string pathname = header_include_names[i];
        int end;
        // If the include path is a tree of subdirectories, make those first:
        if ((end = pathname.rfind("/")) != std::string::npos) {
            std::string header_path = pathname.substr(0, end);
            mkdirTree(header_path, tmp_headers_path);
        }

        // Make tmp header file...
        // and copy header file contents there (previously stored in the program object)
        Coal::Program *prog = (Coal::Program *)input_headers[i];
        assert (prog->type() == Coal::Program::Source);
	std::string fullpath = tmp_headers_path + pathname;
        try
        {
            std::ofstream header_file(fullpath, std::ios::out);
            header_file << prog->source();
            header_file.close();
        }
        catch(...) {
            std::cout << "Could not write: " << fullpath << std::endl;
            ret = 1;
        }
    }
    return (ret);
}

static void removeInputHeaders(cl_uint num_input_headers,
                     const char **header_include_names)
{
    for (int i = 0; i < num_input_headers; i++) {
        std::string pathname = tmp_headers_path;
        pathname += header_include_names[i];
        remove(pathname.c_str());
    }
    if (num_input_headers > 0) {
        remove(headers_dir.c_str());
    }
}


cl_int Program::compile(const char *options,
                     void (CL_CALLBACK *pfn_notify)(cl_program program,
                                                    void *user_data),
                     void *user_data, cl_uint num_devices,
                     DeviceInterface * const*device_list,
                     cl_uint num_input_headers,
                     const cl_program *input_headers,
                     const char **header_include_names)
{
    cl_int retcode = CL_SUCCESS;

#if 0
    // If we've already built this program and are re-building
    // (for example, with different user options) then clear out the
    // device dependent information in preparation for building again.
    if( p_state == Built || p_state == Compiled) resetDeviceDependent();
#else
    // Per the v1.2 spec for clBuildProgram() and clCompileProgram():
    // "Returns: CL_INVALID_OPERATION if there are kernel objects attached to program."
    // Note this causes some Khronos tests to fail, like test_compiler options_build_macro,
    // options_build_macro_existence, and options_include_directory, and
    // test_api binary_create, as they all violate the spec in this regard.
    if (getNumKernels() > 0) {
        p_state = Failed;
        return(CL_INVALID_OPERATION);
    }
#endif

    p_state = Failed;

    // Set device infos
    if (!p_device_dependent.size())
    {
        setDevices(num_devices, device_list);
    }

    std::string comp_opts = options ? options : std::string();

    // Place the input headers in a special directory under /tmp so compiler can find them
    if (storeInputHeaders(num_input_headers, input_headers, header_include_names)) {
        retcode = CL_BUILD_PROGRAM_FAILURE;
	goto cleanup;
    }
    else if (input_headers) {
        comp_opts =  std::string("-I /") + headers_dir + std::string(" ") + comp_opts;
    }

    // ASW TODO - optimize to compile for each device type only once.
    for (cl_uint i=0; i<p_device_dependent.size(); ++i)
    {
        DeviceDependent &dep = deviceDependent(device_list[i]);

        // Do we need to compile the source for each device ?
        if (p_type == Source)
        {
            // Load source
            const llvm::StringRef s_data(p_source);
            const llvm::StringRef s_name("<source>");

            std::unique_ptr<llvm::MemoryBuffer> buffer =
	      llvm::MemoryBuffer::getMemBuffer(s_data, s_name);

            // Compile
	    int compile_result = dep.compiler->compile(comp_opts, buffer.get());
	    if (compile_result)
            {
                if (compile_result == CL_INVALID_BUILD_OPTIONS) {
                    retcode = CL_INVALID_BUILD_OPTIONS;
		    goto cleanup;
		}
                else {
                    retcode = CL_COMPILE_PROGRAM_FAILURE;
		    goto cleanup;
		}
            }

            // Get module and its bitcode
            dep.linked_module = dep.compiler->module();

            llvm::raw_string_ostream ostream(dep.unlinked_binary);
            llvm::WriteBitcodeToFile(dep.linked_module, ostream);
            ostream.flush();
        }
    }

cleanup:
    removeInputHeaders(num_input_headers, header_include_names);

    if (retcode == CL_SUCCESS) {
        p_state = Compiled;
        p_binary_type = CL_PROGRAM_BINARY_TYPE_COMPILED_OBJECT;
    }
    return retcode;
}


cl_int Program::link(const char *options,
                     void (CL_CALLBACK *pfn_notify)(cl_program program,
                                                    void *user_data),
                     void *user_data, cl_uint num_devices,
                     DeviceInterface * const * device_list,
                     cl_uint num_input_programs,
		     const cl_program * input_programs)
{
    bool linkAsLibrary = false;

    p_state = Failed;

    if (options && strstr(options, "-create-library")) {
        linkAsLibrary = true;
    }

    // Set device infos
    if (!p_device_dependent.size())
    {
        setDevices(num_devices, device_list);
    }

    for (cl_uint i=0; i<p_device_dependent.size(); ++i)  {
        DeviceDependent &dep = deviceDependent(device_list[i]);

        // Link with the stdlib if the device needs that
        Module *stdlib = NULL;
        if (!dep.is_native_binary && dep.program->linkStdLib() && !linkAsLibrary)
        {
            // Load the stdlib bitcode
            const llvm::StringRef s_data(embed_stdlib_c_bc,
                                         sizeof(embed_stdlib_c_bc) - 1);
            const llvm::StringRef s_name("stdlib.bc");
            std::string errMsg;

            std::unique_ptr<llvm::MemoryBuffer> buffer =
	      llvm::MemoryBuffer::getMemBuffer(s_data, s_name, false);

            if (!buffer)
                return CL_OUT_OF_HOST_MEMORY;

            ErrorOr<Module *> ModuleOrErr =
                    parseBitcodeFile(buffer->getMemBufferRef(), llvm::getGlobalContext());
            if (ModuleOrErr) {
                 stdlib =  ModuleOrErr.get();
            }
            else {
	         std::error_code EC = ModuleOrErr.getError();
                 errMsg = EC.message();
            }
	}

	int start = 0;
	// If no module, initialize with the first input program:
	if (!dep.linked_module) {
	    dep.linked_module = CloneModule(
	      ((Coal::Program *)input_programs[0])->deviceDependent(device_list[i]).linked_module);
	    start = 1;
            if (stdlib) num_input_programs++;
	}

	// Link programs into this program:
        std::string errMsg;

	for (int j = start; j < num_input_programs; j++) {
	    Module *other;
            if (j == (num_input_programs-1) && stdlib) {
                other = stdlib;
            }
            else {
                DeviceDependent &other_dep =
                     ((Coal::Program *)input_programs[j])->deviceDependent(device_list[i]);
                other = other_dep.linked_module;
            }

            // Link
	    std::string Message;
	    raw_string_ostream Stream(Message);
	    DiagnosticPrinterRawOStream DP(Stream);

	    LLVMBool Result = Linker::LinkModules(dep.linked_module, other,
		    [&](const DiagnosticInfo &DI) { DI.print(DP); });
	    if (Result) {
		errMsg += strdup(Message.c_str());
                dep.compiler->appendLog("link error: ");
                dep.compiler->appendLog(errMsg);
                dep.compiler->appendLog("\n");

                // DEBUG
                std::cout << dep.compiler->log() << std::endl;

                return CL_BUILD_PROGRAM_FAILURE;
            }
	}


        if (!dep.is_native_binary && !linkAsLibrary)
        {
            // Get list of kernels to strip other unused functions
            std::vector<const char *> api;
            std::vector<std::string> api_s; // Needed to keep valid data in api
            const std::vector<llvm::Function *> &kernels = kernelFunctions(dep);

            for (size_t j=0; j<kernels.size(); ++j)
            {
                std::string s = kernels[j]->getName().str();
                api_s.push_back(s);
                api.push_back(s.c_str());
            }

            // determine if module has barrier() function calls
            bool hasBarrier = false;
            llvm::CallInst* call;
            for (llvm::Module::iterator F = dep.linked_module->begin(),
                    EF = dep.linked_module->end(); !hasBarrier && F != EF; ++F)
                for (llvm::inst_iterator I = inst_begin(*F),
                                         E = inst_end(*F); I != E; ++I)
                {
                    if (!(call = llvm::dyn_cast<llvm::CallInst>(&*I))) continue;
                    if (!call->getCalledFunction())                    continue;
                    std::string name(call->getCalledFunction()->getName());
                    if (name == "barrier")
                    {
                        hasBarrier = true;
                        break;
                    }
                }

            // Optimize code
            llvm::PassManager *manager = new llvm::PassManager();

            // Common passes (primary goal : remove unused stdlib functions)
            manager->add(llvm::createTypeBasedAliasAnalysisPass());
            manager->add(llvm::createBasicAliasAnalysisPass());
            manager->add(llvm::createInternalizePass(api));
            manager->add(llvm::createIPSCCPPass());
            manager->add(llvm::createGlobalOptimizerPass());
            manager->add(llvm::createConstantMergePass());
            manager->add(llvm::createAlwaysInlinerPass());

            dep.program->createOptimizationPasses(manager,
                                       dep.compiler->optimize(), hasBarrier);

            manager->add(llvm::createGlobalDCEPass());

            manager->run(*dep.linked_module);
            delete manager;
        }

        // Now that the LLVM module is built, build the device-specific
        // representation
        if (!dep.program->build(dep.linked_module, &dep.unlinked_binary))
        {
            return CL_BUILD_PROGRAM_FAILURE;
        }
    }

    p_state = (linkAsLibrary? Compiled : Built);
    p_binary_type = (linkAsLibrary? CL_PROGRAM_BINARY_TYPE_LIBRARY :
                      CL_PROGRAM_BINARY_TYPE_EXECUTABLE);
    return CL_SUCCESS;
}

cl_int Program::build(const char *options,
                      void (CL_CALLBACK *pfn_notify)(cl_program program,
                                                     void *user_data),
                      void *user_data, cl_uint num_devices,
                      DeviceInterface * const*device_list)
{
    cl_int result;

    result = compile(options, pfn_notify, user_data, num_devices, device_list,
                     0, NULL, NULL);

    if (result == CL_SUCCESS) {
        cl_uint num_input_programs = 1;
        const cl_program input_programs[] = { (cl_program)this };

        result = link(options, pfn_notify, user_data, num_devices,
                      device_list, num_input_programs, input_programs);
    }
    else if (result == CL_COMPILE_PROGRAM_FAILURE) {
        result = CL_BUILD_PROGRAM_FAILURE;  // clBuildProgram expects this error code.
    }

    return result;
}

Program::Type Program::type() const
{
    return p_type;
}

Program::State Program::state() const
{
    return p_state;
}

cl_int Program::info(cl_program_info param_name,
                     size_t param_value_size,
                     void *param_value,
                     size_t *param_value_size_ret) const
{
    void *value = 0;
    size_t value_length = 0;
    llvm::SmallVector<size_t, 4> binary_sizes;
    llvm::SmallVector<cl_device_id, 4> devices;
    std::string names;

    union {
        cl_uint cl_uint_var;
        cl_context cl_context_var;
        size_t size_t_var;
    };

    switch (param_name)
    {
        case CL_PROGRAM_REFERENCE_COUNT:
            SIMPLE_ASSIGN(cl_uint, references());
            break;

        case CL_PROGRAM_NUM_DEVICES:
            // Use devices associated with any built kernels, otherwise use
            // the devices associated with the program context
            if (p_device_dependent.size() != 0)
               { SIMPLE_ASSIGN(cl_uint, p_device_dependent.size()); }
            else
               return ((Context *)parent())->info(CL_CONTEXT_NUM_DEVICES,
                           param_value_size, param_value, param_value_size_ret);
            break;

        case CL_PROGRAM_DEVICES:
            // Use devices associated with any built kernels, otherwise use
            // the devices associated with the program context
            if (p_device_dependent.size() != 0)
            {
               for (size_t i=0; i<p_device_dependent.size(); ++i)
               {
                  const DeviceDependent &dep = p_device_dependent[i];

                  devices.push_back(desc(dep.device));
               }

               value = devices.data();
               value_length = devices.size() * sizeof(cl_device_id);
           }
           else
              return ((Context *)parent())->info(CL_CONTEXT_DEVICES,
                           param_value_size, param_value, param_value_size_ret);
           break;

        case CL_PROGRAM_CONTEXT:
	    SIMPLE_ASSIGN(cl_context, desc((Context *)parent()));
            break;

        case CL_PROGRAM_SOURCE:
            MEM_ASSIGN(p_source.size() + 1, p_source.c_str());
            break;

        case CL_PROGRAM_BINARY_SIZES:
            for (size_t i=0; i<p_device_dependent.size(); ++i)
            {
                DeviceDependent &dep = const_cast<Coal::Program::DeviceDependent&>
                                          (p_device_dependent[i]);

                if (!dep.linked_module) {
                    binary_sizes.push_back(0);
                }
                else if (dep.unlinked_binary.empty()) {
                    llvm::raw_string_ostream ostream(dep.unlinked_binary);
                    llvm::WriteBitcodeToFile(dep.linked_module, ostream);
                    ostream.flush();
                    binary_sizes.push_back(dep.unlinked_binary.size());
                }
                else {
                    binary_sizes.push_back(dep.unlinked_binary.size());
                }
            }

            value = binary_sizes.data();
            value_length = binary_sizes.size() * sizeof(size_t);
            break;

        case CL_PROGRAM_BINARIES:
            {
            // Special case : param_value points to an array of p_num_devices
            // application-allocated unsigned char* pointers. Check it's good
            // and std::memcpy the data

            unsigned char **binaries = (unsigned char **)param_value;
            value_length = p_device_dependent.size() * sizeof(unsigned char *);

            if (param_value && param_value_size >= value_length)
                for (size_t i=0; i<p_device_dependent.size(); ++i)
                {
                    DeviceDependent &dep = const_cast<Coal::Program::DeviceDependent&>
                                          (p_device_dependent[i]);
                    unsigned char *dest = binaries[i];

                    if (!dest)
                        continue;

                    if (dep.unlinked_binary.empty()) {
                        llvm::raw_string_ostream ostream(dep.unlinked_binary);
                        llvm::WriteBitcodeToFile(dep.linked_module, ostream);
                        ostream.flush();
                    }

                    std::memcpy(dest, dep.unlinked_binary.data(),
                                dep.unlinked_binary.size());
                }

            if (param_value_size_ret)
                *param_value_size_ret = value_length;

            return CL_SUCCESS;
            }

        case CL_PROGRAM_NUM_KERNELS:
            SIMPLE_ASSIGN(size_t, getNumKernels());
            break;

        case CL_PROGRAM_KERNEL_NAMES:
            names = getKernelNames();
            MEM_ASSIGN(names.size()+1, names.c_str());
            break;
        default:
            return CL_INVALID_VALUE;
    }

    if (param_value && param_value_size < value_length)
        return CL_INVALID_VALUE;

    if (param_value_size_ret)
        *param_value_size_ret = value_length;

    if (param_value)
        std::memcpy(param_value, value, value_length);

    return CL_SUCCESS;
}

cl_int Program::buildInfo(DeviceInterface *device,
                          cl_program_build_info param_name,
                          size_t param_value_size,
                          void *param_value,
                          size_t *param_value_size_ret) const
{
    const void *value = 0;
    size_t value_length = 0;
    const DeviceDependent &dep = deviceDependent(device);

    union {
        cl_build_status cl_build_status_var;
        cl_program_binary_type  cl_program_binary_type_var;
    };

    switch (param_name)
    {
        case CL_PROGRAM_BUILD_STATUS:
            switch (p_state)
            {
                case Empty:
                case Loaded:
                    SIMPLE_ASSIGN(cl_build_status, CL_BUILD_NONE);
                    break;
                case Built:
                case Compiled:
                    SIMPLE_ASSIGN(cl_build_status, CL_BUILD_SUCCESS);
                    break;
                case Failed:
                    SIMPLE_ASSIGN(cl_build_status, CL_BUILD_ERROR);
                    break;
                // TODO: CL_BUILD_IN_PROGRESS
            }
            break;

        case CL_PROGRAM_BUILD_OPTIONS:
            value = dep.compiler->options().c_str();
            value_length = dep.compiler->options().size() + 1;
            break;

        case CL_PROGRAM_BUILD_LOG:
            value = dep.compiler->log().c_str();
            value_length = dep.compiler->log().size() + 1;
            break;

        case CL_PROGRAM_BINARY_TYPE:
	    SIMPLE_ASSIGN(cl_program_binary_type, p_binary_type);
            break;
        default:
            return CL_INVALID_VALUE;
    }

    if (param_value && param_value_size < value_length)
        return CL_INVALID_VALUE;

    if (param_value_size_ret)
        *param_value_size_ret = value_length;

    if (param_value)
        std::memcpy(param_value, value, value_length);

    return CL_SUCCESS;
}
