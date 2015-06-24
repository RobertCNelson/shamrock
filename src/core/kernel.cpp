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
 * \file core/kernel.cpp
 * \brief Kernel
 */

#include "kernel.h"
#include "propertylist.h"
#include "program.h"
#include "memobject.h"
#include "sampler.h"
#include "deviceinterface.h"

#include <string>
#include <iostream>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <boost/tuple/tuple.hpp>

#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Metadata.h>
#include <llvm/IR/DataLayout.h>


using namespace Coal;
using namespace llvm;

Kernel::Kernel(Program *program)
: Object(Object::T_Kernel, program), p_has_locals(false), wi_alloca_size(0)
{
    // TODO: Say a kernel is attached to the program (that becomes unalterable)

    null_dep.device   = 0;
    null_dep.kernel   = 0;
    null_dep.function = 0;
    null_dep.module   = 0;
	p_name = "";
}

Kernel::~Kernel()
{
    while (p_device_dependent.size())
    {
        DeviceDependent &dep = p_device_dependent.back();

        delete dep.kernel;

        p_device_dependent.pop_back();
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

const Kernel::DeviceDependent &Kernel::deviceDependent(DeviceInterface *device) const
{
    for (size_t i=0; i<p_device_dependent.size(); ++i)
    {
        const DeviceDependent &rs = p_device_dependent[i];

        if (matchDeviceOrParent(rs.device, device) || (!device && p_device_dependent.size() == 1))
            return rs;
    }

    return null_dep;
}

Kernel::DeviceDependent &Kernel::deviceDependent(DeviceInterface *device)
{
    for (size_t i=0; i<p_device_dependent.size(); ++i)
    {
        DeviceDependent &rs = p_device_dependent[i];

        if (matchDeviceOrParent(rs.device, device) || (!device && p_device_dependent.size() == 1))
            return rs;
    }

    return null_dep;
}

/******************************************************************************
* cl_int Kernel::addFunction
******************************************************************************/
cl_int Kernel::addFunction(DeviceInterface *device, llvm::Function *function,
                           llvm::Module *module)
{
    llvm::DataLayout TD(module);

#if 0  // Uncomment to see the Function IR being generated:
    function->dump();
#endif

    p_name = function->getName().str();

    // Get wi_alloca_size, to be used for computing wg_alloca_size
    std::string fattrs = function->getAttributes().getAsString(
                                           llvm::AttributeSet::FunctionIndex);
    std::size_t found = fattrs.find("_wi_alloca_size=");
    if (found != std::string::npos)
        wi_alloca_size = atoi(fattrs.data() + found + 16);

    /*-------------------------------------------------------------------------
    * Add a device dependent
    *------------------------------------------------------------------------*/
    DeviceDependent dep;

    dep.device   = device;
    dep.function = function;
    dep.module   = module;

    /*-------------------------------------------------------------------------
    * Build the arg list of the kernel (or verify it if a previous function
    * was already registered)
    *------------------------------------------------------------------------*/
    llvm::FunctionType *f = function->getFunctionType();
    bool append = (p_args.size() == 0);

    if (!append && p_args.size() != f->getNumParams())
        return CL_INVALID_KERNEL_DEFINITION;

    int i = 0;
    for (llvm::Function::arg_iterator I = function->arg_begin(),
	   E = function->arg_end(); I != E; ++I, i++)
    {
        llvm::Type *param_type = f->getParamType(i);
        llvm::Argument *arg = I;
        Arg::Kind kind = Arg::Invalid;
        Arg::File file = Arg::Private;
        unsigned short vec_dim = 1;

        llvm::Type *arg_type = arg->getType();
        const unsigned arg_store_size = TD.getTypeStoreSize(arg_type);

        // LLVM IR writes parameters passed by value as pointers:
        if (llvm::isa<llvm::PointerType>(arg_type) && arg->hasByValAttr()) {
            arg_type = llvm::dyn_cast<llvm::PointerType>(arg_type)->getElementType();
        }

	llvm::Type *itype = TD.getSmallestLegalIntType(module->getContext(), arg_store_size * 8);
	llvm::Type *target_type = (itype != NULL && arg_type->isIntegerTy()) ? itype  : arg_type;

        unsigned target_size = TD.getTypeStoreSize(target_type);
        unsigned target_align = TD.getABITypeAlignment(target_type);

#if 0  // Uncomment to see arg info
        arg_type->dump(); std::cout << " Size: " << target_size << " Align: " << target_align << std::endl ;
#endif

        if (arg_type->isPointerTy())
        {
            // It's a pointer, dereference it
            llvm::PointerType *p_type = llvm::cast<llvm::PointerType>(arg_type);

            unsigned int space = p_type->getAddressSpace();
            file = (Arg::File)space;
            arg_type = p_type->getElementType();

            // If it's a __local argument, we'll have to allocate memory at run time
            if (file == Arg::Local)
                p_has_locals = true;

            kind = Arg::Buffer;

            // If it's a struct, get its name
            if (arg_type->isStructTy())
            {
                llvm::StructType *struct_type =
                    llvm::cast<llvm::StructType>(arg_type);
                std::string struct_name = struct_type->getName().str();

                if (struct_name.compare(0, 14, "struct.image2d") == 0)
                {
                    kind = Arg::Image2D;
                    file = Arg::Global;
                }
                else if (struct_name.compare(0, 14, "struct.image3d") == 0)
                {
                    kind = Arg::Image3D;
                    file = Arg::Global;
                }
            }
        }
        else
        {
            if (arg_type->isVectorTy())
            {
                // It's a vector, we need its element's type
                llvm::VectorType *v_type = llvm::cast<llvm::VectorType>(arg_type);

                vec_dim = v_type->getNumElements();
                arg_type = v_type->getElementType();
            }
            else if (arg_type->isArrayTy())
            {
                // We treat this as a vector as well.
                llvm::ArrayType *a_type = llvm::cast<llvm::ArrayType>(arg_type);

                vec_dim = a_type->getNumElements();
                arg_type = a_type->getElementType();
            }

            // Get type kind
            if (arg_type->isFloatTy())
            {
                kind = Arg::Float;
            }
            else if (arg_type->isDoubleTy())
            {
                kind = Arg::Double;
            }
            else if (arg_type->isIntegerTy())
            {
                llvm::IntegerType *i_type = llvm::cast<llvm::IntegerType>(arg_type);

                if (i_type->getBitWidth() == 8)
                {
                    kind = Arg::Int8;
                }
                else if (i_type->getBitWidth() == 16)
                {
                    kind = Arg::Int16;
                }
                else if (i_type->getBitWidth() == 32)
                {
                    // NOTE: May also be a sampler, check done in setArg
                    kind = Arg::Int32;
                }
                else if (i_type->getBitWidth() == 64)
                {
                    kind = Arg::Int64;
                }
            }
            else if (arg_type->isStructTy())
            {
                // We shouldn't need to look into the struct layout, since we
                // already know the target size and alignment.
                kind = Arg::StructType;
            }
        }

        // Check if we recognized the type
        if (kind == Arg::Invalid)
            return CL_INVALID_KERNEL_DEFINITION;

        // Create arg
        Arg *a= new Arg(vec_dim, file, kind, target_align, target_size);

        // If we also have a function registered, check for signature compliance
        if (!append && !a->sameSignature(p_args[i]))
            return CL_INVALID_KERNEL_DEFINITION;

        // Append arg if needed
        if (append)
            p_args.push_back(a);
    }

    fillArgsInfo(module);

    dep.kernel = device->createDeviceKernel(this, dep.function);
    p_device_dependent.push_back(dep);

    return CL_SUCCESS;
}

llvm::Function *Kernel::function(DeviceInterface *device) const
{
    const DeviceDependent &dep = deviceDependent(device);

    return dep.function;
}

/******************************************************************************
* cl_int Kernel::setArg
******************************************************************************/
cl_int Kernel::setArg(cl_uint index, size_t size, const void *value)
{
    if (index > p_args.size())
        return CL_INVALID_ARG_INDEX;

    Arg *arg = p_args[index];

    /*-------------------------------------------------------------------------
    * Special case for __local pointers
    *------------------------------------------------------------------------*/
    if (arg->file() == Arg::Local)
    {
        if (size == 0)  return CL_INVALID_ARG_SIZE;
        if (value != 0) return CL_INVALID_ARG_VALUE;

        arg->setAllocAtKernelRuntime(size);
        return CL_SUCCESS;
    }

    /*-------------------------------------------------------------------------
    * Check that size corresponds to the arg type
    *------------------------------------------------------------------------*/
    size_t arg_size = arg->valueSize() * arg->vecDim();

    /*-------------------------------------------------------------------------
    * Special case for samplers (pointers in C++, uint32 in OpenCL).
    *------------------------------------------------------------------------*/
    if (size == sizeof(cl_sampler) && arg_size == 4 &&
        (*(Object **)value)->isA(T_Sampler))
    {
        unsigned int bitfield = (*(Sampler **)value)->bitfield();

        arg->refineKind(Arg::Sampler);
        arg->alloc();
        arg->loadData(&bitfield, size);

        return CL_SUCCESS;
    }

    // LLVM IR redefines function parameter types to fit the smallest integer type width for the ABI
    // eg: <2xi8> (2 bytes) may actually be pushed as an i32 (4 bytes!), but this knowledge is
    // not known to shamrock.  But, we do know the parameter type alignment in addFunction().
    // So allow sizes less than or equal to the target alignment to succeed the size test:
    if ((size != arg_size) && (size > arg->targetAlignment())) return CL_INVALID_ARG_SIZE;

    /*-------------------------------------------------------------------------
    * Check for null values
    *------------------------------------------------------------------------*/
    cl_mem null_mem = 0;

    if (!value)
    {
        switch (arg->kind())
        {
            /*-------------------------------------------------------------
            * Special case buffers : value can be 0 (or point to 0)
            *------------------------------------------------------------*/
            case Arg::Buffer:
            case Arg::Image2D:
            case Arg::Image3D: value = &null_mem;
            default:           return CL_INVALID_ARG_VALUE;
        }
    }

    /*-------------------------------------------------------------------------
    * Copy just the data actually passed.  Expect LLVM to do the signext/zeroext.
    *------------------------------------------------------------------------*/
    arg->alloc();
    arg->loadData(value, size);

    return CL_SUCCESS;
}

unsigned int Kernel::numArgs() const
{
    return p_args.size();
}

const Kernel::Arg *Kernel::arg(unsigned int index) const
{
    return p_args.at(index);
}

bool Kernel::argsSpecified() const
{
    for (size_t i=0; i<p_args.size(); ++i)
        if (!p_args[i]->defined()) return false;
    return true;
}

bool Kernel::hasLocals() const
{
    return p_has_locals;
}

DeviceKernel *Kernel::deviceDependentKernel(DeviceInterface *device) const
{
    const DeviceDependent &dep = deviceDependent(device);

    return dep.kernel;
}

llvm::Module *Kernel::deviceDependentModule(DeviceInterface *device) const
{
    const DeviceDependent &dep = deviceDependent(device);

    return dep.module;
}

cl_int Kernel::info(cl_kernel_info param_name,
                    size_t param_value_size,
                    void *param_value,
                    size_t *param_value_size_ret) const
{
    void *value = 0;
    size_t value_length = 0;

    union {
        cl_uint cl_uint_var;
        cl_program cl_program_var;
        cl_context cl_context_var;
    };

    switch (param_name)
    {
        case CL_KERNEL_FUNCTION_NAME:
            MEM_ASSIGN(p_name.size() + 1, p_name.c_str());
            break;

        case CL_KERNEL_NUM_ARGS:
            SIMPLE_ASSIGN(cl_uint, p_args.size());
            break;

        case CL_KERNEL_REFERENCE_COUNT:
            SIMPLE_ASSIGN(cl_uint, references());
            break;

        case CL_KERNEL_CONTEXT:
            SIMPLE_ASSIGN(cl_context, parent()->parent());
            break;

        case CL_KERNEL_PROGRAM:
            SIMPLE_ASSIGN(cl_program, parent());
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

cl_int Kernel::argInfo(cl_uint arg_indx,
                       cl_kernel_info param_name,
                       size_t param_value_size,
                       void *param_value,
                       size_t *param_value_size_ret) const
{
    void *value = 0;
    size_t value_length = 0;

    union {
        cl_kernel_arg_address_qualifier cl_kernel_arg_address_qualifier_var;
        cl_kernel_arg_access_qualifier  cl_kernel_arg_access_qualifier_var;
        cl_kernel_arg_type_qualifier    cl_kernel_arg_type_qualifier_var;
    };

    if (!p_argsInfo.size()) {
        return (CL_KERNEL_ARG_INFO_NOT_AVAILABLE);
    }

    if (arg_indx >= p_argsInfo.size()) {
        return (CL_INVALID_ARG_INDEX);
    }

    // Pull info from ArgInfo struct private member previously stored during
    // the Kernel::addFunction() call.
    switch (param_name)
    {
        case CL_KERNEL_ARG_ADDRESS_QUALIFIER:
            SIMPLE_ASSIGN(cl_kernel_arg_address_qualifier,
                          p_argsInfo[arg_indx].address_qualifier);
            break;

        case CL_KERNEL_ARG_ACCESS_QUALIFIER:
            SIMPLE_ASSIGN(cl_kernel_arg_access_qualifier,
                          p_argsInfo[arg_indx].access_qualifier);
            break;

        case CL_KERNEL_ARG_TYPE_NAME:
            MEM_ASSIGN((p_argsInfo[arg_indx].type_name.size() + 1),
                       p_argsInfo[arg_indx].type_name.c_str());
            break;

        case CL_KERNEL_ARG_TYPE_QUALIFIER:
            SIMPLE_ASSIGN(cl_kernel_arg_type_qualifier,
                          p_argsInfo[arg_indx].type_qualifier);
            break;

        case CL_KERNEL_ARG_NAME:
            MEM_ASSIGN((p_argsInfo[arg_indx].name.size() + 1),
                       p_argsInfo[arg_indx].name.c_str());
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

/* Example OpenL kernel MetaData to parse:
!opencl.kernels = !{!0, !7}

!0 = !{void (float addrspace(1)*, float addrspace(1)*, float)* @kernel1, !1, !2, !3, !4, !5, !6}
!1 = !{!"kernel_arg_addr_space", i32 1, i32 1, i32 0}
!2 = !{!"kernel_arg_access_qual", !"none", !"none", !"none"}
!3 = !{!"kernel_arg_type", !"float*", !"float*", !"float"}
!4 = !{!"kernel_arg_base_type", !"float*", !"float*", !"float"}
!5 = !{!"kernel_arg_type_qual", !"", !"", !""}
!6 = !{!"kernel_arg_name", !"a", !"b", !"f"}
!7 = !{void (i32 addrspace(1)*)* @kernel2, !8, !9, !10, !11, !12, !13}
!8 = !{!"kernel_arg_addr_space", i32 1}
!9 = !{!"kernel_arg_access_qual", !"none"}
!10 = !{!"kernel_arg_type", !"uint*"}
!11 = !{!"kernel_arg_base_type", !"uint*"}
!12 = !{!"kernel_arg_type_qual", !""}
!13 = !{!"kernel_arg_name", !"buf"}
*/
void  Kernel::fillArgsInfo(llvm::Module *module) const
{
     const struct ArgInfo defaultArgInfo
     {
            .address_qualifier = CL_KERNEL_ARG_ADDRESS_PRIVATE,
            .access_qualifier = CL_KERNEL_ARG_ACCESS_NONE,
            .type_name = "",
            .type_qualifier =  CL_KERNEL_ARG_TYPE_NONE,
            .name = ""
     };

    if (p_argsInfo.size() > 0) {
        // Already filled, so just return;
        return;
    }

    llvm::NamedMDNode *ocl_kernels = module->getNamedMetadata("opencl.kernels");

    if (!ocl_kernels) {
        return;
    }

    // Iterate over each unnamed kernel node:
    for (NamedMDNode::op_iterator kernNode = ocl_kernels->op_begin(), E = ocl_kernels->op_end();
         kernNode != E; kernNode++)
    {
        llvm::MDNode *kernelNode = *kernNode;
        // If only one operand, that's the kernel function type, and there is no param info, so skip:
        if (1 == kernelNode->getNumOperands()) {
            continue;
        }

        // Look for the kernel prototype with the name matching ours:
        MDNode::op_iterator node = kernelNode->op_begin();
        llvm::Function *kern_signature =
          llvm::cast<llvm::Function>(dyn_cast<llvm::ValueAsMetadata>(*node)->getValue());
        std::string kern_name = kern_signature->getName().str();
        if (p_name != kern_name) {
            continue;
        }

        // Size the p_argsInfo vector to known number of args:
        p_argsInfo.resize(p_args.size(), defaultArgInfo);
        ++node;  // skip over the prototype
        for (MDNode::op_iterator endNode = (*kernNode)->op_end(); node != endNode; node++) {
            llvm::MDNode *meta_node = llvm::cast<llvm::MDNode>(*node);
            // Parse each node: get it's name, string of values, and store in p_argsInfo:

            // This parsing adapted from Mesa/clover:
            // http://patchwork.freedesktop.org/patch/40995/
            // and reference to pocl
            const uint num_ope = meta_node->getNumOperands();
            if ((num_ope - 1) != p_argsInfo.size()) {
                // this could be a different attribute, like "reqd_work_group_size"
                continue;
            }
            // metadata name
            llvm::MDString *md_name =
                        llvm::cast<llvm::MDString>(meta_node->getOperand(0));
            std::string name = md_name->getString().str();

            // metadata value
            for (uint k = 1; k < num_ope; ++k) {
                llvm::Value *value = NULL;
                if (isa<ValueAsMetadata>(meta_node->getOperand(k)))
                  value = dyn_cast<ValueAsMetadata>(meta_node->getOperand(k))->getValue();
                else if (isa<ConstantAsMetadata>(meta_node->getOperand(k)))
                  value = dyn_cast<ConstantAsMetadata>(meta_node->getOperand(k))->getValue();

                ArgInfo &arg_info = p_argsInfo[k-1];

                if (name == "kernel_arg_addr_space") {
                    int v = (llvm::cast<llvm::ConstantInt>(value))->getLimitedValue();

                    switch(v) {
                        // Note: see src/core/kernel.h for mapping.
                    case 0:
                        arg_info.address_qualifier = CL_KERNEL_ARG_ADDRESS_PRIVATE;
                        break;
                     case 1:
                        arg_info.address_qualifier = CL_KERNEL_ARG_ADDRESS_GLOBAL;
                        break;
                     case 2:
                        arg_info.address_qualifier = CL_KERNEL_ARG_ADDRESS_LOCAL;
                        break;
                     case 3:
                        arg_info.address_qualifier = CL_KERNEL_ARG_ADDRESS_CONSTANT;
                        break;
                     }
                   }
                   else {
                      llvm::MDString *m = llvm::cast<MDString>(meta_node->getOperand(k));
                      std::string v = m->getString().str();

                      if (name == "kernel_arg_access_qual") {
                         if (v == "read_only")
                            arg_info.access_qualifier =
                                                  CL_KERNEL_ARG_ACCESS_READ_ONLY;
                         else if (v == "write_only")
                            arg_info.access_qualifier =
                                                  CL_KERNEL_ARG_ACCESS_WRITE_ONLY;
                         else if (v == "read_write")
                            arg_info.access_qualifier =
                                                  CL_KERNEL_ARG_ACCESS_READ_WRITE;
                         else /*if (v == "none")*/
                            arg_info.access_qualifier =
                                                  CL_KERNEL_ARG_ACCESS_NONE;

                      }
                      else if (name == "kernel_arg_type") {
                         arg_info.type_name = v;
                      }
                      else if (name == "kernel_arg_type_qual") {
                         arg_info.type_qualifier = CL_KERNEL_ARG_TYPE_NONE;
                         if (v.find("const") != std::string::npos)
                            arg_info.type_qualifier |= CL_KERNEL_ARG_TYPE_CONST;
                         if (v.find("restrict") != std::string::npos)
                            arg_info.type_qualifier |= CL_KERNEL_ARG_TYPE_RESTRICT;
                         if (v.find("volatile") != std::string::npos)
                            arg_info.type_qualifier |= CL_KERNEL_ARG_TYPE_VOLATILE;

                      }
                      else if (name == "kernel_arg_name") {
                         arg_info.name = v;
                      }
                }
            }
        }
    }
}


boost::tuple<uint,uint,uint> Kernel::reqdWorkGroupSize(llvm::Module *module) const
{
    llvm::NamedMDNode *kernels = module->getNamedMetadata("opencl.kernels");

    boost::tuple<uint,uint,uint> zeros(0,0,0);

    if (!kernels) return zeros;

    for (unsigned int i=0; i<kernels->getNumOperands(); ++i)
    {
        llvm::MDNode *node = kernels->getOperand(i);

        /*---------------------------------------------------------------------
        * Each node has only one operand : a llvm::Function
        *--------------------------------------------------------------------*/
	llvm::Value *value = llvm::dyn_cast<llvm::ValueAsMetadata>(node->getOperand(0))->getValue();


        /*---------------------------------------------------------------------
        * Bug somewhere, don't crash
        *--------------------------------------------------------------------*/
        if (!llvm::isa<llvm::Function>(value)) continue;

        llvm::Function *f = llvm::cast<llvm::Function>(value);
        if(f->getName().str() != p_name) continue;

        if (node->getNumOperands() <= 1) return zeros;

        llvm::MDNode *meta = llvm::cast<llvm::MDNode>(node->getOperand(1));
        std::string meta_name = llvm::cast<MDString>(meta->getOperand(0))->getString().str();
        if ((meta->getNumOperands() == 4) && (meta_name == "reqd_work_group_size"))
        {
	    // See comments in http://llvm.org/docs/doxygen/html/classllvm_1_1ConstantInt.html
	    auto x = llvm::mdconst::dyn_extract<ConstantInt>(meta->getOperand(1))->getLimitedValue();
	    auto y = llvm::mdconst::dyn_extract<ConstantInt>(meta->getOperand(2))->getLimitedValue();
	    auto z = llvm::mdconst::dyn_extract<ConstantInt>(meta->getOperand(3))->getLimitedValue();

            return boost::tuple<uint,uint,uint> (x,y,z);
        }
        return zeros;
    }
    return zeros;
}


cl_int Kernel::workGroupInfo(DeviceInterface *device,
                             cl_kernel_work_group_info param_name,
                             size_t param_value_size,
                             void *param_value,
                             size_t *param_value_size_ret) const
{
    void *value = 0;
    size_t value_length = 0;

    union {
        size_t size_t_var;
        size_t three_size_t[3];
        cl_ulong cl_ulong_var;
    };

    const DeviceDependent &dep = deviceDependent(device);

    if ((!device && p_device_dependent.size() > 1) || (&dep == &null_dep))
        return CL_INVALID_DEVICE;

    switch (param_name)
    {
        case CL_KERNEL_WORK_GROUP_SIZE:
            SIMPLE_ASSIGN(size_t, dep.kernel->workGroupSize());
            break;

        case CL_KERNEL_COMPILE_WORK_GROUP_SIZE:
            {
            boost::tuple<uint,uint,uint> res(reqdWorkGroupSize(dep.module));
            three_size_t[0] = res.get<0>();
            three_size_t[1] = res.get<1>();
            three_size_t[2] = res.get<2>();
            value = &three_size_t;
            value_length = sizeof(three_size_t);
            }
            break;

        case CL_KERNEL_LOCAL_MEM_SIZE:
            SIMPLE_ASSIGN(cl_ulong, dep.kernel->localMemSize());
            break;

        case CL_KERNEL_PRIVATE_MEM_SIZE:
            SIMPLE_ASSIGN(cl_ulong, dep.kernel->privateMemSize());
            break;

        case CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE:
            SIMPLE_ASSIGN(size_t, dep.kernel->preferredWorkGroupSizeMultiple());
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

/*
 * Kernel::Arg
 */
Kernel::Arg::Arg(unsigned short vec_dim, File file, Kind kind, size_t targ_align, size_t targ_size)
  : p_vec_dim(vec_dim), p_file(file), p_kind(kind), p_targ_align(targ_align), p_targ_size(targ_size), p_data(0), p_defined(false),
  p_runtime_alloc(0)
{ }

Kernel::Arg::~Arg()
{
    if (p_data) std::free(p_data);
}

void Kernel::Arg::alloc()
{
    if (!p_data) p_data = std::calloc(p_vec_dim, valueSize());
}

void Kernel::Arg::loadData(const void *data, size_t size)
{
    assert ( size <= p_vec_dim * valueSize());
    std::memcpy(p_data, data, size);
    p_defined = true;
}

void Kernel::Arg::setAllocAtKernelRuntime(size_t size)
{
    p_runtime_alloc = size;
    p_defined       = true;
}

void Kernel::Arg::refineKind (Kernel::Arg::Kind kind)
{
    p_kind = kind;
}

bool Kernel::Arg::sameSignature(Arg *b)
{
    bool same = (p_vec_dim == b->p_vec_dim) &&
                (p_file    == b->p_file) &&
                (p_kind    == b->p_kind);

    return same;
}

size_t Kernel::Arg::valueSize() const
{
    switch (p_kind)
    {
        case Invalid: return 0;
        case Int8:    return 1;
        case Int16:   return 2;
        case Int32:
        case Sampler: return 4;
        case Int64:   return 8;
        case Float:   return sizeof(cl_float);
        case Double:  return sizeof(double);
        case Buffer:
        case Image2D:
        case Image3D: return sizeof(cl_mem);
        case StructType:
	      assert ( p_targ_size > 0);
	      return p_targ_size;
    }

    return 0;
}

unsigned short    Kernel::Arg::vecDim()    const { return p_vec_dim; }
Kernel::Arg::File Kernel::Arg::file()      const { return p_file;    }
Kernel::Arg::Kind Kernel::Arg::kind()      const { return p_kind;    }
size_t            Kernel::Arg::targetAlignment() const { return p_targ_align; }
bool              Kernel::Arg::defined()   const { return p_defined; }
const void *      Kernel::Arg::data()      const { return p_data;    }
size_t       Kernel::Arg::allocAtKernelRuntime() const {return p_runtime_alloc;}

const void *Kernel::Arg::value(unsigned short index) const
{
    const char *data = (const char *)p_data;
    unsigned int offset = index * valueSize();

    data += offset;

    return (const void *)data;
}

