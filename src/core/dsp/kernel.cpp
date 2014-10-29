/******************************************************************************
 * Copyright (c) 2013-2014, Texas Instruments Incorporated - http://www.ti.com/
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
#include "kernel.h"
#include "device.h"
#include "buffer.h"
#include "program.h"
#include "utils.h"
#include "u_locks_pthread.h"
#include "mailbox.h"

#include "../kernel.h"
#include "../memobject.h"
#include "../events.h"
#include "../program.h"

#include <llvm/IR/Function.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>
#include <sys/mman.h>

extern "C"
{
    #include <ti/runtime/mmap/include/mmap_resource.h>
}


#define ROUNDUP(val, pow2)   (((val) + (pow2) - 1) & ~((pow2) - 1))
#define QERR(msg, retcode)   do {if (getenv("TI_OCL_VERBOSE_ERROR")) std::cerr << msg << std::endl; return retcode; } while(0)
#define ERR(x) std::cerr << x << std::endl
#define ERROR() std::cerr << "Unknown error in dsp/kernel.cpp" << std::endl

using namespace Coal;

DSPKernel::DSPKernel(DSPDevice *device, Kernel *kernel)
: DeviceKernel(), p_device(device), p_kernel(kernel), 
    p_device_entry_pt((DSPDevicePtr)0),
    p_data_page_ptr  ((DSPDevicePtr)0xffffffff)
{
}

DSPKernel::~DSPKernel()
{
}


template<typename T>
T k_exp(T base, unsigned int e)
{
    T rs = base;
    for (unsigned int i=1; i<e; ++i) rs *= base;
    return rs;
}

/*-----------------------------------------------------------------------------
* This and the next function are called from the multiple worker threads. They
* may all enter the set the name section, but they will all set the same value,
* so even though there is a race, there is no race error. when work group 
* division is pushed down to the dsp, the race will go away.
*----------------------------------------------------------------------------*/
DSPDevicePtr  DSPKernel::device_entry_pt() 
{ 
    if (!p_device_entry_pt) 
    {
        size_t name_length;
        p_kernel->info(CL_KERNEL_FUNCTION_NAME, 0, 0, &name_length);

        void *name = malloc(name_length);
        p_kernel->info(CL_KERNEL_FUNCTION_NAME, name_length, name, 0);

        Program    *p     = (Program *)p_kernel->parent();
        DSPProgram *prog  = (DSPProgram *)(p->deviceDependentProgram(p_device));

        if (!prog->is_loaded()) ERROR();
        p_device_entry_pt = prog->query_symbol((char*)name);
        free (name);
    }
    return p_device_entry_pt; 
}

/******************************************************************************
* The data page pointer can frequently be 0, so we will initialize it to be 
* 0xffffffff as a start value instead of 0.
******************************************************************************/
DSPDevicePtr  DSPKernel::data_page_ptr()
{ 
    if (p_data_page_ptr == (DSPDevicePtr)0xffffffff) 
    {
        Program    *p     = (Program *)p_kernel->parent();
        DSPProgram *prog  = (DSPProgram *)(p->deviceDependentProgram(p_device));

        if (!prog->is_loaded()) ERROR();
        //p_data_page_ptr = prog->query_symbol("__TI_STATIC_BASE");
        p_data_page_ptr = prog->data_page_ptr();
    }
    return p_data_page_ptr; 
}

/******************************************************************************
* void DSPKernel::preAllocBuffers()
******************************************************************************/
cl_int DSPKernel::preAllocBuffers()
{
    for (unsigned int i=0; i < kernel()->numArgs(); ++i)
    {
        const Kernel::Arg &arg = kernel()->arg(i);

        if (arg.kind() == Kernel::Arg::Buffer &&
            arg.file() != Kernel::Arg::Local)
        {
            MemObject *buffer = *(MemObject **)arg.data();
            if (buffer && !buffer->allocate(device()))
                return CL_MEM_OBJECT_ALLOCATION_FAILURE;
        }
    }
    return CL_SUCCESS;
}


/******************************************************************************
* Try to find the size a work group needs to be executed the fastest on the DSP.
******************************************************************************/
size_t DSPKernel::guessWorkGroupSize(cl_uint num_dims, cl_uint dim,
                                     size_t global_work_size) const
{
    // ASW TODO - what the ????
    unsigned int dsps = p_device->numDSPs();

    /*-------------------------------------------------------------------------
    * Don't break in too small parts
    *------------------------------------------------------------------------*/
    if (k_exp(global_work_size, num_dims) > 64)
        return global_work_size;

    /*-------------------------------------------------------------------------
    * Find the divisor of global_work_size the closest to dsps but >= than it
    *------------------------------------------------------------------------*/
    unsigned int divisor = dsps <= 0 ? 1 : dsps;

    while (true)
    {
        if ((global_work_size % divisor) == 0)
            break;

        /*---------------------------------------------------------------------
        * Don't let the loop go up to global_work_size, the overhead would be
        * too huge
        *--------------------------------------------------------------------*/
        if (divisor > global_work_size || divisor > dsps * 32)
        {
            divisor = 1;  // Not parallel but has no CommandQueue overhead
            break;
        }

        divisor -= 1;
    }

    /*-------------------------------------------------------------------------
    * Return the size
    *------------------------------------------------------------------------*/
    return global_work_size / divisor;
}
 
/******************************************************************************
* localMemSize() 
******************************************************************************/
cl_ulong DSPKernel::localMemSize() const
{
    cl_ulong local_mem = 0;

    for (int i = 0;  i < kernel()->numArgs(); ++i)
    {
        const Kernel::Arg &arg = kernel()->arg(i);

        if (arg.kind() == Kernel::Arg::Buffer && 
            arg.file() == Kernel::Arg::Local)
              local_mem += arg.allocAtKernelRuntime();
    }

    return local_mem;
}

Kernel *         DSPKernel::kernel()   const { return p_kernel; }
DSPDevice *      DSPKernel::device()   const { return p_device; }

// From Wikipedia : http://www.wikipedia.org/wiki/Power_of_two#Algorithm_to_round_up_to_power_of_two
template <class T>
T next_power_of_two(T k) 
{
    if (k == 0) return 1;

    k--;
    for (int i=1; i<sizeof(T)*8; i<<=1)
            k = k | k >> i;
    return k+1;
}

size_t DSPKernel::typeOffset(size_t &offset, size_t type_len)
{
    size_t rs = offset;

    // Align offset to stype_len
    type_len = next_power_of_two(type_len);
    if (type_len > 8) type_len = 8; // The c66 has no alignment need > 8 bytes

    size_t mask = ~(type_len - 1);

    while (rs & mask != rs)
        rs++;

    // Where to try to place the next value
    offset = rs + type_len;

    return rs;
}

static int kernelID = 0;

/*=============================================================================
* DSPKernelEvent
*============================================================================*/
DSPKernelEvent::DSPKernelEvent(DSPDevice *device, KernelEvent *event)
: p_device(device), p_event(event), p_kernel((DSPKernel*)event->deviceKernel()),
  p_kernel_id(kernelID++), p_debug_kernel(false), p_num_arg_words(0),
  p_WG_alloca_start(0)
{ 
    char *dbg = getenv("TI_OCL_DEBUG_KERNEL");
    if (dbg) p_debug_kernel = true;

    callArgs(MAX_ARG_BUF_SIZE);
}

DSPKernelEvent::~DSPKernelEvent() { }

#define READ_ONLY_BUFFER(buffer)  (buffer->flags() & CL_MEM_READ_ONLY)
#define WRITE_ONLY_BUFFER(buffer) (buffer->flags() & CL_MEM_WRITE_ONLY)

#define SETARG(val) if (arg_words < args_in_mem_size) args_in_mem[arg_words++] = val; \
                    else std::cerr << "To many argument bytes are needed" << std::endl

#define SETMOREARG(sz, pval) do \
    { \
        more_arg_offset = ROUNDUP(more_arg_offset, sz); \
        if (ROUNDUP(more_arg_offset + sz, 8) > sizeof(p_msg.u.k.flush.buffers))\
            std::cerr << "Too many arguments, does not fit" << std::endl; \
        memcpy(more_args_in_mem+more_arg_offset, pval, sz); \
        more_arg_offset += sz; \
    } while(0)

//#define SETMOREARG(sz,psrc)

/******************************************************************************
* DSPKernelEvent::callArgs
******************************************************************************/
void DSPKernelEvent::callArgs(unsigned args_in_mem_size)
{
    int        arg_words = 0;
    unsigned  *args_in_mem = (unsigned*)p_msg.u.k.kernel.argBuf;
    char      *more_args_in_mem = (char *)p_msg.u.k.flush.buffers;
    int        more_arg_offset  = 4;
    bool       is_more_arg = false;

    /*-------------------------------------------------------------------------
    * Write Arguments
    *------------------------------------------------------------------------*/
    for (int i = 0;  i < p_kernel->kernel()->numArgs(); ++i)
    {
        is_more_arg = (i >= 10);

        const Kernel::Arg & arg  = p_kernel->kernel()->arg(i);
        size_t              size = arg.valueSize() * arg.vecDim();

        if (size == 0) ERR("Kernel Argument has size == 0");
        if (size != 1 && size != 2 && size != 4 && size != 8)
            ERR("Invalid Kernel Argument size");

        /*---------------------------------------------------------------------
        * We may have to perform some changes in the values (buffers, etc)
        *--------------------------------------------------------------------*/
        switch (arg.kind())
        {
            case Kernel::Arg::Buffer:
            {
                MemObject    *buffer = 0;
                DSPDevicePtr buf_ptr = 0;
                if (arg.data()) buffer = *(MemObject **)arg.data();
                if (!is_more_arg) SETARG(sizeof(DSPVirtPtr));

                DSPVirtPtr *buf_dspvirtptr = (!is_more_arg) ?
                                             (&args_in_mem[arg_words]) :
                   (DSPVirtPtr *)(more_args_in_mem+ROUNDUP(more_arg_offset,4));

                /*-------------------------------------------------------------
                * Alloc a buffer and pass it to the kernel
                *------------------------------------------------------------*/
                if (arg.file() == Kernel::Arg::Local)
                {
                    uint32_t     lbufsz = arg.allocAtKernelRuntime();
                    p_local_bufs.push_back(LocalPair(buf_dspvirtptr, lbufsz));

                    /*-----------------------------------------------------
                    * Since the only reader and writer of local memory (L2)
                    * will be the core itself, I do not believe we need 
                    * to flush local buffers for correctness. 
                    *----------------------------------------------------*/
                    //p_flush_bufs->push_back(DSPMemRange(lbuf, lbufsz));
                }
                else if (buffer != NULL)
                {
                    /*---------------------------------------------------------
                    * Get the DSP buffer, allocate it and get its pointer
                    *--------------------------------------------------------*/
                    if (buffer->flags() & CL_MEM_USE_HOST_PTR)
                    {
                        p_hostptr_tmpbufs.push_back(
                           HostptrPair(buffer, DSPPtrPair(0, buf_dspvirtptr)));
                    }
                    else
                    {
                        DSPBuffer *dspbuf = (DSPBuffer *)buffer->deviceBuffer(p_device);
                        buffer->allocate(p_device);
                        DSPDevicePtr64 addr64 = dspbuf->data();
                        if (addr64 < 0xFFFFFFFF)
                            buf_ptr = addr64;
                        else
                            p_64bit_bufs.push_back(DSPMemRange(DSPPtrPair(
                                     addr64, buf_dspvirtptr), buffer->size()));
      
                        if (! WRITE_ONLY_BUFFER(buffer))
                            p_flush_bufs.push_back(DSPMemRange(DSPPtrPair(
                                     addr64, buf_dspvirtptr), buffer->size()));
                    }
                }

                /*---------------------------------------------------------
                * Use 0 for local buffer address here, it will be overwritten
                * with allocated local buffer address at kernel dispatch time.
                * Same for allocating temporary buffer for use_host_ptr.
                *--------------------------------------------------------*/
                if (!is_more_arg) SETARG(buf_ptr);
                else { SETMOREARG(4, &buf_ptr); }

                break;
            }

            case Kernel::Arg::Image2D:
            case Kernel::Arg::Image3D: ERR("Images not yet supported"); break;

            /*-----------------------------------------------------------------
            * Non-Buffers 
            *----------------------------------------------------------------*/
            default:
                if (!is_more_arg)
                {
                    SETARG((size < 4 ? 4 : size));
                    // Cast to (int) to avoid a codegen bug
                    // ZEXT will happen in LLVM and ICODE, so don't worry
                    if (size == 1) SETARG(((int) *((signed char*)arg.data())));
                    else if (size == 2)  SETARG(((int) *((short*)arg.data())));
                    else                 SETARG(*((unsigned*)    arg.data()));
                    if (size == 8) { SETARG(*(((unsigned*)arg.data()) + 1)); }
                }
                else { SETMOREARG(size, arg.data()); }
                break;
        }
    }
    SETARG(0);  // 0 terminator for args area

    p_num_arg_words = arg_words;
    p_msg.u.k.flush.sizeMoreArgs = (more_arg_offset > 4) ?
                                   ROUNDUP(more_arg_offset, 8) : 0;
}

/******************************************************************************
* debug_pause
******************************************************************************/
static void debug_pause(uint32_t entry, uint32_t dsp_id, 
                        const char* outfile, char *name)
{
    printf("[OCL] Launching kernel %s on DSP %d\n", name, dsp_id);
    printf("[OCL] Connect debugger and set breakpoint at 0x%08x\n", entry);
    printf("[OCL] Load symbols from file %s\n", outfile);
    printf("[OCL] Press any key, then enter to continue\n");
    do { char t; std::cin >> t; } while(0);
}



/******************************************************************************
* bool DSPKernelEvent::run()
******************************************************************************/
cl_int DSPKernelEvent::run(Event::Type evtype)
{
    Program    *p    = (Program *)p_kernel->kernel()->parent();
    DSPProgram *prog = (DSPProgram *)(p->deviceDependentProgram(p_device));

    // TODO perhaps ensure that prog is loaded.

    int dim  = p_event->work_dim();

    /*-------------------------------------------------------------------------
    * Create a message for the DSP
    *------------------------------------------------------------------------*/
    Msg_t &msg = p_msg;
    kernel_config_t *cfg  = &msg.u.k.kernel.config;

    if (evtype == Event::TaskKernel)
    {
        msg.command    = TASK;
        cfg->Kernel_id = p_kernel_id;

        CommandQueue *q = (CommandQueue *) p_event->parent();
        cl_command_queue_properties q_prop = 0;
        q->info(CL_QUEUE_PROPERTIES, sizeof(q_prop), &q_prop, NULL);
        cfg->global_sz_0 = (q_prop & CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE) ?
                           OUT_OF_ORDER_TASK_SIZE : IN_ORDER_TASK_SIZE;
        cfg->local_sz_0  = 1;
        cfg->local_sz_1  = 1;
        cfg->local_sz_2  = 1;
    }
    else 
    {
        msg.command = NDRKERNEL;

        cfg->num_dims         = dim;
        cfg->global_sz_0      = p_event->global_work_size(0);
        cfg->global_sz_1      = dim > 1 ? p_event->global_work_size(1) : 1;
        cfg->global_sz_2      = dim > 2 ? p_event->global_work_size(2) : 1;
        cfg->local_sz_0       = p_event->local_work_size(0);
        cfg->local_sz_1       = dim > 1 ? p_event->local_work_size(1) : 1;
        cfg->local_sz_2       = dim > 2 ? p_event->local_work_size(2) : 1;
        cfg->global_off_0     = p_event->global_work_offset(0);
        cfg->global_off_1     = p_event->global_work_offset(1);
        cfg->global_off_2     = p_event->global_work_offset(2);
        cfg->WG_gid_start_0   = 0;
        cfg->WG_gid_start_1   = 0;
        cfg->WG_gid_start_2   = 0;
        cfg->Kernel_id        = p_kernel_id;
        cfg->WG_id            = 0;
        cfg->stats            = 0;
    }

    msg.u.k.kernel.entry_point   = (unsigned)p_kernel->device_entry_pt();
    msg.u.k.kernel.data_page_ptr = (unsigned)p_kernel->data_page_ptr();

    /*-------------------------------------------------------------------------
    * Allocating local buffer in L2 per kernel run instance
    *------------------------------------------------------------------------*/
    uint32_t total_sz, block_sz;
    DSPDevicePtr local_scratch = p_device->get_local_scratch(total_sz, block_sz);
    for (size_t i = 0; i < p_local_bufs.size(); ++i)
    {
        DSPVirtPtr *p_arg_word     = p_local_bufs[i].first; 
        unsigned    local_buf_size = p_local_bufs[i].second;

        uint32_t rounded_sz = ROUNDUP(local_buf_size, block_sz);
        if (rounded_sz > total_sz)
        {
            QERR("Total local buffer size exceeds available local size",
                 CL_MEM_OBJECT_ALLOCATION_FAILURE);
        }
        *p_arg_word = local_scratch;
        local_scratch += rounded_sz;
        total_sz      -= rounded_sz;
    }

    /*-------------------------------------------------------------------------
    * Allocating temporary space in global memory for kernel alloca'ed data
    *------------------------------------------------------------------------*/
#define NUM_CORES_PER_CHIP	8
    cfg->WG_alloca_size = p_kernel->kernel()->get_wi_alloca_size() * 
                          cfg->local_sz_0 * cfg->local_sz_1 * cfg->local_sz_2;
    if (cfg->WG_alloca_size > 0)
    {
        cfg->WG_alloca_size += 4096;   // 4K bytes padding between WGs' allocas
        uint32_t chip_alloca_size = cfg->WG_alloca_size * NUM_CORES_PER_CHIP;
        p_WG_alloca_start = p_device->malloc_global( // malloc abort if fail
                                                      chip_alloca_size, true);
        if (!p_WG_alloca_start)
        {
            QERR("Alloca size exceeds available global memory",
                 CL_OUT_OF_RESOURCES);
        }

        if (p_WG_alloca_start < 0xFFFFFFFF)
            cfg->WG_alloca_start = (DSPVirtPtr) p_WG_alloca_start;
        else
            p_64bit_bufs.push_back(DSPMemRange(DSPPtrPair(
                 p_WG_alloca_start, &cfg->WG_alloca_start), chip_alloca_size));
    }

    /*-------------------------------------------------------------------------
    * Allocating temporary global buffer for use_host_ptr
    *------------------------------------------------------------------------*/
    for (int i = 0; i < p_hostptr_tmpbufs.size(); ++i)
    {
        MemObject      *buffer       =  p_hostptr_tmpbufs[i].first;
        DSPDevicePtr64 *p_addr64     = &p_hostptr_tmpbufs[i].second.first;
        DSPVirtPtr     *p_arg_word   =  p_hostptr_tmpbufs[i].second.second;
        
        *p_addr64 = p_device->malloc_global(buffer->size(), false);

        if (!p_addr64)
        {
            QERR("Temporary memory for CL_MEM_USE_HOST_PTR buffer exceeds available global memory",
                 CL_MEM_OBJECT_ALLOCATION_FAILURE);
        }

        if (*p_addr64 < 0xFFFFFFFF)
            *p_arg_word = *p_addr64;
        else
            p_64bit_bufs.push_back(DSPMemRange(DSPPtrPair(
                                      *p_addr64, p_arg_word), buffer->size()));

        if (! WRITE_ONLY_BUFFER(buffer))
        {
            void *mapped_tmpbuf = Driver::instance()->map(*p_addr64,
                                                        buffer->size(), false);
            memcpy(mapped_tmpbuf, buffer->host_ptr(), buffer->size());
            p_flush_bufs.push_back(DSPMemRange(DSPPtrPair(
                                      *p_addr64, p_arg_word), buffer->size()));
            Driver::instance()->unmap(mapped_tmpbuf, *p_addr64,
                                      buffer->size(), true);
        }
    }

    /*-------------------------------------------------------------------------
    * Compute MPAX mappings from DSPDevicePtr64 to DSPVirtPtr in p_64bit_bufs
    *------------------------------------------------------------------------*/
    msg.u.k.flush.num_mpaxs = 0;
    uint32_t num_64bit_bufs = p_64bit_bufs.size();
    if (num_64bit_bufs > 0)
    {
        uint64_t *phys_addrs = new uint64_t[num_64bit_bufs];
        uint32_t *lengths    = new uint32_t[num_64bit_bufs];
        uint32_t *prots      = new uint32_t[num_64bit_bufs];
        uint32_t *virt_addrs = new uint32_t[num_64bit_bufs];
        for (int i = 0; i < p_64bit_bufs.size(); ++i)
        {
            phys_addrs[i] = p_64bit_bufs[i].first.first;
            lengths[i]    = p_64bit_bufs[i].second;
            prots[i]      = 0;  // don't care yet
        }

        keystone_mmap_resources_t mpax_res;
        memcpy(&mpax_res, p_device->get_mpax_default_res(),
               sizeof(keystone_mmap_resources_t));
        if (keystone_mmap_resource_alloc(num_64bit_bufs, phys_addrs, lengths, 
		  prots, virt_addrs, &mpax_res) != KEYSTONE_MMAP_RESOURCE_NOERR)
        {
            QERR("MPAX allocation failed!",
                 CL_OUT_OF_RESOURCES);
        }
    
        // set the MPAX settings in the message
        uint32_t mpax_used = 0;
        for (; mpax_res.mapping[mpax_used].segsize_power2 > 0; mpax_used += 1)
        {
            msg.u.k.flush.mpax_settings[2*mpax_used  ] = (uint32_t) 
                 (mpax_res.mapping[mpax_used].raddr >> 12); // e.g. 0x822004
            msg.u.k.flush.mpax_settings[2*mpax_used+1] =     // e.g. 0xC000000D
                  mpax_res.mapping[mpax_used].baddr
               | (mpax_res.mapping[mpax_used].segsize_power2-1);
        }
        msg.u.k.flush.num_mpaxs = mpax_used;
    
        // set the virtual address in arguments
        for (int i = 0; i < p_64bit_bufs.size(); ++i)
        {
            *(p_64bit_bufs[i].first.second) = virt_addrs[i];
            if (p_debug_kernel)
               printf("Virtual = 0x%x, physical = 0x%llx\n",
                      virt_addrs[i], p_64bit_bufs[i].first.first);
        }
        delete [] phys_addrs;
        delete [] lengths;
        delete [] prots;
        delete [] virt_addrs;
    }

    /*-------------------------------------------------------------------------
    * Helpful information for debugging a kernel
    *------------------------------------------------------------------------*/
    if (p_debug_kernel)
    {
        for (int i = 0; i < msg.u.k.flush.num_mpaxs; i++)
            printf("mpax %d: l=0x%x, h=0x%x\n", i,
                   msg.u.k.flush.mpax_settings[2*i],
                   msg.u.k.flush.mpax_settings[2*i+1]); 

        uint32_t *args = msg.u.k.kernel.argBuf;
        int arg_num = 1;
        // TODO: print more args properly
        for (int i=0; i < p_num_arg_words; i++)
        {
            if (args[i] == 4)
            {
                i++;
                printf("[OCL] Kernel argument %d = 0x%08x\n", arg_num, args[i]);
            }
            else if (args[i] == 8)
            {
                printf("[OCL] Kernel argument %d = 0x%08x 0x%08x\n", 
                        arg_num, args[i+1], args[i+2]);
                i+=2;
            }
            arg_num++;
        }
    }

    /*-------------------------------------------------------------------------
    * Make sure we do not overflow the number of commands a mailbox can handle
    *------------------------------------------------------------------------*/
    if (p_flush_bufs.size() > MAX_KERNEL_ARGUMENTS) 
    {
        QERR("To many buffers to flush", CL_OUT_OF_RESOURCES);
    }

    /*-------------------------------------------------------------------------
    * Populate Flush commands for any buffers that are read by the DSP
    *------------------------------------------------------------------------*/
    msg.u.k.flush.numBuffers = p_flush_bufs.size();

#if 0  // YUAN: flush buffers used for more arguments (for now)
    for (int i=0; i < p_flush_bufs.size(); ++i)
    {
        msg.u.k.flush.buffers[2*i]   = p_flush_bufs[i].first; 
        msg.u.k.flush.buffers[2*i+1] = p_flush_bufs[i].second; 
    }
#endif

    /*-------------------------------------------------------------------------
    * Feedback to user for debug
    *------------------------------------------------------------------------*/
    if (p_debug_kernel)
    {
        size_t name_length;
        p_kernel->kernel()->info(CL_KERNEL_FUNCTION_NAME, 0, 0, &name_length);
        char *name = (char*)malloc(name_length);
        if (!name) return CL_OUT_OF_HOST_MEMORY;
        p_kernel->kernel()->info(CL_KERNEL_FUNCTION_NAME, name_length, name, 0);

        debug_pause(p_kernel->device_entry_pt(), p_device->dspID(), 
                    prog->outfile_name(), name);
        free (name);
    }

    /*-------------------------------------------------------------------------
    * Dispatch the commands through the mailbox
    *------------------------------------------------------------------------*/
    p_device->mail_to(msg);

    /*-------------------------------------------------------------------------
    * Do not wait for completion
    *------------------------------------------------------------------------*/
    return CL_SUCCESS;
}

/******************************************************************************
* free_tmp_bufs allocated for kernel allocas, and for use_host_ptr
******************************************************************************/
void DSPKernelEvent::free_tmp_bufs()
{
    if (p_WG_alloca_start > 0)
        p_device->free_global(p_WG_alloca_start);

    for (int i = 0; i < p_hostptr_tmpbufs.size(); ++i)
    {
        MemObject *buffer     = p_hostptr_tmpbufs[i].first; 
        DSPDevicePtr64 addr64 = p_hostptr_tmpbufs[i].second.first; 

        if (! READ_ONLY_BUFFER(buffer))
        {
            void *mapped_tmpbuf = Driver::instance()->map(addr64,
                                                        buffer->size(), true);
            memcpy(buffer->host_ptr(), mapped_tmpbuf, buffer->size());
            Driver::instance()->unmap(mapped_tmpbuf, addr64,
                                      buffer->size(), false);
        }
        p_device->free_global(addr64);
    }

}

