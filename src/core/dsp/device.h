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
#ifndef __DSP_DEVICE_H__
#define __DSP_DEVICE_H__

extern "C" {
#include "dload_api.h"
}

#include "../deviceinterface.h"
#include "dspheap.h"
#include "message.h"
#include "u_concurrent_map.h"
#include "kernel.h"
#include <pthread.h>
#include <string>
#include <list>

namespace Coal
{

class MemObject;
class Event;
class Program;
class Kernel;

class DSPDevice : public DeviceInterface
{
    public:
        DSPDevice(unsigned char dsp_id);
        ~DSPDevice();

        void init();

        cl_int info(cl_device_info param_name,
                    size_t param_value_size,
                    void *param_value,
                    size_t *param_value_size_ret) const;

        DeviceBuffer *createDeviceBuffer(MemObject *buffer, cl_int *rs);
        DeviceProgram *createDeviceProgram(Program *program);
        DeviceKernel *createDeviceKernel(Kernel *kernel,
                                         llvm::Function *function);

        cl_int initEventDeviceData(Event *event);
        void freeEventDeviceData(Event *event);

        void   pushEvent(Event *event);
        bool   stop();
        bool   availableEvent();
        Event *getEvent(bool &stop);

        unsigned int numDSPs() const;
        float dspMhz() const;
        unsigned char dspID() const;
        DLOAD_HANDLE dload_handle() const;

        int    load(const char *filename);
        bool unload(int file_handle);

        /*---------------------------------------------------------------------
        * These malloc routines return a uint32_t instead of a pointer
        * Because the target memory space is 32 bit and is independent of the 
        * size of a host pointer (ie. 32bit vs 64 bit)
        * Device/Target global memory could be 36-bit.
        * get_local_scratch returns max local free block for per kernel use.
        *--------------------------------------------------------------------*/
        DSPDevicePtr   get_local_scratch(uint32_t &size, uint32_t &block_size);
        DSPDevicePtr   malloc_local (size_t   size);
        void           free_local   (DSPDevicePtr add);
        DSPDevicePtr   malloc_msmc  (size_t   size);
        void           free_msmc    (DSPDevicePtr add);
        DSPDevicePtr64 malloc_global(size_t   size, bool prefer_32bit=true);
        void           free_global  (DSPDevicePtr64 add);

        void mail_to   (Msg_t& msg);
        bool mail_query();
        int  mail_from ();

        void push_complete_pending(uint32_t idx, class Event* const data);
        bool get_complete_pending(uint32_t idx, class Event* &data);
        void dump_complete_pending();
        bool any_complete_pending();
        bool gotEnoughToWorkOn();

        std::string builtinsHeader(void) const { return "dsp.h"; }

        DSPDevicePtr get_addr_kernel_config() { return p_addr_kernel_config; }
        void*        get_mpax_default_res();

    private:
        unsigned int       p_cores;
        unsigned int       p_num_events;
        float              p_dsp_mhz;
        pthread_t          p_worker;
        void*              p_rx_mbox; // int 
        void*              p_tx_mbox;
        std::list<Event *> p_events;
        pthread_cond_t     p_events_cond;
        pthread_mutex_t    p_events_mutex;
        bool               p_stop; 
        bool               p_initialized;
        unsigned char      p_dsp_id;
        dspheap            p_device_ddr_heap1;  // persistently mapped memory
        dspheap            p_device_ddr_heap2;  // ondemand mapped memory
        dspheap            p_device_ddr_heap3;  // addl ondemand mapped memory
        dspheap            p_device_l2_heap;
        dspheap            p_device_msmc_heap;
        DLOAD_HANDLE       p_dload_handle;
        concurrent_map<uint32_t, class Event*> p_complete_pending;

        DSPDevicePtr       p_addr_kernel_config;
        DSPDevicePtr64     p_addr64_global_mem;
        DSPDevicePtr       p_addr_local_mem;
        DSPDevicePtr       p_addr_msmc_mem;
        DSPDevicePtr       p_addr_mbox_d2h_phys;
        DSPDevicePtr       p_addr_mbox_h2d_phys;
        uint64_t           p_size64_global_mem;
        uint32_t           p_size_local_mem;
        uint32_t           p_size_msmc_mem;
        uint32_t           p_size_mbox_d2h;
        uint32_t           p_size_mbox_h2d;
        void*              p_mpax_default_res;
};
}
#endif
