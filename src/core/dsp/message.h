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
#ifndef __MESSAGE_H_
#define __MESSAGE_H_

#include <stdint.h>

typedef enum { READY, EXIT, TASK, NDRKERNEL, WORKGROUP, CACHEINV, FREQUENCY, SUCCESS, ERROR, PRINT } command_codes;

#define MAX_KERNEL_ARGUMENTS 10
#define MAX_ARG_BUF_SIZE     (MAX_KERNEL_ARGUMENTS*3)+1
#define MAX_FLUSH_BUF_SIZE   (MAX_KERNEL_ARGUMENTS*2)

#define MAX_XMCSES_MPAXS	7
#define FIRST_FREE_XMC_MPAX	3  // XMC MPAXs available: 3 - F
#define FIRST_FREE_SES_MPAX	1  // SES MPAXs available: 1 - 7

/******************************************************************************
* Need to ensure that the alignments and therefore the offsets of all fields 
* are consistent between the host and the device.
******************************************************************************/
typedef struct
{
    uint32_t num_dims;

    uint32_t global_sz_0;
    uint32_t global_sz_1;
    uint32_t global_sz_2;
    uint32_t local_sz_0;
    uint32_t local_sz_1;
    uint32_t local_sz_2;
    uint32_t global_off_0;
    uint32_t global_off_1;
    uint32_t global_off_2;
    uint32_t WG_gid_start_0;
    uint32_t WG_gid_start_1;
    uint32_t WG_gid_start_2;
    uint32_t Kernel_id;
    uint32_t WG_id;
    uint32_t stats;
    uint32_t WG_alloca_start;
    uint32_t WG_alloca_size;
} kernel_config_t;

typedef struct 
{
    uint8_t  numBuffers;
    uint8_t  num_mpaxs;  // TODO: XMC only mpax for kernel alloca memory
    uint16_t sizeMoreArgs;
    uint32_t buffers[MAX_FLUSH_BUF_SIZE];
    uint32_t mpax_settings[2*MAX_XMCSES_MPAXS];  // (MPAXL, MPAXH) pair
} flush_msg_t;

typedef struct
{
    kernel_config_t config;
    uint32_t        entry_point;
    uint32_t        data_page_ptr;
    uint32_t        argBuf[MAX_ARG_BUF_SIZE];  // NULL size terminated
} kernel_msg_t;

typedef struct 
{
    command_codes command;
    union
    {
        struct
        {
            kernel_msg_t  kernel;
            flush_msg_t   flush;
        } k;
        char message[sizeof(kernel_msg_t) + sizeof(flush_msg_t)];
    } u;
} Msg_t;

static Msg_t exitMsg      = {EXIT};
static Msg_t successMsg   = {SUCCESS};
static Msg_t readyMsg     = {READY};
static Msg_t errorMsg     = {ERROR};
static Msg_t frequencyMsg = {FREQUENCY};
// static far Msg_t printMsg  = {PRINT}; // moved to L2 in monitor

static const uint32_t mbox_payload         = sizeof(Msg_t);

#define MBOX_SIZE 0x2000

#define IN_ORDER_TASK_SIZE	1
#define OUT_OF_ORDER_TASK_SIZE	(IN_ORDER_TASK_SIZE+1)

#endif
