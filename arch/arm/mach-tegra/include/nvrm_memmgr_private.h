/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the NVIDIA Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifndef INCLUDED_NVDDK_MEMMGR_PRIVATE_H
#define INCLUDED_NVDDK_MEMMGR_PRIVATE_H

#include "nvrm_heap.h"

#ifdef __cplusplus
extern "C"
{
#endif  /* __cplusplus */

#define NVRM_HMEM_CHECK_MAGIC       NV_DEBUG

#define NV_RM_HMEM_IS_ALLOCATED(hMem) \
        (((hMem)->PhysicalAddress != NV_RM_INVALID_PHYS_ADDRESS) || \
         ((hMem)->VirtualAddress  != NULL) || \
         ((hMem)->hPageHandle     != NULL) )

typedef struct NvRmMemRec 
{
    void                *VirtualAddress;
    NvRmDeviceHandle    hRmDevice;
    NvOsPageAllocHandle hPageHandle;
    NvRmPhysAddr        PhysicalAddress;
    NvU32               size;
    NvU32               alignment;

    /* Used for GART heap to keep track of the number of GART pages
     * in use by this handle.
     */
    NvU32               Pages; 

    NvS32               refcount;
    NvS32               pin_count;

    NvOsMemAttribute    coherency;
    NvRmHeap            heap;

    NvBool              mapped;
    NvU8                priority; 

#if NVRM_HMEM_CHECK_MAGIC
    NvU32               magic;  // set to NVRM_MEM_MAGIC if valid
#endif
} NvRmMem;

#ifdef __cplusplus
}
#endif  /* __cplusplus */


#endif




