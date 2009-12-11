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

#include "nvcommon.h"
#include "nvos.h"
#include "nvrm_memmgr.h"
#include "nvrm_heap.h"
#include "nvrm_heap_simple.h"
#include "nvrm_hardware_access.h"

static NvRmPrivHeap   gs_CarveoutHeap;
static NvRmPhysAddr   gs_CarveoutBaseAddr;
static void          *gs_CarveoutVaddr;
static NvBool         gs_CarveoutGloballyMapped;

static NvRmHeapSimple gs_CarveoutAllocator;


NvError NvRmPrivHeapCarveoutAlloc(
    NvU32 size,
    NvU32 align,
    NvRmPhysAddr *PAddr)
{
    return NvRmPrivHeapSimpleAlloc(&gs_CarveoutAllocator, size, align, PAddr);
}

NvError NvRmPrivHeapCarveoutPreAlloc(NvRmPhysAddr Address, NvU32 Length)
{
    return NvRmPrivHeapSimplePreAlloc(&gs_CarveoutAllocator, Address, Length);
}

void NvRmPrivHeapCarveoutFree(NvRmPhysAddr addr)
{
    NvRmPrivHeapSimpleFree(&gs_CarveoutAllocator, addr);
}

NvS32 NvRmPrivHeapCarveoutMemoryUsed(void)
{
    return NvRmPrivHeapSimpleMemoryUsed(&gs_CarveoutAllocator);
}

NvS32 NvRmPrivHeapCarveoutLargestFreeBlock(void)
{
    return NvRmPrivHeapSimpleLargestFreeBlock(&gs_CarveoutAllocator);
}

NvS32 NvRmPrivHeapCarveoutTotalSize(void)
{
    return gs_CarveoutHeap.length;
}


void *NvRmPrivHeapCarveoutMemMap(
    NvRmPhysAddr base,
    NvU32 length,
    NvOsMemAttribute attribute)
{
    NvU32  StartOffset = base - gs_CarveoutBaseAddr;
    NvU32  EndOffset   = StartOffset + length - 1;

    if (!gs_CarveoutVaddr)
        return NULL;

    NV_ASSERT(length != 0);

    // sanity checking
    if (StartOffset < gs_CarveoutHeap.length &&
        EndOffset   < gs_CarveoutHeap.length)
    {
        NvUPtr uptr = (NvUPtr)gs_CarveoutVaddr;
        return (void *)(uptr + StartOffset);
    }

    NV_ASSERT(!"Attempt to map something that is not part of the carveout");
    return NULL;
}


NvRmPrivHeap *NvRmPrivHeapCarveoutInit(NvU32 length, NvRmPhysAddr base)
{
    NvError err;
    NvBool  bGloballyMapped = NV_FALSE;
    void    *vAddr = NULL;

#if !(NVOS_IS_LINUX && !NVCPU_IS_X86)
    /* try to map the memory, if we can't map it then bail out */
    err = NvRmPhysicalMemMap(base, length, 
              NVOS_MEM_READ_WRITE | NVOS_MEM_GLOBAL_ADDR, 
              NvOsMemAttribute_Uncached, &vAddr);

    if (err == NvSuccess)
    {
        bGloballyMapped = NV_TRUE;
    }
    else
    {
        // try again to map carveout, but with global flag gone
        err = NvRmPhysicalMemMap(base, length, NVOS_MEM_READ_WRITE, 
                  NvOsMemAttribute_Uncached, &vAddr);

        if (err != NvSuccess)
            return NULL;
    }
#endif

    err = NvRmPrivHeapSimple_HeapAlloc(base, length, &gs_CarveoutAllocator);
    
    if (err != NvSuccess)
    {
        if (vAddr)
            NvRmPhysicalMemUnmap(vAddr, length);
        return NULL;
    }

    gs_CarveoutHeap.heap            = NvRmHeap_ExternalCarveOut;
    gs_CarveoutHeap.length          = length;
    gs_CarveoutHeap.PhysicalAddress = base;
    gs_CarveoutBaseAddr             = base;
    gs_CarveoutVaddr                = vAddr;
    gs_CarveoutGloballyMapped       = bGloballyMapped;

    return &gs_CarveoutHeap;
}


void NvRmPrivHeapCarveoutDeinit(void)
{
    // deinit the carveout allocator
    if (gs_CarveoutVaddr)
    {
        NvRmPhysicalMemUnmap(gs_CarveoutVaddr, gs_CarveoutHeap.length);
        gs_CarveoutVaddr = NULL;
    }

    NvRmPrivHeapSimple_HeapFree(&gs_CarveoutAllocator);
    NvOsMemset(&gs_CarveoutHeap, 0, sizeof(gs_CarveoutHeap));
    NvOsMemset(&gs_CarveoutAllocator, 0, sizeof(gs_CarveoutAllocator));
}


void NvRmPrivHeapCarveoutGetInfo(
    NvU32 *CarveoutPhysBase, 
    void  **pCarveout,
    NvU32 *CarveoutSize)
{
    if (gs_CarveoutGloballyMapped)
    {
        *CarveoutPhysBase = gs_CarveoutHeap.PhysicalAddress;
        *pCarveout = gs_CarveoutVaddr;
        *CarveoutSize = gs_CarveoutHeap.length;
    }
    else
    {
        *CarveoutPhysBase = 0;
        *pCarveout        = NULL;
        *CarveoutSize     = 0;
    }
}

