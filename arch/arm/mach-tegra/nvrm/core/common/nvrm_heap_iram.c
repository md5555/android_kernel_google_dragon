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

#include "nvrm_heap.h"
#include "nvrm_heap_simple.h"
#include "nvrm_hardware_access.h"


static NvRmPrivHeap   gs_IramHeap;
static NvUPtr         gs_IramBaseAddr;
static void          *gs_IramVaddr;
static NvRmHeapSimple gs_IramAllocator;

NvError NvRmPrivHeapIramAlloc(NvU32 size, NvU32 align, NvRmPhysAddr *PAddr)
{
    NvError err;
    err = NvRmPrivHeapSimpleAlloc(&gs_IramAllocator, size, align, PAddr);
    return err;
}

NvError NvRmPrivHeapIramPreAlloc(NvRmPhysAddr Address, NvU32 Length)
{
    return NvRmPrivHeapSimplePreAlloc(&gs_IramAllocator, Address, Length);
}

void NvRmPrivHeapIramFree(NvRmPhysAddr addr)
{
    NvRmPrivHeapSimpleFree(&gs_IramAllocator, addr);
}

NvRmPrivHeap *NvRmPrivHeapIramInit(NvU32 length, NvRmPhysAddr base)
{
    void   *vAddr = NULL;
    NvError err;

#if !(NVOS_IS_LINUX && !NVCPU_IS_X86)
    /* try to map the memory, if we can't map it then bail out */
    err = NvRmPhysicalMemMap(base, length, NVOS_MEM_READ_WRITE, 
              NvOsMemAttribute_Uncached, &vAddr);
    if (err != NvSuccess)
        return NULL;
#endif

    err = NvRmPrivHeapSimple_HeapAlloc(base, length, &gs_IramAllocator);
    
    if (err != NvSuccess)
    {
        if (vAddr)
            NvRmPhysicalMemUnmap(vAddr, length);
        return NULL;
    }

    gs_IramHeap.heap            = NvRmHeap_IRam;
    gs_IramHeap.length          = length;
    gs_IramHeap.PhysicalAddress = base;
    gs_IramBaseAddr             = (NvUPtr)base;
    gs_IramVaddr                = vAddr;

    return &gs_IramHeap;
}

void NvRmPrivHeapIramDeinit(void)
{
    // deinit the carveout allocator
    if (gs_IramVaddr)
    {
        NvRmPhysicalMemUnmap(gs_IramVaddr, gs_IramHeap.length);
        gs_IramVaddr = NULL;
    }

    NvRmPrivHeapSimple_HeapFree(&gs_IramAllocator);
    NvOsMemset(&gs_IramHeap, 0, sizeof(gs_IramHeap));
    NvOsMemset(&gs_IramAllocator, 0, sizeof(gs_IramAllocator));
}

void *NvRmPrivHeapIramMemMap(
    NvRmPhysAddr base,
    NvU32 length,
    NvOsMemAttribute attribute)
{
    NvU32  StartOffset = base - gs_IramBaseAddr;
    NvU32  EndOffset   = StartOffset + length - 1;

    NV_ASSERT(length != 0);

    if (!gs_IramVaddr)
        return NULL;

    // sanity checking
    if (StartOffset < gs_IramHeap.length &&
        EndOffset   < gs_IramHeap.length)
    {
        NvUPtr uptr = (NvUPtr)gs_IramVaddr;
        return (void *)(uptr + StartOffset);
    }

    NV_ASSERT(!"Attempt to map something that is not part of the iram");
    return NULL;
}
