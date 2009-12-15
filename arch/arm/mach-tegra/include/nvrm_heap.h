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

#ifndef NVRM_HEAP_H
#define NVRM_HEAP_H

#include "nvrm_memmgr.h"
#include "nvassert.h"
#include "nvos.h"


#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */


typedef NvRmPhysAddr (*NvRmHeapAlloc)(NvU32 size);
typedef void (*NvRmHeapFree)(NvRmPhysAddr);

typedef struct NvRmPrivHeapRec
{
    NvRmHeap   heap;
    NvRmPhysAddr PhysicalAddress;
    NvU32      length;

} NvRmPrivHeap;


void
NvRmPrivPreservedMemHandleInit(NvRmDeviceHandle hRm);

NvRmPrivHeap *
NvRmPrivHeapCarveoutInit(NvU32      length, 
                         NvRmPhysAddr base);

void
NvRmPrivHeapCarveoutDeinit(void);

NvError
NvRmPrivHeapCarveoutPreAlloc(NvRmPhysAddr Address, NvU32 Length);

NvError
NvRmPrivHeapCarveoutAlloc(NvU32 size, NvU32 align, NvRmPhysAddr *PAddr);

void 
NvRmPrivHeapCarveoutFree(NvRmPhysAddr addr);

void *
NvRmPrivHeapCarveoutMemMap(NvRmPhysAddr base, NvU32 length, NvOsMemAttribute attribute);

void
NvRmPrivHeapCarveoutGetInfo(NvU32 *CarveoutPhysBase, 
                            void  **pCarveout,
                            NvU32 *CarveoutSize);

NvS32 
NvRmPrivHeapCarveoutMemoryUsed(void);

NvS32 
NvRmPrivHeapCarveoutLargestFreeBlock(void);

/**
 * \Note    Not necessarily same as CarveoutSize returned by
 *          NvRmPrivHeapCarveoutGetInfo. No dependency on 
 *          carveout being mapped in.
 */
NvS32
NvRmPrivHeapCarveoutTotalSize(void);

NvRmPrivHeap *
NvRmPrivHeapIramInit(NvU32      length, 
                         NvRmPhysAddr base);

void
NvRmPrivHeapIramDeinit(void);

NvError
NvRmPrivHeapIramAlloc(NvU32 size, NvU32 align, NvRmPhysAddr *PAddr);

NvError
NvRmPrivHeapIramPreAlloc(NvRmPhysAddr Address, NvU32 Length);

void 
NvRmPrivHeapIramFree(NvRmPhysAddr addr);

void *
NvRmPrivHeapIramMemMap(NvRmPhysAddr base, NvU32 length, NvOsMemAttribute attribute);


// -- GART --

#define GART_PAGE_SIZE (4096)
#define GART_MAX_PAGES (4096)

/** 
 * Initialize the GART heap.  This identifies the GART heap's base address
 * and total size to the internal heap manager, so that it may allocate 
 * pages appropriately.
 * 
 * @param hDevice An RM device handle.
 * Size of the GART heap (bytes) and Base address of the GART heap space
 * are in GartMemoryInfo substructure of hDevice
 * 
 * @retval Pointer to the heap data structure, with updated values.
 */
NvRmPrivHeap *
NvRmPrivHeapGartInit(NvRmDeviceHandle hDevice);

void
NvRmPrivHeapGartDeinit(void);

/** 
 * Allocate GART storage space of the specified size (in units of GART_PAGE_SIZE).
 * Alignment is handled internally by this API, since it must align with the 
 * GART_PAGE_SIZE.  This API also updates the GART registers and returns the base
 * address pointer of the space allocated within the GART heap.
 * 
 * @see NvRmPrivHeapGartFree()
 * 
 * @param hDevice An RM device handle.
 * @param pPhysAddrArray Contains an array of page addresses.  This array should
 *  be created using an NVOS call that acquires the underlying memory address
 *  for each page to be mapped by the GART.
 * @param NumberOfPages The size (in pages, not bytes) of mapping requested.  Must
 *  be greater than 0.
 * @param PAddr Points to variable that will be updated with the base address of
 *  the next available GART page.
 * 
 * @retval The address of the first available GART page of the requested size.
 */
NvError
NvRmPrivAp15HeapGartAlloc(
    NvRmDeviceHandle hDevice,
    NvOsPageAllocHandle hPageHandle,
    NvU32 NumberOfPages,
    NvRmPhysAddr *PAddr);

NvError
NvRmPrivAp20HeapGartAlloc(
    NvRmDeviceHandle hDevice,
    NvOsPageAllocHandle hPageHandle,
    NvU32 NumberOfPages,
    NvRmPhysAddr *PAddr);

/**
 * Free the specified GART memory pages.
 * 
 * @see NvRmPrivHeapGartAlloc()
 * 
 * @param hDevice An RM device handle.
 * @param addr Base address (GART space) of the memory page(s) to free.
 *  NULL address pointers are ignored.
 * @param NumberOfPages The size (in pages, not bytes) of mapping to free.
 *  This needs to match the size indicated when allocated.
 */
void 
NvRmPrivAp15HeapGartFree(
    NvRmDeviceHandle hDevice,
    NvRmPhysAddr addr,
    NvU32 NumberOfPages);

void 
NvRmPrivAp20HeapGartFree(
    NvRmDeviceHandle hDevice,
    NvRmPhysAddr addr,
    NvU32 NumberOfPages);

/**
 * Suspend GART.
 */
void
NvRmPrivAp15GartSuspend(NvRmDeviceHandle hDevice);

void
NvRmPrivAp20GartSuspend(NvRmDeviceHandle hDevice);

/**
 * Resume GART.
 */
void
NvRmPrivAp15GartResume(NvRmDeviceHandle hDevice);

void
NvRmPrivAp20GartResume(NvRmDeviceHandle hDevice);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif
