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

#include "nvrm_memmgr.h"
#include "nvrm_memmgr_private.h"
#include "nvrm_heap_simple.h"
#include "ap15/ap15rm_private.h"
#include "nvos.h"
#include "nvbootargs.h"
#include "nvrm_chiplib.h"

/*  FIXME: temporary hack to force all Linux allocations to be page-aligned */
#if NVOS_IS_LINUX
#define NVRM_ALLOC_MIN_ALIGN 4096
#else
#define NVRM_ALLOC_MIN_ALIGN 4
#endif

#define NVRM_CHECK_PIN          0

#define NVRM_MEM_MAGIC          0xdead9812
#define NVRM_HMEM_CHECK(hMem) \
        do { \
            if (NVRM_HMEM_CHECK_MAGIC) { \
                NV_ASSERT(((NvU32)(hMem)&1)==0); \
                if (((NvU32)(hMem)&1)) { \
                    (hMem) = idtomem(hMem); \
                } \
                NV_ASSERT((hMem)->magic == NVRM_MEM_MAGIC); \
            } \
        } while(0)

static NvRmMemHandle idtomem(NvRmMemHandle hMem)
{
    NvOsDebugPrintf("RMMEM id->mem %08x\n",(int)hMem);
    return (NvRmMemHandle)((NvU32)hMem&~1UL);
}

#if NVRM_MEM_TRACE
#undef NvRmMemHandleCreate
#undef NvRmMemHandleFree
#undef NvRmMemGetId
#undef NvRmMemHandleFromId
#endif


/* GART related */
NvBool              gs_GartInited = NV_FALSE;
NvRmHeapSimple      gs_GartAllocator;
NvU32               *gs_GartSave = NULL;
static NvRmPrivHeap gs_GartHeap;
static NvUPtr       gs_GartBaseAddr;

static NvError      (*s_HeapGartAlloc)( NvRmDeviceHandle hDevice,
                        NvOsPageAllocHandle hPageHandle,
                        NvU32 NumberOfPages, NvRmPhysAddr *PAddr);
static void         (*s_HeapGartFree)( NvRmDeviceHandle hDevice,
                        NvRmPhysAddr addr, NvU32 NumberOfPages);
static void         (*s_GartSuspend)( NvRmDeviceHandle hDevice ) = NULL;
static void         (*s_GartResume)( NvRmDeviceHandle hDevice ) = NULL;


static NvU32 gs_NextPreservedMemHandleKey;
static NvRmMemHandle gs_PreservedHandles[NV_BOOTARGS_MAX_PRESERVED_MEMHANDLES];

/*
 * Notes:
 *
 * 1) The allocation of the handles should fall back to a block allocator
 *    that allocates say 1024 at a time to reduce heap fragmentation.
 *
 */

NvError NvRmMemHandleCreate(
    NvRmDeviceHandle  hRmDevice,
    NvRmMemHandle    *phMem,
    NvU32             size)
{
    NvRmMemHandle  pNewHandle = NULL;
    NvError        err        = NvSuccess;

#if NVCPU_IS_X86
    pNewHandle = NvOsAlloc(sizeof(*pNewHandle)+4);
    pNewHandle = (NvRmMemHandle)(((NvU32)pNewHandle+3)&~3UL);
#else
    pNewHandle = NvOsAlloc(sizeof(*pNewHandle));
#endif
    if (!pNewHandle)
    {
        err = NvError_InsufficientMemory;
        goto exit_gracefully;
    }

    NV_ASSERT(((NvU32)pNewHandle & 1) == 0);

    NvOsMemset(pNewHandle, 0, sizeof(*pNewHandle));
    pNewHandle->size      = size;
    pNewHandle->hRmDevice = hRmDevice;
    pNewHandle->PhysicalAddress =  NV_RM_INVALID_PHYS_ADDRESS;
    pNewHandle->VirtualAddress  =  NULL;
    pNewHandle->refcount = 1;
    pNewHandle->pin_count = 0;
    pNewHandle->coherency = NvOsMemAttribute_Uncached;
#if NVRM_HMEM_CHECK_MAGIC
    pNewHandle->magic = NVRM_MEM_MAGIC;
#endif

    *phMem                = pNewHandle;

exit_gracefully:
    if (err != NvSuccess)
        NvOsFree(pNewHandle);

    return err;
}

void NvRmPrivMemIncrRef(NvRmMemHandle hMem)
{
    NV_ASSERT(hMem);
    NvOsAtomicExchangeAdd32(&hMem->refcount, 1);
}

static void NvRmPrivMemFree(NvRmMemHandle hMem)
{
    if (!hMem)
        return;

    NVRM_HMEM_CHECK(hMem);

    if (!NV_RM_HMEM_IS_ALLOCATED(hMem))
        return;

    if(NVCPU_IS_X86 && !NvRmIsSimulation())
    {
        NvOsFree(hMem->VirtualAddress);
        hMem->VirtualAddress = NULL;
    }

    switch (hMem->heap)
    {
    case  NvRmHeap_ExternalCarveOut:
        NvRmPrivHeapCarveoutFree(hMem->PhysicalAddress);
        break;
    case NvRmHeap_IRam:
        NvRmPrivHeapIramFree(hMem->PhysicalAddress);
        break;
    case NvRmHeap_GART:
        NvRmPhysicalMemUnmap(hMem->VirtualAddress, hMem->size);
        (*s_HeapGartFree)(hMem->hRmDevice, hMem->PhysicalAddress,
            hMem->Pages);
        NvOsPageFree(hMem->hPageHandle);
        break;
    case NvRmHeap_External:
        NvOsPageUnmap(hMem->hPageHandle, hMem->VirtualAddress, hMem->size);
        NvOsPageFree(hMem->hPageHandle);
        break;
    default:
        break;
    }

    hMem->PhysicalAddress = NV_RM_INVALID_PHYS_ADDRESS;
    hMem->VirtualAddress = NULL;
    hMem->heap = 0;
#if NVRM_HMEM_CHECK_MAGIC
    hMem->magic = 0;
#endif
}

void NvRmMemHandleFree(NvRmMemHandle hMem)
{
    NvS32 old;
    NvOsMutexHandle mutex;

    if( !hMem )
    {
        return;
    }

    NVRM_HMEM_CHECK(hMem);
    old = NvOsAtomicExchangeAdd32(&hMem->refcount, -1);
    if(old > 1)
    {
        return;
    }

    NV_ASSERT(old != 0);

    mutex = hMem->hRmDevice->MemMgrMutex;
    NvOsMutexLock(mutex);

    NvRmPrivMemFree(hMem);
    NV_ASSERT(hMem->mapped == NV_FALSE);
    if (hMem->mapped == NV_TRUE)
    {
        NvRmMemUnmap(hMem, hMem->VirtualAddress, hMem->size);
    }

#if NVRM_HMEM_CHECK_MAGIC
    hMem->magic = 0;
#endif

    NvOsFree(hMem);

    NvOsMutexUnlock( mutex );
}

#define ERRATA_398959(ChipId) \
    ((ChipId).Id == 0x15 && (ChipId).Major == 1 && (ChipId).Minor == 1)


NvError NvRmMemAlloc(
    NvRmMemHandle     hMem,
    const NvRmHeap    *Heaps,
    NvU32             NumHeaps,
    NvU32             Alignment,
    NvOsMemAttribute  Coherency)
{
    // Default heap list does not include GART due to AP15 hardware bug.  GART
    // will be re-added to default heap list on AP20 and beyond.
    NvRmHeap DefaultHeaps[3];
    NvU32     i;
    NvError err;


    NV_ASSERT(hMem && (!NumHeaps || Heaps));
    NVRM_HMEM_CHECK(hMem);

    /* FIXME: Windows should support full caching for memory handles.
     * But not yet.
     */
#if !NVOS_IS_LINUX
    Coherency = NvOsMemAttribute_Uncached;
#endif

    if (NV_RM_HMEM_IS_ALLOCATED(hMem))
        return NvError_AlreadyAllocated;

    if(NVCPU_IS_X86 && !NvRmIsSimulation())
    {
        hMem->VirtualAddress = NvOsAlloc(hMem->size);
        if(hMem->VirtualAddress)
        {
            if (Heaps)
            {
                hMem->heap = Heaps[0];
            }

            return NvSuccess;
        }
        return NvError_InsufficientMemory;
    }

    NvOsMutexLock(hMem->hRmDevice->MemMgrMutex);

    if (hMem->size <= NVCPU_MIN_PAGE_SIZE &&
        (!NumHeaps || Heaps[0] != NvRmHeap_IRam))
    {
        DefaultHeaps[0] = NvRmHeap_External;
        DefaultHeaps[1] = NvRmHeap_ExternalCarveOut;
        Heaps = DefaultHeaps;
        NumHeaps = 2;
    }
    else if (!NumHeaps)
    {
        DefaultHeaps[0] = NvRmHeap_ExternalCarveOut;
        DefaultHeaps[1] = NvRmHeap_External;
        NumHeaps = 2;
        if (!ERRATA_398959(hMem->hRmDevice->ChipId))
            DefaultHeaps[NumHeaps++] = NvRmHeap_GART;
        Heaps = DefaultHeaps;
    }

    // 4 is the minimum alignment for any heap.
    if (Alignment < NVRM_ALLOC_MIN_ALIGN)
        Alignment = NVRM_ALLOC_MIN_ALIGN;

    for (i=0, err=NvError_InsufficientMemory;
         i<NumHeaps && err!=NvSuccess; i++)
    {
        if (Alignment > NVCPU_MIN_PAGE_SIZE &&
            (Heaps[i]==NvRmHeap_External || Heaps[i]==NvRmHeap_GART))
        {
            NV_ASSERT(!"Invalid alignment request to GART / External heap");
            continue;
        }

        switch (Heaps[i])
        {
        case NvRmHeap_ExternalCarveOut:
            err = NvRmPrivHeapCarveoutAlloc(hMem->size,
                Alignment, &hMem->PhysicalAddress);
            break;
        case NvRmHeap_IRam:
            err = NvRmPrivHeapIramAlloc(hMem->size,
                Alignment, &hMem->PhysicalAddress);
            break;
        case NvRmHeap_External:
            err = NvOsPageAlloc(hMem->size, Coherency,
                NvOsPageFlags_Contiguous, NVOS_MEM_READ_WRITE,
                 &hMem->hPageHandle);
            break;
        case NvRmHeap_GART:
            err = NvOsPageAlloc(hMem->size, Coherency,
                NvOsPageFlags_NonContiguous, NVOS_MEM_READ_WRITE,
                &hMem->hPageHandle);

            if (err != NvSuccess)
                break;

            hMem->Pages = (hMem->size+(GART_PAGE_SIZE-1))/GART_PAGE_SIZE;

            err = (*s_HeapGartAlloc)(hMem->hRmDevice,
                hMem->hPageHandle, hMem->Pages, &hMem->PhysicalAddress);

            if (err == NvSuccess)
                break;

            hMem->Pages = 0;
            NvOsPageFree(hMem->hPageHandle);
            hMem->hPageHandle = NULL;
            break;

        default:
            NV_ASSERT(!"Invalid heap in heaps array");
        }

        if (err==NvSuccess)
            break;
    }

    NvOsMutexUnlock(hMem->hRmDevice->MemMgrMutex);

    if (err == NvSuccess)
    {
        hMem->alignment = Alignment;
        hMem->heap = Heaps[i];
        hMem->coherency = Coherency;

        /* Don't cache virtual mappings for cacheable handles in the RM,
         * since there isn't a good way to ensure proper coherency */
        if (Coherency != NvOsMemAttribute_WriteBack)
        {
            NvRmMemMap(hMem, 0, hMem->size, NVOS_MEM_READ_WRITE,
                &hMem->VirtualAddress);
        }
    }

    return err;
}

NvU32 NvRmMemPin(NvRmMemHandle hMem)
{
    NvS32 old;

    NV_ASSERT(hMem);
    NVRM_HMEM_CHECK(hMem);

    old = NvOsAtomicExchangeAdd32(&hMem->pin_count, 1);

    NV_ASSERT(old != -1);

    // FIXME: finish implementation

    if (NVCPU_IS_X86 && !NvRmIsSimulation())
        return 0xFFFFFFFF;

    switch (hMem->heap)
    {
        case NvRmHeap_External:
            return (NvU32)NvOsPageAddress(hMem->hPageHandle, 0);
        case NvRmHeap_ExternalCarveOut:
        case NvRmHeap_GART:
        case NvRmHeap_IRam:
            return hMem->PhysicalAddress;
        default:
            NV_ASSERT(!"Unknown heap");
            return 0xFFFFFFFF;
    }
}

void NvRmMemPinMult(NvRmMemHandle *hMems, NvU32 *Addrs, NvU32 Count)
{
    NvU32 i;
    for( i = 0; i < Count; i++ )
    {
        Addrs[i] = NvRmMemPin( hMems[i] );
    }
}

void NvRmMemUnpin(NvRmMemHandle hMem)
{
    NvS32 old;

    if( !hMem )
    {
        return;
    }

    NVRM_HMEM_CHECK(hMem);

    old = NvOsAtomicExchangeAdd32(&hMem->pin_count, -1);
    NV_ASSERT(old != 0);
}

void NvRmMemUnpinMult(NvRmMemHandle *hMems, NvU32 Count)
{
    NvU32 i;
    for(i = 0; i < Count; i++)
    {
        NvRmMemUnpin(hMems[i]);
    }
}

NvU32 NvRmMemGetAddress(NvRmMemHandle hMem, NvU32 Offset)
{
    NV_ASSERT(hMem != NULL);
    NV_ASSERT(Offset < hMem->size);
    NVRM_HMEM_CHECK(hMem);

#if NVRM_CHECK_PIN
    NV_ASSERT( hMem->pin_count );
#endif

    if(NVCPU_IS_X86 && !NvRmIsSimulation())
    {
        return (NvU32)-1;
    }

    switch (hMem->heap)
    {
    case NvRmHeap_External:
        return (NvU32)NvOsPageAddress(hMem->hPageHandle, Offset);

    case NvRmHeap_ExternalCarveOut:
    case NvRmHeap_GART:
    case NvRmHeap_IRam:
        return (hMem->PhysicalAddress + Offset);

    default:
        NV_ASSERT(!"Unknown heap");
        break;
    }

    return (NvU32)-1;
}

/* Attempt to use the pre-mapped carveout or iram aperture on Windows CE */
#if !(NVOS_IS_LINUX && !NVCPU_IS_X86)
static void *NvRmMemMapGlobalHeap(
    NvRmPhysAddr base,
    NvU32 len,
    NvRmHeap heap,
    NvOsMemAttribute coherency)
{
    if (coherency == NvOsMemAttribute_WriteBack)
        return NULL;

    if (heap == NvRmHeap_ExternalCarveOut)
        return NvRmPrivHeapCarveoutMemMap(base, len, coherency);
    else if (heap == NvRmHeap_IRam)
        return NvRmPrivHeapIramMemMap(base, len, coherency);

    return NULL;
}
#else
#define NvRmMemMapGlobalHeap(base,len,heap,coherency) NULL
#endif



NvError NvRmMemMap(
    NvRmMemHandle  hMem,
    NvU32          Offset,
    NvU32          Size,
    NvU32          Flags,
    void          **pVirtAddr)
{
    NV_ASSERT(Offset + Size <= hMem->size);
    NVRM_HMEM_CHECK(hMem);

    if (!hMem->VirtualAddress)
        hMem->VirtualAddress = NvRmMemMapGlobalHeap(
            hMem->PhysicalAddress+Offset, Size, hMem->heap, hMem->coherency);

    if (NvRmIsSimulation())
        return NvError_InsufficientMemory;

    if (hMem->VirtualAddress)
    {
        *pVirtAddr = (NvU8 *)hMem->VirtualAddress + Offset;
        return NvSuccess;
    }

    switch (hMem->heap)
    {
    case NvRmHeap_ExternalCarveOut:
    case NvRmHeap_IRam:
#if !(NVOS_IS_LINUX && !NVCPU_IS_X86)
    case NvRmHeap_GART:
#endif
        return NvOsPhysicalMemMap(hMem->PhysicalAddress + Offset,
                   Size, hMem->coherency, Flags, pVirtAddr);
    case NvRmHeap_External:
        return NvOsPageMap(hMem->hPageHandle, Offset, Size, pVirtAddr);
    default:
        *pVirtAddr = NULL;
        return NvError_NotSupported;
    }
}


void NvRmMemUnmap(NvRmMemHandle hMem, void *pVirtAddr, NvU32 length)
{
    if (!hMem || !pVirtAddr || !length)
    {
        return;
    }

    NVRM_HMEM_CHECK(hMem);

    // No mappings ever get created in these cases
    if (NvRmIsSimulation() || NVCPU_IS_X86)
        return;

    if (hMem->VirtualAddress <= pVirtAddr &&
        ((NvU8*)hMem->VirtualAddress + hMem->size) >= (NvU8*)pVirtAddr)
        return;


    switch (hMem->heap)
    {
    case NvRmHeap_External:
        NvOsPageUnmap(hMem->hPageHandle, pVirtAddr, length);
        break;
    case NvRmHeap_ExternalCarveOut:
    case NvRmHeap_IRam:
#if NVOS_IS_WINDOWS
    case NvRmHeap_GART:
#endif
        NvOsPhysicalMemUnmap(pVirtAddr, length);
        break;
    default:
        break;
    }
}

NvU8 NvRmMemRd08(NvRmMemHandle hMem, NvU32 Offset)
{
    void *vaddr;

    NV_ASSERT(hMem->VirtualAddress != NULL);
    if (!hMem->VirtualAddress)
        return 0;

    vaddr = (NvU8 *)hMem->VirtualAddress + Offset;
    NV_ASSERT(Offset + 1 <= hMem->size);
    NVRM_HMEM_CHECK(hMem);
    return NV_READ8(vaddr);
}

NvU16 NvRmMemRd16(NvRmMemHandle hMem, NvU32 Offset)
{
    void *vaddr;

    NV_ASSERT(hMem->VirtualAddress != NULL);
    if (!hMem->VirtualAddress)
        return 0;

    vaddr = (NvU8 *)hMem->VirtualAddress + Offset;
    NV_ASSERT(Offset + 2 <= hMem->size);
    NVRM_HMEM_CHECK(hMem);
    return NV_READ16(vaddr);
}

NvU32 NvRmMemRd32(NvRmMemHandle hMem, NvU32 Offset)
{
    void *vaddr;

    NV_ASSERT(hMem->VirtualAddress != NULL);
    if (!hMem->VirtualAddress)
        return 0;

    vaddr = (NvU8 *)hMem->VirtualAddress + Offset;
    NV_ASSERT(Offset + 4 <= hMem->size);
    NVRM_HMEM_CHECK(hMem);
    return NV_READ32(vaddr);
}

void NvRmMemWr08(NvRmMemHandle hMem, NvU32 Offset, NvU8 Data)
{
    void *vaddr;

    NV_ASSERT(hMem->VirtualAddress != NULL);
    if (!hMem->VirtualAddress)
        return;

    vaddr = (NvU8 *)hMem->VirtualAddress + Offset;
    NV_ASSERT(Offset + 1 <= hMem->size);
    NVRM_HMEM_CHECK(hMem);
    NVRM_RMC_TRACE((&hMem->hRmDevice->rmc, "MemoryWrite8 0x%x 0x%x\n",
                    hMem->PhysicalAddress + Offset, Data));
    NV_WRITE08(vaddr, Data);
}

void NvRmMemWr16(NvRmMemHandle hMem, NvU32 Offset, NvU16 Data)
{
    void *vaddr;

    NV_ASSERT(hMem->VirtualAddress != NULL);
    if (!hMem->VirtualAddress)
        return;

    vaddr = (NvU8 *)hMem->VirtualAddress + Offset;
    NV_ASSERT(Offset + 2 <= hMem->size);
    NVRM_HMEM_CHECK(hMem);
    NVRM_RMC_TRACE((&hMem->hRmDevice->rmc, "MemoryWrite16 0x%x 0x%x\n",
                    hMem->PhysicalAddress + Offset, Data));
    NV_WRITE16(vaddr, Data);
}

void NvRmMemWr32(NvRmMemHandle hMem, NvU32 Offset, NvU32 Data)
{
    void *vaddr;

    NV_ASSERT(hMem->VirtualAddress != NULL);
    if (!hMem->VirtualAddress)
        return;

    vaddr = (NvU8 *)hMem->VirtualAddress + Offset;
    NV_ASSERT(Offset + 4 <= hMem->size);
    NVRM_HMEM_CHECK(hMem);
    NVRM_RMC_TRACE((&hMem->hRmDevice->rmc, "MemoryWrite32 0x%x 0x%x\n",
                    hMem->PhysicalAddress + Offset, Data));
    NV_WRITE32(vaddr, Data);
}

void NvRmMemRead(NvRmMemHandle hMem, NvU32 Offset, void *pDst, NvU32 Size)
{
    void *vaddr = (NvU8 *)hMem->VirtualAddress + Offset;
    NV_ASSERT(Offset + Size <= hMem->size);
    NVRM_HMEM_CHECK(hMem);
    NV_READ(pDst, vaddr, Size);
}

void NvRmMemWrite(
    NvRmMemHandle hMem,
    NvU32 Offset,
    const void *pSrc,
    NvU32 Size)
{
    void *vaddr = (NvU8 *)hMem->VirtualAddress + Offset;
#if NV_DEF_RMC_TRACE
    NvU32 i;
#endif

    NV_ASSERT(Offset + Size <= hMem->size);
    NVRM_HMEM_CHECK(hMem);

#if NV_DEF_RMC_TRACE
    for (i = 0; i < Size; i++)
    {
        NvU8 Data = ((const NvU8 *)pSrc)[i];
        NVRM_RMC_TRACE((&hMem->hRmDevice->rmc, "MemoryWrite8 0x%x 0x%x\n",
                        hMem->PhysicalAddress + i, Data));
    }
#endif

    NV_WRITE(vaddr, pSrc, Size);
}

void NvRmMemReadStrided(
    NvRmMemHandle hMem,
    NvU32 Offset,
    NvU32 SrcStride,
    void *pDst,
    NvU32 DstStride,
    NvU32 ElementSize,
    NvU32 Count)
{
    if ((ElementSize == SrcStride) && (ElementSize == DstStride))
    {
        NvRmMemRead(hMem, Offset, pDst, ElementSize * Count);
    }
    else
    {
        while (Count--)
        {
            NvRmMemRead(hMem, Offset, pDst, ElementSize);
            Offset += SrcStride;
            pDst = (NvU8 *)pDst + DstStride;
        }
    }
}

void NvRmMemWriteStrided(
    NvRmMemHandle hMem,
    NvU32 Offset,
    NvU32 DstStride,
    const void *pSrc,
    NvU32 SrcStride,
    NvU32 ElementSize,
    NvU32 Count)
{
    if ((ElementSize == SrcStride) && (ElementSize == DstStride))
    {
        NvRmMemWrite(hMem, Offset, pSrc, ElementSize * Count);
    }
    else
    {
        while (Count--)
        {
            NvRmMemWrite(hMem, Offset, pSrc, ElementSize);
            Offset += DstStride;
            pSrc = (const NvU8 *)pSrc + SrcStride;
        }
    }
}

void NvRmMemMove(
        NvRmMemHandle dstHMem,
        NvU32 dstOffset,
        NvRmMemHandle srcHMem,
        NvU32 srcOffset,
        NvU32 Size)
{
    NvU32 i;

    NV_ASSERT(dstOffset + Size <= dstHMem->size);
    NV_ASSERT(srcOffset + Size <= srcHMem->size);
    NVRM_HMEM_CHECK(dstHMem);
    NVRM_HMEM_CHECK(srcHMem);

    if (((dstHMem->PhysicalAddress |
          srcHMem->PhysicalAddress |
          dstOffset                       |
          srcOffset                       |
          Size) & 3) == 0)
    {
        // everything is nicely word aligned
        if (dstHMem == srcHMem && srcOffset < dstOffset)
        {
            for (i=Size; i; )
            {
                NvU32 data;
                i -= 4;
                data = NvRmMemRd32(srcHMem, srcOffset+i);
                NvRmMemWr32(dstHMem, dstOffset+i, data);
            }
        }
        else
        {
            for (i=0; i < Size; i+=4)
            {
                NvU32 data = NvRmMemRd32(srcHMem, srcOffset+i);
                NvRmMemWr32(dstHMem, dstOffset+i, data);
            }
        }
    }
    else
    {
        // fall back to writing one byte at a time
        if (dstHMem == srcHMem && srcOffset < dstOffset)
        {
            for (i=Size; i--;)
            {
                NvU8 data = NvRmMemRd08(srcHMem, srcOffset+i);
                NvRmMemWr08(dstHMem, dstOffset+i, data);
            }
        }
        else
        {
            for (i=0; i < Size; ++i)
            {
                NvU8 data = NvRmMemRd08(srcHMem, srcOffset+i);
                NvRmMemWr08(dstHMem, dstOffset+i, data);
            }
        }
    }
}

void NvRmMemCacheMaint(
    NvRmMemHandle hMem,
    void         *pMapping,
    NvU32         Size,
    NvBool        Writeback,
    NvBool        Invalidate)
{
    if (!hMem || !pMapping || !Size || !(Writeback || Invalidate))
        return;

    NVRM_HMEM_CHECK(hMem);
    NV_ASSERT((NvU8*)pMapping+Size <= (NvU8*)hMem->VirtualAddress+Size);
    if (Writeback && Invalidate)
        NvOsDataCacheWritebackInvalidateRange(pMapping, Size);
    else if (Writeback)
        NvOsDataCacheWritebackRange(pMapping, Size);
    else {
        NV_ASSERT(!"Invalidate-only cache maintenance not supported in NvOs");
    }
}

NvU32 NvRmMemGetSize(NvRmMemHandle hMem)
{
    NV_ASSERT(hMem);
    NVRM_HMEM_CHECK(hMem);
    return hMem->size;
}

NvU32 NvRmMemGetAlignment(NvRmMemHandle hMem)
{
    NV_ASSERT(hMem);
    NVRM_HMEM_CHECK(hMem);
    return hMem->alignment;
}

NvU32 NvRmMemGetCacheLineSize(void)
{
    // !!! FIXME:  Currently for all our chips (ap15)
    //             both the L1 and L2 cache line sizes
    //             are 32 bytes.  If this ever changes
    //             we need a way to figure it out on
    //             a chip by chip basis.
    return 32;
}

NvRmHeap NvRmMemGetHeapType(NvRmMemHandle hMem, NvU32 *BaseAddr)
{
    NV_ASSERT(hMem);
    NVRM_HMEM_CHECK(hMem);

    if (hMem->heap == NvRmHeap_External)
        *BaseAddr = (NvU32)NvOsPageAddress(hMem->hPageHandle, 0);
    else
        *BaseAddr = hMem->PhysicalAddress;

    return hMem->heap;
}


void *NvRmHostAlloc(size_t size)
{
    return NvOsAlloc(size);
}

void NvRmHostFree(void *ptr)
{
    NvOsFree(ptr);
}


NvError NvRmMemMapIntoCallerPtr(
    NvRmMemHandle hMem,
    void  *pCallerPtr,
    NvU32 Offset,
    NvU32 Size)
{
    NvError err;
    NVRM_HMEM_CHECK(hMem);

    // The caller should be asking for an even number of pages.  not strictly
    // required, but the caller has already had to do the work to calculate the
    // required number of pages so they might as well pass in a nice round
    // number, which makes it easier to find bugs.
    NV_ASSERT( (Size & (NVCPU_MIN_PAGE_SIZE-1)) == 0);

    // Make sure the supplied virtual address is page aligned.
    NV_ASSERT( (((NvUPtr)pCallerPtr) & (NVCPU_MIN_PAGE_SIZE-1)) == 0);

    if (hMem->heap == NvRmHeap_External ||
        hMem->heap == NvRmHeap_GART)
    {
        err = NvOsPageMapIntoPtr(hMem->hPageHandle,
                                 pCallerPtr,
                                 Offset,
                                 Size);
    }
    else if (hMem->heap == NvRmHeap_ExternalCarveOut ||
             hMem->heap == NvRmHeap_IRam)
    {
        // The caller is responsible for sending a size that
        // is the correct number of pages, including this pageoffset
        // at the beginning of the first page.
        NvU32 PhysicalAddr = hMem->PhysicalAddress + Offset;
        PhysicalAddr = PhysicalAddr & ~(NVCPU_MIN_PAGE_SIZE-1);

        err = NvOsPhysicalMemMapIntoCaller(
                            pCallerPtr,
                            PhysicalAddr,
                            Size,
                            NvOsMemAttribute_Uncached,
                            NVOS_MEM_WRITE | NVOS_MEM_READ);
    }
    else
    {
        return NvError_NotImplemented;
    }

    return err;
}


NvU32 NvRmMemGetId(NvRmMemHandle hMem)
{
    NvU32 id = (NvU32)hMem;

    // !!! FIXME: Need to really create a unique id to handle the case where
    // hMem is freed, and then the next allocated hMem returns the same pointer
    // value.

    NVRM_HMEM_CHECK(hMem);
    NV_ASSERT(((NvU32)hMem & 1) == 0);
    if (!hMem || ((NvU32)hMem & 1))
        return 0;

#if NVRM_MEM_CHECK_ID
    id |= 1;
#endif

    return id;
}

NvError NvRmMemHandleFromId(NvU32 id, NvRmMemHandle *phMem)
{
    NvRmMemHandle hMem;
    // !!! FIXME: (see comment in GetId).  Specifically handle the case where
    // the memory handle has already been freed.

#if NVRM_MEM_CHECK_ID
    *phMem = NULL;
    NV_ASSERT(id & 1);
    if (!(id & 1))
        return NvError_BadParameter;
#endif

    hMem = (NvRmMemHandle)(id & ~1UL);

    NVRM_HMEM_CHECK(hMem);

    NvRmPrivMemIncrRef(hMem);

    *phMem = hMem;
    return NvSuccess;
}

NvError NvRmMemHandlePreserveHandle(
    NvRmMemHandle hMem,
    NvU32        *pKey)
{
    NvError e;
    NvBootArgsPreservedMemHandle ArgMh;

    NV_ASSERT(hMem && pKey);
    NvOsMutexLock(hMem->hRmDevice->MemMgrMutex);
    if (gs_NextPreservedMemHandleKey >=
        (NvU32)NvBootArgKey_PreservedMemHandle_Num)
    {
        e = NvError_InsufficientMemory;
        goto clean;
    }

    ArgMh.Address = (NvUPtr)hMem->PhysicalAddress;
    ArgMh.Size    = hMem->size;

    e = NvOsBootArgSet(gs_NextPreservedMemHandleKey, &ArgMh, sizeof(ArgMh));

    if (e==NvSuccess)
    {
        *pKey = gs_NextPreservedMemHandleKey;
        gs_NextPreservedMemHandleKey++;
    }
    else
    {
        *pKey = 0;
        e = NvError_InsufficientMemory;
    }

 clean:
    NvOsMutexUnlock(hMem->hRmDevice->MemMgrMutex);
    return e;
}


NvError NvRmMemHandleClaimPreservedHandle(
    NvRmDeviceHandle hRm,
    NvU32 Key,
    NvRmMemHandle *pMem)
{
    NvU32 i;
    NV_ASSERT(hRm && pMem && Key);
    if (!pMem || !hRm ||
        Key<NvBootArgKey_PreservedMemHandle_0 ||
        Key>=NvBootArgKey_PreservedMemHandle_Num)
        return NvError_BadParameter;

    *pMem = NULL;

    NvOsMutexLock(hRm->MemMgrMutex);
    i = Key - NvBootArgKey_PreservedMemHandle_0;
    *pMem = gs_PreservedHandles[i];
    gs_PreservedHandles[i] = NULL;
    NvOsMutexUnlock(hRm->MemMgrMutex);

    if (*pMem)
        return NvSuccess;

    return NvError_InsufficientMemory;
}


NvRmPrivHeap *NvRmPrivHeapGartInit(NvRmDeviceHandle hRmDevice)
{
    NvError         err;
    NvU32           length = hRmDevice->GartMemoryInfo.size;
    NvRmPhysAddr    base = hRmDevice->GartMemoryInfo.base;
    NvRmModuleCapability caps[2];
    NvRmModuleCapability *pCap = NULL;

    caps[0].MajorVersion = 1; // AP15, AP16
    caps[0].MinorVersion = 0;
    caps[0].EcoLevel = 0;
    caps[0].Capability = &caps[0];

    caps[1].MajorVersion = 1; // AP20/T20
    caps[1].MinorVersion = 1;
    caps[1].EcoLevel = 0;
    caps[1].Capability = &caps[1];

    NV_ASSERT_SUCCESS(NvRmModuleGetCapabilities(
        hRmDevice,
        NvRmPrivModuleID_MemoryController,
        caps,
        NV_ARRAY_SIZE(caps),
        (void**)&pCap));

    err = NvRmPrivHeapSimple_HeapAlloc(
        base,
        length,
        &gs_GartAllocator);

    if (err != NvSuccess)
        return NULL;

    gs_GartHeap.heap            = NvRmHeap_GART;
    gs_GartHeap.length          = length;
    gs_GartHeap.PhysicalAddress = base;

    gs_GartBaseAddr = (NvUPtr)base;
    (void)gs_GartBaseAddr;

    if ((pCap->MajorVersion == 1) && (pCap->MinorVersion == 0))
    {
        s_HeapGartAlloc = NvRmPrivAp15HeapGartAlloc;
        s_HeapGartFree = NvRmPrivAp15HeapGartFree;
        s_GartSuspend = NvRmPrivAp15GartSuspend;
        s_GartResume = NvRmPrivAp15GartResume;
    }
    else
    {
        s_HeapGartAlloc = NvRmPrivAp20HeapGartAlloc;
        s_HeapGartFree = NvRmPrivAp20HeapGartFree;
        s_GartSuspend = NvRmPrivAp20GartSuspend;
        s_GartResume = NvRmPrivAp20GartResume;
    }

    return &gs_GartHeap;
}

void NvRmPrivHeapGartDeinit(void)
{
    // deinit the gart allocator

    NvRmPrivHeapSimple_HeapFree(&gs_GartAllocator);
    NvOsMemset(&gs_GartHeap, 0, sizeof(gs_GartHeap));
    NvOsMemset(&gs_GartAllocator, 0, sizeof(gs_GartAllocator));
    NvOsFree( gs_GartSave );
    gs_GartInited = NV_FALSE;
}

void NvRmPrivGartSuspend(NvRmDeviceHandle hDevice)
{
    NV_ASSERT(s_GartSuspend);
    (*s_GartSuspend)( hDevice );
}

void NvRmPrivGartResume(NvRmDeviceHandle hDevice)
{
    NV_ASSERT(s_GartResume);
    (*s_GartResume)( hDevice );
}

void NvRmPrivPreservedMemHandleInit(NvRmDeviceHandle hRm)
{
    unsigned int i;
    NvBootArgsPreservedMemHandle mem;

    NvOsMemset(gs_PreservedHandles, 0, sizeof(gs_PreservedHandles));
    gs_NextPreservedMemHandleKey = (NvU32)NvBootArgKey_PreservedMemHandle_0;

    for (i=NvBootArgKey_PreservedMemHandle_0;
         i<NvBootArgKey_PreservedMemHandle_Num; i++)
    {
        NvRmMemHandle hMem;
        NvU32 j;

        if (NvOsBootArgGet(i, &mem, sizeof(mem))!=NvSuccess)
            break;

        if (!mem.Address || !mem.Size)
            break;

        if (NvRmMemHandleCreate(hRm, &hMem, mem.Size)!=NvSuccess)
            continue;

        hMem->PhysicalAddress = mem.Address;
        j = mem.Address;
        hMem->alignment = 1;
        while ((j & 1) == 0)
        {
            hMem->alignment <<= 1;
            j >>= 1;
        }

        if (NvRmPrivHeapCarveoutPreAlloc(mem.Address, mem.Size)==NvSuccess)
        {
            hMem->heap = NvRmHeap_ExternalCarveOut;
            hMem->VirtualAddress = NvRmPrivHeapCarveoutMemMap(mem.Address,
                                        mem.Size, NvOsMemAttribute_Uncached);
        }
        else if (NvRmPrivHeapIramPreAlloc(mem.Address, mem.Size)==NvSuccess)
        {
            hMem->heap = NvRmHeap_IRam;
            hMem->VirtualAddress = NvRmPrivHeapIramMemMap(mem.Address,
                                        mem.Size, NvOsMemAttribute_Uncached);
        }

        if (hMem->heap)
            gs_PreservedHandles[i-NvBootArgKey_PreservedMemHandle_0] = hMem;
        else
            NvRmMemHandleFree(hMem);
    }
}

NvError NvRmMemGetStat(NvRmMemStat Stat, NvS32* Result)
{
    /* Main point of this function is to be compatible backwards and forwards,
     * i.e., breaking analysis apps is the thing to avoid.
     * Minimum hassle - maximum impact.
     * Performance is not that big of a deal.
     * Could be extended to use NvS64 as return value. However, NvS64 is
     * slightly more challenging in terms of printing etc. at the client side.
     * This function should return counts as raw data as possible; conversions
     * to percentages or anything like that should be left to the client.
     */
    if (Stat == NvRmMemStat_TotalCarveout)
    {
        *Result = NvRmPrivHeapCarveoutTotalSize();
    }
    else if (Stat == NvRmMemStat_UsedCarveout)
    {
        *Result = NvRmPrivHeapCarveoutMemoryUsed();
    }
    else if (Stat == NvRmMemStat_LargestFreeCarveoutBlock)
    {
        *Result = NvRmPrivHeapCarveoutLargestFreeBlock();
    }
    else if (Stat == NvRmMemStat_TotalGart)
    {
        *Result = gs_GartHeap.length;
    }
    else if (Stat == NvRmMemStat_UsedGart)
    {
        *Result = NvRmPrivHeapSimpleMemoryUsed(&gs_GartAllocator);
    }
    else if (Stat == NvRmMemStat_LargestFreeGartBlock)
    {
        *Result = NvRmPrivHeapSimpleLargestFreeBlock(&gs_GartAllocator);
    }
    else
    {
        return NvError_BadParameter;
    }
    return NvSuccess;
}
