/*
 * Copyright (c) 2009 NVIDIA Corporation.
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

#include "ap20/armc.h"
#include "nvrm_heap.h"
#include "nvrm_heap_simple.h"
#include "nvrm_hwintf.h"
#include "ap15/ap15rm_private.h"
#include "nvassert.h"
#include "nvcommon.h"
#include "nvrm_drf.h"


/**
 * Initialize the GART entries, and enable the GART 
 */

#define GART_PAGE_SHIFT (12)
#define GART_PAGE_SIZE (4096)

extern NvBool           gs_GartInited;
extern NvRmHeapSimple   gs_GartAllocator;
extern NvU32            *gs_GartSave;

/**
 * Initializes all of the TLB entries in the GART and enables GART translations
 * All entries are initially marked invalid.
 *
 * @param hDevice The RM device handle.
 */
static NvError
NvRmPrivAp20InitGART(NvRmDeviceHandle hDevice);
static NvError
NvRmPrivAp20InitGART(NvRmDeviceHandle hDevice)
{
    NvU32 GartSize;
    NvU32 GartEntries;
    NvU32 GartEntry;
    NvU32 reg;
    NvU32 data;

    NV_ASSERT(hDevice != NULL);

    NvRmModuleGetBaseAddress(
        hDevice, NvRmPrivModuleID_Gart, NULL, &GartSize);

    GartEntries = GartSize /  GART_PAGE_SIZE;

    gs_GartSave = NvOsAlloc( sizeof(NvU32) * GartEntries );
    if ( NULL == gs_GartSave )
        return NvError_InsufficientMemory;

    data = NV_DRF_NUM(MC, GART_ENTRY_DATA, GART_ENTRY_DATA_PHYS_ADDR_VALID, 0);
    for (GartEntry = 0; GartEntry < GartEntries; ++GartEntry)
    {
        // set the address
        reg = NV_DRF_NUM(MC, GART_ENTRY_ADDR, GART_ENTRY_ADDR_TABLE_ADDR, GartEntry);
        NV_REGW(hDevice, NvRmPrivModuleID_MemoryController, 0, MC_GART_ENTRY_ADDR_0, reg);

        // mark the entry invalid
        NV_REGW(hDevice, NvRmPrivModuleID_MemoryController, 0, MC_GART_ENTRY_DATA_0, data);
    }
    
    // now enable the GART
    reg = NV_DRF_DEF(MC, GART_CONFIG, GART_ENABLE, ENABLE);
    NV_REGW(hDevice, NvRmPrivModuleID_MemoryController, 0, MC_GART_CONFIG_0, reg);
    return NvSuccess;
}

NvError
NvRmPrivAp20HeapGartAlloc(
    NvRmDeviceHandle hDevice,
    NvOsPageAllocHandle hPageHandle,
    NvU32 NumberOfPages,
    NvRmPhysAddr *PAddr)
{
    NvError result = NvSuccess;
    NvU32 reg;
    NvU32 i, data;
    NvU32 FirstGartPage;

    NV_ASSERT(hDevice);
    NV_ASSERT(hPageHandle);

    result = NvRmPrivHeapSimpleAlloc(
        &gs_GartAllocator,
        NumberOfPages*GART_PAGE_SIZE,
        GART_PAGE_SIZE,
        PAddr);

    if (result != NvSuccess)
        return result;

    FirstGartPage = *PAddr;

    /* Check that the GART address exists and is page aligned */
    NV_ASSERT(FirstGartPage);
    NV_ASSERT((FirstGartPage & (GART_PAGE_SIZE - 1)) == 0);

    NvOsMutexLock(hDevice->mutex);

    // FIXME: Normally we would do this at init time, but it takes and
    //        egregious amount of csim time, so I'm defering it or the 3d guys
    //        will complain to me, and then to my boss, and then their boss, and then their bosses boss...
    if (gs_GartInited == NV_FALSE)
    {
        result = NvRmPrivAp20InitGART(hDevice);
        if ( NvSuccess != result )
            goto fail;
        gs_GartInited = NV_TRUE;
    }

    for (i = 0; i < NumberOfPages; i++)
    {
        data = (NvU32)NvOsPageAddress(hPageHandle, i * GART_PAGE_SIZE);

        /* Check that each physical address is page aligned */
        NV_ASSERT((data & (GART_PAGE_SIZE - 1)) == 0);

        reg = NV_DRF_NUM(MC, GART_ENTRY_ADDR, GART_ENTRY_ADDR_TABLE_ADDR, ((FirstGartPage + i*GART_PAGE_SIZE) >> GART_PAGE_SHIFT));
        NV_REGW(hDevice, NvRmPrivModuleID_MemoryController, 0, MC_GART_ENTRY_ADDR_0, reg);

        reg = 
            NV_DRF_NUM(MC, GART_ENTRY_DATA, GART_ENTRY_DATA_PHYS_ADDR_VALID, 1) |
            NV_DRF_NUM(MC, GART_ENTRY_DATA, GART_ENTRY_DATA_PHYS_ADDR, (data >> GART_PAGE_SHIFT));

        NV_REGW(hDevice, NvRmPrivModuleID_MemoryController, 0, MC_GART_ENTRY_DATA_0, reg);
        // lame, on csim we have to read this back to make sure the GART entry is valid before we hit the mc
        // with data to this address.
        (void)NV_REGR(hDevice, NvRmPrivModuleID_MemoryController, 0, MC_GART_ENTRY_DATA_0);
    }
fail:
    NvOsMutexUnlock(hDevice->mutex);

    return result;
}

void 
NvRmPrivAp20HeapGartFree(
    NvRmDeviceHandle hDevice,
    NvRmPhysAddr addr,
    NvU32 NumberOfPages)
{
    NvU32        i;
    NvU32        reg;
    NvU32        data;

    NV_ASSERT(hDevice);

    if (addr && NumberOfPages)
    {
        // Invalidate GART page table entries
        data = NV_DRF_NUM(MC, GART_ENTRY_DATA, GART_ENTRY_DATA_PHYS_ADDR_VALID, 0);
        for (i = 0; i < NumberOfPages; i++)
        {
            // set the address
            reg = NV_DRF_NUM(MC, GART_ENTRY_ADDR, GART_ENTRY_ADDR_TABLE_ADDR, ((addr + i*GART_PAGE_SIZE) >> GART_PAGE_SHIFT));
            NV_REGW(hDevice, NvRmPrivModuleID_MemoryController, 0, MC_GART_ENTRY_ADDR_0, reg);

            // mark the entry invalid
            NV_REGW(hDevice, NvRmPrivModuleID_MemoryController, 0, MC_GART_ENTRY_DATA_0, data);
        }
        NvRmPrivHeapSimpleFree(&gs_GartAllocator, addr);
    }
}


void
NvRmPrivAp20GartSuspend(NvRmDeviceHandle hDevice)
{
    NvU32 reg;
    NvU32 GartSize;
    NvU32 GartEntries;
    NvU32 GartEntry;

    NvOsMutexLock(hDevice->mutex);
    if (gs_GartInited == NV_TRUE)
    {
        NvRmModuleGetBaseAddress(
            hDevice, NvRmPrivModuleID_Gart, NULL, &GartSize);
        GartEntries = GartSize / GART_PAGE_SIZE;

        for (GartEntry = 0; GartEntry < GartEntries; GartEntry++)
        {
            reg = NV_DRF_NUM(MC, GART_ENTRY_ADDR, GART_ENTRY_ADDR_TABLE_ADDR,
                    GartEntry);
            NV_REGW(hDevice, NvRmPrivModuleID_MemoryController, 0,
                    MC_GART_ENTRY_ADDR_0, reg);
            gs_GartSave[GartEntry] = NV_REGR(hDevice,
                    NvRmPrivModuleID_MemoryController, 0, MC_GART_ENTRY_DATA_0);
        }
    }
    NvOsMutexUnlock(hDevice->mutex);
}

void
NvRmPrivAp20GartResume(NvRmDeviceHandle hDevice)
{
    NvU32 reg;
    NvU32 GartSize;
    NvU32 GartEntries;
    NvU32 GartEntry;

    NvOsMutexLock(hDevice->mutex);
    if (gs_GartInited == NV_TRUE)
    {
        NvRmModuleGetBaseAddress(
            hDevice, NvRmPrivModuleID_Gart, NULL, &GartSize);
        GartEntries = GartSize / GART_PAGE_SIZE;

        for (GartEntry = 0; GartEntry < GartEntries; GartEntry++)
        {
            reg = NV_DRF_NUM(MC, GART_ENTRY_ADDR, GART_ENTRY_ADDR_TABLE_ADDR,
                                GartEntry);
            NV_REGW(hDevice, NvRmPrivModuleID_MemoryController, 0,
                                MC_GART_ENTRY_ADDR_0, reg);
            NV_REGW(hDevice, NvRmPrivModuleID_MemoryController, 0,
                                MC_GART_ENTRY_DATA_0, gs_GartSave[GartEntry] );
        }

        reg = NV_DRF_DEF(MC, GART_CONFIG, GART_ENABLE, ENABLE);
        NV_REGW(hDevice, NvRmPrivModuleID_MemoryController, 0,
                                            MC_GART_CONFIG_0, reg);

    }
    NvOsMutexUnlock(hDevice->mutex);
}

