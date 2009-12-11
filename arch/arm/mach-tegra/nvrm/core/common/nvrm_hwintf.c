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

#if NV_IS_AVP
#define NV_DEF_RMC_TRACE                0   // NO TRACING FOR AVP
#endif

#include "nvcommon.h"
#include "nvassert.h"
#include "nvrm_hardware_access.h"
#include "nvrm_module_private.h"
#include "nvrm_rmctrace.h"
#include "nvrm_chiplib.h"
#include "nvrm_hwintf.h"

// FIXME:  This file needs to be split up, when we build user/kernel 
//         The NvRegr/NvRegw should thunk to the kernel since the rm
//         handle is not usable in user space.
//
//         NvRmPhysicalMemMap/NvRmPhysicalMemUnmap need to be in user space.
//

NvU32 NvRegr( NvRmDeviceHandle rm, NvRmModuleID aperture, NvU32 instance,
    NvU32 offset )
{
    void *addr;
    NvRmModuleTable *tbl;
    NvRmModuleInstance *inst;

    tbl = NvRmPrivGetModuleTable( rm );

    NV_ASSERT( tbl->Modules[aperture].Index != NVRM_MODULE_INVALID );

    inst = tbl->ModInst + tbl->Modules[aperture].Index + instance;
    addr = (void *)((NvUPtr)inst->VirtAddr + offset);

    return NV_READ32( addr );
}

void NvRegw( NvRmDeviceHandle rm, NvRmModuleID aperture, NvU32 instance,
    NvU32 offset, NvU32 data )
{
    void *addr;
    NvRmModuleTable *tbl;
    NvRmModuleInstance *inst;

    tbl = NvRmPrivGetModuleTable( rm );

    NV_ASSERT( tbl->Modules[aperture].Index != NVRM_MODULE_INVALID );

    inst = tbl->ModInst + tbl->Modules[aperture].Index + instance;
    addr = (void *)((NvUPtr)inst->VirtAddr + offset);

    NV_WRITE32( addr, data );
}

NvU8 NvRegr08( NvRmDeviceHandle rm, NvRmModuleID aperture, NvU32 instance,
    NvU32 offset )
{
    void *addr;
    NvRmModuleTable *tbl;
    NvRmModuleInstance *inst;

    tbl = NvRmPrivGetModuleTable( rm );

    NV_ASSERT( tbl->Modules[aperture].Index != NVRM_MODULE_INVALID );

    inst = tbl->ModInst + tbl->Modules[aperture].Index + instance;
    addr = (void *)((NvUPtr)inst->VirtAddr + offset);

    return NV_READ8( addr );
}


void NvRegw08( NvRmDeviceHandle rm, NvRmModuleID aperture, NvU32 instance,
    NvU32 offset, NvU8 data )
{
    void *addr;
    NvRmModuleTable *tbl;
    NvRmModuleInstance *inst;

    tbl = NvRmPrivGetModuleTable( rm );

    NV_ASSERT( tbl->Modules[aperture].Index != NVRM_MODULE_INVALID );

    inst = tbl->ModInst + tbl->Modules[aperture].Index + instance;
    addr = (void *)((NvUPtr)inst->VirtAddr + offset);

    NV_WRITE08( addr, data );
}



void NvRegrm( NvRmDeviceHandle rm, NvRmModuleID aperture, NvU32 instance,
    NvU32 num, const NvU32 *offsets, NvU32 *values )
{
    void *addr;
    NvRmModuleTable *tbl;
    NvRmModuleInstance *inst;
    NvU32 i;

    tbl = NvRmPrivGetModuleTable( rm );

    NV_ASSERT( tbl->Modules[aperture].Index != NVRM_MODULE_INVALID );

    inst = tbl->ModInst + tbl->Modules[aperture].Index + instance;

    for( i = 0; i < num; i++ )
    {
        addr = (void *)((NvUPtr)inst->VirtAddr + offsets[i]);

        values[i] = NV_READ32( addr );
    }
}

void NvRegwm( NvRmDeviceHandle rm, NvRmModuleID aperture, NvU32 instance,
    NvU32 num, const NvU32 *offsets, const NvU32 *values )
{
    void *addr;
    NvRmModuleTable *tbl;
    NvRmModuleInstance *inst;
    NvU32 i;

    tbl = NvRmPrivGetModuleTable( rm );

    NV_ASSERT( tbl->Modules[aperture].Index != NVRM_MODULE_INVALID );

    inst = tbl->ModInst + tbl->Modules[aperture].Index + instance;

    for( i = 0; i < num; i++ )
    {
        addr = (void *)((NvUPtr)inst->VirtAddr + offsets[i]);

        NV_WRITE32( addr, values[i] );
    }
}

void NvRegwb( NvRmDeviceHandle rm, NvRmModuleID aperture, NvU32 instance,
    NvU32 num, NvU32 offset, const NvU32 *values )
{
    void *addr;
    NvRmModuleTable *tbl;
    NvRmModuleInstance *inst;

    tbl = NvRmPrivGetModuleTable( rm );

    NV_ASSERT( tbl->Modules[aperture].Index != NVRM_MODULE_INVALID );

    inst = tbl->ModInst + tbl->Modules[aperture].Index + instance;

    addr = (void *)((NvUPtr)inst->VirtAddr + offset);
    NV_WRITE( addr, values, (num << 2) );
}

void NvRegrb( NvRmDeviceHandle rm, NvRmModuleID aperture, NvU32 instance,
    NvU32 num, NvU32 offset, NvU32 *values )
{
    void *addr;
    NvRmModuleTable *tbl;
    NvRmModuleInstance *inst;

    tbl = NvRmPrivGetModuleTable( rm );

    NV_ASSERT( tbl->Modules[aperture].Index != NVRM_MODULE_INVALID );

    inst = tbl->ModInst + tbl->Modules[aperture].Index + instance;

    addr = (void *)((NvUPtr)inst->VirtAddr + offset);
    NV_READ( values, addr, (num << 2 ) );
}

