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

#if NV_IS_AVP
#define NV_DEF_RMC_TRACE                0   // NO TRACING FOR AVP
#endif

#include "nvcommon.h"
#include "nvassert.h"
#include "nvos.h"
#include "nvrm_hwintf.h"
#include "nvrm_module.h"
#include "nvrm_module_private.h"
#include "nvrm_moduleids.h"
#include "ap15/ap15rm_private.h"

#define NVRM_ENABLE_PRINTF          0  // Module debug: 0=disable, 1=enable

#if (NV_DEBUG && NVRM_ENABLE_PRINTF)
#define NVRM_MODULE_PRINTF(x)   NvOsDebugPrintf x
#else
#define NVRM_MODULE_PRINTF(x)
#endif

NvError
NvRmPrivModuleInit( NvRmModuleTable *mod_table, NvU32 *reloc_table )
{
    NvError err;
    NvU32 i;

    /* invalidate the module table */
    for( i = 0; i < NvRmPrivModuleID_Num; i++ )
    {
        mod_table->Modules[i].Index = NVRM_MODULE_INVALID;
    }

    /* clear the irq map */
    NvOsMemset( &mod_table->IrqMap, 0, sizeof(mod_table->IrqMap) );

    err = NvRmPrivRelocationTableParse( reloc_table,
        &mod_table->ModInst, &mod_table->LastModInst,
        mod_table->Modules, &mod_table->IrqMap );
    if( err != NvSuccess )
    {
        NV_ASSERT( !"NvRmPrivModuleInit failed" );
        return err;
    }

    NV_ASSERT( mod_table->LastModInst);
    NV_ASSERT( mod_table->ModInst );

    mod_table->NumModuleInstances = mod_table->LastModInst -
        mod_table->ModInst;

    return NvSuccess;
}

void
NvRmPrivModuleDeinit( NvRmModuleTable *mod_table )
{
}

NvError
NvRmPrivGetModuleInstance( NvRmDeviceHandle hDevice, NvRmModuleID ModuleId,
    NvRmModuleInstance **out )
{
    NvRmModuleTable *tbl;
    NvRmModule *module;             // Pointer to module table
    NvRmModuleInstance *inst;       // Pointer to device instance
    NvU32 DeviceId;                 // Hardware device id
    NvU32 Module;
    NvU32 Instance;

    *out = NULL;

    NV_ASSERT( hDevice );

    tbl = NvRmPrivGetModuleTable( hDevice );

    Module   = NVRM_MODULE_ID_MODULE( ModuleId );
    Instance = NVRM_MODULE_ID_INSTANCE( ModuleId );
    NV_ASSERT( (NvU32)Module < (NvU32)NvRmPrivModuleID_Num );

    // Get a pointer to the first instance of this module id type.
    module = tbl->Modules;

    // Check whether the index is valid or not.
    if (module[Module].Index == NVRM_MODULE_INVALID)
    {
        return NvError_NotSupported;
    }

    inst = tbl->ModInst + module[Module].Index;

    // Get its device id.
    DeviceId = inst->DeviceId;

    // Now point to the desired instance.
    inst += Instance;

    // Is this a valid instance and is it of the same hardware type?
    if ((inst >= tbl->LastModInst) || (DeviceId != inst->DeviceId))
    {
        // Invalid instance.
        return NvError_BadValue;
    }

    *out = inst;

    // Check if instance is still valid and not bonded out.
    // Still returning inst structure.
    if ( (NvU8)-1 == inst->DevIdx )
        return NvError_NotSupported;

    return NvSuccess;
}

void
NvRmModuleGetBaseAddress( NvRmDeviceHandle hDevice,
    NvRmModuleID ModuleId, NvRmPhysAddr* pBaseAddress,
    NvU32* pSize )
{
    NvRmModuleInstance *inst;

    NV_ASSERT_SUCCESS(
        NvRmPrivGetModuleInstance(hDevice, ModuleId, &inst)
    );

    if (pBaseAddress)
        *pBaseAddress = inst->PhysAddr;
    if (pSize)
        *pSize = inst->Length;
}

NvU32
NvRmModuleGetNumInstances(
    NvRmDeviceHandle hDevice,
    NvRmModuleID Module)
{
    NvU32 Instances = 0;
    NvU32 numInstances = 0;
    for (;;)
    {
        NvRmModuleInstance *inst;
        NvError e = NvRmPrivGetModuleInstance(
            hDevice, NVRM_MODULE_ID(Module, Instances), &inst);
        if (e != NvSuccess)
        {
            if ( !(inst && ((NvU8)-1 == inst->DevIdx)) )
                break;
             /* else if a module instance not avail (bonded out), continue
                looking for next instance. */
        }
        else
            numInstances++;
        Instances++;
    }
    return numInstances;
}

NvError
NvRmModuleGetModuleInfo(
    NvRmDeviceHandle    hDevice,
    NvRmModuleID        module,
    NvU32 *             pNum,
    NvRmModuleInfo      *pModuleInfo
    )
{
    NvU32   instance = 0;
    NvU32   numInstances = 0;

    if ( NULL == pNum )
        return NvError_BadParameter;

    // if !pModuleInfo, returns total numInstances
    while ( (NULL == pModuleInfo) || (numInstances < *pNum) )
    {
        NvRmModuleInstance *inst;
        NvError e = NvRmPrivGetModuleInstance(
            hDevice, NVRM_MODULE_ID(module, instance), &inst);
        if (e != NvSuccess)
        {
            if ( !(inst && ((NvU8)-1 == inst->DevIdx)) )
                break;
             /* else if a module instance not avail (bonded out), continue
                looking for next instance. */
        }
        else
        {
            if ( pModuleInfo )
            {
                pModuleInfo->Instance = instance;
                pModuleInfo->BaseAddress = inst->PhysAddr;
                pModuleInfo->Length = inst->Length;
                pModuleInfo++;
            }
            numInstances++;
        }
        instance++;
    }
    *pNum = numInstances;   // update with correct number of instances

    return NvSuccess;
}
