/*
 * Copyright (c) 2008-2009 NVIDIA Corporation.
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

#include "nvrm_init.h"
#include "nvassert.h"
#include "nvos.h"
#include "nvrm_module.h"
#include "ap20/armc.h"
#include "ap20/arahb_arbc.h"
#include "nvrm_drf.h"
#include "nvrm_hwintf.h"
#include "nvrm_structure.h"

NvError NvRmPrivAp20McErrorMonitorStart(NvRmDeviceHandle hRm);
void NvRmPrivAp20McErrorMonitorStop(NvRmDeviceHandle hRm);
void NvRmPrivAp20SetupMc(NvRmDeviceHandle hRm);
static void McErrorIntHandler(void* args);
static NvOsInterruptHandle s_McInterruptHandle = NULL;

void McErrorIntHandler(void* args)
{
    NvU32 RegVal;
    NvU32 IntStatus;
    NvU32 IntClear = 0;
    NvRmDeviceHandle hRm = (NvRmDeviceHandle)args;
    
    IntStatus = NV_REGR(hRm, NvRmPrivModuleID_MemoryController, 0, MC_INTSTATUS_0);
    if ( NV_DRF_VAL(MC, INTSTATUS, SECURITY_VIOLATION_INT, IntStatus) )
    {
        IntClear |= NV_DRF_DEF(MC, INTSTATUS, SECURITY_VIOLATION_INT, SET);
        RegVal = NV_REGR(hRm, NvRmPrivModuleID_MemoryController, 0, 
                     MC_SECURITY_VIOLATION_ADR_0);
        NvOsDebugPrintf("SECURITY_VIOLATION DecErrAddress=0x%x ", RegVal);
        RegVal = NV_REGR(hRm, NvRmPrivModuleID_MemoryController, 0, 
                     MC_SECURITY_VIOLATION_STATUS_0);
        NvOsDebugPrintf("SECURITY_VIOLATION DecErrStatus=0x%x ", RegVal);
    }
    if ( NV_DRF_VAL(MC, INTSTATUS, DECERR_EMEM_OTHERS_INT, IntStatus) )
    {
        IntClear |= NV_DRF_DEF(MC, INTSTATUS, DECERR_EMEM_OTHERS_INT, SET);
        RegVal = NV_REGR(hRm, NvRmPrivModuleID_MemoryController, 0, 
                     MC_DECERR_EMEM_OTHERS_ADR_0);
        NvOsDebugPrintf("EMEM DecErrAddress=0x%x ", RegVal);
        RegVal = NV_REGR(hRm, NvRmPrivModuleID_MemoryController, 0, 
                     MC_DECERR_EMEM_OTHERS_STATUS_0);
        NvOsDebugPrintf("EMEM DecErrStatus=0x%x ", RegVal);
    }
    if ( NV_DRF_VAL(MC, INTSTATUS, INVALID_GART_PAGE_INT, IntStatus) )
    {
        IntClear |= NV_DRF_DEF(MC, INTSTATUS, INVALID_GART_PAGE_INT, SET);
        RegVal = NV_REGR(hRm, NvRmPrivModuleID_MemoryController, 0, 
                     MC_GART_ERROR_ADDR_0);
        NvOsDebugPrintf("GART DecErrAddress=0x%x ", RegVal);
        RegVal = NV_REGR(hRm, NvRmPrivModuleID_MemoryController, 0, 
                     MC_GART_ERROR_REQ_0);
        NvOsDebugPrintf("GART DecErrStatus=0x%x ", RegVal);
    }
    
    NV_ASSERT(!"MC Decode Error ");
    // Clear the interrupt.
    NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_INTSTATUS_0, IntClear);
    NvRmInterruptDone(s_McInterruptHandle);
}

NvError NvRmPrivAp20McErrorMonitorStart(NvRmDeviceHandle hRm)
{
    NvU32 val;
    NvU32 IrqList;
    NvError e = NvSuccess;
    NvOsInterruptHandler handler;
    
    if (s_McInterruptHandle == NULL)
    {
        // Install an interrupt handler.
        handler = McErrorIntHandler;
        IrqList = NvRmGetIrqForLogicalInterrupt(hRm,
                      NvRmPrivModuleID_MemoryController, 0);
        NV_CHECK_ERROR( NvRmInterruptRegister(hRm, 1, &IrqList,  &handler, 
            hRm, &s_McInterruptHandle, NV_TRUE) );
        // Enable Dec Err interrupts in memory Controller.
        val = NV_DRF_DEF(MC, INTMASK, SECURITY_VIOLATION_INTMASK, UNMASKED) |
              NV_DRF_DEF(MC, INTMASK, DECERR_EMEM_OTHERS_INTMASK, UNMASKED) |
              NV_DRF_DEF(MC, INTMASK, INVALID_GART_PAGE_INTMASK, UNMASKED);
        NV_REGW(hRm, NvRmPrivModuleID_MemoryController, 0, MC_INTMASK_0, val);
    }
    return e;
}

void NvRmPrivAp20McErrorMonitorStop(NvRmDeviceHandle hRm)
{
    NvRmInterruptUnregister(hRm, s_McInterruptHandle);
    s_McInterruptHandle = NULL;
}

/* This function sets some performance timings for Mc & Emc.  Numbers are from
 * the Arch team.
 *
 */
void NvRmPrivAp20SetupMc(NvRmDeviceHandle hRm)
{
    NvU32   reg, mask;
    reg = NV_REGR(hRm, NvRmPrivModuleID_MemoryController, 0, 
              MC_LOWLATENCY_CONFIG_0);
    mask = NV_DRF_DEF(MC, LOWLATENCY_CONFIG, MPCORER_LL_CTRL, ENABLE) |
           NV_DRF_DEF(MC, LOWLATENCY_CONFIG, MPCORER_LL_SEND_BOTH, ENABLE);
    if ( mask != (reg & mask) )
        NV_ASSERT(!"MC LL Path not enabled!");
    // For AP20, no need to program any MC timeout registers here. Default 
    // values should be good enough.

    // Setup the AHB MEM configuration for USB performance.
    // Enabling the AHB prefetch bits for USB1 USB2 and USB3.
    // 64kiloByte boundaries 
    // 4096 cycles before prefetched data is invalidated due to inactivity. 
    reg = NV_DRF_NUM(AHB_AHB_MEM, PREFETCH_CFG1, ENABLE, 1) |
          NV_DRF_DEF(AHB_AHB_MEM, PREFETCH_CFG1, AHB_MST_ID, AHBDMA)|
          NV_DRF_NUM(AHB_AHB_MEM, PREFETCH_CFG1, ADDR_BNDRY, 0xC) |
          NV_DRF_NUM(AHB_AHB_MEM, PREFETCH_CFG1, SPEC_THROTTLE, 0x0) |
          NV_DRF_NUM(AHB_AHB_MEM, PREFETCH_CFG1, INACTIVITY_TIMEOUT, 0x1000);
    NV_REGW( hRm, NvRmPrivModuleID_Ahb_Arb_Ctrl, 0,
             AHB_AHB_MEM_PREFETCH_CFG1_0, reg );

    reg = NV_DRF_NUM(AHB_AHB_MEM, PREFETCH_CFG2, ENABLE, 1) |
          NV_DRF_DEF(AHB_AHB_MEM, PREFETCH_CFG2, AHB_MST_ID, USB)|
          NV_DRF_DEF(AHB_AHB_MEM, PREFETCH_CFG2, AHB_MST_ID, USB2)|
          NV_DRF_DEF(AHB_AHB_MEM, PREFETCH_CFG2, AHB_MST_ID, USB3)|
          NV_DRF_NUM(AHB_AHB_MEM, PREFETCH_CFG2, ADDR_BNDRY, 0xC) |
          NV_DRF_NUM(AHB_AHB_MEM, PREFETCH_CFG2, SPEC_THROTTLE, 0x0) |
          NV_DRF_NUM(AHB_AHB_MEM, PREFETCH_CFG2, INACTIVITY_TIMEOUT, 0x1000);
    NV_REGW( hRm, NvRmPrivModuleID_Ahb_Arb_Ctrl, 0,
             AHB_AHB_MEM_PREFETCH_CFG2_0, reg );

}


