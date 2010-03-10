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

#include "nvcommon.h"
#include "nvrm_pinmux.h"
#include "nvrm_drf.h"
#include "nvassert.h"
#include "nvrm_hwintf.h"
#include "ap15/ap15rm_private.h"
#include "nvrm_pinmux_utils.h"
#include "nvodm_query_pinmux.h"

/*  Each of the pin mux configurations defined in the pin mux spreadsheet are
 *  stored in chip-specific tables.  For each configuration, every pad group
 *  that must be programmed is stored as a single 32b entry, where the register
 *  offset (for both the tristate and pin mux control registers), field bit
 *  position (ditto), pin mux mask, and new pin mux state are programmed.
 *
 *  The tables are microcode for a simple state machine.  The state machine
 *  supports subroutine call/return (up to 4 levels of nesting), so that
 *  pin mux configurations which have substantial repetition can be
 *  represented compactly by separating common portion of the configurations
 *  into a subroutine.  Additionally, the state machine supports
 *  "unprogramming" of the pin mux registers, so that pad groups which are
 *  incorrectly programmed to mux from a controller may be safely disowned,
 *  ensuring that no conflicts exist where multiple pad groups are muxing
 *  the same set of signals.
 *
 *  Each module instance array has a reserved "reset" configuration at index
 *  zero.  This special configuration is used in order to disown all pad
 *  groups whose reset state refers to the module instance.  When a module
 *  instance configuration is to be applied, the reset configuration will
 *  first be applied, to ensure that no conflicts will arise between register
 *  reset values and the new configuration, followed by the application of
 *  the requested configuration.
 *
 *  Furthermore, for controllers which support dynamic pinmuxing (i.e.,
 *  the "Multiplexed" pin map option), the last table entry is reserved for
 *  a "global unset," which will ensure that all configurations are disowned.
 *  This Multiplexed configuration should be applied before transitioning
 *  from one configuration to a second one.
 *
 *  The table data has been packed into a single 32b entry to minimize code
 *  footprint using macros similar to the hardware register definitions, so
 *  that all of the shift and mask operations can be performed with the DRF
 *  macros.
 */

static void NvRmPrivApplyAllPinMuxes(
    NvRmDeviceHandle hDevice,
    NvBool First);

static void NvRmPrivApplyAllModuleTypePinMuxes(
    NvRmDeviceHandle hDevice,
    NvU32 Module,
    NvBool ApplyReset,
    NvBool ApplyActual);

typedef struct
{
    void (*pfnInitTrisateRefCount)(NvRmDeviceHandle hDevice);
    const NvU32*** (*pfnGetPinMuxConfigs)(NvRmDeviceHandle hDevice);
    void
    (*pfnSetPinMuxCtl)(
    NvRmDeviceHandle hDevice,
        const NvU32* Module,
        NvU32 Config);
    void
    (*pfnSetPadTristates)(
        NvRmDeviceHandle hDevice,
        const NvU32* Module,
        NvU32 Config,
        NvBool EnTristate);
    const NvU32*
    (*pfnFindConfigStart)(
        const NvU32* Instance,
        NvU32 Config,
        NvU32 EndMarker);
    NvBool
    (*pfnGetPinGroupForGpio)(
        NvRmDeviceHandle hDevice,
        NvU32 Port,
        NvU32 Pin,
        NvU32 *pMapping);
    void
    (*pfnSetGpioTristate)(
        NvRmDeviceHandle hDevice,
        NvU32 Port,
        NvU32 Pin,
        NvBool EnableTristate);
    NvError
    (*pfnInterfaceCaps)(
        NvOdmIoModule Module,
        NvU32 Instance,
        NvU32 PinMap,
        void *pCaps);
    void
    (*pfnEnableExtClock)(
        NvRmDeviceHandle hDevice,
        const NvU32* Instance,
        NvU32 Config,
        NvBool ClockState);
    NvU32
    (*pfnGetExtClockFreq)(
        NvRmDeviceHandle hDevice,
        const NvU32* Instance,
        NvU32 Config);
    NvError
    (*pfnGetStraps)(
        NvRmDeviceHandle hDevice,
        NvRmStrapGroup StrapGroup,
        NvU32* pStrapValue);
    void
    (*pfnSetDefaultTristate)(
        NvRmDeviceHandle hDevice);
} NvPinmuxPrivMethods;

static NvPinmuxPrivMethods* NvRmPrivGetPinmuxMethods(NvRmDeviceHandle hDevice)
{
    static NvPinmuxPrivMethods *p;
    static NvPinmuxPrivMethods s_Ap15Methods =
    {
        NvRmPrivAp15InitTrisateRefCount,
        NvRmAp15GetPinMuxConfigs,
        NvRmPrivAp15SetPinMuxCtl,
        NvRmPrivAp15SetPadTristates,
        NvRmPrivAp15FindConfigStart,
        NvRmAp15GetPinGroupForGpio,
        NvRmPrivAp15SetGpioTristate,
        NvRmPrivAp15GetModuleInterfaceCaps,
        NvRmPrivAp15EnableExternalClockSource,
        NvRmPrivAp15GetExternalClockSourceFreq,
        NvRmAp15GetStraps,
        NvRmAp15SetDefaultTristate
    };
    static NvPinmuxPrivMethods s_Ap16Methods =
    {
        NvRmPrivAp15InitTrisateRefCount,
        NvRmAp16GetPinMuxConfigs,
        NvRmPrivAp15SetPinMuxCtl,
        NvRmPrivAp15SetPadTristates,
        NvRmPrivAp15FindConfigStart,
        NvRmAp15GetPinGroupForGpio,
        NvRmPrivAp15SetGpioTristate,
        NvRmPrivAp16GetModuleInterfaceCaps,
        NvRmPrivAp15EnableExternalClockSource,
        NvRmPrivAp15GetExternalClockSourceFreq,
        NvRmAp15GetStraps,
        NvRmAp15SetDefaultTristate
    };
    static NvPinmuxPrivMethods s_Ap20Methods =
    {
        NvRmPrivAp15InitTrisateRefCount,
        NvRmAp20GetPinMuxConfigs,
        NvRmPrivAp15SetPinMuxCtl,
        NvRmPrivAp15SetPadTristates,
        NvRmPrivAp15FindConfigStart,
        NvRmAp20GetPinGroupForGpio,
        NvRmPrivAp15SetGpioTristate,
        NvRmPrivAp20GetModuleInterfaceCaps,
        NvRmPrivAp20EnableExternalClockSource,
        NvRmPrivAp20GetExternalClockSourceFreq,
        NvRmAp20GetStraps,
        NvRmAp20SetDefaultTristate
    };

    NV_ASSERT(hDevice);
    switch (hDevice->ChipId.Id) {
    case 0x15:
        p = &s_Ap15Methods;
        break;
    case 0x16:
        p = &s_Ap16Methods;
        break;
    case 0x20:
        p = &s_Ap20Methods;
        break;
    default:
        NV_ASSERT(!"Unsupported chip ID");
        return NULL;
    }
    return p;
}

static void NvRmPrivApplyAllPinMuxes(
    NvRmDeviceHandle hDevice,
    NvBool First)
{
    NvOdmIoModule Module;

    NV_ASSERT(hDevice->PinMuxTable);

    for (Module=NvOdmIoModule_Ata; Module<NvOdmIoModule_Num; Module++)
    {
        NvBool ApplyActual = NV_TRUE;
        /* During early boot, the only device that has its pin mux correctly
         * initialized is the I2C PMU controller, so that primitive peripherals
         * (EEPROMs, PMU, RTC) can be accessed during the boot process */
        if (First)
            ApplyActual = (Module==NvOdmIoModule_I2c_Pmu);

        NvRmPrivApplyAllModuleTypePinMuxes(hDevice, Module,
            First, ApplyActual);
    }
}

static void NvRmPrivApplyAllModuleTypePinMuxes(
    NvRmDeviceHandle hDevice,
    NvU32 Module,
    NvBool ApplyReset,
    NvBool ApplyActual)
{
    const NvU32 *OdmConfigs;
    NvU32 NumOdmConfigs;
    NvPinmuxPrivMethods *p = NvRmPrivGetPinmuxMethods(hDevice);
    const NvU32 **ModulePrograms = hDevice->PinMuxTable[(NvU32)Module];

    if (!ModulePrograms)
        return;

    if (ApplyActual)
        NvOdmQueryPinMux(Module, &OdmConfigs, &NumOdmConfigs);
    else
    {
        OdmConfigs = NULL;
        NumOdmConfigs = 0;
    }

    for (; *ModulePrograms ; ModulePrograms++)
    {
        /*  Apply the reset configuration to ensure that the module is in
         *  a sane state, then apply the ODM configuration, if one is specified
         */
        if (ApplyReset)
            (p->pfnSetPinMuxCtl)(hDevice, *ModulePrograms, 0);
        if (NumOdmConfigs && ApplyActual)
        {
            (p->pfnSetPinMuxCtl)(hDevice, *ModulePrograms, *OdmConfigs);
            NumOdmConfigs--;
            OdmConfigs++;
        }
    }
    /*  If the ODM pin mux table is created correctly, there should be
     *  the same number of ODM configs as module instances; however, we
     *  allow the ODM to specify fewer configs than instances with assumed
     *  zeros for undefined modules */
    while (NumOdmConfigs)
    {
        NV_ASSERT((*OdmConfigs==0) &&
                  "More ODM configs than module instances!\n");
        NumOdmConfigs--;
        OdmConfigs++;
    }
}

/**
 * RmInitPinMux will program the pin mux settings for all IO controllers to
 * the ODM-selected value (or a safe reset value, if no value is defined in
 * the ODM query.
 *
 * It will also read the current value of the tristate registers, to
 * initialize the reference count
 */
void NvRmInitPinMux(
    NvRmDeviceHandle hDevice,
    NvBool First)
{
    NvPinmuxPrivMethods *p = NvRmPrivGetPinmuxMethods(hDevice);
    if (First)
        (p->pfnSetDefaultTristate)(hDevice);

    if (!hDevice->PinMuxTable)
    {
        hDevice->PinMuxTable = (p->pfnGetPinMuxConfigs)(hDevice);
        (p->pfnInitTrisateRefCount)(hDevice);
    }

#if (!NVOS_IS_WINDOWS_CE || NV_OAL)
    NvRmPrivApplyAllPinMuxes(hDevice, First);
#endif
}


/* RmPinMuxConfigSelect sets a specific module to a specific configuration.
 * It is used for multiplexed controllers, and should only be called by the
 * ODM service function NvOdmPinMuxSet */
void NvRmPinMuxConfigSelect(
    NvRmDeviceHandle hDevice,
    NvOdmIoModule IoModule,
    NvU32 Instance,
    NvU32 Configuration)
{
    const NvU32 ***ModulePrograms = NULL;
    const NvU32 **InstancePrograms = NULL;
    NvU32 i = 0;
    NvPinmuxPrivMethods *p = NvRmPrivGetPinmuxMethods(hDevice);

    NV_ASSERT(hDevice);
    if (!hDevice)
        return;

    ModulePrograms = hDevice->PinMuxTable;
    NV_ASSERT(ModulePrograms && ((NvU32)IoModule < (NvU32)NvOdmIoModule_Num));

    InstancePrograms = (const NvU32**)ModulePrograms[(NvU32)IoModule];

    /*  Walk through the instance arrays for this module, breaking
     *  when either the requested instance or the end of the list is
     *  reached. */
    if (InstancePrograms)
    {
        while (i<Instance && *InstancePrograms)
        {
            i++;
            InstancePrograms++;
        }

        if (*InstancePrograms)
        {
            (p->pfnSetPinMuxCtl)(hDevice, *InstancePrograms, Configuration);
        }
    }
}

/* RmPinMuxConfigSetTristate will either enable or disable the tristate for a
 * specific IO module configuration.  It is called by the ODM service function
 * OdmPinMuxConfigSetTristate, and by the RM function SetModuleTristate.  RM
 * client drivers should only call RmSetModuleTristate, which will program the
 * tristate correctly based on the ODM query configuration. */
void NvRmPinMuxConfigSetTristate(
    NvRmDeviceHandle hDevice,
    NvOdmIoModule IoModule,
    NvU32 Instance,
    NvU32 Configuration,
    NvBool EnableTristate)
{
    const NvU32 ***ModulePrograms  = NULL;
    const NvU32 **InstancePrograms = NULL;
    NvU32 i = 0;
    NvPinmuxPrivMethods *p = NvRmPrivGetPinmuxMethods(hDevice);

    NV_ASSERT(hDevice);
    if (!hDevice)
        return;

    ModulePrograms = hDevice->PinMuxTable;

    NV_ASSERT(ModulePrograms && ((NvU32)IoModule < (NvU32)NvOdmIoModule_Num));

    InstancePrograms = (const NvU32**)ModulePrograms[(NvU32)IoModule];

    if (InstancePrograms)
    {
        while (i<Instance && *InstancePrograms)
        {
            i++;
            InstancePrograms++;
        }

        if (*InstancePrograms)
        {
            (p->pfnSetPadTristates)(hDevice, *InstancePrograms,
                Configuration, EnableTristate);
        }
    }
}

NvError NvRmSetOdmModuleTristate(
    NvRmDeviceHandle hDevice,
    NvU32 OdmModule,
    NvU32 OdmInstance,
    NvBool EnableTristate)
{
    const NvU32 *OdmConfigs;
    NvU32 NumOdmConfigs;

    NV_ASSERT(hDevice);
    if (!hDevice)
        return NvError_BadParameter;

    NvOdmQueryPinMux(OdmModule, &OdmConfigs, &NumOdmConfigs);

    if ((OdmInstance >= NumOdmConfigs) || !OdmConfigs[OdmInstance])
        return NvError_NotSupported;

    NvRmPinMuxConfigSetTristate(hDevice, OdmModule,
        OdmInstance, OdmConfigs[OdmInstance], EnableTristate);

    return NvSuccess;
}

NvU32 NvRmPrivRmModuleToOdmModule(
    NvU32 ChipId,
    NvU32 RmModule,
    NvOdmIoModule *pOdmModules,
    NvU32 *pOdmInstances)
{
    NvU32 Cnt = 0;
    NvBool Result = NV_FALSE;

    NV_ASSERT(pOdmModules && pOdmInstances);

    if (ChipId==0x15)
    {
        Result = NvRmPrivAp15RmModuleToOdmModule(RmModule,
            pOdmModules, pOdmInstances, &Cnt);
    }
    else if (ChipId==0x16)
    {
        Result = NvRmPrivAp16RmModuleToOdmModule(RmModule,
             pOdmModules, pOdmInstances, &Cnt);
    }
    else if (ChipId==0x20)
    {
        Result = NvRmPrivAp20RmModuleToOdmModule(RmModule,
             pOdmModules, pOdmInstances, &Cnt);
    }

    /*  A default mapping is provided for all standard I/O controllers,
     *  if the chip-specific implementation does not implement a mapping */
    if (!Result)
    {
        NvRmModuleID Module = NVRM_MODULE_ID_MODULE(RmModule);
        NvU32 Instance = NVRM_MODULE_ID_INSTANCE(RmModule);

        Cnt = 1;
        switch (Module) {
        case NvRmModuleID_Display:
            *pOdmModules = NvOdmIoModule_Display;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Ide:
            *pOdmModules = NvOdmIoModule_Ata;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Vi:
            *pOdmModules = NvOdmIoModule_VideoInput;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Usb2Otg:
            *pOdmModules = NvOdmIoModule_Usb;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Pwm:
            *pOdmModules = NvOdmIoModule_Pwm;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Twc:
            *pOdmModules = NvOdmIoModule_Twc;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Hsmmc:
            *pOdmModules = NvOdmIoModule_Hsmmc;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Sdio:
            *pOdmModules = NvOdmIoModule_Sdio;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Nand:
            *pOdmModules = NvOdmIoModule_Nand;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_I2c:
            *pOdmModules = NvOdmIoModule_I2c;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Spdif:
            *pOdmModules = NvOdmIoModule_Spdif;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Uart:
            *pOdmModules = NvOdmIoModule_Uart;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Csi:
            *pOdmModules = NvOdmIoModule_Csi;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Hdmi:
            *pOdmModules = NvOdmIoModule_Hdmi;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Mipi:
            *pOdmModules = NvOdmIoModule_Hsi;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Tvo:
            pOdmModules[0] = NvOdmIoModule_Tvo;
            pOdmModules[1] = NvOdmIoModule_Crt;
            pOdmInstances[0] = 0;
            pOdmInstances[1] = 0;
            Cnt = 2;
            break;
        case NvRmModuleID_Dsi:
            *pOdmModules = NvOdmIoModule_Dsi;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Dvc:
            *pOdmModules = NvOdmIoModule_I2c_Pmu;
            *pOdmInstances = Instance;
            break;
        case NvRmPrivModuleID_Mio_Exio:
            *pOdmModules = NvOdmIoModule_Mio;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Xio:
            *pOdmModules = NvOdmIoModule_Xio;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Spi:
            *pOdmModules = NvOdmIoModule_Sflash;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Slink:
            *pOdmModules = NvOdmIoModule_Spi;
            *pOdmInstances = Instance;
            break;
        case NvRmModuleID_Kbc:
            *pOdmModules = NvOdmIoModule_Kbd;
            *pOdmInstances = Instance;
            break;
        default:
            //  all the RM modules which have no ODM analogs (like 3d)
            Cnt = 0;
            break;
        }
    }

    return Cnt;
}

/* RmSetModuleTristate will enable / disable the pad tristates for the
 * selected pin mux configuration of an IO module.  */
NvError NvRmSetModuleTristate(
    NvRmDeviceHandle hDevice,
    NvRmModuleID RmModule,
    NvBool EnableTristate)
{
    const NvU32 *OdmConfigs;
    NvU32 NumOdmConfigs;
    NvU32 OdmModules[4];
    NvU32 OdmInstances[4];
    NvU32 NumOdmModules = 0;
    NvU32 i;

    NV_ASSERT(hDevice);
    if (!hDevice)
        return NvError_BadParameter;

    NumOdmModules =
        NvRmPrivRmModuleToOdmModule(hDevice->ChipId.Id,
            RmModule, (NvOdmIoModule*)OdmModules, OdmInstances);
    if (!NumOdmModules)
        return NvError_NotSupported;

    /* return NotSupported if the ODM has not defined a pin mux configuration
     * for this module. */
    for (i=0; i<NumOdmModules; i++)
    {
        NvOdmQueryPinMux(OdmModules[i], &OdmConfigs, &NumOdmConfigs);
        if ((!NumOdmConfigs) || (!OdmConfigs[OdmInstances[i]]))
            return NvError_NotSupported;
        if (OdmInstances[i] >= NumOdmConfigs)
        {
            NV_DEBUG_PRINTF(("Attempted to set TRISTATE for Module %u, Instance"
                " %u (ODM module %u instance %u) with undefined config\n",
                NVRM_MODULE_ID_MODULE(RmModule),
                NVRM_MODULE_ID_INSTANCE(RmModule),
                OdmModules[i], OdmInstances[i]));
            return NvError_NotSupported;
            //  NV_ASSERT(OdmInstances[i] < NumOdmConfigs);
        }
    }

    for (i=0; i<NumOdmModules; i++)
    {
        NvOdmQueryPinMux(OdmModules[i], &OdmConfigs, &NumOdmConfigs);
        NV_ASSERT(OdmInstances[i] < NumOdmConfigs);
        NvRmPinMuxConfigSetTristate(hDevice, OdmModules[i],
            OdmInstances[i], OdmConfigs[OdmInstances[i]], EnableTristate);
    }
    return NvSuccess;
}

void NvRmSetGpioTristate(
    NvRmDeviceHandle hDevice,
    NvU32 Port,
    NvU32 Pin,
    NvBool EnableTristate)
{
    NvPinmuxPrivMethods *p = NvRmPrivGetPinmuxMethods(hDevice);

    (p->pfnSetGpioTristate)(hDevice, Port, Pin, EnableTristate);
}

NvU32 NvRmExternalClockConfig(
    NvRmDeviceHandle hDevice,
    NvU32 IoModule,
    NvU32 Instance,
    NvU32 Config,
    NvBool EnableTristate)
{
    const NvU32 ***ModulePrograms  = NULL;
    const NvU32 **InstancePrograms = NULL;
    const NvU32 *CdevInstance;
    NvU32 i = 0;
    NvU32 ret = 0;
    NvPinmuxPrivMethods *p = NvRmPrivGetPinmuxMethods(hDevice);

    NV_ASSERT(hDevice);

    if (!hDevice)
        return NvError_BadParameter;

    ModulePrograms = hDevice->PinMuxTable;

    NV_ASSERT(IoModule==NvOdmIoModule_ExternalClock);

    NV_ASSERT(ModulePrograms && ((NvU32)IoModule < (NvU32)NvOdmIoModule_Num));

    InstancePrograms = (const NvU32**)ModulePrograms[(NvU32)IoModule];

    if (InstancePrograms)
    {
        while (i<Instance && *InstancePrograms)
        {
            i++;
            InstancePrograms++;
        }

        if (*InstancePrograms)
        {
            if (!EnableTristate)
                (p->pfnSetPinMuxCtl)(hDevice, *InstancePrograms, Config);

            (p->pfnSetPadTristates)(hDevice, *InstancePrograms,
                Config, EnableTristate);
            CdevInstance = (p->pfnFindConfigStart)(*InstancePrograms,
                               Config, MODULEDONE());
            (p->pfnEnableExtClock)(hDevice, CdevInstance, Config, !EnableTristate);
            ret = (p->pfnGetExtClockFreq)(hDevice, CdevInstance, Config);
        }
    }
    return ret;
}

NvError NvRmGetModuleInterfaceCapabilities(
    NvRmDeviceHandle hRm,
    NvRmModuleID ModuleId,
    NvU32 CapStructSize,
    void *pCaps)
{
    NvU32 NumOdmConfigs;
    const NvU32 *OdmConfigs;
    NvOdmIoModule OdmModules[4];
    NvU32 OdmInstances[4];
    NvU32 NumOdmModules;
    NvPinmuxPrivMethods *p = NvRmPrivGetPinmuxMethods(hRm);

    NV_ASSERT(hRm);
    NV_ASSERT(pCaps);

    if (!hRm || !pCaps)
        return NvError_BadParameter;

    NumOdmModules =
        NvRmPrivRmModuleToOdmModule(hRm->ChipId.Id, ModuleId,
            (NvOdmIoModule *)OdmModules, OdmInstances);
    NV_ASSERT(NumOdmModules<=1);

    if (!NumOdmModules)
        return NvError_NotSupported;

    switch (OdmModules[0]) {
    case NvOdmIoModule_Hsmmc:
    case NvOdmIoModule_Sdio:
        if (CapStructSize != sizeof(NvRmModuleSdmmcInterfaceCaps))
        {
            NV_ASSERT(!"Invalid cap struct size");
            return NvError_BadParameter;
        }
        break;
    case NvOdmIoModule_Pwm:
        if (CapStructSize != sizeof(NvRmModulePwmInterfaceCaps))
        {
            NV_ASSERT(!"Invalid cap struct size");
            return NvError_BadParameter;
        }
        break;
    case NvOdmIoModule_Nand:
        if (CapStructSize != sizeof(NvRmModuleNandInterfaceCaps))
        {
            NV_ASSERT(!"Invalid cap struct size");
            return NvError_BadParameter;
        }
        break;

    case NvOdmIoModule_Uart:
        if (CapStructSize != sizeof(NvRmModuleUartInterfaceCaps))
        {
            NV_ASSERT(!"Invalid cap struct size");
            return NvError_BadParameter;
        }
        break;

    default:
        return NvError_NotSupported;
    }

    NvOdmQueryPinMux(OdmModules[0], &OdmConfigs, &NumOdmConfigs);
    if (OdmInstances[0]>=NumOdmConfigs || !OdmConfigs[OdmInstances[0]])
        return NvError_NotSupported;

    return (p->pfnInterfaceCaps)(OdmModules[0],OdmInstances[0],
                            OdmConfigs[OdmInstances[0]],pCaps);
}

NvError NvRmGetStraps(
    NvRmDeviceHandle hDevice,
    NvRmStrapGroup StrapGroup,
    NvU32* pStrapValue)
{
    NvPinmuxPrivMethods *p = NvRmPrivGetPinmuxMethods(hDevice);
    NV_ASSERT(hDevice && pStrapValue);

    if (!hDevice || !pStrapValue)
        return NvError_BadParameter;
    return (p->pfnGetStraps)(hDevice, StrapGroup, pStrapValue);
}

