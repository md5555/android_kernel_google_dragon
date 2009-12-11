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

#define NV_ENABLE_DEBUG_PRINTS 0
#define SKIP_TRISTATE_REFCNT 0

#include "nvcommon.h"
#include "nvrm_pinmux.h"
#include "nvrm_drf.h"
#include "nvassert.h"
#include "nvrm_hwintf.h"
#include "ap15/ap15rm_private.h"
#include "ap15/arapb_misc.h"
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

/*  FindConfigStart searches through an array of configuration data to find the
 *  starting position of a particular configuration in a module instance array.
 *  The stop position is programmable, so that sub-routines can be placed after
 *  the last valid true configuration */

static const NvU32* NvRmPrivFindConfigStart(
    const NvU32* Instance,
    NvU32 Config,
    NvU32 EndMarker)
{
    NvU32 Cnt = 0;
    while ((Cnt < Config) && (*Instance!=EndMarker))
    {
        switch (NV_DRF_VAL(MUX, ENTRY, STATE, *Instance)) {
        case PinMuxConfig_BranchLink:
        case PinMuxConfig_OpcodeExtend:
            if (*Instance==CONFIGEND())
                Cnt++;
            Instance++;
            break;
        default:
            Instance += NVRM_PINMUX_SET_OPCODE_SIZE;
            break;
        }
    }

    /* Ugly postfix.  In modules with bonafide subroutines, the last
     * configuration CONFIGEND() will be followed by a MODULEDONE()
     * token, with the first Set/Unset/Branch of the subroutine 
     * following that.  To avoid leaving the "PC" pointing to a
     * MODULEDONE() in the case where the first subroutine should be
     * executed, fudge the "PC" up by one, to point to the subroutine. */
    if (EndMarker==SUBROUTINESDONE() && *Instance==MODULEDONE())
        Instance++;

    if (*Instance==EndMarker)
        Instance = NULL;

    return Instance;
}

/*  NvRmSetPadTristates will increment / decrement the reference count for
 *  each pad group's global tristate value for each "ConfigSet" command in
 *  a pad group configuration, and update the register as needed */
static void NvRmPrivSetPadTristates(
    NvRmDeviceHandle hDevice,
    const NvU32* Module,
    NvU32 Config,
    NvBool EnableTristate)
{
    int StackDepth = 0;
    const NvU32 *Instance = NULL;
    const NvU32 *ReturnStack[MAX_NESTING_DEPTH+1];

    /* The re-multiplexing configuration is stored in program 0,
     * along with the reset config. */
    if (Config==NVODM_QUERY_PINMAP_MULTIPLEXED)
        Config = 0;

    Instance = NvRmPrivFindConfigStart(Module, Config, MODULEDONE());
    /* The first stack return entry is NULL, so that when a ConfigEnd is
     * encountered in the "main" configuration program, we pop off a NULL
     * pointer, which causes the configuration loop to terminate. */
    ReturnStack[0] = NULL;

    /*  This loop iterates over all of the pad groups that need to be updated,
     *  and updates the reference count for each appropriately.  */

    NvOsMutexLock(hDevice->mutex);

    while (Instance)
    {
        switch (NV_DRF_VAL(MUX,ENTRY, STATE, *Instance)) {
        case PinMuxConfig_OpcodeExtend:
            /* Pop the most recent return address off of the return stack 
             * (which will be NULL if no values have been pushed onto the 
             * stack) */
            if (NV_DRF_VAL(MUX,ENTRY, OPCODE_EXTENSION, 
                           *Instance)==PinMuxOpcode_ConfigEnd)
            {
                Instance = ReturnStack[StackDepth--];
            }
            /* ModuleDone & SubroutinesDone should never be encountered 
             * during execution, for properly-formatted tables. */
            else
            {
                NV_ASSERT(0 && "Logical entry in table!\n");
            }
            break;
        case PinMuxConfig_BranchLink:
            /*  Push the next instruction onto the return stack if nesting space
                is available, and jump to the target. */
            NV_ASSERT(StackDepth<MAX_NESTING_DEPTH);
            ReturnStack[++StackDepth] = Instance+1;
            Instance = NvRmPrivFindConfigStart(Module,
                           NV_DRF_VAL(MUX,ENTRY,BRANCH_ADDRESS,*Instance),
                           SUBROUTINESDONE());
            NV_ASSERT(Instance && "Invalid branch configuration in table!\n");
            break;
        case PinMuxConfig_Set:
        {
            NvS16 SkipUpdate;
            NvU32 TsOffs = NV_DRF_VAL(MUX,ENTRY, TS_OFFSET, *Instance);
            NvU32 TsShift = NV_DRF_VAL(MUX,ENTRY, TS_SHIFT, *Instance);

/*  abuse pre/post-increment, to ensure that skipUpdate is 0 when the
 *  register needs to be programmed (i.e., enabling and previous value was 0,
 *  or disabling and new value is 0).
 */
            if (EnableTristate)
#if (SKIP_TRISTATE_REFCNT == 0)
                SkipUpdate =  --hDevice->TristateRefCount[TsOffs*32 + TsShift];
            else
                SkipUpdate = hDevice->TristateRefCount[TsOffs*32 + TsShift]++;
#else
                SkipUpdate = 1;
            else
                SkipUpdate = 0;
#endif

#if (SKIP_TRISTATE_REFCNT == 0)
            if (SkipUpdate < 0)
            {
                hDevice->TristateRefCount[TsOffs*32 + TsShift] = 0;
                NV_DEBUG_PRINTF(("(%s:%s) Negative reference count detected "
                                 "on TRISTATE_REG_%c_0, bit %u\n", 
                    __FILE__, __LINE__, ('A'+(TsOffs)), TsShift));
                //NV_ASSERT(SkipUpdate>=0);
            }
#endif

            if (!SkipUpdate)
            {
                NvU32 Curr = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                      APB_MISC_PP_TRISTATE_REG_A_0 + 4*TsOffs);
                Curr &= ~(1<<TsShift);
#if (SKIP_TRISTATE_REFCNT == 0)
                Curr |= (EnableTristate?1:0)<<TsShift;
#endif
                NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                    APB_MISC_PP_TRISTATE_REG_A_0 + 4*TsOffs, Curr);

#if NVRM_PINMUX_DEBUG_FLAG
                NV_DEBUG_PRINTF(("Setting TRISTATE_REG_%s to %s\n", 
                    (const char*)Instance[2], 
                    (EnableTristate)?"TRISTATE" : "NORMAL"));
#endif
            }
        }
        /* fall through.
         * The "Unset" configurations are not applicable to tristate
         * configuration, so skip over them. */
        case PinMuxConfig_Unset:
            Instance += NVRM_PINMUX_SET_OPCODE_SIZE;
            break;       
        }
    }
    NvOsMutexUnlock(hDevice->mutex);
}

/*  NvRmSetPinMuxCtl will apply new pin mux configurations to the pin mux
 *  control registers.  */
static void NvRmPrivSetPinMuxCtl(
    NvRmDeviceHandle hDevice,
    const NvU32* Module,
    NvU32 Config)
{
    NvU32 MuxCtlOffset, MuxCtlShift, MuxCtlMask, MuxCtlSet, MuxCtlUnset;
    const NvU32 *ReturnStack[MAX_NESTING_DEPTH+1];
    const NvU32 *Instance;
    int StackDepth = 0;
    NvU32 Curr;

    ReturnStack[0] = NULL;
    Instance = Module;

    NvOsMutexLock(hDevice->mutex);

    /* The re-multiplexing configuration is stored in program 0,
     * along with the reset config. */
    if (Config==NVODM_QUERY_PINMAP_MULTIPLEXED)
        Config = 0;

    Instance = NvRmPrivFindConfigStart(Module, Config, MODULEDONE());

    //  Apply the new configuration, setting / unsetting as appropriate
    while (Instance)
    {
        switch (NV_DRF_VAL(MUX,ENTRY, STATE, *Instance)) {
        case PinMuxConfig_OpcodeExtend:
            if (NV_DRF_VAL(MUX,ENTRY, OPCODE_EXTENSION, 
                           *Instance)==PinMuxOpcode_ConfigEnd)
            {
                Instance = ReturnStack[StackDepth--];
            }
            else
            {
                NV_ASSERT(0 && "Logical entry in table!\n");
            }
            break;
        case PinMuxConfig_BranchLink:
            NV_ASSERT(StackDepth<MAX_NESTING_DEPTH);
            ReturnStack[++StackDepth] = Instance+1;
            Instance = NvRmPrivFindConfigStart(Module,
                           NV_DRF_VAL(MUX,ENTRY,BRANCH_ADDRESS,*Instance),
                           SUBROUTINESDONE());
            NV_ASSERT(Instance && "Invalid branch configuration in table!\n");
            break;
        default:
        {
            MuxCtlOffset = NV_DRF_VAL(MUX,ENTRY, MUX_CTL_OFFSET, *Instance);
            MuxCtlShift = NV_DRF_VAL(MUX,ENTRY, MUX_CTL_SHIFT, *Instance);
            MuxCtlUnset = NV_DRF_VAL(MUX,ENTRY, MUX_CTL_UNSET, *Instance);
            MuxCtlSet = NV_DRF_VAL(MUX,ENTRY, MUX_CTL_SET, *Instance);
            MuxCtlMask = NV_DRF_VAL(MUX, ENTRY, MUX_CTL_MASK, *Instance);
            
            Curr = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                APB_MISC_PP_PIN_MUX_CTL_A_0 + 4*MuxCtlOffset);
            
            if (NV_DRF_VAL(MUX,ENTRY,STATE,*Instance)==PinMuxConfig_Set)
            {
                Curr &= ~(MuxCtlMask<<MuxCtlShift);
                Curr |= (MuxCtlSet<<MuxCtlShift);
#if NVRM_PINMUX_DEBUG_FLAG
                NV_DEBUG_PRINTF(("Configuring PINMUX_CTL_%s\n", 
                                 (const char *)Instance[1]));
#endif

            }
            else if (((Curr>>MuxCtlShift)&MuxCtlMask)==MuxCtlUnset)
            {
                NV_ASSERT(NV_DRF_VAL(MUX,ENTRY,STATE,
                                     *Instance)==PinMuxConfig_Unset);
                Curr &= ~(MuxCtlMask<<MuxCtlShift);
                Curr |= (MuxCtlSet<<MuxCtlShift);
#if NVRM_PINMUX_DEBUG_FLAG
                NV_DEBUG_PRINTF(("Unconfiguring PINMUX_CTL_%s\n",
                                 (const char *)Instance[1]));
#endif
            }
            
            NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                APB_MISC_PP_PIN_MUX_CTL_A_0 + 4*MuxCtlOffset, Curr);
            Instance += NVRM_PINMUX_SET_OPCODE_SIZE;
            break;
        }
        }
    }
    NvOsMutexUnlock(hDevice->mutex);
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
            NvRmPrivSetPinMuxCtl(hDevice, *ModulePrograms, 0);
        if (NumOdmConfigs && ApplyActual)
        {
            NvRmPrivSetPinMuxCtl(hDevice, *ModulePrograms, *OdmConfigs);
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
    NvU32 i, j, curr;

    if (!hDevice->PinMuxTable)
    {
        switch (hDevice->ChipId.Id) {
        case 0x15:
            hDevice->PinMuxTable = NvRmAp15GetPinMuxConfigs(hDevice); break;
        case 0x16:
            hDevice->PinMuxTable = NvRmAp16GetPinMuxConfigs(hDevice); break;
        case 0x20:
            hDevice->PinMuxTable = NvRmAp20GetPinMuxConfigs(hDevice); break;
        default:
            NV_ASSERT(!"Unsupported chip ID");
            hDevice->PinMuxTable = NULL;
            return;
        }

        NvOsMutexLock(hDevice->mutex);
        NvOsMemset(hDevice->TristateRefCount, 0,
            sizeof(hDevice->TristateRefCount));

        for (i=0; i<=((APB_MISC_PP_TRISTATE_REG_D_0-
                       APB_MISC_PP_TRISTATE_REG_A_0)>>2); i++)
        {
            curr = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                           APB_MISC_PP_TRISTATE_REG_A_0 + 4*i);
            // swap from 0=normal, 1=tristate to 0=tristate, 1=normal
            curr = ~curr;
            for (j=0; curr; j++, curr>>=1)
            {
                /* the oppositely-named tristate reference count keeps track
                 * of the number of active users of each pad group, and
                 * enables tristate when the count reaches zero. */
                hDevice->TristateRefCount[i*32 + j] = (NvS16)(curr & 0x1);
            }
        }
        NvOsMutexUnlock(hDevice->mutex);
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
            NvRmPrivSetPinMuxCtl(hDevice, *InstancePrograms, Configuration);
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
            NvRmPrivSetPadTristates(hDevice, *InstancePrograms,
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
    NvU32 Mapping = 0;
    NvS16 SkipUpdate;
    NvBool ret = NV_FALSE;

    NV_ASSERT(hDevice);

    switch (hDevice->ChipId.Id) {
    case 0x15:
    case 0x16:
        ret = NvRmAp15GetPinGroupForGpio(hDevice, Port, Pin, &Mapping);
        break;
    case 0x20:
        ret = NvRmAp20GetPinGroupForGpio(hDevice, Port, Pin, &Mapping);
        break;
    default:
        NV_ASSERT(!"Chip ID not supported");
        return;
    }

    if (ret)
    {
        NvU32 TsOffs  = NV_DRF_VAL(MUX, GPIOMAP, TS_OFFSET, Mapping);
        NvU32 TsShift = NV_DRF_VAL(MUX, GPIOMAP, TS_SHIFT, Mapping);

        NvOsMutexLock(hDevice->mutex);
 
        if (EnableTristate)
#if (SKIP_TRISTATE_REFCNT == 0)
            SkipUpdate = --hDevice->TristateRefCount[TsOffs*32 + TsShift];
        else
            SkipUpdate = hDevice->TristateRefCount[TsOffs*32 + TsShift]++;
#else
            SkipUpdate = 1;
        else
            SkipUpdate = 0;
#endif

#if (SKIP_TRISTATE_REFCNT == 0)
        if (SkipUpdate < 0)
        {
            hDevice->TristateRefCount[TsOffs*32 + TsShift] = 0;
            NV_DEBUG_PRINTF(("(%s:%s) Negative reference count detected on "
                "TRISTATE_REG_%c_0, bit %u\n", __FILE__, __LINE__,
                ('A'+(TsOffs)), TsShift));
            //NV_ASSERT(SkipUpdate>=0);
        }
#endif

        if (!SkipUpdate)
        {
            NvU32 Curr = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                  APB_MISC_PP_TRISTATE_REG_A_0 + 4*TsOffs);
            Curr &= ~(1<<TsShift);
#if (SKIP_TRISTATE_REFCNT == 0)
            Curr |= (EnableTristate?1:0)<<TsShift;
#endif
            NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                APB_MISC_PP_TRISTATE_REG_A_0 + 4*TsOffs, Curr);
        }

        NvOsMutexUnlock(hDevice->mutex);
    }
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

    void (*pfnEnableExtClock)(NvRmDeviceHandle, const NvU32 *, NvU32, NvBool);
    NvU32 (*pfnGetExtClockFreq)(NvRmDeviceHandle, const NvU32 *, NvU32);

    NV_ASSERT(hDevice);

    if (!hDevice)
        return NvError_BadParameter;

    switch (hDevice->ChipId.Id) {
    case 0x15:
    case 0x16:
        pfnEnableExtClock = NvRmPrivAp15EnableExternalClockSource;
        pfnGetExtClockFreq = NvRmPrivAp15GetExternalClockSourceFreq;
        break;
    case 0x20:
        pfnEnableExtClock = NvRmPrivAp20EnableExternalClockSource;
        pfnGetExtClockFreq = NvRmPrivAp20GetExternalClockSourceFreq;
        break;
    default:
        NV_ASSERT(!"Unsupported Chip ID");
        return 0;
    }

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
                NvRmPrivSetPinMuxCtl(hDevice, *InstancePrograms, Config);

            NvRmPrivSetPadTristates(hDevice, *InstancePrograms,
                Config, EnableTristate);
            CdevInstance = NvRmPrivFindConfigStart(*InstancePrograms,
                               Config, MODULEDONE());
            pfnEnableExtClock(hDevice, CdevInstance, Config, !EnableTristate);
            ret = pfnGetExtClockFreq(hDevice, CdevInstance, Config);
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
    NvError (*pfnInterfaceCaps)(NvOdmIoModule,NvU32,NvU32,void*);
    
    NV_ASSERT(hRm);
    NV_ASSERT(pCaps);

    if (!hRm || !pCaps)
        return NvError_BadParameter;

    switch (hRm->ChipId.Id) {
    case 0x15:
        pfnInterfaceCaps = NvRmPrivAp15GetModuleInterfaceCaps;
        break;
    case 0x16:
        pfnInterfaceCaps = NvRmPrivAp16GetModuleInterfaceCaps;
        break;
    case 0x20:
        pfnInterfaceCaps = NvRmPrivAp20GetModuleInterfaceCaps;
        break;
    default:
        NV_ASSERT(!"Unsupported chip ID!");
        return NvError_NotSupported;
    }

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

    return pfnInterfaceCaps(OdmModules[0],OdmInstances[0],
                            OdmConfigs[OdmInstances[0]],pCaps);
}

NvError NvRmGetStraps(
    NvRmDeviceHandle hDevice,
    NvRmStrapGroup StrapGroup,
    NvU32* pStrapValue)
{
    NV_ASSERT(hDevice && pStrapValue);

    if (!hDevice || !pStrapValue)
        return NvError_BadParameter;

    switch (hDevice->ChipId.Id) {
        case 0x15:
        case 0x16:
            return NvRmAp15GetStraps(hDevice, StrapGroup, pStrapValue);
        case 0x20:
            return NvRmAp20GetStraps(hDevice, StrapGroup, pStrapValue);
        default:
            NV_ASSERT(!"Unsupported Chip ID");
            return 0;
    }
}
