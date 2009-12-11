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

#include "ap20rm_power_dfs.h"
#include "nvassert.h"
#include "nvrm_drf.h"
#include "nvrm_hwintf.h"
#include "nvrm_pmu.h"
#include "ap20/aremc.h"
#include "ap20/arclk_rst.h"
#include "ap20/arapb_misc.h"

/*****************************************************************************/

// Regsiter access macros for EMC module
#define NV_EMC_REGR(pEmcRegs, reg) \
            NV_READ32((((NvU32)(pEmcRegs)) + EMC_##reg##_0))
#define NV_EMC_REGW(pEmcRegs, reg, val) \
            NV_WRITE32((((NvU32)(pEmcRegs)) + EMC_##reg##_0), (val))

// Regsiter access macros for APB MISC module
#define NV_APB_REGR(pApbRegs, reg) \
         NV_READ32((((NvU32)(pApbRegs)) + APB_MISC_##reg##_0))
#define NV_APB_REGW(pApbRegs, reg, val) \
         NV_WRITE32((((NvU32)(pApbRegs)) + APB_MISC_##reg##_0), (val))

// TODO: Always Disable before check-in
#define NVRM_TEST_PMREQUEST_UP_MODE (0)

/*****************************************************************************/
// EMC MODULE INTERFACES
/*****************************************************************************/

NvError NvRmPrivAp20EmcMonitorsInit(NvRmDfs* pDfs)
{
    NvU32 RegValue;
    void* pEmcRegs = pDfs->Modules[NvRmDfsModuleId_Emc].pBaseReg;
    NV_ASSERT(pEmcRegs);

    /*
     * EMC power management monitor belongs to EMC module - just reset it,
     * and do not touch anything else in EMC.
     */ 
    RegValue = NV_EMC_REGR(pEmcRegs, STAT_CONTROL);
    RegValue = NV_FLD_SET_DRF_DEF(EMC, STAT_CONTROL, PWR_GATHER, RST, RegValue);
    NV_EMC_REGW(pEmcRegs, STAT_CONTROL, RegValue);

    /*
    * EMC active clock cycles = EMC monitor reading * 2^M, where M depends
    * on DRAM type and bus width. Power M is stored as EMC readouts scale
    */
    #define COUNT_SHIFT_DDR1_X32 (1)
    RegValue = NV_EMC_REGR(pEmcRegs, FBIO_CFG5);
    switch (NV_DRF_VAL(EMC, FBIO_CFG5, DRAM_TYPE, RegValue))
    {
        case EMC_FBIO_CFG5_0_DRAM_TYPE_DDR1:
        case EMC_FBIO_CFG5_0_DRAM_TYPE_LPDDR2:
        case EMC_FBIO_CFG5_0_DRAM_TYPE_DDR2:
            pDfs->Modules[NvRmDfsModuleId_Emc].Scale = COUNT_SHIFT_DDR1_X32;
            break;
        default:
            NV_ASSERT(!"Not supported DRAM type");
    }
    if (NV_DRF_VAL(EMC, FBIO_CFG5, DRAM_WIDTH, RegValue) ==
        EMC_FBIO_CFG5_0_DRAM_WIDTH_X16)
    {
        pDfs->Modules[NvRmDfsModuleId_Emc].Scale++;
    }
    return NvSuccess;
}

void NvRmPrivAp20EmcMonitorsDeinit(NvRmDfs* pDfs)
{
     // Stop monitor using initialization procedure
    (void)NvRmPrivAp20EmcMonitorsInit(pDfs);
}

void
NvRmPrivAp20EmcMonitorsStart(
    const NvRmDfs* pDfs,
    const NvRmDfsFrequencies* pDfsKHz,
    const NvU32 IntervalMs)
{
    NvU32 RegValue, SavedRegValue;
    void* pEmcRegs = pDfs->Modules[NvRmDfsModuleId_Emc].pBaseReg;

    // EMC sample period is specified in EMC clock cycles, accuracy 0-16 cycles.
    #define MEAN_EMC_LIMIT_ERROR (8)
    NvU32 cycles = IntervalMs * pDfsKHz->Domains[NvRmDfsClockId_Emc] +
                    MEAN_EMC_LIMIT_ERROR;
    /*
     * Start EMC power monitor for the next sample period: clear EMC counters,
     * set sample interval limit in EMC cycles, enable monitoring. Monitor is
     * counting EMC 1x clock cycles while any memory access is detected. 
     */
    SavedRegValue = NV_EMC_REGR(pEmcRegs, STAT_CONTROL);
    RegValue = NV_FLD_SET_DRF_DEF(EMC, STAT_CONTROL, PWR_GATHER, CLEAR, SavedRegValue);
    NV_EMC_REGW(pEmcRegs, STAT_CONTROL, RegValue);

    RegValue = NV_DRF_NUM(EMC, STAT_PWR_CLOCK_LIMIT, PWR_CLOCK_LIMIT, cycles);
    NV_EMC_REGW(pEmcRegs, STAT_PWR_CLOCK_LIMIT, RegValue);

    RegValue = NV_FLD_SET_DRF_DEF(EMC, STAT_CONTROL, PWR_GATHER, ENABLE, SavedRegValue);
    NV_EMC_REGW(pEmcRegs, STAT_CONTROL, RegValue);
}

void
NvRmPrivAp20EmcMonitorsRead(
    const NvRmDfs* pDfs,
    const NvRmDfsFrequencies* pDfsKHz,
    NvRmDfsIdleData* pIdleData)
{
    NvU32 RegValue, TotalClocks;
    NvU32 CountShift = pDfs->Modules[NvRmDfsModuleId_Emc].Scale;
    void* pEmcRegs = pDfs->Modules[NvRmDfsModuleId_Emc].pBaseReg;

    /*
     * Read EMC monitor: disable it (=stop, the readings are preserved), and
     * determine idle count based on total and active clock counts. Monitor
     * readings are multiplied by 2^M factor to determine active count, where
     * power M depends on DRAM type and bus width. Store result in the idle
     * data packet.
     */
    RegValue = NV_EMC_REGR(pEmcRegs, STAT_CONTROL);
    RegValue = NV_FLD_SET_DRF_DEF(EMC, STAT_CONTROL, PWR_GATHER, DISABLE, RegValue);
    NV_EMC_REGW(pEmcRegs, STAT_CONTROL, RegValue);

    RegValue = NV_EMC_REGR(pEmcRegs, STAT_PWR_CLOCKS);
    TotalClocks = NV_DRF_VAL(EMC, STAT_PWR_CLOCKS, PWR_CLOCKS, RegValue);
    RegValue = NV_EMC_REGR(pEmcRegs, STAT_PWR_COUNT);
    RegValue = NV_DRF_VAL(EMC, STAT_PWR_COUNT, PWR_COUNT, RegValue) << CountShift;

    pIdleData->Readings[NvRmDfsClockId_Emc] = 
        (TotalClocks > RegValue) ? (TotalClocks - RegValue) : 0;
}

/*****************************************************************************/

// AP20 Thermal policy definitions

#define NVRM_THERMAL_DEGREES_HIGH           (85L)
#define NVRM_THERMAL_DEGREES_LOW            (50L)
#define NVRM_THERMAL_DEGREES_HYSTERESIS     (5L)

#define NVRM_THERMAL_POLL_MS_SLOW           (200UL)
#define NVRM_THERMAL_POLL_MS_FAST           (100UL)
#define NVRM_THERMAL_POLL_MS_CRITICAL       (50UL)
#define NVRM_THERMAL_POLL_INTR_FACTOR       (10UL)

#define NVRM_THERMAL_CPU_KHZ_LOW            (500000UL)

#define NVRM_THERMAL_CPU_DELTA_KHZ_LOW      (200000L)
#define NVRM_THERMAL_CPU_DELTA_KHZ_HIGH     (100000L)
#define NVRM_THERMAL_CPU_DELTA_KHZ_CRITICAL (-100000L)

void
NvRmPrivAp20DttGetTcorePolicy(
    NvS32 TemperatureC,
    const NvRmDtt* pDtt,
    NvS32* pLowLimit,
    NvS32* pHighLimit,
    NvU32* pPollMs)
{
    NvU32 msec;
    NvS32 LowLimit, HighLimit;

    NV_ASSERT(pDtt->TcoreCaps.Tmin <
              (NVRM_THERMAL_DEGREES_LOW - NVRM_THERMAL_DEGREES_HYSTERESIS));
    NV_ASSERT(pDtt->TcoreCaps.Tmax > NVRM_THERMAL_DEGREES_HIGH);

    /*
     * Temperature limits policy: limits are laways set "around" current
     * temperature for the next out-of-limit interrupt; range boundaries
     * are used for low and critical temperature.
     */
    if (TemperatureC <= NVRM_THERMAL_DEGREES_LOW)
    {
        LowLimit = pDtt->TcoreLowLimitCaps.MinValue;
        HighLimit = NVRM_THERMAL_DEGREES_LOW;
    }
    else if (TemperatureC <= NVRM_THERMAL_DEGREES_HIGH)
    {
        LowLimit = NVRM_THERMAL_DEGREES_LOW - NVRM_THERMAL_DEGREES_HYSTERESIS;
        HighLimit = NVRM_THERMAL_DEGREES_HIGH;

    }
    else
    {
        LowLimit = NVRM_THERMAL_DEGREES_HIGH - NVRM_THERMAL_DEGREES_HYSTERESIS;
        HighLimit = pDtt->TcoreHighLimitCaps.MaxValue;
    }

    /*
     * Polling time policy:
     * - low/high temperature in polling mode: return policy value
     * - low/high temperature in interrupt mode: policy value increased by intr
     *   factor (do not need polling at all in this mode, but just in case ...)
     * - critical temperature and any mode: return policy value (we do need
     *   polling even in interrupt mode for active throttling)
     * Keep higher polling rate inside hysteresis range.
     */
    if (TemperatureC <= 
        (NVRM_THERMAL_DEGREES_LOW - NVRM_THERMAL_DEGREES_HYSTERESIS))
    {
        if (pDtt->UseIntr)
            msec = NVRM_THERMAL_POLL_MS_SLOW * NVRM_THERMAL_POLL_INTR_FACTOR;
        else
            msec = NVRM_THERMAL_POLL_MS_SLOW;
    }
    else if (TemperatureC <= 
             (NVRM_THERMAL_DEGREES_HIGH - NVRM_THERMAL_DEGREES_HYSTERESIS))
    {
        if (pDtt->UseIntr)
            msec = NVRM_THERMAL_POLL_MS_FAST * NVRM_THERMAL_POLL_INTR_FACTOR;
        else
            msec = NVRM_THERMAL_POLL_MS_FAST;

    }
    else
    {
        msec = NVRM_THERMAL_POLL_MS_CRITICAL;
    }

    // Fill in return values
    *pLowLimit = LowLimit;
    *pHighLimit = HighLimit;
    *pPollMs = msec;  
}

NvBool
NvRmPrivAp20DttClockUpdate(
    NvRmDeviceHandle hRmDevice,
    NvS32 TemperatureC,
    const NvRmDfsFrequencies* pCurrentKHz,
    NvRmDfsFrequencies* pDfsKHz)
{
    // Only CPU throttling for now
    NvRmFreqKHz DeltaKHz;
    NvRmFreqKHz CpuTargetKHz = pDfsKHz->Domains[NvRmDfsClockId_Cpu];
    NvRmFreqKHz CpuThrottledKHz = pCurrentKHz->Domains[NvRmDfsClockId_Cpu];
    NvBool Throttled = NV_FALSE;

    // If CPU target is already low, no throttling
    if (CpuTargetKHz <= NVRM_THERMAL_CPU_KHZ_LOW)
        return Throttled;

    // Determine max frequency delta based on temperature
    if (TemperatureC <= NVRM_THERMAL_DEGREES_LOW)
        DeltaKHz = NVRM_THERMAL_CPU_DELTA_KHZ_LOW;
    else if (TemperatureC <= NVRM_THERMAL_DEGREES_HIGH)
        DeltaKHz = NVRM_THERMAL_CPU_DELTA_KHZ_HIGH;
    else
        DeltaKHz = NVRM_THERMAL_CPU_DELTA_KHZ_CRITICAL;

    // Find throttled limit
    CpuThrottledKHz += DeltaKHz;
    if (((NvS32)CpuThrottledKHz) < 0)
        CpuThrottledKHz = 0;

    // Find and set new target
    CpuTargetKHz = NV_MIN(CpuTargetKHz, CpuThrottledKHz);
    CpuTargetKHz = NV_MAX(CpuTargetKHz, NVRM_THERMAL_CPU_KHZ_LOW); 
    if (CpuTargetKHz < pDfsKHz->Domains[NvRmDfsClockId_Cpu])
    {
        Throttled = NV_TRUE;
    }
    pDfsKHz->Domains[NvRmDfsClockId_Cpu] = CpuTargetKHz;
    return Throttled;
}

/*****************************************************************************/

NvRmPmRequest 
NvRmPrivAp20GetPmRequest(
    NvRmDeviceHandle hRmDevice,
    const NvRmDfsSampler* pCpuSampler,
    NvRmFreqKHz CpuKHz)
{
    // Assume initial slave CPU1 On request
    static NvRmPmRequest s_LastPmRequest = (NvRmPmRequest_CpuOnFlag | 0x1);
    static NvRmFreqKHz s_Cpu1OnMinKHz = 0, s_Cpu1OffMaxKHz = 0;

    NvRmPmRequest PmRequest = NvRmPmRequest_None;
    NvBool Cpu1Off =
        (0 != NV_DRF_VAL(CLK_RST_CONTROLLER, RST_CPU_CMPLX_SET, SET_CPURESET1,
                         NV_REGR(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                 CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET_0)));
    NvRmFreqKHz CpuLoadGaugeKHz = pCpuSampler->BumpedAverageKHz;

    // Slave CPU1 power management policy thresholds:
    // - use fixed values if they are defined explicitly, otherwise
    // - use max CPU frequency at min CPU voltage) as CPU1 OffMax threshold,
    //   and half of that frequency as CPU1 OnMin threshold
    if ((s_Cpu1OffMaxKHz == 0) && (s_Cpu1OnMinKHz == 0))
    {
        NvU32 n;
        const NvRmFreqKHz* p = NvRmPrivModuleVscaleGetMaxKHzList(
            hRmDevice, NvRmModuleID_Cpu, &n);

        NV_ASSERT (p && n);
        s_Cpu1OnMinKHz = NVRM_CPU1_ON_MIN_KHZ ?
                         NVRM_CPU1_ON_MIN_KHZ : (p[0] >> 1);
        s_Cpu1OffMaxKHz = NVRM_CPU1_OFF_MAX_KHZ ?
                          NVRM_CPU1_OFF_MAX_KHZ : p[0];
        NV_ASSERT(s_Cpu1OnMinKHz < s_Cpu1OffMaxKHz);
    }

    /*
     * Request OS kernel to turn CPU1 Off if all of the following is true:
     * (a) CPU frequency is below OnMin threshold, 
     * (b) Last request was CPU1 On request
     * (c) CPU1 is actually On
     *
     * Request OS kernel to turn CPU1 On if all of the following is true:
     * (a) CPU frequency is above OffMax threshold 
     * (b) Last request was CPU1 Off request
     * (c) CPU1 is actually Off
     */
    if (CpuLoadGaugeKHz < s_Cpu1OnMinKHz)
    {
        if ((s_LastPmRequest & NvRmPmRequest_CpuOnFlag) && (!Cpu1Off))
            s_LastPmRequest = PmRequest = (NvRmPmRequest_CpuOffFlag | 0x1);
#if NVRM_TEST_PMREQUEST_UP_MODE
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET_0,
            CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET_0_SET_CPURESET1_FIELD);
#endif
    }
    else if (CpuLoadGaugeKHz > s_Cpu1OffMaxKHz)
    {
        if ((s_LastPmRequest & NvRmPmRequest_CpuOffFlag) && Cpu1Off)
            s_LastPmRequest = PmRequest = (NvRmPmRequest_CpuOnFlag | 0x1);
#if NVRM_TEST_PMREQUEST_UP_MODE
        NV_REGW(hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR_0,
            CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR_0_CLR_CPURESET1_FIELD);
#endif
    }
    return PmRequest;
}

/*****************************************************************************/
