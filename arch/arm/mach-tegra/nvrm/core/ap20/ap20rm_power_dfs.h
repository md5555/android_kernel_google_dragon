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

/** 
 * @file
 * @brief <b>nVIDIA Driver Development Kit: 
 *           Power Resource manager </b>
 *
 * @b Description: NvRM DFS parameters. 
 * 
 */

#ifndef INCLUDED_AP20RM_POWER_DFS_H
#define INCLUDED_AP20RM_POWER_DFS_H

#include "nvrm_power_dfs.h"

#ifdef __cplusplus
extern "C"
{
#endif  /* __cplusplus */

// Min KHz for CPU and AVP with regards to JTAG support - 1MHz * 8  = 8MHz
// TODO: any other limitations on min KHz?
// TODO: adjust boost parameters based on testing

/**
 * Default DFS algorithm parameters for CPU domain
 */
#define NVRM_DFS_PARAM_CPU_AP20 \
    NvRmFreqMaximum, /* Maximum domain frequency set to h/w limit */ \
    40000,  /* Minimum domain frequency 40 MHz */ \
    1000,   /* Frequency change upper band 1 MHz */ \
    1000,   /* Frequency change lower band 1 MHz */ \
    {          /* RT starvation control parameters */ \
        32000, /* Fixed frequency boost increase 32 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    {          /* NRT starvation control parameters */ \
        4000,  /* Fixed frequency boost increase 4 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    3,      /* Relative adjustement of average freqiency 1/2^3 ~ 12% */ \
    1,      /* Number of smaple intervals with NRT to trigger boost = 2 */ \
    1       /* NRT idle cycles threshold = 1 */ 

/**
 *  Default DFS algorithm parameters for AVP domain
 */
#define NVRM_DFS_PARAM_AVP_AP20 \
    NvRmFreqMaximum, /* Maximum domain frequency set to h/w limit */ \
    24000,  /* Minimum domain frequency 24 MHz */ \
    1000,   /* Frequency change upper band 1 MHz */ \
    1000,   /* Frequency change lower band 1 MHz */ \
    {          /* RT starvation control parameters */ \
        8000,  /* Fixed frequency boost increase 8 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    {          /* NRT starvation control parameters */ \
        1000,  /* Fixed frequency NRT boost increase 1 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    3,      /* Relative adjustement of average freqiency 1/2^3 ~ 12% */ \
    2,      /* Number of smaple intervals with NRT to trigger boost = 3 */ \
    1       /* NRT idle cycles threshold = 1 */ 

/**
 * Default DFS algorithm parameters for System clock domain
 */
#define NVRM_DFS_PARAM_SYSTEM_AP20 \
    NvRmFreqMaximum, /* Maximum domain frequency set to h/w limit */ \
    24000,  /* Minimum domain frequency 24 MHz */ \
    1000,   /* Frequency change upper band 1 MHz */ \
    1000,   /* Frequency change lower band 1 MHz */ \
    {          /* RT starvation control parameters */ \
        8000,  /* Fixed frequency boost increase 8 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    {          /* NRT starvation control parameters */ \
        1000,  /* Fixed frequency NRT boost increase 1 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        32,    /* Proportional frequency boost decrease 32/256 ~ 12% */  \
    },\
    5,      /* Relative adjustement of average freqiency 1/2^5 ~ 3% */ \
    2,      /* Number of smaple intervals with NRT to trigger boost = 3 */ \
    1       /* NRT idle cycles threshold = 1 */ 

/**
 * Default DFS algorithm parameters for AHB clock domain
 */
#define NVRM_DFS_PARAM_AHB_AP20 \
    NvRmFreqMaximum, /* Maximum domain frequency set to h/w limit */ \
    24000,  /* Minimum domain frequency 24 MHz */ \
    1000,   /* Frequency change upper band 1 MHz */ \
    1000,   /* Frequency change lower band 1 MHz */ \
    {          /* RT starvation control parameters */ \
        8000,  /* Fixed frequency boost increase 8 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    {          /* NRT starvation control parameters */ \
        1000,  /* Fixed frequency NRT boost increase 1 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        32,    /* Proportional frequency boost decrease 32/256 ~ 12% */  \
    },\
    0,      /* Relative adjustement of average freqiency 1/2^0 ~ 100% */ \
    0,      /* Number of smaple intervals with NRT to trigger boost = 1 */ \
    1       /* NRT idle cycles threshold = 1 */ 

/**
 * Default DFS algorithm parameters for APB clock domain
 */
#define NVRM_DFS_PARAM_APB_AP20 \
    NVRM_AP20_APB_MAX_KHZ, /* AP20 APB limit is lower than other buses */ \
    24000,  /* Minimum domain frequency 24 MHz */ \
    1000,   /* Frequency change upper band 1 MHz */ \
    1000,   /* Frequency change lower band 1 MHz */ \
    {          /* RT starvation control parameters */ \
        8000,  /* Fixed frequency boost increase 8 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    {          /* NRT starvation control parameters */ \
        1000,  /* Fixed frequency NRT boost increase 1 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        32,    /* Proportional frequency boost decrease 32/256 ~ 12% */  \
    },\
    0,      /* Relative adjustement of average freqiency 1/2^0 ~ 100% */ \
    0,      /* Number of smaple intervals with NRT to trigger boost = 1 */ \
    1       /* NRT idle cycles threshold = 1 */ 

/**
 * Default DFS algorithm parameters for Video-pipe clock domain
 */
#define NVRM_DFS_PARAM_VPIPE_AP20 \
    NvRmFreqMaximum, /* Maximum domain frequency set to h/w limit */ \
    24000,  /* Minimum domain frequency 24 MHz */ \
    1000,   /* Frequency change upper band 1 MHz */ \
    1000,   /* Frequency change lower band 1 MHz */ \
    {          /* RT starvation control parameters */ \
        16000, /* Fixed frequency RT boost increase 16 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    {          /* NRT starvation control parameters */ \
        1000,  /* Fixed frequency NRT boost increase 1 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    5,      /* Relative adjustement of average freqiency 1/2^5 ~ 3% */ \
    3,      /* Number of smaple intervals with NRT to trigger boost = 4 */ \
    1       /* NRT idle cycles threshold = 1 */ 

/**
 * Default DFS algorithm parameters for EMC clock domain
 */
#define NVRM_DFS_PARAM_EMC_AP20 \
    NvRmFreqMaximum, /* Maximum domain frequency set to h/w limit */ \
    16000,  /* Minimum domain frequency 16 MHz */ \
    1000,   /* Frequency change upper band 1 MHz */ \
    1000,   /* Frequency change lower band 1 MHz */ \
    {          /* RT starvation control parameters */ \
        16000, /* Fixed frequency RT boost increase 16 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    {          /* NRT starvation control parameters */ \
        1000,  /* Fixed frequency NRT boost increase 1 MHz */ \
        255,   /* Proportional frequency boost increase 255/256 ~ 100% */ \
        128,   /* Proportional frequency boost decrease 128/256 ~ 50% */  \
    },\
    0,      /* Relative adjustement of average freqiency 1/2^0 ~ 100% */ \
    0,      /* Number of smaple intervals with NRT to trigger boost = 1 */ \
    1       /* NRT idle cycles threshold = 1 */ 


/**
 * Defines CPU frequency threshold for slave CPU1 power management:
 * - CPU1 is turned Off when cpu clock is below ON_MIN
 * - CPU1 is turned On when cpu clock is above OFF_MAX
 * If set to 0, the threshold value is derived at run time from the
 * characterization data
 */
#define NVRM_CPU1_ON_MIN_KHZ (0)
#define NVRM_CPU1_OFF_MAX_KHZ (0)

/// Default low corners for core and dedicated CPU voltages
#define NVRM_AP20_LOW_CORE_MV (1200)
#define NVRM_AP20_LOW_CPU_MV (850)

/*****************************************************************************/

/**
 * Initializes activity monitors within the DFS module. Only activity
 * monitors are affected. The rest of module's h/w is preserved.
 * 
 * @param pDfs - A pointer to DFS structure.
 * 
 * @return NvSuccess if initialization completed successfully
 * or one of common error codes on failure.
 */
NvError NvRmPrivAp20EmcMonitorsInit(NvRmDfs* pDfs);

/**
 * Deinitializes activity monitors within the DFS module. Only activity
 * monitors are affected. The rest of module's h/w is preserved.
 * 
 * @param pDfs - A pointer to DFS structure.
 */
void NvRmPrivAp20EmcMonitorsDeinit(NvRmDfs* pDfs);

/**
 * Starts activity monitors in the DFS module for the next sample interval.
 * 
 * @param pDfs - A pointer to DFS structure.
 * @param pDfsKHz - A pointer to current DFS clock frequencies structure.
 * @param IntervalMs Next sampling interval in ms.
 */
void
NvRmPrivAp20EmcMonitorsStart(
    const NvRmDfs* pDfs,
    const NvRmDfsFrequencies* pDfsKHz,
    const NvU32 IntervalMs);

/**
 * Reads idle count from activity monitors in the DFS module. The monitors are
 * stopped.
 * 
 * @param pDfs - A pointer to DFS structure.
 * @param pDfsKHz - A pointer to current DFS clock frequencies structure.
 * @param pIdleData - A pointer to idle cycles structure to be filled in with
 *  data read from the monitor.
 * 
 */
void
NvRmPrivAp20EmcMonitorsRead(
    const NvRmDfs* pDfs,
    const NvRmDfsFrequencies* pDfsKHz,
    NvRmDfsIdleData* pIdleData);

/**
 * Changes core and rtc voltages, keeping them in synch
 * 
 * @param hRm The RM device handle.
 * @param A pointer to DVS structure.
 * @param TargetMv Requested core/rtc voltage in mV.
 * 
 */
void
NvRmPrivAp20DvsChangeCoreVoltage(
    NvRmDeviceHandle hRm,
    NvRmDvs* pDvs,
    NvRmMilliVolts TargetMv);

/**
 * Determines temperature monitoring policy.
 * 
 * @param TemperatureC Current core temperature in degrees C.
 * @param pDtt A pointer to dynamic thermal throttling structure.
 * @param pLowLimit A pointer to the returned variable with low boundary for
 *  temperature out-of-limit interrupt.
 * @param pHighLimit A pointer to the returned variable with high boundary for
 *  temperature out-of-limit interrupt.
 * @param pPollMs A pointer to the returned variable with temperature polling
 *  interval in milliseconds.
 */

void
NvRmPrivAp20DttGetTcorePolicy(
    NvS32 TemperatureC,
    const NvRmDtt* pDtt,
    NvS32* pLowLimit,
    NvS32* pHighLimit,
    NvU32* pPollMs);

/**
 * Throttles DFS target clocks.
 * 
 * @param hRmDevice The RM device handle.
 * @param TemperatureC Current core temperature in degrees C.
 * @param pCurrentKHz A pointer to current DFS clock frequencies structure.
 * @param pDfsKHz A pointer to DFS clock structure with target frequencies
 *  on entry, and throttled frequencies on exit.
 * 
 * @return NV_TRUE if target clocks were throttled, and NV_FALSE otherwise.
 */
NvBool
NvRmPrivAp20DttClockUpdate(
    NvRmDeviceHandle hRmDevice,
    NvS32 TemperatureC,
    const NvRmDfsFrequencies* pCurrentKHz,
    NvRmDfsFrequencies* pDfsKHz);

/**
 * Determines PM request to change CPU(s) power state.
 * 
 * @param hRmDevice The RM device handle.
 * @param pCpuSampler Pointer to the DFS CPU clock sampling records
 * @param CpuKHz CPU clock frequency target
 * 
 * @return New PM request to change CPU power state
 */
NvRmPmRequest 
NvRmPrivAp20GetPmRequest(
    NvRmDeviceHandle hRmDevice,
    const NvRmDfsSampler* pCpuSampler,
    NvRmFreqKHz CpuKHz);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif // INCLUDED_AP20RM_POWER_DFS_H
