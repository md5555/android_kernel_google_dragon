/*
 * arch/arm/mach-tegra/idle-t1.c
 *
 * dynamic voltage and frequency scaling support for Tegra 1x SoCs
 *
 * Copyright (c) 2009, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include "ap15/arflow_ctlr.h"
#include "ap15/arapbpm.h"
#include "nvcommon.h"
#include "nvrm_init.h"
#include "mach/nvrm_linux.h"
#include "nvrm_hardware_access.h"
#include "nvrm_drf.h"
#include "nvrm_module.h"
#include "nvos.h"
#include "nvassert.h"

// Debug-only: set to zero to disable use of flow-controller based idling
#define USE_FLOW_CONTROLLER 1

// When non-zero, collects and prints aggregate statistics about idle times
#define COLLECT_STATS 0
static volatile NvU8 *s_pFlowCtrl = NULL;
static volatile NvU8 *s_pBarrier = NULL;

void cpu_ap15_do_idle(void);
void mach_tegra_reset(void);
void mach_tegra_idle(void);

void __init NvAp15InitFlowController(void);

void __init NvAp15InitFlowController(void)
{
#if USE_FLOW_CONTROLLER
    NvU32 len;
    NvRmPhysAddr pa;
    volatile NvU8 *pTempFc;
    volatile NvU8 *pTempBar;
    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmModuleID_FlowCtrl, 0), &pa, &len);

    if (NvRmPhysicalMemMap(pa, len, NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached, (void**)&pTempFc)!=NvSuccess)
    {
        printk(KERN_INFO "failed to map flow controller; DVFS will not function"
               " correctly as a result\n");
        return;
    }
    /*  HACK: To quiesce the memory interface before using a flow-controller
     *  wait for interrupt, a write & read is performed to this unused register
     *  address.  See NvBug 531184 */
    if (NvOsPhysicalMemMap((NvOsPhysAddr)0x6000C5C0UL, 4,
            NvOsMemAttribute_Uncached, NVOS_MEM_READ_WRITE,
            (void**)&pTempBar)!=NvSuccess)
    {
        printk(KERN_INFO "failed to map flow controller barrier; DVFS will not"
               " function correctly as a result\n");
        return;
    }
    s_pFlowCtrl = pTempFc;
    s_pBarrier = pTempBar;
#endif
}

#if COLLECT_STATS
static NvU64 s_SleepUs = 0;
static unsigned int s_Cnt = 0;
static NvU64 s_Enter = 0;
static void SleepEnter(void)
{
    s_Enter = NvOsGetTimeUS();
}
static void SleepExit(void)
{
    NvU64 Exit = NvOsGetTimeUS();
    s_SleepUs += (Exit - s_Enter);
    s_Cnt++;
    if ((s_Cnt == 1000) || (s_SleepUs>1000000ULL))
    {
        NvU32 average = (NvU32)NvDiv64(s_SleepUs, s_Cnt);
        printk("average flow controller sleep: %u us\n", average);
        s_Cnt = 0;
        s_SleepUs = 0;
    }
}
#else
#define SleepEnter()
#define SleepExit()
#endif

void cpu_ap15_do_idle(void)
{
    unsigned int tmp = 0;
    unsigned long flags;
    if (unlikely(!s_pFlowCtrl))
    {
        __asm__ volatile ("mcr p15, 0, %0, c7, c0, 4" : : "r"(tmp) : "cc");
    }
    else
    {
        SleepEnter();
        /* Disable interrupts to ensure no TLB walks will occur until after
         * this code exits */
        local_irq_save(flags);
        /* To make sure that the memory system is quiescent, write to an
         * address and then immediately read the result. This works because
         * s_pBarrier is using an address that is mapped uncached (shared
         * device in this case), so read-after-write hazards are guaranteed to
         * block until writes have passed the point of coherency  */
        NV_WRITE32(s_pBarrier, 0);
        tmp = NV_READ32(s_pBarrier);
        tmp = (NV_DRF_DEF(FLOW_CTLR, HALT_CPU_EVENTS, MODE,
                          FLOW_MODE_STOP_UNTIL_INT) |
               NV_DRF_NUM(FLOW_CTLR, HALT_CPU_EVENTS, JTAG, 1));
        NV_WRITE32(s_pFlowCtrl + FLOW_CTLR_HALT_CPU_EVENTS_0, tmp);
        tmp = NV_READ32(s_pFlowCtrl + FLOW_CTLR_HALT_CPU_EVENTS_0);

        dsb();
        NV_WRITE32(s_pBarrier, 0);
        NV_READ32(s_pBarrier);

        local_irq_restore(flags);
        SleepExit();
    }
}

void mach_tegra_idle()
{
    cpu_ap15_do_idle();
}

void mach_tegra_reset()
{
    NvU32 reg;

    reg = NV_DRF_DEF(APBDEV_PMC, CNTRL, MAIN_RST, ENABLE);
    NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0, APBDEV_PMC_CNTRL_0, reg);    
}

