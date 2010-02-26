/*
 * arch/arm/mach-tegra/idle-t2.c
 *
 * CPU hotplug support for Tegra SoCs
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

#include "ap20/arapbpm.h"
#include "nvrm_module.h"
#include "ap20/arflow_ctlr.h"
#include "nvbootargs.h"
#include "nvrm_memmgr.h"
#include "power.h"
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include "nvos.h"

extern NvRmDeviceHandle s_hRmGlobal;
extern void cpu_ap20_do_lp2(void);
extern void cpu_ap20_do_lp1(void);
extern void cpu_ap20_do_lp0(void);
extern void resume(unsigned int state);
extern uintptr_t g_resume, g_contextSavePA, g_contextSaveVA;
extern uintptr_t g_iramContextSaveVA;
extern NvU32 g_NumActiveCPUs, g_ArmPerif;
extern NvU32 g_enterLP2PA;
extern volatile void *g_pPMC, *g_pAHB, *g_pCLK_RST_CONTROLLER;
extern volatile void *g_pEMC, *g_pMC, *g_pAPB_MISC, *g_pIRAM, *g_pTimerus;
#ifdef CONFIG_WAKELOCK
extern struct wake_lock main_wake_lock;
#endif

// Debug-only: set to zero to disable use of flow-controller based idling
#define CPU_CONTEXT_SAVE_AREA_SIZE 4096
#define TEMP_SAVE_AREA_SIZE 16
#define ENABLE_LP2 1
#define LP2_ROUNDTRIP_TIME	1
#define LP2_PADDING_FACTOR	100

// When non-zero, collects and prints aggregate statistics about idle times
static volatile NvU8 *s_pFlowCtrl = NULL;

void cpu_ap20_do_idle(void);
void mach_tegra_reset(void);
void mach_tegra_idle(void);
extern void enter_lp2(NvU32, NvU32);
extern void exit_power_state(void);
extern void module_context_init(void);
extern void power_lp0_init(void);
extern void NvSpareTimerTrigger(unsigned long); /* timer.c */
NvRmMemHandle s_hWarmboot = NULL;
NvU32 g_AvpWarmbootEntry;
NvU32 g_IramPA = 0;

NvU32 lp2count = 0, lp3count = 0, lp2safe = 0;

void __init NvAp20InitFlowController(void);

void __init NvAp20InitFlowController(void)
{
    NvU32 len;
    NvRmPhysAddr pa;
    volatile NvU8 *pTempFc, *pTempArmPerif;
    NvBootArgsWarmboot WarmbootArgs;

    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmModuleID_FlowCtrl, 0), &pa, &len);
            
    if (NvRmPhysicalMemMap(pa, len, NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached, (void**)&pTempFc)!=NvSuccess)
    {
        printk(KERN_INFO "failed to map flow controller; "
               " DVFS will not function correctly as a result\n");
        return;
    }

    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmPrivModuleID_ArmPerif, 0), &pa, &len);

    if (NvRmPhysicalMemMap(pa, len, NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached, (void**)&pTempArmPerif)!=NvSuccess)
    {
        printk(KERN_INFO "failed to map Arm Perif; DVFS will not function"
               " correctly as a result\n");
        return;
    }

    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmModuleID_Pmif, 0), &pa, &len);

    if (NvRmPhysicalMemMap(pa, len, NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached, (void**)&g_pPMC)!=NvSuccess)
    {
        printk(KERN_INFO "failed to map pmif; DVFS will not function"
               " correctly as a result\n");
        return;
    }

    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmPrivModuleID_Ahb_Arb_Ctrl, 0), &pa, &len);

    if (NvRmPhysicalMemMap(pa, len, NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached, (void**)&g_pAHB)!=NvSuccess)
    {
        printk(KERN_INFO "failed to map ahb; DVFS will not function"
               " correctly as a result\n");
        return;
    }

    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmPrivModuleID_ClockAndReset, 0), &pa, &len);

    if (NvRmPhysicalMemMap(pa, len, NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached,
            (void**)&g_pCLK_RST_CONTROLLER)!=NvSuccess)
    {
        printk(KERN_INFO "failed to map car; DVFS will not function"
               " correctly as a result\n");
        return;
    }

    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmPrivModuleID_ExternalMemoryController, 0),
            &pa, &len);

    if (NvRmPhysicalMemMap(pa, len, NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached,
            (void**)&g_pEMC)!=NvSuccess)
    {
        printk(KERN_INFO "failed to map emc; DVFS will not function"
               " correctly as a result\n");
        return;
    }

    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmPrivModuleID_MemoryController, 0), &pa, &len);

    if (NvRmPhysicalMemMap(pa, len, NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached,
            (void**)&g_pMC)!=NvSuccess)
    {
        printk(KERN_INFO "failed to map mc; DVFS will not function"
               " correctly as a result\n");
        return;
    }

    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmModuleID_Misc, 0), &pa, &len);

    if (NvRmPhysicalMemMap(pa, len, NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached,
            (void**)&g_pAPB_MISC)!=NvSuccess)
    {
        printk(KERN_INFO "failed to map misc; DVFS will not function"
               " correctly as a result\n");
        return;
    }

    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmPrivModuleID_Iram, 0), &g_IramPA, &len);

    if (NvRmPhysicalMemMap(g_IramPA, len, NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached,
            (void**)&g_pIRAM)!=NvSuccess)
    {
        printk(KERN_INFO "failed to map iram; DVFS will not function"
               " correctly as a result\n");
        return;
    }

    NvRmModuleGetBaseAddress(s_hRmGlobal,
        NVRM_MODULE_ID(NvRmModuleID_TimerUs, 0), &pa, &len);

    if (NvRmPhysicalMemMap(pa, len, NVOS_MEM_READ_WRITE,
            NvOsMemAttribute_Uncached,
            (void**)&g_pTimerus)!=NvSuccess)
    {
        printk(KERN_INFO "failed to map iram; DVFS will not function"
               " correctly as a result\n");
        return;
    }

    s_pFlowCtrl = pTempFc;
    g_ArmPerif = (NvU32)pTempArmPerif;

    /* Get physical and virtual addresses for a bunch of things
     * 1) The resume address physical - we have to come back here with MMU off
     * 2) The context save physical address - We need to save a bunch of
     * stuff here
     */
    g_resume = virt_to_phys((void*)exit_power_state);
    g_contextSaveVA =
        (uintptr_t)kmalloc(CPU_CONTEXT_SAVE_AREA_SIZE, GFP_ATOMIC);
    g_iramContextSaveVA =
        (uintptr_t)kmalloc(AVP_CONTEXT_SAVE_AREA_SIZE, GFP_ATOMIC);
    g_contextSavePA = virt_to_phys((void*)g_contextSaveVA);
    g_NumActiveCPUs = num_online_cpus();
    g_enterLP2PA = virt_to_phys((void*)enter_lp2);

    NvOsBootArgGet(NvBootArgKey_WarmBoot,
        &WarmbootArgs, sizeof(NvBootArgsWarmboot));
    if (NvRmMemHandleClaimPreservedHandle(s_hRmGlobal,
        WarmbootArgs.MemHandleKey, &s_hWarmboot))
    {
        printk("Could not locate Warm booloader!\n");
    }
    else
    {
        g_AvpWarmbootEntry = NvRmMemPin(s_hWarmboot);
    }

    module_context_init();
    power_lp0_init();
}

/*
 *  cpu_ap20_do_idle()
 *
 *  Idle the processor (eg, wait for interrupt).
 *
 *  IRQs are already disabled.
 */
void cpu_ap20_do_idle(void)
{
    unsigned int tmp = 0;
    volatile uint32_t *addr = 0;
    
    dsb();

    if (likely(s_pFlowCtrl))
    {
        /* 
         *  Trigger the "stats monitor" to count the CPU idle cycles.
         */
        if (smp_processor_id())
        {
            addr = (volatile uint32_t*)(s_pFlowCtrl + FLOW_CTLR_HALT_CPU1_EVENTS_0);
        } else
        {
            addr = (volatile uint32_t*)(s_pFlowCtrl + FLOW_CTLR_HALT_CPU_EVENTS_0);
        }

        tmp = NV_DRF_DEF(FLOW_CTLR, HALT_CPU1_EVENTS, MODE, FLOW_MODE_WAITEVENT) 
            | NV_DRF_NUM(FLOW_CTLR, HALT_CPU1_EVENTS, JTAG, 1);

        NV_WRITE32(addr, tmp);
        tmp = NV_READ32(addr);
    }

    // Wait for any interrupt
    __asm__ volatile ("wfi");

    if (addr)
    {
        /*
         * Signal "stats monitor" to stop counting the idle cycles.
         */
        tmp = NV_DRF_DEF(FLOW_CTLR, HALT_CPU1_EVENTS, MODE, FLOW_MODE_NONE); 
        NV_WRITE32(addr, tmp);
        tmp = NV_READ32(addr);
    }
}

void mach_tegra_idle(void)
{ 
	static NvU32 flag = 0;
	static NvU64 cur_jiffies = 0, next_timer;
	static NvU32 msec = 0;
	NvU64 delta_jif = 0;
	static NvS32 lp2_time = 0;

#ifdef CONFIG_WAKELOCK
	//The wake lock api is ready if the main lock is ready
	if (main_wake_lock.flags)
	{
		//Check if there are any IDLE locks pending
		//If there are, then we do LP3. Else we do LP2
		if (!has_wake_lock(WAKE_LOCK_IDLE) && flag)
		{
			flag = 0;
#if ENABLE_LP2            
			//Only attempt LP2 if the flow controller is ready
			if (s_pFlowCtrl)
			{
				if (num_online_cpus() > 1)
					lp2safe = 1;

				if (num_online_cpus() == 1 && lp2safe) {
					NvU32 dfs_low_corner = (NvRmPrivGetDfsFlags(s_hRmGlobal) &
											NvRmDfsStatusFlags_Pause);

					if (dfs_low_corner) {
						lp2count++;
						NvSpareTimerTrigger(lp2_time);
						cpu_ap20_do_lp2();

						jiffies += msecs_to_jiffies(lp2_time / 1000);
						NvRmPrivSetLp2TimeUS(s_hRmGlobal, lp2_time);
						return;
					}
				}
			}
#endif
		}
	}
#endif // CONFIG_WAKELOCK
	lp3count++;
	cur_jiffies = get_jiffies_64();
	next_timer = get_next_timer_interrupt(cur_jiffies);
	delta_jif = next_timer - cur_jiffies;
	msec = jiffies_to_msecs(delta_jif);

	lp2_time = msec - LP2_ROUNDTRIP_TIME * LP2_PADDING_FACTOR;

	if (lp2_time > 0)
	{
		if (num_online_cpus() > 1)
			lp2safe = 1;
		else
			flag = 1;

		lp2_time *= 1000;
	}

	cpu_ap20_do_idle();
}

void mach_tegra_reset(void)
{
    NvU32 reg;

    reg = NV_DRF_DEF(APBDEV_PMC, CNTRL, MAIN_RST, ENABLE);
    NV_REGW(s_hRmGlobal, NvRmModuleID_Pmif, 0, APBDEV_PMC_CNTRL_0, reg);
}
